#include "avionics/render/MapView.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/Terrain.h"
#include "render/map/MapProjection.h"

namespace avionics {
namespace {

constexpr float kWtCanvasHeight = 768.0f;

// Nav-feature symbol size, in 768-px-canvas units. Symbols are sized off the
// display (like text) rather than the viewport, so they stay a fixed, legible
// size on both the small PFD inset and the full-screen MFD MAP page instead of
// ballooning on the larger viewport.
constexpr float kFeatureSymbolWt = 8.0f;

// Range-based feature declutter (NM): each class is hidden once the map range
// exceeds its threshold, matching the G1000's progressive decluttering. Fixes
// (intersections) are the densest, so they only appear when zoomed well in, and
// at most kMaxFixesDrawn of the nearest ones are drawn.
constexpr float kFeatureRangeAirportNm = 150.0f;
constexpr float kFeatureRangeVorNm = 100.0f;
constexpr float kFeatureRangeNdbNm = 40.0f;
constexpr float kFeatureRangeFixNm = 7.5f;
constexpr int kMaxFixesDrawn = 40;

float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kWtCanvasHeight);
}

float orientationDeg(MapOrientation mode, const FlightData& flight) {
  switch (mode) {
    case MapOrientation::NorthUp:
      return 0.0f;
    case MapOrientation::HeadingUp:
      return flight.headingDeg;
    case MapOrientation::TrackUp:
      return flight.trackDeg;
  }
  return 0.0f;
}

// Map orientation annunciation, as on the real unit ("NORTH UP" / "TRK UP" /
// "HDG UP", G1000 NXi Pilot's Guide, Navigation Map).
const char* orientationLabel(MapOrientation mode) {
  switch (mode) {
    case MapOrientation::NorthUp:
      return "NORTH UP";
    case MapOrientation::HeadingUp:
      return "HDG UP";
    case MapOrientation::TrackUp:
      return "TRK UP";
  }
  return "";
}

// Topographic color ramp (ft MSL -> color). Muted/desaturated so the white,
// cyan and magenta map symbology stays legible on top: water is dark blue,
// lowlands green, hills tan/brown, peaks grey. The shoreline is a sharp step
// from blue (<=0 ft) to green (>0 ft).
struct TerrainStop {
  float ft;
  Color color;
};

constexpr TerrainStop kTerrainStops[] = {
    {-400.0f, {0.03f, 0.06f, 0.11f, 1.0f}},   // deep water
    {0.0f, {0.05f, 0.10f, 0.16f, 1.0f}},      // shoreline water
    {1.0f, {0.06f, 0.16f, 0.08f, 1.0f}},      // lowland green
    {1000.0f, {0.10f, 0.22f, 0.10f, 1.0f}},   // green
    {2500.0f, {0.24f, 0.26f, 0.12f, 1.0f}},   // olive
    {4500.0f, {0.32f, 0.24f, 0.12f, 1.0f}},   // tan
    {7000.0f, {0.34f, 0.22f, 0.14f, 1.0f}},   // brown
    {10000.0f, {0.45f, 0.42f, 0.40f, 1.0f}},  // grey rock
    {13000.0f, {0.62f, 0.60f, 0.58f, 1.0f}},  // light grey / snow
};

Color terrainColor(float ft) {
  constexpr int n =
      static_cast<int>(sizeof(kTerrainStops) / sizeof(kTerrainStops[0]));
  if (ft <= kTerrainStops[0].ft) return kTerrainStops[0].color;
  if (ft >= kTerrainStops[n - 1].ft) return kTerrainStops[n - 1].color;
  for (int i = 1; i < n; ++i) {
    if (ft <= kTerrainStops[i].ft) {
      const Color& a = kTerrainStops[i - 1].color;
      const Color& b = kTerrainStops[i].color;
      const float t =
          (ft - kTerrainStops[i - 1].ft) /
          (kTerrainStops[i].ft - kTerrainStops[i - 1].ft);
      return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
              a.b + (b.b - a.b) * t, 1.0f};
    }
  }
  return kTerrainStops[n - 1].color;
}

// Fills the map viewport with a topographic raster: a grid of quads sampled
// from the terrain source and colored by elevation. The grid is built in
// geographic space around ownship and projected (with the map rotation) so it
// scrolls and rotates with the rest of the map. The grid covers ~2x the range
// in each direction so it still fills the rotated viewport corners.
void drawTerrain(Renderer& r, const TerrainSource& terrain, double centerLat,
                 double centerLon, float cx, float cy, float pixelsPerNm,
                 float rotation, float rangeNm, const MapViewConfig& config) {
  constexpr int kN = 40;  // cells per side
  const float halfNm = rangeNm * 2.0f;
  const float stepNm = (2.0f * halfNm) / kN;
  const double nmLon = map::nmPerDegLon(centerLat);

  std::vector<Point> pts(static_cast<size_t>(kN + 1) * (kN + 1));
  for (int i = 0; i <= kN; ++i) {  // i: north (top) -> south
    const double northNm = halfNm - i * stepNm;
    for (int j = 0; j <= kN; ++j) {  // j: west -> east
      const double eastNm = -halfNm + j * stepNm;
      const double lat = centerLat + northNm / map::kNmPerDegLat;
      const double lon = centerLon + eastNm / nmLon;
      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(lat, lon, centerLat, centerLon, cx, cy, pixelsPerNm,
                           rotation, x, y);
      pts[static_cast<size_t>(i) * (kN + 1) + j] = {x, y};
    }
  }

  const float minX = config.x, maxX = config.x + config.w;
  const float minY = config.y, maxY = config.y + config.h;
  for (int i = 0; i < kN; ++i) {
    const double cellNorthNm = halfNm - (i + 0.5) * stepNm;
    for (int j = 0; j < kN; ++j) {
      const Point& a = pts[static_cast<size_t>(i) * (kN + 1) + j];
      const Point& b = pts[static_cast<size_t>(i) * (kN + 1) + j + 1];
      const Point& c = pts[static_cast<size_t>(i + 1) * (kN + 1) + j + 1];
      const Point& d = pts[static_cast<size_t>(i + 1) * (kN + 1) + j];
      const float qMinX = std::min(std::min(a.x, b.x), std::min(c.x, d.x));
      const float qMaxX = std::max(std::max(a.x, b.x), std::max(c.x, d.x));
      const float qMinY = std::min(std::min(a.y, b.y), std::min(c.y, d.y));
      const float qMaxY = std::max(std::max(a.y, b.y), std::max(c.y, d.y));
      if (qMaxX < minX || qMinX > maxX || qMaxY < minY || qMinY > maxY) continue;

      const double cellEastNm = -halfNm + (j + 0.5) * stepNm;
      const double lat = centerLat + cellNorthNm / map::kNmPerDegLat;
      const double lon = centerLon + cellEastNm / nmLon;
      const Point quad[4] = {a, b, c, d};
      r.fillPolygon(quad, 4, terrainColor(terrain.elevationFt(lat, lon)));
    }
  }
}

// Ownship symbol, rotated to the aircraft heading in screen space. On a
// north-up map it turns with the aircraft; on a track-up map it shows the
// crab angle (heading minus track).
void drawOwnshipSymbol(Renderer& r, float cx, float cy, float size,
                       float rotationDeg) {
  const Point nose{0.0f, -size};
  const Point left{-size * 0.55f, size * 0.45f};
  const Point right{size * 0.55f, size * 0.45f};
  const Point tri[3] = {nose, left, right};
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(rotationDeg);
  r.fillPolygon(tri, 3, colors::kWhite);
  r.restore();
}

Color airspaceColor(AirspaceClass cls) {
  switch (cls) {
    case AirspaceClass::ClassB:
    case AirspaceClass::ClassD:
      return colors::kAirspaceBlue;
    case AirspaceClass::ClassC:
      return colors::kMagenta;
    case AirspaceClass::Restricted:
    case AirspaceClass::Prohibited:
    case AirspaceClass::Danger:
      return colors::kBandRed;
    default:
      return colors::kLabelText;
  }
}

// Class D boundaries are dashed in the Garmin scheme; everything else solid.
bool airspaceDashed(AirspaceClass cls) {
  return cls == AirspaceClass::ClassD;
}

// Draws a closed polyline through pre-projected boundary points (connecting the
// last point back to the first). Dashed mode walks each edge emitting fixed
// pixel-length dashes so arcs and straight segments dash consistently.
void drawBoundary(Renderer& r, const Point* pts, int count, float widthPx,
                  const Color& c, bool dashed) {
  if (count < 2) return;
  if (!dashed) {
    r.strokePolyline(pts, count, widthPx, c);
    // Close the ring.
    const Point closing[2] = {pts[count - 1], pts[0]};
    r.strokePolyline(closing, 2, widthPx, c);
    return;
  }

  constexpr float kDashPx = 6.0f;
  constexpr float kGapPx = 5.0f;
  float phase = 0.0f;  // distance carried across edges so dashes stay even
  for (int i = 0; i < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[(i + 1) % count];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    float pos = -phase;  // start partway in to honor the carried phase
    while (pos < len) {
      const float dashStart = std::max(pos, 0.0f);
      const float dashEnd = std::min(pos + kDashPx, len);
      if (dashEnd > dashStart) {
        const Point seg[2] = {{a.x + ux * dashStart, a.y + uy * dashStart},
                              {a.x + ux * dashEnd, a.y + uy * dashEnd}};
        r.strokePolyline(seg, 2, widthPx, c);
      }
      pos += kDashPx + kGapPx;
    }
    phase = std::fmod(phase + len, kDashPx + kGapPx);
  }
}

void drawRangeRing(Renderer& r, float cx, float cy, float radiusPx,
                   const Color& c) {
  constexpr int kSeg = 48;
  Point ring[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a =
        static_cast<float>(i) / static_cast<float>(kSeg) * 2.0f * 3.14159265f;
    ring[i] = {cx + radiusPx * std::cos(a), cy + radiusPx * std::sin(a)};
  }
  r.strokePolyline(ring, kSeg + 1, 1.5f, c);
}

Color featureColor(MapFeatureType type) {
  switch (type) {
    case MapFeatureType::Airport:
      return colors::kWhite;
    case MapFeatureType::Vor:
      return colors::kCyan;
    case MapFeatureType::Ndb:
      return colors::kActiveGreen;
    case MapFeatureType::Waypoint:
      return colors::kMagenta;
    case MapFeatureType::Fix:
    default:
      return colors::kLabelText;
  }
}

void drawFeature(Renderer& r, MapFeatureType type, float x, float y, float s,
                 const Color& c) {
  switch (type) {
    case MapFeatureType::Airport: {
      // Circle with a runway cross -- the Garmin towered-airport symbol, which
      // reads as an airport far better than a bare plus sign.
      constexpr int kSeg = 16;
      const float rad = s * 0.85f;
      Point ring[kSeg + 1];
      for (int i = 0; i <= kSeg; ++i) {
        const float a = static_cast<float>(i) / static_cast<float>(kSeg) *
                        2.0f * 3.14159265f;
        ring[i] = {x + rad * std::cos(a), y + rad * std::sin(a)};
      }
      r.strokePolyline(ring, kSeg + 1, 1.5f, c);
      r.strokeLine(x - rad, y, x + rad, y, 1.5f, c);
      r.strokeLine(x, y - rad, x, y + rad, 1.5f, c);
      break;
    }
    case MapFeatureType::Vor: {
      // Closed triangle outline (repeat the first vertex so the polyline closes).
      const Point tri[4] = {{x, y - s}, {x - s, y + s * 0.6f},
                            {x + s, y + s * 0.6f}, {x, y - s}};
      r.strokePolyline(tri, 4, 1.5f, c);
      break;
    }
    case MapFeatureType::Ndb:
      r.fillCircle(x, y, s * 0.4f, c);
      break;
    case MapFeatureType::Fix: {
      // Intersection: small solid triangle, distinct from the NDB's filled dot.
      const Point tri[3] = {{x, y - s * 0.7f}, {x - s * 0.6f, y + s * 0.5f},
                            {x + s * 0.6f, y + s * 0.5f}};
      r.fillPolygon(tri, 3, c);
      break;
    }
    default:
      r.fillCircle(x, y, s * 0.3f, c);
      break;
  }
}

}  // namespace

void MapView::render(Renderer& r, const MapData& map, const FlightData& flight,
                     const MapViewConfig& config, float displayH) {
  if (config.w <= 0.0f || config.h <= 0.0f) return;

  const float cx = config.x + config.w * 0.5f;
  const float cy = config.y + config.h * 0.5f;
  const float mapRadiusPx = 0.45f * std::min(config.w, config.h);
  // A per-view range override lets the MFD MAP page zoom independently of the
  // PFD inset while reading the same MapData.
  const float rangeNm =
      std::max(0.5f, config.rangeNm > 0.0f ? config.rangeNm : map.rangeNm);
  const float pixelsPerNm = mapRadiusPx / rangeNm;
  const float rotation = orientationDeg(config.orientation, flight);
  const float labelSize = fontPx(config.style.labelFontWt, displayH);

  const double viewCenterLat =
      config.hasCenterOverride ? config.centerLat : map.ownshipLat;
  const double viewCenterLon =
      config.hasCenterOverride ? config.centerLon : map.ownshipLon;

  r.save();
  r.clip(config.x, config.y, config.w, config.h);

  // Topographic terrain background, drawn first so everything else overlays it.
  // Falls back to the plain background when no terrain source is available.
  const bool terrainDrawn = config.style.showTerrain &&
                            map.terrain != nullptr && map.positionValid;
  if (terrainDrawn) {
    drawTerrain(r, *map.terrain, viewCenterLat, viewCenterLon, cx, cy,
                pixelsPerNm, rotation, rangeNm, config);
  }

  if (config.style.showChrome) {
    if (!terrainDrawn) {
      r.fillRect(config.x, config.y, config.w, config.h,
                 Color{0.0f, 0.0f, 0.0f, 0.82f});
    }
    r.strokeLine(config.x, config.y, config.x + config.w, config.y, 2.0f,
                 colors::kTapeTopBorder);
    r.strokeLine(config.x, config.y + config.h, config.x + config.w,
                 config.y + config.h, 2.0f, colors::kTapeTopBorder);
    r.strokeLine(config.x, config.y, config.x, config.y + config.h, 2.0f,
                 colors::kTapeTopBorder);
    r.strokeLine(config.x + config.w, config.y, config.x + config.w,
                 config.y + config.h, 2.0f, colors::kTapeTopBorder);
  }

  if (!map.positionValid) {
    if (config.style.showChrome) {
      r.fillText(cx, cy, "NO GPS POSITION", labelSize, TextAlign::Center,
                 colors::kLabelText);
    }
    r.restore();
    return;
  }

  if (config.style.showRangeRings) {
    drawRangeRing(r, cx, cy, mapRadiusPx, colors::kLabelText);
    drawRangeRing(r, cx, cy, mapRadiusPx * 0.5f, colors::kLabelText);
  }

  const double centerLat = viewCenterLat;
  const double centerLon = viewCenterLon;
  const float symSize = std::max(5.0f, fontPx(kFeatureSymbolWt, displayH));

  // Airspace boundaries draw beneath the route and features. An altitude
  // declutter hides airspace whose vertical band is far from ownship, matching
  // the G1000's behavior so distant overlying/underlying airspace isn't drawn.
  if (config.style.showAirspace && !map.airspaces.empty()) {
    constexpr float kAltMarginFt = 2000.0f;
    const float ownAlt = flight.altitudeValid ? flight.altitudeFt : 0.0f;
    std::vector<Point> ring;
    for (const MapAirspace& as : map.airspaces) {
      if (as.boundary.size() < 2) continue;
      if (flight.altitudeValid &&
          (as.floorFt > ownAlt + kAltMarginFt ||
           as.ceilingFt < ownAlt - kAltMarginFt)) {
        continue;
      }
      ring.clear();
      ring.reserve(as.boundary.size());
      for (const GeoPoint& g : as.boundary) {
        float x = 0.0f, y = 0.0f;
        map::latLonToLocalPx(g.lat, g.lon, centerLat, centerLon, cx, cy,
                             pixelsPerNm, rotation, x, y);
        ring.push_back({x, y});
      }
      drawBoundary(r, ring.data(), static_cast<int>(ring.size()), 1.5f,
                   airspaceColor(as.airspaceClass),
                   airspaceDashed(as.airspaceClass));
    }
  }

  if (config.style.showFlightPlan && map.flightPlan.size() >= 2) {
    // Garmin route coloring: the whole flight plan is white except the active
    // leg (and its TO waypoint), which is magenta. The active leg is located
    // by matching the FMS active waypoint ident against the route.
    int activeTo = -1;
    if (!flight.fmaToWpt.empty()) {
      for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
        if (map.flightPlan[i].id == flight.fmaToWpt) {
          activeTo = static_cast<int>(i);
          break;
        }
      }
    }

    std::vector<Point> route;
    route.reserve(map.flightPlan.size());
    for (const MapLeg& leg : map.flightPlan) {
      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(leg.lat, leg.lon, centerLat, centerLon, cx, cy,
                           pixelsPerNm, rotation, x, y);
      route.push_back({x, y});
    }
    r.strokePolyline(route.data(), static_cast<int>(route.size()), 2.0f,
                     colors::kWhite);
    if (activeTo >= 1) {
      const Point activeLeg[2] = {route[static_cast<size_t>(activeTo) - 1],
                                  route[static_cast<size_t>(activeTo)]};
      r.strokePolyline(activeLeg, 2, 2.0f, colors::kMagenta);
    }
    for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
      const Color c = static_cast<int>(i) == activeTo ? colors::kMagenta
                                                      : colors::kWhite;
      r.fillCircle(route[i].x, route[i].y, symSize * 0.35f, c);
      if (config.style.showLabels && !map.flightPlan[i].id.empty()) {
        r.fillText(route[i].x + symSize, route[i].y - symSize * 0.3f,
                   map.flightPlan[i].id, labelSize * 0.85f, TextAlign::Left,
                   c);
      }
    }
  }

  if (config.style.showFeatures) {
    // Range-based declutter, mirroring the G1000: drop the densest feature
    // classes as the range opens up so the map stays readable, and cap the
    // number of intersections drawn (the list is nearest-first, so the closest
    // ones win) since terminal areas hold hundreds of them.
    auto visibleAtRange = [&](MapFeatureType type) {
      switch (type) {
        case MapFeatureType::Airport:
          return rangeNm <= kFeatureRangeAirportNm;
        case MapFeatureType::Vor:
          return rangeNm <= kFeatureRangeVorNm;
        case MapFeatureType::Ndb:
          return rangeNm <= kFeatureRangeNdbNm;
        case MapFeatureType::Fix:
        case MapFeatureType::Waypoint:
          return rangeNm <= kFeatureRangeFixNm;
      }
      return true;
    };

    int fixesDrawn = 0;
    for (const MapFeature& f : map.features) {
      if (!visibleAtRange(f.type)) continue;
      const bool isFix =
          f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
      if (isFix && (!config.style.showFixes || fixesDrawn >= kMaxFixesDrawn)) {
        continue;
      }

      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(f.lat, f.lon, centerLat, centerLon, cx, cy,
                           pixelsPerNm, rotation, x, y);
      if (x < config.x - symSize || x > config.x + config.w + symSize ||
          y < config.y - symSize || y > config.y + config.h + symSize) {
        continue;
      }
      if (isFix) ++fixesDrawn;
      const Color c = featureColor(f.type);
      drawFeature(r, f.type, x, y, symSize, c);
      if (config.style.showLabels && !f.id.empty()) {
        r.fillText(x + symSize, y - symSize * 0.2f, f.id, labelSize * 0.85f,
                   TextAlign::Left, c);
      }
    }
  }

  // Ownship sits at the view center unless the center is overridden (the WPT/
  // NRST airport maps center on the airport), in which case project it.
  float ownX = cx, ownY = cy;
  if (config.hasCenterOverride) {
    map::latLonToLocalPx(map.ownshipLat, map.ownshipLon, centerLat, centerLon,
                         cx, cy, pixelsPerNm, rotation, ownX, ownY);
  }
  drawOwnshipSymbol(r, ownX, ownY, symSize, flight.headingDeg - rotation);

  if (config.style.showChrome) {
    char rangeBuf[16];
    std::snprintf(rangeBuf, sizeof(rangeBuf), "%dNM",
                  static_cast<int>(std::lround(rangeNm)));
    r.fillText(config.x + config.w * 0.06f, config.y + config.h * 0.12f,
               rangeBuf, labelSize, TextAlign::Left, colors::kWhite);
    r.fillText(config.x + config.w * 0.94f, config.y + config.h * 0.12f,
               orientationLabel(config.orientation), labelSize,
               TextAlign::Right, colors::kWhite);
  }

  r.restore();
}

}  // namespace avionics
