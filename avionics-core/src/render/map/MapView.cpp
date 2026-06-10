#include "avionics/render/MapView.h"

#include <cmath>
#include <cstdio>
#include <string>

#include "avionics/Color.h"
#include "render/map/MapProjection.h"

namespace avionics {
namespace {

constexpr float kWtCanvasHeight = 768.0f;

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

const char* orientationLabel(MapOrientation mode) {
  switch (mode) {
    case MapOrientation::NorthUp:
      return "NORTH";
    case MapOrientation::HeadingUp:
      return "HDG";
    case MapOrientation::TrackUp:
      return "TRK";
  }
  return "";
}

void drawOwnshipSymbol(Renderer& r, float cx, float cy, float size) {
  const Point nose{0.0f, -size};
  const Point left{-size * 0.55f, size * 0.45f};
  const Point right{size * 0.55f, size * 0.45f};
  const Point tri[3] = {nose, left, right};
  r.save();
  r.translate(cx, cy);
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
    case MapFeatureType::Airport:
      r.strokeLine(x - s, y, x + s, y, 1.5f, c);
      r.strokeLine(x, y - s, x, y + s, 1.5f, c);
      break;
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
  const float rangeNm = std::max(0.5f, map.rangeNm);
  const float pixelsPerNm = mapRadiusPx / rangeNm;
  const float rotation = orientationDeg(config.orientation, flight);
  const float labelSize = fontPx(config.style.labelFontWt, displayH);

  r.save();
  r.clip(config.x, config.y, config.w, config.h);

  if (config.style.showChrome) {
    r.fillRect(config.x, config.y, config.w, config.h,
               Color{0.0f, 0.0f, 0.0f, 0.82f});
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

  const double centerLat = map.ownshipLat;
  const double centerLon = map.ownshipLon;
  const float symSize = std::max(4.0f, config.w * 0.028f);

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
    std::vector<Point> route;
    route.reserve(map.flightPlan.size());
    for (const MapLeg& leg : map.flightPlan) {
      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(leg.lat, leg.lon, centerLat, centerLon, cx, cy,
                           pixelsPerNm, rotation, x, y);
      route.push_back({x, y});
    }
    if (route.size() >= 2) {
      r.strokePolyline(route.data(), static_cast<int>(route.size()), 2.0f,
                       colors::kMagenta);
    }
    for (const MapLeg& leg : map.flightPlan) {
      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(leg.lat, leg.lon, centerLat, centerLon, cx, cy,
                           pixelsPerNm, rotation, x, y);
      r.fillCircle(x, y, symSize * 0.35f, colors::kMagenta);
      if (config.style.showChrome && !leg.id.empty()) {
        r.fillText(x + symSize, y - symSize * 0.3f, leg.id, labelSize * 0.85f,
                   TextAlign::Left, colors::kMagenta);
      }
    }
  }

  if (config.style.showFeatures) {
    for (const MapFeature& f : map.features) {
      float x = 0.0f, y = 0.0f;
      map::latLonToLocalPx(f.lat, f.lon, centerLat, centerLon, cx, cy,
                           pixelsPerNm, rotation, x, y);
      if (x < config.x - symSize || x > config.x + config.w + symSize ||
          y < config.y - symSize || y > config.y + config.h + symSize) {
        continue;
      }
      const Color c = featureColor(f.type);
      drawFeature(r, f.type, x, y, symSize, c);
      if (config.style.showChrome && !f.id.empty()) {
        r.fillText(x + symSize, y - symSize * 0.2f, f.id, labelSize * 0.85f,
                   TextAlign::Left, c);
      }
    }
  }

  drawOwnshipSymbol(r, cx, cy, symSize);

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
