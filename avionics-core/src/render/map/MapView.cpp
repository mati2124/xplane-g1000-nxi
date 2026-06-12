#include "avionics/render/MapView.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "avionics/Terrain.h"
#include "avionics/WeatherRadar.h"
#include "avionics/render/MapSymbols.h"
#include "render/map/MapProjection.h"
#include "render/map/TerrainRaster.h"
#include "render/map/WeatherRaster.h"

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
// at most kMaxFixesDrawn of the nearest ones are drawn. Airports are not listed
// here: they declutter per-size against the Map Setup "Aviation" ranges carried
// in MapViewStyle (see kLargeAirportRunwayFt below).
constexpr float kFeatureRangeVorNm = 100.0f;
constexpr float kFeatureRangeNdbNm = 40.0f;
constexpr float kFeatureRangeFixNm = 7.5f;
constexpr int kMaxFixesDrawn = 40;

// Airport size classification by longest runway, mirroring Garmin's
// AirportWaypoint: a hard-surface runway >= 8100 ft is a Large airport, >= 5000
// ft (or any towered field) is Medium, and everything else is Small. Each size
// then declutters against its own Map Setup "Aviation" max range.
constexpr int kLargeAirportRunwayFt = 8100;
constexpr int kMediumAirportRunwayFt = 5000;

// Layer range declutter (NM), mirroring the G1000 Map Setup maximum-range
// defaults: each layer disappears once the range opens past its threshold.
constexpr float kAirwayMaxRangeNm = 80.0f;
constexpr float kAirwayLabelMaxRangeNm = 30.0f;
constexpr float kRunwayDiagramMaxRangeNm = 5.0f;
// Taxiways appear at the same range as the runway diagram so the airport's
// pavement shows as one picture the moment you zoom in, rather than the
// taxiways lagging a zoom step behind the runways.
constexpr float kTaxiwayDiagramMaxRangeNm = 5.0f;
// SafeTaxi identifier labels (runway numbers, taxiway letters) only appear at
// close range so they don't clutter the diagram when the whole field is small.
constexpr float kAirportLabelMaxRangeNm = 2.5f;
constexpr float kRoadMaxRangeNm = 60.0f;
constexpr float kRiverMaxRangeNm = 150.0f;
constexpr float kLakeMaxRangeNm = 150.0f;
// Railroads are a close-in detail feature on the NXi Land group; declutter
// past short range so their crosstie ticks don't clutter a wide view.
constexpr float kRailroadMaxRangeNm = 30.0f;
// State/province lines declutter past continental scale so a near-global view
// shows only nation borders and coastlines; nation borders have no cutoff.
constexpr float kStateBorderMaxRangeNm = 1000.0f;
constexpr float kObstacleMaxRangeNm = 20.0f;
// City label tiers by Natural Earth rank (higher rank = larger city).
constexpr float kCityLargeMaxRangeNm = 150.0f;
constexpr float kCityMediumMaxRangeNm = 50.0f;
constexpr float kCitySmallMaxRangeNm = 20.0f;
constexpr float kCityCapitalMaxRangeNm = 500.0f;

// Track vector lookahead (Map Setup "Track Vector", 60 sec default).
constexpr float kTrackVectorSeconds = 60.0f;
// Fuel range ring: reserve deducted from total endurance (45 min default).
constexpr float kFuelReserveHours = 0.75f;

// Land-data styling to sit under the white/cyan/magenta symbology like the
// Garmin base map: hydro features are medium blue (Fig 5-13/5-14, ~35/61/137),
// roads dark brown, borders gray.
constexpr Color kWaterFill{0.137f, 0.239f, 0.537f, 1.0f};
constexpr Color kRiverStroke{0.22f, 0.42f, 0.68f, 1.0f};
constexpr Color kRoadStroke{0.38f, 0.30f, 0.20f, 1.0f};
constexpr Color kBorderStroke{0.55f, 0.55f, 0.55f, 0.8f};
// State/province boundaries: dimmer than nation borders so the political
// hierarchy reads at a glance (nation lines dominate).
constexpr Color kStateBorderStroke{0.42f, 0.42f, 0.42f, 0.7f};
constexpr Color kCoastStroke{0.35f, 0.42f, 0.52f, 0.85f};
constexpr Color kRailroadStroke{0.62f, 0.62f, 0.62f, 0.8f};
constexpr Color kCityDot{0.85f, 0.78f, 0.45f, 1.0f};
// Low-altitude Victor/T-routes: gray, same shade as roads (Fig 5-15).
constexpr Color kAirwayLowStroke{0.55f, 0.55f, 0.55f, 0.75f};
constexpr Color kRunwayFill{0.75f, 0.75f, 0.78f, 1.0f};
// Taxiway/apron pavement: a darker gray so the lighter runway quads drawn on
// top stay distinct (Garmin SafeTaxi shows taxiways darker than runways).
constexpr Color kTaxiwayFill{0.40f, 0.40f, 0.43f, 1.0f};
// Cap on pavement-polygon vertices projected per frame, a safety valve against
// a pathological mega-polygon.
constexpr int kMaxPavementVerts = 512;

// Obstacle proximity coloring (same bands as relative terrain).
constexpr float kObstacleRedBelowFt = 100.0f;
constexpr float kObstacleYellowBelowFt = 1000.0f;

// Traffic advisory / vertical trend thresholds for the TIS symbology.
constexpr float kTrafficTrendFpm = 500.0f;

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

// Shared projection context for the layer painters: geographic center,
// scale/rotation, and the viewport bounds for culling.
struct Proj {
  double centerLat = 0.0;
  double centerLon = 0.0;
  float cx = 0.0f;
  float cy = 0.0f;
  float pixelsPerNm = 0.0f;
  float rotation = 0.0f;
  float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;

  // Per-frame projection constants, computed once in init(). The hot path
  // projects thousands of vertices per render (coastlines, airspaces, airways,
  // nav features), and the rotation sin/cos and longitude scale are identical
  // for every one of them, so deriving them once here instead of per point
  // removes ~3 trig calls per vertex from each map redraw.
  double cosR = 1.0;
  double sinR = 0.0;
  double nmPerLon = map::kNmPerDegLat;

  void init() {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double rot = static_cast<double>(rotation) * kDegToRad;
    cosR = std::cos(rot);
    sinR = std::sin(rot);
    nmPerLon = map::nmPerDegLon(centerLat);
  }

  void toPx(double lat, double lon, float& x, float& y) const {
    const double northNm = (lat - centerLat) * map::kNmPerDegLat;
    const double eastNm = (lon - centerLon) * nmPerLon;
    const double mapEast = eastNm * cosR - northNm * sinR;
    const double mapNorth = eastNm * sinR + northNm * cosR;
    x = cx + static_cast<float>(mapEast * static_cast<double>(pixelsPerNm));
    y = cy - static_cast<float>(mapNorth * static_cast<double>(pixelsPerNm));
  }

  bool onScreen(float x, float y, float margin) const {
    return x >= minX - margin && x <= maxX + margin && y >= minY - margin &&
           y <= maxY + margin;
  }
};

// Per-pixel symbology (dashes, comb teeth, railroad ties) steps along a line's
// full on-screen pixel length. At close map zoom an airspace/border edge can be
// enormous in pixels and reach far off-screen, so without clipping it generates
// hundreds of thousands of invisible segments (a ~100x cost cliff measured on
// the MFD MAP page). This clips one segment to the viewport rect (expanded by
// `margin`) and returns the visible distance interval [lo, hi] along it, so the
// generators can iterate only the portion that can actually be seen. Liang-
// Barsky parametric clip with t in [0, len] (direction is the unit vector).
// Returns false when the segment is entirely outside the rect.
inline bool segmentVisibleSpan(float ax, float ay, float ux, float uy,
                               float len, float minX, float minY, float maxX,
                               float maxY, float margin, float& lo, float& hi) {
  lo = 0.0f;
  hi = len;
  const float x0 = minX - margin, x1 = maxX + margin;
  const float y0 = minY - margin, y1 = maxY + margin;
  const float p[4] = {-ux, ux, -uy, uy};
  const float q[4] = {ax - x0, x1 - ax, ay - y0, y1 - ay};
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(p[i]) < 1e-6f) {
      if (q[i] < 0.0f) return false;  // parallel to this edge and outside it
    } else {
      const float t = q[i] / p[i];
      if (p[i] < 0.0f) {
        if (t > lo) lo = t;
      } else {
        if (t < hi) hi = t;
      }
    }
  }
  return hi >= lo;
}

// Viewport bounds passed to the per-pixel symbology generators so they can clip
// each edge before stepping along it. A null clip disables clipping (used by
// small, already-bounded callers like the fuel-reserve ring).
struct ClipBounds {
  float minX, minY, maxX, maxY;
};

// Margin (px) added around the viewport when clipping per-pixel symbology, so
// teeth/dashes near the edge are not cut early.
constexpr float kSymbologyClipMarginPx = 32.0f;

// Strokes an open polyline emitting fixed-length dashes (screen-space), used
// for borders and the fuel-reserve ring.
void strokeDashedPolyline(Renderer& r, const Point* pts, int count,
                          float widthPx, const Color& c,
                          const ClipBounds* clip = nullptr) {
  constexpr float kDashPx = 6.0f;
  constexpr float kGapPx = 5.0f;
  // Accumulate every dash as a disjoint segment and stroke them all in one
  // batched submission (a single draw call) instead of one per dash.
  static thread_local std::vector<Point> segs;
  segs.clear();
  float phase = 0.0f;
  for (int i = 0; i + 1 < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    constexpr float kPeriod = kDashPx + kGapPx;
    // Restrict the dash walk to the visible span of this edge; off-screen
    // portions would emit dashes that are never seen.
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;
      if (pos < lo) pos += std::floor((lo - pos) / kPeriod) * kPeriod;
      for (; pos < len && pos <= hi; pos += kPeriod) {
        // Clamp the dash to the visible span [lo, hi], not the full segment
        // [0, len]: a dash that starts before the visible region must be cut
        // at lo, otherwise it would extend into off-screen pixels.
        const float dashStart = std::max(pos, lo);
        const float dashEnd = std::min(pos + kDashPx, hi);
        if (dashEnd > dashStart) {
          segs.push_back({a.x + ux * dashStart, a.y + uy * dashStart});
          segs.push_back({a.x + ux * dashEnd, a.y + uy * dashEnd});
        }
      }
    }
    phase = std::fmod(phase + len, kPeriod);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), widthPx, c);
  }
}

// Railroad: a thin base line with periodic perpendicular crossties, matching
// the G1000 railroad symbol.
void drawRailroad(Renderer& r, const Point* pts, int count, const Color& c,
                  const ClipBounds* clip = nullptr) {
  if (count < 2) return;
  r.strokePolyline(pts, count, 1.0f, c);
  constexpr float kTickStepPx = 8.0f;
  constexpr float kTickHalfPx = 2.5f;
  // Batch all crossties into one stroke submission rather than one per tie.
  static thread_local std::vector<Point> segs;
  segs.clear();
  float phase = 0.0f;  // carry spacing across segments so ties stay even
  for (int i = 0; i + 1 < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    const float nx = -uy;  // perpendicular for the crosstie
    const float ny = ux;
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;
      if (pos < lo) pos += std::floor((lo - pos) / kTickStepPx) * kTickStepPx;
      for (; pos < len && pos <= hi; pos += kTickStepPx) {
        if (pos >= 0.0f) {
          const float px = a.x + ux * pos;
          const float py = a.y + uy * pos;
          segs.push_back({px - nx * kTickHalfPx, py - ny * kTickHalfPx});
          segs.push_back({px + nx * kTickHalfPx, py + ny * kTickHalfPx});
        }
      }
    }
    phase = std::fmod(phase + len, kTickStepPx);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), 1.0f, c);
  }
}

// Land data: lakes filled, rivers/roads/borders stroked, with per-class range
// declutter. Drawn right above the map background so everything overlays it.
void drawLandData(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm) {
  const ClipBounds clip{proj.minX, proj.minY, proj.maxX, proj.maxY};
  std::vector<Point> pts;
  for (const MapLandLine& line : map.landLines) {
    if (line.points.size() < 2) continue;
    if (line.landClass == LandClass::Road && rangeNm > kRoadMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::River && rangeNm > kRiverMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::Lake && rangeNm > kLakeMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::StateBorder &&
        rangeNm > kStateBorderMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::Railroad &&
        rangeNm > kRailroadMaxRangeNm) {
      continue;
    }

    pts.clear();
    pts.reserve(line.points.size());
    bool anyVisible = false;
    for (const GeoPoint& g : line.points) {
      float x = 0.0f, y = 0.0f;
      proj.toPx(g.lat, g.lon, x, y);
      anyVisible = anyVisible || proj.onScreen(x, y, 80.0f);
      pts.push_back({x, y});
    }
    if (!anyVisible) continue;

    const int n = static_cast<int>(pts.size());
    switch (line.landClass) {
      case LandClass::Lake:
        r.fillPolygon(pts.data(), n, kWaterFill);
        break;
      case LandClass::River:
        r.strokePolyline(pts.data(), n, 1.2f, kRiverStroke);
        break;
      case LandClass::Road:
        r.strokePolyline(pts.data(), n, 1.2f, kRoadStroke);
        break;
      case LandClass::Border:
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kBorderStroke, &clip);
        break;
      case LandClass::StateBorder:
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kStateBorderStroke, &clip);
        break;
      case LandClass::Coast:
        r.strokePolyline(pts.data(), n, 1.0f, kCoastStroke);
        break;
      case LandClass::Railroad:
        drawRailroad(r, pts.data(), n, kRailroadStroke, &clip);
        break;
      case LandClass::City:
        break;  // cities are point features (MapData::cities)
    }
  }
}

// Populated places: a dot plus name, decluttered by city rank vs. range.
void drawCities(Renderer& r, const MapData& map, const Proj& proj,
                float rangeNm, float symSize, float labelSize) {
  for (const MapLandCity& city : map.cities) {
    const float maxRange = city.rank >= 9   ? kCityCapitalMaxRangeNm
                           : city.rank >= 8 ? kCityLargeMaxRangeNm
                           : city.rank >= 4 ? kCityMediumMaxRangeNm
                                            : kCitySmallMaxRangeNm;
    if (rangeNm > maxRange) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(city.lat, city.lon, x, y);
    if (!proj.onScreen(x, y, symSize)) continue;
    r.fillCircle(x, y, symSize * 0.28f, kCityDot);
    r.fillText(x + symSize * 0.6f, y - symSize * 0.2f, city.name,
               labelSize * 0.8f, TextAlign::Left, kCityDot);
  }
}

void drawChromeBox(Renderer& r, float x, float y, float w, float h);
void drawAirwayLabel(Renderer& r, float cx, float cy, const char* text,
                     float size);

bool airwayLevelVisible(AirwayDisplay display, AirwayLevel level) {
  switch (display) {
    case AirwayDisplay::Off:
      return false;
    case AirwayDisplay::All:
      return true;
    case AirwayDisplay::Low:
      return level == AirwayLevel::Low;
    case AirwayDisplay::High:
      return level == AirwayLevel::High;
  }
  return false;
}

constexpr Color airwayStrokeFor(AirwayLevel level) {
  return level == AirwayLevel::High ? colors::kGreen : kAirwayLowStroke;
}

// One altitude class: gray (low) or green (high) edges with a boxed ident at
// each segment midpoint (Fig 5-15), filtered by the AWY softkey state.
void drawAirwaySegments(Renderer& r, const MapData& map, const Proj& proj,
                        AirwayDisplay display, AirwayLevel level, float rangeNm,
                        float labelSize) {
  if (!airwayLevelVisible(display, level)) return;
  const Color stroke = airwayStrokeFor(level);
  const float textSize = labelSize * 0.8f;
  for (const MapAirwaySegment& seg : map.airways) {
    if (seg.level != level) continue;
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    proj.toPx(seg.a.lat, seg.a.lon, ax, ay);
    proj.toPx(seg.b.lat, seg.b.lon, bx, by);
    if (!proj.onScreen(ax, ay, 120.0f) && !proj.onScreen(bx, by, 120.0f)) {
      continue;
    }
    r.strokeLine(ax, ay, bx, by, 1.0f, stroke);

    if (rangeNm <= kAirwayLabelMaxRangeNm && !seg.name.empty()) {
      const float mx = (ax + bx) * 0.5f;
      const float my = (ay + by) * 0.5f;
      if (!proj.onScreen(mx, my, 0.0f)) continue;
      drawAirwayLabel(r, mx, my, seg.name.c_str(), textSize);
    }
  }
}

// Airways declutter above kAirwayMaxRangeNm. Low-altitude routes draw first;
// high-altitude Jet/Q-routes draw on top when both are shown (Fig 5-15).
void drawAirways(Renderer& r, const MapData& map, const Proj& proj,
                 AirwayDisplay display, float rangeNm, float labelSize) {
  if (display == AirwayDisplay::Off || rangeNm > kAirwayMaxRangeNm) return;
  drawAirwaySegments(r, map, proj, display, AirwayLevel::Low, rangeNm,
                     labelSize);
  drawAirwaySegments(r, map, proj, display, AirwayLevel::High, rangeNm,
                     labelSize);
}

// Taxiway/apron diagram: each hard-surface pavement polygon filled once the
// range is at/below kTaxiwayDiagramMaxRangeNm. Drawn before (under) the runway
// quads so runways read on top, SafeTaxi-style.
void drawTaxiways(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm) {
  if (rangeNm > kTaxiwayDiagramMaxRangeNm) return;
  std::vector<Point> pts;
  for (const MapPavement& pav : map.taxiways) {
    const std::size_t count =
        std::min(pav.outline.size(), static_cast<std::size_t>(kMaxPavementVerts));
    if (count < 3) continue;
    pts.clear();
    pts.reserve(count);
    bool anyOnScreen = false;
    for (std::size_t i = 0; i < count; ++i) {
      float x = 0.0f, y = 0.0f;
      proj.toPx(pav.outline[i].lat, pav.outline[i].lon, x, y);
      if (proj.onScreen(x, y, 0.0f)) anyOnScreen = true;
      pts.push_back({x, y});
    }
    if (!anyOnScreen) continue;
    r.fillPolygon(pts.data(), static_cast<int>(pts.size()), kTaxiwayFill);
  }
}

// Paints a runway-end designator near its threshold, rotated to read along the
// runway centerline (flipped so it never appears upside down), like the painted
// numbers on the SafeTaxi diagram.
void drawRunwayNumber(Renderer& r, float ex, float ey, float dirX, float dirY,
                      const std::string& id, float labelSize) {
  if (id.empty()) return;
  const float len = std::sqrt(dirX * dirX + dirY * dirY);
  if (len < 1.0f) return;
  const float ux = dirX / len;
  const float uy = dirY / len;
  // Inset the label from the threshold toward the runway center.
  const float inset = labelSize * 1.2f;
  const float tx = ex + ux * inset;
  const float ty = ey + uy * inset;
  float angle = std::atan2(uy, ux) * 180.0f / 3.14159265358979323846f;
  if (angle > 90.0f) angle -= 180.0f;
  if (angle < -90.0f) angle += 180.0f;
  r.save();
  r.translate(tx, ty);
  r.rotateDegrees(angle);
  r.fillText(0.0f, 0.0f, id, labelSize * 0.95f, TextAlign::Center,
             colors::kWhite);
  r.restore();
}

// Runway diagrams: each runway drawn as a filled pavement quad once the range
// is at/below kRunwayDiagramMaxRangeNm, replacing the symbolic airport ring
// visually (the symbol still draws underneath at these ranges). The runway-end
// numbers paint on top at close range.
void drawRunways(Renderer& r, const MapData& map, const Proj& proj,
                 float rangeNm, float labelSize) {
  if (rangeNm > kRunwayDiagramMaxRangeNm) return;
  constexpr float kMetersPerNm = 1852.0f;
  const bool showNumbers = rangeNm <= kAirportLabelMaxRangeNm;
  for (const MapRunway& rwy : map.runways) {
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    proj.toPx(rwy.a.lat, rwy.a.lon, ax, ay);
    proj.toPx(rwy.b.lat, rwy.b.lon, bx, by);
    if (!proj.onScreen(ax, ay, 200.0f) && !proj.onScreen(bx, by, 200.0f)) {
      continue;
    }
    const float dx = bx - ax;
    const float dy = by - ay;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f) continue;
    // Perpendicular half-width in pixels (floor so short ranges still show a
    // visible strip).
    const float halfW = std::max(
        1.5f, (rwy.widthM / kMetersPerNm) * proj.pixelsPerNm * 0.5f);
    const float px = -dy / len * halfW;
    const float py = dx / len * halfW;
    const Point quad[4] = {{ax + px, ay + py},
                           {bx + px, by + py},
                           {bx - px, by - py},
                           {ax - px, ay - py}};
    r.fillPolygon(quad, 4, kRunwayFill);
    // Only label runways long enough on screen to hold their numbers.
    if (showNumbers && len > labelSize * 4.0f) {
      drawRunwayNumber(r, ax, ay, dx, dy, rwy.idA, labelSize);
      drawRunwayNumber(r, bx, by, -dx, -dy, rwy.idB, labelSize);
    }
  }
}

// SafeTaxi taxiway identifier labels: white text in a small dark box at each
// named taxiway, drawn at close range only so the diagram stays uncluttered.
void drawTaxiwayLabels(Renderer& r, const MapData& map, const Proj& proj,
                       float rangeNm, float labelSize) {
  if (rangeNm > kAirportLabelMaxRangeNm) return;
  const float fontPx = labelSize * 0.85f;
  for (const MapTaxiwayLabel& label : map.taxiwayLabels) {
    if (label.text.empty()) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.pos.lat, label.pos.lon, x, y);
    if (!proj.onScreen(x, y, 0.0f)) continue;
    const float halfW = r.measureTextWidth(label.text, fontPx) * 0.5f + 2.5f;
    const float halfH = fontPx * 0.5f + 1.5f;
    r.fillRect(x - halfW, y - halfH, halfW * 2.0f, halfH * 2.0f,
               Color{0.0f, 0.0f, 0.0f, 0.72f});
    r.fillText(x, y - fontPx * 0.5f, label.text, fontPx, TextAlign::Center,
               colors::kWhite);
  }
}

// Obstacles (FAA DOF): tower symbol colored by proximity below ownship like
// relative terrain (red within 100 ft, amber within 1000 ft, else white).
void drawObstacles(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, float ownAltFt, bool altValid,
                   float symSize) {
  if (rangeNm > kObstacleMaxRangeNm) return;
  for (const MapObstacle& ob : map.obstacles) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(ob.lat, ob.lon, x, y);
    if (!proj.onScreen(x, y, symSize)) continue;

    Color c = colors::kWhite;
    if (altValid) {
      const float rel = ob.mslFt - ownAltFt;
      if (rel >= -kObstacleRedBelowFt) {
        c = colors::kBandRed;
      } else if (rel >= -kObstacleYellowBelowFt) {
        c = colors::kBandYellow;
      }
    }
    // Slender obstacle/tower glyph: tall open triangle with a tip dot.
    const float s = symSize * 0.9f;
    const Point tri[4] = {{x, y - s},
                          {x - s * 0.45f, y + s * 0.5f},
                          {x + s * 0.45f, y + s * 0.5f},
                          {x, y - s}};
    r.strokePolyline(tri, 4, 1.5f, c);
    r.fillCircle(x, y - s, 1.5f, c);
  }
}

// Traffic overlay, TIS symbology: open white diamond for other traffic, solid
// yellow circle for a Traffic Advisory; relative altitude in hundreds of feet
// (+ above / - below) with a climb/descend arrow when trending.
void drawTraffic(Renderer& r, const MapData& map, const Proj& proj,
                 float symSize, float labelSize) {
  char tag[8];
  for (const MapTraffic& t : map.traffic) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(t.lat, t.lon, x, y);
    if (!proj.onScreen(x, y, symSize * 2.0f)) continue;

    const Color c = t.trafficAdvisory ? colors::kBandYellow : colors::kWhite;
    const float s = symSize * 0.75f;
    if (t.trafficAdvisory) {
      r.fillCircle(x, y, s * 0.8f, c);
    } else {
      const Point diamond[5] = {
          {x, y - s}, {x + s, y}, {x, y + s}, {x - s, y}, {x, y - s}};
      r.strokePolyline(diamond, 5, 1.8f, c);
    }

    // Relative altitude tag in hundreds of feet, above the symbol when the
    // target is above ownship, below when below (TIS display convention).
    const int relHundreds =
        static_cast<int>(std::lround(t.relAltFt / 100.0f));
    std::snprintf(tag, sizeof(tag), "%+03d", relHundreds);
    const float tagY = t.relAltFt >= 0.0f ? y - s - labelSize * 0.55f
                                          : y + s + labelSize * 0.55f;
    r.fillText(x, tagY, tag, labelSize * 0.85f, TextAlign::Center, c);

    // Vertical trend arrow beside the symbol.
    if (std::fabs(t.verticalSpeedFpm) >= kTrafficTrendFpm) {
      const bool up = t.verticalSpeedFpm > 0.0f;
      const float axCol = x + s + labelSize * 0.4f;
      const float a0 = up ? y + s * 0.6f : y - s * 0.6f;
      const float a1 = up ? y - s * 0.6f : y + s * 0.6f;
      r.strokeLine(axCol, a0, axCol, a1, 1.5f, c);
      const float head = up ? 1.0f : -1.0f;
      r.strokeLine(axCol, a1, axCol - s * 0.3f, a1 + head * s * 0.4f, 1.5f, c);
      r.strokeLine(axCol, a1, axCol + s * 0.3f, a1 + head * s * 0.4f, 1.5f, c);
    }
  }
}

// Track vector: a line projected along the current ground track covering the
// next kTrackVectorSeconds of travel (Map Setup "Track Vector").
void drawTrackVector(Renderer& r, const FlightData& flight, float ownX,
                     float ownY, float pixelsPerNm, float rotation) {
  if (flight.groundSpeedKts < 30.0f) return;
  const float lenNm = flight.groundSpeedKts * kTrackVectorSeconds / 3600.0f;
  const float lenPx = lenNm * pixelsPerNm;
  const float angleRad =
      (flight.trackDeg - rotation) * 3.14159265f / 180.0f;
  const float ex = ownX + lenPx * std::sin(angleRad);
  const float ey = ownY - lenPx * std::cos(angleRad);
  r.strokeLine(ownX, ownY, ex, ey, 2.0f, colors::kCyan);
  // Small crossbar tip so the lookahead end is readable.
  const float tx = std::cos(angleRad) * 4.0f;
  const float ty = std::sin(angleRad) * 4.0f;
  r.strokeLine(ex - tx, ey - ty, ex + tx, ey + ty, 2.0f, colors::kCyan);
}

// Rounded map-chrome box (the NXi "NORTH UP" / range / wind-vector plates):
// dark fill with a 1px light-gray border. Returns nothing; the caller places
// the contents.
void drawChromeBox(Renderer& r, float x, float y, float w, float h) {
  constexpr int kCornerSegs = 4;
  constexpr int kPts = 4 * (kCornerSegs + 1);
  const float radius = std::min(h * 0.32f, w * 0.5f);
  struct Corner {
    float cx, cy, a0;
  };
  const Corner corners[4] = {
      {x + radius, y + radius, 3.14159265f},
      {x + w - radius, y + radius, 4.71238898f},
      {x + w - radius, y + h - radius, 0.0f},
      {x + radius, y + h - radius, 1.57079633f},
  };
  Point pts[kPts + 1];
  int n = 0;
  for (const Corner& c : corners) {
    for (int s = 0; s <= kCornerSegs; ++s) {
      const float a =
          c.a0 + 1.57079633f * static_cast<float>(s) / kCornerSegs;
      pts[n++] = {c.cx + radius * std::cos(a), c.cy + radius * std::sin(a)};
    }
  }
  pts[n] = pts[0];
  r.fillPolygon(pts, kPts, Color{0.0f, 0.0f, 0.0f, 0.78f});
  r.strokePolyline(pts, n + 1, 1.0f, Color{0.75f, 0.75f, 0.75f, 1.0f});
}

// Boxed airway ident centered on the segment (Fig 5-15).
void drawAirwayLabel(Renderer& r, float cx, float cy, const char* text,
                     float size) {
  const float padX = size * 0.35f;
  const float w = r.measureTextWidth(text, size) + 2.0f * padX;
  const float h = size * 1.35f;
  const float x = cx - w * 0.5f;
  const float y = cy - h * 0.5f;
  drawChromeBox(r, x, y, w, h);
  r.fillText(cx, y + h * 0.52f, text, size, TextAlign::Center, colors::kWhite);
}

// Boxed single-text chrome label ("NORTH UP", Fig 5-2/5-26: cyan text on the
// rounded plate). x/y is the top-left corner; returns the box height.
float drawChromeLabel(Renderer& r, float x, float y, const char* text,
                      float size, const Color& textColor) {
  const float padX = size * 0.45f;
  const float w = r.measureTextWidth(text, size) + 2.0f * padX;
  const float h = size * 1.5f;
  drawChromeBox(r, x, y, w, h);
  r.fillText(x + padX, y + h * 0.52f, text, size, TextAlign::Left, textColor);
  return h;
}

// North indicator below the orientation label (Fig 5-26): a white compass
// arrow with "N" near its head.
void drawNorthArrow(Renderer& r, float cx, float cy, float size,
                    float rotation) {
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotation);
  const float h = size;        // half height
  const float w = size * 0.45f;  // half width
  // Two mirrored halves so the arrow reads like the printed compass glyph.
  const Point left[3] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}};
  const Point right[3] = {{0.0f, -h}, {w, h}, {0.0f, h * 0.45f}};
  r.fillPolygon(left, 3, colors::kWhite);
  r.fillPolygon(right, 3, Color{0.62f, 0.62f, 0.62f, 1.0f});
  const Point outline[5] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}, {w, h},
                            {0.0f, -h}};
  r.strokePolyline(outline, 5, 1.0f, Color{0.2f, 0.2f, 0.2f, 1.0f});
  r.fillText(0.0f, 0.0f, "N", size * 0.8f, TextAlign::Center,
             Color{0.1f, 0.1f, 0.1f, 1.0f});
  r.restore();
}

// Range readout on the outer range ring (Fig 5-2 "15NM"): cyan value with the
// smaller unit suffix on a chrome plate, centered on the ring's upper-left.
void drawRangeLabel(Renderer& r, float x, float y, float rangeNm,
                    float labelSize, bool centerOnPoint) {
  char buf[16];
  formatMapRange(buf, sizeof(buf), rangeNm);
  // Split the value digits from the unit suffix so the unit renders smaller.
  std::size_t unitStart = 0;
  while (buf[unitStart] != '\0' &&
         ((buf[unitStart] >= '0' && buf[unitStart] <= '9') ||
          buf[unitStart] == '.')) {
    ++unitStart;
  }
  const std::string value(buf, unitStart);
  const std::string unit(buf + unitStart);
  const float unitSize = labelSize * 0.72f;
  const float padX = labelSize * 0.45f;
  const float textW = r.measureTextWidth(value, labelSize) +
                      r.measureTextWidth(unit, unitSize);
  const float w = textW + 2.0f * padX;
  const float h = labelSize * 1.5f;
  const float bx = centerOnPoint ? x - w * 0.5f : x - w;
  const float by = centerOnPoint ? y - h * 0.5f : y - h;
  drawChromeBox(r, bx, by, w, h);
  r.fillText(bx + padX, by + h * 0.52f, value, labelSize, TextAlign::Left,
             colors::kCyan);
  r.fillText(bx + padX + r.measureTextWidth(value, labelSize), by + h * 0.52f,
             unit, unitSize, TextAlign::Left, colors::kCyan);
}

// Wind vector (Fig 5-18): a chrome plate in the upper right holding a white
// arrow that points where the wind blows toward (rotated with the map) and
// the speed with a small KT suffix.
void drawWindVector(Renderer& r, const FlightData& flight,
                    const MapViewConfig& config, float rotation,
                    float labelSize) {
  if (!flight.windValid || flight.windSpeedKts < 1.0f) return;
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%d",
                static_cast<int>(std::lround(flight.windSpeedKts)));
  const float speedSize = labelSize * 1.25f;
  const float unitSize = speedSize * 0.72f;
  const float arrowLen = labelSize * 1.5f;
  const float padX = labelSize * 0.5f;
  const float w = arrowLen + r.measureTextWidth(buf, speedSize) +
                  r.measureTextWidth("KT", unitSize) + 2.6f * padX;
  const float h = labelSize * 2.1f;
  const float bx = config.x + config.w - w - labelSize * 0.6f;
  const float by = config.y + labelSize * 2.4f;
  drawChromeBox(r, bx, by, w, h);

  // Arrow in the left cell, pointing where the wind blows toward.
  const float cx = bx + padX + arrowLen * 0.5f;
  const float cy = by + h * 0.5f;
  const float len = arrowLen;
  const float angleRad =
      (flight.windDirectionDeg + 180.0f - rotation) * 3.14159265f / 180.0f;
  const float dx = std::sin(angleRad);
  const float dy = -std::cos(angleRad);
  const float ax = cx - dx * len * 0.5f;
  const float ay = cy - dy * len * 0.5f;
  const float ex = cx + dx * len * 0.5f;
  const float ey = cy + dy * len * 0.5f;
  r.strokeLine(ax, ay, ex, ey, 2.5f, colors::kWhite);
  const float hx = -dy, hy = dx;
  const Point head[3] = {
      {ex, ey},
      {ex - dx * len * 0.42f + hx * len * 0.26f,
       ey - dy * len * 0.42f + hy * len * 0.26f},
      {ex - dx * len * 0.42f - hx * len * 0.26f,
       ey - dy * len * 0.42f - hy * len * 0.26f}};
  r.fillPolygon(head, 3, colors::kWhite);

  const float tx = bx + padX * 1.6f + arrowLen;
  r.fillText(tx, cy, buf, speedSize, TextAlign::Left, colors::kWhite);
  r.fillText(tx + r.measureTextWidth(buf, speedSize), cy, "KT", unitSize,
             TextAlign::Left, colors::kWhite);
}

// Fuel range ring: solid ring at the total-endurance range, dashed ring at
// the range remaining once the 45-minute reserve is protected.
void drawFuelRing(Renderer& r, const FlightData& flight, float ownX,
                  float ownY, float pixelsPerNm, float viewRadiusPx) {
  const float fuelGal = flight.fuelQtyLeftGal + flight.fuelQtyRightGal;
  if (flight.fuelFlowGph < 0.5f || fuelGal <= 0.0f ||
      flight.groundSpeedKts < 30.0f) {
    return;
  }
  const float enduranceH = fuelGal / flight.fuelFlowGph;
  const float reserveH = std::max(0.0f, enduranceH - kFuelReserveHours);
  const float totalPx =
      flight.groundSpeedKts * enduranceH * pixelsPerNm;
  const float reservePx =
      flight.groundSpeedKts * reserveH * pixelsPerNm;
  // Skip when even the reserve ring is far outside the viewport.
  if (reservePx > viewRadiusPx * 4.0f) return;

  constexpr int kSeg = 72;
  Point ring[kSeg + 1];
  auto buildRing = [&](float radius) {
    for (int i = 0; i <= kSeg; ++i) {
      const float a = static_cast<float>(i) / kSeg * 2.0f * 3.14159265f;
      ring[i] = {ownX + radius * std::cos(a), ownY + radius * std::sin(a)};
    }
  };
  if (reservePx > 1.0f) {
    buildRing(reservePx);
    strokeDashedPolyline(r, ring, kSeg + 1, 1.5f, colors::kActiveGreen);
  }
  if (totalPx <= viewRadiusPx * 4.0f) {
    buildRing(totalPx);
    r.strokePolyline(ring, kSeg + 1, 1.5f, colors::kActiveGreen);
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

// How an airspace boundary is stroked, mirroring the Garmin SDK
// MapAirspaceRendering: Class B solid blue, Class C solid maroon, Class D
// dashed blue, the restricted group blue with comb teeth, and MOA/Alert maroon
// with comb teeth. Types Garmin renders elsewhere map to a thin dashed line.
enum class AirspaceStroke { None, Solid, Dashed, Combed };

struct AirspaceRenderStyle {
  Color color;
  AirspaceStroke stroke;
};

AirspaceRenderStyle airspaceStyle(AirspaceClass cls) {
  switch (cls) {
    case AirspaceClass::ClassB:
      return {colors::kAirspaceBlue, AirspaceStroke::Solid};
    case AirspaceClass::ClassC:
      return {colors::kAirspaceMaroon, AirspaceStroke::Solid};
    case AirspaceClass::ClassD:
      return {colors::kAirspaceBlue, AirspaceStroke::Dashed};
    case AirspaceClass::Restricted:
    case AirspaceClass::Prohibited:
    case AirspaceClass::Warning:
    case AirspaceClass::Danger:
    case AirspaceClass::Training:
      return {colors::kAirspaceBlue, AirspaceStroke::Combed};
    case AirspaceClass::MOA:
    case AirspaceClass::Alert:
      return {colors::kAirspaceMaroon, AirspaceStroke::Combed};
    case AirspaceClass::TFR:
      return {colors::kBandRed, AirspaceStroke::Solid};
    case AirspaceClass::Caution:
    case AirspaceClass::TRSA:
    case AirspaceClass::ADIZ:
    case AirspaceClass::Other:
      return {colors::kLabelText, AirspaceStroke::Dashed};
    default:
      return {colors::kLabelText, AirspaceStroke::None};
  }
}

// Largest map range (NM) at which an airspace boundary is still drawn, mirroring
// the NXi Map Setup "Airspace" group default range per class (Working Title
// MapUserSettings: Class D defaults to the 10 NM ladder step, every other class
// to the 50 NM step). Above the class's range the boundary declutters off so a
// zoomed-out map isn't buried in outlines.
constexpr float kAirspaceClassDMaxRangeNm = 10.0f;
constexpr float kAirspaceDefaultMaxRangeNm = 50.0f;

float airspaceMaxDisplayRangeNm(AirspaceClass cls) {
  switch (cls) {
    case AirspaceClass::ClassD:
      return kAirspaceClassDMaxRangeNm;
    default:
      return kAirspaceDefaultMaxRangeNm;
  }
}

// Draws a closed polyline through pre-projected boundary points (connecting the
// last point back to the first). Dashed mode walks each edge emitting fixed
// pixel-length dashes so arcs and straight segments dash consistently.
void drawBoundary(Renderer& r, const Point* pts, int count, float widthPx,
                  const Color& c, bool dashed, const ClipBounds* clip = nullptr) {
  if (count < 2) return;
  if (!dashed) {
    r.strokePolyline(pts, count, widthPx, c);
    // Close the ring.
    const Point closing[2] = {pts[count - 1], pts[0]};
    r.strokePolyline(closing, 2, widthPx, c);
    return;
  }

  // Garmin MapSingleLineAirspaceRenderer uses a 5/5 dash for Class D.
  constexpr float kDashPx = 5.0f;
  constexpr float kGapPx = 5.0f;
  constexpr float kPeriod = kDashPx + kGapPx;
  static thread_local std::vector<Point> segs;
  segs.clear();
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
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;  // start partway in to honor the carried phase
      if (pos < lo) pos += std::floor((lo - pos) / kPeriod) * kPeriod;
      for (; pos < len && pos <= hi; pos += kPeriod) {
        // Clamp the dash to the visible span [lo, hi], not the full segment
        // [0, len]: a dash that starts before the visible region must be cut
        // at lo, otherwise it would extend into off-screen pixels.
        const float dashStart = std::max(pos, lo);
        const float dashEnd = std::min(pos + kDashPx, hi);
        if (dashEnd > dashStart) {
          segs.push_back({a.x + ux * dashStart, a.y + uy * dashStart});
          segs.push_back({a.x + ux * dashEnd, a.y + uy * dashEnd});
        }
      }
    }
    phase = std::fmod(phase + len, kPeriod);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), widthPx, c);
  }
}

// Special-use airspace (Restricted/Prohibited/MOA/...) draws a solid base line
// with a comb of short teeth pointing into the airspace, matching the Garmin
// SDK CombedAirspaceRenderer (a base line plus an inward-offset toothed line).
void drawCombedBoundary(Renderer& r, const Point* pts, int count,
                        float widthPx, const Color& c,
                        const ClipBounds* clip = nullptr) {
  if (count < 2) return;
  // Solid base ring.
  r.strokePolyline(pts, count, widthPx, c);
  const Point closing[2] = {pts[count - 1], pts[0]};
  r.strokePolyline(closing, 2, widthPx, c);

  // Centroid gives the inward direction for the teeth.
  float cxSum = 0.0f, cySum = 0.0f;
  for (int i = 0; i < count; ++i) {
    cxSum += pts[i].x;
    cySum += pts[i].y;
  }
  const float cx = cxSum / static_cast<float>(count);
  const float cy = cySum / static_cast<float>(count);

  constexpr float kStepPx = 5.0f;   // spacing between teeth
  constexpr float kToothPx = 5.0f;  // tooth length (inward)
  static thread_local std::vector<Point> segs;
  segs.clear();
  float phase = 0.0f;
  for (int i = 0; i < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[(i + 1) % count];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    // Inward normal: pick the perpendicular pointing toward the centroid.
    float nx = -uy;
    float ny = ux;
    const float midX = (a.x + b.x) * 0.5f;
    const float midY = (a.y + b.y) * 0.5f;
    if (nx * (cx - midX) + ny * (cy - midY) < 0.0f) {
      nx = -nx;
      ny = -ny;
    }
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;  // carry the spacing across edges so teeth stay even
      if (pos < lo) pos += std::floor((lo - pos) / kStepPx) * kStepPx;
      for (; pos < len && pos <= hi; pos += kStepPx) {
        if (pos >= 0.0f) {
          const float px = a.x + ux * pos;
          const float py = a.y + uy * pos;
          segs.push_back({px, py});
          segs.push_back({px + nx * kToothPx, py + ny * kToothPx});
        }
      }
    }
    phase = std::fmod(phase + len, kStepPx);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), widthPx, c);
  }
}

// Relative-terrain legend, shown while TER REL is enabled on a map (NXi
// Pilot's Guide, Hazard Avoidance: "an icon to indicate the feature is
// enabled for display and a legend for the relative terrain colors are
// shown"). Two color bands: red within 100 ft below ownship, yellow within
// 1000 ft.
void drawRelTerrainLegend(Renderer& r, const MapViewConfig& config,
                          float labelSize) {
  const float pad = labelSize * 0.45f;
  const float rowH = labelSize * 1.25f;
  const float chipW = labelSize * 1.5f;
  const float w = labelSize * 7.2f;
  const float h = rowH * 3.0f + pad;
  const float x = config.x + config.w * 0.02f;
  const float y = config.y + config.h - h - config.h * 0.02f;

  r.fillRect(x, y, w, h, Color{0.0f, 0.0f, 0.0f, 0.78f});
  r.strokeLine(x, y, x + w, y, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y + h, x + w, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y, x, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x + w, y, x + w, y + h, 1.0f, colors::kPanelBorder);

  r.fillText(x + w * 0.5f, y + pad + rowH * 0.35f, "TERRAIN",
             labelSize * 0.85f, TextAlign::Center, colors::kWhite);

  struct Row {
    Color chip;
    const char* label;
  };
  const Row rows[2] = {{colors::kBandRed, "-100 FT"},
                       {colors::kBandYellow, "-1000 FT"}};
  for (int i = 0; i < 2; ++i) {
    const float rowY = y + pad + rowH * (0.9f + static_cast<float>(i));
    r.fillRect(x + pad, rowY, chipW, rowH * 0.6f, rows[i].chip);
    r.fillText(x + pad + chipW + pad, rowY + rowH * 0.3f, rows[i].label,
               labelSize * 0.8f, TextAlign::Left, colors::kLabelText);
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
  // The on-screen scale follows the animated zoom value (when supplied) so the
  // map glides between ladder steps, while `rangeNm` -- the selected step --
  // still drives the readout, range rings, and symbol declutter so those don't
  // flicker through intermediate values during the zoom.
  const float scaleRangeNm =
      std::max(0.5f, config.displayRangeNm > 0.0f ? config.displayRangeNm
                                                  : rangeNm);
  const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
  const float rotation = orientationDeg(config.orientation, flight);
  const float labelSize = fontPx(config.style.labelFontWt, displayH);

  const double viewCenterLat =
      config.hasCenterOverride ? config.centerLat : map.ownshipLat;
  const double viewCenterLon =
      config.hasCenterOverride ? config.centerLon : map.ownshipLon;

  r.save();
  r.clip(config.x, config.y, config.w, config.h);

  // Terrain background (topo or relative), drawn first so everything else
  // overlays it. Falls back to the plain background when no terrain source is
  // available or the raster has not finished its first build yet.
  bool terrainDrawn = false;
  if (config.style.terrain != TerrainDisplay::Off && map.terrain != nullptr &&
      map.positionValid) {
    const map::TerrainRasterMode mode =
        config.style.terrain == TerrainDisplay::Rel
            ? map::TerrainRasterMode::Relative
            : map::TerrainRasterMode::Absolute;
    terrainDrawn = map::drawTerrainRaster(
        r, *map.terrain, mode, flight.altitudeValid ? flight.altitudeFt : 0.0f,
        viewCenterLat, viewCenterLon, cx, cy, pixelsPerNm, rotation, rangeNm);
  }

  if (config.style.showWeather && map.weather != nullptr &&
      map.weather->active() && map.positionValid) {
    map::drawWeatherRaster(r, *map.weather, cx, cy, pixelsPerNm, rotation);
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

  const double centerLat = viewCenterLat;
  const double centerLon = viewCenterLon;
  const float symSize = std::max(5.0f, fontPx(kFeatureSymbolWt, displayH));

  Proj proj;
  proj.centerLat = centerLat;
  proj.centerLon = centerLon;
  proj.cx = cx;
  proj.cy = cy;
  proj.pixelsPerNm = pixelsPerNm;
  proj.rotation = rotation;
  proj.minX = config.x;
  proj.minY = config.y;
  proj.maxX = config.x + config.w;
  proj.maxY = config.y + config.h;
  proj.init();

  // Land data sits directly on the background (under everything, including
  // the range rings); lakes/rivers are skipped over the terrain raster's
  // water-colored TOPO base only when terrain is off-screen -- they overlay
  // consistently either way.
  if (config.style.showLand &&
      (!map.landLines.empty() || !map.cities.empty())) {
    drawLandData(r, map, proj, rangeNm);
    if (config.style.showLabels) {
      drawCities(r, map, proj, rangeNm, symSize, labelSize);
    }
  }

  if (config.style.showRangeRings) {
    drawRangeRing(r, cx, cy, mapRadiusPx, colors::kLabelText);
    drawRangeRing(r, cx, cy, mapRadiusPx * 0.5f, colors::kLabelText);
  }

  // Airspace boundaries draw beneath the route and features. Two declutters
  // match the G1000: a per-class map-range cap hides each airspace once zoomed
  // out past its Map Setup range, and an altitude declutter hides airspace
  // whose vertical band is far from ownship so distant overlying/underlying
  // airspace isn't drawn.
  if (config.style.showAirspace && !map.airspaces.empty()) {
    constexpr float kAltMarginFt = 2000.0f;
    const float ownAlt = flight.altitudeValid ? flight.altitudeFt : 0.0f;
    std::vector<Point> ring;
    for (const MapAirspace& as : map.airspaces) {
      if (as.boundary.size() < 2) continue;
      const AirspaceRenderStyle style = airspaceStyle(as.airspaceClass);
      if (style.stroke == AirspaceStroke::None) continue;
      if (rangeNm > airspaceMaxDisplayRangeNm(as.airspaceClass)) continue;
      if (flight.altitudeValid &&
          (as.floorFt > ownAlt + kAltMarginFt ||
           as.ceilingFt < ownAlt - kAltMarginFt)) {
        continue;
      }
      ring.clear();
      ring.reserve(as.boundary.size());
      for (const GeoPoint& g : as.boundary) {
        float x = 0.0f, y = 0.0f;
        proj.toPx(g.lat, g.lon, x, y);
        ring.push_back({x, y});
      }
      const int n = static_cast<int>(ring.size());
      const ClipBounds clip{proj.minX, proj.minY, proj.maxX, proj.maxY};
      if (style.stroke == AirspaceStroke::Combed) {
        drawCombedBoundary(r, ring.data(), n, 1.5f, style.color, &clip);
      } else {
        drawBoundary(r, ring.data(), n, 1.5f, style.color,
                     style.stroke == AirspaceStroke::Dashed, &clip);
      }
    }
  }

  // Airways draw above airspace but under the route and nav features.
  if (!map.airways.empty()) {
    drawAirways(r, map, proj, config.style.airways, rangeNm, labelSize);
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
      proj.toPx(leg.lat, leg.lon, x, y);
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

  // Procedure preview (PROC menu): dashed cyan course through the published
  // fixes, drawn over the route like the NXi procedure preview.
  if (config.procedurePreview != nullptr &&
      config.procedurePreview->size() >= 2) {
    std::vector<Point> preview;
    preview.reserve(config.procedurePreview->size());
    for (const MapLeg& leg : *config.procedurePreview) {
      float x = 0.0f, y = 0.0f;
      proj.toPx(leg.lat, leg.lon, x, y);
      preview.push_back({x, y});
    }
    strokeDashedPolyline(r, preview.data(), static_cast<int>(preview.size()),
                         2.0f, colors::kCyan);
    for (std::size_t i = 0; i < preview.size(); ++i) {
      r.fillCircle(preview[i].x, preview[i].y, symSize * 0.32f, colors::kCyan);
      if (config.style.showLabels &&
          !(*config.procedurePreview)[i].id.empty()) {
        r.fillText(preview[i].x + symSize, preview[i].y - symSize * 0.3f,
                   (*config.procedurePreview)[i].id, labelSize * 0.85f,
                   TextAlign::Left, colors::kCyan);
      }
    }
  }

  // Direct-To course: a magenta line from the aircraft straight to the
  // direct-to waypoint, drawn over the flight plan (G1000 GPS Direct-To). The
  // origin tracks ownship, so the leg shrinks as the aircraft flies toward it.
  if (config.style.showFlightPlan && map.directToActive && map.positionValid) {
    float ox = 0.0f, oy = 0.0f, tx = 0.0f, ty = 0.0f;
    proj.toPx(map.ownshipLat, map.ownshipLon, ox, oy);
    proj.toPx(map.directTo.lat, map.directTo.lon, tx, ty);
    const Point dto[2] = {{ox, oy}, {tx, ty}};
    r.strokePolyline(dto, 2, 2.0f, colors::kMagenta);
    r.fillCircle(tx, ty, symSize * 0.35f, colors::kMagenta);
    if (config.style.showLabels && !map.directTo.id.empty()) {
      r.fillText(tx + symSize, ty - symSize * 0.3f, map.directTo.id,
                 labelSize * 0.85f, TextAlign::Left, colors::kMagenta);
    }
  }

  // Taxiway/apron pavement at very close range, under the runway quads.
  if (config.style.showTaxiways && config.style.showFeatures &&
      !map.taxiways.empty()) {
    drawTaxiways(r, map, proj, rangeNm);
  }

  // Runway pavement quads at close range, under the airport symbols/labels.
  if (config.style.showRunways && config.style.showFeatures &&
      !map.runways.empty()) {
    drawRunways(r, map, proj, rangeNm, labelSize);
  }

  // SafeTaxi taxiway identifier labels, on top of the pavement.
  if (config.style.showTaxiways && config.style.showFeatures &&
      config.style.showLabels && !map.taxiwayLabels.empty()) {
    drawTaxiwayLabels(r, map, proj, rangeNm, labelSize);
  }

  if (config.style.showFeatures) {
    // Range-based declutter, mirroring the G1000: drop the densest feature
    // classes as the range opens up so the map stays readable, and cap the
    // number of intersections drawn (the list is nearest-first, so the closest
    // ones win) since terminal areas hold hundreds of them.
    auto visibleAtRange = [&](MapFeatureType type) {
      switch (type) {
        case MapFeatureType::Airport:
          return true;  // airports declutter by size (airportVisible)
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

    // Airports declutter per-size against the Map Setup "Aviation" ranges, so a
    // wide view keeps the major (Large) fields long after the small ones drop.
    auto airportVisible = [&](const MapFeature& f) {
      if (f.longestRunwayFt >= kLargeAirportRunwayFt) {
        return config.style.showLargeAirports &&
               rangeNm <= config.style.largeAirportRangeNm;
      }
      if (f.longestRunwayFt >= kMediumAirportRunwayFt || f.airportTowered) {
        return config.style.showMediumAirports &&
               rangeNm <= config.style.mediumAirportRangeNm;
      }
      return config.style.showSmallAirports &&
             rangeNm <= config.style.smallAirportRangeNm;
    };

    int fixesDrawn = 0;
    for (const MapFeature& f : map.features) {
      if (f.type == MapFeatureType::Airport) {
        if (!airportVisible(f)) continue;
      } else if (!visibleAtRange(f.type)) {
        continue;
      }
      const bool isFix =
          f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
      if (isFix && (!config.style.showFixes || fixesDrawn >= kMaxFixesDrawn)) {
        continue;
      }

      float x = 0.0f, y = 0.0f;
      proj.toPx(f.lat, f.lon, x, y);
      if (x < config.x - symSize || x > config.x + config.w + symSize ||
          y < config.y - symSize || y > config.y + config.h + symSize) {
        continue;
      }
      if (isFix) ++fixesDrawn;
      drawMapFeatureSymbol(r, f, x, y, symSize);
      const Color labelC = mapFeatureColor(f);
      if (config.style.showLabels && !f.id.empty()) {
        r.fillText(x + symSize, y - symSize * 0.2f, f.id, labelSize * 0.85f,
                   TextAlign::Left, labelC);
      }
    }
  }

  // Obstacles draw with the nav features (same declutter switch).
  if (config.style.showObstacles && config.style.showFeatures &&
      !map.obstacles.empty()) {
    drawObstacles(r, map, proj, rangeNm,
                  flight.altitudeValid ? flight.altitudeFt : 0.0f,
                  flight.altitudeValid, symSize);
  }

  // Ownship sits at the view center unless the center is overridden (the WPT/
  // NRST airport maps center on the airport), in which case project it.
  float ownX = cx, ownY = cy;
  if (config.hasCenterOverride) {
    proj.toPx(map.ownshipLat, map.ownshipLon, ownX, ownY);
  }

  if (config.style.showFuelRing) {
    drawFuelRing(r, flight, ownX, ownY, pixelsPerNm, mapRadiusPx);
  }
  if (config.style.showTrackVector) {
    drawTrackVector(r, flight, ownX, ownY, pixelsPerNm, rotation);
  }

  // Traffic overlays everything except ownship and the chrome.
  if (config.style.showTraffic && !map.traffic.empty()) {
    drawTraffic(r, map, proj, symSize, labelSize);
  }

  drawOwnshipSymbol(r, ownX, ownY, symSize, flight.headingDeg - rotation);

  if (config.style.showWindVector) {
    drawWindVector(r, flight, config, rotation, labelSize);
  }

  if (config.style.terrain == TerrainDisplay::Rel && terrainDrawn) {
    drawRelTerrainLegend(r, config, labelSize);
  }

  if (config.style.showChrome || config.style.showOrientationLabel) {
    // Boxed orientation annunciation in the top-left corner with the north
    // indicator below it (Fig 5-2 / Fig 5-26).
    const float ox = config.x + labelSize * 0.45f;
    const float oy = config.y + labelSize * 0.35f;
    const float boxH = drawChromeLabel(r, ox, oy,
                                       orientationLabel(config.orientation),
                                       labelSize, colors::kCyan);
    if (config.style.showNorthArrow) {
      drawNorthArrow(r, ox + labelSize * 1.2f,
                     oy + boxH + labelSize * 1.6f, labelSize * 1.2f, rotation);
    }

    // Boxed range readout: on the outer range ring when rings are shown,
    // otherwise tucked into the lower-right corner (PFD inset).
    if (config.style.showRangeRings) {
      drawRangeLabel(r, cx - mapRadiusPx * 0.7071f, cy - mapRadiusPx * 0.7071f,
                     rangeNm, labelSize, true);
    } else {
      drawRangeLabel(r, config.x + config.w - labelSize * 0.4f,
                     config.y + config.h - labelSize * 0.35f, rangeNm,
                     labelSize, false);
    }
  }

  r.restore();
}

}  // namespace avionics
