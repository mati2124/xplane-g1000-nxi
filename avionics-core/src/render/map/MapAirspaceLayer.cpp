#include "render/map/MapViewInternal.h"

#include <cmath>
#include <vector>

namespace avionics::mapview {
namespace {

// How an airspace boundary is stroked, mirroring the Garmin SDK
// MapAirspaceRendering: Class B solid blue, Class C solid maroon, Class D
// dashed blue, the restricted group blue with comb teeth, and MOA/Alert maroon
// with comb teeth. Types Garmin renders elsewhere map to a thin dashed line.
enum class AirspaceStroke { None, Solid, Dashed, Combed };

struct AirspaceRenderStyle {
  Color color;
  AirspaceStroke stroke;
  float widthPx;
};

// Standard airspace boundary stroke width. Class C is drawn a touch heavier so
// its thin solid maroon ring stays legible against the map's terrain shading.
constexpr float kAirspaceWidthPx = 1.5f;
constexpr float kAirspaceClassCWidthPx = 2.5f;

AirspaceRenderStyle airspaceStyle(AirspaceClass cls) {
  switch (cls) {
    case AirspaceClass::ClassB:
      return {colors::kAirspaceBlue, AirspaceStroke::Solid, kAirspaceWidthPx};
    case AirspaceClass::ClassC:
      return {colors::kAirspaceMaroon, AirspaceStroke::Solid,
              kAirspaceClassCWidthPx};
    case AirspaceClass::ClassD:
      return {colors::kAirspaceBlue, AirspaceStroke::Dashed, kAirspaceWidthPx};
    case AirspaceClass::Restricted:
    case AirspaceClass::Prohibited:
    case AirspaceClass::Warning:
    case AirspaceClass::Danger:
    case AirspaceClass::Training:
      return {colors::kAirspaceBlue, AirspaceStroke::Combed, kAirspaceWidthPx};
    case AirspaceClass::MOA:
    case AirspaceClass::Alert:
      return {colors::kAirspaceMaroon, AirspaceStroke::Combed, kAirspaceWidthPx};
    case AirspaceClass::TFR:
      return {colors::kBandRed, AirspaceStroke::Solid, kAirspaceWidthPx};
    case AirspaceClass::Caution:
    case AirspaceClass::TRSA:
    case AirspaceClass::ADIZ:
    case AirspaceClass::Other:
      return {colors::kLabelText, AirspaceStroke::Dashed, kAirspaceWidthPx};
    default:
      return {colors::kLabelText, AirspaceStroke::None, kAirspaceWidthPx};
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
                  const Color& c, bool dashed, const ClipBounds* clip) {
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
                        float widthPx, const Color& c, const ClipBounds* clip) {
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

}  // namespace

void drawAirspaces(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, const FlightData& flight, bool pointerActive,
                   double pointerLat, double pointerLon) {
  // Two declutters match the G1000: a per-class map-range cap hides each
  // airspace once zoomed out past its Map Setup range, and an altitude
  // declutter hides airspace whose vertical band is far from ownship so
  // distant overlying/underlying airspace isn't drawn.
  constexpr float kAltMarginFt = 2000.0f;
  // The boundary the pan pointer is over is redrawn in white and a touch
  // heavier, like the real unit highlighting a selected airspace.
  constexpr float kAirspaceHighlightWidthPx = 3.0f;
  const float ownAlt = flight.altitudeValid ? flight.altitudeFt : 0.0f;
  std::vector<Point> ring;
  for (const MapAirspace& as : map.airspaces) {
    if (as.boundary.size() < 2) continue;
    const AirspaceRenderStyle style = airspaceStyle(as.airspaceClass);
    if (style.stroke == AirspaceStroke::None) continue;
    // A highlighted (cursor-selected) airspace always draws, even past its
    // declutter range/altitude, so the selection the pointer box reports is
    // always visible on the map.
    const bool highlighted =
        pointerActive && airspaceContainsPoint(as, pointerLat, pointerLon);
    if (!highlighted) {
      if (rangeNm > airspaceMaxDisplayRangeNm(as.airspaceClass)) continue;
      if (flight.altitudeValid &&
          (as.floorFt > ownAlt + kAltMarginFt ||
           as.ceilingFt < ownAlt - kAltMarginFt)) {
        continue;
      }
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
    const Color color = highlighted ? colors::kWhite : style.color;
    const float widthPx =
        highlighted ? kAirspaceHighlightWidthPx : style.widthPx;
    if (style.stroke == AirspaceStroke::Combed) {
      drawCombedBoundary(r, ring.data(), n, widthPx, color, &clip);
    } else {
      drawBoundary(r, ring.data(), n, widthPx, color,
                   style.stroke == AirspaceStroke::Dashed, &clip);
    }
  }
}

}  // namespace avionics::mapview
