#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace avionics::mapview {
namespace {

int activeFlightPlanToIndex(const MapData& map, const FlightData& flight) {
  if (flight.fmaToWpt.empty()) return -1;
  for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
    if (map.flightPlan[i].id == flight.fmaToWpt) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// Fly-by turn radius on the map (NM). LNAV anticipates turns before each
// waypoint rather than overflying the fix; 2 NM is a reasonable GA display
// value at typical MAP zoom ranges.
constexpr float kFlyByTurnRadiusNm = 2.0f;
constexpr int kRouteArcSegments = 8;
constexpr float kMinSmoothTurnDeg = 8.0f;

float pointDist(const Point& a, const Point& b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

Point projectLeg(const Proj& proj, const MapLeg& leg) {
  float x = 0.0f, y = 0.0f;
  proj.toPx(leg.lat, leg.lon, x, y);
  return {x, y};
}

struct SmoothedRoute {
  std::vector<Point> points;
  // Per-leg span in `points` for the leg from waypoint k to waypoint k+1.
  std::vector<std::size_t> legStart;
  std::vector<std::size_t> legEnd;
};

// Replace sharp waypoint corners with fly-by quadratic arcs. Waypoint symbols
// stay at the published fix locations; only the course line is smoothed
// tangent through the turns.
SmoothedRoute buildSmoothedRoute(const std::vector<MapLeg>& legs,
                                 const Proj& proj) {
  SmoothedRoute out;
  if (legs.empty()) return out;

  std::vector<Point> wpts;
  wpts.reserve(legs.size());
  for (const MapLeg& leg : legs) {
    wpts.push_back(projectLeg(proj, leg));
  }

  const std::size_t n = wpts.size();
  if (n == 1) {
    out.points = wpts;
    return out;
  }

  const float cornerRadiusPx =
      std::max(4.0f, proj.pixelsPerNm * kFlyByTurnRadiusNm);

  out.points.push_back(wpts.front());
  out.legStart.resize(n - 1);
  out.legEnd.resize(n - 1);
  out.legStart[0] = 0;

  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Point& prev = wpts[i - 1];
    const Point& cur = wpts[i];
    const Point& next = wpts[i + 1];
    const float lenIn = pointDist(prev, cur);
    const float lenOut = pointDist(cur, next);
    if (lenIn < 1e-3f || lenOut < 1e-3f) {
      out.points.push_back(cur);
      out.legEnd[i - 1] = out.points.size() - 1;
      out.legStart[i] = out.points.size() - 1;
      continue;
    }

    const float inUx = (cur.x - prev.x) / lenIn;
    const float inUy = (cur.y - prev.y) / lenIn;
    const float outUx = (next.x - cur.x) / lenOut;
    const float outUy = (next.y - cur.y) / lenOut;
    const float dot =
        std::clamp(inUx * outUx + inUy * outUy, -1.0f, 1.0f);
    const float turnDeg =
        std::acos(dot) * 180.0f / static_cast<float>(M_PI);

    float rad = cornerRadiusPx;
    rad = std::min(rad, 0.45f * std::min(lenIn, lenOut));

    if (turnDeg < kMinSmoothTurnDeg || rad < 2.0f) {
      out.points.push_back(cur);
      out.legEnd[i - 1] = out.points.size() - 1;
      out.legStart[i] = out.points.size() - 1;
      continue;
    }

    const Point t1{cur.x - inUx * rad, cur.y - inUy * rad};
    const Point t2{cur.x + outUx * rad, cur.y + outUy * rad};
    out.points.push_back(t1);
    out.legEnd[i - 1] = out.points.size() - 1;

    for (int s = 1; s <= kRouteArcSegments; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(kRouteArcSegments);
      const float u = 1.0f - t;
      out.points.push_back(
          {u * u * t1.x + 2.0f * u * t * cur.x + t * t * t2.x,
           u * u * t1.y + 2.0f * u * t * cur.y + t * t * t2.y});
    }
    out.legStart[i] = out.points.size() - 1;
  }

  out.points.push_back(wpts.back());
  out.legEnd[n - 2] = out.points.size() - 1;

  return out;
}

void drawSmoothedRoutePolyline(Renderer& r, const SmoothedRoute& route,
                               float width, const Color& color) {
  if (route.points.size() < 2) return;
  r.strokePolyline(route.points.data(),
                   static_cast<int>(route.points.size()), width, color);
}

void drawSmoothedRouteDashedPolyline(Renderer& r, const SmoothedRoute& route,
                                     float width, const Color& color) {
  if (route.points.size() < 2) return;
  strokeDashedPolyline(r, route.points.data(),
                       static_cast<int>(route.points.size()), width, color);
}

}  // namespace

void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;
  if (map.flightPlan.size() < 2) return;

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const SmoothedRoute route = buildSmoothedRoute(map.flightPlan, proj);

  drawSmoothedRoutePolyline(r, route, 2.0f, colors::kWhite);
  if (activeTo >= 1) {
    const std::size_t leg = static_cast<std::size_t>(activeTo) - 1;
    if (leg < route.legStart.size()) {
      const std::size_t start = route.legStart[leg];
      const std::size_t end = route.legEnd[leg];
      if (end > start && end < route.points.size()) {
        r.strokePolyline(route.points.data() + start,
                         static_cast<int>(end - start + 1), 2.0f,
                         colors::kMagenta);
      }
    }
  }

  for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
    const Point pt = projectLeg(proj, map.flightPlan[i]);
    const Color c = static_cast<int>(i) == activeTo ? colors::kMagenta
                                                    : colors::kWhite;
    r.fillCircle(pt.x, pt.y, symSize * 0.35f, c);
  }
}

void drawFlightPlanLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, const FlightData& flight,
                          float symSize, float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm || !config.style.showLabels) {
    return;
  }

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const float textSize = labelSize * kMapIdentLabelScale;

  for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
    if (map.flightPlan[i].id.empty()) continue;
    const Point pt = projectLeg(proj, map.flightPlan[i]);
    const Color c = static_cast<int>(i) == activeTo ? colors::kMagenta
                                                    : colors::kWhite;
    r.fillText(pt.x, pt.y - symSize * 0.95f - kMapLabelLiftPx,
               map.flightPlan[i].id, textSize, TextAlign::Center, c,
               kMapLabelFace);
  }
}

void drawProcedurePreview(Renderer& r, const Proj& proj,
                          const MapViewConfig& config, float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;
  if (config.procedurePreview == nullptr ||
      config.procedurePreview->size() < 2) {
    return;
  }

  const SmoothedRoute preview =
      buildSmoothedRoute(*config.procedurePreview, proj);
  drawSmoothedRouteDashedPolyline(r, preview, 2.0f, colors::kCyan);

  for (const MapLeg& leg : *config.procedurePreview) {
    const Point pt = projectLeg(proj, leg);
    r.fillCircle(pt.x, pt.y, symSize * 0.32f, colors::kCyan);
  }
}

void drawProcedurePreviewLabels(Renderer& r, const Proj& proj,
                                const MapViewConfig& config, float symSize,
                                float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm || !config.style.showLabels) {
    return;
  }

  const float textSize = labelSize * kMapIdentLabelScale;
  for (const MapLeg& leg : *config.procedurePreview) {
    if (leg.id.empty()) continue;
    const Point pt = projectLeg(proj, leg);
    r.fillText(pt.x, pt.y - symSize * 0.95f - kMapLabelLiftPx, leg.id, textSize,
               TextAlign::Center, colors::kCyan, kMapLabelFace);
  }
}

void drawDirectToCourse(Renderer& r, const MapData& map, const Proj& proj,
                        const MapViewConfig& config, float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  float ox = 0.0f, oy = 0.0f, tx = 0.0f, ty = 0.0f;
  proj.toPx(map.ownshipLat, map.ownshipLon, ox, oy);
  proj.toPx(map.directTo.lat, map.directTo.lon, tx, ty);
  const Point dto[2] = {{ox, oy}, {tx, ty}};
  r.strokePolyline(dto, 2, 2.0f, colors::kMagenta);
  r.fillCircle(tx, ty, symSize * 0.35f, colors::kMagenta);
}

void drawDirectToCourseLabel(Renderer& r, const MapData& map, const Proj& proj,
                             const MapViewConfig& config, float symSize,
                             float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm || !config.style.showLabels ||
      map.directTo.id.empty()) {
    return;
  }

  float tx = 0.0f, ty = 0.0f;
  proj.toPx(map.directTo.lat, map.directTo.lon, tx, ty);
  r.fillText(tx, ty - symSize * 0.95f - kMapLabelLiftPx, map.directTo.id,
             labelSize * kMapIdentLabelScale, TextAlign::Center,
             colors::kMagenta, kMapLabelFace);
}

}  // namespace avionics::mapview
