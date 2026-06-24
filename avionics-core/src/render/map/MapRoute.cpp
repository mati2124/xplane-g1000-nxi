#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"

namespace avionics::mapview {
namespace {

int activeFlightPlanToIndex(const MapData& map, const FlightData& flight) {
  if (flight.fmaActiveLegIndex >= 0 &&
      flight.fmaActiveLegIndex < static_cast<int>(map.flightPlan.size())) {
    return flight.fmaActiveLegIndex;
  }
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

bool resolveNavIdentCoords(const MapData& map, const std::string& id,
                           double& lat, double& lon) {
  if (id.empty()) return false;
  for (const MapFeature& f : map.features) {
    if (f.id == id) {
      lat = f.lat;
      lon = f.lon;
      return true;
    }
  }
  return false;
}

// Active GPS Direct-To target for map rendering: the display overlay when set,
// otherwise the sim destination when there is no stored plan leg to highlight.
bool activeDirectNavTarget(const MapData& map, const FlightData& flight,
                           MapLeg& out) {
  if (map.directToActive && !map.directTo.id.empty()) {
    out = map.directTo;
    if (out.lat == 0.0 && out.lon == 0.0 &&
        !resolveNavIdentCoords(map, out.id, out.lat, out.lon)) {
      return false;
    }
    return true;
  }
  // When a flight plan is on the map, leg highlighting covers navigation; a
  // second ownship-to-fix line duplicates the active-leg segment (common after
  // loading an approach while fmaFromWpt is still empty).
  if (map.flightPlan.size() >= 2) return false;
  if (flight.fmaToWpt.empty() || !flight.fmaFromWpt.empty()) return false;
  out.id = flight.fmaToWpt;
  return resolveNavIdentCoords(map, out.id, out.lat, out.lon);
}

// Loaded approach / procedure tail in the active flight plan (includes untagged
// feeder fixes before the first IAF, matching the FPL page grouping).
bool procedureBlockForMap(const std::vector<MapLeg>& plan, int& start,
                          int& count) {
  const InferredProcedureBlock block = inferProcedureBlockInPlan(plan);
  if (block.start < 0 || block.count < 2) return false;
  start = block.start;
  count = block.count;
  return true;
}

int procedureBlockStartIndex(const MapData& map) {
  int start = -1;
  int count = 0;
  if (!procedureBlockForMap(map.flightPlan, start, count)) return 0;
  return start;
}

// Last match mirrors FPL page Direct-To resolution when duplicate idents exist.
int directToTargetIndexInPlan(const MapData& map) {
  if (!map.directToActive || map.directTo.id.empty()) return -1;
  for (int i = static_cast<int>(map.flightPlan.size()) - 1; i >= 0; --i) {
    if (map.flightPlan[static_cast<std::size_t>(i)].id == map.directTo.id) {
      return i;
    }
  }
  return -1;
}

int mapRouteSliceStart(const MapData& map) {
  if (!map.directToActive) return 0;
  const int dtoIdx = directToTargetIndexInPlan(map);
  if (dtoIdx >= 0) return dtoIdx;
  return procedureBlockStartIndex(map);
}

std::vector<MapLeg> legsForMapRoute(const MapData& map) {
  if (!map.directToActive || map.flightPlan.size() < 2) {
    return map.flightPlan;
  }
  const int dtoIdx = directToTargetIndexInPlan(map);
  if (dtoIdx >= 0) {
    // In-plan Direct-To: drop bypassed legs before the target (e.g. UZAWO→BUTLY
    // when flying direct to BUTLY); keep the remaining flight plan tail.
    return std::vector<MapLeg>(map.flightPlan.begin() + dtoIdx,
                               map.flightPlan.end());
  }
  int procStart = -1;
  int procCount = 0;
  if (!procedureBlockForMap(map.flightPlan, procStart, procCount)) {
    // GPS Direct-To with no loaded procedure: only the magenta ownship course
    // should appear; stale sim FMS legs must not draw a white route.
    return {};
  }
  return std::vector<MapLeg>(
      map.flightPlan.begin() + procStart,
      map.flightPlan.begin() + procStart + procCount);
}

}  // namespace

void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  const std::vector<MapLeg> routeLegs = legsForMapRoute(map);
  if (routeLegs.size() < 2) return;

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const SmoothedRoute route = buildSmoothedRoute(routeLegs, proj);
  const int procOffset = map.directToActive ? mapRouteSliceStart(map) : 0;

  drawSmoothedRoutePolyline(r, route, 2.0f, colors::kWhite);
  // In-plan Direct-To draws the magenta course from ownship to the target;
  // do not also highlight a plan leg (index math differs once the route is
  // sliced to the loaded procedure during approach Direct-To).
  if (!map.directToActive && activeTo >= 0) {
    const int activeToInRoute = activeTo - procOffset;
    if (activeToInRoute >= 1 &&
        activeToInRoute < static_cast<int>(routeLegs.size())) {
      const std::size_t leg = static_cast<std::size_t>(activeToInRoute) - 1;
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
  }

  for (std::size_t i = 0; i < routeLegs.size(); ++i) {
    const Point pt = projectLeg(proj, routeLegs[i]);
    const int planIndex = static_cast<int>(i) + procOffset;
    const Color c =
        (!map.directToActive && planIndex == activeTo) ? colors::kMagenta
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

  const std::vector<MapLeg> routeLegs = legsForMapRoute(map);
  if (routeLegs.empty()) return;

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const float textSize = labelSize * kMapIdentLabelScale;
  const int procOffset = map.directToActive ? mapRouteSliceStart(map) : 0;

  for (std::size_t i = 0; i < routeLegs.size(); ++i) {
    if (routeLegs[i].id.empty()) continue;
    const Point pt = projectLeg(proj, routeLegs[i]);
    const int planIndex = static_cast<int>(i) + procOffset;
    const Color c =
        (!map.directToActive && planIndex == activeTo) ? colors::kMagenta
                                                       : colors::kWhite;
    r.fillText(pt.x, pt.y - symSize * 0.95f - kMapLabelLiftPx, routeLegs[i].id,
               textSize, TextAlign::Center, c, kMapLabelFace);
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

void drawDirectToCourse(Renderer& r, const MapData& map,
                        const FlightData& flight, const Proj& proj,
                        const MapViewConfig& config, float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  MapLeg target;
  if (!activeDirectNavTarget(map, flight, target)) return;

  double fromLat = map.ownshipLat;
  double fromLon = map.ownshipLon;
  if (map.directToOriginValid) {
    fromLat = map.directToOriginLat;
    fromLon = map.directToOriginLon;
  }
  float ox = 0.0f, oy = 0.0f, tx = 0.0f, ty = 0.0f;
  proj.toPx(fromLat, fromLon, ox, oy);
  proj.toPx(target.lat, target.lon, tx, ty);
  const Point dto[2] = {{ox, oy}, {tx, ty}};
  r.strokePolyline(dto, 2, 2.0f, colors::kMagenta);
  r.fillCircle(tx, ty, symSize * 0.35f, colors::kMagenta);
}

void drawDirectToCourseLabel(Renderer& r, const MapData& map,
                             const FlightData& flight, const Proj& proj,
                             const MapViewConfig& config, float symSize,
                             float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm || !config.style.showLabels) {
    return;
  }

  MapLeg target;
  if (!activeDirectNavTarget(map, flight, target) || target.id.empty()) {
    return;
  }

  float tx = 0.0f, ty = 0.0f;
  proj.toPx(target.lat, target.lon, tx, ty);
  r.fillText(tx, ty - symSize * 0.95f - kMapLabelLiftPx, target.id,
             labelSize * kMapIdentLabelScale, TextAlign::Center,
             colors::kMagenta, kMapLabelFace);
}

}  // namespace avionics::mapview
