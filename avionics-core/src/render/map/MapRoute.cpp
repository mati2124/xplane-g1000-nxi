#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/HoldGeometry.h"
#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

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
constexpr int kHoldArcSegments = 10;
constexpr float kRfArcSegmentDeg = 6.0f;
constexpr int kRfArcMinSegments = 4;
constexpr int kRfArcMaxSegments = 48;

void drawHoldRacetrack(Renderer& r, const MapLeg& leg, const Proj& proj,
                       float width, const Color& color, float groundSpeedKts) {
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, groundSpeedKts);
  if (!geom.valid) return;

  const auto latLonPts = tessellateHoldRacetrack(geom, kHoldArcSegments);
  if (latLonPts.size() < 2) return;

  std::vector<Point> pts;
  pts.reserve(latLonPts.size());
  for (const auto& ll : latLonPts) {
    float x = 0.0f;
    float y = 0.0f;
    proj.toPx(ll.first, ll.second, x, y);
    pts.push_back({x, y});
  }
  r.strokePolyline(pts.data(), static_cast<int>(pts.size()), width, color);
}

float flightPlanRouteWidth(float symSize) {
  return std::max(3.0f, symSize * kFlightPlanRouteWidthWt / kFeatureSymbolWt);
}

float missedApproachRouteWidth(float symSize) {
  return std::max(1.0f, symSize * kMissedApproachRouteWidthWt / kFeatureSymbolWt);
}

int findMaptIndexInRoute(const std::vector<MapLeg>& legs) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (legs[static_cast<std::size_t>(i)].procedureRole == "mapt") {
      return i;
    }
  }
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    const std::string& id = legs[static_cast<std::size_t>(i)].id;
    if (id.size() >= 4 && id.rfind("RW", 0) == 0 &&
        std::isdigit(static_cast<unsigned char>(id[2]))) {
      return i;
    }
  }
  return -1;
}

bool missedApproachSegmentActive(int maptIdx, int activeTo) {
  return maptIdx >= 0 && activeTo > maptIdx;
}

// Boxed flight-plan fix ident (PC Trainer screenshot013: black plate, white or
// magenta border/text centered on the published fix location).
void drawRouteIdentBox(Renderer& r, float cx, float cy, const std::string& text,
                       float textSize, const Color& color) {
  if (text.empty()) return;
  const float padX = textSize * 0.35f;
  const float w =
      r.measureTextWidth(text, textSize, kMapLabelFace) + 2.0f * padX;
  const float h = textSize * 1.35f;
  const float x = cx - w * 0.5f;
  const float y = cy - h * 0.5f;
  const Point box[5] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}};
  r.fillPolygon(box, 4, Color{0.0f, 0.0f, 0.0f, 0.82f});
  r.strokePolyline(box, 5, 1.0f, color);
  r.fillText(cx, cy, text, textSize, TextAlign::Center, color, kMapLabelFace);
}

void drawHoldLabel(Renderer& r, const Proj& proj, const MapLeg& leg,
                   float symSize, const Color& color) {
  if (!leg.hold.active) return;
  float x = 0.0f;
  float y = 0.0f;
  proj.toPx(leg.lat, leg.lon, x, y);
  const float textSize = std::max(10.0f, symSize * 0.55f);
  drawRouteIdentBox(r, x, y - symSize * 1.4f - kMapLabelLiftPx, "HOLD", textSize,
                    color);
}

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

bool hasProcedureArcLegs(const std::vector<MapLeg>& legs) {
  for (const MapLeg& leg : legs) {
    if (leg.procedureArc.active) return true;
  }
  return false;
}

double rfSweepDeg(double startDeg, double endDeg, HoldTurnDirection turn) {
  double sweep = shortestTurnDeltaDeg(startDeg, endDeg);
  if (turn == HoldTurnDirection::Right && sweep < 0.0) sweep += 360.0;
  if (turn == HoldTurnDirection::Left && sweep > 0.0) sweep -= 360.0;
  return sweep;
}

void appendProcedureArcPoints(std::vector<Point>& points, const MapLeg& from,
                              const MapLeg& to, const Proj& proj) {
  if (!to.procedureArc.active) {
    points.push_back(projectLeg(proj, to));
    return;
  }

  const double startDeg =
      navBearingDeg(to.procedureArc.centerLat, to.procedureArc.centerLon,
                    from.lat, from.lon);
  const double endDeg =
      navBearingDeg(to.procedureArc.centerLat, to.procedureArc.centerLon, to.lat,
                    to.lon);
  const double sweep = rfSweepDeg(startDeg, endDeg, to.procedureArc.turn);
  double radiusNm = to.procedureArc.radiusNm;
  if (radiusNm <= 0.0) {
    radiusNm =
        0.5 *
        (navDistanceNm(to.procedureArc.centerLat, to.procedureArc.centerLon,
                       from.lat, from.lon) +
         navDistanceNm(to.procedureArc.centerLat, to.procedureArc.centerLon,
                       to.lat, to.lon));
  }
  if (radiusNm <= 0.0 || std::fabs(sweep) < 0.1) {
    points.push_back(projectLeg(proj, to));
    return;
  }

  const int segments = std::clamp(
      static_cast<int>(std::ceil(std::fabs(sweep) / kRfArcSegmentDeg)),
      kRfArcMinSegments, kRfArcMaxSegments);
  for (int i = 1; i <= segments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segments);
    const double brg = startDeg + sweep * t;
    double lat = 0.0;
    double lon = 0.0;
    navOffsetPoint(to.procedureArc.centerLat, to.procedureArc.centerLon, brg,
                   radiusNm, lat, lon);
    if (i == segments) {
      lat = to.lat;
      lon = to.lon;
    }
    float x = 0.0f;
    float y = 0.0f;
    proj.toPx(lat, lon, x, y);
    points.push_back({x, y});
  }
}

SmoothedRoute buildProcedureArcAwareRoute(const std::vector<MapLeg>& legs,
                                          const Proj& proj) {
  SmoothedRoute out;
  if (legs.empty()) return out;
  out.points.push_back(projectLeg(proj, legs.front()));
  if (legs.size() == 1) return out;

  out.legStart.resize(legs.size() - 1);
  out.legEnd.resize(legs.size() - 1);
  for (std::size_t i = 0; i + 1 < legs.size(); ++i) {
    out.legStart[i] = out.points.size() - 1;
    appendProcedureArcPoints(out.points, legs[i], legs[i + 1], proj);
    out.legEnd[i] = out.points.size() - 1;
  }
  return out;
}

void drawRouteLegDirect(Renderer& r, const Proj& proj, const MapLeg& from,
                        const MapLeg& to, float width, const Color& color) {
  const Point seg[2] = {projectLeg(proj, from), projectLeg(proj, to)};
  r.strokePolyline(seg, 2, width, color);
}

void drawRouteLeg(Renderer& r, const SmoothedRoute& route, std::size_t leg,
                  const std::vector<MapLeg>& legs, const Proj& proj, float width,
                  const Color& color) {
  if (leg >= route.legStart.size() || leg + 1 >= legs.size()) return;
  const std::size_t start = route.legStart[leg];
  const std::size_t end = route.legEnd[leg];
  if (end > start && end < route.points.size()) {
    r.strokePolyline(route.points.data() + start,
                     static_cast<int>(end - start + 1), width, color);
    return;
  }
  drawRouteLegDirect(r, proj, legs[leg], legs[leg + 1], width, color);
}

// Replace sharp waypoint corners with fly-by quadratic arcs. Waypoint symbols
// stay at the published fix locations; only the course line is smoothed tangent
// through the turns. When `groundSpeedKts > 0` the tangent (lead) distance for
// each corner is the same fly-by lead the autopilot flies (turnLeadDistanceNm),
// so the drawn curve matches the actual path; otherwise a fixed display radius
// is used (procedure preview, which has no live speed).
SmoothedRoute buildSmoothedRoute(const std::vector<MapLeg>& legs,
                                 const Proj& proj, float groundSpeedKts = 0.0f) {
  if (hasProcedureArcLegs(legs)) return buildProcedureArcAwareRoute(legs, proj);

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

  const bool speedDriven = groundSpeedKts > 1.0f;
  // Match the autopilot's lead-distance floor (FmsNavigator) so the drawn turn
  // begins exactly where the aircraft does.
  const double gsKts = std::max(40.0, static_cast<double>(groundSpeedKts));
  const float fixedRadiusPx =
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

    // The Bézier tangent length is the corner lead distance: lead = R*tan(Δ/2),
    // which is exactly what turnLeadDistanceNm returns (capped at a 90° turn).
    // With live speed, draw the *actual* autopilot lead so the rendered curve
    // matches the flown path. The fixed display radius is only a fallback for
    // the speedless procedure preview; previously the active route max()'d with
    // it, forcing every turn to a >=2 NM radius far wider than the aircraft
    // actually turns.
    float rad = fixedRadiusPx;
    if (speedDriven) {
      rad = static_cast<float>(turnLeadDistanceNm(gsKts, turnDeg)) *
            proj.pixelsPerNm;
    }
    // Never let adjacent turns overlap on short legs.
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

    // Apex of the symmetric quadratic Bézier (t=0.5) is the point closest to
    // the fix; it marks where the inbound leg ends and the outbound leg begins.
    constexpr int kArcMidSegment = kRouteArcSegments / 2;
    std::size_t apexIdx = out.points.size() - 1;
    for (int s = 1; s <= kRouteArcSegments; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(kRouteArcSegments);
      const float u = 1.0f - t;
      out.points.push_back(
          {u * u * t1.x + 2.0f * u * t * cur.x + t * t * t2.x,
           u * u * t1.y + 2.0f * u * t * cur.y + t * t * t2.y});
      if (s == kArcMidSegment) {
        apexIdx = out.points.size() - 1;
      }
    }
    // Split the turn arc at the fix: the inbound leg (i-1) curves in and ends at
    // the apex (over the fix); the outbound leg (i) rolls out from the apex.
    // This keeps the magenta active leg from extending past its TO waypoint, and
    // every point stays in exactly one leg span so per-leg drawing (width /
    // magenta active leg) renders the curves continuously.
    out.legEnd[i - 1] = apexIdx;
    out.legStart[i] = apexIdx;
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
  if (map.directToActive) {
    const int dtoIdx = directToTargetIndexInPlan(map);
    if (dtoIdx >= 0) return dtoIdx;
    return procedureBlockStartIndex(map);
  }
  if (map.flightPlanRouteStartIndex > 0 &&
      map.flightPlanRouteStartIndex <
          static_cast<int>(map.flightPlan.size())) {
    return map.flightPlanRouteStartIndex;
  }
  return 0;
}

std::vector<MapLeg> legsForMapRoute(const MapData& map) {
  if (map.flightPlan.size() < 2) {
    return map.flightPlan;
  }

  const int sliceStart = mapRouteSliceStart(map);
  if (sliceStart > 0) {
    return std::vector<MapLeg>(map.flightPlan.begin() + sliceStart,
                               map.flightPlan.end());
  }

  if (!map.directToActive) {
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

// Direct-To whose target is the first fix shown on the map route (e.g.
// Direct-To AZOMY with UZAWO→GRAMS→… still ahead on the approach). The magenta
// Direct-To course flows straight into the fix, but the fix also has an
// outbound leg, so it must get the same fly-by turn curve as every other route
// fix. drawFlightPlan handles this by prepending the course origin; the plain
// straight drawDirectToCourse line is suppressed so the two do not overlap.
bool directToTargetIsRouteHead(const MapData& map) {
  if (!map.directToActive || map.directTo.id.empty()) return false;
  const std::vector<MapLeg> legs = legsForMapRoute(map);
  return legs.size() >= 2 && legs.front().id == map.directTo.id;
}

// Present-position course origin for the active Direct-To leg: the captured
// origin when present, otherwise live ownship.
bool directToCourseOrigin(const MapData& map, double& lat, double& lon) {
  if (map.directToOriginValid) {
    lat = map.directToOriginLat;
    lon = map.directToOriginLon;
    return true;
  }
  if (map.positionValid) {
    lat = map.ownshipLat;
    lon = map.ownshipLon;
    return true;
  }
  return false;
}

bool mapHoldRacetrackActive(const MapData& map, const FlightData& flight,
                            int planLegIndex, int activeTo) {
  if (planLegIndex != activeTo) return false;
  if (flight.fmaLegIsHold && !map.directToActive) return true;
  if (map.directToHold && map.directToActive) return true;
  return false;
}

}  // namespace

void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  const std::vector<MapLeg> routeLegs = legsForMapRoute(map);
  if (routeLegs.size() < 2) return;

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const int procOffset = mapRouteSliceStart(map);
  const float activeWidth = flightPlanRouteWidth(symSize);
  const float previewWidth = missedApproachRouteWidth(symSize);
  const int maptIdx = findMaptIndexInRoute(routeLegs);
  const bool missedActive = missedApproachSegmentActive(maptIdx, activeTo);

  // Direct-To into the head of the route: prepend the course origin so the
  // target fix is an interior vertex and earns a fly-by turn curve into its
  // outbound leg, instead of a sharp corner. The leading origin→fix segment is
  // the magenta Direct-To course; the remaining fix-to-fix legs stay white.
  double dtoLat = 0.0;
  double dtoLon = 0.0;
  if (directToTargetIsRouteHead(map) &&
      directToCourseOrigin(map, dtoLat, dtoLon)) {
    std::vector<MapLeg> originLegs;
    originLegs.reserve(routeLegs.size() + 1);
    MapLeg origin;
    origin.lat = dtoLat;
    origin.lon = dtoLon;
    originLegs.push_back(origin);
    originLegs.insert(originLegs.end(), routeLegs.begin(), routeLegs.end());

    const SmoothedRoute originRoute =
        buildSmoothedRoute(originLegs, proj, flight.groundSpeedKts);
    for (std::size_t leg = 0; leg + 1 < originLegs.size(); ++leg) {
      // routeLegs index for this segment (the prepended origin leg is index 0).
      const int routeLegIdx = static_cast<int>(leg) - 1;
      const bool preview =
          maptIdx >= 0 && routeLegIdx >= maptIdx && !missedActive;
      const float width = preview ? previewWidth : activeWidth;
      const Color& color =
          (leg == 0 && !flight.fmaLegIsHold) ? colors::kMagenta : colors::kWhite;
      drawRouteLeg(r, originRoute, leg, originLegs, proj, width, color);
    }

    const float holdWidth = missedActive ? activeWidth : previewWidth;
    for (std::size_t i = 0; i < routeLegs.size(); ++i) {
      const MapLeg& leg = routeLegs[i];
      if (!leg.hold.active) continue;
      const bool holdActive = mapHoldRacetrackActive(
          map, flight, static_cast<int>(i) + procOffset, activeTo);
      const Color& holdColor = holdActive ? colors::kMagenta : colors::kWhite;
      drawHoldRacetrack(r, leg, proj, holdWidth, holdColor,
                        flight.groundSpeedKts);
      drawHoldLabel(r, proj, leg, symSize, holdColor);
    }
    return;
  }

  // Draw fix-to-fix segments with fly-by turn smoothing driven by live ground
  // speed, so the course line follows the same curved path the autopilot flies
  // (the line cuts the corner at each fly-by fix rather than overflying it).
  const SmoothedRoute route =
      buildSmoothedRoute(routeLegs, proj, flight.groundSpeedKts);
  for (std::size_t leg = 0; leg + 1 < routeLegs.size(); ++leg) {
    const float width = (maptIdx >= 0 && static_cast<int>(leg) >= maptIdx &&
                         !missedActive)
                            ? previewWidth
                            : activeWidth;
    drawRouteLeg(r, route, leg, routeLegs, proj, width, colors::kWhite);
  }

  // In-plan Direct-To draws the magenta course from ownship to the target;
  // do not also highlight a plan leg (index math differs once the route is
  // sliced to the loaded procedure during approach Direct-To). While flying the
  // hold itself the inbound leg reverts to white and the racetrack carries the
  // magenta, so skip the active-leg highlight when in the hold.
  if (!map.directToActive && !flight.fmaLegIsHold && activeTo >= 0) {
    const int activeToInRoute = activeTo - procOffset;
    if (activeToInRoute >= 1 &&
        activeToInRoute < static_cast<int>(routeLegs.size())) {
      const std::size_t leg = static_cast<std::size_t>(activeToInRoute) - 1;
      drawRouteLeg(r, route, leg, routeLegs, proj, activeWidth,
                   colors::kMagenta);
    }
  }

  const float holdWidth = missedActive ? activeWidth : previewWidth;
  for (std::size_t i = 0; i < routeLegs.size(); ++i) {
    const MapLeg& leg = routeLegs[i];
    if (!leg.hold.active) continue;
    const bool holdActive =
        mapHoldRacetrackActive(map, flight,
                               static_cast<int>(i) + procOffset, activeTo);
    const Color& holdColor = holdActive ? colors::kMagenta : colors::kWhite;
    drawHoldRacetrack(r, leg, proj, holdWidth, holdColor,
                      flight.groundSpeedKts);
    drawHoldLabel(r, proj, leg, symSize, holdColor);
  }
}

void drawFlightPlanLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, const FlightData& flight,
                          float symSize, float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  const std::vector<MapLeg> routeLegs = legsForMapRoute(map);
  if (routeLegs.empty()) return;

  const int activeTo = activeFlightPlanToIndex(map, flight);
  const int procOffset = mapRouteSliceStart(map);
  const float textSize = labelSize * kMapIdentLabelScale;

  for (std::size_t i = 0; i < routeLegs.size(); ++i) {
    if (routeLegs[i].id.empty()) continue;
    const Point pt = projectLeg(proj, routeLegs[i]);
    const int planIndex = static_cast<int>(i) + procOffset;
    const Color c =
        (!map.directToActive && planIndex == activeTo) ? colors::kMagenta
                                                       : colors::kWhite;
    drawRouteIdentBox(r, pt.x, pt.y, routeLegs[i].id, textSize, c);
  }
}

void drawProcedurePreview(Renderer& r, const Proj& proj,
                          const MapViewConfig& config, float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;
  if (config.procedurePreview == nullptr ||
      config.procedurePreview->size() < 2) {
    return;
  }

  const std::vector<MapLeg>& legs = *config.procedurePreview;
  // Inbound approach legs use the full-weight white course; the missed-approach
  // segment (from the MAP onward, e.g. RW05->SERFS) draws as the same thin white
  // line a loaded approach shows before the missed approach is activated.
  const int maptIdx = findMaptIndexInRoute(legs);
  const float activeWidth = flightPlanRouteWidth(symSize);
  const float previewWidth = missedApproachRouteWidth(symSize);
  const SmoothedRoute preview = buildSmoothedRoute(legs, proj);
  for (std::size_t leg = 0; leg + 1 < legs.size(); ++leg) {
    const float width =
        (maptIdx >= 0 && static_cast<int>(leg) >= maptIdx) ? previewWidth
                                                           : activeWidth;
    drawRouteLeg(r, preview, leg, legs, proj, width, colors::kWhite);
  }

  for (const MapLeg& leg : legs) {
    if (leg.hold.active) {
      drawHoldRacetrack(r, leg, proj, previewWidth, colors::kWhite, 90.0f);
    }
  }
}

void drawProcedurePreviewLabels(Renderer& r, const Proj& proj,
                                const MapViewConfig& config, float symSize,
                                float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm ||
      config.procedurePreview == nullptr) {
    return;
  }
  // Boxed white idents, exactly like the live flight-plan waypoint labels, so
  // the previewed procedure fixes read the same as a loaded approach.
  const float textSize = labelSize * kMapIdentLabelScale;
  for (const MapLeg& leg : *config.procedurePreview) {
    if (leg.id.empty()) continue;
    const Point pt = projectLeg(proj, leg);
    drawRouteIdentBox(r, pt.x, pt.y, leg.id, textSize, colors::kWhite);
  }
  (void)symSize;
}

void drawDirectToCourse(Renderer& r, const MapData& map,
                        const FlightData& flight, const Proj& proj,
                        const MapViewConfig& config, float symSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;
  // drawFlightPlan renders the curved magenta course into a route-head fix.
  if (directToTargetIsRouteHead(map)) return;

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
  r.strokePolyline(dto, 2, flightPlanRouteWidth(symSize), colors::kMagenta);
}

void drawDirectToCourseLabel(Renderer& r, const MapData& map,
                             const FlightData& flight, const Proj& proj,
                             const MapViewConfig& config, float symSize,
                             float labelSize) {
  if (config.rangeNm > kContinentalChartRangeNm) return;

  MapLeg target;
  if (!activeDirectNavTarget(map, flight, target) || target.id.empty()) {
    return;
  }

  float tx = 0.0f, ty = 0.0f;
  proj.toPx(target.lat, target.lon, tx, ty);
  drawRouteIdentBox(r, tx, ty, target.id, labelSize * kMapIdentLabelScale,
                    colors::kMagenta);
}

}  // namespace avionics::mapview
