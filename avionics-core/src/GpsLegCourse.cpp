#include "avionics/GpsLegCourse.h"

#include <algorithm>
#include <cmath>

#include "avionics/FplRouteEdit.h"
#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLegMatchDistanceNm = 0.75;
constexpr double kWaypointCaptureNm = 0.4;
constexpr double kWaypointPassedNm = 0.25;

double crossTrackNm(double lat, double lon, double fromLat, double fromLon,
                    double toLat, double toLon) {
  const double legBrg = navBearingDeg(fromLat, fromLon, toLat, toLon);
  const double brgToFrom = navBearingDeg(lat, lon, fromLat, fromLon);
  const double distToFrom = navDistanceNm(lat, lon, fromLat, fromLon);
  const double angleRad = (brgToFrom - legBrg) * kPi / 180.0;
  return distToFrom * std::sin(angleRad);
}

GpsLegNavigation computePureDirectToNav(const MapData& map,
                                          const FlightData& data) {
  GpsLegNavigation nav;
  nav.active = true;

  if (map.directTo.lat == 0.0 && map.directTo.lon == 0.0) {
    nav.courseDeg = data.fmaLegBearingDeg;
    return nav;
  }

  if (map.directToOriginValid) {
    nav.courseDeg = static_cast<float>(navBearingDeg(
        map.directToOriginLat, map.directToOriginLon, map.directTo.lat,
        map.directTo.lon));
    nav.crossTrackNm = static_cast<float>(crossTrackNm(
        map.ownshipLat, map.ownshipLon, map.directToOriginLat,
        map.directToOriginLon, map.directTo.lat, map.directTo.lon));
    return nav;
  }

  nav.courseDeg = data.fmaLegBearingDeg;
  return nav;
}

}  // namespace

bool flightPlanIdentsEqual(const std::string& a, const std::string& b) {
  if (a == b) return true;
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const char ca = static_cast<char>(std::tolower(
        static_cast<unsigned char>(a[i])));
    const char cb = static_cast<char>(std::tolower(
        static_cast<unsigned char>(b[i])));
    if (ca != cb) return false;
  }
  return true;
}

int legIndexInPlan(const std::vector<MapLeg>& plan, const std::string& id) {
  if (id.empty()) return -1;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (flightPlanIdentsEqual(plan[i].id, id)) return static_cast<int>(i);
  }
  return -1;
}

int resolveActiveLegToIndex(const std::vector<MapLeg>& plan,
                            const std::string& fromWpt,
                            const std::string& toWpt) {
  if (toWpt.empty()) return -1;
  if (!fromWpt.empty()) {
    for (std::size_t i = 1; i < plan.size(); ++i) {
      if (flightPlanIdentsEqual(plan[i - 1].id, fromWpt) &&
          flightPlanIdentsEqual(plan[i].id, toWpt)) {
        return static_cast<int>(i);
      }
    }
  }
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (flightPlanIdentsEqual(plan[i].id, toWpt)) return static_cast<int>(i);
  }
  return -1;
}

int advanceLegIfWaypointCaptured(const std::vector<MapLeg>& plan,
                                 const MapData& map, int legIdx) {
  if (legIdx < 0 || legIdx + 1 >= static_cast<int>(plan.size()) ||
      !map.positionValid) {
    return legIdx;
  }
  const MapLeg& wpt = plan[static_cast<std::size_t>(legIdx)];
  const double distNm =
      navDistanceNm(map.ownshipLat, map.ownshipLon, wpt.lat, wpt.lon);
  if (distNm > kWaypointCaptureNm) return legIdx;

  const MapLeg& next = plan[static_cast<std::size_t>(legIdx + 1)];
  const double distNextNm =
      navDistanceNm(map.ownshipLat, map.ownshipLon, next.lat, next.lon);
  if (distNextNm <= distNm + 0.15) return legIdx + 1;
  if (distNm < kWaypointPassedNm) return legIdx + 1;
  return legIdx;
}

int resolveNavLegToIndex(const std::vector<MapLeg>& plan, const FlightData& data,
                         const MapData& map) {
  if (plan.empty() || !map.positionValid) return -1;

  int nearestIdx = -1;
  double nearestDistNm = 1e9;
  for (int i = 0; i < static_cast<int>(plan.size()); ++i) {
    const MapLeg& leg = plan[static_cast<std::size_t>(i)];
    const double dist =
        navDistanceNm(map.ownshipLat, map.ownshipLon, leg.lat, leg.lon);
    if (dist < nearestDistNm) {
      nearestDistNm = dist;
      nearestIdx = i;
    }
  }

  int toIdx = resolveActiveLegToIndex(plan, data.fmaFromWpt, data.fmaToWpt);
  if (toIdx >= 0) {
    toIdx = advanceLegIfWaypointCaptured(plan, map, toIdx);
    if (nearestIdx > toIdx && nearestIdx >= 0 &&
        nearestDistNm < kLegMatchDistanceNm) {
      const double distActive =
          navDistanceNm(map.ownshipLat, map.ownshipLon,
                        plan[static_cast<std::size_t>(toIdx)].lat,
                        plan[static_cast<std::size_t>(toIdx)].lon);
      if (nearestDistNm + 0.15 < distActive) toIdx = nearestIdx;
    }
    return toIdx;
  }

  if (nearestIdx >= 0 && nearestDistNm < kLegMatchDistanceNm) {
    return advanceLegIfWaypointCaptured(plan, map, nearestIdx);
  }

  if (data.fmaLegDistanceNm <= 0.05f) return -1;

  // Direct-To can leave fmaToWpt on the airport while the sim sequences an
  // approach fix; match the fix whose DME equals the live GPS distance.
  int bestIdx = -1;
  double bestDiff = 1e9;
  for (int i = 0; i < static_cast<int>(plan.size()); ++i) {
    const MapLeg& leg = plan[static_cast<std::size_t>(i)];
    const double dist =
        navDistanceNm(map.ownshipLat, map.ownshipLon, leg.lat, leg.lon);
    const double diff =
        std::fabs(dist - static_cast<double>(data.fmaLegDistanceNm));
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIdx = i;
    }
  }
  if (bestIdx >= 0 && bestDiff < kLegMatchDistanceNm) {
    return advanceLegIfWaypointCaptured(plan, map, bestIdx);
  }
  return -1;
}

GpsLegNavigation computeGpsLegNavigation(const MapData& map,
                                         const FlightData& data, bool obsMode,
                                         CdiSource cdiSource) {
  GpsLegNavigation nav;
  if (obsMode) return nav;
  if (cdiSource != CdiSource::Gps) return nav;
  if (!map.positionValid || !data.dataLinkValid) return nav;
  if (data.fmaToWpt.empty()) return nav;

  const std::vector<MapLeg>& plan = map.flightPlan;
  const int toIdx = resolveNavLegToIndex(plan, data, map);

  if (toIdx < 0) {
    if (navDirectToActive(data) || map.directToActive) {
      return computePureDirectToNav(map, data);
    }
    return nav;
  }

  if (plan.size() < 2) return nav;

  int courseFromIdx = toIdx - 1;
  int courseToIdx = toIdx;
  int xtkFromIdx = toIdx - 1;
  int xtkToIdx = toIdx;

  const TurnAnticipation ta = computeTurnAnticipation(map, data, obsMode);
  if (ta.active && toIdx + 1 < static_cast<int>(plan.size()) &&
      ta.message.find(" now") != std::string::npos) {
    courseFromIdx = toIdx;
    courseToIdx = toIdx + 1;
  }

  if (courseFromIdx < 0) {
    if (navDirectToActive(data) || map.directToActive) {
      return computePureDirectToNav(map, data);
    }
    nav.active = true;
    nav.courseDeg = data.fmaLegBearingDeg;
    return nav;
  }

  const MapLeg& courseFrom = plan[static_cast<std::size_t>(courseFromIdx)];
  const MapLeg& courseTo = plan[static_cast<std::size_t>(courseToIdx)];
  nav.active = true;
  nav.courseDeg = static_cast<float>(navBearingDeg(
      courseFrom.lat, courseFrom.lon, courseTo.lat, courseTo.lon));

  if (xtkFromIdx >= 0) {
    const MapLeg& xtkFrom = plan[static_cast<std::size_t>(xtkFromIdx)];
    const MapLeg& xtkTo = plan[static_cast<std::size_t>(xtkToIdx)];
    nav.crossTrackNm = static_cast<float>(crossTrackNm(
        map.ownshipLat, map.ownshipLon, xtkFrom.lat, xtkFrom.lon, xtkTo.lat,
        xtkTo.lon));
  }
  return nav;
}

void applyGpsLegNavigation(FlightData& data, const MapData& map, bool obsMode,
                           CdiSource cdiSource, float nmPerDot) {
  const GpsLegNavigation nav =
      computeGpsLegNavigation(map, data, obsMode, cdiSource);
  if (!nav.active) return;

  data.courseDeg = nav.courseDeg;
  if (map.positionValid && nmPerDot > 0.01f) {
    const float scale = std::max(0.05f, nmPerDot);
    data.cdiDeviationDots = std::max(
        -2.5f, std::min(2.5f, nav.crossTrackNm / scale));
    data.cdiToFlag = true;
    data.navSignalValid = true;
  }
}

}  // namespace avionics
