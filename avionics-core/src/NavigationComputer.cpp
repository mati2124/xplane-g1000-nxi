#include "avionics/NavigationComputer.h"

#include <cmath>

#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;

double crossTrackNm(double lat, double lon, double fromLat, double fromLon,
                    double toLat, double toLon) {
  const double legBrg = navBearingDeg(fromLat, fromLon, toLat, toLon);
  const double brgToFrom = navBearingDeg(lat, lon, fromLat, fromLon);
  const double distToFrom = navDistanceNm(lat, lon, fromLat, fromLon);
  const double angleRad = (brgToFrom - legBrg) * kPi / 180.0;
  return distToFrom * std::sin(angleRad);
}

}  // namespace

bool flightPlansEqual(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!flightPlanIdentsEqual(a[i].id, b[i].id)) return false;
    if (std::fabs(a[i].lat - b[i].lat) > 1e-6 ||
        std::fabs(a[i].lon - b[i].lon) > 1e-6) {
      return false;
    }
  }
  return true;
}

bool shouldSyncActiveLegToSimulator(const FmsNavigator& nav,
                                    const FlightData& data, bool obsMode) {
  return !obsMode && !nav.directToActive() && data.fmaActiveLegIndex >= 0;
}

void syncNavigatorFlightPlan(FmsNavigator& nav, const std::vector<MapLeg>& plan) {
  if (flightPlansEqual(nav.flightPlan(), plan)) return;
  const int prevIdx = nav.activeLegIndex();
  nav.setFlightPlan(plan);
  if (prevIdx >= 0 && prevIdx < static_cast<int>(plan.size())) {
    nav.setActiveLegIndex(prevIdx);
  }
}

void syncNavigatorDirectTo(FmsNavigator& nav, const MapData& map) {
  if (map.directToActive && !map.directTo.id.empty()) {
    const MapLeg& prev = nav.directToTarget();
    const bool targetChanged =
        !nav.directToActive() ||
        !flightPlanIdentsEqual(prev.id, map.directTo.id) ||
        ((prev.lat != 0.0 || prev.lon != 0.0) &&
         (map.directTo.lat != 0.0 || map.directTo.lon != 0.0) &&
         navDistanceNm(prev.lat, prev.lon, map.directTo.lat,
                       map.directTo.lon) > kFlightPlanLegMatchNm);
    const bool originChanged =
        map.directToOriginValid != nav.directToOriginValid() ||
        (map.directToOriginValid &&
         navDistanceNm(map.directToOriginLat, map.directToOriginLon,
                       nav.directToOriginLat(), nav.directToOriginLon()) >
             0.001);
    if (targetChanged || originChanged) {
      nav.activateDirectTo(map.directTo, map.directToOriginLat,
                           map.directToOriginLon, map.directToOriginValid);
    }
    return;
  }
  if (nav.directToActive()) {
    nav.clearDirectTo();
  }
}

NavigationSolution applyFlyByTurnCourse(NavigationSolution sol,
                                        const MapData& map,
                                        const FlightData& data, bool obsMode,
                                        CdiSource cdiSource) {
  if (!sol.active || sol.directTo || obsMode || cdiSource != CdiSource::Gps) {
    return sol;
  }
  if (!map.positionValid || map.flightPlan.size() < 2) return sol;

  const TurnAnticipation ta = computeTurnAnticipation(map, data, obsMode);
  if (!ta.active || ta.message.find(" now") == std::string::npos) return sol;

  const int toIdx = sol.activeLegIndex;
  if (toIdx < 0 || toIdx + 1 >= static_cast<int>(map.flightPlan.size())) {
    return sol;
  }

  const MapLeg& fromLeg = map.flightPlan[static_cast<std::size_t>(toIdx)];
  const MapLeg& toLeg = map.flightPlan[static_cast<std::size_t>(toIdx + 1)];
  sol.desiredTrackDeg = static_cast<float>(navBearingDeg(
      fromLeg.lat, fromLeg.lon, toLeg.lat, toLeg.lon));
  sol.crossTrackNm = static_cast<float>(crossTrackNm(
      map.ownshipLat, map.ownshipLon, fromLeg.lat, fromLeg.lon, toLeg.lat,
      toLeg.lon));
  return sol;
}

void clearNavigationFields(FlightData& data) {
  data.fmaFromWpt.clear();
  data.fmaToWpt.clear();
  data.fmaActiveLegIndex = -1;
  data.fmaLegDistanceNm = 0.0f;
  data.fmaLegBearingDeg = 0.0f;
}

void runNavigationFrame(FmsNavigator& navigator, const MapData& map,
                        FlightData& data, bool obsMode, CdiSource cdiSource,
                        float nmPerDot, const NavigationCallbacks& callbacks) {
  if (!map.positionValid || !data.dataLinkValid) return;

  const bool hadDirectTo = navigator.directToActive();
  syncNavigatorFlightPlan(navigator, map.flightPlan);
  syncNavigatorDirectTo(navigator, map);
  navigator.setObsMode(obsMode);

  if (map.flightPlan.empty() && !navigator.directToActive()) {
    clearNavigationFields(data);
    return;
  }

  NavigationSolution sol = navigator.update(map.ownshipLat, map.ownshipLon,
                                          data.groundSpeedKts);
  if (!sol.active) {
    clearNavigationFields(data);
    return;
  }

  applyNavigationSolution(data, sol, nmPerDot);

  if (cdiSource == CdiSource::Gps && !obsMode) {
    sol = applyFlyByTurnCourse(sol, map, data, obsMode, cdiSource);
    applyNavigationSolution(data, sol, nmPerDot);
  }

  if (hadDirectTo && !navigator.directToActive() &&
      callbacks.onDirectToCaptured) {
    callbacks.onDirectToCaptured(navigator.activeLegIndex());
  }
}

}  // namespace avionics
