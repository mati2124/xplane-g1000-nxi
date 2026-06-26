#include "avionics/NavigationComputer.h"

#include <cmath>

#include "avionics/GpsLegCourse.h"
#include "avionics/MissedApproachGuidance.h"
#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

namespace avionics {

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
  return !obsMode && !nav.missedApproachSuspended() && !nav.inHold() &&
         !nav.directToActive() && data.fmaActiveLegIndex >= 0;
}

void syncNavigatorFlightPlan(FmsNavigator& nav, const MapData& map) {
  const std::vector<MapLeg>& plan = map.flightPlan;
  if (flightPlansEqual(nav.flightPlan(), plan)) return;
  const int prevIdx = nav.activeLegIndex();
  const bool preserveDirectTo = map.directToActive && !map.directTo.id.empty();
  nav.setFlightPlan(plan);
  if (preserveDirectTo) {
    nav.activateDirectTo(map.directTo, map.directToOriginLat,
                         map.directToOriginLon, map.directToOriginValid);
  } else if (prevIdx >= 0 && prevIdx < static_cast<int>(plan.size())) {
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

namespace {

// While completing a fly-by turn, hold the injected GPS cross-track within this
// deflection. X-Plane's autopilot tracks the override CDI we feed it; a needle
// pegged to full scale (2.5 dots) through a sharp turn makes it give up the
// intercept and revert NAV → ROLL, which is the mid-turn disengage pilots see.
// 1.5 dots keeps a firm correction without ever saturating.
constexpr float kTurnSmoothMaxCdiDots = 1.5f;
// Only smooth turns that change course enough to throw the outbound cross-track
// off; gentle fly-bys never approach full scale, so leave their tracking exact.
constexpr double kTurnSmoothMinDeltaDeg = 15.0;
// Margin past the lead point that still counts as "inside the turn" so the
// clamp covers the full arc until the aircraft intercepts the outbound leg.
constexpr double kTurnRegionMarginNm = avionics::kTurnSteeringMarginNm;

bool steerDirectToOutboundTurn(NavigationSolution& sol, const MapData& map,
                               double gsKts) {
  if (!map.directToActive || map.directTo.id.empty()) return false;
  const std::vector<MapLeg>& plan = map.flightPlan;
  const int dtoIdx = legIndexInPlan(plan, map.directTo);
  if (dtoIdx < 0 || dtoIdx + 1 >= static_cast<int>(plan.size())) {
    return false;
  }

  const MapLeg& target = plan[static_cast<std::size_t>(dtoIdx)];
  const MapLeg& nextLeg = plan[static_cast<std::size_t>(dtoIdx + 1)];
  double inboundDeg;
  if (map.directToOriginValid) {
    inboundDeg = navBearingDeg(map.directToOriginLat, map.directToOriginLon,
                               target.lat, target.lon);
  } else {
    inboundDeg = navBearingDeg(map.ownshipLat, map.ownshipLon, target.lat,
                              target.lon);
  }
  const double outboundDeg =
      navBearingDeg(target.lat, target.lon, nextLeg.lat, nextLeg.lon);
  const double turnDeltaDeg = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  if (std::fabs(turnDeltaDeg) < 1.0) return false;

  const double gs = std::max(40.0, static_cast<double>(gsKts));
  const double leadNm =
      turnLeadDistanceNm(gs, turnDeltaDeg, kDirectToFlyByMaxTurnDegCap);
  const double distToNm =
      navDistanceNm(map.ownshipLat, map.ownshipLon, target.lat, target.lon);
  if (distToNm > leadNm + kTurnRegionMarginNm) return false;

  sol.desiredTrackDeg = static_cast<float>(outboundDeg);
  sol.crossTrackNm = 0.0f;
  return true;
}

}  // namespace

NavigationSolution applyFlyByTurnCourse(NavigationSolution sol,
                                        const MapData& map,
                                        const FlightData& data, bool obsMode,
                                        CdiSource cdiSource, float nmPerDot) {
  if (!sol.active || obsMode || data.gpsSequencingSuspended ||
      cdiSource != CdiSource::Gps) {
    return sol;
  }
  if (!map.positionValid || map.flightPlan.size() < 2) return sol;

  const std::vector<MapLeg>& plan = map.flightPlan;
  const int toIdx = sol.activeLegIndex;

  // (A) Pre-sequence corner round: the turn-anticipation advisory reads "… now"
  // while the navigator is still on the inbound leg (the moderate fly-by whose
  // lead point sits inside the capture radius). Steer the outbound DTK with a
  // centered CDI so the autopilot starts the turn instead of the inbound
  // cross-track fighting it (e.g. KFMY CITAG→BUTLY→UZAWO needs a right turn but
  // the outbound XTK reads "fly left" until the fix is captured).
  //
  // Also applies during an on-plan Direct-To: the AP must track the direct
  // course until the advisory says "now", then begin the outbound turn smoothly
  // (regression: Direct-To AZOMY then left turn to UZAWO dropped NAV mid-turn).
  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, obsMode, cdiSource);
  if (ta.active && ta.message.find(" now") != std::string::npos &&
      toIdx >= 0 && toIdx + 1 < static_cast<int>(plan.size())) {
    const MapLeg& fromLeg = plan[static_cast<std::size_t>(toIdx)];
    const MapLeg& nextLeg = plan[static_cast<std::size_t>(toIdx + 1)];
    sol.desiredTrackDeg = static_cast<float>(
        navBearingDeg(fromLeg.lat, fromLeg.lon, nextLeg.lat, nextLeg.lon));
    sol.crossTrackNm = 0.0f;
    return sol;
  }

  // Fly-by post-sequence smoothing applies only after Direct-To capture; during
  // Direct-To the AP must track the direct course to the fix until the fly-by
  // lead point, then steer the outbound leg (see steerDirectToOutboundTurn).
  if (sol.directTo) {
    steerDirectToOutboundTurn(sol, map, data.groundSpeedKts);
    return sol;
  }

  // (B) Post-sequence turn completion. The navigator flips the leg at the same
  // lead distance the advisory uses, so on the standard fly-by the aircraft is
  // already on the outbound leg by the time it reaches the fly-by point and (A)
  // never fires. The raw outbound cross-track then jumps to ~lead·sin(turnΔ) the
  // instant the leg sequences -- on a sharp turn that pegs the injected CDI full
  // scale and X-Plane drops NAV to ROLL. While the aircraft is still inside the
  // turn near the fix, clamp the commanded cross-track so the needle never
  // saturates; it relaxes to exact tracking as the real offset shrinks below the
  // clamp and the aircraft intercepts the outbound leg.
  if (toIdx < 1) return sol;
  const MapLeg& turnFix = plan[static_cast<std::size_t>(toIdx - 1)];
  const MapLeg& outLeg = plan[static_cast<std::size_t>(toIdx)];
  double inboundDeg;
  if (toIdx >= 2) {
    const MapLeg& priorFix = plan[static_cast<std::size_t>(toIdx - 2)];
    inboundDeg =
        navBearingDeg(priorFix.lat, priorFix.lon, turnFix.lat, turnFix.lon);
  } else {
    inboundDeg = navBearingDeg(map.ownshipLat, map.ownshipLon, turnFix.lat,
                               turnFix.lon);
  }
  const double outboundDeg =
      navBearingDeg(turnFix.lat, turnFix.lon, outLeg.lat, outLeg.lon);
  const double turnDeltaDeg = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  if (std::fabs(turnDeltaDeg) < kTurnSmoothMinDeltaDeg) return sol;

  const double gsKts = std::max(40.0, static_cast<double>(data.groundSpeedKts));
  const double leadNm = turnLeadDistanceNm(gsKts, turnDeltaDeg);
  const double distFixNm =
      navDistanceNm(map.ownshipLat, map.ownshipLon, turnFix.lat, turnFix.lon);
  if (distFixNm > leadNm + kTurnRegionMarginNm) return sol;

  const float scale = std::max(0.05f, nmPerDot > 0.01f ? nmPerDot : 0.5f);
  const float maxXtkNm = kTurnSmoothMaxCdiDots * scale;
  if (sol.crossTrackNm > maxXtkNm) {
    sol.crossTrackNm = maxXtkNm;
  } else if (sol.crossTrackNm < -maxXtkNm) {
    sol.crossTrackNm = -maxXtkNm;
  }
  return sol;
}

void clearNavigationFields(FlightData& data) {
  data.fmaFromWpt.clear();
  data.fmaToWpt.clear();
  data.fmaLegIsHold = false;
  data.fmaActiveLegIndex = -1;
  data.fmaLegDistanceNm = 0.0f;
  data.fmaLegBearingDeg = 0.0f;
  data.navStatusAnnunciation.clear();
  data.navStatusAnnunciationFlash = false;
  data.gpsCrossTrackNm = 0.0f;
  data.gpsSequencingSuspended = false;
  data.missedApproachActive = false;
}

void runNavigationFrame(FmsNavigator& navigator, const MapData& map,
                        FlightData& data, bool obsMode, CdiSource cdiSource,
                        float nmPerDot, const NavigationCallbacks& callbacks) {
  if (!map.positionValid || !data.dataLinkValid) {
    data.navStatusAnnunciation.clear();
    data.navStatusAnnunciationFlash = false;
    return;
  }

  const bool hadDirectTo = navigator.directToActive();
  syncNavigatorFlightPlan(navigator, map);
  syncNavigatorDirectTo(navigator, map);
  navigator.setObsMode(obsMode);

  if (map.flightPlan.empty() && !navigator.directToActive()) {
    clearNavigationFields(data);
    return;
  }

  NavigationSolution sol = navigator.update(map.ownshipLat, map.ownshipLon,
                                          data.groundSpeedKts, data.altitudeFt);
  if (!sol.active) {
    clearNavigationFields(data);
    return;
  }

  applyNavigationSolution(data, sol, nmPerDot);

  if (cdiSource == CdiSource::Gps && !obsMode) {
    sol = applyFlyByTurnCourse(sol, map, data, obsMode, cdiSource, nmPerDot);
    applyNavigationSolution(data, sol, nmPerDot);
  }

  if (hadDirectTo && !navigator.directToActive() &&
      callbacks.onDirectToCaptured) {
    callbacks.onDirectToCaptured(navigator.activeLegIndex());
  }

  applyTurnAnticipation(data, map, obsMode, cdiSource);

  const MissedClimbProfile missedClimb =
      computeMissedClimbProfile(map, data);
  applyMissedClimbProfile(data, missedClimb);
}

}  // namespace avionics
