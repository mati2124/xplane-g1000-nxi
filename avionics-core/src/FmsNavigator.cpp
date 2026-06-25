#include "avionics/FmsNavigator.h"

#include <algorithm>
#include <cmath>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/HoldNavigation.h"
#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWaypointCaptureNm = 0.4;

bool isCourseToAltLeg(const MapLeg& leg) {
  return leg.pathTerminator == "CA" || leg.pathTerminator == "FM" ||
         leg.pathTerminator == "VM" || leg.pathTerminator == "VI";
}

bool altitudeConstraintSatisfied(int altFt, AltConstraintType kind,
                                 float altitudeFt) {
  if (altFt <= 0) return false;
  switch (kind) {
    case AltConstraintType::AtOrAbove:
      return altitudeFt >= static_cast<float>(altFt) - 50.0f;
    case AltConstraintType::At:
      return std::fabs(altitudeFt - static_cast<float>(altFt)) < 75.0f;
    case AltConstraintType::AtOrBelow:
      return altitudeFt <= static_cast<float>(altFt) + 50.0f;
    default:
      return false;
  }
}

bool altitudeConstraintSatisfied(const MapLeg& leg, float altitudeFt) {
  return altitudeConstraintSatisfied(leg.altitudeConstraintFt,
                                       leg.altitudeConstraint, altitudeFt);
}

bool flyingMissedInitialManeuver(const MapLeg& leg, bool missedActive) {
  return missedActive && leg.missedInitial.active;
}

double crossTrackNm(double lat, double lon, double fromLat, double fromLon,
                    double toLat, double toLon) {
  const double legBrg = navBearingDeg(fromLat, fromLon, toLat, toLon);
  const double brgToFrom = navBearingDeg(lat, lon, fromLat, fromLon);
  const double distToFrom = navDistanceNm(lat, lon, fromLat, fromLon);
  const double angleRad = (brgToFrom - legBrg) * kPi / 180.0;
  return distToFrom * std::sin(angleRad);
}

}  // namespace

void FmsNavigator::setFlightPlan(std::vector<MapLeg> plan) {
  plan_ = std::move(plan);
  activeLegIndex_ = plan_.empty() ? -1 : 0;
  refreshMaptIndex();
  missedSuspended_ = false;
  missedActive_ = false;
  inHold_ = false;
  holdLegIndex_ = -1;
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
}

void FmsNavigator::refreshMaptIndex() {
  maptLegIndex_ = findMaptLegIndex(plan_);
}

void FmsNavigator::syncMissedApproachStateFromLeg() {
  if (maptLegIndex_ < 0) {
    missedSuspended_ = false;
    missedActive_ = false;
    return;
  }
  if (activeLegIndex_ > maptLegIndex_) {
    missedActive_ = true;
    missedSuspended_ = false;
  } else if (activeLegIndex_ == maptLegIndex_ && missedActive_) {
    missedSuspended_ = false;
  } else if (activeLegIndex_ < maptLegIndex_) {
    missedActive_ = false;
    missedSuspended_ = false;
  }
}

void FmsNavigator::setActiveLegIndex(int toLegIndex) {
  if (toLegIndex < 0 || toLegIndex >= static_cast<int>(plan_.size())) return;
  activeLegIndex_ = toLegIndex;
  if (toLegIndex != holdLegIndex_) {
    inHold_ = false;
    holdLegIndex_ = -1;
  }
  syncMissedApproachStateFromLeg();
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
}

bool FmsNavigator::activateMissedApproach() {
  if (maptLegIndex_ < 0) return false;
  const int missedStart = maptLegIndex_ + 1;
  if (missedStart >= static_cast<int>(plan_.size())) return false;
  missedSuspended_ = false;
  missedActive_ = true;
  inHold_ = false;
  holdLegIndex_ = -1;
  // Stay on the MAPt while flying a published CA/FM initial climb; the next fix
  // (IBITE) is sequenced once the altitude constraint is satisfied.
  const MapLeg& mapt = plan_[static_cast<std::size_t>(maptLegIndex_)];
  activeLegIndex_ =
      mapt.missedInitial.active ? maptLegIndex_ : missedStart;
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
  return true;
}

bool FmsNavigator::resumeFromAutoSuspend() {
  if (inHold_) {
    exitHold();
    return true;
  }
  if (missedSuspended_ && !missedActive_) {
    return activateMissedApproach();
  }
  return false;
}

void FmsNavigator::exitHold() {
  inHold_ = false;
  holdLegIndex_ = -1;
}

void FmsNavigator::tryEnterHold(double lat, double lon) {
  if (inHold_ || obsMode_ || activeLegIndex_ < 0) return;
  const MapLeg& leg = plan_[static_cast<std::size_t>(activeLegIndex_)];
  if (!leg.hold.active) return;
  if (!captureWaypoint(lat, lon, leg)) return;
  inHold_ = true;
  holdLegIndex_ = activeLegIndex_;
  const double distToFix = navDistanceNm(lat, lon, leg.lat, leg.lon);
  const float approachTrack =
      distToFix < 0.05
          ? leg.hold.inboundCourseDeg
          : static_cast<float>(navBearingDeg(lat, lon, leg.lat, leg.lon));
  const HoldEntryType entry =
      classifyHoldEntry(approachTrack, leg.hold.inboundCourseDeg);
  holdPhase_ = initialHoldPhase(entry);
}

void FmsNavigator::activateDirectTo(MapLeg target, double originLat,
                                    double originLon, bool originValid) {
  directToActive_ = true;
  directTo_ = std::move(target);
  directToOriginLat_ = originLat;
  directToOriginLon_ = originLon;
  directToOriginValid_ = originValid;
  const int dtoIdx = legIndexInPlan(plan_, directTo_);
  if (dtoIdx >= 0) activeLegIndex_ = dtoIdx;
}

void FmsNavigator::clearDirectTo() {
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
}

bool FmsNavigator::captureWaypoint(double lat, double lon,
                                   const MapLeg& wpt) const {
  return navDistanceNm(lat, lon, wpt.lat, wpt.lon) <= kWaypointCaptureNm;
}

bool FmsNavigator::shouldSequenceLeg(double lat, double lon,
                                     float groundSpeedKts, int legIdx) const {
  if (legIdx < 0 || legIdx + 1 >= static_cast<int>(plan_.size())) {
    return false;
  }

  const MapLeg& toWpt = plan_[static_cast<std::size_t>(legIdx)];
  if (toWpt.hold.active) return false;
  const MapLeg& nextWpt = plan_[static_cast<std::size_t>(legIdx + 1)];
  const double distToNm = navDistanceNm(lat, lon, toWpt.lat, toWpt.lon);

  double inboundDeg;
  if (legIdx > 0) {
    const MapLeg& fromWpt = plan_[static_cast<std::size_t>(legIdx - 1)];
    inboundDeg =
        navBearingDeg(fromWpt.lat, fromWpt.lon, toWpt.lat, toWpt.lon);
  } else {
    inboundDeg = navBearingDeg(lat, lon, toWpt.lat, toWpt.lon);
  }
  const double outboundDeg =
      navBearingDeg(toWpt.lat, toWpt.lon, nextWpt.lat, nextWpt.lon);
  const double turnDeltaDeg = shortestTurnDeltaDeg(inboundDeg, outboundDeg);

  const double gsKts = std::max(40.0, static_cast<double>(groundSpeedKts));
  const double absTurnDeg = std::fabs(turnDeltaDeg);
  const double leadNm = legIdx > 0 && absTurnDeg >= 1.0
                            ? turnLeadDistanceNm(gsKts, turnDeltaDeg)
                            : 0.0;

  const double turnPointNm = leadNm > 0.0 ? leadNm : kWaypointCaptureNm;
  if (distToNm <= turnPointNm) return true;

  const double bearingFixToAcDeg = navBearingDeg(toWpt.lat, toWpt.lon, lat, lon);
  const double alongInbound = std::cos(
      shortestTurnDeltaDeg(inboundDeg, bearingFixToAcDeg) * kPi / 180.0);
  const double passedRadiusNm = std::max(kWaypointCaptureNm, leadNm) + 0.75;
  if (alongInbound > 0.0 && distToNm <= passedRadiusNm) return true;

  return false;
}

void FmsNavigator::sequenceActiveLeg(double lat, double lon,
                                     float groundSpeedKts, float altitudeFt) {
  if (obsMode_ || missedSuspended_ || inHold_ || plan_.empty() ||
      activeLegIndex_ < 0) {
    return;
  }

  const MapLeg& activeLeg = plan_[static_cast<std::size_t>(activeLegIndex_)];
  if (flyingMissedInitialManeuver(activeLeg, missedActive_)) {
    const auto& mi = activeLeg.missedInitial;
    if (altitudeConstraintSatisfied(mi.altitudeFt, mi.altitudeConstraint,
                                    altitudeFt) &&
        activeLegIndex_ + 1 < static_cast<int>(plan_.size())) {
      ++activeLegIndex_;
    }
    return;
  }

  if (isCourseToAltLeg(activeLeg)) {
    if (altitudeConstraintSatisfied(activeLeg, altitudeFt) &&
        activeLegIndex_ + 1 < static_cast<int>(plan_.size())) {
      ++activeLegIndex_;
    }
    return;
  }

  if (!shouldSequenceLeg(lat, lon, groundSpeedKts, activeLegIndex_)) return;

  if (activeLegIndex_ + 1 >= static_cast<int>(plan_.size())) return;

  if (maptLegIndex_ >= 0 && activeLegIndex_ == maptLegIndex_ &&
      !missedActive_) {
    missedSuspended_ = true;
    return;
  }

  ++activeLegIndex_;
}

NavigationSolution FmsNavigator::computeDirectToSolution(double lat,
                                                         double lon) const {
  NavigationSolution sol;
  sol.active = true;
  sol.directTo = true;
  sol.toWpt = directTo_.id;

  sol.distanceToWaypointNm = static_cast<float>(
      navDistanceNm(lat, lon, directTo_.lat, directTo_.lon));
  sol.bearingToWaypointDeg = static_cast<float>(
      navBearingDeg(lat, lon, directTo_.lat, directTo_.lon));

  if (directToOriginValid_) {
    const double originToTargetNm = navDistanceNm(
        directToOriginLat_, directToOriginLon_, directTo_.lat, directTo_.lon);
    if (originToTargetNm > 0.01) {
      sol.desiredTrackDeg = static_cast<float>(navBearingDeg(
          directToOriginLat_, directToOriginLon_, directTo_.lat, directTo_.lon));
      sol.crossTrackNm = static_cast<float>(crossTrackNm(
          lat, lon, directToOriginLat_, directToOriginLon_, directTo_.lat,
          directTo_.lon));
    } else {
      sol.desiredTrackDeg = sol.bearingToWaypointDeg;
      sol.crossTrackNm = 0.0f;
    }
  } else {
    sol.desiredTrackDeg = sol.bearingToWaypointDeg;
    sol.crossTrackNm = 0.0f;
  }

  const int dtoIdx = legIndexInPlan(plan_, directTo_);
  if (dtoIdx >= 0) sol.activeLegIndex = dtoIdx;
  return sol;
}

NavigationSolution FmsNavigator::computeLegSolution(double lat, double lon,
                                                    int toIdx) const {
  NavigationSolution sol;
  if (toIdx < 0 || toIdx >= static_cast<int>(plan_.size())) return sol;

  const MapLeg& toLeg = plan_[static_cast<std::size_t>(toIdx)];
  sol.active = true;
  sol.directTo = false;
  sol.toWpt = toLeg.id;
  sol.activeLegIndex = toIdx;

  sol.distanceToWaypointNm = static_cast<float>(
      navDistanceNm(lat, lon, toLeg.lat, toLeg.lon));
  sol.bearingToWaypointDeg = static_cast<float>(
      navBearingDeg(lat, lon, toLeg.lat, toLeg.lon));

  if (toIdx > 0) {
    const MapLeg& fromLeg = plan_[static_cast<std::size_t>(toIdx - 1)];
    sol.fromWpt = fromLeg.id;
    sol.desiredTrackDeg = static_cast<float>(navBearingDeg(
        fromLeg.lat, fromLeg.lon, toLeg.lat, toLeg.lon));
    sol.crossTrackNm = static_cast<float>(crossTrackNm(
        lat, lon, fromLeg.lat, fromLeg.lon, toLeg.lat, toLeg.lon));
  } else {
    sol.desiredTrackDeg = sol.bearingToWaypointDeg;
    sol.crossTrackNm = 0.0f;
  }

  if (flyingMissedInitialManeuver(toLeg, missedActive_) &&
      toLeg.missedInitial.courseDeg > 0.0f) {
    sol.desiredTrackDeg = toLeg.missedInitial.courseDeg;
    double farLat = 0.0;
    double farLon = 0.0;
    navOffsetPoint(toLeg.lat, toLeg.lon, toLeg.missedInitial.courseDeg, 10.0,
                   farLat, farLon);
    sol.crossTrackNm = static_cast<float>(
        crossTrackNm(lat, lon, toLeg.lat, toLeg.lon, farLat, farLon));
    if (toIdx + 1 < static_cast<int>(plan_.size())) {
      const MapLeg& nextLeg = plan_[static_cast<std::size_t>(toIdx + 1)];
      sol.distanceToWaypointNm = static_cast<float>(
          navDistanceNm(lat, lon, nextLeg.lat, nextLeg.lon));
      sol.bearingToWaypointDeg = static_cast<float>(
          navBearingDeg(lat, lon, nextLeg.lat, nextLeg.lon));
      sol.toWpt = nextLeg.id;
    }
  } else if (isCourseToAltLeg(toLeg) && toLeg.legCourseDeg > 0.0f) {
    sol.desiredTrackDeg = toLeg.legCourseDeg;
    double farLat = 0.0;
    double farLon = 0.0;
    navOffsetPoint(toLeg.lat, toLeg.lon, toLeg.legCourseDeg, 10.0, farLat,
                   farLon);
    sol.crossTrackNm = static_cast<float>(
        crossTrackNm(lat, lon, toLeg.lat, toLeg.lon, farLat, farLon));
    if (toIdx + 1 < static_cast<int>(plan_.size())) {
      const MapLeg& nextLeg = plan_[static_cast<std::size_t>(toIdx + 1)];
      sol.distanceToWaypointNm = static_cast<float>(
          navDistanceNm(lat, lon, nextLeg.lat, nextLeg.lon));
      sol.bearingToWaypointDeg = static_cast<float>(
          navBearingDeg(lat, lon, nextLeg.lat, nextLeg.lon));
      sol.toWpt = nextLeg.id;
    }
  }
  return sol;
}

NavigationSolution FmsNavigator::computeHoldSolution(double lat, double lon,
                                                   float groundSpeedKts) const {
  NavigationSolution sol;
  if (!inHold_ || holdLegIndex_ < 0 ||
      holdLegIndex_ >= static_cast<int>(plan_.size())) {
    return sol;
  }

  const MapLeg& leg = plan_[static_cast<std::size_t>(holdLegIndex_)];
  const HoldGuidance hold =
      computeHoldGuidance(lat, lon, leg, holdPhase_, groundSpeedKts);
  if (!hold.active) return sol;

  sol.active = true;
  sol.directTo = false;
  sol.activeLegIndex = holdLegIndex_;
  sol.fromWpt = hold.fromWpt;
  sol.toWpt = hold.toWpt;
  sol.desiredTrackDeg = hold.desiredTrackDeg;
  sol.crossTrackNm = hold.crossTrackNm;
  sol.distanceToWaypointNm = hold.distanceToWaypointNm;
  sol.bearingToWaypointDeg = hold.bearingToWaypointDeg;
  sol.sequencingSuspended = true;
  return sol;
}

NavigationSolution FmsNavigator::update(double lat, double lon,
                                        float groundSpeedKts, float altitudeFt) {
  NavigationSolution sol;

  if (directToActive_ && !directTo_.id.empty() && !directToOriginValid_) {
    directToOriginLat_ = lat;
    directToOriginLon_ = lon;
    directToOriginValid_ = true;
  }

  if (directToActive_ && !directTo_.id.empty()) {
    if (captureWaypoint(lat, lon, directTo_)) {
      const int dtoIdx = legIndexInPlan(plan_, directTo_);
      if (dtoIdx >= 0 && dtoIdx + 1 < static_cast<int>(plan_.size())) {
        activeLegIndex_ = dtoIdx + 1;
      }
      clearDirectTo();
    }
    if (directToActive_) {
      NavigationSolution sol = computeDirectToSolution(lat, lon);
      sol.missedApproachActive = missedActive_;
      return sol;
    }
  }

  if (plan_.empty()) return sol;

  if (activeLegIndex_ < 0) activeLegIndex_ = 0;
  sequenceActiveLeg(lat, lon, groundSpeedKts, altitudeFt);
  tryEnterHold(lat, lon);

  if (inHold_ && holdLegIndex_ >= 0 &&
      holdLegIndex_ < static_cast<int>(plan_.size())) {
    const MapLeg& holdLeg =
        plan_[static_cast<std::size_t>(holdLegIndex_)];
    holdPhase_ = advanceHoldPatternPhase(lat, lon, holdLeg, holdPhase_,
                                         groundSpeedKts);
    sol = computeHoldSolution(lat, lon, groundSpeedKts);
    sol.sequencingSuspended = true;
    sol.missedApproachActive = missedActive_;
    sol.inHold = true;
    sol.holdRightTurn = holdLeg.hold.turn != HoldTurnDirection::Left;
    return sol;
  }

  sol = computeLegSolution(lat, lon, activeLegIndex_);
  sol.sequencingSuspended = missedSuspended_;
  sol.missedApproachActive = missedActive_;
  return sol;
}

void applyNavigationSolution(FlightData& data, const NavigationSolution& nav,
                             float nmPerDot) {
  if (!nav.active) return;

  data.fmaFromWpt = nav.fromWpt;
  data.fmaToWpt = nav.toWpt;
  data.fmaActiveLegIndex = nav.activeLegIndex;
  data.fmaLegBearingDeg = nav.bearingToWaypointDeg;
  data.fmaLegDistanceNm = nav.distanceToWaypointNm;
  data.courseDeg = nav.desiredTrackDeg;
  data.gpsSequencingSuspended = nav.sequencingSuspended;
  data.missedApproachActive = nav.missedApproachActive;
  data.fmaLegIsHold = nav.inHold;
  data.fmaLegHoldRightTurn = nav.holdRightTurn;

  if (nmPerDot > 0.01f) {
    const float scale = std::max(0.05f, nmPerDot);
    data.cdiDeviationDots =
        std::max(-2.5f, std::min(2.5f, nav.crossTrackNm / scale));
    data.cdiToFlag = true;
    data.navSignalValid = true;
  }
}

}  // namespace avionics
