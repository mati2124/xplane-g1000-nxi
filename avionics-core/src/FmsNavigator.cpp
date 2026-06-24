#include "avionics/FmsNavigator.h"

#include <algorithm>
#include <cmath>

#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWaypointCaptureNm = 0.4;

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
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
}

void FmsNavigator::setActiveLegIndex(int toLegIndex) {
  if (toLegIndex < 0 || toLegIndex >= static_cast<int>(plan_.size())) return;
  activeLegIndex_ = toLegIndex;
  directToActive_ = false;
  directTo_ = {};
  directToOriginValid_ = false;
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

void FmsNavigator::sequenceActiveLeg(double lat, double lon) {
  if (obsMode_ || plan_.empty() || activeLegIndex_ < 0) return;

  const MapLeg& toWpt = plan_[static_cast<std::size_t>(activeLegIndex_)];
  if (!captureWaypoint(lat, lon, toWpt)) return;

  if (activeLegIndex_ + 1 < static_cast<int>(plan_.size())) {
    ++activeLegIndex_;
  }
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
  return sol;
}

NavigationSolution FmsNavigator::update(double lat, double lon,
                                        float /*groundSpeedKts*/) {
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
      return computeDirectToSolution(lat, lon);
    }
  }

  if (plan_.empty()) return sol;

  if (activeLegIndex_ < 0) activeLegIndex_ = 0;
  sequenceActiveLeg(lat, lon);
  return computeLegSolution(lat, lon, activeLegIndex_);
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

  if (nmPerDot > 0.01f) {
    const float scale = std::max(0.05f, nmPerDot);
    data.cdiDeviationDots =
        std::max(-2.5f, std::min(2.5f, nav.crossTrackNm / scale));
    data.cdiToFlag = true;
    data.navSignalValid = true;
  }
}

}  // namespace avionics
