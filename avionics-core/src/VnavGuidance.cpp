#include "avionics/VnavGuidance.h"

#include <algorithm>
#include <cmath>

#include "avionics/GlidepathGuidance.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/MissedApproachGuidance.h"
#include "avionics/NavMath.h"

namespace avionics {

namespace {

constexpr double kFeetPerNm = 6076.12;
constexpr double kPi = 3.14159265358979323846;

int activeLegIndex(const std::vector<MapLeg>& plan, const std::string& toWpt) {
  const int idx = legIndexInPlan(plan, toWpt);
  return idx >= 0 ? idx : 0;
}

double alongTrackDistanceNm(const MapData& map, const FlightData& data,
                            const std::vector<MapLeg>& plan, int targetIdx) {
  if (targetIdx < 0 || targetIdx >= static_cast<int>(plan.size())) return 0.0;
  int activeIdx = data.fmaActiveLegIndex;
  if (activeIdx < 0) activeIdx = legIndexInPlan(plan, data.fmaToWpt);
  if (activeIdx < 0) activeIdx = 0;
  double distNm = navDistanceNm(map.ownshipLat, map.ownshipLon, plan[activeIdx].lat,
                                plan[activeIdx].lon);
  for (int i = activeIdx; i < targetIdx; ++i) {
    distNm += navDistanceNm(plan[i].lat, plan[i].lon, plan[i + 1].lat,
                            plan[i + 1].lon);
  }
  return distNm;
}

}  // namespace

VnvProfile computeVnvProfile(const MapData& map, const FlightData& data) {
  VnvProfile vnv;
  if (data.missedApproachActive) return vnv;
  if (!map.positionValid || map.flightPlan.size() < 1) return vnv;

  const std::vector<MapLeg>& plan = map.flightPlan;

  const int activeIdx = activeLegIndex(plan, data.fmaToWpt);

  int targetIdx = -1;
  for (std::size_t i = static_cast<std::size_t>(activeIdx); i < plan.size();
       ++i) {
    const MapLeg& leg = plan[i];
    if (leg.altitudeConstraint != AltConstraintType::None &&
        leg.altitudeConstraintFt > 0 &&
        static_cast<float>(leg.altitudeConstraintFt) < data.altitudeFt - 50.0f) {
      targetIdx = static_cast<int>(i);
      break;
    }
  }
  if (targetIdx < 0) return vnv;

  const double distNm = alongTrackDistanceNm(map, data, plan, targetIdx);

  float fpaDeg = kDefaultVnavFpaDeg;
  for (int i = activeIdx; i <= targetIdx; ++i) {
    const float legGpa = plan[static_cast<std::size_t>(i)].glidePathAngleDeg;
    if (legGpa > 0.0f) {
      fpaDeg = legGpa;
      break;
    }
  }
  const double tanFpa = std::tan(static_cast<double>(fpaDeg) * kPi / 180.0);
  const double gsKts = std::max(1.0f, data.groundSpeedKts);

  vnv.active = true;
  vnv.targetWpt = plan[static_cast<std::size_t>(targetIdx)].id;
  vnv.targetAltFt = plan[static_cast<std::size_t>(targetIdx)].altitudeConstraintFt;
  vnv.fpaDeg = fpaDeg;

  const double gsFpm = gsKts * kFeetPerNm / 60.0;
  vnv.vsTargetFpm = static_cast<float>(-tanFpa * gsFpm);

  const double altToLoseFt =
      static_cast<double>(data.altitudeFt) - vnv.targetAltFt;
  const double descentDistNm = (altToLoseFt / tanFpa) / kFeetPerNm;
  vnv.distanceToTodNm = static_cast<float>(distNm - descentDistNm);
  vnv.timeToTodSec =
      vnv.distanceToTodNm > 0.0f
          ? static_cast<int>(std::lround(vnv.distanceToTodNm / gsKts * 3600.0))
          : 0;

  const double timeToTargetMin = (distNm / gsKts) * 60.0;
  vnv.vsRequiredFpm =
      timeToTargetMin > 0.01
          ? static_cast<float>((vnv.targetAltFt - data.altitudeFt) /
                               timeToTargetMin)
          : 0.0f;

  // Deviation from the (possibly extended) descent path at the current along-
  // track position. Valid before TOD too: the path sits above the aircraft and
  // descends to meet it at TOD, so the indicator rides in from the top.
  const double pathAltFt = vnv.targetAltFt + tanFpa * distNm * kFeetPerNm;
  vnv.verticalDeviationFt = static_cast<float>(data.altitudeFt - pathAltFt);
  if (vnv.distanceToTodNm <= 0.0f) {
    vnv.capturing = true;
  }
  return vnv;
}

void applyVnav(FlightData& data, const MapData& map) {
  data.vnv = computeVnvProfile(map, data);
  const bool nearTod = data.vnv.timeToTodSec >= 0 &&
                       data.vnv.timeToTodSec <= kVnavVdiShowBeforeTodSec;
  if (data.vnv.active && (data.vnv.capturing || nearTod) &&
      data.vdiKind == VerticalDeviationKind::None) {
    data.vdiKind = VerticalDeviationKind::Vnav;
    data.vdiValid = true;
    data.vdiDeviationDots = std::max(
        -2.0f, std::min(2.0f, data.vnv.verticalDeviationFt / kVnavDevFtPerDot));
    data.requiredVsValid = true;
    data.requiredVsFpm = data.vnv.vsRequiredFpm;
  }
}

float computeVnavTargetVerticalSpeed(const VnvProfile& vnv) {
  if (!vnv.capturing) return 0.0f;
  const float corrected =
      vnv.vsTargetFpm - kVnavVsGainFpmPerFt * vnv.verticalDeviationFt;
  return std::max(kMinVnavTrackVsFpm, std::min(kMaxVnavTrackVsFpm, corrected));
}

bool suppressVnav(const MapData& map, const FlightData& data) {
  if (data.missedApproachActive) return true;
  if (suppressGlidepath(map, data)) return true;
  // VPTH yields to the approach glidepath only once the aircraft reaches the
  // glidepath's capture window (intercept from below near the FAF) -- not while
  // still descending well below it on the geometric VNAV path. Otherwise the
  // VNAV descent would drop to VS the moment a glidepath became computable, far
  // out and well below the path.
  const GlidepathSolution gp = computeGlidepath(map, data);
  return glidepathSupersedesVnav(gp);
}

}  // namespace avionics
