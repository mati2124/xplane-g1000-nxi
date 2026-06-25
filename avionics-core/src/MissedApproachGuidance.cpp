#include "avionics/MissedApproachGuidance.h"

#include <cmath>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

namespace avionics {

namespace {

double alongTrackDistanceNm(double ownshipLat, double ownshipLon,
                            const std::vector<MapLeg>& plan, int activeIdx,
                            int targetIdx) {
  if (targetIdx < 0 || targetIdx >= static_cast<int>(plan.size())) return 0.0;
  if (activeIdx < 0) activeIdx = 0;
  if (activeIdx > targetIdx) activeIdx = targetIdx;

  double distNm = navDistanceNm(ownshipLat, ownshipLon,
                                plan[static_cast<std::size_t>(activeIdx)].lat,
                                plan[static_cast<std::size_t>(activeIdx)].lon);
  for (int i = activeIdx; i < targetIdx; ++i) {
    distNm += navDistanceNm(plan[static_cast<std::size_t>(i)].lat,
                            plan[static_cast<std::size_t>(i)].lon,
                            plan[static_cast<std::size_t>(i + 1)].lat,
                            plan[static_cast<std::size_t>(i + 1)].lon);
  }
  return distNm;
}

int activeLegIndex(const std::vector<MapLeg>& plan, const FlightData& data) {
  if (data.fmaActiveLegIndex >= 0 &&
      data.fmaActiveLegIndex < static_cast<int>(plan.size())) {
    return data.fmaActiveLegIndex;
  }
  const int idx = legIndexInPlan(plan, data.fmaToWpt);
  return idx >= 0 ? idx : 0;
}

bool isClimbConstraint(const MapLeg& leg, float altitudeFt) {
  if (leg.altitudeConstraintFt <= 0) return false;
  if (static_cast<float>(leg.altitudeConstraintFt) <= altitudeFt + 50.0f) {
    return false;
  }
  return leg.altitudeConstraint == AltConstraintType::At ||
         leg.altitudeConstraint == AltConstraintType::AtOrAbove;
}

}  // namespace

bool suppressGlidepath(const MapData& map, const FlightData& data) {
  if (data.missedApproachActive) return true;
  if (!map.positionValid || map.flightPlan.empty()) return false;

  const std::vector<MapLeg>& plan = map.flightPlan;
  const int maptIdx = findMaptLegIndex(plan);
  if (maptIdx < 0) return false;

  const int activeIdx = activeLegIndex(plan, data);
  if (activeIdx > maptIdx) return true;
  if (activeIdx == maptIdx && data.gpsSequencingSuspended) return true;
  return false;
}

MissedClimbProfile computeMissedClimbProfile(const MapData& map,
                                             const FlightData& data) {
  MissedClimbProfile profile;
  if (!data.missedApproachActive) return profile;
  if (!map.positionValid || map.flightPlan.empty()) return profile;
  if (data.cdiSource != CdiSource::Gps) return profile;

  const std::vector<MapLeg>& plan = map.flightPlan;
  const int activeIdx = activeLegIndex(plan, data);

  int targetIdx = -1;
  for (std::size_t i = static_cast<std::size_t>(activeIdx); i < plan.size();
       ++i) {
    if (isClimbConstraint(plan[i], data.altitudeFt)) {
      targetIdx = static_cast<int>(i);
      break;
    }
  }
  if (targetIdx < 0) return profile;

  const double distNm =
      alongTrackDistanceNm(map.ownshipLat, map.ownshipLon, plan, activeIdx,
                           targetIdx);
  if (distNm < 0.01) return profile;

  const double gsKts = std::max(1.0, static_cast<double>(data.groundSpeedKts));
  const double timeToTargetMin = (distNm / gsKts) * 60.0;
  const int targetAltFt = plan[static_cast<std::size_t>(targetIdx)].altitudeConstraintFt;
  const float vsRequired =
      timeToTargetMin > 0.01
          ? static_cast<float>((targetAltFt - data.altitudeFt) / timeToTargetMin)
          : 0.0f;
  if (vsRequired <= 50.0f) return profile;

  profile.active = true;
  profile.targetWpt = plan[static_cast<std::size_t>(targetIdx)].id;
  profile.targetAltFt = targetAltFt;
  profile.vsRequiredFpm = vsRequired;
  return profile;
}

void applyMissedClimbProfile(FlightData& data, const MissedClimbProfile& profile) {
  if (!profile.active) return;

  data.requiredVsValid = true;
  data.requiredVsFpm = profile.vsRequiredFpm;

  if (data.vdiKind == VerticalDeviationKind::Glidepath ||
      data.vdiKind == VerticalDeviationKind::Glideslope) {
    data.vdiKind = VerticalDeviationKind::None;
    data.vdiValid = false;
  }
}

}  // namespace avionics
