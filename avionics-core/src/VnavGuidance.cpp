#include "avionics/VnavGuidance.h"

#include <algorithm>
#include <cmath>
#include <vector>

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

// Walks the route from ownship along-track by `distNm` and returns the
// geographic point reached, matching the metric used by alongTrackDistanceNm
// (ownship -> active leg fix, then fix-to-fix). Used to place the MFD map "TOD"
// marker on the course. Returns false when the plan runs out before `distNm`.
bool pointAlongTrackNm(const MapData& map, const FlightData& data,
                       const std::vector<MapLeg>& plan, double distNm,
                       double& outLat, double& outLon) {
  if (distNm < 0.0 || plan.empty()) return false;
  int activeIdx = data.fmaActiveLegIndex;
  if (activeIdx < 0) activeIdx = legIndexInPlan(plan, data.fmaToWpt);
  if (activeIdx < 0) activeIdx = 0;
  if (activeIdx >= static_cast<int>(plan.size())) return false;

  double curLat = map.ownshipLat;
  double curLon = map.ownshipLon;
  double remaining = distNm;
  for (int i = activeIdx; i < static_cast<int>(plan.size()); ++i) {
    const double segNm =
        navDistanceNm(curLat, curLon, plan[i].lat, plan[i].lon);
    if (remaining <= segNm || i + 1 >= static_cast<int>(plan.size())) {
      const double brg =
          navBearingDeg(curLat, curLon, plan[i].lat, plan[i].lon);
      navOffsetPoint(curLat, curLon, brg, std::min(remaining, segNm), outLat,
                     outLon);
      return true;
    }
    remaining -= segNm;
    curLat = plan[i].lat;
    curLon = plan[i].lon;
  }
  return false;
}

// A constraint ahead of the aircraft, with its along-track distance.
struct VnvConstraint {
  int legIdx = -1;
  double distNm = 0.0;
  int altFt = 0;
  AltConstraintType type = AltConstraintType::None;
};

// A constrained altitude only drives the descent once it is meaningfully below
// the aircraft (avoids "descending" to constraints we are level with or below).
constexpr float kVnavTargetBelowFt = 50.0f;

// Flight-path angle used to reach a target leg: the first published glidepath
// angle on the active..target legs, otherwise the 3 deg default.
float pathFpaToTarget(const std::vector<MapLeg>& plan, int activeIdx,
                      int targetIdx) {
  for (int i = activeIdx; i <= targetIdx && i < static_cast<int>(plan.size());
       ++i) {
    const float legGpa = plan[static_cast<std::size_t>(i)].glidePathAngleDeg;
    if (legGpa > 0.0f) return legGpa;
  }
  return kDefaultVnavFpaDeg;
}

// Along-track distance to the top of descent that meets altFt by distNm ahead
// at the given flight-path angle. Smaller (earlier) = more urgent to start down.
double topOfDescentDistNm(double distNm, float currentAltFt, int altFt,
                          float fpaDeg) {
  const double tanFpa = std::tan(static_cast<double>(fpaDeg) * kPi / 180.0);
  if (tanFpa <= 0.0) return distNm;
  const double descentDistNm =
      ((static_cast<double>(currentAltFt) - altFt) / tanFpa) / kFeetPerNm;
  return distNm - descentDistNm;
}

// Selects the active VNAV target leg from the constraints ahead, honoring the
// constraint TYPE the way the G1000 NXi does:
//   * "At" / "At or below" are binding gates: the path must be at (or below)
//     them by the fix, so the controlling gate is the one whose top of descent
//     comes first. Starting down early enough for it automatically keeps the
//     aircraft at or below every other ceiling encountered on the way down.
//   * "At or above" is a floor: never something the path descends BELOW. It
//     becomes the active target only when no binding gate is lower, or when the
//     descent toward a lower gate would bust the floor -- in which case the
//     aircraft levels at the floor first (the nearest violated floor wins).
// `cons` must be ordered by ascending along-track distance.
int selectVnavTargetIndex(const std::vector<MapLeg>& plan, int activeIdx,
                          const std::vector<VnvConstraint>& cons,
                          float currentAltFt) {
  int best = -1;
  double bestTod = 0.0;
  for (const VnvConstraint& c : cons) {
    if (static_cast<float>(c.altFt) >= currentAltFt - kVnavTargetBelowFt)
      continue;
    if (c.type != AltConstraintType::At &&
        c.type != AltConstraintType::AtOrBelow)
      continue;
    const float fpa = pathFpaToTarget(plan, activeIdx, c.legIdx);
    const double tod = topOfDescentDistNm(c.distNm, currentAltFt, c.altFt, fpa);
    if (best < 0 || tod < bestTod) {
      best = c.legIdx;
      bestTod = tod;
    }
  }

  if (best < 0) {
    // No binding gate below the aircraft: descend to the nearest floor, if any.
    for (const VnvConstraint& c : cons) {
      if (static_cast<float>(c.altFt) >= currentAltFt - kVnavTargetBelowFt)
        continue;
      if (c.type != AltConstraintType::AtOrAbove) continue;
      best = c.legIdx;  // ascending order: first match is nearest
      break;
    }
  }

  if (best < 0) return -1;

  const VnvConstraint* target = nullptr;
  for (const VnvConstraint& c : cons) {
    if (c.legIdx == best) {
      target = &c;
      break;
    }
  }
  if (target == nullptr) return best;
  if (target->distNm <= 0.0) return best;

  // Floor protection: a crossing restriction we must not descend below before
  // its fix becomes the target if the descent toward the chosen (deeper, lower)
  // gate would bust it. This covers both an "at or above" floor AND the lower
  // edge of an "at" gate -- "cross MCFIE at 12000" must not be flown through at
  // 8000 just because FAROT at 3000 sits further along.
  //
  // Two different trajectories can each bust such a restriction, so guard
  // against the more restrictive (lower) of the two:
  //   * the ideal FPA descent path that arrives at the chosen gate -- catches
  //     the common case where the aircraft is still high and a single
  //     continuous descent to the deep gate slices below a nearer, higher
  //     crossing restriction (e.g. heading straight from cruise to "FAROT at
  //     3000" cuts through "MCFIE at 12000"); and
  //   * the straight line from the aircraft's CURRENT altitude to the gate --
  //     catches the case where the aircraft descended early and is already
  //     below the ideal path, where the FPA path alone would still look clear
  //     (e.g. "cross BUNGE at or above 16000").
  // If either dips below a restriction before its fix, level at the nearest
  // such fix instead, then resume the descent once past it.
  const float fpaToBest = pathFpaToTarget(plan, activeIdx, best);
  const double tanFpaToBest =
      std::tan(static_cast<double>(fpaToBest) * kPi / 180.0);
  for (const VnvConstraint& c : cons) {
    if (c.legIdx >= best) break;
    if (c.type != AltConstraintType::AtOrAbove &&
        c.type != AltConstraintType::At)
      continue;
    const double frac = c.distNm / target->distNm;
    const double projAltCurrent =
        currentAltFt +
        (static_cast<double>(target->altFt) - currentAltFt) * frac;
    const double projAltIdeal =
        static_cast<double>(target->altFt) +
        tanFpaToBest * (target->distNm - c.distNm) * kFeetPerNm;
    const double projAltFt = std::min(projAltCurrent, projAltIdeal);
    if (projAltFt < static_cast<double>(c.altFt) - 1.0) {
      return c.legIdx;
    }
  }
  return best;
}

}  // namespace

VnvProfile computeVnvProfile(const MapData& map, const FlightData& data) {
  VnvProfile vnv;
  if (data.missedApproachActive) return vnv;
  if (!map.positionValid || map.flightPlan.size() < 1) return vnv;

  const std::vector<MapLeg>& plan = map.flightPlan;

  const int activeIdx = activeLegIndex(plan, data.fmaToWpt);

  std::vector<VnvConstraint> cons;
  for (std::size_t i = static_cast<std::size_t>(activeIdx); i < plan.size();
       ++i) {
    const MapLeg& leg = plan[i];
    if (leg.altitudeConstraint == AltConstraintType::None ||
        leg.altitudeConstraintFt <= 0) {
      continue;
    }
    VnvConstraint c;
    c.legIdx = static_cast<int>(i);
    c.distNm = alongTrackDistanceNm(map, data, plan, c.legIdx);
    c.altFt = leg.altitudeConstraintFt;
    c.type = leg.altitudeConstraint;
    cons.push_back(c);
  }

  const int targetIdx =
      selectVnavTargetIndex(plan, activeIdx, cons, data.altitudeFt);
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

  // Deviation from the (possibly extended) descent path at the current along-
  // track position. Valid before TOD too: the path sits above the aircraft and
  // descends to meet it at TOD, so the indicator rides in from the top.
  const double pathAltFt = vnv.targetAltFt + tanFpa * distNm * kFeetPerNm;
  vnv.verticalDeviationFt = static_cast<float>(data.altitudeFt - pathAltFt);

  // Geographic TOD position for the MFD map marker: only while the top of
  // descent is still ahead AND the aircraft is still clearly above the descent
  // path. The geometry is recomputed from the current altitude each frame, so
  // gating on distance alone lets the marker cling to the ownship as it descends
  // just below the path; the deviation dead band makes it disappear at/near the
  // real top of descent the way the unit removes it once the descent begins.
  if (vnv.distanceToTodNm > 0.0f &&
      vnv.verticalDeviationFt < -kVnavTodHideDevFt) {
    vnv.todValid = pointAlongTrackNm(map, data, plan, vnv.distanceToTodNm,
                                     vnv.todLat, vnv.todLon);
  }

  // Geographic bottom-of-descent position for the MFD map marker: the point
  // where the path levels at the target constraint, i.e. the target waypoint
  // (distNm along track). Shown while the target leg is still ahead.
  if (distNm > 0.0) {
    vnv.bodValid =
        pointAlongTrackNm(map, data, plan, distNm, vnv.bodLat, vnv.bodLon);
  }

  const double timeToTargetMin = (distNm / gsKts) * 60.0;
  vnv.vsRequiredFpm =
      timeToTargetMin > 0.01
          ? static_cast<float>((vnv.targetAltFt - data.altitudeFt) /
                               timeToTargetMin)
          : 0.0f;

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
