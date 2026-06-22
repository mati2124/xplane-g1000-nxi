#include "avionics/GlidepathGuidance.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "avionics/NavMath.h"

namespace avionics {

namespace {

constexpr double kFeetPerNm = 6076.12;
constexpr double kPi = 3.14159265358979323846;
// VS trim gain while GS is captured (fpm per foot of path error).
constexpr float kGlidepathVsGainFpmPerFt = 3.0f;
constexpr float kMinGsTrackVsFpm = -2500.0f;
constexpr float kMaxGsTrackVsFpm = 500.0f;

int activeLegIndex(const std::vector<MapLeg>& plan, const std::string& toWpt) {
  if (plan.empty()) return 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (!toWpt.empty() && plan[i].id == toWpt) {
      return static_cast<int>(i);
    }
  }
  return 0;
}

double alongTrackDistanceNm(const MapData& map, const FlightData& data,
                            const std::vector<MapLeg>& plan, int targetIdx) {
  if (targetIdx < 0 || targetIdx >= static_cast<int>(plan.size())) return 0.0;
  const int activeIdx = activeLegIndex(plan, data.fmaToWpt);
  double distNm = navDistanceNm(map.ownshipLat, map.ownshipLon, plan[activeIdx].lat,
                                plan[activeIdx].lon);
  for (int i = activeIdx; i < targetIdx; ++i) {
    distNm += navDistanceNm(plan[i].lat, plan[i].lon, plan[i + 1].lat,
                            plan[i + 1].lon);
  }
  return distNm;
}

bool isRunwayFixIdent(const std::string& id) {
  if (id.size() < 3) return false;
  if (id[0] != 'R' || id[1] != 'W') return false;
  for (std::size_t i = 2; i < id.size(); ++i) {
    const char c = id[i];
    if (!std::isdigit(static_cast<unsigned char>(c)) && c != 'L' && c != 'R') {
      return false;
    }
  }
  return true;
}

int approachSegmentStart(const std::vector<MapLeg>& plan) {
  for (int i = 0; i < static_cast<int>(plan.size()); ++i) {
    if (!plan[static_cast<std::size_t>(i)].procedureRole.empty() ||
        plan[static_cast<std::size_t>(i)].glidePathAngleDeg > 0.0f) {
      return i;
    }
  }
  return -1;
}

int approachThresholdLegIndex(const std::vector<MapLeg>& plan, int approachStart) {
  if (approachStart < 0) return -1;
  for (int i = static_cast<int>(plan.size()) - 1; i >= approachStart; --i) {
    const MapLeg& leg = plan[static_cast<std::size_t>(i)];
    if (leg.procedureRole == "mapt") return i;
    if (isRunwayFixIdent(leg.id)) return i;
  }
  for (int i = static_cast<int>(plan.size()) - 1; i >= approachStart; --i) {
    if (!plan[static_cast<std::size_t>(i)].procedureRole.empty()) {
      return i;
    }
  }
  return -1;
}

float glidePathAngleForSegment(const std::vector<MapLeg>& plan, int startIdx,
                               int endIdx) {
  float gpa = 0.0f;
  for (int i = startIdx; i <= endIdx; ++i) {
    gpa = std::max(gpa, plan[static_cast<std::size_t>(i)].glidePathAngleDeg);
  }
  return gpa;
}

int thresholdElevationFt(const std::vector<MapLeg>& plan, int thrIdx) {
  if (thrIdx < 0) return 0;
  const MapLeg& thr = plan[static_cast<std::size_t>(thrIdx)];
  if (thr.altitudeConstraintFt > 0) return thr.altitudeConstraintFt;
  for (int i = thrIdx + 1; i < static_cast<int>(plan.size()); ++i) {
    const MapLeg& leg = plan[static_cast<std::size_t>(i)];
    if (leg.procedureRole == "mapt" && leg.altitudeConstraintFt > 0) {
      return leg.altitudeConstraintFt;
    }
  }
  return 0;
}

}  // namespace

GlidepathSolution computeGlidepath(const MapData& map, const FlightData& data) {
  GlidepathSolution gp;
  if (!map.positionValid || map.flightPlan.size() < 2) return gp;
  if (data.cdiSource != CdiSource::Gps) return gp;

  const std::vector<MapLeg>& plan = map.flightPlan;
  const int approachStart = approachSegmentStart(plan);
  if (approachStart < 0) return gp;

  const int thrIdx = approachThresholdLegIndex(plan, approachStart);
  if (thrIdx < 0) return gp;

  const float gpaDeg = glidePathAngleForSegment(plan, approachStart, thrIdx);
  if (gpaDeg <= 0.0f) return gp;

  const int thrAltFt = thresholdElevationFt(plan, thrIdx);
  if (thrAltFt <= 0) return gp;

  const double distNm = alongTrackDistanceNm(map, data, plan, thrIdx);
  if (distNm <= 0.05 || distNm > kGlidepathMaxDistNm) return gp;

  const double tanGpa = std::tan(static_cast<double>(gpaDeg) * kPi / 180.0);
  const double pathAltFt = thrAltFt + tanGpa * distNm * kFeetPerNm;
  const float altErrorFt = data.altitudeFt - static_cast<float>(pathAltFt);
  const double distFt = distNm * kFeetPerNm;
  const double angleErrDeg =
      std::atan(static_cast<double>(altErrorFt) / distFt) * 180.0 / kPi;

  gp.valid = true;
  gp.glidePathAngleDeg = gpaDeg;
  gp.pathAltitudeFt = static_cast<float>(pathAltFt);
  gp.altitudeErrorFt = altErrorFt;
  gp.angleErrorDeg = static_cast<float>(angleErrDeg);
  gp.deviationDots = static_cast<float>(
      std::max(-2.5, std::min(2.5, angleErrDeg / kGlidepathDegPerDot)));

  const double gsKts = std::max(1.0, static_cast<double>(data.groundSpeedKts));
  const double pathVsFpm = -tanGpa * gsKts * kFeetPerNm / 60.0;
  gp.targetVerticalSpeedFpm = static_cast<float>(
      std::max(static_cast<double>(kMinGsTrackVsFpm),
               std::min(static_cast<double>(kMaxGsTrackVsFpm),
                        pathVsFpm -
                            static_cast<double>(kGlidepathVsGainFpmPerFt) *
                                altErrorFt)));
  return gp;
}

void applyGlidepathSolution(FlightData& data, const GlidepathSolution& gp) {
  if (!gp.valid) return;
  data.vdiKind = VerticalDeviationKind::Glidepath;
  data.vdiValid = true;
  data.vdiDeviationDots = gp.deviationDots;
}

}  // namespace avionics
