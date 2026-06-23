#pragma once

#include <vector>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics::test {

// Synthetic northbound RNAV final: IAF -> FAF (3 deg GPA, 2000 ft) -> MAPt (50 ft TDZE).
inline std::vector<MapLeg> makeRnavFinalApproachPlan() {
  std::vector<MapLeg> plan;
  MapLeg iaf;
  iaf.id = "IAF01";
  iaf.lat = 26.000000;
  iaf.lon = -81.200000;
  iaf.procedureRole = "iaf";
  iaf.altitudeConstraintFt = 3000;
  iaf.altitudeConstraint = AltConstraintType::AtOrAbove;

  MapLeg faf;
  faf.id = "FAF01";
  faf.lat = 26.050000;
  faf.lon = -81.200000;
  faf.procedureRole = "faf";
  faf.altitudeConstraintFt = 2000;
  faf.altitudeConstraint = AltConstraintType::AtOrAbove;
  faf.glidePathAngleDeg = 3.0f;

  MapLeg mapt;
  mapt.id = "RW09";
  mapt.lat = 26.100000;
  mapt.lon = -81.200000;
  mapt.procedureRole = "mapt";
  mapt.altitudeConstraintFt = 50;
  mapt.altitudeConstraint = AltConstraintType::At;

  plan.push_back(iaf);
  plan.push_back(faf);
  plan.push_back(mapt);
  return plan;
}

inline MapData makeMapAt(double lat, double lon,
                         const std::vector<MapLeg>& plan) {
  MapData map;
  map.positionValid = true;
  map.ownshipLat = lat;
  map.ownshipLon = lon;
  map.flightPlan = plan;
  return map;
}

inline FlightData makeGpsFlightData(const std::string& fromWpt,
                                    const std::string& toWpt,
                                    float legDistanceNm, float altitudeFt) {
  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.fmaFromWpt = fromWpt;
  data.fmaToWpt = toWpt;
  data.fmaLegDistanceNm = legDistanceNm;
  data.altitudeFt = altitudeFt;
  data.groundSpeedKts = 90.0f;
  return data;
}

}  // namespace avionics::test
