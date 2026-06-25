#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// Missed-approach climb target from published at-or-above constraints on the
// missed segment (e.g. IBITE +2600 on KFMY R05).
struct MissedClimbProfile {
  bool active = false;
  std::string targetWpt;
  int targetAltFt = 0;
  float vsRequiredFpm = 0.0f;
};

// True when GPS glidepath should not be shown or coupled (MAPt SUSP or missed
// segment active).
bool suppressGlidepath(const MapData& map, const FlightData& data);

MissedClimbProfile computeMissedClimbProfile(const MapData& map,
                                             const FlightData& data);

void applyMissedClimbProfile(FlightData& data, const MissedClimbProfile& profile);

}  // namespace avionics
