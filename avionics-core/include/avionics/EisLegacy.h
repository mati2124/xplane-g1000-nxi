#pragma once

#include "avionics/FlightData.h"

namespace avionics {

// Read a value from the generic EIS channel bag; returns fallback when missing.
float eisChannelValue(const FlightData& data, const std::string& channel,
                      float fallback = 0.0f);

// Copy well-known EIS channels into the legacy FlightData scalar fields so
// CAS annunciations, trip planning, and map endurance keep working without
// knowing about the channel bag.
void syncEisLegacyFields(FlightData& data);

}  // namespace avionics
