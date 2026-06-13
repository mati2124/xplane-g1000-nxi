#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Verbatim service labels for decoded COM station idents (G1000 NXi Pilot's
// Guide, Table 5-6 Airport Frequency Abbreviations).
const char* airportCommServiceLabel(AirportCommService service);

// True when two COM frequencies are the same 25 kHz (or 8.33 kHz) channel.
bool comFrequenciesMatch(float aMhz, float bMhz);

// Fills FlightData::com1Ident / com2Ident from the active COM frequencies and
// the loaded airport database (format: "KTRM UNICOM"). Clears each field when
// the nav source is unavailable or no published frequency matches.
void applyComDecodedIdents(FlightData& data, const MapData& map,
                           const NavFeatureSource* navSource);

}  // namespace avionics
