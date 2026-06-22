#pragma once

#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Programs the X-Plane FMS from the sim thread (XPLM navigation APIs). Shared
// by the in-process plugin data source and the UDP flight-plan bridge.
void programFmsRoute(const std::vector<MapLeg>& legs);
void programFmsDirectTo(bool active, const MapLeg& target);

}  // namespace avionics
