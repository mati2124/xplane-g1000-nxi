#pragma once

#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Programs the X-Plane FMS from the sim thread (XPLM navigation APIs). Shared
// by the in-process plugin data source and the UDP flight-plan bridge.
void programFmsRoute(const std::vector<MapLeg>& legs);
void programFmsDirectTo(bool active, const MapLeg& target);
// Sets the sim FMS destination (active TO waypoint) to match our flight-plan
// leg index. Matches by ident when the FMS entry layout differs from the plan.
void programFmsActiveLeg(int legIndex, const std::vector<MapLeg>& plan);

}  // namespace avionics
