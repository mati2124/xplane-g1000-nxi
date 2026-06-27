#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// Default VNAV flight-path angle when none is published (3.0 deg, G1000 NXi).
inline constexpr float kDefaultVnavFpaDeg = 3.0f;
// PFD VNAV vertical-deviation full-scale: +/-2 dots at +/-500 ft.
inline constexpr float kVnavDevFtPerDot = 250.0f;
// VS trim while VNAV is captured: fpm added per foot above/below path (+ = above).
inline constexpr float kVnavVsGainFpmPerFt = 2.5f;
inline constexpr float kMinVnavTrackVsFpm = -3000.0f;
inline constexpr float kMaxVnavTrackVsFpm = 1500.0f;
// G1000 NXi AFCS: VPTH must be acknowledged within 5 minutes of path interception.
inline constexpr int kVnavAckWindowSec = 300;
// White armed VPTH flashes within 1 minute of interception if still unacknowledged.
inline constexpr int kVnavFlashBeforeInterceptSec = 60;
// The PFD VNAV deviation indicator appears one minute before the top of descent
// (G1000 NXi Pilot's Guide, Vertical Navigation).
inline constexpr int kVnavVdiShowBeforeTodSec = 60;

// Computes the active VNAV profile from the flight plan's altitude constraints.
VnvProfile computeVnvProfile(const MapData& map, const FlightData& data);

// Applies a valid profile to FlightData (FPL VNV box + PFD vertical deviation).
void applyVnav(FlightData& data, const MapData& map);

// VS target for autopilot path tracking once past top of descent.
float computeVnavTargetVerticalSpeed(const VnvProfile& vnv);

// True when geometric VNAV should yield to an active approach glidepath.
bool suppressVnav(const MapData& map, const FlightData& data);

}  // namespace avionics
