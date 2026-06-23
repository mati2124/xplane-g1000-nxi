#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// PFD glidepath full-scale: +/-2 dots at +/-0.7 degrees from path centerline.
inline constexpr float kGlidepathDegPerDot = 0.35f;
// Glidepath guidance is shown within this distance of the runway threshold.
inline constexpr double kGlidepathMaxDistNm = 15.0;
// GS capture when armed and within this many dots of the computed path.
inline constexpr float kGlidepathCaptureMaxDots = 1.25f;
// VS trim while GS is captured: fpm added per degree above/below path (+ = above).
// Angle-based gain keeps correction consistent with the VDI at all distances.
inline constexpr float kGlidepathVsGainFpmPerDeg = 650.0f;

struct GlidepathSolution {
  bool valid = false;
  float deviationDots = 0.0f;
  float angleErrorDeg = 0.0f;
  float altitudeErrorFt = 0.0f;
  float pathAltitudeFt = 0.0f;
  float glidePathAngleDeg = 0.0f;
  float targetVerticalSpeedFpm = 0.0f;
};

// Computes LPV/LNAV+V glidepath deviation from loaded CIFP approach legs.
GlidepathSolution computeGlidepath(const MapData& map, const FlightData& data);

// Applies a valid solution to FlightData VDI fields (magenta GPS glidepath).
void applyGlidepathSolution(FlightData& data, const GlidepathSolution& gp);

}  // namespace avionics
