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
// Capture is only allowed within this vertical window of the path so the AP
// intercepts the glidepath from (near) level flight rather than latching on far
// below it: at long final range the 1.25-dot window spans hundreds of feet, so
// without an absolute-altitude gate the AP would capture well low and then sag
// down the path. Slightly above is allowed for a normal from-above intercept.
inline constexpr float kGlidepathCaptureAbovePathFt = 100.0f;
inline constexpr float kGlidepathCaptureBelowPathFt = 150.0f;
// VS trim while GS is captured: fpm added per degree above/below path (+ = above).
// Angle-based gain keeps correction consistent with the VDI at all distances.
inline constexpr float kGlidepathVsGainFpmPerDeg = 1150.0f;

struct GlidepathSolution {
  bool valid = false;
  float deviationDots = 0.0f;
  float angleErrorDeg = 0.0f;
  float altitudeErrorFt = 0.0f;
  float pathAltitudeFt = 0.0f;
  float glidePathAngleDeg = 0.0f;
  float targetVerticalSpeedFpm = 0.0f;
};

// True once the aircraft has flown up into the glidepath's vertical capture
// window (from kGlidepathCaptureBelowPathFt below it and above). At that point
// the approach glidepath supersedes the geometric VNAV descent (VPTH) for both
// the PFD vertical deviation and the AFCS vertical mode. While still well below
// the glidepath the VNAV descent keeps flying through the intermediate altitude
// constraints rather than dropping out the moment a glidepath becomes
// computable far from the runway.
inline bool glidepathSupersedesVnav(const GlidepathSolution& gp) {
  return gp.valid && gp.altitudeErrorFt >= -kGlidepathCaptureBelowPathFt;
}

// Computes LPV/LNAV+V glidepath deviation from loaded CIFP approach legs.
GlidepathSolution computeGlidepath(const MapData& map, const FlightData& data);

// Applies a valid solution to FlightData VDI fields (magenta GPS glidepath).
void applyGlidepathSolution(FlightData& data, const GlidepathSolution& gp);

}  // namespace avionics
