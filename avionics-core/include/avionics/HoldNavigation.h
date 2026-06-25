#pragma once

#include <string>

#include "avionics/HoldGeometry.h"
#include "avionics/MapData.h"

namespace avionics {

enum class HoldEntryType { Direct, Parallel, Teardrop };

enum class HoldPatternPhase {
  EntryTeardrop,
  Outbound,
  OutboundTurn,
  InboundParallel,
  InboundTurn,
};

// Lateral guidance for one racetrack-hold segment.
struct HoldGuidance {
  bool active = false;
  float desiredTrackDeg = 0.0f;
  float crossTrackNm = 0.0f;
  float distanceToWaypointNm = 0.0f;
  float bearingToWaypointDeg = 0.0f;
  std::string fromWpt;
  std::string toWpt;
};

// ICAO holding-entry classification from the inbound track to the hold fix.
HoldEntryType classifyHoldEntry(float approachTrackDeg, float inboundCourseDeg);

// Initial racetrack phase for the classified entry type.
HoldPatternPhase initialHoldPhase(HoldEntryType entry);

// Signed along-track distance (NM) from the hold fix on `courseDeg`.
double holdAlongTrackNm(double lat, double lon, double fixLat, double fixLon,
                        double courseDeg);

HoldGuidance computeHoldGuidance(double lat, double lon, const MapLeg& leg,
                                 HoldPatternPhase phase, float groundSpeedKts);

// Advance the racetrack phase (outbound, turn arcs, inbound, fix capture).
HoldPatternPhase advanceHoldPatternPhase(double lat, double lon,
                                         const MapLeg& leg,
                                         HoldPatternPhase phase,
                                         float groundSpeedKts);

}  // namespace avionics
