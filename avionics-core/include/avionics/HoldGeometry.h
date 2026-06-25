#pragma once

#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Racetrack holding-pattern geometry shared by map symbology and lateral guidance.
struct HoldRacetrackGeom {
  double fixLat = 0.0;
  double fixLon = 0.0;
  double outboundEndLat = 0.0;
  double outboundEndLon = 0.0;
  double outboundParLat = 0.0;
  double outboundParLon = 0.0;
  double inboundParLat = 0.0;
  double inboundParLon = 0.0;
  double turn1CenterLat = 0.0;
  double turn1CenterLon = 0.0;
  double turn2CenterLat = 0.0;
  double turn2CenterLon = 0.0;
  float inboundCourseDeg = 0.0f;
  float outboundCourseDeg = 0.0f;
  float legLengthNm = 0.0f;
  float turnRadiusNm = 0.0f;
  bool rightTurn = true;
  bool valid = false;
};

float publishedHoldTurnRadiusNm(float legLengthNm);

// Effective outbound leg length: published NM, or GS-derived from legTimeMin.
float effectiveHoldLegLengthNm(const MapHoldPattern& hold, float groundSpeedKts);

HoldRacetrackGeom buildHoldRacetrack(const MapLeg& leg, float groundSpeedKts = 90.0f);

// Tessellated racetrack path (lat/lon pairs) for map drawing.
std::vector<std::pair<double, double>> tessellateHoldRacetrack(
    const HoldRacetrackGeom& geom, int arcSegments = 10);

}  // namespace avionics
