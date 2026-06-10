#pragma once

#include <cmath>

namespace avionics::map {

// Project a lat/lon offset from the map center into viewport-local pixels.
// centerLat/centerLon is ownship; rotationDeg rotates the map so that angle
// points toward the top of the screen (0 = north up, trackDeg = track up).
// Returns false if the point is outside a generous clip margin (caller may
// still draw off-screen segments clipped by the viewport).
bool latLonToLocalPx(double lat, double lon, double centerLat, double centerLon,
                     float cx, float cy, float pixelsPerNm, float rotationDeg,
                     float& outX, float& outY);

constexpr double kNmPerDegLat = 60.0;

inline double nmPerDegLon(double latDeg) {
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  return kNmPerDegLat * std::cos(latDeg * kDegToRad);
}

}  // namespace avionics::map
