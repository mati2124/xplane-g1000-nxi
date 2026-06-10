#include "render/map/MapProjection.h"

#include <cmath>

namespace avionics::map {

bool latLonToLocalPx(double lat, double lon, double centerLat, double centerLon,
                     float cx, float cy, float pixelsPerNm, float rotationDeg,
                     float& outX, float& outY) {
  const double dLat = lat - centerLat;
  const double dLon = lon - centerLon;
  const double northNm = dLat * kNmPerDegLat;
  const double eastNm = dLon * nmPerDegLon(centerLat);

  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double rot = rotationDeg * kDegToRad;
  const double cosR = std::cos(rot);
  const double sinR = std::sin(rot);
  // Rotate the north/east offset so the chosen reference (north, heading, or
  // track) points toward the top of the screen (+y down, so "up" is -y).
  const double mapEast = eastNm * cosR - northNm * sinR;
  const double mapNorth = eastNm * sinR + northNm * cosR;

  outX = cx + static_cast<float>(mapEast * pixelsPerNm);
  outY = cy - static_cast<float>(mapNorth * pixelsPerNm);
  return true;
}

}  // namespace avionics::map
