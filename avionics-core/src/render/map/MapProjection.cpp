#include "render/map/MapProjection.h"

#include <cmath>

namespace avionics::map {

bool latLonToLocalPx(double lat, double lon, double centerLat, double centerLon,
                     float cx, float cy, float pixelsPerNm, float rotationDeg,
                     float& outX, float& outY) {
  const float mercatorPxPerRad =
      pixelsPerNm * static_cast<float>(kNmPerEarthRad);
  const double rot = static_cast<double>(rotationDeg) * kDegToRad;
  const double cosR = std::cos(rot);
  const double sinR = std::sin(rot);

  double eastRad = 0.0;
  double northRad = 0.0;
  mercatorOffsetRad(lat, lon, centerLat, centerLon, eastRad, northRad);
  mercatorToScreen(eastRad, northRad, cx, cy, mercatorPxPerRad, cosR, sinR,
                   outX, outY);
  return true;
}

}  // namespace avionics::map
