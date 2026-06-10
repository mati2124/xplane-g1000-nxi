#pragma once

#include <cmath>

namespace avionics {

// Short-range great-circle helpers for map lists and FMS readouts. Distances
// are in nautical miles; bearings are true degrees.

inline double navDistanceNm(double fromLat, double fromLon, double toLat,
                            double toLon) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  constexpr double kNmPerDeg = 60.0;
  const double cosLat = std::max(0.05, std::cos(fromLat * kDegToRad));
  const double north = (toLat - fromLat) * kNmPerDeg;
  const double east = (toLon - fromLon) * kNmPerDeg * cosLat;
  return std::sqrt(north * north + east * east);
}

inline double navBearingDeg(double fromLat, double fromLon, double toLat,
                            double toLon) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  const double cosLat = std::max(0.05, std::cos(fromLat * kDegToRad));
  const double north = toLat - fromLat;
  const double east = (toLon - fromLon) * cosLat;
  double deg = std::atan2(east, north) / kDegToRad;
  if (deg < 0.0) deg += 360.0;
  return deg;
}

}  // namespace avionics
