#pragma once

#include <cmath>

namespace avionics {

// X-Plane sim/cockpit/radios/gps_dme_dist_m is slant range in meters.
inline constexpr float kMetersPerNm = 1852.0f;

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

// Great-circle offset from a lat/lon by true bearing (deg) and distance (NM).
inline void navOffsetPoint(double fromLat, double fromLon, double bearingDeg,
                           double distanceNm, double& outLat, double& outLon) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  constexpr double kNmPerDeg = 60.0;
  const double brg = bearingDeg * kDegToRad;
  const double cosLat = std::max(0.05, std::cos(fromLat * kDegToRad));
  outLat = fromLat + (distanceNm * std::cos(brg)) / kNmPerDeg;
  outLon = fromLon + (distanceNm * std::sin(brg)) / (kNmPerDeg * cosLat);
}

inline float normalizeHeadingDeg(float deg) {
  deg = std::fmod(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

// X-Plane magnetic_variation: positive = east. MH = TH - variation.
inline float trueToMagneticDeg(float trueDeg, float variationDegEast) {
  return normalizeHeadingDeg(trueDeg - variationDegEast);
}

}  // namespace avionics
