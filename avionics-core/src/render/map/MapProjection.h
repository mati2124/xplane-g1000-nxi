#pragma once

#include <algorithm>
#include <cmath>

namespace avionics::map {

// G1000 NXi moving maps use a Mercator projection (Working Title MapProjection /
// MercatorProjection). Range is calibrated along the meridian from the view
// center to the range ring so panning north/south does not change the perceived
// scale.

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kNmPerDegLat = 60.0;
// Nautical miles per radian of meridian arc (Earth mean radius).
constexpr double kNmPerEarthRad = kNmPerDegLat / kDegToRad;

// Equirectangular longitude scale at a latitude; still used for approximate
// ground-distance queries (terrain sampling, nav snap radii).
inline double nmPerDegLon(double latDeg) {
  return kNmPerDegLat * std::cos(latDeg * kDegToRad);
}

// Wrap a longitude delta into [-180, 180).
inline double lonDeltaDeg(double lonDeg, double centerLonDeg) {
  double d = lonDeg - centerLonDeg;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// Wrap an angle into (-180, 180] (Garmin GeoPoint::set / MathUtils.normalizeAngleDeg).
inline double wrapLonDeg(double lonDeg) {
  while (lonDeg <= -180.0) lonDeg += 360.0;
  while (lonDeg > 180.0) lonDeg -= 360.0;
  return lonDeg;
}

// Mercator Y at the chartable pole limit (see mercatorYRad).
inline double mercatorPoleYRad() {
  constexpr double kMaxLat = 89.5;
  const double latRad = kMaxLat * kDegToRad;
  return std::asinh(std::tan(latRad));
}

// Mercator Y in radians (Garmin MercatorProjection::projectRaw). Use the full
// formula to ~89.5°; clamping earlier (±85°) stacked polar vertices and drew a
// visible horizontal seam when the view reached high latitudes at wide range.
inline double mercatorYRad(double latDeg) {
  constexpr double kMaxLat = 89.5;
  const double clamped = std::max(-kMaxLat, std::min(kMaxLat, latDeg));
  const double latRad = clamped * kDegToRad;
  return std::asinh(std::tan(latRad));
}

inline double mercatorLatDegFromY(double mercatorY) {
  return std::atan(std::sinh(mercatorY)) / kDegToRad;
}

// Pixels per Mercator radian so `rangeNm` along a meridian spans `mapRadiusPx`.
inline float mercatorPixelsPerRad(float mapRadiusPx, float rangeNm) {
  const double mercatorRangeRad =
      static_cast<double>(std::max(0.5f, rangeNm)) / kNmPerEarthRad;
  return mapRadiusPx / static_cast<float>(mercatorRangeRad);
}

// Mercator offset from the view center, in radians (east = dLon, north = dY).
inline void mercatorOffsetRad(double lat, double lon, double centerLat,
                              double centerLon, double& eastRad,
                              double& northRad) {
  eastRad = lonDeltaDeg(lon, centerLon) * kDegToRad;
  northRad = mercatorYRad(lat) - mercatorYRad(centerLat);
}

// Rotate a Mercator east/north offset and convert to screen pixels.
inline void mercatorToScreen(double eastRad, double northRad, float cx, float cy,
                             float mercatorPxPerRad, double cosR, double sinR,
                             float& outX, float& outY) {
  const double mapEast = eastRad * cosR - northRad * sinR;
  const double mapNorth = eastRad * sinR + northRad * cosR;
  outX = cx + static_cast<float>(mapEast * static_cast<double>(mercatorPxPerRad));
  outY = cy - static_cast<float>(mapNorth * static_cast<double>(mercatorPxPerRad));
}

// Project a lat/lon into viewport-local pixels. `pixelsPerNm` is the nominal
// scale (mapRadiusPx / rangeNm); internally converted to Mercator radians.
bool latLonToLocalPx(double lat, double lon, double centerLat, double centerLon,
                     float cx, float cy, float pixelsPerNm, float rotationDeg,
                     float& outX, float& outY);

// Inverse of mercatorToScreen for a map-center scroll delta in pixels (used
// when the pan pointer reaches the edge of its free-move zone).
inline void screenDeltaToMercatorRad(float dx, float dy, float mercatorPxPerRad,
                                     double cosR, double sinR, double& eastRad,
                                     double& northRad) {
  const double mapEast =
      static_cast<double>(dx) / static_cast<double>(mercatorPxPerRad);
  const double mapNorth =
      -static_cast<double>(dy) / static_cast<double>(mercatorPxPerRad);
  eastRad = mapEast * cosR + mapNorth * sinR;
  northRad = -mapEast * sinR + mapNorth * cosR;
}

// Shift a Mercator view center by east/north radians. When the offset crosses a
// pole, reflect through the pole and flip longitude so panning continues on the
// far side of the Earth instead of stalling at ±89.5°.
inline void offsetMercatorCenter(double centerLat, double centerLon,
                                 double eastRad, double northRad,
                                 double& outLat, double& outLon) {
  const double yMax = mercatorPoleYRad();
  double y = mercatorYRad(centerLat) + northRad;
  double lonDeg = centerLon + eastRad / kDegToRad;
  while (y > yMax) {
    y = 2.0 * yMax - y;
    lonDeg += 180.0;
  }
  while (y < -yMax) {
    y = -2.0 * yMax - y;
    lonDeg += 180.0;
  }
  outLat = mercatorLatDegFromY(y);
  outLon = wrapLonDeg(lonDeg);
}

}  // namespace avionics::map
