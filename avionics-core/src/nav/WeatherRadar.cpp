#include "avionics/WeatherRadar.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace avionics {

void ProceduralWeatherRadar::advance(double dtSeconds) {
  elapsedSeconds_ += dtSeconds;
  if (buffer_.empty()) {
    buffer_.resize(static_cast<std::size_t>(kSize) * static_cast<std::size_t>(kSize));
  }
  regenerate();
}

namespace {

// One simulated storm cell in normalized texture space (u,v in [0,1]). The
// aircraft sits at the texture center (0.5, 0.5); u increases east, v increases
// south. `intensity` is the peak return at the core.
struct Cell {
  double u;
  double v;
  double radius;
  double intensity;
};

// Smooth pseudo-noise from a couple of incommensurate sinusoids, in [0,1]:
// scatters light (green) returns between the discrete cells so the field reads
// like a broken area of precipitation rather than a few clean blobs.
double scatterNoise(double u, double v, double t) {
  const double n = std::sin(u * 14.0 + t * 0.20) * std::cos(v * 11.0 - t * 0.13) +
                   0.6 * std::sin(u * 23.0 - v * 19.0 + t * 0.31) +
                   0.4 * std::cos(u * 31.0 + v * 27.0 - t * 0.17);
  return std::max(0.0, n / 2.0);  // bias toward 0 so most of the field is clear
}

}  // namespace

void ProceduralWeatherRadar::regenerate() {
  const double t = elapsedSeconds_;
  constexpr double kPi = 3.14159265358979323846;

  // Datalink-style NEXRAD: storm cells scattered on all sides of the aircraft
  // (which is at the texture center), so the overlay surrounds ownship like the
  // real G1000 picture. Cells drift slowly south (increasing v) so the weather
  // "moves" while NEXRAD is displayed, with a gentle east-west wobble. One cell
  // sits close to ownship so precipitation is visible even at the default map
  // range; the severe cell (intensity 1.0) develops the red/magenta core a short
  // distance out.
  const double driftS = 0.020 * std::sin(t * 0.020);  // slow N-S sway
  const Cell cells[] = {
      {0.470 + 0.008 * std::sin(t * 0.05), 0.575 + driftS, 0.045, 0.70},  // near
      {0.690 + 0.015 * std::sin(t * 0.05), 0.330 + driftS, 0.080, 1.00},  // NE severe
      {0.315 + 0.015 * std::cos(t * 0.06), 0.395 + driftS, 0.055, 0.65},  // NW
      {0.380 + 0.012 * std::sin(t * 0.04), 0.690 + driftS, 0.050, 0.50},  // SW
      {0.760 + 0.012 * std::sin(t * 0.08), 0.640 + driftS, 0.050, 0.55},  // SE
  };

  // A diagonal squall line: a tilted band of moderate-to-heavy precipitation
  // sweeping across the field near the aircraft.
  const double lineAngle = 0.6;  // radians, fixed orientation
  const double lineOffset = 0.70 + 0.06 * std::sin(t * 0.03);
  const double lineHalfWidth = 0.016;

  for (int row = 0; row < kSize; ++row) {
    const double v = static_cast<double>(row) / static_cast<double>(kSize - 1);
    const double dvc = v - 0.5;
    unsigned char* out =
        buffer_.data() + static_cast<std::size_t>(row) * kSize;
    for (int col = 0; col < kSize; ++col) {
      const double u = static_cast<double>(col) / static_cast<double>(kSize - 1);

      // Soft circular vignette: fade returns to nothing at the texture edge so
      // the round overlay never shows a hard square boundary on the map.
      const double duc = u - 0.5;
      const double rr = 2.0 * std::sqrt(duc * duc + dvc * dvc);  // 0 center, 1 edge
      if (rr >= 1.0) {
        out[col] = 0;
        continue;
      }
      const double edge = std::min(1.0, (1.0 - rr) / 0.08);

      double strength = 0.0;

      // Discrete cells: a fairly flat, intense core that falls off toward the
      // edges (so heavy cells show a solid red/magenta center ringed by
      // yellow/green), with a turbulent ripple breaking up the core.
      for (const Cell& c : cells) {
        const double du = u - c.u;
        const double dv = v - c.v;
        const double d2 = (du * du + dv * dv) / (c.radius * c.radius);
        if (d2 > 4.0) continue;
        const double core = std::exp(-d2 * d2 * 0.6);  // plateau then steep edge
        const double ripple =
            0.85 + 0.15 * std::sin((du * 18.0 + dv * 15.0 + t * 0.8) * kPi);
        strength = std::max(strength, c.intensity * core * ripple);
      }

      // Squall line band (distance to the tilted line).
      const double lineDist =
          std::fabs(u * std::cos(lineAngle) + v * std::sin(lineAngle) -
                    lineOffset);
      if (lineDist < lineHalfWidth) {
        const double band = 0.55 * (1.0 - lineDist / lineHalfWidth);
        strength = std::max(strength, band);
      }

      // Broken light returns between the systems: keep only the noise peaks so
      // the field reads as scattered patches with clear map between, rather than
      // a solid wash of green.
      const double scat = scatterNoise(u, v, t);
      const double broken = std::max(0.0, scat - 0.62);
      if (broken > 0.0) {
        strength = std::max(strength, std::min(0.45, broken * 1.4));
      }

      strength *= edge;
      out[col] = static_cast<unsigned char>(
          std::lround(std::min(255.0, std::max(0.0, strength * 255.0))));
    }
  }
  active_ = true;
  ++revision_;
}

}  // namespace avionics
