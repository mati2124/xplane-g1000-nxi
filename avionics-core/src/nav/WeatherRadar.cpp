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

// One simulated storm cell in normalized texture space (u,v in [0,1], with the
// aircraft at the bottom-center). `intensity` is the peak return at the core.
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

  // A handful of storm cells of varying intensity, drifting slowly toward the
  // aircraft (decreasing v) so the weather "moves" while NEXRAD is displayed.
  // A severe cell (intensity 1.0) develops the red/magenta core; weaker cells
  // stay green/yellow.
  const double drift = std::fmod(t * 0.004, 1.0);
  const Cell cells[] = {
      {0.62 + 0.05 * std::sin(t * 0.05), 0.30 + drift, 0.16, 1.00},  // severe
      {0.34 + 0.04 * std::cos(t * 0.06), 0.55 + drift, 0.13, 0.70},
      {0.78 + 0.03 * std::sin(t * 0.08), 0.68 + drift, 0.10, 0.55},
      {0.20 + 0.03 * std::sin(t * 0.04), 0.82 + drift, 0.08, 0.45},
  };

  // A diagonal squall line: a tilted band of moderate-to-heavy precipitation
  // sweeping across the field ahead of the cells.
  const double lineAngle = 0.6;  // radians, fixed orientation
  const double lineOffset = 0.18 + 0.10 * std::sin(t * 0.03);
  const double lineHalfWidth = 0.045;

  for (int row = 0; row < kSize; ++row) {
    const double v = static_cast<double>(row) / static_cast<double>(kSize - 1);
    unsigned char* out =
        buffer_.data() + static_cast<std::size_t>(row) * kSize;
    for (int col = 0; col < kSize; ++col) {
      const double u = static_cast<double>(col) / static_cast<double>(kSize - 1);

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
        const double band = 0.6 * (1.0 - lineDist / lineHalfWidth);
        strength = std::max(strength, band);
      }

      // Scattered light returns between systems.
      strength = std::max(strength, 0.28 * scatterNoise(u, v, t));

      out[col] = static_cast<unsigned char>(
          std::lround(std::min(255.0, std::max(0.0, strength * 255.0))));
    }
  }
  active_ = true;
  ++revision_;
}

}  // namespace avionics
