#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace avionics {

// Shared G1000 moving-map range ladder, in NM. The PFD inset map and the MFD
// MAP page both step through this list (RNG- / RNG+ on the softkeys or the
// on-screen bezel range rocker), so they share one definition rather than each
// hard-coding their own.
inline constexpr float kMapRangeLadderNm[] = {
    0.5f,   1.0f,   1.5f,   2.5f,   5.0f,   10.0f,  15.0f,  25.0f,
    50.0f,  100.0f, 150.0f, 250.0f, 350.0f, 500.0f, 750.0f, 1000.0f,
    1500.0f, 2000.0f};
inline constexpr int kMapRangeLadderCount =
    static_cast<int>(sizeof(kMapRangeLadderNm) / sizeof(kMapRangeLadderNm[0]));

// Default ladder position (10 NM), the G1000 power-on map range.
inline constexpr int kMapRangeDefaultIndex = 5;

// Top of the shared range ladder (2000 NM on the NXi).
inline constexpr float kMapRangeMaxNm =
    kMapRangeLadderNm[kMapRangeLadderCount - 1];

// Land/border vector queries must cover the full ladder with margin so
// coastlines and country borders reach the edges of a wide view.
inline constexpr float kLandQueryRangeNm = kMapRangeMaxNm * 1.1f;

// Embedded WPT Airport Information map range: tight enough to show the
// SafeTaxi-style runway + taxiway pavement diagram.
inline constexpr float kAirportDiagramRangeNm = 2.5f;

// Clamp a ladder index into range and return the corresponding NM value.
inline float mapRangeNmAt(int index) {
  index = std::max(0, std::min(kMapRangeLadderCount - 1, index));
  return kMapRangeLadderNm[index];
}

// Time constant (seconds) for the moving-map zoom animation. The displayed
// scale eases toward the selected ladder step instead of snapping, matching
// the Working Title G1000 NXi's smooth zoom. Small enough to feel responsive
// to a range-rocker press, large enough to read as an animation.
inline constexpr float kMapZoomTimeConstantSec = 0.12f;

// Ease an animated map range (NM) toward `target` over `dtSeconds`, using
// frame-rate-independent exponential smoothing in log space. Working in log
// space makes each ladder step cover a similar-looking zoom regardless of the
// absolute range (0.5->1 NM reads like 100->250 NM). Returns the target exactly
// once within ~0.4% so the animation settles; a non-positive `current` (an
// uninitialized animator) jumps straight to the target.
inline float animateMapRange(float current, float target, double dtSeconds) {
  if (current <= 0.0f || target <= 0.0f) return target;
  const float logCur = std::log(current);
  const float logTgt = std::log(target);
  if (std::fabs(logTgt - logCur) < 0.004f) return target;
  const float a =
      1.0f - std::exp(-static_cast<float>(dtSeconds) / kMapZoomTimeConstantSec);
  return std::exp(logCur + (logTgt - logCur) * a);
}

// Format a map range for the on-screen range readout, matching the G1000 NXi:
// whole-NM ranges print as integers ("10NM"), the half-NM ladder steps keep one
// decimal ("2.5NM"). Rounds to the nearest 0.1 NM first so float error in the
// ladder values doesn't leak a spurious decimal.
inline void formatMapRange(char* buf, std::size_t n, float rangeNm) {
  const float tenths = std::round(rangeNm * 10.0f) / 10.0f;
  if (std::fabs(tenths - std::round(tenths)) < 0.05f) {
    std::snprintf(buf, n, "%dNM", static_cast<int>(std::lround(tenths)));
  } else {
    std::snprintf(buf, n, "%.1fNM", static_cast<double>(tenths));
  }
}

}  // namespace avionics
