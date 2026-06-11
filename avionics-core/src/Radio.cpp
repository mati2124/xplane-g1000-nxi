#include "avionics/Radio.h"

#include <algorithm>
#include <cmath>

namespace avionics {

namespace {

float wrapChannel(float mhz, float minMhz, float maxMhz, float stepMhz,
                  int direction) {
  float next = mhz + static_cast<float>(direction) * stepMhz;
  if (next > maxMhz + stepMhz * 0.5f) next = minMhz;
  if (next < minMhz - stepMhz * 0.5f) next = maxMhz;
  const int channels =
      static_cast<int>(std::lround((next - minMhz) / stepMhz));
  return minMhz + static_cast<float>(channels) * stepMhz;
}

// Steps the whole-MHz part by `direction`, wrapping the integer MHz within the
// band while preserving the kHz fraction (the outer tuning knob behavior).
float wrapWholeMhz(float mhz, float minMhz, float maxMhz, int direction) {
  const float frac = mhz - std::floor(mhz);
  const float minWhole = std::floor(minMhz);
  const float maxWhole = std::floor(maxMhz);
  float whole = std::floor(mhz) + static_cast<float>(direction);
  if (whole > maxWhole) whole = minWhole;
  if (whole < minWhole) whole = maxWhole;
  float next = whole + frac;
  if (next > maxMhz) next = maxMhz;
  if (next < minMhz) next = minMhz;
  return next;
}

}  // namespace

float stepNavStandbyMhz(float mhz, int direction) {
  return wrapChannel(mhz, kNavFreqMinMhz, kNavFreqMaxMhz, 0.05f, direction);
}

float stepComStandbyMhz(float mhz, int direction) {
  return wrapChannel(mhz, kComFreqMinMhz, kComFreqMaxMhz, 0.025f, direction);
}

float stepNavStandbyMhzCoarse(float mhz, int direction) {
  return wrapWholeMhz(mhz, kNavFreqMinMhz, kNavFreqMaxMhz, direction);
}

float stepComStandbyMhzCoarse(float mhz, int direction) {
  return wrapWholeMhz(mhz, kComFreqMinMhz, kComFreqMaxMhz, direction);
}

}  // namespace avionics
