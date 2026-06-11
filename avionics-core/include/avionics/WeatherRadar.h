#pragma once

#include <vector>

namespace avionics {

// Precipitation / weather-radar return data for the moving-map overlay. The map
// samples this each frame (via WeatherRaster), so any backend (X-Plane's radar
// return-strength texture or the procedural demo below) can drive the same
// NEXRAD-style coloring without the renderer knowing the difference.
class WeatherRadarSource {
 public:
  virtual ~WeatherRadarSource() = default;

  // True when the source is powered and has return data to draw.
  virtual bool active() const = 0;

  // Return-strength grid dimensions (pixels).
  virtual int width() const = 0;
  virtual int height() const = 0;

  // Per-pixel return strength 0-255 (w*h bytes, row-major). Only the RED
  // channel is used from X-Plane's radar texture; other backends may fill this
  // directly.
  virtual const unsigned char* returnStrength() const = 0;

  // Forward range represented by the texture height, in NM (aircraft at the
  // bottom edge, forward toward the top).
  virtual float rangeNm() const = 0;

  // Half-width of the scanned sector at max range, in NM (texture left/right
  // edges). Defaults to rangeNm() for a ~90-degree sweep.
  virtual float halfWidthNm() const { return rangeNm(); }

  // Monotonic counter bumped whenever the return grid changes, so cached
  // weather rasters know to re-color.
  virtual unsigned revision() const { return 0; }
};

// Deterministic procedural weather for the mock feed (and offline rendering):
// a slow-drifting precip cell so the NEXRAD overlay can be exercised without
// X-Plane's in-process radar texture.
class ProceduralWeatherRadar : public WeatherRadarSource {
 public:
  void advance(double dtSeconds);

  bool active() const override { return active_; }
  int width() const override { return kSize; }
  int height() const override { return kSize; }
  const unsigned char* returnStrength() const override { return buffer_.data(); }
  float rangeNm() const override { return kRangeNm; }
  unsigned revision() const override { return revision_; }

 private:
  static constexpr int kSize = 256;
  static constexpr float kRangeNm = 80.0f;

  void regenerate();

  bool active_ = true;
  unsigned revision_ = 0;
  double elapsedSeconds_ = 0.0;
  std::vector<unsigned char> buffer_;
};

}  // namespace avionics
