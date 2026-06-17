#pragma once

#include <vector>

namespace avionics {

// Reflectivity full-scale (dBZ) used to pack a return into the 0-255
// returnStrength() byte: byte = clamp(dBZ, 0, kNexradFullScaleDbz) /
// kNexradFullScaleDbz * 255. The shared overlay (WeatherRaster::nexradColor)
// unpacks it to recolor with the G1000 reflectivity legend. Sources that report
// a generic 0-1 intensity (the onboard radar) effectively span 0..full-scale.
inline constexpr float kNexradFullScaleDbz = 70.0f;

// Default max map range (NM) for the NEXRAD overlay (Map Setup "NEXRAD Data"
// range). Beyond this the overlay declutters even when NEXRAD is toggled on.
inline constexpr float kNexradMapRangeDefaultNm = 250.0f;

// How the return-strength grid is laid out relative to the aircraft, so the map
// overlay and the radar page can place/sample it correctly.
enum class WeatherRadarLayout {
  // Onboard radar sweep: aircraft at the bottom-center, forward toward the top,
  // left/right edges at +/- halfWidthNm. This is what X-Plane's radar texture
  // delivers.
  ForwardSweep,
  // Datalink NEXRAD-style: aircraft at the texture center, north toward the top,
  // covering rangeNm in every direction (a full 360-degree overlay).
  Centered,
};

// Precipitation / weather-radar return data for the moving-map overlay. The map
// samples this each frame (via WeatherRaster), so any backend (X-Plane's radar
// return-strength texture or the procedural demo below) can drive the same
// NEXRAD-style coloring without the renderer knowing the difference.
class WeatherRadarSource {
 public:
  virtual ~WeatherRadarSource() = default;

  // True when the source is powered and has return data to draw.
  virtual bool active() const = 0;

  // Geometry of the return grid. Defaults to the onboard forward sweep; the
  // datalink-style procedural source below reports Centered.
  virtual WeatherRadarLayout layout() const {
    return WeatherRadarLayout::ForwardSweep;
  }

  // Return-strength grid dimensions (pixels).
  virtual int width() const = 0;
  virtual int height() const = 0;

  // Per-pixel return strength 0-255 (w*h bytes, row-major). Only the RED
  // channel is used from X-Plane's radar texture; other backends may fill this
  // directly.
  virtual const unsigned char* returnStrength() const = 0;

  // Optional pre-colorized overlay (w*h*4 bytes, row-major RGBA). When non-null
  // the map overlay blits these pixels directly instead of running the
  // return-strength ramp -- used by datalink NEXRAD, whose source tiles already
  // carry the standard reflectivity palette. Null (default) keeps the
  // strength -> color path.
  virtual const unsigned char* colorRgba() const { return nullptr; }

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
// slow-drifting precip cells so the NEXRAD overlay can be exercised without
// X-Plane's in-process radar texture. Modeled as datalink NEXRAD: the field is
// centered on the aircraft and covers kRadiusNm in every direction, so the
// overlay surrounds ownship like the real G1000 datalink picture rather than a
// forward-only radar wedge.
class ProceduralWeatherRadar : public WeatherRadarSource {
 public:
  void advance(double dtSeconds);

  bool active() const override { return active_; }
  WeatherRadarLayout layout() const override {
    return WeatherRadarLayout::Centered;
  }
  int width() const override { return kSize; }
  int height() const override { return kSize; }
  const unsigned char* returnStrength() const override { return buffer_.data(); }
  // For the centered layout this is the radius from ownship (texture center) to
  // the texture edge; halfWidthNm() (defaulting to rangeNm()) makes it square.
  float rangeNm() const override { return kRadiusNm; }
  unsigned revision() const override { return revision_; }

 private:
  static constexpr int kSize = 512;
  static constexpr float kRadiusNm = 300.0f;

  void regenerate();

  bool active_ = true;
  unsigned revision_ = 0;
  double elapsedSeconds_ = 0.0;
  std::vector<unsigned char> buffer_;
};

}  // namespace avionics
