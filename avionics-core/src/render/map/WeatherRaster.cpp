#include "render/map/WeatherRaster.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "avionics/Color.h"
#include "avionics/WeatherRadar.h"

namespace avionics::map {
namespace {

// NEXRAD precipitation coloring, matched verbatim to the real G1000 NXi
// datalink NEXRAD legend (Pilot's Guide Fig. 6-8, "Rain" column). Reflectivity
// runs in discrete bands by dBZ: green (light) -> yellow -> orange -> red. The
// real unit's rain scale tops out at red (magenta/purple there denotes *mixed*
// precip, which this data source can't distinguish), so we cap at red. Rising
// opacity keeps heavy cells solid while light returns stay translucent.
Color nexradColor(unsigned char strength) {
  // returnStrength() packs reflectivity as a fraction of full scale (see
  // kNexradFullScaleDbz); unpack it back to dBZ to pick the legend band.
  const float dbz = (static_cast<float>(strength) / 255.0f) * kNexradFullScaleDbz;

  struct Band {
    float minDbz;
    Color c;  // alpha carries the per-band opacity
  };
  // Garmin G1000 "Rain" legend swatches (Fig. 6-8), sampled from the Pilot's
  // Guide and applied per dBZ threshold.
  static constexpr Band kBands[] = {
      {55.0f, {0.937f, 0.263f, 0.149f, 0.95f}},  // >55 dBZ: red
      {50.0f, {0.949f, 0.451f, 0.161f, 0.93f}},  // >50: red-orange
      {45.0f, {0.973f, 0.643f, 0.169f, 0.91f}},  // >45: orange
      {40.0f, {1.000f, 0.859f, 0.169f, 0.86f}},  // >40: yellow
      {30.0f, {0.992f, 0.867f, 0.169f, 0.84f}},  // >30: yellow
      {20.0f, {0.345f, 0.682f, 0.278f, 0.78f}},  // >20: green
      {10.0f, {0.416f, 0.741f, 0.290f, 0.70f}},  // >10: light green
  };
  // Below the lightest band (>10 dBZ) the cell is clear, so the overlay shows
  // discrete precipitation (matching the legend) rather than a uniform haze.
  for (const Band& band : kBands) {
    if (dbz >= band.minDbz) return band.c;
  }
  return {0.0f, 0.0f, 0.0f, 0.0f};
}

void writePixel(unsigned char* px, const Color& c) {
  px[0] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.r)) * 255.0f));
  px[1] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.g)) * 255.0f));
  px[2] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.b)) * 255.0f));
  px[3] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.a)) * 255.0f));
}

struct ViewWeather {
  Renderer* renderer = nullptr;
  int keyX = 0;
  int keyY = 0;
  std::uint64_t lastUse = 0;

  int imageId = -1;
  int sourceRevision = -1;
  int sourceWidth = 0;
  int sourceHeight = 0;
  std::vector<unsigned char> rgba;
};

constexpr std::size_t kMaxViewWeather = 8;

std::vector<std::unique_ptr<ViewWeather>>& registry() {
  static std::vector<std::unique_ptr<ViewWeather>> views;
  return views;
}

ViewWeather& viewFor(Renderer& r, float cx, float cy) {
  static std::uint64_t useCounter = 0;
  const int kx = static_cast<int>(std::lround(cx));
  const int ky = static_cast<int>(std::lround(cy));
  auto& views = registry();
  for (auto& v : views) {
    if (v->renderer == &r && v->keyX == kx && v->keyY == ky) {
      v->lastUse = ++useCounter;
      return *v;
    }
  }
  if (views.size() >= kMaxViewWeather) {
    auto oldest = std::min_element(
        views.begin(), views.end(),
        [](const auto& a, const auto& b) { return a->lastUse < b->lastUse; });
    if ((*oldest)->renderer == &r && (*oldest)->imageId >= 0) {
      r.deleteImage((*oldest)->imageId);
    }
    views.erase(oldest);
  }
  views.push_back(std::make_unique<ViewWeather>());
  ViewWeather& v = *views.back();
  v.renderer = &r;
  v.keyX = kx;
  v.keyY = ky;
  v.lastUse = ++useCounter;
  return v;
}

void colorize(ViewWeather& v, const WeatherRadarSource& weather) {
  const int w = weather.width();
  const int h = weather.height();
  if (w <= 0 || h <= 0) return;

  const std::size_t bytes =
      static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4;

  // Datalink NEXRAD (and any source carrying its own palette) provides a
  // pre-colorized RGBA grid: blit it straight through.
  if (const unsigned char* rgba = weather.colorRgba()) {
    v.rgba.assign(rgba, rgba + bytes);
    return;
  }

  const unsigned char* src = weather.returnStrength();
  if (src == nullptr) return;
  v.rgba.resize(bytes);
  for (int row = 0; row < h; ++row) {
    unsigned char* px =
        v.rgba.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(w) * 4;
    const unsigned char* in =
        src + static_cast<std::size_t>(row) * static_cast<std::size_t>(w);
    for (int col = 0; col < w; ++col, px += 4) {
      writePixel(px, nexradColor(in[col]));
    }
  }
}

}  // namespace

bool drawWeatherRaster(Renderer& r, const WeatherRadarSource& weather, float cx,
                       float cy, float pixelsPerNm, float rotationDeg) {
  if (!weather.active()) return false;

  const int w = weather.width();
  const int h = weather.height();
  if (w <= 0 || h <= 0 ||
      (weather.returnStrength() == nullptr && weather.colorRgba() == nullptr)) {
    return false;
  }

  ViewWeather& v = viewFor(r, cx, cy);
  const int rev = static_cast<int>(weather.revision());
  const bool needsRebuild = v.imageId < 0 || rev != v.sourceRevision ||
                            w != v.sourceWidth || h != v.sourceHeight;
  if (needsRebuild) {
    colorize(v, weather);
    if (v.imageId < 0) {
      v.imageId = r.createImageRGBA(w, h, v.rgba.data());
    } else {
      r.updateImageRGBA(v.imageId, v.rgba.data());
    }
    v.sourceRevision = rev;
    v.sourceWidth = w;
    v.sourceHeight = h;
  }

  if (v.imageId < 0) return false;

  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotationDeg);
  if (weather.layout() == WeatherRadarLayout::Centered) {
    // Datalink NEXRAD: the aircraft is at the texture center, north toward the
    // top, covering rangeNm in every direction. Draw a square spanning the full
    // diameter centered on ownship and rotate into the map's orientation.
    const float halfPx = std::max(1.0f, weather.rangeNm() * pixelsPerNm);
    const float side = 2.0f * halfPx;
    r.drawImage(v.imageId, -halfPx, -halfPx, side, side, 1.0f);
  } else {
    // X-Plane's radar texture places the aircraft at the bottom center; forward
    // is toward the top of the texture. Scale to the source's geographic extent.
    const float forwardPx = std::max(1.0f, weather.rangeNm() * pixelsPerNm);
    const float halfWidthPx =
        std::max(1.0f, weather.halfWidthNm() * pixelsPerNm);
    const float drawW = 2.0f * halfWidthPx;
    const float drawH = forwardPx;
    r.drawImage(v.imageId, -halfWidthPx, -drawH, drawW, drawH, 1.0f);
  }
  r.restore();
  return true;
}

}  // namespace avionics::map
