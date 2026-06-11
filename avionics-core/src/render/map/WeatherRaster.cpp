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

// NEXRAD precipitation intensity ramp (G1000 datalink weather legend):
// light/heavy green -> yellow -> red -> magenta, with rising opacity so heavy
// cells read as solid while light returns stay translucent over the map.
Color nexradColor(unsigned char strength) {
  const float t = static_cast<float>(strength) / 255.0f;
  // Below the lightest return threshold the cell is clear (fully transparent),
  // so the overlay shows discrete precipitation rather than a uniform haze.
  if (t < 0.12f) return {0.0f, 0.0f, 0.0f, 0.0f};

  struct Stop {
    float t;
    Color c;  // alpha carries the per-band opacity
  };
  // Bands span the return-strength range; colors are vivid NEXRAD tones.
  static constexpr Stop kStops[] = {
      {0.12f, {0.10f, 0.55f, 0.10f, 0.70f}},  // light green
      {0.35f, {0.00f, 0.85f, 0.00f, 0.80f}},  // green
      {0.55f, {0.95f, 0.95f, 0.00f, 0.88f}},  // yellow
      {0.75f, {1.00f, 0.55f, 0.00f, 0.92f}},  // orange
      {0.90f, {1.00f, 0.00f, 0.00f, 0.95f}},  // red
      {1.00f, {1.00f, 0.00f, 1.00f, 0.97f}},  // magenta (extreme)
  };
  constexpr int n = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
  if (t <= kStops[0].t) return kStops[0].c;
  for (int i = 1; i < n; ++i) {
    if (t <= kStops[i].t) {
      const Color& a = kStops[i - 1].c;
      const Color& b = kStops[i].c;
      const float u = (t - kStops[i - 1].t) / (kStops[i].t - kStops[i - 1].t);
      return {a.r + (b.r - a.r) * u, a.g + (b.g - a.g) * u,
              a.b + (b.b - a.b) * u, a.a + (b.a - a.a) * u};
    }
  }
  return kStops[n - 1].c;
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
  const unsigned char* src = weather.returnStrength();
  if (w <= 0 || h <= 0 || src == nullptr) return;

  v.rgba.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
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
  if (w <= 0 || h <= 0 || weather.returnStrength() == nullptr) return false;

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

  // X-Plane's radar texture places the aircraft at the bottom center; forward
  // is toward the top of the texture. Scale to the source's geographic extent
  // and rotate into the map's current orientation.
  const float forwardPx = std::max(1.0f, weather.rangeNm() * pixelsPerNm);
  const float halfWidthPx =
      std::max(1.0f, weather.halfWidthNm() * pixelsPerNm);
  const float drawW = 2.0f * halfWidthPx;
  const float drawH = forwardPx;

  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotationDeg);
  r.drawImage(v.imageId, -halfWidthPx, -drawH, drawW, drawH, 1.0f);
  r.restore();
  return true;
}

}  // namespace avionics::map
