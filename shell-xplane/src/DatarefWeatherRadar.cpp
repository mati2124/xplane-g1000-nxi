#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
// Must precede DatarefWeatherRadar.h: its XPLM headers pull in windows.h.
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "DatarefWeatherRadar.h"

#include <algorithm>
#include <cmath>

#include "XPLMGraphics.h"
#include "avionics/Datarefs.h"
#include "avionics/MapRange.h"
#include "avionics/MfdController.h"

#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace avionics {
namespace {

constexpr double kReadIntervalSec = 0.25;
constexpr float kDefaultRangeNm = 80.0f;

float rangeFromSelector(int selector) {
  const int idx = std::max(0, std::min(kMapRangeLadderCount - 1, selector));
  return mapRangeNmAt(idx);
}

// Half-width (degrees) of the antenna sweep for each sector-scan setting. Full
// is the radar's native +/-45 deg scan; narrower sectors trim the sweep.
float sectorHalfDeg(RadarSector sector) {
  switch (sector) {
    case RadarSector::Sixty:
      return 30.0f;
    case RadarSector::Forty:
      return 20.0f;
    case RadarSector::Twenty:
      return 10.0f;
    case RadarSector::Full:
    default:
      return 45.0f;
  }
}

}  // namespace

DatarefWeatherRadar::DatarefWeatherRadar() {
  weatherMode_ = XPLMFindDataRef(datarefs::kEfisWeatherMode);
  mapRangeSelector_ = XPLMFindDataRef(datarefs::kEfisMapRangeSelector);
  sectorWidth_ = XPLMFindDataRef(datarefs::kEfisWeatherSectorWidth);
  tilt_ = XPLMFindDataRef(datarefs::kEfisWeatherTilt);
  gain_ = XPLMFindDataRef(datarefs::kEfisWeatherGain);
}

void DatarefWeatherRadar::syncFromController(const MfdController& ui) {
  if (!weatherMode_) return;

  // Resolve the desired sim EFIS weather mode. The dedicated Weather Radar page
  // drives it directly; otherwise the NEXRAD map overlay keeps the radar in Wx
  // so the return-strength texture stays populated.
  using datarefs::EfisWeatherMode;
  const bool radarPage = ui.page() == MfdPage::WeatherRadar;
  int desired = static_cast<int>(EfisWeatherMode::Off);
  if (radarPage) {
    switch (ui.radarMode()) {
      case RadarMode::Weather:
        desired = static_cast<int>(EfisWeatherMode::Wx);
        break;
      case RadarMode::Ground:
        desired = static_cast<int>(EfisWeatherMode::Map);
        break;
      case RadarMode::Standby:
        desired = static_cast<int>(EfisWeatherMode::Off);
        break;
    }
  }
  if (ui.showWeather() && desired == static_cast<int>(EfisWeatherMode::Off)) {
    desired = static_cast<int>(EfisWeatherMode::Wx);
  }
  if (XPLMGetDatai(weatherMode_) != desired) {
    XPLMSetDatai(weatherMode_, desired);
  }

  // The antenna tilt and sector width are only meaningful while the radar page
  // is the active control surface; leave them to the sim otherwise.
  if (!radarPage) return;
  if (tilt_) XPLMSetDataf(tilt_, ui.radarTiltDeg());
  if (sectorWidth_) XPLMSetDataf(sectorWidth_, sectorHalfDeg(ui.radarSector()));
  if (gain_ && !ui.radarGainCalibrated()) {
    // Manual gain is a -ve..+ve offset around the calibrated centre; map it to
    // the sim's 0..1 gain control with 0.5 as calibrated.
    const float g = 0.5f + ui.radarGainManual() * 0.5f;
    XPLMSetDataf(gain_, std::max(0.0f, std::min(1.0f, g)));
  }
}

void DatarefWeatherRadar::update(double dtSeconds) {
  mode_ = weatherMode_ ? XPLMGetDatai(weatherMode_) : 0;

  // Detect the airframe's radar fit. The sim only hands out the radar return
  // texture for aircraft configured with a weather radar in PlaneMaker, so a
  // valid texture id means this airframe is equipped. Latched so the answer is
  // stable regardless of the current radar mode (resetEquipment clears it on an
  // aircraft change). Requires a current GL context, which the avionics draw
  // path that calls update() always has.
  if (!equipped_ && XPLMGetTexture(xplm_Tex_Radar_Pilot) > 0) {
    equipped_ = true;
  }

  if (mapRangeSelector_) {
    rangeNm_ = rangeFromSelector(XPLMGetDatai(mapRangeSelector_));
  } else {
    rangeNm_ = kDefaultRangeNm;
  }

  if (sectorWidth_) {
    const float sectorHalfDeg = XPLMGetDataf(sectorWidth_);
    halfWidthNm_ =
        rangeNm_ * std::tan(sectorHalfDeg * 3.14159265f / 180.0f);
    halfWidthNm_ = std::max(rangeNm_ * 0.5f, halfWidthNm_);
  } else {
    halfWidthNm_ = rangeNm_;
  }

  sinceReadSeconds_ += dtSeconds;
  if (sinceReadSeconds_ < kReadIntervalSec) return;
  sinceReadSeconds_ = 0.0;
  readTexture();
}

bool DatarefWeatherRadar::active() const {
  return mode_ != static_cast<int>(datarefs::EfisWeatherMode::Off) &&
         !strength_.empty() && width_ > 0 && height_ > 0;
}

void DatarefWeatherRadar::readTexture() {
  if (mode_ == static_cast<int>(datarefs::EfisWeatherMode::Off)) {
    strength_.clear();
    width_ = 0;
    height_ = 0;
    return;
  }

  const int texId = XPLMGetTexture(xplm_Tex_Radar_Pilot);
  if (texId <= 0) return;

  XPLMBindTexture2d(texId, 0);

  GLint texW = 0;
  GLint texH = 0;
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &texW);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &texH);
  if (texW <= 0 || texH <= 0) return;

  rgbaScratch_.resize(static_cast<std::size_t>(texW) *
                      static_cast<std::size_t>(texH) * 4);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                rgbaScratch_.data());

  const int pixels = texW * texH;
  strength_.resize(static_cast<std::size_t>(pixels));
  for (int i = 0; i < pixels; ++i) {
    strength_[static_cast<std::size_t>(i)] =
        rgbaScratch_[static_cast<std::size_t>(i) * 4];
  }

  width_ = static_cast<int>(texW);
  height_ = static_cast<int>(texH);
  ++revision_;
}

}  // namespace avionics
