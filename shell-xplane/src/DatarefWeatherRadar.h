#pragma once

#include <vector>

#include "XPLMDataAccess.h"
#include "avionics/WeatherRadar.h"

namespace avionics {

class MfdController;

// Reads X-Plane 12.3's pilot-side weather radar return-strength texture
// (xplm_Tex_Radar_Pilot) into a CPU buffer for the shared WeatherRaster
// overlay. Must be updated on the sim thread with a current GL context.
class DatarefWeatherRadar : public WeatherRadarSource {
 public:
  DatarefWeatherRadar();

  void update(double dtSeconds);
  // Push the desired radar state to the sim from the MFD controller: the EFIS
  // weather mode follows the Weather Radar page (Weather->Wx, Ground->Map,
  // Standby->Off) or the NEXRAD map overlay, and the antenna tilt / sector
  // width follow the page's controls.
  void syncFromController(const MfdController& ui);

  bool active() const override;
  int width() const override { return width_; }
  int height() const override { return height_; }
  const unsigned char* returnStrength() const override {
    return strength_.empty() ? nullptr : strength_.data();
  }
  float rangeNm() const override { return rangeNm_; }
  float halfWidthNm() const override { return halfWidthNm_; }
  unsigned revision() const override { return revision_; }

 private:
  void readTexture();

  XPLMDataRef weatherMode_ = nullptr;
  XPLMDataRef mapRangeSelector_ = nullptr;
  XPLMDataRef sectorWidth_ = nullptr;
  XPLMDataRef tilt_ = nullptr;
  XPLMDataRef gain_ = nullptr;

  int mode_ = 0;
  float rangeNm_ = 80.0f;
  float halfWidthNm_ = 80.0f;
  int width_ = 0;
  int height_ = 0;
  unsigned revision_ = 0;
  double sinceReadSeconds_ = 0.0;

  std::vector<unsigned char> strength_;
  std::vector<unsigned char> rgbaScratch_;
};

}  // namespace avionics
