#pragma once

#include "XPLMDataAccess.h"
#include "avionics/DataSource.h"

namespace avionics {

// In-process DataSource for the X-Plane plugin: resolves dataref handles once
// and reads them each frame. Dataref reads inside the sim are cheap, so no
// interpolation is needed here (unlike the standalone/network source).
class DatarefDataSource : public DataSource {
 public:
  DatarefDataSource();

  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }

 private:
  FlightData data_;

  XPLMDataRef airspeed_ = nullptr;
  XPLMDataRef altitude_ = nullptr;
  XPLMDataRef heading_ = nullptr;
  XPLMDataRef pitch_ = nullptr;
  XPLMDataRef roll_ = nullptr;
  XPLMDataRef verticalSpeed_ = nullptr;
  XPLMDataRef slip_ = nullptr;

  // GPS active-leg navigation status box (destination identifier, distance and
  // magnetic bearing). The identifier is a byte[] string read via XPLMGetDatab.
  XPLMDataRef gpsDistance_ = nullptr;
  XPLMDataRef gpsBearing_ = nullptr;
  XPLMDataRef gpsNavId_ = nullptr;
};

}  // namespace avionics
