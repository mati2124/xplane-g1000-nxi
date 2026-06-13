#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "DatarefDataSource.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Minimal NavFeatureSource for the X-Plane plugin: exposes apt.dat airport comm
// frequencies so the core can decode active COM station names (e.g. KTRM UNICOM).
// The plugin's moving map is driven directly by X-Plane datarefs, so the point-
// feature query (nearby) is not provided here -- only the comm-frequency lookup
// is.
class PluginNavMapData : public NavFeatureSource {
 public:
  explicit PluginNavMapData(DatarefDataSource* data) : data_(data) {}

  bool ready() const override {
    return data_ != nullptr && data_->aptDatReady();
  }

  std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                 std::size_t maxCount) const override {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }

  std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const override {
    if (data_ == nullptr) return {};
    return data_->airportFrequencies(icao);
  }

 private:
  DatarefDataSource* data_ = nullptr;
};

}  // namespace avionics
