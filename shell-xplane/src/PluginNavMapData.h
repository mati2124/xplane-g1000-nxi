#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "DatarefDataSource.h"
#include "ProcedureStore.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// NavFeatureSource for the X-Plane plugin: airport comm frequencies, CIFP
// terminal procedures, and FMS waypoint lookup via the live nav database.
class PluginNavMapData : public NavFeatureSource {
 public:
  PluginNavMapData(DatarefDataSource* data, ProcedureStore* procedures)
      : data_(data), procedures_(procedures) {}

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

  std::vector<MapFeature> lookupIdent(const std::string& ident,
                                      std::size_t maxCount) const override {
    if (data_ == nullptr) return {};
    return data_->lookupNavIdent(ident, maxCount);
  }

  std::string firstIdentWithPrefix(const std::string& prefix) const override {
    if (data_ == nullptr) return {};
    return data_->firstNavIdentWithPrefix(prefix);
  }

  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    if (procedures_ == nullptr) return {};
    return procedures_->proceduresForAirport(icao, type);
  }

  std::vector<MapLeg> expandProcedure(const std::string& icao,
                                      ProcedureType type,
                                      const std::string& name,
                                      const std::string& transition)
      const override {
    if (procedures_ == nullptr) return {};
    return procedures_->expandProcedure(icao, type, name, transition, this);
  }

  std::vector<ApproachTransitionOption> approachTransitionsFor(
      const std::string& icao, const std::string& approachName) const override {
    if (procedures_ == nullptr) return {};
    return procedures_->approachTransitionsFor(icao, approachName);
  }

 private:
  DatarefDataSource* data_ = nullptr;
  ProcedureStore* procedures_ = nullptr;
};

}  // namespace avionics
