#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "XPLMDataAccess.h"
#include "avionics/DataSource.h"
#include "avionics/MapData.h"

namespace avionics {

// In-process DataSource for the X-Plane plugin: resolves dataref handles once
// and reads them each frame. Dataref reads inside the sim are cheap, so no
// interpolation is needed here (unlike the standalone/network source).
class DatarefDataSource : public DataSource {
 public:
  DatarefDataSource();
  ~DatarefDataSource() override;

  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }

 private:
  // Rebuild the moving-map snapshot (ownship position, active flight plan, and
  // the range-filtered nearby-feature layer) each frame; the feature scan is
  // throttled by dtSeconds rather than run every frame.
  void updateMap(double dtSeconds);

  // One-time walk of X-Plane's in-RAM navigation database (airports, VORs,
  // NDBs) into a flat cache, range-filtered into map_.features as ownship
  // moves. The XPLMNavigation API must be called on the sim thread, which is
  // where update() runs, so no synchronization is needed.
  void buildNavCache();

  // Background load of X-Plane's OpenAir airspace file (located via
  // XPLMGetSystemPath in the constructor). The XPLMNavigation API doesn't
  // expose airspace, so the plugin parses the same file the standalone does.
  // Parsing runs off the sim thread; airspaceLoaded_ publishes the result.
  void loadAirspaceAsync(std::string airspaceFilePath);

  FlightData data_;
  MapData map_;

  std::vector<MapFeature> navCache_;
  bool navCacheBuilt_ = false;
  double sinceMapRebuildSeconds_ = 0.0;

  std::vector<MapAirspace> airspaceCache_;
  std::atomic<bool> airspaceLoaded_{false};
  std::thread airspaceThread_;

  XPLMDataRef airspeed_ = nullptr;
  XPLMDataRef altitude_ = nullptr;
  XPLMDataRef heading_ = nullptr;
  XPLMDataRef pitch_ = nullptr;
  XPLMDataRef roll_ = nullptr;
  XPLMDataRef verticalSpeed_ = nullptr;
  XPLMDataRef slip_ = nullptr;

  // Ownship geographic position for the moving map (read as doubles).
  XPLMDataRef latitude_ = nullptr;
  XPLMDataRef longitude_ = nullptr;

  // GPS active-leg navigation status box (destination identifier, distance and
  // magnetic bearing). The identifier is a byte[] string read via XPLMGetDatab.
  XPLMDataRef gpsDistance_ = nullptr;
  XPLMDataRef gpsBearing_ = nullptr;
  XPLMDataRef gpsNavId_ = nullptr;
};

}  // namespace avionics
