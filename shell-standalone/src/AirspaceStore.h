#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Loads X-Plane's airspace boundaries so the standalone shell can draw them on
// the moving map. X-Plane ships them in OpenAir format at
// Resources/default data/airspaces/airspace.txt (and user data under
// Custom Data/Airspaces); the XPLMNavigation API does not expose airspace, so
// both shells parse the file directly.
//
// Mirrors NavDataStore: the install is located via x-plane_install_*.txt, the
// (potentially large) file is parsed on a background thread, and once loaded
// the list is immutable so nearby() reads it without locking.
class AirspaceStore {
 public:
  AirspaceStore();
  ~AirspaceStore();

  AirspaceStore(const AirspaceStore&) = delete;
  AirspaceStore& operator=(const AirspaceStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  const std::string& sourcePath() const { return sourcePath_; }

  // Airspaces whose bounding box is within rangeNm (+margin) of (lat, lon),
  // capped at maxCount. Empty until loaded().
  std::vector<MapAirspace> nearby(double lat, double lon, float rangeNm,
                                  std::size_t maxCount) const;

 private:
  void load();  // background-thread entry point

  std::vector<MapAirspace> airspaces_;
  std::string sourcePath_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
