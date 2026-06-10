#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Locates an X-Plane installation and loads its navaid + fix databases so the
// standalone shell can draw a moving map without the in-sim XPLMNavigation API
// (which is only reachable from the plugin shell).
//
// X-Plane records its install paths in a text file (x-plane_install_12.txt /
// _11.txt) under a per-OS preferences directory; each line is an install root.
// Inside a root the nav data lives in "Custom Data/" (user-updated, preferred)
// or "Resources/default data/". We parse earth_nav.dat (VORs + NDBs) and
// earth_fix.dat (fixes) into flat feature lists.
//
// Loading runs on a background thread so the ~tens-of-MB parse never stalls the
// render loop. Once loaded the lists are immutable, so nearby() reads them
// without locking (publication is via the loaded_ acquire/release flag).
class NavDataStore {
 public:
  NavDataStore();
  ~NavDataStore();

  NavDataStore(const NavDataStore&) = delete;
  NavDataStore& operator=(const NavDataStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }

  // Directory the nav data was loaded from, for diagnostics (empty if none).
  const std::string& sourceDir() const { return sourceDir_; }

  // Features within rangeNm of (lat, lon), nearest first, capped at maxCount.
  // Returns empty until loaded().
  std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                 std::size_t maxCount) const;

 private:
  void load();  // background-thread entry point

  std::vector<MapFeature> navaids_;  // VORs + NDBs
  std::vector<MapFeature> fixes_;
  std::string sourceDir_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
