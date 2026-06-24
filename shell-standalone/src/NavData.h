#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/NavDatabase.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Locates an X-Plane installation and loads its navaid + fix databases so the
// standalone shell can draw a moving map without the in-sim XPLMNavigation API
// (which is only reachable from the plugin shell).
//
// X-Plane records its install paths in a text file (x-plane_install_12.txt /
// _11.txt) under a per-OS preferences directory; each line is an install root.
// Inside a root the nav data lives in "Custom Data/" (user-updated, preferred)
// or "Resources/default data/". We parse earth_nav.dat (VORs + NDBs),
// earth_fix.dat (fixes), and earth_aptmeta.dat (airport ICAO/lat/lon) into flat
// feature lists.
//
// Loading runs on a background thread so the ~tens-of-MB parse never stalls the
// render loop. Once loaded the lists are immutable, so nearby() reads them
// without locking (publication is via the loaded_ acquire/release flag).
class NavDataStore : public NavFeatureSource {
 public:
  NavDataStore();
  ~NavDataStore() override;

  NavDataStore(const NavDataStore&) = delete;
  NavDataStore& operator=(const NavDataStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  bool ready() const override { return loaded(); }

  // Directory the nav data was loaded from, for diagnostics (empty if none).
  const std::string& sourceDir() const { return sourceDir_; }

  // AIRAC cycle / currency of the loaded data, parsed from the earth_nav.dat
  // header ("... data cycle 2506 ..."). Unavailable until loaded() or if the
  // header carries no cycle.
  NavDatabaseInfo navDatabaseInfo() const override {
    return loaded() ? dbInfo_ : NavDatabaseInfo{};
  }

  // Features within rangeNm of (lat, lon), nearest first, capped at maxCount.
  // Returns empty until loaded().
  std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                 std::size_t maxCount) const override;

  // FMS waypoint entry lookups, backed by an ident-sorted index built during
  // load: all features matching an exact identifier, and the alphabetically
  // first identifier with a given prefix (the G1000 spell-ahead fill-in).
  std::vector<MapFeature> lookupIdent(const std::string& ident,
                                      std::size_t maxCount) const override;
  std::vector<MapFeature> lookupIdentNear(const std::string& ident,
                                          double refLat, double refLon,
                                          std::size_t maxCount) const override;
  std::string firstIdentWithPrefix(const std::string& prefix) const override;

  std::vector<MapApproach> approachesForAirport(
      const std::string& icao) const override;

 private:
  void load();  // background-thread entry point

  std::vector<MapFeature> airports_;
  std::vector<MapFeature> navaids_;  // VORs + NDBs
  std::vector<MapFeature> fixes_;
  // ILS/LOC approaches keyed by airport ICAO (earth_nav row codes 4 and 5).
  std::unordered_map<std::string, std::vector<MapApproach>> approachesByAirport_;
  // Every feature above, sorted by ident (airports/navaids before fixes for
  // equal idents). Built on the loader thread, immutable afterwards.
  std::vector<const MapFeature*> identIndex_;
  std::string sourceDir_;
  NavDatabaseInfo dbInfo_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
