#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Loads X-Plane's enroute airway graph (earth_awy.dat) so the map can draw the
// victor/jet airway overlay behind the AWY softkey. Airway rows reference fix
// and navaid idents, so earth_fix.dat / earth_nav.dat are parsed first into an
// ident+region -> lat/lon table used to resolve each segment to coordinates.
//
// Mirrors NavDataStore: install located via x-plane_install_*.txt, parsing on
// a background thread, immutable once loaded so nearby() reads without locks.
class AirwayStore {
 public:
  AirwayStore();
  ~AirwayStore();

  AirwayStore(const AirwayStore&) = delete;
  AirwayStore& operator=(const AirwayStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  const std::string& sourcePath() const { return sourcePath_; }

  // Segments with either endpoint within rangeNm (+margin) of (lat, lon),
  // capped at maxCount. Empty until loaded().
  std::vector<MapAirwaySegment> nearby(double lat, double lon, float rangeNm,
                                       std::size_t maxCount) const;

  bool isAirwayName(const std::string& name) const;
  std::vector<MapLeg> expandAirway(const std::string& airwayName,
                                   const std::string& fromIdent,
                                   const std::string& toIdent) const;

  // Published airways passing through `ident`, sorted and de-duplicated.
  std::vector<std::string> airwaysThrough(const std::string& ident) const;
  // Ordered fix chain of `airwayName` from `fromIdent` toward the far end.
  std::vector<MapLeg> airwayFixes(const std::string& airwayName,
                                  const std::string& fromIdent) const;

 private:
  void load();  // background-thread entry point

  std::vector<MapAirwaySegment> segments_;
  std::unordered_set<std::string> airwayNames_;
  std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>
      graph_;  // ident|region -> (neighbor ident|region, airway name)
  std::unordered_map<std::string, std::pair<double, double>> points_;
  std::string sourcePath_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
