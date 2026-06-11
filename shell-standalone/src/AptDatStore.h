#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "avionics/AptDatParser.h"
#include "avionics/MapData.h"

namespace avionics {

// Loads runway geometry (row-100 land-runway records), hard-surface taxiway /
// apron pavement (row-110 polygons) and airport metadata (tower, fuel
// services, heliport/seaplane/private) from the X-Plane Global Airports
// apt.dat.
//
// The file is large (hundreds of MB), so it is parsed once on a background
// thread into a 1-degree-cell spatial index plus an ICAO-keyed metadata table;
// immutable once loaded so nearby() / airportMeta() read without locks.
class AptDatStore {
 public:
  AptDatStore();
  ~AptDatStore();

  AptDatStore(const AptDatStore&) = delete;
  AptDatStore& operator=(const AptDatStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  const std::string& sourcePath() const { return sourcePath_; }

  // Runways with either threshold within rangeNm (+margin) of (lat, lon),
  // capped at maxCount. Empty until loaded().
  std::vector<MapRunway> nearby(double lat, double lon, float rangeNm,
                                std::size_t maxCount) const;

  // Hard-surface pavement polygons (taxiways/aprons) whose centroid is within
  // rangeNm (+margin) of (lat, lon), capped at maxCount. Empty until loaded().
  std::vector<MapPavement> nearbyTaxiways(double lat, double lon, float rangeNm,
                                          std::size_t maxCount) const;

  // Taxiway identifier labels (SafeTaxi) within rangeNm (+margin) of
  // (lat, lon), capped at maxCount. Empty until loaded().
  std::vector<MapTaxiwayLabel> nearbyTaxiwayLabels(double lat, double lon,
                                                   float rangeNm,
                                                   std::size_t maxCount) const;

  // Airport map-symbol metadata keyed by ICAO ident. Returns nullptr when
  // unknown (caller keeps defaults).
  const AirportMeta* airportMeta(const std::string& icao) const;

  // Copies tower/fuel/kind fields from apt.dat into a MapFeature when present.
  void enrichAirport(MapFeature& feature) const;

  std::vector<MapAirportFrequency> frequenciesForAirport(
      const std::string& icao) const;

 private:
  void load();  // background-thread entry point

  std::unordered_map<int, std::vector<MapRunway>> cells_;
  std::unordered_map<int, std::vector<MapPavement>> pavementCells_;
  std::unordered_map<int, std::vector<MapTaxiwayLabel>> taxiwayLabelCells_;
  std::unordered_map<std::string, AirportMeta> metaByIcao_;
  std::string sourcePath_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
