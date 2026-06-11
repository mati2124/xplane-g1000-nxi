#pragma once

#include <istream>
#include <string>
#include <unordered_map>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Metadata parsed from apt.dat airport records for map symbology and the
// WPT/NRST information boxes.
struct AirportMeta {
  bool hasControlTower = false;
  bool hasFuelServices = false;
  AirportFacilityKind kind = AirportFacilityKind::Land;
  std::string name;  // facility name from the airport header row
  std::string city;  // 1302 "city" metadata row
  std::vector<MapAirportFrequency> frequencies;
  std::vector<AirportRunwayInfo> runways;
};

// Result of a single-pass apt.dat parse: runway and taxiway-pavement geometry
// indexed by 1° grid cell plus ICAO-keyed airport metadata (tower, fuel,
// heliport/seaplane).
struct AptDatParseResult {
  std::unordered_map<int, std::vector<MapRunway>> runwayCells;
  std::unordered_map<int, std::vector<MapPavement>> pavementCells;
  std::unordered_map<int, std::vector<MapTaxiwayLabel>> taxiwayLabelCells;
  std::unordered_map<std::string, AirportMeta> metaByIcao;
};

// Parses X-Plane Global Airports apt.dat (row 1/16/17 headers, row 14/54
// towers, row 1400 fuel, row 100 runways, row 110 hard-surface pavement for
// the taxiway diagram). Returns empty maps on failure.
AptDatParseResult parseAptDat(std::istream& in);

// Copies tower/fuel/kind fields from apt.dat into a MapFeature when present.
void enrichAirportFromMeta(MapFeature& feature,
                           const std::unordered_map<std::string, AirportMeta>&
                               metaByIcao);

// Spatial queries against the 1° grid cells produced by parseAptDat().
std::vector<MapRunway> nearbyRunwaysFromCells(
    const std::unordered_map<int, std::vector<MapRunway>>& cells, double lat,
    double lon, float rangeNm, std::size_t maxCount);

std::vector<MapPavement> nearbyPavementFromCells(
    const std::unordered_map<int, std::vector<MapPavement>>& cells, double lat,
    double lon, float rangeNm, std::size_t maxCount);

std::vector<MapTaxiwayLabel> nearbyTaxiwayLabelsFromCells(
    const std::unordered_map<int, std::vector<MapTaxiwayLabel>>& cells,
    double lat, double lon, float rangeNm, std::size_t maxCount);

}  // namespace avionics
