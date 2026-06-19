#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "AirspaceStore.h"
#include "AirwayStore.h"
#include "AptDatStore.h"
#include "LandDataStore.h"
#include "NavData.h"
#include "ObstacleStore.h"
#include "ProcedureStore.h"
#include "avionics/MapData.h"
#include "avionics/NavDatabase.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Aggregates the standalone shell's X-Plane database stores behind the core's
// NavFeatureSource interface, so every feed (the live UDP connection and the
// motion-only mock) draws the same real navigation data and never fabricates
// it. The stores are owned by the shell and shared by reference, so the large
// databases (notably the Global Airports apt.dat) are parsed only once.
//
// All stores load on their own background threads; each query returns whatever
// is available so far (empty until the corresponding store finishes loading).
class ShellNavMapData : public NavFeatureSource {
 public:
  ShellNavMapData(NavDataStore& navData, AirspaceStore& airspace,
                  AirwayStore& airways, AptDatStore& aptData,
                  LandDataStore& landData, ProcedureStore& procedures,
                  const ObstacleStore* obstacles)
      : navData_(navData),
        airspace_(airspace),
        airways_(airways),
        aptData_(aptData),
        landData_(landData),
        procedures_(procedures),
        obstacles_(obstacles) {}

  bool ready() const override { return navData_.ready(); }

  std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                 std::size_t maxCount) const override {
    std::vector<MapFeature> features =
        navData_.nearby(lat, lon, rangeNm, maxCount);
    // Enrich airports with apt.dat tower/fuel/kind so the map symbology
    // (towered = cyan, serviced = fuel tabs, heliport/seaplane/private glyphs)
    // matches the live feed.
    if (aptData_.loaded()) {
      for (MapFeature& f : features) aptData_.enrichAirport(f);
    }
    return features;
  }

  std::vector<MapFeature> lookupIdent(const std::string& ident,
                                      std::size_t maxCount) const override {
    std::vector<MapFeature> features = navData_.lookupIdent(ident, maxCount);
    if (aptData_.loaded()) {
      for (MapFeature& f : features) aptData_.enrichAirport(f);
    }
    return features;
  }

  std::string firstIdentWithPrefix(const std::string& prefix) const override {
    return navData_.firstIdentWithPrefix(prefix);
  }

  NavDatabaseInfo navDatabaseInfo() const override {
    return navData_.navDatabaseInfo();
  }

  std::vector<MapApproach> approachesForAirport(
      const std::string& icao) const override {
    return navData_.approachesForAirport(icao);
  }

  std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const override {
    if (!aptData_.loaded()) return {};
    return aptData_.frequenciesForAirport(icao);
  }

  std::vector<AirportRunwayInfo> airportRunways(
      const std::string& icao) const override {
    if (!aptData_.loaded()) return {};
    const AirportMeta* meta = aptData_.airportMeta(icao);
    return meta ? meta->runways : std::vector<AirportRunwayInfo>{};
  }

  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    return procedures_.proceduresForAirport(icao, type);
  }

  std::vector<MapLeg> expandProcedure(const std::string& icao,
                                      ProcedureType type,
                                      const std::string& name,
                                      const std::string& transition) const override {
    return procedures_.expandProcedure(icao, type, name, transition, this);
  }

  bool isAirwayName(const std::string& name) const override {
    return airways_.isAirwayName(name);
  }

  std::vector<MapLeg> expandAirway(const std::string& airwayName,
                                   const std::string& fromIdent,
                                   const std::string& toIdent) const override {
    return airways_.expandAirway(airwayName, fromIdent, toIdent);
  }

  std::vector<MapAirspace> nearbyAirspaces(double lat, double lon, float rangeNm,
                                           std::size_t maxCount) const override {
    return airspace_.nearby(lat, lon, rangeNm, maxCount);
  }

  std::vector<MapAirwaySegment> nearbyAirways(
      double lat, double lon, float rangeNm,
      std::size_t maxCount) const override {
    return airways_.nearby(lat, lon, rangeNm, maxCount);
  }

  std::vector<MapRunway> nearbyRunways(double lat, double lon, float rangeNm,
                                       std::size_t maxCount) const override {
    return aptData_.nearby(lat, lon, rangeNm, maxCount);
  }

  std::vector<MapPavement> nearbyTaxiways(double lat, double lon, float rangeNm,
                                          std::size_t maxCount) const override {
    return aptData_.nearbyTaxiways(lat, lon, rangeNm, maxCount);
  }

  std::vector<MapLandLine> nearbyLandLines(double lat, double lon, float rangeNm,
                                           std::size_t maxCount,
                                           float viewHalfExtentNm = 0.0f) const override {
    return landData_.nearbyLines(lat, lon, rangeNm, maxCount, viewHalfExtentNm);
  }

  std::vector<MapLandCity> nearbyCities(double lat, double lon, float rangeNm,
                                        std::size_t maxCount) const override {
    return landData_.nearbyCities(lat, lon, rangeNm, maxCount);
  }

  std::vector<MapObstacle> nearbyObstacles(double lat, double lon, float rangeNm,
                                           std::size_t maxCount) const override {
    if (obstacles_ == nullptr) return {};
    return obstacles_->nearby(lat, lon, rangeNm, maxCount);
  }

 private:
  NavDataStore& navData_;
  AirspaceStore& airspace_;
  AirwayStore& airways_;
  AptDatStore& aptData_;
  LandDataStore& landData_;
  ProcedureStore& procedures_;
  const ObstacleStore* obstacles_ = nullptr;  // optional (US-only DOF)
};

}  // namespace avionics
