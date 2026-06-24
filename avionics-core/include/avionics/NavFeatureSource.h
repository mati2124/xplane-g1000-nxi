#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// "Where nearby navigation data comes from" for the moving map. This lets the
// platform-agnostic core (e.g. MockDataSource) pull real nav data from a
// shell-provided database (which knows how to find and parse the X-Plane files)
// without the core depending on the shell.
//
// Point features (navaids/fixes/airports) are the required surface; the
// remaining map layers (airspaces, airways, runways, land vectors, obstacles)
// are optional and default to empty so a backend that only supplies features
// (or a unit-test double) need not implement them.
class NavFeatureSource {
 public:
  virtual ~NavFeatureSource() = default;

  // True once the database has finished loading (loading is async on most
  // backends, so this is false for a short while after construction).
  virtual bool ready() const = 0;

  // Up to maxCount features within rangeNm of (lat, lon), nearest first.
  // Returns empty until ready(). Airport features should already carry their
  // map-symbol attributes (towered/serviced/kind) when the backend can supply
  // them, so the caller need not enrich them separately.
  virtual std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                         std::size_t maxCount) const = 0;

  // Additional moving-map layers, each within rangeNm of (lat, lon) and capped
  // at maxCount. Default to empty for backends that do not provide them; a
  // backend that loads its layers on independent background threads may return
  // empty for some while others are populated.
  virtual std::vector<MapAirspace> nearbyAirspaces(double lat, double lon,
                                                   float rangeNm,
                                                   std::size_t maxCount) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }
  virtual std::vector<MapAirwaySegment> nearbyAirways(
      double lat, double lon, float rangeNm, std::size_t maxCount) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }
  virtual std::vector<MapRunway> nearbyRunways(double lat, double lon,
                                               float rangeNm,
                                               std::size_t maxCount) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }
  virtual std::vector<MapPavement> nearbyTaxiways(double lat, double lon,
                                                  float rangeNm,
                                                  std::size_t maxCount) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }
  virtual std::vector<MapLandLine> nearbyLandLines(double lat, double lon,
                                                   float rangeNm,
                                                   std::size_t maxCount,
                                                   float viewHalfExtentNm =
                                                       0.0f) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount, (void)viewHalfExtentNm;
    return {};
  }
  virtual std::vector<MapLandCity> nearbyCities(double lat, double lon,
                                                float rangeNm,
                                                std::size_t maxCount,
                                                float viewHalfExtentNm =
                                                    0.0f) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount,
        (void)viewHalfExtentNm;
    return {};
  }
  virtual std::vector<MapObstacle> nearbyObstacles(double lat, double lon,
                                                   float rangeNm,
                                                   std::size_t maxCount) const {
    (void)lat, (void)lon, (void)rangeNm, (void)maxCount;
    return {};
  }

  // All features whose identifier exactly matches `ident` (the same ident can
  // name an airport, a fix, and a navaid in different regions), capped at
  // maxCount. Used by FMS waypoint entry to resolve a spelled identifier.
  // Default: no lookup support (entry reports the waypoint as not found).
  virtual std::vector<MapFeature> lookupIdent(const std::string& ident,
                                              std::size_t maxCount) const {
    (void)ident;
    (void)maxCount;
    return {};
  }

  // Same as lookupIdent but prefers the match nearest to (refLat, refLon).
  // Terminal-area fixes (e.g. BUTLY at KFMY) require this bias in X-Plane.
  virtual std::vector<MapFeature> lookupIdentNear(const std::string& ident,
                                                    double refLat, double refLon,
                                                    std::size_t maxCount) const {
    return lookupIdent(ident, maxCount);
  }

  // The alphabetically-first identifier starting with `prefix`, for the FMS
  // knob's spell-ahead auto-fill (the G1000 completes the ident as characters
  // are entered). Empty when nothing matches or lookup is unsupported.
  virtual std::string firstIdentWithPrefix(const std::string& prefix) const {
    (void)prefix;
    return {};
  }

  // Cycle / currency metadata for the loaded database, if the backend knows
  // it (parsed from the data file headers). Default: unavailable.
  virtual NavDatabaseInfo navDatabaseInfo() const { return {}; }

  // Published approaches at an airport (ILS/LOC from earth_nav.dat). Empty when
  // the backend does not parse procedure data.
  virtual std::vector<MapApproach> approachesForAirport(
      const std::string& icao) const {
    (void)icao;
    return {};
  }

  // Terminal procedures (SID/STAR/approach) from CIFP, optionally merged with
  // earth_nav ILS approaches for the Approach type.
  virtual std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const {
    (void)icao;
    (void)type;
    return {};
  }

  virtual std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const {
    (void)icao;
    return {};
  }

  // Published runways of an airport (designation, dimensions, surface,
  // lighting) for the WPT/NRST Runways information boxes. Empty when the
  // backend does not parse apt.dat runway records.
  virtual std::vector<AirportRunwayInfo> airportRunways(
      const std::string& icao) const {
    (void)icao;
    return {};
  }

  virtual std::vector<MapLeg> expandProcedure(const std::string& icao,
                                              ProcedureType type,
                                              const std::string& name,
                                              const std::string& transition)
      const {
    (void)icao;
    (void)type;
    (void)name;
    (void)transition;
    return {};
  }

  // Approach transition choices from CIFP feeder routes (empty when unavailable).
  virtual std::vector<ApproachTransitionOption> approachTransitionsFor(
      const std::string& icao, const std::string& approachName) const {
    (void)icao;
    (void)approachName;
    return {};
  }

  // Expand an enroute airway between two published fixes into the intermediate
  // waypoints along that airway. Returns empty when the segment cannot be
  // resolved (unknown airway, disconnected graph, or missing endpoints).
  virtual std::vector<MapLeg> expandAirway(const std::string& airwayName,
                                           const std::string& fromIdent,
                                           const std::string& toIdent) const {
    (void)airwayName;
    (void)fromIdent;
    (void)toIdent;
    return {};
  }

  // True when `name` matches a published airway ident in the loaded graph.
  virtual bool isAirwayName(const std::string& name) const {
    (void)name;
    return false;
  }
};

}  // namespace avionics
