#pragma once

#include <string>
#include <vector>

namespace avionics {

class TerrainSource;

// Kind of point feature drawn on the map. Shared by the PFD inset and the MFD
// MAP page.
enum class MapFeatureType { Airport, Vor, Ndb, Fix, Waypoint };

struct MapFeature {
  MapFeatureType type = MapFeatureType::Fix;
  double lat = 0.0;
  double lon = 0.0;
  std::string id;

  // Airport detail for the WPT/NRST information boxes (0 = unknown, so the page
  // shows dashes the way a real unit does when data is unavailable). Ignored for
  // non-airport features.
  float elevationFt = 0.0f;
  int longestRunwayFt = 0;
  std::string region;  // FAA/ICAO region code (e.g. "K2")

  // Navaid tuning frequency for the WPT/NRST information boxes: MHz for VORs
  // (e.g. 113.90), kHz for NDBs (e.g. 362). 0 = unknown (shown dashed).
  float frequency = 0.0f;
};

// One vertex of the active flight plan (FMS/GPS route).
struct MapLeg {
  double lat = 0.0;
  double lon = 0.0;
  std::string id;
};

// A bare geographic vertex, used for airspace boundary rings.
struct GeoPoint {
  double lat = 0.0;
  double lon = 0.0;
};

// Airspace category, which drives boundary color/style on the map. "Other"
// covers classes we parse but don't draw (Class A, CTR, etc.).
enum class AirspaceClass {
  ClassB,
  ClassC,
  ClassD,
  Restricted,
  Prohibited,
  Danger,
  Other
};

// One airspace boundary: a closed ring (the renderer connects the last point
// back to the first) plus its class and vertical limits. Arcs and circles from
// the source data are pre-tessellated into the boundary point list. Altitudes
// are in feet MSL (AGL bounds are approximated as MSL); ceilingFt holds a large
// sentinel for "unlimited".
struct MapAirspace {
  AirspaceClass airspaceClass = AirspaceClass::Other;
  std::string name;
  float floorFt = 0.0f;
  float ceilingFt = 60000.0f;
  std::vector<GeoPoint> boundary;
};

// Slow-changing navigation map snapshot, distinct from the per-frame FlightData
// scalars. Shells rebuild feature lists on a timer; the renderer reads this
// each frame.
struct MapData {
  double ownshipLat = 0.0;
  double ownshipLon = 0.0;
  bool positionValid = false;

  float rangeNm = 10.0f;
  std::vector<MapLeg> flightPlan;
  std::vector<MapFeature> features;
  std::vector<MapAirspace> airspaces;

  // Optional terrain elevation source for the topographic map background. Owned
  // by the DataSource (not this struct); null when no terrain data is available
  // (the map then keeps its plain black background).
  const TerrainSource* terrain = nullptr;
};

}  // namespace avionics
