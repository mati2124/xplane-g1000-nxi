#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "avionics/NavDatabase.h"

namespace avionics {

class TerrainSource;
class WeatherRadarSource;

// Kind of point feature drawn on the map. Shared by the PFD inset and the MFD
// MAP page.
enum class MapFeatureType { Airport, Vor, Ndb, Fix, Waypoint };

// Airport facility category for map symbology (G1000 NXi Map Symbols appendix).
enum class AirportFacilityKind {
  Land,
  Seaplane,
  Heliport,
  Private,
};

// Published navaid names end with the station-kind suffix ("ANTON CHICO
// VORTAC", "NOLLA NDB"). Splits that suffix off `name` (in place) and returns
// it, so the WPT pages can show the type separately from the facility name;
// returns empty (name untouched) when no known suffix is present. The slash
// form "VOR/DME" normalizes to the hyphenated "VOR-DME" label the G1000 uses.
inline std::string splitNavaidTypeSuffix(std::string& name) {
  static const char* kSuffixes[] = {"VORTAC",  "VOR/DME", "VOR-DME", "VOR",
                                    "NDB/DME", "NDB",     "DME",     "TACAN",
                                    "LOM",     "LMM"};
  const std::size_t space = name.find_last_of(' ');
  std::string tail = space == std::string::npos ? name : name.substr(space + 1);
  for (char& c : tail) {
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  }
  for (const char* suffix : kSuffixes) {
    if (tail == suffix) {
      name = space == std::string::npos ? std::string() : name.substr(0, space);
      return tail == "VOR/DME" ? std::string("VOR-DME") : tail;
    }
  }
  return std::string();
}

struct MapFeature {
  MapFeatureType type = MapFeatureType::Fix;
  double lat = 0.0;
  double lon = 0.0;
  std::string id;

  // Facility name for the WPT information boxes ("LEE COUNTY (FORT MYERS)",
  // "PAGE FIELD"), from earth_nav.dat (navaids) or apt.dat (airports). Empty =
  // unknown (the page dashes it). The navaid type suffix (VOR/VORTAC/VOR-DME/
  // NDB) is split off into `navaidType` during parsing.
  std::string name;
  // Airport city from the apt.dat 1302 metadata ("FORT MYERS"). Airports only.
  std::string city;

  // Airport detail for the WPT/NRST information boxes (0 = unknown, so the page
  // shows dashes the way a real unit does when data is unavailable). Ignored for
  // non-airport features.
  float elevationFt = 0.0f;
  int longestRunwayFt = 0;
  std::string region;  // FAA/ICAO region code (e.g. "K2")

  // Navaid tuning frequency for the WPT/NRST information boxes: MHz for VORs
  // (e.g. 113.90), kHz for NDBs (e.g. 362). 0 = unknown (shown dashed).
  float frequency = 0.0f;

  // Navaid station detail (earth_nav.dat). The type string is the published
  // station kind ("VOR", "VORTAC", "VOR-DME", "NDB", "LOM"); rangeNm is the
  // service-volume range used to classify VORs (Terminal / Low Altitude /
  // High Altitude); magvarDeg is the slaved variation (+E / -W, VORs only).
  std::string navaidType;
  int rangeNm = 0;
  float magvarDeg = 0.0f;
  bool hasMagvar = false;

  // Airport map-symbol attributes (from apt.dat / earth_aptmeta). Towered
  // airports draw cyan; non-towered draw magenta. Serviced airports add the
  // horizontal fuel tabs on the circle. Ignored for non-airport features.
  bool airportTowered = false;
  bool airportServiced = false;
  AirportFacilityKind airportKind = AirportFacilityKind::Land;
};

// VNAV altitude constraint kind for a flight-plan leg (G1000 NXi Pilot's Guide,
// Section 6 "Vertical Navigation"). The "Active VNV Profile" builds a descent
// path to these constraints. None = no constraint (the ALT column shows dashes).
enum class AltConstraintType { None, At, AtOrAbove, AtOrBelow };

// One vertex of the active flight plan (FMS/GPS route).
struct MapLeg {
  double lat = 0.0;
  double lon = 0.0;
  std::string id;

  // VNAV altitude constraint (0 ft = none). A `designated` constraint is one the
  // pilot entered manually (drawn cyan, large in the FPL ALT column); a
  // non-designated constraint comes from a loaded procedure (drawn white).
  int altitudeConstraintFt = 0;
  AltConstraintType altitudeConstraint = AltConstraintType::None;
  bool altitudeDesignated = false;

  // Loaded procedure role suffix (e.g. "iaf", "faf") for the FPL ident column.
  std::string procedureRole;
};

// A bare geographic vertex, used for airspace boundary rings.
struct GeoPoint {
  double lat = 0.0;
  double lon = 0.0;
};

// Airspace category, which drives boundary color/style on the map. Matches the
// G1000 NXi airspace symbol groups (Pilot's Guide, Map Symbols appendix).
// ClassA is parsed but not drawn (no lateral boundary on the moving map).
enum class AirspaceClass {
  ClassA,
  ClassB,
  ClassC,
  ClassD,
  Restricted,
  Prohibited,
  Danger,
  Warning,
  Alert,
  Caution,
  Training,
  MOA,
  TRSA,
  ADIZ,
  TFR,
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

// Victor (low) vs jet (high) airway class, filtered by the AWY softkey.
enum class AirwayLevel { Low, High };

// One airway edge between two published fixes, pre-resolved to coordinates.
// Long airways arrive as many segments; `name` is the airway ident drawn at
// the segment midpoint (e.g. "V521" / "J79").
struct MapAirwaySegment {
  AirwayLevel level = AirwayLevel::Low;
  std::string name;
  GeoPoint a;
  GeoPoint b;
};

// One TIS/TAS traffic target for the map overlay (TIS symbology: open white
// diamond for non-threat traffic, solid yellow circle for a Traffic
// Advisory, with a relative-altitude tag and climb/descend arrow).
struct MapTraffic {
  double lat = 0.0;
  double lon = 0.0;
  float relAltFt = 0.0f;           // relative to ownship; + is above
  float verticalSpeedFpm = 0.0f;   // drives the climb/descend arrow
  bool trafficAdvisory = false;    // TA threat level
};

// Land (cultural/hydro) vector data classes, from the bundled Natural Earth
// extract: drawn per the G1000 Map Setup "Land" group.
enum class LandClass {
  River,
  Lake,
  Road,
  Border,
  City,
  Coast,
  StateBorder,
  Railroad,
  LandMass,
};

struct MapLandLine {
  LandClass landClass = LandClass::River;
  std::vector<GeoPoint> points;
};

// Land label kinds packed into land_data.bin (v2+). Cities are populated
// places; Hydro covers lakes and marine areas; Region covers states/provinces.
enum class LandLabelKind : std::uint8_t {
  City = 0,
  Hydro = 1,
  Region = 2,
};

struct MapLandCity {
  std::string name;
  double lat = 0.0;
  double lon = 0.0;
  int rank = 0;  // higher = more prominent; drives range declutter
  LandLabelKind labelKind = LandLabelKind::City;
};

// One runway of a nearby airport, for the close-range runway diagrams: the
// two threshold center points plus the paved width. idA/idB are the runway-end
// designators (e.g. "05"/"23"), painted on the SafeTaxi diagram at each end.
struct MapRunway {
  GeoPoint a;
  GeoPoint b;
  float widthM = 30.0f;
  std::string idA;
  std::string idB;
};

// Runway surface category for the WPT/NRST Runways boxes, using the G1000
// surface-type vocabulary (Pilot's Guide, Airport Information: "Hard, Turf,
// Sealed, Gravel, Dirt, Soft, Unknown, or Water").
enum class RunwaySurface { Unknown, Hard, Turf, Gravel, Dirt, Soft, Water };

// One runway of a specific airport for the WPT/NRST Runways information boxes
// (as opposed to MapRunway, which is map-diagram geometry): the paired
// designation with dimensions, surface, and lighting from the apt.dat row 100.
struct AirportRunwayInfo {
  std::string designation;  // e.g. "05-23"
  int lengthFt = 0;
  int widthFt = 0;
  RunwaySurface surface = RunwaySurface::Unknown;
  bool lighted = false;  // edge lights present
};

// One closed paved polygon (taxiway, apron, or ramp) from an apt.dat row-110
// pavement chunk, for the close-range airport diagram (SafeTaxi-style). Only
// the outer boundary is kept; holes and bezier curves are approximated as
// straight segments between the boundary nodes.
struct MapPavement {
  std::vector<GeoPoint> outline;
};

// A taxiway identifier label (e.g. "A", "E2") for the SafeTaxi diagram, placed
// at a representative point on the named taxiway. Derived from the apt.dat ATC
// taxi-route network (row 1201 nodes + row 1202 edges).
struct MapTaxiwayLabel {
  GeoPoint pos;
  std::string text;
};

// One obstacle from the (optional) FAA Digital Obstacle File.
struct MapObstacle {
  double lat = 0.0;
  double lon = 0.0;
  float mslFt = 0.0f;
  float aglFt = 0.0f;
  bool lighted = false;     // DDOF LIGHTING column (Pilot's Guide Table 6-7)
  bool windTurbine = false; // DDOF TYPE = WINDMILL (Table 6-8)
  bool isPole = false;      // POLE / UTILITY POLE — single-segment tower glyph
  int quantity = 1;         // DDOF QUANTITY (grouped obstacles draw as a pair)
};

// Procedure category for the FPL PROC menu (Pilot's Guide, Section 5.8).
enum class ProcedureType { Departure, Arrival, Approach };

// One published instrument approach parsed from earth_nav.dat (ILS/LOC rows).
// Grouped by airport ICAO for the WPT/NRST Approaches boxes.
struct MapApproach {
  std::string ident;        // localizer ident (e.g. I04R)
  std::string airportIcao;  // e.g. KSEA
  std::string runway;       // e.g. 16L
  float frequencyMhz = 0.0f;
  float courseDeg = 0.0f;
  bool hasGlideslope = false;
};

// Airport communication service from apt.dat row codes 50–56.
enum class AirportCommService {
  Atis,
  Unicom,
  Clearance,
  Ground,
  Tower,
  Approach,
  Departure,
  Other
};

struct MapAirportFrequency {
  AirportCommService service = AirportCommService::Other;
  float mhz = 0.0f;
  std::string label;
};

// One selectable terminal procedure (SID/STAR/approach) from CIFP data.
struct MapProcedure {
  ProcedureType type = ProcedureType::Approach;
  std::string name;         // e.g. ECHOO1, CHINS5, I16C
  std::string transition;   // e.g. RW12B, PDT, ERYKA
  std::string runway;       // derived runway label when transition is RWxx
  std::string approachKind; // ILS/RNAV/etc. letter from CIFP (approaches only)
  std::string levelOfService; // LPV, LNAV, LNAV/VNAV from CIFP/PRDAT (approaches only)
  float frequencyMhz = 0.0f;  // merged from earth_nav when available
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

  // Active GPS Direct-To target. When set the map overlays a magenta course
  // line from ownship straight to this waypoint (the direct-to leg), on top of
  // any flight plan. `directTo.id` also names the active waypoint.
  bool directToActive = false;
  MapLeg directTo;
  std::vector<MapFeature> features;
  std::vector<MapAirspace> airspaces;
  std::vector<MapAirwaySegment> airways;
  std::vector<MapTraffic> traffic;
  std::vector<MapLandLine> landLines;
  std::vector<MapLandCity> cities;
  std::vector<MapRunway> runways;
  std::vector<MapPavement> taxiways;
  std::vector<MapTaxiwayLabel> taxiwayLabels;
  std::vector<MapObstacle> obstacles;

  // Bumped when airport diagram geometry (runways/taxiways) changes so shells
  // with a render cache can invalidate stale map frames.
  std::uint32_t geometryEpoch = 0;

  // Currency of the loaded navigation database, shown on the power-up page and
  // the AUX - System Status database window (amber when expired, like the real
  // unit). Default (unavailable) when the shell has no cycle metadata.
  NavDatabaseInfo navDatabase;

  // Optional terrain elevation source for the topographic map background. Owned
  // by the DataSource (not this struct); null when no terrain data is available
  // (the map then keeps its plain black background).
  const TerrainSource* terrain = nullptr;

  // Optional onboard weather-radar source (the dedicated MFD Weather Radar
  // page, and the map NEXRAD overlay when no datalink source is wired). Owned by
  // the DataSource (not this struct); null when unavailable.
  const WeatherRadarSource* weather = nullptr;

  // Optional datalink NEXRAD source for the map precipitation overlay (real
  // ground weather radar). When set the overlay prefers this over `weather`.
  // Owned by the DataSource; null when unavailable.
  const WeatherRadarSource* nexrad = nullptr;
};

}  // namespace avionics
