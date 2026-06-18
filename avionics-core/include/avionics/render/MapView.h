#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/Renderer.h"
#include "avionics/WeatherRadar.h"

namespace avionics {

// How the map is rotated inside its viewport. The PFD inset and MFD MAP page
// share the same renderer; each caller picks orientation via MapViewConfig.
enum class MapOrientation { NorthUp, HeadingUp, TrackUp };

// Terrain background mode, matching the NXi TER softkey cycle: Off (plain
// black map), Topo (absolute topographic shading), Rel (altitude-relative
// proximity coloring -- red/yellow vs. ownship altitude).
enum class TerrainDisplay { Off, Topo, Rel };

// Airway overlay state, matching the NXi AWY softkey cycle (Pilot's Guide,
// Displaying/removing airways): AWY Off -> AWY On (all) -> AWY LO -> AWY HI.
enum class AirwayDisplay { Off, All, Low, High };

// Map declutter level, matching the NXi Detail softkey cycle (Pilot's Guide:
// "Detail All, Detail 3, Detail 2 and Detail 1"). Detail3 removes land data,
// Detail2 also removes airspace/airways, Detail1 leaves only the active
// flight plan (and ownship).
enum class MapDetail { All, Detail3, Detail2, Detail1 };

struct MapViewStyle {
  bool showRangeRings = true;
  bool showFlightPlan = true;
  bool showFeatures = true;
  bool showAirspace = true;
  // Terrain background (requires MapData::terrain). When Off, the map keeps
  // its plain dark background.
  TerrainDisplay terrain = TerrainDisplay::Off;
  // Airway overlay (AWY softkey state).
  AirwayDisplay airways = AirwayDisplay::Off;
  // Traffic overlay (Traffic softkey).
  bool showTraffic = false;
  // Max map range (NM) at which traffic symbols are drawn (Map Setup "Traffic
  // Symbols" range). Declutters when zoomed out past this step.
  float trafficSymbolsRangeNm = kTrafficMapRangeDefaultNm;
  // Relative-altitude tags beside traffic symbols (Map Setup "Traffic Labels").
  bool showTrafficLabels = true;
  float trafficLabelsRangeNm = kTrafficMapRangeDefaultNm;
  // NEXRAD / precipitation overlay (NEXRAD softkey in MAP OPT).
  bool showWeather = false;
  // Max map range (NM) at which the NEXRAD overlay is drawn (Map Setup
  // "NEXRAD Data" range). Declutters when zoomed out past this step.
  float nexradRangeNm = kNexradMapRangeDefaultNm;
  // Land data: rivers/lakes, roads, cities, borders (Map Setup "Land" group).
  bool showLand = true;
  // Runway diagrams at airports once zoomed below ~5 NM range.
  bool showRunways = true;
  // Taxiway / apron pavement (SafeTaxi-style diagram), drawn under the runway
  // diagrams once zoomed below ~5 NM range (same as the runways).
  bool showTaxiways = true;
  // Obstacle symbols (requires the optional FAA DOF data to be loaded).
  bool showObstacles = true;
  // Max map range (NM) at which obstacle symbols are drawn (Map Setup
  // "Obstacle Data" range). Declutters when zoomed out past this step.
  float obstacleRangeNm = 10.0f;
  // Map Setup "Map" group items, on for the MFD navigation map by default:
  // ground-track lookahead line, wind arrow, and the fuel endurance rings.
  bool showTrackVector = false;
  bool showWindVector = false;
  bool showFuelRing = false;
  // Border, range label, and orientation annunciation. The inset map shows
  // these; a full-screen MFD page may supply its own chrome instead.
  bool showChrome = true;
  // Boxed orientation label + range readout without the border/background
  // chrome (the embedded WPT/NRST page maps, Fig 5-26).
  bool showOrientationLabel = false;
  // White compass arrow below the orientation label (Fig 5-26).
  bool showNorthArrow = false;
  // Identifier labels next to nav features and flight-plan waypoints. Kept
  // separate from showChrome so embedded windows (WPT/NRST airport maps) can
  // drop the chrome but keep the idents.
  bool showLabels = true;
  // Intersections / VFR waypoints. The small embedded airport maps turn these
  // off (like the G1000's detail declutter) -- at close range the fix class is
  // dense enough to bury the airport the window is meant to show.
  bool showFixes = true;
  // Airport declutter by Garmin size class (Map Setup "Aviation" group): each
  // size shows only at/below its own max map range. The renderer classifies an
  // airport from its longest runway (>= 8100 ft Large, >= 5000 ft or towered
  // Medium, else Small). Defaults mirror MfdController's Aviation defaults so
  // callers that don't drive Map Settings (the PFD inset) still show airports.
  bool showLargeAirports = true;
  bool showMediumAirports = true;
  bool showSmallAirports = true;
  float largeAirportRangeNm = kAirportMaxRangeNm;
  float mediumAirportRangeNm = kMediumAirportMaxRangeNm;
  float smallAirportRangeNm = kSmallAirportMaxRangeNm;
  // Max map range (NM) for TER Topo/Rel shading (Map Setup "Terrain Data"
  // range). Declutters when zoomed out past this step on the real NXi.
  float terrainMaxRangeNm = kMapRangeMaxNm;
  float labelFontWt = 14.0f;
};

// Rectangle and display options for a single map instance. Any screen region
// (PFD inset box, MFD full map, traffic overlay) passes the same struct.
struct MapViewConfig {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
  MapOrientation orientation = MapOrientation::TrackUp;
  // Per-view range, in NM. When > 0 it overrides MapData::rangeNm so several
  // map instances (PFD inset vs. MFD MAP page) can show the same nav data at
  // independent zooms. Defaults to 0 = follow the shared MapData range.
  float rangeNm = 0.0f;
  // Animated zoom scale, in NM. When > 0 it drives the on-screen map scale
  // (pixels-per-NM) while `rangeNm` stays the selected ladder step that labels
  // the range readout and gates symbol declutter. This lets the map glide
  // between ladder steps (Working Title G1000 NXi smooth zoom) without the
  // symbol set or readout flickering mid-animation. Defaults to 0 = no
  // animation (the scale follows `rangeNm`).
  float displayRangeNm = 0.0f;
  // Optional map center override (e.g. the WPT/NRST airport maps center on the
  // selected airport rather than ownship). When false, centers on ownship.
  bool hasCenterOverride = false;
  double centerLat = 0.0;
  double centerLon = 0.0;
  // Optional procedure preview polyline (PROC menu on the FPL page).
  const std::vector<MapLeg>* procedurePreview = nullptr;
  // Obstacle selected by the map pointer: skip its always-on MSL label (the
  // detailed MSL/AGL tag is drawn at the pointer instead).
  const MapObstacle* selectedObstacle = nullptr;
  MapViewStyle style;
};

// Detail softkey label for a declutter level, verbatim from the Pilot's
// Guide ("Detail All, Detail 3, Detail 2 and Detail 1"). Shared by the PFD
// Map/HSI submenu and the MFD root bar.
inline const char* mapDetailLabel(MapDetail d) {
  switch (d) {
    case MapDetail::Detail3:
      return "Detail 3";
    case MapDetail::Detail2:
      return "Detail 2";
    case MapDetail::Detail1:
      return "Detail 1";
    case MapDetail::All:
      break;
  }
  return "Detail All";
}

// Advances a Detail level one step in the softkey cycle.
inline MapDetail nextMapDetail(MapDetail d) {
  switch (d) {
    case MapDetail::All:
      return MapDetail::Detail3;
    case MapDetail::Detail3:
      return MapDetail::Detail2;
    case MapDetail::Detail2:
      return MapDetail::Detail1;
    case MapDetail::Detail1:
      break;
  }
  return MapDetail::All;
}

// Applies a Detail (declutter) level to a style, per the Pilot's Guide:
// Detail 3 removes land data, Detail 2 also removes airspace (SUA) and
// airways, Detail 1 leaves only the active flight plan.
inline void applyMapDetail(MapViewStyle& style, MapDetail detail) {
  const int level = static_cast<int>(detail);
  if (level >= static_cast<int>(MapDetail::Detail3)) style.showLand = false;
  if (level >= static_cast<int>(MapDetail::Detail2)) {
    style.showAirspace = false;
    style.airways = AirwayDisplay::Off;
    style.showObstacles = false;  // Table 5-4: obstacles declutter at Detail 2
  }
  if (level >= static_cast<int>(MapDetail::Detail1)) {
    style.showFeatures = false;
    style.showFixes = false;
  }
}

// Shared moving-map renderer. PFD inset and future MFD MAP page both call
// MapView::render with different viewport rects and styles.
class MapView {
 public:
  static void render(Renderer& r, const MapData& map, const FlightData& flight,
                     const MapViewConfig& config, float displayH);
};

}  // namespace avionics
