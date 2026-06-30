#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "avionics/Charts.h"
#include "avionics/Checklist.h"
#include "avionics/FlightData.h"
#include "avionics/FlightPlanCatalog.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/FmsWaypointEntry.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenuTypes.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/SimBrief.h"
#include "avionics/SimBriefOfpSupport.h"
#include "avionics/StationWeather.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/MapView.h"

namespace avionics {

// Persisted-preferences view of this controller (avionics/PersistentState.h).
struct MfdPersistentState;
class DataSource;

// The MFD page groups (G1000 Pilot's Guide for Cessna Nav III, Section 1.4).
// MAP/WPT/AUX/NRST and Checklist are selected from the bottom softkey bar
// (Checklist) or the large FMS knob (page groups); the FPL group is entered with the FPL
// bezel key, as on the real unit. Each checklist in the loaded file is a page
// within the Checklist group (stepped like any other group's pages).
enum class MfdPageGroup { Map, Waypoint, Aux, Nearest, FlightPlan, Checklist };

// Pages within each group, in real on-unit order. Optional equipment pages
// (Stormscope, weather data link, XM, telephone, video) and pages that need
// data this suite has no source for (user waypoints, ARTCC/FSS frequencies,
// System Setup editing) are omitted.
enum class MfdPage {
  // MAP group.
  NavigationMap,
  TrafficMap,
  WeatherRadar,
  // WPT group.
  AirportInformation,
  IntersectionInformation,
  NdbInformation,
  VorInformation,
  // AUX group.
  TripPlanning,
  Utility,
  GpsStatus,
  SystemSetup,
  SystemStatus,
  SimBrief,
  // NRST group.
  NearestAirports,
  NearestIntersections,
  NearestNdb,
  NearestVor,
  NearestFrequencies,
  NearestAirspaces,
  // FPL group.
  ActiveFlightPlan,
  FlightPlanCatalog,
};

// Sub-views of the WPT - Airport Information page, selected from that page's
// softkey bar (NXi trainer apt_054..058). The page reuses one MfdPage value
// (AirportInformation); this picks which information panel it shows. Airport is
// the default (the Info softkey); DP/STAR/APR preview the first published
// procedure of that category for the selected airport; Weather shows the
// METAR/TAF panel (WX softkey).
enum class WptInfoView { Airport, Departure, Arrival, Approach, Weather };

// Pre-resolved data for a WPT - Airport Information procedure sub-page (DP /
// STAR / APR). The page pre-selects the first published procedure of the
// category for the selected airport and previews it, read-only, mirroring the
// PROC loading window. `available` is false when the airport publishes no
// procedure of that category (the page then shows the dashed empty state).
struct WptProcedureInfo {
  bool available = false;
  std::string name;        // CSHEL6 / JOSFF5 / "ILS 05"
  std::string transition;  // LAL / PIE / VECTORS
  std::string runway;      // RW05 / ALL
  // Approach primary nav radio (ILS/LOC/VOR/NDB approaches only); 0 when none.
  float primaryFreqMhz = 0.0f;
  std::string primaryIdent;
  bool primaryIsNdb = false;
  std::vector<MapLeg> legs;  // sequenced procedure legs (empty when none)
};

// Map Settings window groups (Navigation Map -> MENU -> Map Settings, Pilot's
// Guide Fig. 5-7). The real unit also lists Airways and VSD, but greys them
// out (no data source / not modeled), so only the selectable groups are here,
// in on-unit order (WT NXi MFDMapSettings GROUP_ITEMS).
enum class MapSettingsGroup { Map, Weather, Traffic, Aviation, Airspace, Land };

// Every control shown in the Map Settings window, kept in one contiguous range
// so the controller can key its value arrays by the id. The Map group is taken
// verbatim from Fig. 5-7; the other groups from the WT NXi setting groups.
// Some controls (Orientation, Terrain Display, NEXRAD Data, Traffic) share
// state with the existing softkey options so the window and the softkeys stay
// in sync, like the real unit.
enum class MapSetting {
  // Map group.
  Orientation,
  NorthUpAboveOn,
  NorthUpAboveRange,
  TerrainMode,
  TerrainRange,
  TopoScaleOn,
  ObstacleOn,
  ObstacleRange,
  AutoZoomOn,
  AutoZoomMax,
  MaxLookFwd,
  MinLookFwd,
  TimeOut,
  TrackVectorOn,
  TrackVectorTime,
  AltArcOn,
  WindVectorOn,
  FuelRangeOn,
  FuelRangeRsv,
  FieldOfViewOn,
  // Weather group.
  NexradOn,
  NexradRange,
  // Traffic group.
  TrafficOn,
  TrafficMode,
  TrafficSymbolsRange,
  TrafficLabelsOn,
  TrafficLabelsRange,
  // Aviation group.
  LargeAirportOn,
  LargeAirportRange,
  MediumAirportOn,
  MediumAirportRange,
  SmallAirportOn,
  SmallAirportRange,
  IntOn,
  IntRange,
  NdbOn,
  NdbRange,
  VorOn,
  VorRange,
  // Airspace group.
  ClassBOn,
  ClassBRange,
  ClassCOn,
  ClassCRange,
  ClassDOn,
  ClassDRange,
  RestrictedOn,
  RestrictedRange,
  MoaOn,
  MoaRange,
  OtherOn,
  OtherRange,
  // Land group.
  UserWaypointOn,
  UserWaypointRange,
  Count,
};

// Airborne color weather-radar (GWX) controls for the MAP - Weather Radar page
// (G1000 NXi Pilot's Guide, Hazard Avoidance - Airborne Color Weather Radar).
// The radar mode annunciation in the page's upper-left reads from RadarMode;
// the scan toggle (Horizon / Vertical Softkeys) from RadarScan.
enum class RadarMode { Standby, Weather, Ground };
enum class RadarScan { Horizontal, Vertical };
// Sector-scan width about the bearing line; Full is the 90-degree scan.
enum class RadarSector { Full, Sixty, Forty, Twenty };

// G1000 NXi chart page softkey levels (Pilot's Guide §8.3).
enum class ChartsMenu { Selection, ChartOpt };
enum class ChartsField { Airport, Approach };
enum class ChartsViewMode { All, Header, Plan, Profile, Minimums };
enum class ChartsCategoryFilter {
  All,
  Departure,
  Arrival,
  Approach,
  Airport,
};

// Accumulated flight-session statistics for the AUX - Utility page's Timers
// and Trip Statistics boxes (G1000 Pilot's Guide for Cessna Nav III,
// Section 5.10). Accumulated since power-on from the live data, the way the
// real unit runs its timers and odometers (this suite has no cross-session
// persistence source, so totals are per power cycle).
struct FlightSessionStats {
  // Generic up timer: counts from power-on (the page's GENERIC row).
  double genericTimerSec = 0.0;
  // Flight timer: starts at the first in-air detection and runs from there
  // (the unit's default "In-Air" flight-timer criterion).
  bool airborneSeen = false;
  double flightTimerSec = 0.0;
  // UTC clock captured at the first in-air detection (DEPARTURE TIME row);
  // hour is -1 until liftoff, shown dashed.
  int departureHour = -1;
  int departureMinute = 0;
  // Distance traveled since power-on; ODOMETER and TRIP ODOMETER both read
  // this session total (no persisted lifetime odometer source).
  double odometerNm = 0.0;
  // Average groundspeed counts only time spent moving, like the real trip
  // average that excludes stationary time.
  double movingTimeSec = 0.0;
  float maxGroundSpeedKts = 0.0f;
};

// Discrete antenna-tilt and bearing-line step sizes (degrees) and their limits
// (GWX 70: manual tilt +/-15 deg; the horizontal scan spans +/-45 deg).
inline constexpr float kRadarTiltStepDeg = 0.25f;
inline constexpr float kRadarTiltLimitDeg = 15.0f;
inline constexpr float kRadarBearingStepDeg = 1.0f;
inline constexpr float kRadarBearingLimitDeg = 45.0f;
// One antenna look (sweep across the sector), in seconds (GWX: 12 looks/min).
inline constexpr float kRadarSweepSeconds = 5.0f;

// Owns the interactive state of the MFD's softkey bar: the selected page group
// and the moving-map range. It mirrors SoftkeyController's shape (label /
// pressLevel / keyActive read by the renderer, pressKey / update driven by
// the engine) so the shared softkey-bar drawing style applies to both displays.
//
// The MFD keeps its own range independent of the PFD inset map, so zooming one
// display does not affect the other.
class MfdController {
 public:
  // The MFD reuses the 12-cell softkey bar layout shared with the PFD.
  static constexpr int kSoftkeyCount = 12;

  // Press-flash decay, in seconds (matches the PFD softkey feel).
  static constexpr float kPressFlashSeconds = 0.18f;

  // How long the page-select popup stays up after a group/page change before
  // auto-closing (the WT NXi page-select dialog closes after 3 s idle).
  static constexpr float kPageSelectSeconds = 3.0f;

  // Open/close slide+fade duration for the pop-up windows (Direct-To, Page
  // Menu), matching the PFD pop-ups (SoftkeyController::kWindowAnimSeconds) so
  // every menu animates in with the same feel.
  static constexpr float kWindowAnimSeconds = 0.22f;

  MfdController();

  // Advance the key-press flash animations, the ~1 Hz cursor-blink phase, and
  // the flight-session timers/odometers (from the live data snapshot).
  void update(double dtSeconds, const FlightData& data);

  // Session timers and trip statistics for the AUX - Utility page.
  const FlightSessionStats& flightStats() const { return flightStats_; }

  // ~1 Hz blink phase (true for the first half of each second) used to pulse
  // highlight-select cursor fields (WT .highlight-select @keyframes pulse).
  bool blinkOn() const { return blinkOn_; }
  // Map Pointer flash (Garmin map-pointer-flash): inverted colors for the last
  // ~10% of each one-second cycle.
  bool mapPointerFlashInverted() const {
    return std::fmod(blinkSeconds_, 1.0) >= 0.9;
  }

  // Apply a press of physical softkey `key` (0..kSoftkeyCount-1, the hardware
  // keys below the display; the on-screen bar is labels only, like the real
  // unit). Returns true if the key currently has a function.
  bool pressKey(int key);

  MfdPageGroup pageGroup() const { return pageGroup_; }
  // True when this MFD page group owns ENT/CLR/FMS-push on the MFD GDU even if
  // a PFD pop-up would otherwise claim GCU FMS input (Active Flight Plan,
  // Checklist, and the WPT - Airport Information chart view, which uses ENT to
  // commit the highlighted chart and the FMS knob to browse the chart list).
  bool ownsLocalFmsInput() const {
    return pageGroup_ == MfdPageGroup::FlightPlan ||
           pageGroup_ == MfdPageGroup::Checklist || chartViewActive_;
  }

  // Number of pages in a group and the index of the page currently selected
  // within the active group (the FMS rocker / repeated group-softkey presses
  // step it, like the small FMS knob). The MAP group drops its Weather Radar
  // page when the airframe is not equipped (see setWeatherRadarAvailable), so
  // this depends on instance state and is not static.
  int pageCount(MfdPageGroup group) const;
  int pageIndex() const;

  // Whether the airframe carries an airborne weather radar. When false, the
  // dedicated MAP - Weather Radar page is removed from the page rotation (the
  // real unit only lists it on radar-equipped installations); the NEXRAD map
  // overlay is unaffected. The shell sets this from the sim (the X-Plane plugin
  // probes the radar return texture). Defaults to true so the standalone shell
  // and unit tests keep the page.
  void setWeatherRadarAvailable(bool available);
  bool weatherRadarAvailable() const { return weatherRadarAvailable_; }
  // The specific page on screen, resolved from the group + page index.
  MfdPage page() const;

  // Current map range, in NM (a discrete G1000-style range ladder). This is
  // the selected step: it labels the range readout and gates symbol declutter.
  float rangeNm() const;

  // Snap the range ladder to the nearest step for `rangeNm` (sim EFIS sync).
  void setRangeFromNm(float rangeNm);

  // Step the navigation-map range ladder (+1 zoom out, -1 zoom in).
  bool stepMapRange(int direction);

  // Animated zoom scale, in NM, easing toward rangeNm() each update(). Passed
  // to the map renderer as the on-screen scale so zooming glides between ladder
  // steps (Working Title G1000 NXi smooth zoom) rather than snapping.
  float displayRangeNm() const { return displayRangeNm_; }

  // Half the map viewport diagonal in NM (corner reach from the view center).
  // Used by land-data queries so GSHHG lon-band overlap matches MapView.
  float mapViewHalfExtentNm() const;

  // Navigation Map display options, set from the Map Opt softkey submenu and
  // the Detail (declutter) softkey, mirroring the NXi MFD softkey map.
  TerrainDisplay terrainDisplay() const { return terrain_; }
  AirwayDisplay airwayDisplay() const { return airways_; }
  bool showTraffic() const { return showTraffic_; }
  bool showWeather() const { return showWeather_; }
  MapDetail mapDetail() const { return detail_; }
  void setMapDetail(MapDetail detail);

  // MAP page orientation (TRK softkey toggles north-up vs track-up).
  MapOrientation mapOrientation() const { return mapOrientation_; }

  // ---- MAP - Weather Radar page (airborne GWX radar) ----
  // Read by the page renderer to draw the mode annunciation, scan geometry,
  // bearing line, antenna-tilt/gain/sector readouts, and feature status, and
  // by the shell to drive X-Plane's EFIS weather-radar datarefs.
  RadarMode radarMode() const { return radarMode_; }
  RadarScan radarScan() const { return radarScan_; }
  bool radarBearingLineOn() const { return radarBearingLineOn_; }
  float radarBearingDeg() const { return radarBearingDeg_; }
  float radarTiltDeg() const { return radarTiltDeg_; }
  bool radarGainCalibrated() const { return radarGainCalibrated_; }
  // Manual-gain offset (-1..+1) shown as the movable bar relative to the
  // calibrated reference; 0 at the calibrated position.
  float radarGainManual() const { return radarGainManual_; }
  RadarSector radarSector() const { return radarSector_; }
  bool radarStab() const { return radarStab_; }
  bool radarAct() const { return radarAct_; }
  // Antenna sweep position, 0..1 across one look, for the animated scan line.
  float radarSweepPhase() const { return static_cast<float>(radarSweepPhase_); }
  bool radarCursorOn() const { return radarCursorOn_; }

  // Navigation Map pointer / pan mode (Pilot's Guide, Map Panning): push the
  // RANGE joystick on the MAP page to place a pan cursor; moving the joystick
  // slides the cursor across the map. The map stays put until the cursor
  // reaches the inner edge of the view, then the map scrolls to keep it on
  // screen. ENT on a highlighted waypoint opens its Waypoint Information page;
  // Direct-To opens on the waypoint under the pointer.
  bool mapPointerActive() const { return mapPointerActive_; }
  double mapPointerLat() const { return mapPointerLat_; }
  double mapPointerLon() const { return mapPointerLon_; }
  // Map view center while the pointer is active (may differ from the pointer
  // geo until the cursor reaches the edge of its free-move zone).
  double mapPanViewCenterLat() const { return mapPanViewCenterLat_; }
  double mapPanViewCenterLon() const { return mapPanViewCenterLon_; }
  // Called each frame from the MAP page so edge-scroll can use the live
  // viewport geometry and orientation.
  void setMapViewport(float x, float y, float w, float h, float displayH);
  void mapPointerSyncScroll(const FlightData& flight);
  // Push the panned map view center (not the pointer geo) into the data
  // source so land vectors and nav features load for what is on screen.
  void applyMapPanToDataSource(DataSource& source, const FlightData& flight);
  // Load land/nav data around the Direct-To target for the inset chart.
  void applyDirectToInsetToDataSource(DataSource& source, const MapData& map);
  // The map feature under the pan pointer (within a small, range-scaled snap
  // radius), or nullptr. The page highlights it and fills the Map Pointer
  // information box with its ident, like the real unit selecting a waypoint as
  // the pointer passes over it.
  const MapFeature* mapPointerFeature() const;
  // Obstacle under the pan pointer (same snap envelope as mapPointerFeature).
  const MapObstacle* mapPointerObstacle() const;
  // Airspaces whose lateral boundary contains the pan pointer (point-in-polygon
  // over each ring). The page highlights each boundary and lists its name,
  // class, and vertical limits in a box beside the cursor, like the real unit
  // selecting an airspace as the pointer passes over it (Pilot's Guide, Map
  // Panning). Capped at a few entries so a stack of overlapping SUA stays
  // readable.
  std::vector<const MapAirspace*> mapPointerAirspaces() const;

  // WPT facility ident entry (Pilot's Guide, Waypoint Pages). ENT on a
  // resolved ident selects that waypoint for the page.
  bool wptEntryActive() const { return wptEntry_.active; }
  std::string wptEntryIdent() const { return wptEntry_.ident(); }
  int wptEntryCursor() const { return wptEntry_.pos; }
  int wptEntryTypedCount() const { return wptEntry_.typedCount(); }
  bool wptEntryNotFound() const { return wptEntry_.notFound; }
  bool wptHasSelection() const { return wptHasSelection_; }
  const MapFeature& wptSelectedFeature() const { return wptFeature_; }

  // Which information panel the WPT - Airport Information page shows (Airport /
  // DP / STAR / APR / Weather). Set by that page's softkey bar.
  WptInfoView wptInfoView() const { return wptInfoView_; }
  // First published procedure of `type` for `airport`, expanded for preview on
  // the WPT DP/STAR/APR sub-pages (read-only, "first available" selection).
  WptProcedureInfo wptProcedureInfo(const MapFeature& airport,
                                    ProcedureType type) const;

  // NRST nearest-list cursor (Pilot's Guide, Nearest pages).
  bool nrstCursorOn() const { return nrstCursorOn_; }
  int nrstSelected() const { return nrstSelected_; }

  // Seconds the page-select popup (group tabs + page list, bottom right)
  // remains visible; 0 = hidden. Refreshed by any group/page change and run
  // down by update(), like the real popup's idle auto-close.
  float pageSelectSecondsLeft() const { return pageSelectSec_; }

  // ---- checklists (Checklist page group) ----
  // Cache the latest loaded checklists each frame so item navigation tracks the
  // file (the data is owned by the DataSource). Resizes/clamps the interactive
  // checked state and selection when the file's shape changes.
  void syncChecklist(const ChecklistData& data);
  // Flat index of the displayed checklist, and the highlighted item within it.
  // The cursor ranges [0, itemCount]; itemCount selects the "go to next
  // checklist" prompt below the last item.
  int checklistIndex() const { return checklistIndex_; }
  int checklistCursor() const { return cursorItem_; }
  int checklistCount() const;
  bool checklistItemChecked(int checklistIndex, int itemIndex) const;

  // ---- FMS waypoint identifier entry ----
  // The character-entry state for the FPL insert window, the Direct-To window,
  // and the Waypoint pages is the shared avionics::FmsWaypointEntry (also used
  // by the PFD's Direct-To window), so entry behaves identically everywhere.

  // Longest identifier enterable (covers ICAO airports, navaids, fixes).
  static constexpr int kFplEntryMaxChars = FmsWaypointEntry::kMaxChars;

  // ---- Active Flight Plan page (FPL group) ----
  // The G1000 flight-plan editing flow (Pilot's Guide for Cessna Nav III,
  // Section 5.6): push the FMS knob to turn the cursor on, large knob selects
  // a leg row, small knob opens the Waypoint Information window and spells an
  // identifier (ENT inserts it before the selected row), CLR on a row opens
  // the "Remove <wpt>?" confirmation, and MENU offers Delete Flight Plan.

  // Confirmation window opened by CLR on a waypoint row, CLR on a loaded
  // SID/STAR/approach, or a page-menu Remove/Delete option. ENT executes the
  // highlighted OK/CANCEL choice.
  enum class FplConfirm {
    None,
    RemoveWaypoint,
    RemoveDeparture,
    RemoveArrival,
    RemoveApproach,
    RemoveAirway,
    DeleteFlightPlan,
  };

  // Editable field within a flight-plan row. The large FMS knob steps the cursor
  // through the identifier and the VNAV altitude-constraint column (Pilot's
  // Guide, Section 6 "Vertical Navigation": altitude constraints are entered in
  // the FPL page ALT column).
  enum class FplCursorCol { Ident, Altitude };

  // Cache the latest flight plan + ownship each frame (called by the engine).
  // External plan changes (a SimBrief fetch, an .fms reload) are adopted; the
  // locally edited plan stays authoritative while the shell applies it.
  // `activeWaypoint` is the FMS active leg's TO ident (Direct-To default).
  // `navDirectTo` mirrors the PFD Navigation Status Box Direct-To display.
  void syncFlightPlan(const MapData& map, const std::string& activeWaypoint,
                      bool navDirectTo);
  // Ident lookups for waypoint entry (the shell wires its nav database in;
  // without one, entry falls back to the nearby map features).
  void setNavFeatureSource(const NavFeatureSource* source) {
    navSource_ = source;
  }
  // Datalink weather lookup for the WPT - Weather Information page (the shell
  // wires its station-weather source in; without one the page shows dashes).
  void setStationWeatherSource(const StationWeatherSource* source) {
    weatherSource_ = source;
  }
  // Latest weather for an airport ICAO, or nullopt when no source is wired or
  // the source has no report for the station (the page then dashes the fields).
  std::optional<StationWeather> stationWeather(const std::string& icao) const;
  // Edited-plan latch for the shell: true once after each edit, copying the
  // new plan out so the shell can push it to the data sources.
  bool consumeFlightPlanEdit(std::vector<MapLeg>& out);
  void fplPublishEdit();
  void stripCourseReversalHoldAtFix(const std::string& fixId);

  void replaceFlightPlanFromExternal(const std::vector<MapLeg>& plan);
  PersistedFlightPlan persistedFlightPlanSnapshot() const;
  PersistedDirectTo persistedDirectToSnapshot() const;
  void restorePersistedFlightPlan(const PersistedFlightPlan& saved);

  // ---- Flight Plan Catalog (FPL group, 2nd page) ----
  // Stored flight plans the pilot previews and activates. An imported SimBrief/
  // Navigraph OFP is stored here (storeFlightPlanInCatalog) instead of being
  // auto-loaded; only Activate loads a stored plan into the active route.
  // Confirmation window shown for the destructive / route-changing catalog
  // actions (matching the real unit's "activate stored flight plan?" etc.).
  enum class CatalogConfirm { None, Activate, InvertActivate, Delete, DeleteAll };

  const FlightPlanCatalog& flightPlanCatalog() const { return catalog_; }
  // Highlighted catalog slot (clamped to [0, size)), and whether the list
  // cursor is on (FMS knob pushed) so the catalog actions apply to a slot.
  int catalogSelected() const { return catalogSelected_; }
  bool catalogCursorOn() const { return catalogCursorOn_; }
  CatalogConfirm catalogConfirm() const { return catalogConfirm_; }
  bool catalogConfirmOk() const { return catalogConfirmOk_; }

  // Persisted-catalog round trip (the shell saves/restores it in AppSettings).
  std::vector<PersistedFlightPlan> flightPlanCatalogSnapshot() const {
    return catalog_.plans();
  }
  void restoreFlightPlanCatalog(const std::vector<PersistedFlightPlan>& plans);
  // Edited-catalog latch for the shell: true once after any catalog change so
  // the shell re-persists the catalog (import, activate-copy, delete, ...).
  bool consumeCatalogDirty();

  // Store a route (e.g. a fetched SimBrief OFP) as a new catalog entry WITHOUT
  // touching the active flight plan or the map. Returns the new slot index, or
  // -1 when the catalog is full.
  int storeFlightPlanInCatalog(const std::vector<MapLeg>& legs);
  int storeFlightPlanFromSimBriefImport(const SimBriefOfpImport& imp);

  // After a catalog plan is activated, yields the full PersistedFlightPlan once
  // (including SID/STAR/approach grouping) so the shell can mirror the procedure
  // metadata onto the peer GDU. The route legs already propagate via the route
  // override, but the procedure block ranges + headers are per-controller state
  // and would otherwise be lost on the PFD (it would show the SID/STAR fixes as
  // plain Enroute legs). Returns false when no activation is pending.
  bool consumeActivatedFlightPlan(PersistedFlightPlan& out);

  // ---- catalog actions (softkeys / page menu; also exercised by tests) ----
  // Add a new empty stored plan and select it.
  void catalogCreateNew();
  // Load the selected stored plan into the active flight plan and publish it so
  // the shell pushes it to the sim/map. Returns false when nothing to activate.
  bool catalogActivateSelected();
  // Invert (reverse) the selected stored plan, then activate it.
  bool catalogInvertActivateSelected();
  // Copy the selected stored plan into a new slot. Returns its index, or -1.
  int catalogCopySelected();
  // Delete the selected stored plan. Returns false when nothing to delete.
  bool catalogDeleteSelected();
  // Delete every stored plan.
  void catalogDeleteAll();

  void setPersistedLoadedApproach(const PersistedLoadedApproach& saved);
  FlightPlanApproachState flightPlanApproachState() const;
  void applyFlightPlanApproachState(const FlightPlanApproachState& state);
  FlightPlanTerminalProcedureState flightPlanDepartureState() const;
  void applyFlightPlanDepartureState(
      const FlightPlanTerminalProcedureState& state);
  FlightPlanTerminalProcedureState flightPlanArrivalState() const;
  void applyFlightPlanArrivalState(const FlightPlanTerminalProcedureState& state);
  // Copy the peer GDU's displayed plan so the PFD FPL window and MFD FPL page
  // always show the same route (called from AvionicsEngine::syncFlightPlanPeer).
  void adoptFlightPlanFromPeer(
      const std::vector<MapLeg>& legs, bool destinationFilled,
      const FlightPlanApproachState& approach,
      const FlightPlanTerminalProcedureState& departure = {},
      const FlightPlanTerminalProcedureState& arrival = {},
      bool peerLocalDraft = false);
  // Mirror the peer GDU's FPL list scroll/selection (PFD window vs MFD page).
  void adoptFlightPlanCursorFromPeer(int cursorRow, bool followsActive);

  // Infer approach grouping from procedure-tagged legs when metadata is missing.
  void fplEnsureApproachInferred();

  struct FplEffectiveApproach {
    int start = 0;
    int count = 0;
    bool loaded() const { return count > 0; }
  };
  FplEffectiveApproach fplEffectiveApproach() const;

  // ---- read by the FPL page renderer ----
  // The plan as the page shows it (mirrors the map plan plus pending edits).
  const std::vector<MapLeg>& fplLegs() const { return fplLegs_; }
  bool fplHasLoadedApproach() const { return fplApproachLegCount_ > 0; }
  bool fplHasLoadedDeparture() const {
    return !fplLoadedDeparture_.name.empty() || fplDepartureLegCount_ > 0;
  }
  bool fplHasLoadedArrival() const {
    return !fplLoadedArrival_.name.empty() || fplArrivalLegCount_ > 0;
  }
  bool fplDestinationFilled() const { return fplDestinationFilled_; }
  // FPL airway display: when collapsed, each loaded-airway segment shows only
  // its "Airway - <name>.<exit>" header + the exit fix; expanded lists every
  // intermediate fix (page menu "Collapse Airways"/"Expand Airways").
  bool fplAirwaysCollapsed() const { return fplAirwaysCollapsed_; }
  // True when the plan carries any loaded-airway leg (gates the collapse toggle
  // and the airway-grouped FPL list display).
  bool fplHasAirwayLegs() const;
  // FPL list layout: destination is the airport (plus STAR/approach when loaded).
  bool fplDestinationFilledForLayout() const;
  bool fplLocalDraft() const { return fplLocalDraft_; }
  int fplDepartureLegStart() const { return fplDepartureLegStart_; }
  int fplDepartureLegCount() const { return fplDepartureLegCount_; }
  int fplArrivalLegStart() const { return fplArrivalLegStart_; }
  int fplArrivalLegCount() const { return fplArrivalLegCount_; }
  std::string fplDepartureAirportIcao() const {
    if (!persistedDepartureRestore_.airportIcao.empty()) {
      return persistedDepartureRestore_.airportIcao;
    }
    return fplLoadedDeparture_.name.empty() || fplLegs_.empty()
               ? std::string()
               : fplLegs_.front().id;
  }
  std::string fplDepartureHeaderLabel() const { return fplDepartureHeaderLabel_; }
  std::string fplArrivalAirportIcao() const {
    if (!persistedArrivalRestore_.airportIcao.empty()) {
      return persistedArrivalRestore_.airportIcao;
    }
    return fplLoadedArrival_.name.empty() || fplLegs_.empty()
               ? std::string()
               : fplLegs_.back().id;
  }
  std::string fplArrivalHeaderLabel() const { return fplArrivalHeaderLabel_; }
  int fplApproachLegStart() const { return fplApproachLegStart_; }
  int fplApproachLegCount() const { return fplApproachLegCount_; }
  std::string fplApproachAirportIcao() const;
  std::string fplApproachHeaderLabel() const { return fplApproachHeaderLabel_; }
  std::string fplApproachTransition() const {
    return fplLoadedApproach_.transition;
  }
  // Selection cursor over the leg list. The cursor ranges [0, legCount]:
  // legCount selects the blank slot after the last waypoint (append).
  bool fplCursorOn() const { return fplCursorOn_; }
  int fplCursorRow() const { return fplCursorRow_; }
  // Leg index (into fplLegs()) under the list cursor, or -1 when the cursor is
  // on a blank/template/separator row. Lets the FPL page pan its route preview
  // to the highlighted fix.
  int fplCursorLegIndexPublic() const { return fplCursorLegIndex(); }
  // True while the list cursor tracks the active nav leg (false after scrolling).
  bool fplListCursorFollowsActive() const { return fplListCursorFollowsActive_; }
  // Waypoint Information entry window state. The displayed ident is the typed
  // prefix completed by the database spell-ahead match (no padding).
  bool fplEntryActive() const { return fplEntry_.active; }
  std::string fplEntryIdent() const { return fplEntry_.ident(); }
  int fplEntryCursor() const { return fplEntry_.pos; }
  int fplEntryTypedCount() const { return fplEntry_.typedCount(); }
  bool fplEntryNotFound() const { return fplEntry_.notFound; }
  // The waypoint the current entry resolves to (valid when hasMatch).
  bool fplEntryHasMatch() const { return fplEntry_.hasMatch; }
  const MapFeature& fplEntryMatch() const { return fplEntry_.match; }
  // Which column the FPL cursor is on (drives the row highlight + which field
  // the small knob edits).
  FplCursorCol fplCursorCol() const { return fplCursorCol_; }
  // VNAV altitude-constraint entry window state (small knob on the ALT column).
  bool fplAltEntryActive() const { return fplAltEntry_.active; }
  int fplAltEntryRow() const { return fplAltEntry_.row; }
  const std::string& fplAltEntryDigits() const { return fplAltEntry_.digits; }
  int fplAltEntryCursor() const { return fplAltEntry_.pos; }
  // Remove / delete confirmation window.
  FplConfirm fplConfirm() const { return fplConfirm_; }
  bool fplConfirmOk() const { return fplConfirmOk_; }
  const std::string& fplRemoveIdent() const { return fplRemoveIdent_; }

  // ---- Direct-To window (Direct-To bezel key) ----
  // The GPS Direct-To window (Pilot's Guide, Section 5.5): the Direct-To key
  // opens it over any MFD page, pre-filled with the active (or FPL-selected)
  // waypoint. The FMS knob spells the destination ident; the first ENT
  // confirms the waypoint and arms ACTIVATE?, the second ENT engages the
  // direct course. CLR (or the knob push) cancels the window.
  bool directToWindowOpen() const { return dtoOpen_; }
  // Opens the Direct-To window with an explicit initial ident (empty for a
  // blank entry field). Used by dev screenshot states and tests.
  void openDirectToWindow(const std::string& initial = "");
  void openProcApproachLoading(
      const std::string& icao, const std::string& approachName,
      const std::string& transition,
      ProcLoadingList openList = ProcLoadingList::None);
  // 0..1 open progress for the slide+fade animation (1 fully open, eases back
  // to 0 on close so the window animates out as well as in).
  float directToWindowAnim() const { return dtoAnim_; }
  bool directToEntryActive() const { return dtoEntry_.active; }
  std::string directToIdent() const { return dtoEntry_.ident(); }
  int directToCursor() const { return dtoEntry_.pos; }
  int directToTypedCount() const { return dtoEntry_.typedCount(); }
  bool directToSelectAll() const { return dtoEntry_.selectAll; }
  bool directToNotFound() const { return dtoEntry_.notFound; }
  bool directToHasMatch() const { return dtoEntry_.hasMatch; }
  const MapFeature& directToMatch() const { return dtoEntry_.match; }
  // True once the waypoint is confirmed and the ACTIVATE? prompt is armed.
  bool directToArmed() const { return dtoArmed_; }
  // Activation latch for the shell: true once after ENT on ACTIVATE?, copying
  // out the target waypoint so the shell engages the direct course.
  bool consumeDirectToRequest(MapLeg& out, bool* flyHold = nullptr);
  // "4.0NM hold-icon BOSTN" Activate/Cancel prompt when Direct-To is pressed
  // on a published HOLD row in the flight plan (trainer FPL Direct-To hold).
  bool holdActivatePromptActive() const { return holdActivatePromptActive_; }
  bool holdActivatePromptActivateSelected() const {
    return holdActivatePromptActivate_;
  }
  const MapLeg& holdActivatePromptLeg() const { return holdActivatePromptLeg_; }
  // FPL Activate Leg: ENT on a highlighted waypoint row (Pilot's Guide 5.6).
  bool consumeActivateLegRequest(int& toLegIndex);
  // Published airways passing through `ident` (from the nav database); empty
  // when no source is wired or the fix lies on no airway.
  std::vector<std::string> airwaysThroughFix(const std::string& ident) const;

  // ---- Load Airway window (FPL page MENU -> Load Airway, Pilot's Guide,
  // Flight Planning - Load Airway) ----
  // Opened with the list cursor on an enroute fix that lies on at least one
  // published airway. The Airway field (small knob) picks the airway, the Exit
  // field (small/large knob) scrolls the fix chain to the exit waypoint, and
  // Load? inserts the expanded segment after the entry fix (tagged viaAirway so
  // the FPL list groups it under an "Airway - <name>.<exit>" header).
  enum class LoadAirwayField { Airway, Exit, Load };
  bool loadAirwayWindowOpen() const { return loadAirway_.open; }
  float loadAirwayWindowAnim() const { return loadAirway_.anim; }
  const std::string& loadAirwayEntryIdent() const {
    return loadAirway_.entryIdent;
  }
  std::string loadAirwayName() const;
  std::string loadAirwayExitIdent() const;
  LoadAirwayField loadAirwayField() const { return loadAirway_.field; }
  // Published airways through the entry fix, and the index of the chosen one,
  // for the Airway dropdown.
  const std::vector<std::string>& loadAirwayAirways() const {
    return loadAirway_.airways;
  }
  int loadAirwayAirwaySel() const { return loadAirway_.airwaySel; }
  // The fix chain shown in the scrolling list (entry fix first, then the legs
  // toward the far end of the airway). Empty when the selected airway resolves
  // to no chain.
  const std::vector<MapLeg>& loadAirwayFixes() const { return loadAirway_.fixes; }
  // List index of the highlighted exit fix (into loadAirwayFixes()).
  int loadAirwayExitSel() const { return loadAirway_.exitSel; }
  // DTK (deg) / cumulative DIS (NM) from the entry fix to the highlighted exit,
  // shown beside the list (dashes when unavailable).
  bool loadAirwayHasCourse() const { return loadAirway_.hasCourse; }
  float loadAirwayDtkDeg() const { return loadAirway_.dtkDeg; }
  float loadAirwayDisNm() const { return loadAirway_.disNm; }
  // True once a valid airway + exit is chosen so Load? can run.
  bool loadAirwayCanLoad() const;
  // Open the window for an explicit entry fix (used by tests / dev states).
  void openLoadAirwayWindow(const std::string& entryIdent);

  // GCU alphanumeric keypad during waypoint-ident entry (Direct-To / FPL / WPT).
  bool applyGcuEntryKey(char ch);

  // ---- SimBrief / Navigraph (AUX - SIMBRIEF page) ----
  // Latest sign-in + fetch status, published by the shell each frame (the shell
  // owns the network client and token store) and read back by the renderer.
  void setSimbriefState(const SimBriefState& state) {
    simbriefState_ = state;
    updateNavigraphAutoLogin();
  }
  const SimBriefState& simbriefState() const { return simbriefState_; }
  // LOGIN softkey latch: returns true once per press, so the shell can start
  // the Navigraph device-authorization sign-in.
  bool consumeNavigraphLoginRequest();
  // LOGOUT softkey latch: returns true once per press, so the shell can forget
  // the session (and clear the persisted refresh token).
  bool consumeNavigraphLogoutRequest();
  // FETCH softkey latch: returns true once per press, so the shell can kick off
  // the OFP download for the signed-in account.
  bool consumeSimbriefFetchRequest();
  // Catalog-open auto-refresh latch: returns true once each time the pilot
  // opens the FPL - Flight Plan Catalog page while signed in to Navigraph, so
  // the shell re-fetches the latest SimBrief OFP and a freshly generated plan
  // shows up without restarting. Distinct from the FETCH latch above because
  // the standalone shell lands the result into the catalog, whereas the in-sim
  // plugin replaces the active route -- the plugin therefore ignores this one.
  bool consumeCatalogRefreshRequest();

  // ---- Navigraph charts (AUX - Charts page) ----
  // Latest chart index + selected chart image, published by the shell each
  // frame (the shell owns the Charts API client + access token) and read back
  // by the page renderer. Selection (which chart in the list) is clamped here.
  void setChartsState(const ChartsState& state);
  const ChartsState& chartsState() const { return chartsState_; }
  // The airport the page wants charts for: the active flight plan's destination
  // when the Airport box is on dest, otherwise its origin. Empty with no plan.
  // The shell reads this to drive the chart index fetch.
  std::string chartsDesiredAirport() const;
  // Which flight-plan endpoint the Airport box shows (false = origin, true =
  // dest). Seeds the free ICAO entry on the Airport box.
  bool chartsUseDestination() const { return chartsUseDestination_; }
  // In-progress free ICAO entry on the Airport box (the FMS knob spells an
  // airport ident; ENT applies it as the charts airport). Inactive otherwise.
  bool chartsAirportEntryActive() const { return chartsAirportEntry_.active; }
  std::string chartsAirportEntryIdent() const {
    return chartsAirportEntry_.ident();
  }
  int chartsAirportEntryCursor() const { return chartsAirportEntry_.pos; }
  int chartsAirportEntryTypedCount() const {
    return chartsAirportEntry_.typedCount();
  }
  bool chartsAirportEntrySelectAll() const {
    return chartsAirportEntry_.selectAll;
  }
  // Live spell-ahead match for the in-progress ident, so the page can show the
  // resolved airport's name/city while typing (mirrors the FPL/PROC entry).
  bool chartsAirportEntryHasMatch() const {
    return chartsAirportEntry_.hasMatch;
  }
  MapFeature chartsAirportEntryMatch() const {
    return chartsAirportEntry_.match;
  }
  ChartsMenu chartsMenu() const { return chartsMenu_; }
  ChartsField chartsField() const { return chartsField_; }
  ChartsViewMode chartsViewMode() const { return chartsViewMode_; }
  ChartsCategoryFilter chartsCategoryFilter() const {
    return chartsCategoryFilter_;
  }
  bool chartsFullScreen() const { return chartsFullScreen_; }
  // True while the WPT - Airport Information page is showing terminal charts
  // (entered via the Charts softkey, Pilot's Guide §8.3). On the real unit
  // charts are part of the Airport Information page, not a standalone page.
  bool chartViewActive() const { return chartViewActive_; }
  // Chart softkey toggle: show the associated nav map instead of the chart
  // image (Pilot's Guide §8.3, "switches between the diagram and the map").
  bool chartsShowMap() const { return chartsShowMap_; }
  // RANGE-joystick chart zoom (1 = base fit) and pan offset as a fraction of the
  // drawn chart size, plus the CHRT Opt "Fit WDTH" base-fit mode.
  float chartsZoom() const { return chartsZoom_; }
  float chartsPanXFrac() const { return chartsPanXFrac_; }
  float chartsPanYFrac() const { return chartsPanYFrac_; }
  bool chartsFitWidth() const { return chartsFitWidth_; }
  // Committed chart row (the chart actually shown / downloaded), and its id
  // (empty when the list is empty). The shell reads the id (with chartsNight())
  // to drive the chart image download, so the image only changes when the pilot
  // commits a new selection with ENT.
  int chartsSelected() const { return chartsSelected_; }
  std::string chartsSelectedChartId() const;
  // Highlighted row in the open selection list (the popup). While the pilot
  // scrolls the list this differs from chartsSelected() until ENT commits it.
  int chartsPending() const { return chartsPending_; }
  // Day/night chart variant (Info softkey toggles; Chart Setup on the real unit).
  bool chartsNight() const { return chartsNight_; }

  // True when a modal MFD interaction owns the FMS knob (Direct-To, FPL edit,
  // WPT ident entry, map pointer).
  bool blocksRadioBezel() const;

  // Published approaches for an airport ICAO (from the nav database).
  std::vector<MapApproach> approachesForAirport(
      const std::string& icao) const;

  // Terminal procedures for an airport (WPT/NRST approach boxes).
  std::vector<MapProcedure> proceduresForAirport(const std::string& icao,
                                                 ProcedureType type) const;
  // Procedures for the PROC menu at the active flight-plan airport.
  std::vector<MapProcedure> proceduresFor(ProcedureType type) const;

  // Airport comm frequencies (apt.dat rows 50–56).
  std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const;

  // Airport runways (apt.dat row 100) for the WPT/NRST Runways boxes.
  std::vector<AirportRunwayInfo> airportRunways(const std::string& icao) const;

  // ---- Procedures window (PROC bezel key on the FPL page, Pilot's Guide 5.8) ----
  // Shared menu logic with the PFD (ProcedureMenu.cpp); MFD-specific host wiring
  // only supplies airport resolution and close/activate callbacks.
  using ProcStep = avionics::ProcStep;
  using ProcMode = avionics::ProcMode;
  using ProcApproachField = avionics::ProcApproachField;

  bool procMenuOpen() const { return procMenuOpen_; }
  // "Fly Course Reversal at <fix>?" prompt shown after loading an approach via
  // an IAF that has a HILPT course reversal (overlays the page until answered).
  bool courseReversalPromptActive() const {
    return procMenu_.courseReversalPromptActive;
  }
  const std::string& courseReversalPromptFix() const {
    return procMenu_.courseReversalFix;
  }
  bool courseReversalPromptYes() const { return procMenu_.courseReversalYes; }
  // Mutable procedure-menu state, used by the engine to mirror the course-
  // reversal prompt onto the peer GDU.
  ProcedureMenuState& procedureMenuStateRef() { return procMenu_; }
  bool procSelectMode() const { return procMenu_.mode == ProcMode::Select; }
  // Approach-loading map preview range: the preview auto-frames the highlighted
  // procedure until the user turns the RANGE knob, after which the manually
  // selected range is honored (Pilot's Guide 5.8).
  bool procPreviewRangeManual() const { return procPreviewRangeManual_; }
  void setProcPreviewFitRange(int ladderIndex);
  // FPL page inset map: auto-frames the route until the pilot turns RANGE.
  bool fplPreviewRangeManual() const { return fplPreviewRangeManual_; }
  void setFplPreviewFitRange(int ladderIndex);
  const char* procWindowTitle() const;
  int procMenuItemCount() const {
    return static_cast<int>(procMenu_.menuItems.size());
  }
  const std::string& procMenuItemText(int i) const;
  bool procMenuItemEnabled(int i) const;
  int procMenuSelected() const { return procMenu_.menuSel; }
  ProcStep procStep() const { return procMenu_.step; }
  ProcedureType procCategory() const { return procMenu_.category; }
  int procSelected() const { return procMenu_.selected; }
  int procListSelected() const { return procMenu_.selected; }
  const std::string& procSelectedName() const { return procMenu_.selectedName; }
  std::string procAirportIcao() const;
  std::vector<std::string> procProcedureNames(ProcedureType type) const;
  std::vector<std::string> procTransitions(ProcedureType type,
                                           const std::string& name) const;
  std::vector<std::string> procTransitionLabels(ProcedureType type,
                                                const std::string& name) const;
  std::vector<std::string> procListItems() const;
  bool procSubListOpen() const { return procMenu_.subListOpen; }
  ProcApproachField procApproachField() const { return procMenu_.approachField; }
  bool procAirportEntryActive() const { return procMenu_.airportEntry.active; }
  std::string procAirportEntryIdent() const {
    return procMenu_.airportEntry.ident();
  }
  int procAirportEntryCursor() const { return procMenu_.airportEntry.pos; }
  int procAirportEntryTypedCount() const {
    return procMenu_.airportEntry.typedCount();
  }
  bool procAirportEntrySelectAll() const {
    return procMenu_.airportEntry.selectAll;
  }
  bool procAirportEntryHasMatch() const {
    return procMenu_.airportEntry.hasMatch;
  }
  MapFeature procAirportEntryMatch() const {
    return procMenu_.airportEntry.match;
  }
  std::string procAirportEntryCityLine() const;
  std::string procAirportCityLine() const;
  std::string procAirportNameLine() const;
  MapFeature procAirportFeature() const;
  std::string procApproachDisplayName(int index) const;
  std::string procSelectedApproachDisplay() const;
  std::string procSelectedTransitionDisplay() const;
  std::string procSelectedRunwayDisplay() const;
  float procPrimaryFreqMhz() const;
  bool procPrimaryNavIsNdb() const;
  bool procShowsPrimaryNavFreq() const;
  std::string procPrimaryIdent() const;
  bool procLoadArmed() const { return procMenu_.loadArmed; }
  bool procActivateArmed() const { return procMenu_.activateArmed; }
  // Whether the FMS field cursor has descended into the Sequence box, and the
  // previewed-leg row it currently highlights.
  bool procSequenceFocused() const { return procMenu_.sequenceFocused; }
  int procSequenceSelected() const { return procMenu_.sequenceSelected; }
  bool minimumsBaroOn() const { return minsBaroOn_; }
  float minimumsAltitudeFt() const { return minsAltFt_; }
  float minimumsTempC() const { return minsTempC_; }
  std::vector<MapLeg> procPreviewLegs() const;
  bool consumeProcLoadRequest(MapProcedure& out);
  bool consumeActivateMissedRequest();

  // ---- Page menu (MENU bezel key, Pilot's Guide Fig. 5-6) ----
  // The context "Page Menu" popout, opened by the MENU key on pages that
  // define one (the Navigation Map and the Active Flight Plan page). It lists
  // the page's options; the FMS knob moves the highlight, ENT runs the
  // highlighted option and closes the menu, and CLR / MENU / the FMS knob push
  // back out to the base page. Options this suite does not yet model are listed
  // either greyed (Disabled, the cursor skips them) or selectable-but-inert
  // (DisplayOnly), matching the real unit's greyed/active states for each row.
  enum class PageMenuAction {
    Disabled,            // greyed and skipped (the feature is not modeled)
    DisplayOnly,         // selectable but inert (the feature is not modeled)
    MapDeclutter,        // cycle the Navigation Map declutter (Detail) level
    OpenMapSettings,     // open the Map Settings window (Fig. 5-7)
    FplLoadAirway,       // open the Select Airway window for the cursor fix
    FplCollapseAirways,  // toggle the FPL airway collapse/expand display
    FplDeleteFlightPlan, // open the Delete Flight Plan confirmation
    FplRemoveDeparture,  // open the Remove Departure confirmation
    FplRemoveArrival,    // open the Remove Arrival confirmation
    FplRemoveApproach,   // open the Remove Approach confirmation
    ChartsFullScreen,    // Chart Setup: toggle the full-screen chart view
    ChartsColorScheme,   // Chart Setup: toggle day/night color scheme
    // Flight Plan Catalog page menu (Pilot's Guide, Flight Plan Storage).
    CatalogCreateNew,      // add a new (empty) stored flight plan
    CatalogActivate,       // open "activate stored flight plan?" confirmation
    CatalogInvertActivate, // open "invert and activate stored flight plan?"
    CatalogCopy,           // copy the selected stored plan to a new slot
    CatalogDelete,         // open the "delete flight plan?" confirmation
    CatalogDeleteAll,      // open the "delete all flight plans?" confirmation
  };
  struct PageMenuItem {
    std::string text;
    PageMenuAction action = PageMenuAction::Disabled;
    // Draw the Direct-To glyph after the label (the FPL menu's "VNV D->" row).
    bool dtoSuffix = false;
  };

  bool pageMenuOpen() const { return pageMenuOpen_; }
  // 0..1 open progress for the slide+fade animation, like directToWindowAnim().
  float pageMenuAnim() const { return pageMenuAnim_; }
  int pageMenuItemCount() const {
    return static_cast<int>(pageMenuItems_.size());
  }
  const std::string& pageMenuItemText(int i) const;
  bool pageMenuItemEnabled(int i) const;
  // True for rows that draw the Direct-To glyph after the label (FPL "VNV").
  bool pageMenuItemDtoSuffix(int i) const;
  int pageMenuSelected() const { return pageMenuSel_; }

  // ---- Map Settings window (MENU -> Map Settings on the Navigation Map) ----
  // A popout dialog (Pilot's Guide Fig. 5-7) with a Group selector and the
  // active group's settings rows. The large FMS knob moves the field cursor,
  // the small knob edits the highlighted control, and the FMS knob push / CLR
  // close the window.
  bool mapSettingsOpen() const { return mapSettingsOpen_; }
  // 0..1 open progress for the slide+fade animation, like directToWindowAnim().
  float mapSettingsAnim() const { return mapSettingsAnim_; }
  MapSettingsGroup mapSettingsGroup() const { return mapSettingsGroup_; }
  // Cursor position: 0 = the Group selector, 1..N = the active group's editable
  // controls in row order (see the field walk in MfdMapSettings.h).
  int mapSettingsCursor() const { return mapSettingsCursor_; }
  // Display string for a control (the value the page renders in cyan).
  std::string mapSettingText(MapSetting id) const;
  // Toggle value (for the show/hide controls).
  bool mapSettingOn(MapSetting id) const;
  // The map range threshold (NM) above which a range-gated control hides its
  // symbols, read by the navigation-map renderer for the controls it models.
  float mapSettingRangeNm(MapSetting id) const;

  // ---- read by the renderer ----
  const std::string& label(int i) const { return labels_[i]; }
  float pressLevel(int i) const { return press_[i]; }
  // All kSoftkeyCount press levels, for the shell's physical softkey row.
  const float* pressLevels() const { return press_.data(); }
  // False for greyed options (Charts, unavailable Engine pages, etc.).
  bool keyEnabled(int i) const;
  // True while the cell's page group is the selected one (radio highlight).
  bool keyActive(int i) const;

  // ---- window bezel key column (drawn by the standalone shell) ----
  // Apply a press of a hardware bezel key: flashes the key and, for the range
  // rocker, steps the MFD map range.
  void pressBezelKey(BezelKey key);
  void flashBezelKey(BezelKey key);
  const float* bezelPressLevels() const { return bezelPress_.data(); }
  // CLR (DFLT MAP) held: abandon any in-progress entry or submenu and display
  // the Navigation Map page immediately (Pilot's Guide: "press and hold CLR
  // (MFD only)"). The shell calls this after kClrDefaultMapHoldSeconds.
  void clrDefaultMap();

  // True when a modal popup currently owns the CLR key (Remove/Delete
  // confirmation, waypoint or VNAV-altitude entry, catalog confirmation). While
  // one is open the press-and-hold CLR (DFLT MAP) must not fire, or it would
  // blow away the popup the same CLR press just opened.
  //
  // The FPL cursor parked on the ALT column also claims CLR: there a short CLR
  // removes the fix's VNAV altitude constraint in place (no modal), so the
  // press-and-hold must not escalate to DFLT MAP and close the FPL page out
  // from under the edit.
  bool clrDefaultMapHoldSuppressed() const {
    return fplConfirm_ != FplConfirm::None ||
           catalogConfirm_ != CatalogConfirm::None || fplEntry_.active ||
           fplAltEntry_.active || loadAirway_.open ||
           (pageGroup_ == MfdPageGroup::FlightPlan && fplCursorOn_ &&
            fplCursorCol_ == FplCursorCol::Altitude);
  }

 private:
  // The persistence helpers read/write the durable display options directly.
  friend void captureMfdState(const MfdController&, MfdPersistentState&);
  friend void applyMfdState(MfdController&, const MfdPersistentState&);

  void tryRestorePersistedApproach();
  void tryRestorePersistedTerminalProcedures();
  void reinferApproachFromProcedureLegs();
  // Leg index under the FPL cursor (-1 for blank / sep rows in approach view).
  int fplCursorLegIndex() const;
  FplRouteEdit fplRouteEditState() const;
  void fplClampCursorRow();

  // The MFD softkey bar is a small menu stack like the PFD's: the root bar
  // can open the Engine or Map Opt submenus, which carry a Back key (NXi
  // trainer screenshots / WT MFDNavMapRootMenu).
  enum class Menu { Root, Engine, MapOpt, RadarMode };

  // Refresh the visible cell labels for the current menu, including the
  // state-carrying labels (TER / AWY / Detail show their selection).
  void rebuildLabels();
  // Step the active group's page index by +/-1, wrapping (small FMS knob).
  void stepPage(int direction);
  void selectGroup(MfdPageGroup group);
  // Step the displayed checklist by +/-1, wrapping, and reset the item cursor.
  void stepChecklist(int direction);
  // Number of items in the checklist at the given flat index (0 if none).
  int checklistItemCount(int checklistIndex) const;
  // ENT on the Checklist page: check the cursor item and auto-advance, or (on
  // the "go to next checklist?" prompt) advance to the next checklist.
  void checklistEnter();
  // CLR on the Checklist page: uncheck the cursor item.
  void checklistClear();

  // Step the page group with the large FMS knob, cycling MAP/WPT/AUX/NRST
  // (the FPL and Checklist groups are entered with their own keys, as on the
  // real unit, so the knob steps out of them to MAP).
  void stepPageGroup(int direction);
  // Bezel keys while the FPL page is displayed. Returns true when consumed
  // (cursor/entry/menu interactions); unconsumed keys fall through to the
  // common handling (FPL toggle, range rocker).
  bool fplBezelKey(BezelKey key);
  // Procedures window (PROC key): build the top-level menu on open, route the
  // FMS knob / ENT / CLR while it is open, and load / activate the selection.
  bool procBezelKey(BezelKey key);
  // Modal "Fly Course Reversal?" prompt: owns the FMS knob / ENT / CLR while up.
  bool courseReversalPromptBezelKey(BezelKey key);
  // Modal hold Direct-To confirmation (Activate/Cancel) on a selected HOLD row.
  bool holdActivatePromptBezelKey(BezelKey key);
  // ---- Load Airway window (MfdControllerLoadAirway.cpp) ----
  // Route a bezel key while the Select Airway window is open (always consumes).
  bool loadAirwayBezelKey(BezelKey key);
  // Recompute the fix chain + exit selection for the current airway pick.
  void loadAirwayRefreshFixes();
  // Recompute the entry->exit DTK/DIS preview for the highlighted exit.
  void loadAirwayRefreshCourse();
  // Insert the expanded airway segment after the entry fix and publish (Load?).
  void loadAirwayCommit();
  void closeLoadAirwayWindow();
  void openHoldActivatePrompt(int legIndex);
  void closeHoldActivatePrompt();
  void buildProcMenu();
  std::string procDefaultAirportIcao() const;
  ProcedureMenuHost procedureMenuHost();
  ProcedureMenuHost procedureMenuHost() const;
  // ---- Page menu (MENU key) ----
  // Build the option list for the current page (empty when the page has no
  // page menu, so MENU is inert there, like the real unit).
  std::vector<PageMenuItem> buildPageMenu() const;
  // Open the page menu for the current page, highlighting the first enabled
  // option (no-op when the page defines no menu).
  void openPageMenu();
  // Route a bezel key while the page menu is open; always consumes the key.
  bool pageMenuBezelKey(BezelKey key);
  // Move the highlight to the next/previous enabled option, wrapping.
  void pageMenuStep(int direction);
  // ENT on the highlighted option: run its action and close the menu.
  void pageMenuActivate();
  // ---- Map Settings window ----
  // Open the window (from the page menu), reset the cursor to the Group field.
  void openMapSettings();
  // Route a bezel key while the window is open; always consumes the key.
  bool mapSettingsBezelKey(BezelKey key);
  // Move the field cursor by +/-1, wrapping over [Group selector + the active
  // group's editable controls].
  void mapSettingsStepCursor(int dir);
  // Edit the control under the cursor (small knob): cycle the group on the
  // Group selector, flip a toggle, cycle an enum, or step a range threshold.
  void mapSettingsEdit(int dir);
  // Number of editable controls in the active group (cursor stops 1..N).
  int mapSettingsFieldCount() const;
  // The MapSetting at cursor position `cursor` (1..N), or MapSetting::Count for
  // the Group selector / an out-of-range cursor.
  MapSetting mapSettingAtCursor(int cursor) const;
  // Reset every FPL interaction state (cursor, entry, menu, confirmation).
  void fplResetInteraction();
  // Save the MAP zoom when a page-group change enters FPL, and restore it when
  // the change leaves FPL, so the FPL route auto-fit preview never clobbers the
  // pilot's MAP range. Call with the target group before assigning pageGroup_.
  void syncFplPreviewRange(MfdPageGroup target);
  // Open the Remove Departure/Arrival/Approach confirmation, seeding the prompt
  // subject (the loaded procedure name) for the confirmation window.
  void fplOpenProcedureRemoveConfirm(FplConfirm which);
  // Execute the highlighted Remove Departure/Arrival/Approach: erase the
  // procedure's legs, clear its grouping + persisted-restore state, and publish.
  void fplRemoveLoadedDeparture();
  void fplRemoveLoadedArrival();
  void fplRemoveLoadedApproach();
  // ---- Flight Plan Catalog ----
  // Bezel keys while the Flight Plan Catalog page is up: FMS push toggles the
  // list cursor, the large knob scrolls slots, ENT activates the selection, the
  // small knob falls through to FPL-group page stepping. Always consumes the
  // routed keys, returning false only to let a key fall through to page nav.
  bool catalogBezelKey(BezelKey key);
  // Move the catalog selection by +/-1 (clamped), turning the cursor on.
  void catalogStepSelection(int direction);
  // Clamp the selection to the current catalog size.
  void catalogClampSelection();
  // Load a stored plan into the active flight plan and publish the edit.
  void loadStoredPlanIntoActive(const PersistedFlightPlan& entry);
  // ENT in the FPL entry window: insert the matched waypoint before the cursor
  // row (append on the blank end slot) and advance the cursor.
  void fplCommitEntry();
  FmsWaypointEntry* activeWaypointEntry();
  void requestActivateFlightPlanLeg(int toLegIndex);
  // ---- VNAV altitude-constraint entry (ALT column) ----
  // Open the 5-digit entry over the given row, seeded with its constraint.
  void fplAltEntryOpen(int row);
  // ENT: parse the digits and set (or clear, when 0) the leg's constraint.
  void fplAltEntryCommit();

  // ---- Direct-To ----
  // Direct-To bezel-key handling (open the window, route keys while open).
  // Returns true when the key was consumed by the Direct-To window.
  bool directToBezelKey(BezelKey key);
  void directToOpen();

  // Bezel keys while a WPT / NRST / MAP page owns the FMS knob.
  bool wptBezelKey(BezelKey key);
  bool nrstBezelKey(BezelKey key);
  bool mapBezelKey(BezelKey key);
  // FMS knob handling while the Weather Radar page is up: the small knob trims
  // antenna tilt, or the bearing line when it is displayed (Pilot's Guide,
  // Radar Controls). The large knob falls through to page-group selection.
  bool radarBezelKey(BezelKey key);
  // FMS knob handling while the AUX - Charts page is up: the small knob scrolls
  // the chart list. Returns true only when it consumed the key (charts are
  // present); otherwise the key falls through to normal page stepping.
  bool chartsBezelKey(BezelKey key);
  // True when either the origin or destination airport can be resolved (so the
  // Charts page Origin/Dest softkeys are live).
  bool chartsAnyAirportAvailable() const;
  // Resolve the flight plan's origin / destination airport ICAO for charts
  // (4-letter idents only); empty when none is available at that end.
  std::string chartsOriginAirport() const;
  std::string chartsDestinationAirport() const;
  // Enter chart view on the WPT - Airport Information page (Charts softkey).
  void selectChartsPage();
  void wptResetInteraction();
  void nrstResetInteraction();
  // Highlighted NRST list facility (airport / fix / NDB / VOR), or null.
  const MapFeature* nrstSelectedFeature() const;
  void mapResetPointer();
  void mapPointerScrollTowardPointer();
  void wptCommitEntry();

  MfdPageGroup pageGroup_ = MfdPageGroup::Map;
  // Per-group selected page, remembered across group switches like the real
  // unit (indexed by MfdPageGroup). Sized for every group so an out-of-range
  // index is never possible; the Checklist group uses checklistIndex_ instead.
  std::array<int, 6> pageIndex_{};
  // Group shown before the FPL key was pressed, restored when it is pressed
  // again (the FPL page is a toggle overlaid on normal page navigation).
  MfdPageGroup groupBeforeFpl_ = MfdPageGroup::Map;
  int rangeIndex_ = kMapRangeDefaultIndex;  // ladder index (defaults to 10 NM)
  // Map zoom captured when the FPL page is opened. The FPL route preview shares
  // rangeIndex_ and auto-fits it to the loaded route every frame, so the MAP
  // zoom is saved here on entry and restored when FPL is left (see
  // syncFplPreviewRange) -- otherwise MAP forgets its scale after a FPL visit.
  int rangeIndexBeforeFpl_ = kMapRangeDefaultIndex;
  // Animated scale eased toward mapRangeNmAt(rangeIndex_) by update(); seeded
  // to the default so the first frame is already at the right zoom.
  float displayRangeNm_ = mapRangeNmAt(kMapRangeDefaultIndex);
  Menu menu_ = Menu::Root;
  TerrainDisplay terrain_ = TerrainDisplay::Off;  // terrain off by default
  AirwayDisplay airways_ = AirwayDisplay::Off;
  bool showTraffic_ = false;
  bool showWeather_ = false;
  // Airframe radar fit; gates the MAP - Weather Radar page (see
  // setWeatherRadarAvailable). Defaults to available.
  bool weatherRadarAvailable_ = true;
  MapDetail detail_ = MapDetail::All;
  MapOrientation mapOrientation_ = MapOrientation::NorthUp;

  // Weather Radar page state. The radar powers up in Standby (antenna parked);
  // the Mode submenu selects Weather/Ground. Tilt/gain/bearing/sector mirror
  // the GWX controls.
  RadarMode radarMode_ = RadarMode::Standby;
  RadarScan radarScan_ = RadarScan::Horizontal;
  bool radarBearingLineOn_ = false;
  float radarBearingDeg_ = 0.0f;
  float radarTiltDeg_ = 0.0f;
  bool radarGainCalibrated_ = true;
  float radarGainManual_ = 0.0f;
  RadarSector radarSector_ = RadarSector::Full;
  bool radarStab_ = true;
  bool radarAct_ = false;
  double radarSweepPhase_ = 0.0;
  bool radarCursorOn_ = false;

  bool mapPointerActive_ = false;
  double mapPointerLat_ = 0.0;
  double mapPointerLon_ = 0.0;
  double mapPanViewCenterLat_ = 0.0;
  double mapPanViewCenterLon_ = 0.0;
  bool mapViewportValid_ = false;
  float mapViewportX_ = 0.0f;
  float mapViewportY_ = 0.0f;
  float mapViewportW_ = 0.0f;
  float mapViewportH_ = 0.0f;
  float mapViewportDisplayH_ = 0.0f;

  // WPT ident search state.
  FmsWaypointEntry wptEntry_;
  MapFeature wptFeature_{};
  bool wptHasSelection_ = false;
  // WPT - Airport Information sub-view (reset to Airport when the page is left
  // or the selected airport changes, like chartViewActive_).
  WptInfoView wptInfoView_ = WptInfoView::Airport;

  // NRST list cursor state.
  bool nrstCursorOn_ = false;
  int nrstSelected_ = 0;

  float pageSelectSec_ = 0.0f;  // page-select popup time remaining

  // Checklist page group state. checklist_ is the latest data cached by
  // syncChecklist (owned by the DataSource, not this controller). The checked
  // flags are kept here because they are interactive UI state, not file data.
  const ChecklistData* checklist_ = nullptr;
  int checklistIndex_ = 0;
  int cursorItem_ = 0;
  std::vector<std::vector<std::uint8_t>> checked_;

  // FPL page state. fplLegs_ mirrors the map's flight plan and carries local
  // edits until the shell applies them to the data sources; fplLastMapPlan_
  // detects external plan changes, fplLastPublished_ keeps the source catching
  // up with our own edit from being mistaken for one.
  const NavFeatureSource* navSource_ = nullptr;
  const StationWeatherSource* weatherSource_ = nullptr;
  const MapData* mapData_ = nullptr;  // latest synced map (ownship, features)
  std::string activeWaypoint_;        // FMS active leg TO ident (DTO default)
  std::vector<MapLeg> fplLegs_;
  std::vector<MapLeg> fplLastMapPlan_;
  std::vector<MapLeg> fplLastPublished_;
  bool fplEditPending_ = false;
  bool fplLocalDraft_ = false;
  bool fplNavDirectToActive_ = false;
  // Set when a catalog plan is activated so the shell can mirror its procedure
  // grouping onto the peer GDU (see consumeActivatedFlightPlan).
  bool activatedPlanPending_ = false;
  PersistedFlightPlan activatedPlan_{};
  bool fplDestinationFilled_ = false;
  MapProcedure fplLoadedApproach_{};
  int fplApproachLegStart_ = 0;
  int fplApproachLegCount_ = 0;
  std::string fplApproachHeaderLabel_;
  MapProcedure fplLoadedDeparture_{};
  int fplDepartureLegStart_ = 0;
  int fplDepartureLegCount_ = 0;
  std::string fplDepartureHeaderLabel_;
  PersistedLoadedApproach persistedDepartureRestore_{};
  MapProcedure fplLoadedArrival_{};
  int fplArrivalLegStart_ = 0;
  int fplArrivalLegCount_ = 0;
  std::string fplArrivalHeaderLabel_;
  PersistedLoadedApproach persistedArrivalRestore_{};
  PersistedLoadedApproach persistedApproachRestore_{};
  // Re-expand the CIFP approach once nav data is ready so the procedure's holds,
  // altitudes, and glidepath (not persisted per-leg) are re-attached after a
  // restart. Set on restore, cleared once applied (or known unmatchable).
  bool fplApproachRestorePending_ = false;
  bool fplDepartureRestorePending_ = false;
  bool fplArrivalRestorePending_ = false;
  bool fplCursorOn_ = false;
  bool fplListCursorFollowsActive_ = true;
  int fplCursorRow_ = 0;
  FplCursorCol fplCursorCol_ = FplCursorCol::Ident;
  FmsWaypointEntry fplEntry_;

  // VNAV altitude-constraint entry: a fixed 5-digit field (feet) edited with the
  // FMS knob, seeded from the row's existing constraint.
  struct FplAltEntry {
    bool active = false;
    int row = 0;
    std::string digits = "00000";  // exactly 5 cells, '0'-'9'
    int pos = 0;                   // cell under the entry cursor
  };
  FplAltEntry fplAltEntry_;
  FplConfirm fplConfirm_ = FplConfirm::None;
  bool fplConfirmOk_ = true;
  std::string fplRemoveIdent_;

  // Flight Plan Catalog page state: the stored plans, the highlighted slot, the
  // list cursor, the action confirmation window, and the shell re-persist latch.
  FlightPlanCatalog catalog_;
  int catalogSelected_ = 0;
  bool catalogCursorOn_ = false;
  CatalogConfirm catalogConfirm_ = CatalogConfirm::None;
  bool catalogConfirmOk_ = true;
  bool catalogDirty_ = false;

  // Page menu (MENU key) state: the option list built for the current page,
  // the highlighted row, and whether the popout is up.
  std::vector<PageMenuItem> pageMenuItems_;
  int pageMenuSel_ = 0;
  bool pageMenuOpen_ = false;
  float pageMenuAnim_ = 0.0f;  // 0..1 open progress, eased by update()

  // Map Settings window state. The show/hide toggles and the range thresholds
  // not already owned by a softkey option live here, keyed by MapSetting; the
  // shared controls (Orientation, Terrain, NEXRAD, Traffic) read/write the
  // existing members so the window and softkeys stay in sync.
  bool mapSettingsOpen_ = false;
  float mapSettingsAnim_ = 0.0f;  // 0..1 open progress, eased by update()
  MapSettingsGroup mapSettingsGroup_ = MapSettingsGroup::Map;
  int mapSettingsCursor_ = 0;  // 0 = Group selector, 1..N = a setting control
  std::array<bool, static_cast<std::size_t>(MapSetting::Count)> msToggle_{};
  std::array<int, static_cast<std::size_t>(MapSetting::Count)> msRange_{};
  int msTrafficMode_ = 0;  // 0 All Traffic / 1 TA/PA / 2 TA Only

  // Procedures window (PROC key): shared menu state; logic in ProcedureMenu.cpp.
  bool procMenuOpen_ = false;
  bool procPreviewRangeManual_ = false;
  bool fplPreviewRangeManual_ = false;
  ProcedureMenuState procMenu_;
  bool procActivateMissedPending_ = false;
  bool minsBaroOn_ = false;
  float minsAltFt_ = 0.0f;
  float minsTempC_ = 26.7f;  // ~80 deg F for TEMP At display on approach loading

  // Direct-To window state.
  bool dtoOpen_ = false;
  float dtoAnim_ = 0.0f;   // 0..1 open progress, eased by update()
  bool dtoArmed_ = false;  // waypoint confirmed, ACTIVATE? selectable
  FmsWaypointEntry dtoEntry_;
  bool dtoRequestPending_ = false;
  bool dtoRequestHold_ = false;
  MapLeg dtoRequestTarget_;
  bool holdActivatePromptActive_ = false;
  bool holdActivatePromptActivate_ = true;
  MapLeg holdActivatePromptLeg_;
  bool fplActivateLegPending_ = false;
  int fplActivateLegIndex_ = -1;

  // Load Airway window state (FPL page MENU -> Load Airway). The entry fix is
  // fixed from the cursor leg when the window opens; the airway pick and exit
  // selection scroll the fix chain; Load? inserts the expanded segment.
  struct LoadAirwayState {
    bool open = false;
    float anim = 0.0f;  // 0..1 open progress, eased by update()
    std::string entryIdent;
    int entryLegIndex = -1;            // index in fplLegs_ of the entry fix
    std::vector<std::string> airways;  // airways through the entry fix
    int airwaySel = 0;                 // index into airways
    std::vector<MapLeg> fixes;         // entry fix + chain toward the far end
    int exitSel = 1;                   // index into fixes (>=1, never the entry)
    LoadAirwayField field = LoadAirwayField::Airway;
    bool hasCourse = false;
    float dtkDeg = 0.0f;
    float disNm = 0.0f;
  };
  LoadAirwayState loadAirway_;
  // FPL airway collapse/expand display toggle (page menu).
  bool fplAirwaysCollapsed_ = false;
  bool dtoPreservePlan_ = false;
  int dtoPreserveLegIndex_ = -1;
  int dtoPreserveFplCursorRow_ = -1;
  // True while this controller has driven the data source's (shared) Direct-To
  // inset map query active. Multi-display shells run a PFD and an MFD engine off
  // one data source; only the controller that requested the inset may relinquish
  // it, so the PFD's dormant controller cannot clear the MFD popup's request.
  bool insetQueryOwned_ = false;

  // SimBrief / Navigraph page state. The sign-in and fetch lifecycle lives in
  // the shell (which owns the HTTPS client and token store) and is mirrored
  // here for rendering; the softkeys post action latches the shell drains.
  SimBriefState simbriefState_;
  bool navigraphLoginRequested_ = false;
  bool navigraphLogoutRequested_ = false;
  bool simbriefFetchRequested_ = false;
  // Auto-refresh the catalog when the pilot enters the Flight Plan Catalog
  // page: re-fetch the latest SimBrief OFP so a newly generated plan appears
  // without a restart. Latched on the page-enter transition (not every frame)
  // and tracked via the last page seen in update().
  bool catalogRefreshRequested_ = false;
  MfdPage lastPageForCatalogRefresh_ = MfdPage::NavigationMap;
  void updateCatalogAutoRefresh();
  // Auto-start the device-authorization sign-in once per signed-out visit to the
  // SimBrief page, so the QR + code appear without a manual Login press. Armed
  // so exactly one device code is issued (not one request per frame).
  bool navigraphAutoLoginArmed_ = false;
  void updateNavigraphAutoLogin();

  // Navigraph charts page state. The chart index + selected image come from the
  // shell (which owns the Charts API client); selection, layout, and view mode
  // are local UI state mirroring the NXi chart page (Pilot's Guide §8.3).
  ChartsState chartsState_;
  bool chartViewActive_ = false;       // WPT - Airport Info showing charts
  MfdPageGroup groupBeforeCharts_ = MfdPageGroup::Map;  // for "Go Back"
  ChartsMenu chartsMenu_ = ChartsMenu::Selection;
  ChartsField chartsField_ = ChartsField::Airport;
  ChartsViewMode chartsViewMode_ = ChartsViewMode::All;
  ChartsCategoryFilter chartsCategoryFilter_ = ChartsCategoryFilter::Airport;
  bool chartsFullScreen_ = false;
  bool chartsShowMap_ = false;         // Chart softkey: nav map vs chart image
  bool chartsFitWidth_ = false;        // CHRT Opt Fit WDTH base-fit mode
  float chartsZoom_ = 1.0f;            // RANGE-joystick chart zoom (>= 1)
  float chartsPanXFrac_ = 0.0f;        // pan offset, fraction of drawn width
  float chartsPanYFrac_ = 0.0f;        // pan offset, fraction of drawn height
  bool chartsUseDestination_ = true;  // Airport box: dest vs origin (entry seed)
  int chartsSelected_ = 0;             // committed chart shown / downloaded
  int chartsPending_ = 0;              // highlighted row in the open list popup
  bool chartsNight_ = false;           // day/night chart variant
  // Free ICAO entry on the Airport box and the resulting explicit airport. When
  // chartsAirportOverride_ is set it wins over the flight-plan origin/dest in
  // chartsDesiredAirport(), so any airport's charts can be pulled up.
  FmsWaypointEntry chartsAirportEntry_;
  std::string chartsAirportOverride_;
  void chartsSelectCategory(ChartsCategoryFilter filter);
  // Commit the highlighted popup row (chartsPending_) as the shown chart (ENT).
  void chartsCommitSelection();
  // Apply the typed Airport-box ident as the charts airport (ENT).
  void chartsCommitAirportEntry();
  // True when the chart index has at least one chart of the given category
  // (greys the DP/STAR/APR/Info softkeys when none exists).
  bool chartsHasCategory(ChartsCategoryFilter filter) const;
  void chartsStepSelection(int delta);
  // Reset zoom/pan/fit when the displayed chart changes.
  void chartsResetView();

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};
  std::array<float, kBezelKeyCount> bezelPress_{};

  // ~1 Hz blink phase for pulsing highlight-select cursor fields.
  double blinkSeconds_ = 0.0;
  bool blinkOn_ = true;

  // AUX Utility timers / trip statistics, accumulated by update().
  FlightSessionStats flightStats_;
};

}  // namespace avionics
