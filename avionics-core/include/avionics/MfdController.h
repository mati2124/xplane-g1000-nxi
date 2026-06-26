#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "avionics/Checklist.h"
#include "avionics/FlightData.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FmsWaypointEntry.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenuTypes.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/SimBrief.h"
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

  // Confirmation window opened by CLR on a waypoint row or the page menu's
  // Delete Flight Plan option. ENT executes the highlighted OK/CANCEL choice.
  enum class FplConfirm { None, RemoveWaypoint, DeleteFlightPlan };

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
  // Edited-plan latch for the shell: true once after each edit, copying the
  // new plan out so the shell can push it to the data sources.
  bool consumeFlightPlanEdit(std::vector<MapLeg>& out);

  void replaceFlightPlanFromExternal(const std::vector<MapLeg>& plan);
  PersistedFlightPlan persistedFlightPlanSnapshot() const;
  PersistedDirectTo persistedDirectToSnapshot() const;
  void restorePersistedFlightPlan(const PersistedFlightPlan& saved);

  void setPersistedLoadedApproach(const PersistedLoadedApproach& saved);
  FlightPlanApproachState flightPlanApproachState() const;
  void applyFlightPlanApproachState(const FlightPlanApproachState& state);
  // Copy the peer GDU's displayed plan so the PFD FPL window and MFD FPL page
  // always show the same route (called from AvionicsEngine::syncFlightPlanPeer).
  void adoptFlightPlanFromPeer(const std::vector<MapLeg>& legs,
                               bool destinationFilled,
                               const FlightPlanApproachState& approach);

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
  bool fplDestinationFilled() const { return fplDestinationFilled_; }
  bool fplLocalDraft() const { return fplLocalDraft_; }
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
  bool consumeDirectToRequest(MapLeg& out);
  // FPL Activate Leg: ENT on a highlighted waypoint row (Pilot's Guide 5.6).
  bool consumeActivateLegRequest(int& toLegIndex);

  // GCU alphanumeric keypad during waypoint-ident entry (Direct-To / FPL / WPT).
  bool applyGcuEntryKey(char ch);

  // ---- SimBrief (AUX - SIMBRIEF page) ----
  // Latest fetch status, published by the shell each frame (the shell owns the
  // network client) and read back by the page renderer.
  void setSimbriefState(const SimBriefState& state) { simbriefState_ = state; }
  const SimBriefState& simbriefState() const { return simbriefState_; }
  // Committed Pilot ID (digits only). The shell seeds it from settings at
  // startup and persists it when the user commits a new one on the page.
  void setSimbriefPilotId(const std::string& id) { simbriefPilotId_ = id; }
  const std::string& simbriefPilotId() const { return simbriefPilotId_; }
  // True while the softkey bar is in Pilot ID digit-entry mode (the page shows
  // the in-progress digits with the edit cursor instead of the committed ID).
  bool simbriefIdEntryActive() const { return simbriefIdEntry_; }
  const std::string& simbriefPendingId() const { return simbriefPendingId_; }
  // FETCH softkey latch: returns true once per press and clears it, so the
  // shell can kick off the OFP download.
  bool consumeSimbriefFetchRequest();

  // True when a modal MFD interaction owns the FMS knob (Direct-To, FPL edit,
  // WPT ident entry, map pointer, SimBrief ID entry).
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
  std::string procAirportCityLine() const;
  std::string procAirportNameLine() const;
  MapFeature procAirportFeature() const;
  std::string procApproachDisplayName(int index) const;
  std::string procSelectedApproachDisplay() const;
  std::string procSelectedTransitionDisplay() const;
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
    FplDeleteFlightPlan, // open the Delete Flight Plan confirmation
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

 private:
  // The persistence helpers read/write the durable display options directly.
  friend void captureMfdState(const MfdController&, MfdPersistentState&);
  friend void applyMfdState(MfdController&, const MfdPersistentState&);

  void tryRestorePersistedApproach();
  void reinferApproachFromProcedureLegs();
  // Leg index under the FPL cursor (-1 for blank / sep rows in approach view).
  int fplCursorLegIndex() const;
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
  // Apply a softkey press while SimBrief Pilot ID digit entry is active
  // (digits append, BKSP erases, Back abandons the entry).
  void simbriefEntryKey(int key);

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
  // ENT in the FPL entry window: insert the matched waypoint before the cursor
  // row (append on the blank end slot) and advance the cursor.
  void fplCommitEntry();
  FmsWaypointEntry* activeWaypointEntry();
  // Mark the edited plan for the shell to pick up.
  void fplPublishEdit();
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
  const MapData* mapData_ = nullptr;  // latest synced map (ownship, features)
  std::string activeWaypoint_;        // FMS active leg TO ident (DTO default)
  std::vector<MapLeg> fplLegs_;
  std::vector<MapLeg> fplLastMapPlan_;
  std::vector<MapLeg> fplLastPublished_;
  bool fplEditPending_ = false;
  bool fplLocalDraft_ = false;
  bool fplNavDirectToActive_ = false;
  bool fplDestinationFilled_ = false;
  MapProcedure fplLoadedApproach_{};
  int fplApproachLegStart_ = 0;
  int fplApproachLegCount_ = 0;
  std::string fplApproachHeaderLabel_;
  PersistedLoadedApproach persistedApproachRestore_{};
  // Re-expand the CIFP approach once nav data is ready so the procedure's holds,
  // altitudes, and glidepath (not persisted per-leg) are re-attached after a
  // restart. Set on restore, cleared once applied (or known unmatchable).
  bool fplApproachRestorePending_ = false;
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
  bool dtoArmed_ = false;  // waypoint confirmed, ACTIVATE? highlighted
  FmsWaypointEntry dtoEntry_;
  bool dtoRequestPending_ = false;
  MapLeg dtoRequestTarget_;
  bool fplActivateLegPending_ = false;
  int fplActivateLegIndex_ = -1;
  bool dtoPreservePlan_ = false;
  int dtoPreserveLegIndex_ = -1;
  int dtoPreserveFplCursorRow_ = -1;

  // SimBrief page state. The pending ID is UI-only until ENT commits it; the
  // fetch state itself lives in the shell (which owns the HTTPS client) and is
  // mirrored here for rendering.
  SimBriefState simbriefState_;
  std::string simbriefPilotId_;
  std::string simbriefPendingId_;
  bool simbriefIdEntry_ = false;
  bool simbriefFetchRequested_ = false;

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
