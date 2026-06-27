#pragma once

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "avionics/FlightData.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/FmsWaypointEntry.h"
#include "avionics/MapData.h"
#include "avionics/ProcedureMenuTypes.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/MapRange.h"
#include "avionics/Radio.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/MapView.h"

namespace avionics {

// Persisted-preferences view of this controller (avionics/PersistentState.h).
struct PfdPersistentState;

// The PFD softkey bar has 12 cells (the bottom-of-screen menu row).
inline constexpr int kSoftkeyCount = 12;

// Severity of an entry in the Alerts/messages window, which drives its color
// (warning = red, caution = amber, advisory = white), mirroring CAS levels.
enum class AlertLevel { Warning, Caution, Advisory };

struct AlertMessage {
  std::string text;
  AlertLevel level = AlertLevel::Advisory;
};

// One entry of the PFD Nearest Airports window: identifier, GPS bearing and
// distance, primary COM frequency, and longest-runway length (Pilot's Guide,
// Fig. 5-28). Built by the controller from the map snapshot; read by the
// renderer.
struct NearestAirport {
  std::string id;
  float bearingDeg = 0.0f;
  float distanceNm = 0.0f;
  float frequencyMhz = 0.0f;  // 0 = unknown (shown dashed)
  int longestRunwayFt = 0;    // 0 = unknown (shown dashed)
  // Map-symbol attributes for the waypoint icon (towered = cyan, else magenta).
  bool airportTowered = false;
  bool airportServiced = false;
  AirportFacilityKind airportKind = AirportFacilityKind::Land;
  // Primary contact frequency label (TOWER / UNICOM / MULTICOM), empty when
  // none is published.
  std::string comLabel;
  // Best available approach on the longest runway (ILS, LOC, RNA, VOR, NDB,
  // or VFR when none), per the WT NearestStore priority.
  std::string approachType;
};

// The softkey bar is a menu stack: the root (top-level) menu can open submenus,
// and every submenu carries a "Back" key that pops back to its parent. These
// identify which menu is currently shown, matching the G1000 NXi PFD softkey
// map (Pilot's Guide Table 1-3): Map/HSI -> Layout, PFD Opt -> SVT / Wind /
// ALT Units, XPDR -> Code.
enum class SoftkeyMenu {
  Root,
  MapHsi,
  Layout,
  PfdOpt,
  Svt,
  Wind,
  AltUnits,
  Xpdr,
  XpdrCode,
};

// Transponder operating mode selected from the XPDR softkey submenu. Standby,
// On, and ALT are the keys offered on the NXi XPDR softkeys; Ground is entered
// automatically on the ground and has no softkey.
enum class XpdrMode { Standby, On, Alt, Ground };

// Pop-up windows in the lower-right of the PFD. The real unit shows one at a
// time in that region (Alerts, Timer/References, Nearest Airports, PFD Setup
// Menu, Active Flight Plan); opening one replaces any other.
enum class PfdWindow {
  None,
  Alerts,
  References,
  Nearest,
  Setup,
  FlightPlan,
  Procedures
};
inline constexpr int kPfdWindowCount = 7;  // including None

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8 "Procedures"): see
// ProcedureMenuTypes.h for shared menu types and ProcedureMenu.cpp for logic. (PFD MENU key, Pilot's Guide Fig. 1-18). Adjusts the PFD and
// MFD display/key backlighting. The two rows the menu shows -- indexed by this
// in the controller and renderer.
enum class PfdSetupRow { Pfd, Mfd, Count };
inline constexpr int kPfdSetupRowCount = static_cast<int>(PfdSetupRow::Count);

// Backlight control mode (Pilot's Guide: 'Auto' tracks the photocell, 'Manual'
// holds a pilot-set intensity). The key backlight is Auto-only on the real unit.
enum class BacklightMode { Auto, Manual };

// The backlight target a row's arrow-toggle selects: the display screen or the
// bezel keys (turn the small FMS knob "in the direction of the green
// arrowhead" to switch 'PFD Display' to 'PFD Key').
enum class BacklightTarget { Display, Key };

// Fields the FMS cursor can highlight in the PFD Setup Menu. The large FMS knob
// moves between fields (skipping a row's intensity unless that row is in
// Manual); the small knob changes the highlighted field; ENT confirms. Two
// rows (PFD, MFD), each a target toggle, a mode, and an intensity value.
enum class PfdSetupField {
  PfdTarget,  // 'PFD Display' / 'PFD Key' arrow-toggle
  PfdMode,    // Auto / Manual
  PfdValue,   // intensity %
  MfdTarget,
  MfdMode,
  MfdValue,
  Count,
};

// Backlight intensity entry range and small-knob step (percent).
inline constexpr float kBacklightMinPct = 0.0f;
inline constexpr float kBacklightMaxPct = 100.0f;
inline constexpr float kBacklightStepPct = 1.0f;

// Pilot-selectable V-speed reference bugs, in the row order of the NXi
// Timer/References window (Pilot's Guide Fig. 2-6 / Table 2-1).
enum class VspeedRef { Glide, Vr, Vx, Vy, Count };
inline constexpr int kVspeedRefCount = static_cast<int>(VspeedRef::Count);

// Default V-speed reference values (kt) for the delivered airframe (Cessna
// 172S), indexed by VspeedRef. Single source of truth shared by the controller
// (editable working values) and the airspeed-tape default table.
inline constexpr float kDefaultVspeedKt[kVspeedRefCount] = {65.0f, 55.0f, 62.0f,
                                                            74.0f};
// Editable range and small-knob step for the V-speed reference values.
inline constexpr float kVspeedMinKt = 20.0f;
inline constexpr float kVspeedMaxKt = 400.0f;
inline constexpr float kVspeedStepKt = 1.0f;

// Fields the FMS cursor can highlight in the Timer/References window. The FMS
// rocker stands in for the large FMS knob (moves the cursor); ENT activates
// the highlighted field. On MinsValue the rocker stands in for the small knob
// (steps the altitude) and ENT moves the cursor on.
enum class RefField {
  TimerCmd,   // Start? / Stop? / Reset?
  Glide,
  Vr,
  Vx,
  Vy,
  MinsMode,   // Off / BARO / TEMP
  MinsValue,  // MDA/DH altitude (reachable when MinsMode is BARO or TEMP)
  MinsTemp,   // destination temperature (only reachable when MinsMode is TEMP)
  Count,
};

// Minimums (MDA/DH) source selected in the References window (Pilot's Guide,
// Minimums): Off, barometric, or temperature-compensated (TEMP COMP), which
// raises the displayed minimum for cold destination temperatures.
enum class MinimumsMode { Off, Baro, Temp };

// TEMP COMP correction reference and rule-of-thumb rate (Pilot's Guide /
// ICAO cold-temperature correction): ~4 ft of added height per degree C below
// ISA (15 C) per 1000 ft of minimum height.
inline constexpr float kTempCompIsaC = 15.0f;
inline constexpr float kTempCompFtPerCPer1000Ft = 4.0f;
// Destination-temperature entry range and small-knob step.
inline constexpr float kMinsTempMinC = -60.0f;
inline constexpr float kMinsTempMaxC = 50.0f;
inline constexpr float kMinsTempStepC = 1.0f;
inline constexpr float kMinsTempDefaultC = 15.0f;

// Selected Altitude alerting phase (Pilot's Guide, Altitude Alerting, Fig.
// 2-32). Each transition flashes the Selected Altitude box for five seconds:
// within 1000 ft black-on-cyan, within 200 ft cyan-on-black, and a post-
// capture deviation beyond +/-200 ft amber.
enum class AltAlertPhase { Armed, Within1000, Within200, Captured, Deviation };

// Visual treatment the altimeter applies to the Selected Altitude readout for
// the current alerting phase, resolved against the blink clock.
struct SelectedAltStyle {
  bool cyanBackground = false;  // black text on a cyan plate (within 1000 ft)
  bool amberText = false;       // deviation alert
  bool hideText = false;        // blink-off half of a flash cycle
};

// PFD Map layout (Map/HSI > Layout): a radio group selecting where the PFD map
// is shown, or off (G1000 NXi Pilot's Guide).
enum class MapLayout { Off, Inset, Hsi };

// Wind display option (PFD Opt > Wind): off, or one of three formats matching
// the NXi Wind submenu (Option 1/2/3).
enum class WindOption { Off, Option1, Option2, Option3 };

// Display options the softkeys turn on and off. They are surfaced to the
// renderer through keyActive() (the cell highlights while its option is on) and
// can be read by the gauges to honor these options. Count must remain last so
// it sizes the backing array.
enum class DisplayToggle {
  Traffic,
  Obs,
  Dme,
  Bearing1,
  Bearing2,
  AltMeters,
  BaroHpa,
  Pathways,
  SynTerr,
  HdgLbl,
  AptSign,
  MapTopo,
  MapRelTer,
  Count,
};
inline constexpr int kDisplayToggleCount =
    static_cast<int>(DisplayToggle::Count);

// Owns the interactive state of the PFD softkey bar and the pop-up windows it
// opens (currently the Alerts/messages window). One instance lives in the
// AvionicsEngine: it is advanced once per frame by update() and read by the
// renderer. All animation is time-based (frame-rate independent) so it matches
// the Working Title NXi feel -- a brief key-press flash and an eased window
// slide/fade.
class SoftkeyController {
 public:
  // Index of the "Alerts" softkey in the bar (rightmost cell).
  static constexpr int kAlertsKey = 11;

  // Press-flash decay and window open/close durations, in seconds.
  static constexpr float kPressFlashSeconds = 0.18f;
  static constexpr float kWindowAnimSeconds = 0.22f;

  SoftkeyController();

  // Advance animations, timers, and alerting state from the current flight
  // data; the map snapshot feeds the Nearest Airports window list.
  void update(double dtSeconds, const FlightData& data, const MapData& map);

  // Apply a press of physical softkey `key` (0..kSoftkeyCount-1, the hardware
  // keys below the display; the on-screen bar is labels only, like the real
  // unit). Returns true if the key currently has a function.
  bool pressKey(int key);

  // ---- read by the renderer ----
  const std::string& label(int i) const { return labels_[i]; }
  // 0..1 press-flash brightness for the key-press highlight animation.
  float pressLevel(int i) const { return press_[i]; }
  // All kSoftkeyCount press levels, for the shell's physical softkey row.
  const float* pressLevels() const { return press_.data(); }
  // True while the cell's associated window/mode is the active one.
  bool keyActive(int i) const;
  // False for options not yet modeled (e.g. SVT); the bar greys them out and
  // pressKey() ignores them.
  bool keyEnabled(int i) const;

  // PFD pop-up windows (Alerts / References / Nearest). activeWindow() is the
  // one currently selected; windowAnim() is each window's linear open progress
  // in 0..1 (a window should be drawn whenever its progress is > 0, so a
  // closing window fades out while its replacement fades in). The renderer
  // eases the raw value for the visual slide/fade.
  PfdWindow activeWindow() const { return window_; }
  float windowAnim(PfdWindow w) const {
    return windowAnim_[static_cast<int>(w)];
  }
  const std::vector<AlertMessage>& alerts() const { return alerts_; }

  // ---- Timer/References window state ----
  // FMS cursor field currently highlighted in the References window.
  RefField referencesCursor() const { return refCursor_; }
  // Generic timer (Pilot's Guide, Generic Timer): elapsed seconds and whether
  // it is counting. The bottom info panel shows the timer whenever it is
  // running or holds a nonzero value.
  bool timerRunning() const { return timerRunning_; }
  int timerSeconds() const { return static_cast<int>(timerSeconds_); }
  bool timerVisible() const { return timerRunning_ || timerSeconds() > 0; }
  // Label of the timer's command field: Start? / Stop? / Reset?.
  const char* timerCommandLabel() const;
  // V-speed reference bug enable state (References window On/Off fields).
  bool vspeedEnabled(VspeedRef v) const {
    return vspeedOn_[static_cast<int>(v)];
  }
  // Pilot-editable V-speed reference value (kt). Set in the References window
  // with the small FMS knob; read by the airspeed tape bugs/list.
  float vspeedValueKt(VspeedRef v) const {
    return vspeedKt_[static_cast<int>(v)];
  }
  // Minimums (MDA/DH): mode and barometric altitude set in the References
  // window. The altimeter draws the BARO/TEMP MIN box and bug from these.
  MinimumsMode minimumsMode() const { return minsMode_; }
  float minimumsAltitudeFt() const { return minsAltFt_; }
  // Destination temperature (C) used for the TEMP COMP correction.
  float minimumsTempC() const { return minsTempC_; }
  // Effective (displayed) minimum altitude. In TEMP COMP it is the published
  // minimum raised by the cold-temperature correction; otherwise it is the
  // published minimum itself.
  float effectiveMinimumsFt() const {
    if (minsMode_ != MinimumsMode::Temp) return minsAltFt_;
    const float belowIsa = kTempCompIsaC - minsTempC_;
    if (belowIsa <= 0.0f) return minsAltFt_;  // no correction when warmer
    return minsAltFt_ + (minsAltFt_ / 1000.0f) * belowIsa *
                            kTempCompFtPerCPer1000Ft;
  }

  // Nav database for airport comm frequencies and approach types (Nearest
  // Airports window, Fig. 5-28). Optional; fields dash when unset.
  void setNavFeatureSource(const NavFeatureSource* source) {
    navSource_ = source;
  }
  const NavFeatureSource* navFeatureSource() const { return navSource_; }

  // ---- Nearest Airports window state ----
  const std::vector<NearestAirport>& nearestAirports() const {
    return nearest_;
  }
  // Index of the FMS-cursor-selected entry in nearestAirports().
  int nearestCursor() const { return nearestCursor_; }

  // ---- Active Flight Plan window (FPL bezel key) ----
  // The PFD Active Flight Plan window (Pilot's Guide Fig. 5-48, "Active Flight
  // Plan Window on PFD"): the FPL key opens the active flight-plan legs as the
  // same lower-right popout the other PFD windows use. With the FMS-knob cursor
  // off the knob scrolls the list; pushing the knob turns the cursor on so the
  // pilot can insert/remove waypoints (and therefore change the origin and
  // destination, which are the first and last rows). Edits are kept in a local
  // working copy until the shell applies them to the FMS.
  // The displayed/edited legs (working copy, mirroring the map feed plus any
  // pending pilot edits).
  const std::vector<MapLeg>& flightPlanLegs() const { return fplLegs_; }
  // True once the destination slot has a committed waypoint (origin+dest with no
  // enroute, or a three-or-more-leg route). Distinguishes [origin, enroute] from
  // [origin, destination] when both have two legs.
  bool flightPlanDestinationFilled() const { return fplDestinationFilled_; }
  // True while the pilot is building a route locally that has not been adopted
  // from the simulator feed (partial plans stay in-app only).
  bool flightPlanLocalDraft() const { return fplLocalDraft_; }
  PersistedFlightPlan persistedFlightPlanSnapshot() const;
  PersistedDirectTo persistedDirectToSnapshot() const;
  void restorePersistedFlightPlan(const PersistedFlightPlan& saved);
  // Adopt a route pushed from outside the FPL editor (SimBrief OFP, sim FMS).
  void replaceFlightPlanFromExternal(const std::vector<MapLeg>& plan);
  const std::string& activeWaypointId() const { return activeWaypoint_; }
  // True while GPS Direct-To is engaged (present-position nav to the active TO).
  bool mapDirectToActive() const {
    return mapData_ != nullptr && mapData_->directToActive;
  }
  bool mapDirectToHold() const {
    return mapData_ != nullptr && mapData_->directToHold;
  }
  // FMS cursor row (also the scroll anchor). Equals flightPlanLegs().size() when
  // it sits on the blank append slot below the last waypoint.
  int flightPlanCursor() const { return fplCursorRow_; }
  // True while the FMS selection cursor is on (knob pushed), so a row highlights
  // and the knob/ENT/CLR edit instead of scroll.
  bool flightPlanCursorOn() const { return fplCursorOn_; }

  // Waypoint-ident entry overlay (the insert "Waypoint Information" entry):
  // active while spelling an identifier to insert before the cursor row.
  bool flightPlanEntryActive() const { return fplEntry_.active; }
  std::string flightPlanEntryIdent() const { return fplEntry_.ident(); }
  int flightPlanEntryCursor() const { return fplEntry_.pos; }
  int flightPlanEntryTypedCount() const { return fplEntry_.typedCount(); }
  bool flightPlanEntryNotFound() const { return fplEntry_.notFound; }
  bool flightPlanEntryHasMatch() const { return fplEntry_.hasMatch; }
  const MapFeature& flightPlanEntryMatch() const { return fplEntry_.match; }
  bool flightPlanEntryHasGeo() const;
  float flightPlanEntryBearingDeg() const;
  float flightPlanEntryDistanceNm() const;

  // Modal confirmation prompt shown over the window (CLR removes a waypoint,
  // MENU deletes the whole plan).
  enum class FplConfirm { None, RemoveWaypoint, DeleteFlightPlan };
  FplConfirm flightPlanConfirm() const { return fplConfirm_; }
  bool flightPlanConfirmOk() const { return fplConfirmOk_; }
  const std::string& flightPlanRemoveIdent() const { return fplRemoveIdent_; }

  // Latch for the shell: true once after an edit, copying out the edited legs so
  // the shell programs them into the FMS (mirrors consumeDirectToRequest).
  bool consumeFlightPlanEdit(std::vector<MapLeg>& out);
  void flightPlanPublishEdit();
  // Remove a HILPT course-reversal hold at `fixId` from the working flight plan.
  void stripCourseReversalHoldAtFix(const std::string& fixId);
  // Re-sync after the data-source pump when the map snapshot was stale earlier
  // in the frame (route override from consumeFlightPlanEdit).
  void syncFlightPlanFromMap(const MapData& map, bool navDirectTo = false) {
    syncFlightPlanLegs(map, navDirectTo);
  }

  // Restores PROC/FPL approach metadata after reload (waypoints come from the sim).
  void setPersistedLoadedApproach(const PersistedLoadedApproach& saved);
  PersistedLoadedApproach persistedLoadedApproachSnapshot() const;
  FlightPlanApproachState flightPlanApproachState() const;
  void applyFlightPlanApproachState(const FlightPlanApproachState& state);
  FlightPlanTerminalProcedureState flightPlanDepartureState() const;
  void applyFlightPlanDepartureState(const FlightPlanTerminalProcedureState& state);
  FlightPlanTerminalProcedureState flightPlanArrivalState() const;
  void applyFlightPlanArrivalState(const FlightPlanTerminalProcedureState& state);
  // Copy the peer GDU's displayed plan so the PFD FPL window and MFD FPL page
  // always show the same route (called from AvionicsEngine::syncFlightPlanPeer).
  void adoptFlightPlanFromPeer(
      const std::vector<MapLeg>& legs, bool destinationFilled,
      const FlightPlanApproachState& approach,
      const FlightPlanTerminalProcedureState& departure = {},
      const FlightPlanTerminalProcedureState& arrival = {});

  // ---- Procedures window (PROC bezel key) ----
  using ProcStep = avionics::ProcStep;
  using ProcMode = avionics::ProcMode;
  using ProcApproachField = avionics::ProcApproachField;
  // True while the selection sub-window is shown rather than the top-level menu.
  bool procSelectMode() const { return procMenu_.mode == ProcMode::Select; }
  // "Fly Course Reversal at <fix>?" prompt shown after loading an approach via
  // an IAF that has a HILPT course reversal (overlays any page until answered).
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

  // Window title: "Procedures" for the menu, "Select Approach/Arrival/Departure"
  // for the selection sub-window.
  const char* procWindowTitle() const;
  // Top-level menu rows.
  int procMenuItemCount() const {
    return static_cast<int>(procMenu_.menuItems.size());
  }
  const std::string& procMenuItemText(int i) const;
  bool procMenuItemEnabled(int i) const;
  int procMenuSelected() const { return procMenu_.menuSel; }
  // Selection sub-window: the flight-plan airport, the visible list (procedure
  // names on the ProcedureList step, transitions on the TransitionList step),
  // the highlighted row, the step, and the procedure picked before transitions.
  std::string procAirportIcao() const;
  ProcedureType procCategory() const { return procMenu_.category; }
  ProcStep procStep() const { return procMenu_.step; }
  const std::string& procSelectedName() const { return procMenu_.selectedName; }
  std::vector<std::string> procListItems() const;
  int procListSelected() const { return procMenu_.selected; }
  bool procSubListOpen() const { return procMenu_.subListOpen; }
  ProcApproachField procApproachField() const { return procMenu_.approachField; }
  bool procAirportEntryActive() const { return procMenu_.airportEntry.active; }
  std::string procAirportEntryIdent() const { return procMenu_.airportEntry.ident(); }
  int procAirportEntryCursor() const { return procMenu_.airportEntry.pos; }
  int procAirportEntryTypedCount() const {
    return procMenu_.airportEntry.typedCount();
  }
  bool procAirportEntrySelectAll() const { return procMenu_.airportEntry.selectAll; }
  bool procAirportEntryHasMatch() const { return procMenu_.airportEntry.hasMatch; }
  MapFeature procAirportEntryMatch() const { return procMenu_.airportEntry.match; }
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
  // Loaded approach shown in the PFD Flight Plan body (PROC Load?).
  bool flightPlanHasLoadedApproach() const { return fplApproachLegCount_ > 0; }
  bool flightPlanHasLoadedDeparture() const {
    return !fplLoadedDeparture_.name.empty() || fplDepartureLegCount_ > 0;
  }
  bool flightPlanHasLoadedArrival() const {
    return !fplLoadedArrival_.name.empty() || fplArrivalLegCount_ > 0;
  }
  int flightPlanDepartureLegStart() const { return fplDepartureLegStart_; }
  int flightPlanDepartureLegCount() const { return fplDepartureLegCount_; }
  int flightPlanArrivalLegStart() const { return fplArrivalLegStart_; }
  int flightPlanArrivalLegCount() const { return fplArrivalLegCount_; }
  std::string flightPlanDepartureAirportIcao() const;
  std::string flightPlanDepartureHeaderLabel() const;
  std::string flightPlanArrivalAirportIcao() const;
  std::string flightPlanArrivalHeaderLabel() const;
  int flightPlanApproachLegStart() const { return fplApproachLegStart_; }
  int flightPlanApproachLegCount() const { return fplApproachLegCount_; }
  std::string flightPlanApproachAirportIcao() const;
  std::string flightPlanApproachHeaderLabel() const;
  std::string flightPlanApproachTransition() const {
    return fplLoadedApproach_.transition;
  }
  // NAV1-tune latch for the shell: true once after a procedure load, copying out
  // the loaded procedure (ILS approaches carry a frequency).
  bool consumeProcLoadRequest(MapProcedure& out);

  // ---- Direct-To window (Direct-To bezel key) ----
  // The GPS Direct-To window (Pilot's Guide Fig. 5-45, "Direct-to Window -
  // PFD"): the Direct-To key opens it over the PFD blank for ident entry. The
  // FMS knob spells the destination ident; the first ENT confirms the waypoint
  // and arms Activate?, the second ENT engages the direct course. CLR (or the
  // knob push) cancels the window.
  void openDirectToWindow(const std::string& initial = "");
  bool directToWindowOpen() const { return dtoOpen_; }
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
  // Geographic readouts (Pilot's Guide Fig. 5-45 BRG/DIS/CRS): bearing,
  // distance, and the GPS desired course (equal to the bearing until the
  // direct-to is activated) from the present position to the resolved waypoint.
  // Valid only when a match and ownship position are both available.
  bool directToHasGeo() const;
  float directToBearingDeg() const;
  float directToDistanceNm() const;
  // True once the waypoint is confirmed and the Activate? prompt is armed.
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
  // PROC Activate Missed Approach: queues activation on the FMS navigator.
  void requestActivateMissedApproach();
  bool consumeActivateMissedRequest();

  // ---- Page Menu (MENU key on an open PFD popout, Pilot's Guide Fig. 1-10) ----
  // Context-sensitive options for the active popout window. With no popout
  // open, MENU opens the PFD Setup Menu instead. The Page Menu shares the
  // 310x220 lower-right popout shell (WT popout-dialog on the 1024x768 GDU).
  bool pageMenuOpen() const { return pageMenuOpen_; }
  float pageMenuAnim() const { return pageMenuAnim_; }
  int pageMenuItemCount() const {
    return static_cast<int>(pageMenuItems_.size());
  }
  const std::string& pageMenuItemText(int i) const;
  bool pageMenuItemEnabled(int i) const;
  int pageMenuSelected() const { return pageMenuSel_; }
  int pageMenuScrollOffset() const { return pageMenuScroll_; }

  // ---- PFD Setup Menu state ----
  // FMS-cursor field currently highlighted in the PFD Setup Menu.
  PfdSetupField pfdSetupCursor() const { return setupCursor_; }
  // Per-row backlight target (Display / Key), mode (Auto / Manual), and the
  // stored intensity percentage. Row is a PfdSetupRow (Pfd / Mfd).
  BacklightTarget pfdSetupTarget(PfdSetupRow row) const {
    return setupTarget_[static_cast<int>(row)];
  }
  BacklightMode pfdSetupMode(PfdSetupRow row) const {
    return setupMode_[static_cast<int>(row)];
  }
  float pfdSetupIntensityPct(PfdSetupRow row) const {
    return setupIntensity_[static_cast<int>(row)];
  }

  // ---- Transponder ----
  // True while the 18-second IDNT annunciation is active in the Transponder
  // Data Box (Pilot's Guide, Ident Function).
  bool identActive() const { return identSecondsLeft_ > 0.0; }
  // Squawk digits typed so far on the XPDR > Code softkeys; empty when no code
  // entry is in progress. The Transponder Data Box shows the entry in place of
  // the active code while typing.
  const std::string& xpdrPendingCode() const { return xpdrPending_; }

  // Visual treatment of the Selected Altitude readout for the current Altitude
  // Alerting phase (resolved against the blink clock).
  SelectedAltStyle selectedAltStyle() const;

  // Crew Alerting System annunciations (short codes like "LOW VACUUM") that are
  // always drawn in the PFD annunciation window, independent of whether the
  // pop-up Alerts window is open.
  const std::vector<AlertMessage>& annunciations() const {
    return annunciations_;
  }

  // When active, a persistent advisory annunciation is added to the always-on
  // CAS window telling the pilot the displays are running on the built-in demo
  // feed and how to leave it. Set by the standalone shell's demo-feed toggle.
  void setDemoBanner(bool active) { demoBanner_ = active; }

  // Which softkey menu is currently displayed (top of the menu stack).
  SoftkeyMenu currentMenu() const { return menuStack_.back(); }
  // State of a display option for gauges that want to honor it.
  bool displayToggle(DisplayToggle t) const {
    return toggles_[static_cast<int>(t)];
  }
  // Transponder mode selected on the XPDR submenu.
  XpdrMode xpdrMode() const { return xpdrMode_; }

  // CDI navigation source the pilot has cycled with the root CDI softkey
  // (GPS -> VOR1 -> VOR2). Until the key is pressed the display follows the
  // source reported by the data feed; once cycled, the chosen source is held.
  // The engine resolves the effective source the HSI draws with this.
  CdiSource cdiSourceFor(CdiSource feedSource) const {
    return cdiOverride_ ? cdiSource_ : feedSource;
  }

  // PFD map layout (Map/HSI > Layout) and the derived inset-map visibility used
  // by the PFD inset map gauge.
  MapLayout mapLayout() const { return mapLayout_; }
  bool insetMapVisible() const { return mapLayout_ == MapLayout::Inset; }
  // HSI Map layout (Map/HSI > Layout > HSI Map): the moving map is shown
  // around the HSI compass rose instead of in the lower-left inset.
  bool hsiMapVisible() const { return mapLayout_ == MapLayout::Hsi; }
  // Inset-map declutter level (Map/HSI > Detail softkey cycle).
  MapDetail mapDetail() const { return mapDetail_; }
  // Wind display option (PFD Opt > Wind) honored by the HSI wind box.
  WindOption windOption() const { return windOption_; }

  // Slow ~1 Hz blink phase (true for the first half of each second) used to
  // flash alerting annunciations such as the Alerts softkey and the Baro
  // Transition Alert.
  bool blinkOn() const { return blinkOn_; }
  // True when softkey cell i should flash as an alert annunciation (currently
  // only the Alerts/Warning/Caution/Messages cell).
  bool softkeyFlashing(int i) const;
  // Severity that the flashing Alerts softkey is annunciating, so the renderer
  // can color it (Warning red, Caution amber, Advisory white).
  AlertLevel alertSoftkeyLevel() const { return alertSoftkeyLevel_; }

  // ---- window bezel key column (drawn by the standalone shell) ----
  // Apply a press of a hardware bezel key: flashes the key and, for the range
  // rocker, steps the PFD inset-map range.
  void pressBezelKey(BezelKey key);
  // Press-flash levels (0..1) for the bezel keys, indexed by BezelKey.
  const float* bezelPressLevels() const { return bezelPress_.data(); }
  // Range the bezel rocker drives for the PFD inset map (the selected ladder
  // step; labels the readout and gates declutter).
  float insetRangeNm() const { return mapRangeNmAt(insetRangeIndex_); }
  // Animated zoom scale, in NM, easing toward insetRangeNm() each update().
  // Passed to the map renderer as the on-screen scale so the inset/HSI map
  // glides between ladder steps (Working Title G1000 NXi smooth zoom).
  float insetDisplayRangeNm() const { return insetDisplayRangeNm_; }

  // NAV/COM bezel tuning (Pilot's Guide, Audio Panel): FMS push cycles the
  // selected radio, the small knob steps its standby frequency, and ENT swaps
  // active and standby (the cyan transfer arrow). Returns true when consumed.
  bool radioBezelKey(BezelKey key, const FlightData& d);
  // True when no pop-up window or the Direct-To window is using the FMS knob on
  // the PFD.
  bool canUseRadioBezel() const {
    return window_ == PfdWindow::None && !dtoOpen_ && !pageMenuOpen_;
  }
  // True while a PFD pop-up (Direct-To, Flight Plan, etc.) owns the GCU / FMS
  // knob, ENT, and CLR instead of the MFD.
  bool pfdClaimsFmsInput() const { return !canUseRadioBezel(); }
  // GCU alphanumeric keypad (A-Z, 0-9, backspace) during waypoint entry.
  bool applyGcuEntryKey(char ch);
  RadioUnit radioSelected() const { return radioSelected_; }

  // Dedicated NAV/COM tuning knobs (the real GDU has a COM knob and a NAV knob,
  // each with its own 1/2 toggle, inner/outer tuning rings and flip-flop). The
  // COM and NAV cursors are independent, so selectCom/selectNav toggle which
  // unit each side is tuning. tuneCom/tuneNav step the selected standby (coarse
  // = outer knob = whole MHz; fine = inner knob = one channel). transferCom/
  // transferNav swap that side's active and standby.
  void selectCom();
  void selectNav();
  void tuneCom(int direction, bool coarse, const FlightData& d);
  void tuneNav(int direction, bool coarse, const FlightData& d);
  void transferCom(const FlightData& d);
  void transferNav(const FlightData& d);

  // Dedicated COM VOL/SQ and NAV VOL/ID knobs (Pilot's Guide Fig. 4-3 / 4-8).
  // Turning steps the selected radio's audio volume one click (0..100%, queued
  // as a sim write) and shows the level in place of its standby frequency for
  // two seconds. The NAV VOL/ID push toggles the selected NAV's Morse ident
  // audio (queued as a sim write; white "ID" while on). The COM VOL/SQ push
  // would toggle automatic squelch on the real unit, but X-Plane has no squelch
  // dataref, so it is inert.
  void adjustComVolume(int direction, const FlightData& d);
  void adjustNavVolume(int direction, const FlightData& d);
  void toggleNavIdent(const FlightData& d);

  // Volume indication shown in the NavCom box: true while the given band's
  // percentage is replacing the selected radio's standby frequency. The renderer
  // queries radioVolumeUnit()/radioVolumePct() for the value to draw.
  bool radioVolumeShown(RadioBand band) const;
  bool radioVolumeAnnunciationActive() const {
    return radioVolumeShownSeconds_ > 0.0;
  }
  double radioVolumeSecondsLeft() const { return radioVolumeShownSeconds_; }
  RadioUnit radioVolumeUnit() const { return radioVolumeUnit_; }
  int radioVolumePct() const { return radioVolumePct_; }
  // Copy the transient VOL percentage readout to the other GDU's radio bar.
  void mirrorRadioVolumeAnnunciation(const SoftkeyController& src);

  // Dedicated HDG knob (left bezel) and CRS/BARO knob (right bezel). These step
  // the selected-heading bug, selected course, and altimeter barometric setting
  // by one click and queue the new value as a sim write, consumed like the
  // radio/transponder commits below. The push caps sync the bug/course to the
  // current heading.
  void adjustHeadingBug(int direction, const FlightData& d);
  void syncHeadingBug(const FlightData& d);
  void adjustCourse(int direction, const FlightData& d);
  void adjustBaro(int direction, const FlightData& d);
  // CRS/BARO knob push ("PUSH STD"): set the altimeter to standard pressure.
  void setBaroStandard();

  // Flash a bezel control's press-feedback without any other effect (the audio
  // VOL/SQ/ID knobs, which the display does not model beyond the animation).
  void flashBezelKey(BezelKey key);

  // Currently selected COM and NAV unit (each side's cyan tuning cursor).
  RadioUnit comSelected() const { return comSelected_; }
  RadioUnit navSelected() const { return navSelected_; }
  // The band whose tuning cursor is "armed" (flashes for a few seconds after a
  // selection/tuning action, like the real unit) and whether it is still
  // within that flash window. The renderer flashes the selector box while
  // armed and draws it solid otherwise.
  RadioBand radioArmedBand() const { return this->radioArmedBand_; }
  bool radioArmed() const { return radioArmedSeconds_ > 0.0; }

  // Active/standby transfer slide animation (0..1 for `unit`, 0 when idle).
  float radioTransferAnim(RadioUnit unit) const;
  float radioTransferFromActive(RadioUnit unit) const;
  float radioTransferFromStandby(RadioUnit unit) const;
  int radioTransferDecimals(RadioUnit unit) const;

  // Transponder commits for the sim feed. Mode values use the X-Plane
  // transponder_mode enum (off=0, stdby=1, on=2, alt=3).
  bool consumeXpdrCodeCommit(int& code);
  bool consumeXpdrModeCommit(int& mode);

  // NAV/COM commits for the sim feed.
  bool consumeRadioTune(RadioUnit& unit, float& standbyMhz);
  bool consumeRadioTransfer(RadioUnit& unit);
  // Audio-volume commit for the sim feed (0..1 ratio for the audio_volume_*
  // dataref of `unit`).
  bool consumeRadioVolume(RadioUnit& unit, float& volume);
  // NAV Morse-ident audio commit for the sim feed (audio_selection_nav* of
  // `unit`, on/off), queued by the NAV VOL/ID knob press.
  bool consumeNavIdent(RadioUnit& unit, bool& on);

  // HDG bug / selected course / baro commits for the sim feed (degrees magnetic
  // and inches of mercury).
  bool consumeHeadingBug(float& deg);
  bool consumeCourse(float& deg);
  bool consumeBaro(float& inHg);

 private:
  // The persistence helpers read/write the durable display options directly.
  friend void capturePfdState(const SoftkeyController&, PfdPersistentState&);
  friend void applyPfdState(SoftkeyController&, const PfdPersistentState&);
  friend void syncRadioVolumeAnnunciation(SoftkeyController&, SoftkeyController&);

  void rebuildAlerts(const FlightData& data);
  // Refresh the visible cell labels from the menu now on top of the stack.
  void rebuildLabels();
  // Toggle a PFD pop-up window: opens it (replacing any other) or closes it if
  // it is already the active one.
  void toggleWindow(PfdWindow w);
  // Rebuild the Nearest Airports list from the map snapshot (airports sorted
  // by distance from ownship).
  void rebuildNearest(const MapData& map);
  // Direct-To window (Direct-To bezel key): open the window seeded with the
  // active waypoint, then route the FMS knob / ENT / CLR while it is open.
  // Returns true when the key was consumed by the Direct-To window.
  bool directToBezelKey(BezelKey key);
  void directToOpen(const std::string& initial = "");
  // Active Flight Plan window (FPL bezel key): route the FMS knob / ENT / CLR /
  // MENU while the window is open. Returns true when the key was consumed so it
  // does not also scroll/close. Adopts external plan changes each frame and
  // publishes pilot edits to the shell.
  bool flightPlanBezelKey(BezelKey key);
  void flightPlanCommitEntry();
  void requestActivateFlightPlanLeg(int toLegIndex);
  // Direct-To a plan leg while keeping the route (FPL ENT / approach activate).
  void requestDirectToFlightPlanLeg(int legIndex);
  // Ident of the waypoint highlighted on the FPL window (empty when none).
  std::string flightPlanSelectedLegIdent() const;
  int flightPlanSelectedLegIndex() const;
  // Replace the editable plan with a single Direct-To leg (clears approach
  // metadata) and publish the edit to the data source.
  void flightPlanApplyDirectTo(const MapLeg& target);
  FmsWaypointEntry* activeWaypointEntry();
  void syncFlightPlanLegs(const MapData& map, bool navDirectTo = false);
  void tryRestorePersistedApproach();
  void reinferApproachFromProcedureLegs();
  // Procedures window (PROC bezel key): build the top-level menu on open, route
  // the FMS knob / ENT / CLR while it is open, move the menu cursor (skipping
  // disabled rows), and load the selected procedure's legs into the plan.
  void buildProcMenu();
  bool procBezelKey(BezelKey key);
  // Modal "Fly Course Reversal?" prompt: owns the FMS knob / ENT / CLR while up.
  bool courseReversalPromptBezelKey(BezelKey key);
  // Modal hold Direct-To confirmation (Activate/Cancel) on a selected HOLD row.
  bool holdActivatePromptBezelKey(BezelKey key);
  void openHoldActivatePrompt(int legIndex);
  void closeHoldActivatePrompt();
  std::vector<std::string> procProcedureNames(ProcedureType type) const;
  std::vector<std::string> procTransitions(ProcedureType type,
                                           const std::string& name) const;
  enum class PfdPageMenuAction {
    Disabled,
    RefAllOn,
    RefAllOff,
    RefRestoreDefaults,
  };
  struct PfdPageMenuItem {
    std::string text;
    PfdPageMenuAction action = PfdPageMenuAction::Disabled;
  };
  std::vector<PfdPageMenuItem> buildPfdPageMenu() const;
  void openPfdPageMenu();
  bool pageMenuBezelKey(BezelKey key);
  void pageMenuStep(int direction);
  void pageMenuActivate();
  // Advance the Selected Altitude alerting state machine (Pilot's Guide,
  // Altitude Alerting).
  void updateAltAlert(double dtSeconds, const FlightData& data);
  // FMS-cursor movement / ENT activation inside the References window. The
  // large knob moves the cursor, the small knob steps the highlighted value.
  void moveReferencesCursor(int step);
  void adjustReferencesValue(int step);
  void activateReferencesField();
  // PFD Setup Menu (MENU key): large knob moves the cursor (skipping a row's
  // intensity field unless that row is in Manual), small knob edits the
  // highlighted field, ENT confirms (advancing onto a row's intensity once it
  // is set to Manual). Mirrors the Pilot's Guide backlighting procedures.
  bool pfdSetupFieldReachable(PfdSetupField field) const;
  void movePfdSetupCursor(int step);
  void adjustPfdSetupValue(int step);
  void activatePfdSetupField();
  // Start the 18-second IDNT annunciation; from the XPDR menus this also
  // reverts to the top-level softkeys, like the real unit.
  void startIdent();
  void queueXpdrModeCommit();
  static int xpdrModeToSim(XpdrMode mode);
  float standbyMhzFor(RadioUnit unit, const FlightData& d) const;
  void setStandbyMhzFor(RadioUnit unit, float mhz);
  void queueRadioTune(RadioUnit unit, float standbyMhz);
  void queueRadioTransfer(RadioUnit unit, const FlightData& d);
  // Step `unit`'s audio volume one click, queue the write, and show the level.
  void adjustRadioVolume(RadioUnit unit, RadioBand band, int direction,
                         const FlightData& d);
  void cycleRadioSelect();
  // Flash the given band's tuning cursor for kRadioArmedSeconds.
  void armRadioBand(RadioBand band);

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};

  // Menu navigation: the stack always has the root menu at the bottom; opening
  // a submenu pushes it, "Back" pops it.
  std::vector<SoftkeyMenu> menuStack_;
  std::array<bool, kDisplayToggleCount> toggles_{};
  XpdrMode xpdrMode_ = XpdrMode::Alt;

  // CDI source the root CDI softkey cycles. cdiOverride_ stays false until the
  // pilot first presses CDI, so the display tracks the feed by default;
  // lastCdiFeed_ remembers the feed's source so the first press cycles from it.
  CdiSource cdiSource_ = CdiSource::Gps;
  bool cdiOverride_ = false;
  CdiSource lastCdiFeed_ = CdiSource::Gps;
  MapLayout mapLayout_ = MapLayout::Inset;
  MapDetail mapDetail_ = MapDetail::All;
  WindOption windOption_ = WindOption::Option2;

  // ~1 Hz blink phase for flashing alert annunciations.
  double blinkSeconds_ = 0.0;
  bool blinkOn_ = true;

  // On-screen bezel key column: per-key press-flash and the inset-map range it
  // controls (the range rocker steps kMapRangeLadderNm).
  std::array<float, kBezelKeyCount> bezelPress_{};
  int insetRangeIndex_ = kMapRangeDefaultIndex;
  // Animated scale eased toward mapRangeNmAt(insetRangeIndex_) by update();
  // seeded to the default so the first frame is already at the right zoom.
  float insetDisplayRangeNm_ = mapRangeNmAt(kMapRangeDefaultIndex);

  // Active PFD pop-up window and the per-window open/close animations (indexed
  // by PfdWindow; the None slot is unused).
  PfdWindow window_ = PfdWindow::None;
  std::array<float, kPfdWindowCount> windowAnim_{};
  std::vector<AlertMessage> alerts_;
  std::vector<AlertMessage> annunciations_;
  // Demo-feed banner: when set, rebuildAlerts() injects a persistent advisory
  // into the always-on CAS window (see setDemoBanner()).
  bool demoBanner_ = false;

  // Timer/References window state: FMS cursor, generic timer, V-speed bug
  // enables (all on by default, like the delivered unit), and minimums.
  RefField refCursor_ = RefField::TimerCmd;
  bool timerRunning_ = false;
  double timerSeconds_ = 0.0;
  std::array<bool, kVspeedRefCount> vspeedOn_{true, true, true, true};
  std::array<float, kVspeedRefCount> vspeedKt_{
      kDefaultVspeedKt[0], kDefaultVspeedKt[1], kDefaultVspeedKt[2],
      kDefaultVspeedKt[3]};
  MinimumsMode minsMode_ = MinimumsMode::Off;
  float minsAltFt_ = 0.0f;
  float minsTempC_ = kMinsTempDefaultC;

  const NavFeatureSource* navSource_ = nullptr;

  // Nearest Airports window: distance-sorted list and the FMS cursor index.
  std::vector<NearestAirport> nearest_;
  int nearestCursor_ = 0;

  // Active Flight Plan window editing state (mirrors the MFD FPL page, minus the
  // VNAV ALT column the PFD window does not show). fplLegs_ is the working copy;
  // fplLastMapPlan_ detects external plan changes and fplLastPublished_ keeps
  // the data source's echo of our own edit from being re-adopted.
  std::vector<MapLeg> fplLegs_;
  std::vector<MapLeg> fplLastMapPlan_;
  std::vector<MapLeg> fplLastPublished_;
  bool fplEditPending_ = false;
  bool fplCursorOn_ = false;
  bool fplListCursorFollowsActive_ = true;
  bool fplDestinationFilled_ = false;
  bool fplLocalDraft_ = false;
  int fplCursorRow_ = 0;
  FmsWaypointEntry fplEntry_;
  FplConfirm fplConfirm_ = FplConfirm::None;
  bool fplConfirmOk_ = true;
  std::string fplRemoveIdent_;

  // Procedures window (PROC key): shared menu state; logic in ProcedureMenu.cpp.
  ProcedureMenuState procMenu_;
  MapProcedure fplLoadedApproach_{};
  int fplApproachLegStart_ = 0;
  int fplApproachLegCount_ = 0;
  PersistedLoadedApproach persistedApproachRestore_{};
  // A restored plan keeps only per-leg roles; the procedure's holds, altitudes,
  // and glidepath are re-attached by re-expanding the CIFP approach once nav data
  // is ready. Set on restore, cleared once the re-expansion has been applied (or
  // is known to be unmatchable).
  bool fplApproachRestorePending_ = false;
  MapProcedure procSelectedProcedure() const;
  std::string formatApproachLabel(const MapProcedure& proc) const;
  std::string procDefaultAirportIcao() const;
  ProcedureMenuHost procedureMenuHost();
  ProcedureMenuHost procedureMenuHost() const;

  // Direct-To window state. Latest map snapshot (for ident lookups / geographic
  // readouts) and the active flight-plan waypoint (the default destination) are
  // cached each update().
  const MapData* mapData_ = nullptr;
  std::string activeWaypoint_;
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
  bool missedActivatePending_ = false;
  // Direct-To from a highlighted FPL leg: keep the plan and fly direct to that
  // fix (skip), rather than replacing the plan with a single leg.
  bool dtoPreservePlan_ = false;
  int dtoPreserveLegIndex_ = -1;

  // Page Menu (MENU on an open popout): option list for the active window.
  std::vector<PfdPageMenuItem> pageMenuItems_;
  int pageMenuSel_ = 0;
  int pageMenuScroll_ = 0;
  bool pageMenuOpen_ = false;
  float pageMenuAnim_ = 0.0f;

  // PFD Setup Menu state: the highlighted field plus each row's backlight
  // target, mode, and intensity. Backlighting has no visible effect in this
  // suite (the rendered display is not dimmed), so the intensities are stored
  // display-only; the menu reproduces the real layout and FMS-knob behavior.
  // Opens in Auto, with the cursor on the PFD mode field, like the real unit.
  PfdSetupField setupCursor_ = PfdSetupField::PfdMode;
  std::array<BacklightTarget, kPfdSetupRowCount> setupTarget_{
      BacklightTarget::Display, BacklightTarget::Display};
  std::array<BacklightMode, kPfdSetupRowCount> setupMode_{BacklightMode::Auto,
                                                          BacklightMode::Auto};
  std::array<float, kPfdSetupRowCount> setupIntensity_{85.0f, 85.0f};

  // Transponder: remaining IDNT annunciation time and the in-progress code
  // entry digits.
  double identSecondsLeft_ = 0.0;
  std::string xpdrPending_;
  bool xpdrCodeCommitPending_ = false;
  int xpdrCodeCommit_ = 1200;
  bool xpdrModeCommitPending_ = false;
  int xpdrModeCommit_ = 3;

  // NAV/COM tuning state. radioSelected_ is the unified focus the FMS knob
  // cycles (standalone bezel); comSelected_/navSelected_ are the per-side
  // cursors the dedicated COM/NAV knobs toggle and the bar draws.
  RadioUnit radioSelected_ = RadioUnit::Nav1;
  RadioUnit comSelected_ = RadioUnit::Com1;
  RadioUnit navSelected_ = RadioUnit::Nav1;
  bool radioTunePending_ = false;
  RadioUnit radioTuneUnit_ = RadioUnit::Nav1;
  float radioTuneMhz_ = 0.0f;
  bool radioTransferPending_ = false;
  RadioUnit radioTransferUnit_ = RadioUnit::Nav1;
  // Flashing tuning-cursor ("armed") state: which band is armed and how long
  // its selector box keeps flashing before settling solid.
  static constexpr double kRadioArmedSeconds = 5.0;
  RadioBand radioArmedBand_ = RadioBand::None;
  double radioArmedSeconds_ = 0.0;

  // Audio volume (VOL/SQ, VOL/ID knobs). The percentage replaces the selected
  // radio's standby frequency for kRadioVolumeShownSeconds after a turn;
  // radioVolume*Commit_ is the pending sim write. navIdent*Commit_ is the
  // pending sim write for the NAV VOL/ID push (the "ID" state itself is read
  // back from FlightData's nav?IdentAudio, so the annunciation tracks the sim).
  double radioVolumeShownSeconds_ = 0.0;
  RadioBand radioVolumeBand_ = RadioBand::None;
  RadioUnit radioVolumeUnit_ = RadioUnit::Com1;
  int radioVolumePct_ = 0;
  // Incremented on each knob turn so peer sync can tell a fresh annunciation
  // from a side that already dismissed it (timer at 0) but has not ticked yet.
  std::uint32_t radioVolumeEpoch_ = 0;
  bool radioVolumePending_ = false;
  RadioUnit radioVolumeCommitUnit_ = RadioUnit::Com1;
  float radioVolumeCommitValue_ = 0.0f;
  bool navIdentPending_ = false;
  RadioUnit navIdentCommitUnit_ = RadioUnit::Nav1;
  bool navIdentCommitValue_ = false;

  // Brief slide when active and standby swap (Pilot's Guide flip-flop).
  struct RadioXferAnim {
    RadioUnit unit = RadioUnit::Nav1;
    float progress = 0.0f;
    float fromActiveMhz = 0.0f;
    float fromStandbyMhz = 0.0f;
    int decimals = 2;
  };
  static constexpr double kRadioTransferAnimSeconds = 0.30;
  bool radioXferAnimActive_ = false;
  RadioXferAnim radioXferAnim_{};

  // Pending HDG bug / selected course / baro writes from the dedicated knobs,
  // committed to the sim feed by the shell (consumeHeadingBug/Course/Baro).
  bool headingBugPending_ = false;
  float headingBugDeg_ = 0.0f;
  bool coursePending_ = false;
  float coursePendingDeg_ = 0.0f;
  bool baroPending_ = false;
  float baroPendingInHg_ = 0.0f;

  // Selected Altitude alerting state: phase, remaining flash time for the
  // current phase's five-second flash, and the reference the alerter was last
  // armed against (a new selection re-arms it).
  AltAlertPhase altAlertPhase_ = AltAlertPhase::Armed;
  double altAlertFlashLeft_ = 0.0;
  float altAlertSelectedFt_ = 0.0f;
  float altAlertDiffFt_ = 0.0f;

  // Dynamic Alerts-softkey annunciation: when unacknowledged alerts are
  // present, the rightmost root softkey relabels to Warning/Caution/Messages
  // and flashes (G1000 NXi Pilot's Guide, Appendix A).
  bool alertSoftkeyActive_ = false;
  AlertLevel alertSoftkeyLevel_ = AlertLevel::Advisory;

  // Acknowledged alerts, keyed by message text and kept across the per-frame
  // rebuild so the Warning/Caution softkey (annunciation-window CAS messages)
  // and the Messages softkey (Alerts-window advisories) stop flashing once the
  // pilot presses them. Entries whose condition has cleared are pruned in
  // rebuildAlerts so a later re-occurrence flashes again.
  std::set<std::string> acknowledgedAnnunciations_;
  std::set<std::string> acknowledgedMessages_;

  // Handles a press of the dynamic Alerts/Warning/Caution/Messages softkey:
  // while it is flashing, the press acknowledges the displayed alert level
  // (and, for Messages, opens the Alerts window); otherwise it toggles the
  // Alerts window (Pilot's Guide, Appendix A).
  void pressAlertsSoftkey();

  // Recomputes the dynamic Alerts-softkey label and flash state from the
  // current (unacknowledged) alert lists; called each frame after the alert
  // lists are rebuilt.
  void updateAlertSoftkey();
};

// Keep the transient VOL readout in sync on both GDUs.
void syncRadioVolumeAnnunciation(SoftkeyController& a, SoftkeyController& b);

}  // namespace avionics
