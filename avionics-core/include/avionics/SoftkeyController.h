#pragma once

#include <array>
#include <string>
#include <vector>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {

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
// time in that region (Alerts, Timer/References, Nearest Airports); opening
// one replaces any other.
enum class PfdWindow { None, Alerts, References, Nearest };
inline constexpr int kPfdWindowCount = 4;  // including None

// Pilot-selectable V-speed reference bugs, in the row order of the NXi
// Timer/References window (Pilot's Guide Fig. 2-6 / Table 2-1).
enum class VspeedRef { Glide, Vr, Vx, Vy, Count };
inline constexpr int kVspeedRefCount = static_cast<int>(VspeedRef::Count);

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
  MinsMode,   // Off / BARO
  MinsValue,  // MDA/DH altitude (only reachable when MinsMode is BARO)
  Count,
};

// Minimums (MDA/DH) source selected in the References window. TEMP COMP is not
// fitted on this airframe, so the choice is Off or barometric.
enum class MinimumsMode { Off, Baro };

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
  // Minimums (MDA/DH): mode and barometric altitude set in the References
  // window. The altimeter draws the BARO MIN box and bug from these.
  MinimumsMode minimumsMode() const { return minsMode_; }
  float minimumsAltitudeFt() const { return minsAltFt_; }

  // ---- Nearest Airports window state ----
  const std::vector<NearestAirport>& nearestAirports() const {
    return nearest_;
  }
  // Index of the FMS-cursor-selected entry in nearestAirports().
  int nearestCursor() const { return nearestCursor_; }

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

  // Which softkey menu is currently displayed (top of the menu stack).
  SoftkeyMenu currentMenu() const { return menuStack_.back(); }
  // State of a display option for gauges that want to honor it.
  bool displayToggle(DisplayToggle t) const {
    return toggles_[static_cast<int>(t)];
  }
  // Transponder mode selected on the XPDR submenu.
  XpdrMode xpdrMode() const { return xpdrMode_; }

  // PFD map layout (Map/HSI > Layout) and the derived inset-map visibility used
  // by the PFD inset map gauge.
  MapLayout mapLayout() const { return mapLayout_; }
  bool insetMapVisible() const { return mapLayout_ == MapLayout::Inset; }
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
  // Range the bezel rocker drives for the PFD inset map.
  float insetRangeNm() const { return mapRangeNmAt(insetRangeIndex_); }

 private:
  void rebuildAlerts(const FlightData& data);
  // Refresh the visible cell labels from the menu now on top of the stack.
  void rebuildLabels();
  // Toggle a PFD pop-up window: opens it (replacing any other) or closes it if
  // it is already the active one.
  void toggleWindow(PfdWindow w);
  // Rebuild the Nearest Airports list from the map snapshot (airports sorted
  // by distance from ownship).
  void rebuildNearest(const MapData& map);
  // Advance the Selected Altitude alerting state machine (Pilot's Guide,
  // Altitude Alerting).
  void updateAltAlert(double dtSeconds, const FlightData& data);
  // FMS-cursor movement / ENT activation inside the References window.
  void moveReferencesCursor(int step);
  void activateReferencesField();
  // Start the 18-second IDNT annunciation; from the XPDR menus this also
  // reverts to the top-level softkeys, like the real unit.
  void startIdent();

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};

  // Menu navigation: the stack always has the root menu at the bottom; opening
  // a submenu pushes it, "Back" pops it.
  std::vector<SoftkeyMenu> menuStack_;
  std::array<bool, kDisplayToggleCount> toggles_{};
  XpdrMode xpdrMode_ = XpdrMode::Alt;
  MapLayout mapLayout_ = MapLayout::Inset;
  WindOption windOption_ = WindOption::Option2;

  // ~1 Hz blink phase for flashing alert annunciations.
  double blinkSeconds_ = 0.0;
  bool blinkOn_ = true;

  // On-screen bezel key column: per-key press-flash and the inset-map range it
  // controls (the range rocker steps kMapRangeLadderNm).
  std::array<float, kBezelKeyCount> bezelPress_{};
  int insetRangeIndex_ = kMapRangeDefaultIndex;

  // Active PFD pop-up window and the per-window open/close animations (indexed
  // by PfdWindow; the None slot is unused).
  PfdWindow window_ = PfdWindow::None;
  std::array<float, kPfdWindowCount> windowAnim_{};
  std::vector<AlertMessage> alerts_;
  std::vector<AlertMessage> annunciations_;

  // Timer/References window state: FMS cursor, generic timer, V-speed bug
  // enables (all on by default, like the delivered unit), and minimums.
  RefField refCursor_ = RefField::TimerCmd;
  bool timerRunning_ = false;
  double timerSeconds_ = 0.0;
  std::array<bool, kVspeedRefCount> vspeedOn_{true, true, true, true};
  MinimumsMode minsMode_ = MinimumsMode::Off;
  float minsAltFt_ = 0.0f;

  // Nearest Airports window: distance-sorted list and the FMS cursor index.
  std::vector<NearestAirport> nearest_;
  int nearestCursor_ = 0;

  // Transponder: remaining IDNT annunciation time and the in-progress code
  // entry digits. Committing the completed code to the radio is owned by the
  // sim feed (no command channel yet), matching the VFR/STD Baro keys.
  double identSecondsLeft_ = 0.0;
  std::string xpdrPending_;

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

  // Recomputes the dynamic Alerts-softkey label and flash state from the
  // current alert lists; called each frame after the alert lists are rebuilt.
  void updateAlertSoftkey();
};

}  // namespace avionics
