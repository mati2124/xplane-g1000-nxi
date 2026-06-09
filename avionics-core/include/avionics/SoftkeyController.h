#pragma once

#include <array>
#include <string>
#include <vector>

#include "avionics/FlightData.h"

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

// The softkey bar is a menu stack: the root (top-level) menu can open submenus,
// and every submenu carries a "Back" key that pops back to its parent. These
// identify which menu is currently shown.
enum class SoftkeyMenu { Root, MapHsi, PfdOpt, Xpdr };

// Transponder operating mode selected from the XPDR softkey submenu. The four
// keys form a radio group: selecting one deselects the others, and the chosen
// key stays highlighted.
enum class XpdrMode { Standby, On, Alt, Ground };

// Display options the softkeys turn on and off. They are surfaced to the
// renderer through keyActive() (the cell highlights while its option is on) and
// can be read by the gauges as they grow to honor these options. Count must
// remain last so it sizes the backing array.
enum class DisplayToggle {
  Traffic,
  Inset,
  Obs,
  Dme,
  Svt,
  Wind,
  Bearing1,
  Bearing2,
  HsiFormat,
  AltMeters,
  StdBaro,
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

  // Advance animations and refresh the alert list from the current flight data.
  void update(double dtSeconds, const FlightData& data);

  // Forward a pointer press in display-pixel coordinates. Returns true if it
  // landed on an interactive element (and was therefore consumed).
  bool pointerDown(float xPx, float yPx, float widthPx, float heightPx);

  // ---- read by the renderer ----
  const std::string& label(int i) const { return labels_[i]; }
  // 0..1 press-flash brightness for the key-press highlight animation.
  float pressLevel(int i) const { return press_[i]; }
  // True while the cell's associated window/mode is the active one.
  bool keyActive(int i) const;

  // Alerts window: linear animation progress in 0..1 (0 = hidden, 1 = fully
  // shown). The window should be drawn whenever this is > 0. The renderer eases
  // the raw value for the visual slide/fade.
  float alertsAnim() const { return alertsAnim_; }
  bool alertsOpen() const { return alertsOpen_; }
  const std::vector<AlertMessage>& alerts() const { return alerts_; }

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

 private:
  // Maps a pointer position to a softkey cell index, or -1 if outside the bar.
  static int hitTest(float xPx, float yPx, float widthPx, float heightPx);
  void rebuildAlerts(const FlightData& data);
  // Refresh the visible cell labels from the menu now on top of the stack.
  void rebuildLabels();

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};

  // Menu navigation: the stack always has the root menu at the bottom; opening
  // a submenu pushes it, "Back" pops it.
  std::vector<SoftkeyMenu> menuStack_;
  std::array<bool, kDisplayToggleCount> toggles_{};
  XpdrMode xpdrMode_ = XpdrMode::Alt;

  bool alertsOpen_ = false;
  float alertsAnim_ = 0.0f;
  std::vector<AlertMessage> alerts_;
  std::vector<AlertMessage> annunciations_;
};

}  // namespace avionics
