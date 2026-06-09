#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>

namespace avionics {
namespace {

// Softkey bar height as a fraction of the display: the Working Title NXi GDU
// canvas is 1024x768 with a 35 px softkey row at the bottom.
constexpr float kBarHeightFraction = 35.0f / 768.0f;

// What a softkey cell does when pressed. Kept internal to the controller: the
// renderer only ever sees labels and the keyActive()/pressLevel() highlight
// state, never the action.
enum class SoftkeyAction {
  None,        // blank cell, ignores presses
  Momentary,   // press-flash only; not yet wired to a function
  Back,        // pop to the parent menu
  OpenMapHsi,  // push the Map/HSI submenu
  OpenPfdOpt,  // push the PFD Options submenu
  OpenXpdr,    // push the transponder submenu
  ToggleAlerts,
  ToggleTraffic,
  ToggleInset,
  ToggleObs,
  ToggleDme,
  ToggleSvt,
  ToggleWind,
  ToggleBearing1,
  ToggleBearing2,
  ToggleHsiFormat,
  ToggleAltMeters,
  ToggleStdBaro,
  XpdrStandby,
  XpdrOn,
  XpdrAlt,
  XpdrGround,
};

struct SoftkeyDef {
  const char* label;
  SoftkeyAction action;
};

using Menu = std::array<SoftkeyDef, kSoftkeyCount>;

// Root (top-level) PFD menu. Cell 0 is intentionally blank, matching the G1000.
constexpr Menu kRootMenu = {{
    {"", SoftkeyAction::None},
    {"Map/HSI", SoftkeyAction::OpenMapHsi},
    {"TFC Map", SoftkeyAction::ToggleTraffic},
    {"PFD Opt", SoftkeyAction::OpenPfdOpt},
    {"OBS", SoftkeyAction::ToggleObs},
    {"CDI", SoftkeyAction::Momentary},
    {"DME", SoftkeyAction::ToggleDme},
    {"XPDR", SoftkeyAction::OpenXpdr},
    {"Ident", SoftkeyAction::Momentary},
    {"Tmr/Ref", SoftkeyAction::Momentary},
    {"Nearest", SoftkeyAction::Momentary},
    {"Alerts", SoftkeyAction::ToggleAlerts},
}};

constexpr Menu kMapHsiMenu = {{
    {"", SoftkeyAction::None},
    {"TFC Map", SoftkeyAction::ToggleTraffic},
    {"Inset", SoftkeyAction::ToggleInset},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

constexpr Menu kPfdOptMenu = {{
    {"", SoftkeyAction::None},
    {"SVT", SoftkeyAction::ToggleSvt},
    {"Wind", SoftkeyAction::ToggleWind},
    {"DME", SoftkeyAction::ToggleDme},
    {"BRG1", SoftkeyAction::ToggleBearing1},
    {"BRG2", SoftkeyAction::ToggleBearing2},
    {"HSI Frmt", SoftkeyAction::ToggleHsiFormat},
    {"", SoftkeyAction::None},
    {"ALT Unit", SoftkeyAction::ToggleAltMeters},
    {"STD Baro", SoftkeyAction::ToggleStdBaro},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

constexpr Menu kXpdrMenu = {{
    {"", SoftkeyAction::None},
    {"STBY", SoftkeyAction::XpdrStandby},
    {"ON", SoftkeyAction::XpdrOn},
    {"ALT", SoftkeyAction::XpdrAlt},
    {"GND", SoftkeyAction::XpdrGround},
    {"VFR", SoftkeyAction::Momentary},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Ident", SoftkeyAction::Momentary},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

const Menu& menuDefs(SoftkeyMenu menu) {
  switch (menu) {
    case SoftkeyMenu::MapHsi:
      return kMapHsiMenu;
    case SoftkeyMenu::PfdOpt:
      return kPfdOptMenu;
    case SoftkeyMenu::Xpdr:
      return kXpdrMenu;
    case SoftkeyMenu::Root:
      break;
  }
  return kRootMenu;
}

// Maps a toggle action to its DisplayToggle index, or -1 if the action is not a
// display toggle.
int toggleIndex(SoftkeyAction action) {
  switch (action) {
    case SoftkeyAction::ToggleTraffic:
      return static_cast<int>(DisplayToggle::Traffic);
    case SoftkeyAction::ToggleInset:
      return static_cast<int>(DisplayToggle::Inset);
    case SoftkeyAction::ToggleObs:
      return static_cast<int>(DisplayToggle::Obs);
    case SoftkeyAction::ToggleDme:
      return static_cast<int>(DisplayToggle::Dme);
    case SoftkeyAction::ToggleSvt:
      return static_cast<int>(DisplayToggle::Svt);
    case SoftkeyAction::ToggleWind:
      return static_cast<int>(DisplayToggle::Wind);
    case SoftkeyAction::ToggleBearing1:
      return static_cast<int>(DisplayToggle::Bearing1);
    case SoftkeyAction::ToggleBearing2:
      return static_cast<int>(DisplayToggle::Bearing2);
    case SoftkeyAction::ToggleHsiFormat:
      return static_cast<int>(DisplayToggle::HsiFormat);
    case SoftkeyAction::ToggleAltMeters:
      return static_cast<int>(DisplayToggle::AltMeters);
    case SoftkeyAction::ToggleStdBaro:
      return static_cast<int>(DisplayToggle::StdBaro);
    default:
      return -1;
  }
}

float approach(float current, float target, float maxStep) {
  if (current < target) return std::min(target, current + maxStep);
  return std::max(target, current - maxStep);
}

// Crew Alerting System messages driven by the X-Plane annunciator conditions in
// FlightData. Each entry maps a condition flag to the text and severity shown
// in the Alerts window. Adding a new CAS message is just another row here (plus
// the dataref that feeds its flag).
struct CasMessageDef {
  bool FlightData::* condition;
  const char* text;
  AlertLevel level;
};

constexpr CasMessageDef kCasMessages[] = {
    {&FlightData::casOilPressureLow, "OIL PRESSURE", AlertLevel::Warning},
    {&FlightData::casLowVoltage, "LOW VOLTS", AlertLevel::Warning},
    {&FlightData::casStallWarning, "STALL", AlertLevel::Warning},
    {&FlightData::casGearUnsafe, "CHECK GEAR", AlertLevel::Warning},
    {&FlightData::casLowVacuum, "LOW VACUUM", AlertLevel::Caution},
    {&FlightData::casFuelLow, "FUEL LOW", AlertLevel::Caution},
    {&FlightData::casOilTempHigh, "OIL TEMP", AlertLevel::Caution},
    {&FlightData::casFuelPressureLow, "FUEL PRESS", AlertLevel::Caution},
    {&FlightData::casPitotHeatOff, "PITOT HEAT", AlertLevel::Caution},
    {&FlightData::casIcing, "ANTI ICE", AlertLevel::Caution},
};

}  // namespace

SoftkeyController::SoftkeyController() : menuStack_{SoftkeyMenu::Root} {
  rebuildLabels();
}

void SoftkeyController::rebuildLabels() {
  const Menu& menu = menuDefs(currentMenu());
  for (int i = 0; i < kSoftkeyCount; ++i) labels_[i] = menu[i].label;
}

void SoftkeyController::update(double dtSeconds, const FlightData& data) {
  const float dt = static_cast<float>(dtSeconds);

  // Key-press flashes decay linearly back to rest.
  const float pressStep = dt / kPressFlashSeconds;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    press_[i] = std::max(0.0f, press_[i] - pressStep);
  }

  // Alerts window eases toward its open/closed target at a constant rate.
  const float target = alertsOpen_ ? 1.0f : 0.0f;
  alertsAnim_ = approach(alertsAnim_, target, dt / kWindowAnimSeconds);

  rebuildAlerts(data);
}

bool SoftkeyController::keyActive(int i) const {
  if (i < 0 || i >= kSoftkeyCount) return false;
  const SoftkeyAction action = menuDefs(currentMenu())[i].action;

  if (action == SoftkeyAction::ToggleAlerts) return alertsOpen_;

  const int toggle = toggleIndex(action);
  if (toggle >= 0) return toggles_[toggle];

  switch (action) {
    case SoftkeyAction::XpdrStandby:
      return xpdrMode_ == XpdrMode::Standby;
    case SoftkeyAction::XpdrOn:
      return xpdrMode_ == XpdrMode::On;
    case SoftkeyAction::XpdrAlt:
      return xpdrMode_ == XpdrMode::Alt;
    case SoftkeyAction::XpdrGround:
      return xpdrMode_ == XpdrMode::Ground;
    default:
      return false;
  }
}

int SoftkeyController::hitTest(float xPx, float yPx, float widthPx,
                               float heightPx) {
  const float barH = heightPx * kBarHeightFraction;
  const float barTop = heightPx - barH;
  if (yPx < barTop || yPx > heightPx || xPx < 0.0f || xPx > widthPx) return -1;
  const float cellW = widthPx / static_cast<float>(kSoftkeyCount);
  int idx = static_cast<int>(xPx / cellW);
  idx = std::max(0, std::min(kSoftkeyCount - 1, idx));
  return idx;
}

bool SoftkeyController::pointerDown(float xPx, float yPx, float widthPx,
                                    float heightPx) {
  const int key = hitTest(xPx, yPx, widthPx, heightPx);
  if (key < 0) return false;

  const SoftkeyAction action = menuDefs(currentMenu())[key].action;
  if (action == SoftkeyAction::None) return false;

  press_[key] = 1.0f;  // trigger the press-flash animation

  const auto openMenu = [this](SoftkeyMenu menu) {
    if (currentMenu() == menu) return;
    menuStack_.push_back(menu);
    rebuildLabels();
  };

  switch (action) {
    case SoftkeyAction::None:
    case SoftkeyAction::Momentary:
      break;
    case SoftkeyAction::Back:
      if (menuStack_.size() > 1) {
        menuStack_.pop_back();
        rebuildLabels();
      }
      break;
    case SoftkeyAction::OpenMapHsi:
      openMenu(SoftkeyMenu::MapHsi);
      break;
    case SoftkeyAction::OpenPfdOpt:
      openMenu(SoftkeyMenu::PfdOpt);
      break;
    case SoftkeyAction::OpenXpdr:
      openMenu(SoftkeyMenu::Xpdr);
      break;
    case SoftkeyAction::ToggleAlerts:
      alertsOpen_ = !alertsOpen_;
      break;
    case SoftkeyAction::XpdrStandby:
      xpdrMode_ = XpdrMode::Standby;
      break;
    case SoftkeyAction::XpdrOn:
      xpdrMode_ = XpdrMode::On;
      break;
    case SoftkeyAction::XpdrAlt:
      xpdrMode_ = XpdrMode::Alt;
      break;
    case SoftkeyAction::XpdrGround:
      xpdrMode_ = XpdrMode::Ground;
      break;
    default: {
      const int toggle = toggleIndex(action);
      if (toggle >= 0) toggles_[toggle] = !toggles_[toggle];
      break;
    }
  }
  return true;
}

void SoftkeyController::rebuildAlerts(const FlightData& data) {
  alerts_.clear();
  annunciations_.clear();
  // Reversionary sensor failures surface here as warnings, alongside the
  // per-instrument red X. AHRS feeds attitude + heading; the ADC feeds the
  // air-data instruments.
  if (!data.attitudeValid) {
    alerts_.push_back({"AHRS1 SERVICE - Attitude data unavailable",
                       AlertLevel::Warning});
  }
  if (!data.headingValid) {
    alerts_.push_back(
        {"HDG - Heading reference unavailable", AlertLevel::Warning});
  }
  if (!data.airspeedValid || !data.altitudeValid || !data.verticalSpeedValid) {
    alerts_.push_back(
        {"ADC1 SERVICE - Air data unavailable", AlertLevel::Warning});
  }
  if (!data.navSignalValid) {
    alerts_.push_back(
        {"NAV - No usable navigation signal", AlertLevel::Caution});
  }

  // CAS messages from the live X-Plane annunciators populate the always-on PFD
  // annunciation window only. The Alerts window (above) is for system message
  // advisories, which on the real G1000 are kept separate from CAS messages.
  for (const CasMessageDef& m : kCasMessages) {
    if (data.*(m.condition)) annunciations_.push_back({m.text, m.level});
  }

  // Real crew alerting stacks the highest-severity messages at the top
  // (warnings, then cautions, then advisories). stable_sort keeps the relative
  // order within each level the one defined above.
  const auto byLevel = [](const AlertMessage& a, const AlertMessage& b) {
    return static_cast<int>(a.level) < static_cast<int>(b.level);
  };
  std::stable_sort(alerts_.begin(), alerts_.end(), byLevel);
  std::stable_sort(annunciations_.begin(), annunciations_.end(), byLevel);
}

}  // namespace avionics
