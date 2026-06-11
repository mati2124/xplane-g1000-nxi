#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// IDNT annunciation duration after the Ident key (Pilot's Guide, Ident
// Function) and the Selected Altitude alert flash duration (Fig. 2-32).
constexpr double kIdentSeconds = 18.0;
constexpr double kAltAlertFlashSeconds = 5.0;

// Minimums altitude entry: small-knob step and the settable range (Pilot's
// Guide: "from zero to 16,000 feet").
constexpr float kMinsStepFt = 100.0f;
constexpr float kMinsMaxFt = 16000.0f;

// Nearest Airports window: list capacity and search radius (Pilot's Guide:
// "a list of up to 25 of the nearest airports", "None Within 200nm").
constexpr int kNearestMaxAirports = 25;
constexpr double kNearestMaxRangeNm = 200.0;

// Altitude Alerting thresholds (Pilot's Guide, Altitude Alerting).
constexpr float kAltAlertArmFt = 1000.0f;
constexpr float kAltAlertCaptureFt = 200.0f;
// A Selected Altitude change beyond this re-arms the alerter.
constexpr float kAltAlertRearmFt = 25.0f;

// What a softkey cell does when pressed. Kept internal to the controller: the
// renderer only ever sees labels and the keyActive()/pressLevel() highlight
// state, never the action.
enum class SoftkeyAction {
  None,       // blank cell, ignores presses
  Momentary,  // press-flash only; not yet wired to a function
  Back,       // pop to the parent menu
  // Submenu openers (G1000 NXi PFD softkey map, Pilot's Guide Table 1-3).
  OpenMapHsi,
  OpenLayout,
  OpenPfdOpt,
  OpenSvt,
  OpenWind,
  OpenAltUnits,
  OpenXpdr,
  OpenXpdrCode,
  ToggleAlerts,
  // PFD pop-up windows (lower right, one at a time).
  ToggleTmrRef,
  ToggleNearest,
  // Transponder ident and code entry.
  Ident,
  Digit,  // squawk digit key; the cell label carries the digit value
  Bksp,
  // Map/HSI > Layout radio group.
  LayoutMapOff,
  LayoutInset,
  LayoutHsi,
  // Wind submenu radio group.
  WindOff,
  WindOpt1,
  WindOpt2,
  WindOpt3,
  // ALT Units submenu: Meters toggle plus an IN/HPA baro-units radio.
  BaroIn,
  BaroHpa,
  StdBaro,
  // Display toggles.
  ToggleTraffic,
  ToggleObs,
  ToggleDme,
  ToggleBearing1,
  ToggleBearing2,
  ToggleAltMeters,
  TogglePathways,
  ToggleSynTerr,
  ToggleHdgLbl,
  ToggleAptSign,
  ToggleTopo,
  ToggleRelTer,
  // Map declutter level cycle (Detail All -> 3 -> 2 -> 1).
  DetailCycle,
  // Root CDI key: cycle the displayed nav source GPS -> VOR1 -> VOR2.
  CdiSrcCycle,
  // Transponder mode radio group.
  XpdrStandby,
  XpdrOn,
  XpdrAlt,
  XpdrVfr,
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
    {"CDI", SoftkeyAction::CdiSrcCycle},
    {"DME", SoftkeyAction::ToggleDme},
    {"XPDR", SoftkeyAction::OpenXpdr},
    {"Ident", SoftkeyAction::Ident},
    {"Tmr/Ref", SoftkeyAction::ToggleTmrRef},
    {"Nearest", SoftkeyAction::ToggleNearest},
    {"Alerts", SoftkeyAction::ToggleAlerts},
}};

// Map/HSI submenu (Pilot's Guide Table 1-3): Layout opens the map-placement
// radio; Detail cycles the declutter level; the remaining keys overlay
// traffic/topo/relative-terrain on the map.
constexpr Menu kMapHsiMenu = {{
    {"", SoftkeyAction::None},
    {"Layout", SoftkeyAction::OpenLayout},
    {"Detail", SoftkeyAction::DetailCycle},
    {"Traffic", SoftkeyAction::ToggleTraffic},
    {"Topo", SoftkeyAction::ToggleTopo},
    {"Rel Ter", SoftkeyAction::ToggleRelTer},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// Map/HSI > Layout: a radio group selecting the PFD map placement.
constexpr Menu kLayoutMenu = {{
    {"", SoftkeyAction::None},
    {"Map Off", SoftkeyAction::LayoutMapOff},
    {"Inset Map", SoftkeyAction::LayoutInset},
    {"HSI Map", SoftkeyAction::LayoutHsi},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// PFD Opt submenu (Pilot's Guide Table 1-3). There is no "HSI Frmt" key on the
// NXi; SVT, Wind, and ALT Units open their own submenus.
constexpr Menu kPfdOptMenu = {{
    {"", SoftkeyAction::None},
    {"SVT", SoftkeyAction::OpenSvt},
    {"Wind", SoftkeyAction::OpenWind},
    {"DME", SoftkeyAction::ToggleDme},
    {"Bearing 1", SoftkeyAction::ToggleBearing1},
    {"Bearing 2", SoftkeyAction::ToggleBearing2},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"ALT Units", SoftkeyAction::OpenAltUnits},
    {"STD Baro", SoftkeyAction::StdBaro},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// PFD Opt > SVT overlay toggles.
constexpr Menu kSvtMenu = {{
    {"", SoftkeyAction::None},
    {"Pathways", SoftkeyAction::TogglePathways},
    {"Syn Terr", SoftkeyAction::ToggleSynTerr},
    {"HDG LBL", SoftkeyAction::ToggleHdgLbl},
    {"APT Sign", SoftkeyAction::ToggleAptSign},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// PFD Opt > Wind: Off / Option 1 / Option 2 / Option 3 radio group.
constexpr Menu kWindMenu = {{
    {"", SoftkeyAction::None},
    {"Off", SoftkeyAction::WindOff},
    {"Option 1", SoftkeyAction::WindOpt1},
    {"Option 2", SoftkeyAction::WindOpt2},
    {"Option 3", SoftkeyAction::WindOpt3},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// PFD Opt > ALT Units: Meters toggle and an IN/HPA baro-units radio.
constexpr Menu kAltUnitsMenu = {{
    {"", SoftkeyAction::None},
    {"Meters", SoftkeyAction::ToggleAltMeters},
    {"IN", SoftkeyAction::BaroIn},
    {"HPA", SoftkeyAction::BaroHpa},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// XPDR submenu (Pilot's Guide Table 1-3): Standby / On / ALT mode keys, VFR,
// the Code-entry submenu, and Ident. There is no "GND" softkey on the NXi.
constexpr Menu kXpdrMenu = {{
    {"", SoftkeyAction::None},
    {"Standby", SoftkeyAction::XpdrStandby},
    {"On", SoftkeyAction::XpdrOn},
    {"ALT", SoftkeyAction::XpdrAlt},
    {"VFR", SoftkeyAction::XpdrVfr},
    {"Code", SoftkeyAction::OpenXpdrCode},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Ident", SoftkeyAction::Ident},
    {"", SoftkeyAction::None},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

// XPDR > Code: digit-entry keys 0-7 plus BKSP and Ident.
constexpr Menu kXpdrCodeMenu = {{
    {"0", SoftkeyAction::Digit},
    {"1", SoftkeyAction::Digit},
    {"2", SoftkeyAction::Digit},
    {"3", SoftkeyAction::Digit},
    {"4", SoftkeyAction::Digit},
    {"5", SoftkeyAction::Digit},
    {"6", SoftkeyAction::Digit},
    {"7", SoftkeyAction::Digit},
    {"Ident", SoftkeyAction::Ident},
    {"BKSP", SoftkeyAction::Bksp},
    {"", SoftkeyAction::None},
    {"Back", SoftkeyAction::Back},
}};

const Menu& menuDefs(SoftkeyMenu menu) {
  switch (menu) {
    case SoftkeyMenu::MapHsi:
      return kMapHsiMenu;
    case SoftkeyMenu::Layout:
      return kLayoutMenu;
    case SoftkeyMenu::PfdOpt:
      return kPfdOptMenu;
    case SoftkeyMenu::Svt:
      return kSvtMenu;
    case SoftkeyMenu::Wind:
      return kWindMenu;
    case SoftkeyMenu::AltUnits:
      return kAltUnitsMenu;
    case SoftkeyMenu::Xpdr:
      return kXpdrMenu;
    case SoftkeyMenu::XpdrCode:
      return kXpdrCodeMenu;
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
    case SoftkeyAction::ToggleObs:
      return static_cast<int>(DisplayToggle::Obs);
    case SoftkeyAction::ToggleDme:
      return static_cast<int>(DisplayToggle::Dme);
    case SoftkeyAction::ToggleBearing1:
      return static_cast<int>(DisplayToggle::Bearing1);
    case SoftkeyAction::ToggleBearing2:
      return static_cast<int>(DisplayToggle::Bearing2);
    case SoftkeyAction::ToggleAltMeters:
      return static_cast<int>(DisplayToggle::AltMeters);
    case SoftkeyAction::TogglePathways:
      return static_cast<int>(DisplayToggle::Pathways);
    case SoftkeyAction::ToggleSynTerr:
      return static_cast<int>(DisplayToggle::SynTerr);
    case SoftkeyAction::ToggleHdgLbl:
      return static_cast<int>(DisplayToggle::HdgLbl);
    case SoftkeyAction::ToggleAptSign:
      return static_cast<int>(DisplayToggle::AptSign);
    case SoftkeyAction::ToggleTopo:
      return static_cast<int>(DisplayToggle::MapTopo);
    case SoftkeyAction::ToggleRelTer:
      return static_cast<int>(DisplayToggle::MapRelTer);
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
  // The inset map ships with the topographic background on, matching the MFD
  // navigation map default.
  toggles_[static_cast<int>(DisplayToggle::MapTopo)] = true;
  rebuildLabels();
}

void SoftkeyController::rebuildLabels() {
  const Menu& menu = menuDefs(currentMenu());
  for (int i = 0; i < kSoftkeyCount; ++i) {
    // The Detail key's label carries the current declutter level.
    labels_[i] = menu[i].action == SoftkeyAction::DetailCycle
                     ? mapDetailLabel(mapDetail_)
                     : menu[i].label;
  }
}

void SoftkeyController::update(double dtSeconds, const FlightData& data,
                               const MapData& map) {
  const float dt = static_cast<float>(dtSeconds);

  // Key-press flashes decay linearly back to rest.
  const float pressStep = dt / kPressFlashSeconds;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    press_[i] = std::max(0.0f, press_[i] - pressStep);
  }
  for (int i = 0; i < kBezelKeyCount; ++i) {
    bezelPress_[i] = std::max(0.0f, bezelPress_[i] - pressStep);
  }
  insetDisplayRangeNm_ = animateMapRange(
      insetDisplayRangeNm_, mapRangeNmAt(insetRangeIndex_), dtSeconds);

  // Each pop-up window eases toward its open/closed target at a constant rate,
  // so a replaced window fades out while the new one fades in.
  for (int w = 1; w < kPfdWindowCount; ++w) {
    const float target = (static_cast<int>(window_) == w) ? 1.0f : 0.0f;
    windowAnim_[w] = approach(windowAnim_[w], target, dt / kWindowAnimSeconds);
  }

  // ~1 Hz blink phase for flashing annunciations (Alerts softkey, Baro
  // Transition Alert): on for the first half of each second.
  blinkSeconds_ += dtSeconds;
  blinkOn_ = std::fmod(blinkSeconds_, 1.0) < 0.5;

  // The NAV/COM tuning cursor flashes for a few seconds after a tuning action,
  // then settles solid (Working Title NXi armed-border behavior).
  if (radioArmedSeconds_ > 0.0) {
    radioArmedSeconds_ = std::max(0.0, radioArmedSeconds_ - dtSeconds);
  }

  // Generic timer and the IDNT annunciation countdown.
  if (timerRunning_) timerSeconds_ += dtSeconds;
  if (identSecondsLeft_ > 0.0) {
    identSecondsLeft_ = std::max(0.0, identSecondsLeft_ - dtSeconds);
  }

  if (window_ == PfdWindow::Nearest ||
      windowAnim_[static_cast<int>(PfdWindow::Nearest)] > 0.0f) {
    rebuildNearest(map);
  }

  // Remember the feed's CDI source so the first CDI-key press cycles on from it.
  lastCdiFeed_ = data.cdiSource;

  updateAltAlert(dtSeconds, data);
  rebuildAlerts(data);
  updateAlertSoftkey();
}

const char* SoftkeyController::timerCommandLabel() const {
  if (timerRunning_) return "Stop?";
  return timerSeconds() > 0 ? "Reset?" : "Start?";
}

void SoftkeyController::toggleWindow(PfdWindow w) {
  window_ = (window_ == w) ? PfdWindow::None : w;
  // Opening a window puts the FMS cursor at its first field/entry.
  if (window_ == PfdWindow::References) refCursor_ = RefField::TimerCmd;
  if (window_ == PfdWindow::Nearest) nearestCursor_ = 0;
}

void SoftkeyController::rebuildNearest(const MapData& map) {
  nearest_.clear();
  if (!map.positionValid) {
    nearestCursor_ = 0;
    return;
  }
  for (const MapFeature& f : map.features) {
    if (f.type != MapFeatureType::Airport) continue;
    const double distNm =
        navDistanceNm(map.ownshipLat, map.ownshipLon, f.lat, f.lon);
    if (distNm > kNearestMaxRangeNm) continue;
    NearestAirport a;
    a.id = f.id;
    a.distanceNm = static_cast<float>(distNm);
    a.bearingDeg = static_cast<float>(
        navBearingDeg(map.ownshipLat, map.ownshipLon, f.lat, f.lon));
    a.frequencyMhz = f.frequency;
    a.longestRunwayFt = f.longestRunwayFt;
    nearest_.push_back(std::move(a));
  }
  std::sort(nearest_.begin(), nearest_.end(),
            [](const NearestAirport& a, const NearestAirport& b) {
              return a.distanceNm < b.distanceNm;
            });
  if (nearest_.size() > static_cast<size_t>(kNearestMaxAirports)) {
    nearest_.resize(kNearestMaxAirports);
  }
  nearestCursor_ = std::max(
      0, std::min(nearestCursor_, static_cast<int>(nearest_.size()) - 1));
}

void SoftkeyController::updateAltAlert(double dtSeconds,
                                       const FlightData& data) {
  if (altAlertFlashLeft_ > 0.0) {
    altAlertFlashLeft_ = std::max(0.0, altAlertFlashLeft_ - dtSeconds);
  }

  // No usable selection or altitude: keep the alerter quietly armed.
  if (std::fabs(data.selectedAltitudeFt) <= 1.0f || !data.altitudeValid) {
    altAlertPhase_ = AltAlertPhase::Armed;
    altAlertSelectedFt_ = data.selectedAltitudeFt;
    altAlertFlashLeft_ = 0.0;
    return;
  }
  // Whenever the Selected Altitude is changed, the Altitude Alerter is reset.
  if (std::fabs(data.selectedAltitudeFt - altAlertSelectedFt_) >
      kAltAlertRearmFt) {
    altAlertPhase_ = AltAlertPhase::Armed;
    altAlertSelectedFt_ = data.selectedAltitudeFt;
    altAlertFlashLeft_ = 0.0;
  }

  const float diff = std::fabs(data.altitudeFt - data.selectedAltitudeFt);
  switch (altAlertPhase_) {
    case AltAlertPhase::Armed:
      if (diff <= kAltAlertArmFt) {
        altAlertPhase_ = (diff <= kAltAlertCaptureFt) ? AltAlertPhase::Within200
                                                      : AltAlertPhase::Within1000;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Within1000:
      if (diff <= kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Within200;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Within200:
      if (altAlertFlashLeft_ <= 0.0) altAlertPhase_ = AltAlertPhase::Captured;
      break;
    case AltAlertPhase::Captured:
      if (diff > kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Deviation;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Deviation:
      if (diff <= kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Captured;
        altAlertFlashLeft_ = 0.0;
      } else if (altAlertFlashLeft_ <= 0.0 && diff > kAltAlertArmFt) {
        altAlertPhase_ = AltAlertPhase::Armed;  // left the capture band
      }
      break;
  }
}

SelectedAltStyle SoftkeyController::selectedAltStyle() const {
  SelectedAltStyle s;
  if (altAlertFlashLeft_ <= 0.0) return s;  // steady between transitions
  // During a five-second flash the alert look alternates with the normal look
  // on the blink clock (Pilot's Guide Fig. 2-32).
  switch (altAlertPhase_) {
    case AltAlertPhase::Within1000:
      s.cyanBackground = blinkOn_;  // black text on a cyan plate
      break;
    case AltAlertPhase::Within200:
      s.hideText = !blinkOn_;  // cyan-on-black readout blinks
      break;
    case AltAlertPhase::Deviation:
      s.amberText = true;
      s.hideText = !blinkOn_;
      break;
    default:
      break;
  }
  return s;
}

bool SoftkeyController::keyActive(int i) const {
  if (i < 0 || i >= kSoftkeyCount) return false;
  const SoftkeyAction action = menuDefs(currentMenu())[i].action;

  if (action == SoftkeyAction::ToggleAlerts) return window_ == PfdWindow::Alerts;
  if (action == SoftkeyAction::ToggleTmrRef) {
    return window_ == PfdWindow::References;
  }
  if (action == SoftkeyAction::ToggleNearest) {
    return window_ == PfdWindow::Nearest;
  }

  const int toggle = toggleIndex(action);
  if (toggle >= 0) return toggles_[toggle];

  switch (action) {
    case SoftkeyAction::XpdrStandby:
      return xpdrMode_ == XpdrMode::Standby;
    case SoftkeyAction::XpdrOn:
      return xpdrMode_ == XpdrMode::On;
    case SoftkeyAction::XpdrAlt:
      return xpdrMode_ == XpdrMode::Alt;
    case SoftkeyAction::LayoutMapOff:
      return mapLayout_ == MapLayout::Off;
    case SoftkeyAction::LayoutInset:
      return mapLayout_ == MapLayout::Inset;
    case SoftkeyAction::LayoutHsi:
      return mapLayout_ == MapLayout::Hsi;
    case SoftkeyAction::WindOff:
      return windOption_ == WindOption::Off;
    case SoftkeyAction::WindOpt1:
      return windOption_ == WindOption::Option1;
    case SoftkeyAction::WindOpt2:
      return windOption_ == WindOption::Option2;
    case SoftkeyAction::WindOpt3:
      return windOption_ == WindOption::Option3;
    case SoftkeyAction::BaroIn:
      return !toggles_[static_cast<int>(DisplayToggle::BaroHpa)];
    case SoftkeyAction::BaroHpa:
      return toggles_[static_cast<int>(DisplayToggle::BaroHpa)];
    default:
      return false;
  }
}

bool SoftkeyController::softkeyFlashing(int i) const {
  return i == kAlertsKey && currentMenu() == SoftkeyMenu::Root &&
         alertSoftkeyActive_;
}

void SoftkeyController::pressBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i < 0 || i >= kBezelKeyCount) return;
  bezelPress_[i] = 1.0f;  // trigger the press-flash animation
  if (key == BezelKey::RangeUp) {
    insetRangeIndex_ = std::min(kMapRangeLadderCount - 1, insetRangeIndex_ + 1);
    return;
  }
  if (key == BezelKey::RangeDown) {
    insetRangeIndex_ = std::max(0, insetRangeIndex_ - 1);
    return;
  }

  // CLR removes the open pop-up window (Pilot's Guide: "To remove the window,
  // press the CLR Key or the Tmr/Ref Softkey").
  if (key == BezelKey::Clr && window_ != PfdWindow::None) {
    window_ = PfdWindow::None;
    return;
  }

  // Inside the References window the FMS knob works as on the real unit: the
  // large knob moves the field cursor, the small knob changes the highlighted
  // value (the MINS altitude), and ENT activates the highlighted field.
  if (window_ == PfdWindow::References) {
    if (key == BezelKey::FmsOuterCw) moveReferencesCursor(+1);
    if (key == BezelKey::FmsOuterCcw) moveReferencesCursor(-1);
    if (key == BezelKey::FmsInnerCw || key == BezelKey::FmsInnerCcw) {
      adjustReferencesValue(key == BezelKey::FmsInnerCw ? +1 : -1);
    }
    if (key == BezelKey::Ent) activateReferencesField();
    return;
  }

  // Inside the Nearest Airports window the FMS knob scrolls the list. (On
  // the real unit ENT loads the highlighted COM frequency into the standby
  // field; the radios are owned by the sim feed, so ENT is press-flash only.)
  if (window_ == PfdWindow::Nearest && !nearest_.empty()) {
    const int last = static_cast<int>(nearest_.size()) - 1;
    if (key == BezelKey::FmsOuterCw || key == BezelKey::FmsInnerCw) {
      nearestCursor_ = std::min(last, nearestCursor_ + 1);
    }
    if (key == BezelKey::FmsOuterCcw || key == BezelKey::FmsInnerCcw) {
      nearestCursor_ = std::max(0, nearestCursor_ - 1);
    }
    return;
  }
}

void SoftkeyController::moveReferencesCursor(int step) {
  // The cursor walks every field except MinsValue, which is only reachable
  // via ENT from the MINS mode field once BARO is selected (mirroring the
  // real unit, where ENT highlights the next field after a selection).
  // Turning the large knob while on the altitude steps off it.
  const int lastField = static_cast<int>(RefField::MinsMode);
  int cur = (refCursor_ == RefField::MinsValue ||
             refCursor_ == RefField::MinsTemp)
                ? static_cast<int>(RefField::MinsMode)
                : static_cast<int>(refCursor_);
  cur += step;
  if (cur < 0) cur = lastField;
  if (cur > lastField) cur = 0;
  refCursor_ = static_cast<RefField>(cur);
}

void SoftkeyController::adjustReferencesValue(int step) {
  // Small FMS knob: the MINS altitude and the V-speed reference values are the
  // numeric fields (the On/Off selections are activated with ENT).
  if (refCursor_ == RefField::MinsValue) {
    minsAltFt_ = std::max(
        0.0f, std::min(kMinsMaxFt, minsAltFt_ + step * kMinsStepFt));
    return;
  }
  if (refCursor_ == RefField::MinsTemp) {
    minsTempC_ = std::max(
        kMinsTempMinC,
        std::min(kMinsTempMaxC, minsTempC_ + step * kMinsTempStepC));
    return;
  }
  if (refCursor_ == RefField::Glide || refCursor_ == RefField::Vr ||
      refCursor_ == RefField::Vx || refCursor_ == RefField::Vy) {
    const int v = static_cast<int>(refCursor_) - static_cast<int>(RefField::Glide);
    vspeedKt_[v] = std::max(
        kVspeedMinKt,
        std::min(kVspeedMaxKt, vspeedKt_[v] + step * kVspeedStepKt));
  }
}

void SoftkeyController::activateReferencesField() {
  switch (refCursor_) {
    case RefField::TimerCmd:
      // Start? -> Stop? -> Reset? cycle (Pilot's Guide, Generic Timer).
      if (timerRunning_) {
        timerRunning_ = false;
      } else if (timerSeconds() > 0) {
        timerSeconds_ = 0.0;
      } else {
        timerRunning_ = true;
      }
      break;
    case RefField::Glide:
    case RefField::Vr:
    case RefField::Vx:
    case RefField::Vy: {
      const int v = static_cast<int>(refCursor_) - static_cast<int>(RefField::Glide);
      vspeedOn_[v] = !vspeedOn_[v];
      break;
    }
    case RefField::MinsMode:
      // Cycle the minimum source Off -> BARO -> TEMP -> Off. Selecting a source
      // highlights the next field (the altitude), per the real unit.
      switch (minsMode_) {
        case MinimumsMode::Off:
          minsMode_ = MinimumsMode::Baro;
          refCursor_ = RefField::MinsValue;
          break;
        case MinimumsMode::Baro:
          minsMode_ = MinimumsMode::Temp;
          refCursor_ = RefField::MinsValue;
          break;
        case MinimumsMode::Temp:
          minsMode_ = MinimumsMode::Off;
          break;
      }
      break;
    case RefField::MinsValue:
      // In TEMP COMP, ENT advances to the destination-temperature field.
      refCursor_ = (minsMode_ == MinimumsMode::Temp) ? RefField::MinsTemp
                                                     : RefField::TimerCmd;
      break;
    case RefField::MinsTemp:
      refCursor_ = RefField::TimerCmd;  // entry accepted
      break;
    case RefField::Count:
      break;
  }
}

void SoftkeyController::startIdent() {
  // In Standby the Ident key is inoperative (Pilot's Guide, Ident Function).
  if (xpdrMode_ == XpdrMode::Standby) return;
  identSecondsLeft_ = kIdentSeconds;
  // From the Mode or Code selection softkeys, Ident reverts to the top level.
  if (currentMenu() == SoftkeyMenu::Xpdr ||
      currentMenu() == SoftkeyMenu::XpdrCode) {
    xpdrPending_.clear();
    menuStack_.resize(1);
    rebuildLabels();
  }
}

bool SoftkeyController::pressKey(int key) {
  if (key < 0 || key >= kSoftkeyCount) return false;

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
      if (currentMenu() == SoftkeyMenu::XpdrCode) {
        xpdrPending_.clear();  // abandon the in-progress code entry
      }
      if (menuStack_.size() > 1) {
        menuStack_.pop_back();
        rebuildLabels();
      }
      break;
    case SoftkeyAction::OpenMapHsi:
      openMenu(SoftkeyMenu::MapHsi);
      break;
    case SoftkeyAction::OpenLayout:
      openMenu(SoftkeyMenu::Layout);
      break;
    case SoftkeyAction::OpenPfdOpt:
      openMenu(SoftkeyMenu::PfdOpt);
      break;
    case SoftkeyAction::OpenSvt:
      openMenu(SoftkeyMenu::Svt);
      break;
    case SoftkeyAction::OpenWind:
      openMenu(SoftkeyMenu::Wind);
      break;
    case SoftkeyAction::OpenAltUnits:
      openMenu(SoftkeyMenu::AltUnits);
      break;
    case SoftkeyAction::OpenXpdr:
      openMenu(SoftkeyMenu::Xpdr);
      break;
    case SoftkeyAction::OpenXpdrCode:
      openMenu(SoftkeyMenu::XpdrCode);
      break;
    case SoftkeyAction::ToggleAlerts:
      toggleWindow(PfdWindow::Alerts);
      break;
    case SoftkeyAction::ToggleTmrRef:
      toggleWindow(PfdWindow::References);
      break;
    case SoftkeyAction::ToggleNearest:
      toggleWindow(PfdWindow::Nearest);
      break;
    case SoftkeyAction::Ident:
      startIdent();
      break;
    case SoftkeyAction::Digit:
      // The cell label carries the digit. The new code shows in the
      // Transponder Data Box as it is typed; the fourth digit completes entry
      // and reverts to the top-level softkeys. Committing the completed code
      // to the radio is owned by the sim feed (no command channel yet).
      if (xpdrPending_.size() < 4) {
        xpdrPending_ += menuDefs(currentMenu())[key].label[0];
      }
      if (xpdrPending_.size() == 4) {
        xpdrCodeCommit_ = static_cast<int>(std::strtol(xpdrPending_.c_str(),
                                                       nullptr, 10));
        xpdrCodeCommitPending_ = true;
        xpdrPending_.clear();
        menuStack_.resize(1);
        rebuildLabels();
      }
      break;
    case SoftkeyAction::Bksp:
      if (!xpdrPending_.empty()) xpdrPending_.pop_back();
      break;
    case SoftkeyAction::LayoutMapOff:
      mapLayout_ = MapLayout::Off;
      break;
    case SoftkeyAction::LayoutInset:
      mapLayout_ = MapLayout::Inset;
      break;
    case SoftkeyAction::LayoutHsi:
      mapLayout_ = MapLayout::Hsi;
      break;
    case SoftkeyAction::WindOff:
      windOption_ = WindOption::Off;
      break;
    case SoftkeyAction::WindOpt1:
      windOption_ = WindOption::Option1;
      break;
    case SoftkeyAction::WindOpt2:
      windOption_ = WindOption::Option2;
      break;
    case SoftkeyAction::WindOpt3:
      windOption_ = WindOption::Option3;
      break;
    case SoftkeyAction::BaroIn:
      toggles_[static_cast<int>(DisplayToggle::BaroHpa)] = false;
      break;
    case SoftkeyAction::BaroHpa:
      toggles_[static_cast<int>(DisplayToggle::BaroHpa)] = true;
      break;
    case SoftkeyAction::StdBaro:
      // The NXi STD Baro key sets standard pressure; the actual setting is owned
      // by the sim feed, so here it is press-flash only.
      break;
    case SoftkeyAction::DetailCycle:
      mapDetail_ = nextMapDetail(mapDetail_);
      rebuildLabels();
      break;
    case SoftkeyAction::CdiSrcCycle: {
      // GPS -> VOR1 -> VOR2 -> GPS. The first press starts from the source the
      // feed is currently reporting; subsequent presses cycle the held source.
      const CdiSource from = cdiOverride_ ? cdiSource_ : lastCdiFeed_;
      switch (from) {
        case CdiSource::Gps:  cdiSource_ = CdiSource::Nav1; break;
        case CdiSource::Nav1: cdiSource_ = CdiSource::Nav2; break;
        case CdiSource::Nav2: cdiSource_ = CdiSource::Gps;  break;
      }
      cdiOverride_ = true;
      break;
    }
    case SoftkeyAction::XpdrStandby:
      xpdrMode_ = XpdrMode::Standby;
      queueXpdrModeCommit();
      break;
    case SoftkeyAction::XpdrOn:
      xpdrMode_ = XpdrMode::On;
      queueXpdrModeCommit();
      break;
    case SoftkeyAction::XpdrAlt:
      xpdrMode_ = XpdrMode::Alt;
      queueXpdrModeCommit();
      break;
    case SoftkeyAction::XpdrVfr:
      xpdrCodeCommit_ = 1200;
      xpdrCodeCommitPending_ = true;
      break;
    default: {
      const int toggle = toggleIndex(action);
      if (toggle >= 0) toggles_[toggle] = !toggles_[toggle];
      break;
    }
  }
  return true;
}

void SoftkeyController::updateAlertSoftkey() {
  // The dynamic Alerts softkey only exists on the root menu (submenus carry a
  // Back key in that cell). While alerts are unacknowledged, the key relabels
  // to the highest active severity and flashes; when none are present it reads
  // "Alerts" and is steady.
  if (currentMenu() != SoftkeyMenu::Root) {
    alertSoftkeyActive_ = false;
    return;
  }

  bool warning = false;
  bool caution = false;
  for (const AlertMessage& m : annunciations_) {
    if (m.level == AlertLevel::Warning) warning = true;
    else if (m.level == AlertLevel::Caution) caution = true;
  }
  // System message advisories live in the Alerts window (not the annunciation
  // window) and drive a flashing "Messages" label.
  const bool messages = !alerts_.empty();

  const char* label = "Alerts";
  alertSoftkeyActive_ = false;
  if (warning) {
    label = "Warning";
    alertSoftkeyLevel_ = AlertLevel::Warning;
    alertSoftkeyActive_ = true;
  } else if (caution) {
    label = "Caution";
    alertSoftkeyLevel_ = AlertLevel::Caution;
    alertSoftkeyActive_ = true;
  } else if (messages) {
    label = "Messages";
    alertSoftkeyLevel_ = AlertLevel::Advisory;
    alertSoftkeyActive_ = true;
  }
  // Once the window is open the pilot is viewing the alerts, so stop flashing.
  if (window_ == PfdWindow::Alerts) alertSoftkeyActive_ = false;
  labels_[kAlertsKey] = label;
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

int SoftkeyController::xpdrModeToSim(XpdrMode mode) {
  switch (mode) {
    case XpdrMode::Standby:
      return 1;
    case XpdrMode::On:
      return 2;
    case XpdrMode::Alt:
      return 3;
    case XpdrMode::Ground:
      return 1;
  }
  return 3;
}

void SoftkeyController::queueXpdrModeCommit() {
  xpdrModeCommit_ = xpdrModeToSim(xpdrMode_);
  xpdrModeCommitPending_ = true;
}

bool SoftkeyController::consumeXpdrCodeCommit(int& code) {
  if (!xpdrCodeCommitPending_) return false;
  xpdrCodeCommitPending_ = false;
  code = xpdrCodeCommit_;
  return true;
}

bool SoftkeyController::consumeXpdrModeCommit(int& mode) {
  if (!xpdrModeCommitPending_) return false;
  xpdrModeCommitPending_ = false;
  mode = xpdrModeCommit_;
  return true;
}

bool SoftkeyController::consumeRadioTune(RadioUnit& unit, float& standbyMhz) {
  if (!radioTunePending_) return false;
  radioTunePending_ = false;
  unit = radioTuneUnit_;
  standbyMhz = radioTuneMhz_;
  return true;
}

bool SoftkeyController::consumeRadioTransfer(RadioUnit& unit) {
  if (!radioTransferPending_) return false;
  radioTransferPending_ = false;
  unit = radioTransferUnit_;
  return true;
}

float SoftkeyController::standbyMhzFor(RadioUnit unit,
                                       const FlightData& d) const {
  switch (unit) {
    case RadioUnit::Nav1:
      return d.nav1StandbyMhz;
    case RadioUnit::Nav2:
      return d.nav2StandbyMhz;
    case RadioUnit::Com1:
      return d.com1StandbyMhz;
    case RadioUnit::Com2:
      return d.com2StandbyMhz;
  }
  return d.nav1StandbyMhz;
}

void SoftkeyController::setStandbyMhzFor(RadioUnit unit, float mhz) {
  radioTuneUnit_ = unit;
  radioTuneMhz_ = mhz;
  radioTunePending_ = true;
}

void SoftkeyController::queueRadioTune(RadioUnit unit, float standbyMhz) {
  setStandbyMhzFor(unit, standbyMhz);
  armRadioBand(radioBandOf(unit));
}

void SoftkeyController::queueRadioTransfer(RadioUnit unit) {
  radioTransferUnit_ = unit;
  radioTransferPending_ = true;
  armRadioBand(radioBandOf(unit));
}

void SoftkeyController::armRadioBand(RadioBand band) {
  radioArmedBand_ = band;
  radioArmedSeconds_ = kRadioArmedSeconds;
}

void SoftkeyController::cycleRadioSelect() {
  switch (radioSelected_) {
    case RadioUnit::Nav1:
      radioSelected_ = RadioUnit::Nav2;
      break;
    case RadioUnit::Nav2:
      radioSelected_ = RadioUnit::Com1;
      break;
    case RadioUnit::Com1:
      radioSelected_ = RadioUnit::Com2;
      break;
    case RadioUnit::Com2:
      radioSelected_ = RadioUnit::Nav1;
      break;
  }
  // Mirror the unified focus into the per-side cursor so the bar's COM and NAV
  // boxes track the FMS knob, and flash the side it landed on.
  if (radioBandOf(radioSelected_) == RadioBand::Com) {
    comSelected_ = radioSelected_;
  } else {
    navSelected_ = radioSelected_;
  }
  armRadioBand(radioBandOf(radioSelected_));
}

void SoftkeyController::selectCom() {
  comSelected_ =
      comSelected_ == RadioUnit::Com1 ? RadioUnit::Com2 : RadioUnit::Com1;
  radioSelected_ = comSelected_;
  armRadioBand(RadioBand::Com);
}

void SoftkeyController::selectNav() {
  navSelected_ =
      navSelected_ == RadioUnit::Nav1 ? RadioUnit::Nav2 : RadioUnit::Nav1;
  radioSelected_ = navSelected_;
  armRadioBand(RadioBand::Nav);
}

void SoftkeyController::tuneCom(int direction, bool coarse, const FlightData& d) {
  const float cur = standbyMhzFor(comSelected_, d);
  const float next = coarse ? stepComStandbyMhzCoarse(cur, direction)
                            : stepComStandbyMhz(cur, direction);
  queueRadioTune(comSelected_, next);
}

void SoftkeyController::tuneNav(int direction, bool coarse, const FlightData& d) {
  const float cur = standbyMhzFor(navSelected_, d);
  const float next = coarse ? stepNavStandbyMhzCoarse(cur, direction)
                            : stepNavStandbyMhz(cur, direction);
  queueRadioTune(navSelected_, next);
}

void SoftkeyController::transferCom() { queueRadioTransfer(comSelected_); }

void SoftkeyController::transferNav() { queueRadioTransfer(navSelected_); }

bool SoftkeyController::radioBezelKey(BezelKey key, const FlightData& d) {
  if (!canUseRadioBezel()) return false;

  switch (key) {
    case BezelKey::FmsPush:
      cycleRadioSelect();
      return true;
    case BezelKey::Ent:
      queueRadioTransfer(radioSelected_);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const int dir = key == BezelKey::FmsInnerCw ? +1 : -1;
      const float cur = standbyMhzFor(radioSelected_, d);
      const bool isCom = radioSelected_ == RadioUnit::Com1 ||
                         radioSelected_ == RadioUnit::Com2;
      const float next =
          isCom ? stepComStandbyMhz(cur, dir) : stepNavStandbyMhz(cur, dir);
      queueRadioTune(radioSelected_, next);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace avionics
