#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

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

}  // namespace

SoftkeyController::SoftkeyController() : menuStack_{SoftkeyMenu::Root} {
  // The inset map ships with the topographic background on, matching the MFD
  // navigation map default.
  toggles_[static_cast<int>(DisplayToggle::MapTopo)] = true;
  // The Active Flight Plan insert entry may resolve airway names (Pilot's Guide
  // 5.6), like the MFD's FPL insert window.
  fplEntry_.allowAirways = true;
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

  // The Direct-To window (bezel-key driven) eases the same way. Cache the map
  // snapshot and the active waypoint so the window can resolve idents and seed
  // its default destination. Flight-plan sync runs in AvionicsEngine::update
  // after the data source is pumped so both GDUs adopt the same live route.
  mapData_ = &map;
  activeWaypoint_ = data.fmaToWpt;
  dtoAnim_ = approach(dtoAnim_, dtoOpen_ ? 1.0f : 0.0f, dt / kWindowAnimSeconds);
  pageMenuAnim_ =
      approach(pageMenuAnim_, pageMenuOpen_ ? 1.0f : 0.0f, dt / kWindowAnimSeconds);

  // ~1 Hz blink phase for flashing annunciations (Alerts softkey, Baro
  // Transition Alert): on for the first half of each second.
  blinkSeconds_ += dtSeconds;
  blinkOn_ = std::fmod(blinkSeconds_, 1.0) < 0.5;

  // The NAV/COM tuning cursor flashes for a few seconds after a tuning action,
  // then settles solid (Working Title NXi armed-border behavior).
  if (radioArmedSeconds_ > 0.0) {
    radioArmedSeconds_ = std::max(0.0, radioArmedSeconds_ - dtSeconds);
  }

  // The audio volume percentage replaces the standby frequency for two seconds
  // after a VOL/SQ or VOL/ID knob turn, then the standby frequency returns.
  if (radioVolumeShownSeconds_ > 0.0) {
    radioVolumeShownSeconds_ = std::max(0.0, radioVolumeShownSeconds_ - dtSeconds);
    if (radioVolumeShownSeconds_ == 0.0) radioVolumeBand_ = RadioBand::None;
  }

  if (radioXferAnimActive_) {
    radioXferAnim_.progress =
        static_cast<float>(radioXferAnim_.progress +
                           dtSeconds / kRadioTransferAnimSeconds);
    if (radioXferAnim_.progress >= 1.0f) {
      radioXferAnimActive_ = false;
      radioXferAnim_.progress = 1.0f;
    }
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

void SoftkeyController::toggleWindow(PfdWindow w) {
  pageMenuOpen_ = false;
  window_ = (window_ == w) ? PfdWindow::None : w;
  // Opening a window puts the FMS cursor at its first field/entry.
  if (window_ == PfdWindow::References) refCursor_ = RefField::TimerCmd;
  if (window_ == PfdWindow::Nearest) nearestCursor_ = 0;
  if (window_ == PfdWindow::FlightPlan) {
    fplListCursorFollowsActive_ = true;
    fplCursorOn_ = false;
    fplEntry_.reset();
    fplConfirm_ = FplConfirm::None;
  }
  // The Procedures window opens on its top-level menu (PROC key, 5.8).
  if (window_ == PfdWindow::Procedures) buildProcMenu();
  // The PFD Setup Menu opens with 'Auto' highlighted next to 'PFD Display'.
  if (window_ == PfdWindow::Setup) setupCursor_ = PfdSetupField::PfdMode;
}

bool SoftkeyController::keyEnabled(int i) const {
  if (i < 0 || i >= kSoftkeyCount) return false;
  const SoftkeyAction action = menuDefs(currentMenu())[i].action;
  if (action == SoftkeyAction::OpenSvt) return false;
  return action != SoftkeyAction::None;
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

  // The "Fly Course Reversal?" prompt is modal: it owns the FMS knob / ENT / CLR
  // until answered, overlaying whatever page is up.
  if (courseReversalPromptBezelKey(key)) return;

  // Hold Direct-To confirmation on a selected HOLD row is likewise modal.
  if (holdActivatePromptBezelKey(key)) return;

  // The Direct-To window owns the FMS knob / ENT / CLR while it is open; FPL,
  // PROC, and MENU dismiss it and navigate like the real unit.
  if (directToBezelKey(key)) return;

  // The Active Flight Plan window owns the FMS knob / ENT / CLR / MENU while it
  // is open (cursor on = edit, cursor off = scroll). Keys it does not use (the
  // range rocker, the FPL toggle) fall through.
  if (window_ == PfdWindow::FlightPlan && flightPlanBezelKey(key)) return;

  // The Procedures window owns the FMS knob / ENT / CLR while it is open; FPL,
  // PROC, and MENU fall through to navigate like the real unit.
  if (window_ == PfdWindow::Procedures && procBezelKey(key)) return;

  if (pageMenuOpen_ && pageMenuBezelKey(key)) return;

  if (key == BezelKey::RangeUp) {
    insetRangeIndex_ = std::min(kMapRangeLadderCount - 1, insetRangeIndex_ + 1);
    return;
  }
  if (key == BezelKey::RangeDown) {
    insetRangeIndex_ = std::max(0, insetRangeIndex_ - 1);
    return;
  }

  // FPL opens / closes the Active Flight Plan window (Pilot's Guide Fig. 5-48,
  // "Active Flight Plan Window on PFD").
  if (key == BezelKey::Fpl) {
    toggleWindow(PfdWindow::FlightPlan);
    return;
  }

  // PROC opens / closes the Procedures window (Pilot's Guide 5.8).
  if (key == BezelKey::Proc) {
    toggleWindow(PfdWindow::Procedures);
    return;
  }

  // MENU opens the PFD Setup Menu when no popout is active (Pilot's Guide
  // Fig. 1-18). On an open popout it opens that window's Page Menu (Fig. 1-10).
  if (key == BezelKey::Menu) {
    if (window_ == PfdWindow::None) {
      toggleWindow(PfdWindow::Setup);
    } else if (window_ == PfdWindow::Setup) {
      window_ = PfdWindow::None;
    } else if (pageMenuOpen_) {
      pageMenuOpen_ = false;
    } else {
      openPfdPageMenu();
    }
    return;
  }

  // CLR removes the page menu first, then any open pop-up window.
  if (key == BezelKey::Clr) {
    if (pageMenuOpen_) {
      pageMenuOpen_ = false;
      return;
    }
    if (window_ != PfdWindow::None) {
      window_ = PfdWindow::None;
      return;
    }
  }

  if (pageMenuOpen_) return;

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

  // Inside the PFD Setup Menu the FMS knob works as on the real unit: the large
  // knob moves the field cursor, the small knob changes the highlighted field,
  // and ENT confirms (advancing onto the intensity once a row is set Manual).
  if (window_ == PfdWindow::Setup) {
    if (key == BezelKey::FmsOuterCw) movePfdSetupCursor(+1);
    if (key == BezelKey::FmsOuterCcw) movePfdSetupCursor(-1);
    if (key == BezelKey::FmsInnerCw || key == BezelKey::FmsInnerCcw) {
      adjustPfdSetupValue(key == BezelKey::FmsInnerCw ? +1 : -1);
    }
    if (key == BezelKey::Ent) activatePfdSetupField();
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

bool SoftkeyController::pressKey(int key) {
  if (key < 0 || key >= kSoftkeyCount) return false;

  if (!keyEnabled(key)) return false;

  const SoftkeyAction action = menuDefs(currentMenu())[key].action;

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
      pressAlertsSoftkey();
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
      setBaroStandard();
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

}  // namespace avionics
