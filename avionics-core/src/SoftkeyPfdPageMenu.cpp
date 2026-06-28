#include "avionics/SoftkeyController.h"

#include "avionics/FplRouteEdit.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {

namespace {
constexpr const char* kEmptyItem = "";
constexpr int kPageMenuVisibleRows = 3;
}  // namespace

std::vector<SoftkeyController::PfdPageMenuItem>
SoftkeyController::buildPfdPageMenu() const {
  switch (window_) {
    case PfdWindow::References:
      return {{"All References On", PfdPageMenuAction::RefAllOn},
              {"All References Off", PfdPageMenuAction::RefAllOff},
              {"Restore Defaults", PfdPageMenuAction::RefRestoreDefaults}};
    case PfdWindow::FlightPlan: {
      // The PFD Active Flight Plan window page menu (trainer): Activate Leg,
      // Load Airway, and Collapse/Expand Airways, with Delete Flight Plan below.
      // Delete Flight Plan moved here from the bare MENU shortcut so MENU now
      // opens the real page menu like the trainer.
      const FplRouteEdit edit = flightPlanRouteEditState();
      const std::string approachAirport = flightPlanApproachAirportIcao();
      const int cursorLeg = fplCursorLegIndex(edit, approachAirport,
                                              FplCursorLayout::SectionRows);
      const int legCount = static_cast<int>(fplLegs_.size());

      // Activate Leg is live only with the cursor on a real plan leg.
      const PfdPageMenuAction activateState =
          (fplCursorOn_ && cursorLeg >= 0 && cursorLeg < legCount)
              ? PfdPageMenuAction::FplActivateLeg
              : PfdPageMenuAction::Disabled;

      // Load Airway is live only when the cursor sits on a fix that lies on a
      // published airway (Pilot's Guide, Load Airway); otherwise it greys out.
      PfdPageMenuAction loadAirwayState = PfdPageMenuAction::Disabled;
      if (fplCursorOn_ && cursorLeg >= 0 && cursorLeg < legCount) {
        const std::string& ident =
            fplLegs_[static_cast<std::size_t>(cursorLeg)].id;
        if (!airwaysThroughFix(ident).empty()) {
          loadAirwayState = PfdPageMenuAction::FplLoadAirway;
        }
      }

      // Collapse/Expand Airways toggles the loaded-airway display; it is live
      // (and only changes its label) when the plan carries airway legs.
      const bool hasAirways = flightPlanHasAirwayLegs();
      const char* collapseText =
          fplAirwaysCollapsed_ ? "Expand Airways" : "Collapse Airways";
      const PfdPageMenuAction collapseState =
          hasAirways ? PfdPageMenuAction::FplCollapseAirways
                     : PfdPageMenuAction::DisplayOnly;

      const PfdPageMenuAction deleteState =
          legCount > 0 ? PfdPageMenuAction::FplDeleteFlightPlan
                       : PfdPageMenuAction::Disabled;
      return {
          {"Activate Leg", activateState},
          {"Load Airway", loadAirwayState},
          {collapseText, collapseState},
          {"Delete Flight Plan", deleteState},
      };
    }
    case PfdWindow::Nearest:
    case PfdWindow::Alerts:
      return {};
    default:
      return {};
  }
}

void SoftkeyController::openPfdPageMenu() {
  pageMenuItems_ = buildPfdPageMenu();
  pageMenuOpen_ = true;
  pageMenuSel_ = 0;
  pageMenuScroll_ = 0;
  for (int i = 0; i < static_cast<int>(pageMenuItems_.size()); ++i) {
    if (pageMenuItems_[static_cast<std::size_t>(i)].action !=
        PfdPageMenuAction::Disabled) {
      pageMenuSel_ = i;
      break;
    }
  }
}

const std::string& SoftkeyController::pageMenuItemText(int i) const {
  static const std::string kNoOptions = "No Options";
  static const std::string kEmpty;
  if (pageMenuItems_.empty()) return kNoOptions;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return kEmpty;
  return pageMenuItems_[static_cast<std::size_t>(i)].text;
}

bool SoftkeyController::pageMenuItemEnabled(int i) const {
  if (pageMenuItems_.empty()) return false;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return false;
  return pageMenuItems_[static_cast<std::size_t>(i)].action !=
         PfdPageMenuAction::Disabled;
}

void SoftkeyController::pageMenuStep(int direction) {
  const int n = static_cast<int>(pageMenuItems_.size());
  if (n <= 0) return;
  const int step = direction >= 0 ? 1 : -1;
  for (int tries = 0; tries < n; ++tries) {
    pageMenuSel_ = (pageMenuSel_ + step + n) % n;
    if (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action !=
        PfdPageMenuAction::Disabled) {
      break;
    }
  }
  if (pageMenuSel_ < pageMenuScroll_) {
    pageMenuScroll_ = pageMenuSel_;
  } else if (pageMenuSel_ >= pageMenuScroll_ + kPageMenuVisibleRows) {
    pageMenuScroll_ = pageMenuSel_ - kPageMenuVisibleRows + 1;
  }
}

void SoftkeyController::pageMenuActivate() {
  if (pageMenuSel_ < 0 ||
      pageMenuSel_ >= static_cast<int>(pageMenuItems_.size())) {
    return;
  }
  switch (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action) {
    case PfdPageMenuAction::RefAllOn:
      vspeedOn_.fill(true);
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::RefAllOff:
      vspeedOn_.fill(false);
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::RefRestoreDefaults:
      for (int i = 0; i < kVspeedRefCount; ++i) {
        vspeedKt_[i] = kDefaultVspeedKt[i];
      }
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::FplActivateLeg: {
      pageMenuOpen_ = false;
      const FplRouteEdit edit = flightPlanRouteEditState();
      const int cursorLeg = fplCursorLegIndex(
          edit, flightPlanApproachAirportIcao(), FplCursorLayout::SectionRows);
      if (cursorLeg >= 0 && cursorLeg < static_cast<int>(fplLegs_.size())) {
        requestActivateFlightPlanLeg(cursorLeg);
      }
      break;
    }
    case PfdPageMenuAction::FplLoadAirway: {
      pageMenuOpen_ = false;  // the page menu closes as the window opens
      const FplRouteEdit edit = flightPlanRouteEditState();
      const int cursorLeg = fplCursorLegIndex(
          edit, flightPlanApproachAirportIcao(), FplCursorLayout::SectionRows);
      if (cursorLeg >= 0 && cursorLeg < static_cast<int>(fplLegs_.size())) {
        openLoadAirwayWindow(fplLegs_[static_cast<std::size_t>(cursorLeg)].id);
      }
      break;
    }
    case PfdPageMenuAction::FplCollapseAirways: {
      fplAirwaysCollapsed_ = !fplAirwaysCollapsed_;
      // The row count changes; keep the cursor in range.
      FplRouteEdit edit = flightPlanRouteEditState();
      fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                        FplCursorLayout::SectionRows);
      pageMenuOpen_ = false;
      break;
    }
    case PfdPageMenuAction::FplDeleteFlightPlan:
      pageMenuOpen_ = false;  // the page menu closes as the confirmation opens
      fplConfirm_ = FplConfirm::DeleteFlightPlan;
      fplConfirmOk_ = true;
      break;
    case PfdPageMenuAction::DisplayOnly:
    case PfdPageMenuAction::Disabled:
      break;
  }
}

bool SoftkeyController::pageMenuBezelKey(BezelKey key) {
  if (!pageMenuOpen_) return false;
  switch (key) {
    case BezelKey::Ent:
      if (!pageMenuItems_.empty()) pageMenuActivate();
      return true;
    case BezelKey::Clr:
    case BezelKey::Menu:
    case BezelKey::FmsPush:
      pageMenuOpen_ = false;
      return true;
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsInnerCw:
      if (!pageMenuItems_.empty()) pageMenuStep(+1);
      return true;
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsInnerCcw:
      if (!pageMenuItems_.empty()) pageMenuStep(-1);
      return true;
    default:
      return true;
  }
}

}  // namespace avionics
