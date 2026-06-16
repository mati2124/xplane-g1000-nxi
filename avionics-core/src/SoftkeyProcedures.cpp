#include <algorithm>

#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8). Mirrors the real
// unit's Procedures menu (Working Title PFDProc): a top-level menu of
// approach-activation and procedure-selection items, then a selection
// sub-window that loads the chosen departure / arrival / approach into the
// active flight plan (reusing the PFD flight-plan working copy and its
// shell-publish path). The selection/leg-expansion logic mirrors the MFD's
// MfdControllerProcedures.
namespace avionics {
namespace {

const std::string kEmptyString;

// Insert a procedure's legs at the conventional place in the plan: a departure
// after the origin, an arrival before the destination, an approach at the end
// (mirrors MfdControllerProcedures::insertProcedureLegs).
void insertProcedureLegs(ProcedureType type, std::vector<MapLeg>& fplLegs,
                         const std::vector<MapLeg>& legs) {
  if (legs.empty()) return;
  int row = 0;
  if (type == ProcedureType::Departure) {
    row = fplLegs.empty() ? 0 : 1;
  } else if (type == ProcedureType::Arrival) {
    row = std::max(0, static_cast<int>(fplLegs.size()) - 1);
  } else {
    row = static_cast<int>(fplLegs.size());
  }
  fplLegs.insert(fplLegs.begin() + row, legs.begin(), legs.end());
}

MapProcedure findProcedure(ProcedureType type, const std::string& name,
                           const std::string& transition,
                           const std::vector<MapProcedure>& catalog) {
  for (const MapProcedure& proc : catalog) {
    if (proc.type == type && proc.name == name &&
        proc.transition == transition) {
      return proc;
    }
  }
  MapProcedure fallback;
  fallback.type = type;
  fallback.name = name;
  fallback.transition = transition;
  return fallback;
}

}  // namespace

void SoftkeyController::buildProcMenu() {
  procMenuItems_ = {
      {"Activate Vector-to-Final", ProcMenuAction::ActivateVtf, false},
      {"Activate Approach", ProcMenuAction::ActivateApproach, false},
      {"Activate Missed Approach", ProcMenuAction::ActivateMissed, false},
      {"Select Approach", ProcMenuAction::SelectApproach, true},
      {"Select Arrival", ProcMenuAction::SelectArrival, true},
      {"Select Departure", ProcMenuAction::SelectDeparture, true},
  };
  procMode_ = ProcMode::Menu;
  procStep_ = ProcStep::ProcedureList;
  procSelected_ = 0;
  procSelectedName_.clear();
  // Land the cursor on the first enabled row (the activate items are disabled).
  procMenuSel_ = 0;
  for (int i = 0; i < static_cast<int>(procMenuItems_.size()); ++i) {
    if (procMenuItems_[static_cast<std::size_t>(i)].enabled) {
      procMenuSel_ = i;
      break;
    }
  }
}

const char* SoftkeyController::procWindowTitle() const {
  if (procMode_ == ProcMode::Menu) return "Procedures";
  switch (procCategory_) {
    case ProcedureType::Departure:
      return "Select Departure";
    case ProcedureType::Arrival:
      return "Select Arrival";
    case ProcedureType::Approach:
    default:
      return "Select Approach";
  }
}

const std::string& SoftkeyController::procMenuItemText(int i) const {
  if (i < 0 || i >= static_cast<int>(procMenuItems_.size())) return kEmptyString;
  return procMenuItems_[static_cast<std::size_t>(i)].text;
}

bool SoftkeyController::procMenuItemEnabled(int i) const {
  if (i < 0 || i >= static_cast<int>(procMenuItems_.size())) return false;
  return procMenuItems_[static_cast<std::size_t>(i)].enabled;
}

std::string SoftkeyController::procAirportIcao() const {
  // Departure uses the origin airport; otherwise the last airport-length id in
  // the plan (the destination), falling back to the active waypoint.
  if (procCategory_ == ProcedureType::Departure && !fplLegs_.empty() &&
      fplLegs_.front().id.size() == 4) {
    return fplLegs_.front().id;
  }
  for (int i = static_cast<int>(fplLegs_.size()) - 1; i >= 0; --i) {
    if (fplLegs_[static_cast<std::size_t>(i)].id.size() == 4) {
      return fplLegs_[static_cast<std::size_t>(i)].id;
    }
  }
  return activeWaypoint_;
}

std::vector<std::string> SoftkeyController::procProcedureNames(
    ProcedureType type) const {
  std::vector<std::string> names;
  if (navSource_ == nullptr || !navSource_->ready()) return names;
  const std::string icao = procAirportIcao();
  if (icao.empty()) return names;
  for (const MapProcedure& proc : navSource_->proceduresForAirport(icao, type)) {
    if (std::find(names.begin(), names.end(), proc.name) == names.end()) {
      names.push_back(proc.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> SoftkeyController::procTransitions(
    ProcedureType type, const std::string& name) const {
  std::vector<std::string> transitions;
  if (navSource_ == nullptr || !navSource_->ready()) return transitions;
  const std::string icao = procAirportIcao();
  if (icao.empty()) return transitions;
  for (const MapProcedure& proc : navSource_->proceduresForAirport(icao, type)) {
    if (proc.name != name) continue;
    if (std::find(transitions.begin(), transitions.end(), proc.transition) ==
        transitions.end()) {
      transitions.push_back(proc.transition);
    }
  }
  std::sort(transitions.begin(), transitions.end());
  return transitions;
}

std::vector<std::string> SoftkeyController::procListItems() const {
  if (procStep_ == ProcStep::TransitionList) {
    return procTransitions(procCategory_, procSelectedName_);
  }
  return procProcedureNames(procCategory_);
}

bool SoftkeyController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procLoadPending_) return false;
  procLoadPending_ = false;
  out = procLoadTarget_;
  return true;
}

void SoftkeyController::procMoveMenu(int dir) {
  const int n = static_cast<int>(procMenuItems_.size());
  if (n == 0) return;
  for (int step = 0; step < n; ++step) {
    procMenuSel_ = (procMenuSel_ + dir + n) % n;
    if (procMenuItems_[static_cast<std::size_t>(procMenuSel_)].enabled) break;
  }
}

void SoftkeyController::procLoadSelected(const std::string& name,
                                         const std::string& transition) {
  if (navSource_ == nullptr || !navSource_->ready()) return;
  const std::string icao = procAirportIcao();
  std::vector<MapLeg> legs =
      navSource_->expandProcedure(icao, procCategory_, name, transition);
  if (legs.empty()) return;
  insertProcedureLegs(procCategory_, fplLegs_, legs);
  fplCursorRow_ = static_cast<int>(fplLegs_.size());
  flightPlanPublishEdit();
  procLoadTarget_ = findProcedure(procCategory_, name, transition,
                                  navSource_->proceduresForAirport(
                                      icao, procCategory_));
  procLoadPending_ = true;
  // Loading closes the Procedures window; the new legs show on the FPL window.
  window_ = PfdWindow::None;
  procMode_ = ProcMode::Menu;
  procStep_ = ProcStep::ProcedureList;
  procSelectedName_.clear();
}

bool SoftkeyController::procBezelKey(BezelKey key) {
  // Top-level menu: the knob moves between enabled rows, ENT opens the
  // selection sub-window for a "Select ..." item, CLR / knob push closes.
  if (procMode_ == ProcMode::Menu) {
    switch (key) {
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsOuterCw:
        procMoveMenu(+1);
        return true;
      case BezelKey::FmsInnerCcw:
      case BezelKey::FmsOuterCcw:
        procMoveMenu(-1);
        return true;
      case BezelKey::Ent: {
        if (procMenuSel_ < 0 ||
            procMenuSel_ >= static_cast<int>(procMenuItems_.size())) {
          return true;
        }
        const ProcMenuItem& item =
            procMenuItems_[static_cast<std::size_t>(procMenuSel_)];
        if (!item.enabled) return true;
        switch (item.action) {
          case ProcMenuAction::SelectDeparture:
            procCategory_ = ProcedureType::Departure;
            break;
          case ProcMenuAction::SelectArrival:
            procCategory_ = ProcedureType::Arrival;
            break;
          case ProcMenuAction::SelectApproach:
            procCategory_ = ProcedureType::Approach;
            break;
          default:
            return true;  // activate items are inert in this suite
        }
        procMode_ = ProcMode::Select;
        procStep_ = ProcStep::ProcedureList;
        procSelected_ = 0;
        procSelectedName_.clear();
        return true;
      }
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        window_ = PfdWindow::None;
        return true;
      default:
        return true;  // modal over the FMS knob while open
    }
  }

  // Selection sub-window (mirrors MfdController::procBezelKey, minus the
  // DEP/ARR/APP category cycling since the category is fixed by the menu).
  const std::vector<std::string> items = procListItems();
  switch (key) {
    case BezelKey::Ent:
      if (procStep_ == ProcStep::ProcedureList) {
        if (procSelected_ >= 0 &&
            procSelected_ < static_cast<int>(items.size())) {
          const std::string& name = items[static_cast<std::size_t>(procSelected_)];
          const std::vector<std::string> trans =
              procTransitions(procCategory_, name);
          if (trans.size() == 1) {
            procLoadSelected(name, trans.front());
          } else if (trans.size() > 1) {
            procSelectedName_ = name;
            procStep_ = ProcStep::TransitionList;
            procSelected_ = 0;
          }
        }
      } else if (procSelected_ >= 0 &&
                 procSelected_ < static_cast<int>(items.size())) {
        procLoadSelected(procSelectedName_,
                         items[static_cast<std::size_t>(procSelected_)]);
      }
      return true;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      if (procStep_ == ProcStep::TransitionList) {
        procStep_ = ProcStep::ProcedureList;
        procSelectedName_.clear();
        procSelected_ = 0;
      } else {
        procMode_ = ProcMode::Menu;  // back to the top-level menu
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsOuterCw:
      if (!items.empty()) {
        procSelected_ = (procSelected_ + 1) % static_cast<int>(items.size());
      }
      return true;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsOuterCcw:
      if (!items.empty()) {
        procSelected_ = (procSelected_ - 1 + static_cast<int>(items.size())) %
                        static_cast<int>(items.size());
      }
      return true;
    default:
      return true;
  }
}

}  // namespace avionics
