#include "avionics/FplRouteEdit.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/ProcedureSupport.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8). Bezel routing and
// host wiring for the shared ProcedureMenu logic.
namespace avionics {

ProcedureMenuHost SoftkeyController::procedureMenuHost() {
  return ProcedureMenuHost{
      procMenu_,
      navSource_,
      mapData_,
      fplLegs_,
      fplApproachLegStart_,
      fplApproachLegCount_,
      fplCursorRow_,
      fplLoadedApproach_,
      persistedApproachRestore_,
      nullptr,
      [this]() { return procDefaultAirportIcao(); },
      [this]() { return nearestAirportIds(mapData_); },
      [this]() { flightPlanPublishEdit(); },
      [this](int leg) { requestDirectToFlightPlanLeg(leg); },
      [this]() { requestActivateMissedApproach(); },
      [this]() { window_ = PfdWindow::None; },
      [this]() { return minsMode_ == MinimumsMode::Baro; },
      [this](bool on) {
        minsMode_ = on ? MinimumsMode::Baro : MinimumsMode::Off;
      },
      [this]() { return minsAltFt_; },
      [this](float ft) { minsAltFt_ = ft; },
  };
}

ProcedureMenuHost SoftkeyController::procedureMenuHost() const {
  return const_cast<SoftkeyController*>(this)->procedureMenuHost();
}

void SoftkeyController::buildProcMenu() {
  ProcedureMenuHost host = procedureMenuHost();
  procedureMenuBuild(host);
}

const char* SoftkeyController::procWindowTitle() const {
  return procedureMenuWindowTitle(procedureMenuHost());
}

const std::string& SoftkeyController::procMenuItemText(int i) const {
  return procedureMenuItemText(procedureMenuHost(), i);
}

bool SoftkeyController::procMenuItemEnabled(int i) const {
  return procedureMenuItemEnabled(procedureMenuHost(), i);
}

std::string SoftkeyController::procDefaultAirportIcao() const {
  if (procCategory() == ProcedureType::Departure && !fplLegs_.empty() &&
      isAirportIdent(fplLegs_.front().id)) {
    return fplLegs_.front().id;
  }

  const int approachEnd = fplApproachLegCount_ > 0
                              ? fplApproachLegStart_
                              : static_cast<int>(fplLegs_.size());
  std::string icao = airportIcaoBeforeIndex(fplLegs_, approachEnd);
  if (!icao.empty()) return icao;

  icao = directToAirportIcao(mapData_);
  if (!icao.empty()) return icao;

  if (mapData_ != nullptr) {
    icao = lastAirportInPlan(mapData_->flightPlan);
    if (!icao.empty()) return icao;
  }

  icao = lastAirportInPlan(fplLegs_);
  if (!icao.empty()) return icao;

  if (isAirportIdent(activeWaypoint_)) return activeWaypoint_;

  return nearestAirportIcao(mapData_);
}

std::string SoftkeyController::procAirportIcao() const {
  return procedureMenuAirportIcao(procedureMenuHost());
}

MapFeature SoftkeyController::procAirportFeature() const {
  return procedureMenuAirportFeature(procedureMenuHost());
}

std::vector<std::string> SoftkeyController::procProcedureNames(
    ProcedureType type) const {
  return procedureMenuProcedureNames(procedureMenuHost(), type);
}

std::vector<std::string> SoftkeyController::procTransitions(
    ProcedureType type, const std::string& name) const {
  return procedureMenuTransitions(procedureMenuHost(), type, name);
}

std::string SoftkeyController::formatApproachLabel(const MapProcedure& proc) const {
  return formatApproachProcedureLabel(proc);
}

std::string SoftkeyController::procApproachDisplayName(int index) const {
  return procedureMenuApproachDisplayName(procedureMenuHost(), index);
}

MapProcedure SoftkeyController::procSelectedProcedure() const {
  return procedureMenuSelectedProcedure(procedureMenuHost());
}

std::string SoftkeyController::procAirportCityLine() const {
  return procedureMenuAirportCityLine(procedureMenuHost());
}

std::string SoftkeyController::procAirportNameLine() const {
  return procedureMenuAirportNameLine(procedureMenuHost());
}

std::string SoftkeyController::procSelectedApproachDisplay() const {
  return procedureMenuSelectedApproachDisplay(procedureMenuHost());
}

std::string SoftkeyController::procSelectedTransitionDisplay() const {
  return procedureMenuSelectedTransitionDisplay(procedureMenuHost());
}

float SoftkeyController::procPrimaryFreqMhz() const {
  return procedureMenuPrimaryFreqMhz(procedureMenuHost());
}

bool SoftkeyController::procPrimaryNavIsNdb() const {
  return procedureMenuPrimaryNavIsNdb(procedureMenuHost());
}

bool SoftkeyController::procShowsPrimaryNavFreq() const {
  return procedureMenuShowsPrimaryNavFreq(procedureMenuHost());
}

std::string SoftkeyController::procPrimaryIdent() const {
  return procedureMenuPrimaryIdent(procedureMenuHost());
}

std::vector<std::string> SoftkeyController::procListItems() const {
  return procedureMenuListItems(procedureMenuHost());
}

bool SoftkeyController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procMenu_.loadPending) return false;
  procMenu_.loadPending = false;
  out = procMenu_.loadTarget;
  return true;
}

std::string SoftkeyController::flightPlanApproachAirportIcao() const {
  if (fplApproachLegCount_ <= 0) return {};
  const std::string loadedIcao =
      persistedApproachRestore_.active ? persistedApproachRestore_.airportIcao
                                       : std::string();
  return fplApproachAirportIcao(fplLegs_, fplApproachLegStart_, mapData_, loadedIcao);
}

std::string SoftkeyController::flightPlanApproachHeaderLabel() const {
  if (fplApproachLegCount_ <= 0) return {};
  if (!fplLoadedApproach_.name.empty()) {
    return formatApproachFplHeaderLabel(fplLoadedApproach_);
  }
  if (persistedApproachRestore_.active &&
      !persistedApproachRestore_.name.empty()) {
    return formatApproachFplHeaderLabel(
        mapProcedureFromPersisted(persistedApproachRestore_));
  }
  return {};
}

bool SoftkeyController::procBezelKey(BezelKey key) {
  if (isPageNavigationBezelKey(key)) {
    pageMenuOpen_ = false;
    return false;
  }
  ProcedureMenuHost host = procedureMenuHost();
  return procedureMenuBezelKey(host, key);
}

}  // namespace avionics
