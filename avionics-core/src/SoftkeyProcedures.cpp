#include "avionics/FplRouteEdit.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/ProcedureSupport.h"
#include "avionics/SoftkeyController.h"
#include "avionics/SimBriefOfpSupport.h"
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
      &fplDepartureLegStart_,
      &fplDepartureLegCount_,
      &fplLoadedDeparture_,
      &persistedDepartureRestore_,
      &fplDepartureHeaderLabel_,
      &fplArrivalLegStart_,
      &fplArrivalLegCount_,
      &fplLoadedArrival_,
      &persistedArrivalRestore_,
      &fplArrivalHeaderLabel_,
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
  if (procCategory() == ProcedureType::Departure) {
    std::string origin = firstKnownAirportInPlan(fplLegs_, mapData_, navSource_);
    if (origin.empty() && !fplLegs_.empty() &&
        isAirportIdent(fplLegs_.front().id)) {
      origin = fplLegs_.front().id;
    }
    if (!origin.empty()) return origin;
  }

  std::string loadedApproachIcao;
  if (persistedApproachRestore_.active) {
    loadedApproachIcao = persistedApproachRestore_.airportIcao;
  }
  std::string dest = fplDestinationAirportIcao(
      {fplLegs_,
       fplDestinationFilled_,
       fplApproachLegStart_,
       fplApproachLegCount_,
       fplArrivalLegStart_,
       fplArrivalLegCount_,
       mapData_,
       navSource_,
       loadedApproachIcao,
       flightPlanArrivalAirportIcao(),
       {}});
  if (!dest.empty()) return dest;

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

std::string SoftkeyController::procAirportEntryCityLine() const {
  return procedureMenuAirportCityLineFor(procMenu_.airportEntry.match);
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

std::string SoftkeyController::procSelectedRunwayDisplay() const {
  return procedureMenuSelectedRunwayDisplay(procedureMenuHost());
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
  std::string loadedIcao =
      persistedApproachRestore_.active ? persistedApproachRestore_.airportIcao
                                       : std::string();
  // The approach serves the destination airport — the same airport a loaded
  // arrival/STAR serves. Use it when the approach's own airport wasn't captured
  // (e.g. the approach was inferred from leg roles after a sim/Direct-To resync)
  // so the destination is not mislabeled as the airport before the approach.
  if (loadedIcao.empty()) loadedIcao = flightPlanArrivalAirportIcao();
  const std::string headerLabel = flightPlanApproachHeaderLabel();
  if (loadedIcao.empty() && !headerLabel.empty()) {
    const std::size_t dash = headerLabel.find('-');
    if (dash != std::string::npos) {
      const std::string prefix = headerLabel.substr(0, dash);
      if (isAirportIdent(prefix)) loadedIcao = prefix;
    }
  }
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

std::string SoftkeyController::flightPlanDepartureAirportIcao() const {
  if (!persistedDepartureRestore_.airportIcao.empty()) {
    return persistedDepartureRestore_.airportIcao;
  }
  return fplLoadedDeparture_.name.empty() || fplLegs_.empty()
             ? std::string()
             : fplLegs_.front().id;
}

std::string SoftkeyController::flightPlanDepartureHeaderLabel() const {
  if (!fplDepartureHeaderLabel_.empty()) return fplDepartureHeaderLabel_;
  if (!fplLoadedDeparture_.name.empty()) {
    return formatTerminalProcedureFplHeaderLabel(
        fplLoadedDeparture_.runway, fplLoadedDeparture_.name,
        fplLoadedDeparture_.transition);
  }
  if (persistedDepartureRestore_.active &&
      !persistedDepartureRestore_.name.empty()) {
    const MapProcedure proc =
        mapProcedureFromPersisted(persistedDepartureRestore_);
    return formatTerminalProcedureFplHeaderLabel(
        proc.runway, proc.name, proc.transition);
  }
  return {};
}

std::string SoftkeyController::flightPlanArrivalAirportIcao() const {
  if (!persistedArrivalRestore_.airportIcao.empty()) {
    return persistedArrivalRestore_.airportIcao;
  }
  return fplLoadedArrival_.name.empty() || fplLegs_.empty()
             ? std::string()
             : fplLegs_.back().id;
}

std::string SoftkeyController::flightPlanArrivalHeaderLabel() const {
  if (!fplArrivalHeaderLabel_.empty()) return fplArrivalHeaderLabel_;
  if (!fplLoadedArrival_.name.empty()) {
    return formatTerminalProcedureFplHeaderLabel(
        fplLoadedArrival_.runway, fplLoadedArrival_.name,
        fplLoadedArrival_.transition);
  }
  if (persistedArrivalRestore_.active &&
      !persistedArrivalRestore_.name.empty()) {
    const MapProcedure proc = mapProcedureFromPersisted(persistedArrivalRestore_);
    return formatTerminalProcedureFplHeaderLabel(
        proc.runway, proc.name, proc.transition);
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

bool SoftkeyController::courseReversalPromptBezelKey(BezelKey key) {
  if (!procMenu_.courseReversalPromptActive) return false;
  switch (key) {
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      procMenu_.courseReversalYes = !procMenu_.courseReversalYes;
      procMenu_.courseReversalYesDirty = true;
      return true;
    case BezelKey::Ent: {
      ProcedureMenuHost host = procedureMenuHost();
      procedureMenuAnswerCourseReversal(host, procMenu_.courseReversalYes);
      return true;
    }
    case BezelKey::Clr:
    case BezelKey::FmsPush: {
      ProcedureMenuHost host = procedureMenuHost();
      procedureMenuAnswerCourseReversal(host, false);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace avionics
