#include "avionics/FplRouteEdit.h"
#include "avionics/MfdController.h"
#include "avionics/ProcedureMenu.h"

// MFD Procedures window (PROC bezel on the FPL page). Host wiring for the
// shared ProcedureMenu logic used by the PFD.
namespace avionics {

ProcedureMenuHost MfdController::procedureMenuHost() {
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
      &fplApproachHeaderLabel_,
      [this]() { return procDefaultAirportIcao(); },
      [this]() { return nearestAirportIds(mapData_); },
      [this]() { fplPublishEdit(); },
      [this](int leg) { requestActivateFlightPlanLeg(leg); },
      [this]() { procActivateMissedPending_ = true; },
      [this]() { procMenuOpen_ = false; },
      [this]() { return minsBaroOn_; },
      [this](bool on) { minsBaroOn_ = on; },
      [this]() { return minsAltFt_; },
      [this](float ft) { minsAltFt_ = ft; },
  };
}

ProcedureMenuHost MfdController::procedureMenuHost() const {
  return const_cast<MfdController*>(this)->procedureMenuHost();
}

std::vector<MapApproach> MfdController::approachesForAirport(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->approachesForAirport(icao);
}

std::vector<MapProcedure> MfdController::proceduresForAirport(
    const std::string& icao, ProcedureType type) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->proceduresForAirport(icao, type);
}

std::vector<MapProcedure> MfdController::proceduresFor(ProcedureType type) const {
  return proceduresForAirport(procAirportIcao(), type);
}

std::string MfdController::procDefaultAirportIcao() const {
  if (procMenu_.category == ProcedureType::Departure && !fplLegs_.empty() &&
      isAirportIdent(fplLegs_.front().id)) {
    return fplLegs_.front().id;
  }

  std::string icao = lastAirportInPlan(fplLegs_);
  if (!icao.empty()) return icao;

  icao = directToAirportIcao(mapData_);
  if (!icao.empty()) return icao;

  if (mapData_ != nullptr) {
    icao = lastAirportInPlan(mapData_->flightPlan);
    if (!icao.empty()) return icao;
  }

  if (isAirportIdent(activeWaypoint_)) return activeWaypoint_;

  return nearestAirportIcao(mapData_);
}

bool MfdController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procMenu_.loadPending) return false;
  procMenu_.loadPending = false;
  out = procMenu_.loadTarget;
  return true;
}

bool MfdController::consumeActivateMissedRequest() {
  if (!procActivateMissedPending_) return false;
  procActivateMissedPending_ = false;
  return true;
}

std::vector<MapAirportFrequency> MfdController::airportFrequencies(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->airportFrequencies(icao);
}

std::vector<AirportRunwayInfo> MfdController::airportRunways(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  std::vector<AirportRunwayInfo> runways = navSource_->airportRunways(icao);
  std::stable_sort(runways.begin(), runways.end(),
                   [](const AirportRunwayInfo& a, const AirportRunwayInfo& b) {
                     return a.lengthFt > b.lengthFt;
                   });
  return runways;
}

void MfdController::buildProcMenu() {
  ProcedureMenuHost host = procedureMenuHost();
  procedureMenuBuild(host);
}

void MfdController::openProcApproachLoading(const std::string& icao,
                                            const std::string& approachName,
                                            const std::string& transition,
                                            ProcLoadingList openList) {
  procMenuOpen_ = true;
  procPreviewRangeManual_ = false;
  buildProcMenu();
  ProcedureMenuHost host = procedureMenuHost();
  procedureMenuOpenApproachLoading(host, icao, approachName, transition,
                                   openList);
}

const char* MfdController::procWindowTitle() const {
  return procedureMenuWindowTitle(procedureMenuHost());
}

const std::string& MfdController::procMenuItemText(int i) const {
  return procedureMenuItemText(procedureMenuHost(), i);
}

bool MfdController::procMenuItemEnabled(int i) const {
  return procedureMenuItemEnabled(procedureMenuHost(), i);
}

std::string MfdController::procAirportIcao() const {
  return procedureMenuAirportIcao(procedureMenuHost());
}

MapFeature MfdController::procAirportFeature() const {
  return procedureMenuAirportFeature(procedureMenuHost());
}

std::vector<std::string> MfdController::procProcedureNames(
    ProcedureType type) const {
  return procedureMenuProcedureNames(procedureMenuHost(), type);
}

std::vector<std::string> MfdController::procTransitions(
    ProcedureType type, const std::string& name) const {
  return procedureMenuTransitions(procedureMenuHost(), type, name);
}

std::vector<std::string> MfdController::procTransitionLabels(
    ProcedureType type, const std::string& name) const {
  return procedureMenuTransitionLabels(procedureMenuHost(), type, name);
}

std::vector<std::string> MfdController::procListItems() const {
  return procedureMenuListItems(procedureMenuHost());
}

std::string MfdController::procApproachDisplayName(int index) const {
  return procedureMenuApproachDisplayName(procedureMenuHost(), index);
}

std::string MfdController::procAirportCityLine() const {
  return procedureMenuAirportCityLine(procedureMenuHost());
}

std::string MfdController::procAirportNameLine() const {
  return procedureMenuAirportNameLine(procedureMenuHost());
}

std::string MfdController::procSelectedApproachDisplay() const {
  return procedureMenuSelectedApproachDisplay(procedureMenuHost());
}

std::string MfdController::procSelectedTransitionDisplay() const {
  return procedureMenuSelectedTransitionDisplay(procedureMenuHost());
}

float MfdController::procPrimaryFreqMhz() const {
  return procedureMenuPrimaryFreqMhz(procedureMenuHost());
}

bool MfdController::procPrimaryNavIsNdb() const {
  return procedureMenuPrimaryNavIsNdb(procedureMenuHost());
}

bool MfdController::procShowsPrimaryNavFreq() const {
  return procedureMenuShowsPrimaryNavFreq(procedureMenuHost());
}

std::string MfdController::procPrimaryIdent() const {
  return procedureMenuPrimaryIdent(procedureMenuHost());
}

std::vector<MapLeg> MfdController::procPreviewLegs() const {
  if (!procMenuOpen_) return {};
  return procedureMenuPreviewLegs(procedureMenuHost());
}

bool MfdController::procBezelKey(BezelKey key) {
  if (isMapRangePanBezelKey(key)) return false;
  ProcedureMenuHost host = procedureMenuHost();
  return procedureMenuBezelKey(host, key);
}

}  // namespace avionics
