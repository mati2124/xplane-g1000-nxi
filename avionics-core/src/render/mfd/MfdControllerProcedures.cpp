#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MfdController.h"

// Nav-database queries (approaches, procedures, frequencies, runways) and the
// PROC menu (PROC bezel key on the FPL page, Pilot's Guide 5.8): departures,
// arrivals, and approaches for the flight-plan airport, and loading the
// selected procedure's legs into the plan.
namespace avionics {
namespace {

bool isAirportIdent(const std::string& id) {
  if (id.size() != 4) return false;
  for (char c : id) {
    if (c < 'A' || c > 'Z') return false;
  }
  return true;
}

std::string directToAirportIcao(const MapData* map) {
  if (map == nullptr || !map->directToActive) return {};
  return isAirportIdent(map->directTo.id) ? map->directTo.id : std::string();
}

std::string lastAirportInPlan(const std::vector<MapLeg>& legs) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

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
    // Enroute feeder and IAF share a fix; the approach copy wins on the FPL.
    if (!fplLegs.empty() &&
        fplLegIdentsEqual(fplLegs.back().id, legs.front().id)) {
      fplLegs.pop_back();
      row = static_cast<int>(fplLegs.size());
    }
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

std::string MfdController::procAirportIcao() const {
  if (procCategory_ == ProcedureType::Departure && !fplLegs_.empty() &&
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
  return {};
}

bool MfdController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procLoadPending_) return false;
  procLoadPending_ = false;
  out = procLoadTarget_;
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
  // Primary (longest) runway first, the order the unit pages through them.
  std::stable_sort(runways.begin(), runways.end(),
                   [](const AirportRunwayInfo& a, const AirportRunwayInfo& b) {
                     return a.lengthFt > b.lengthFt;
                   });
  return runways;
}

std::vector<std::string> MfdController::procProcedureNames(
    ProcedureType type) const {
  std::vector<std::string> names;
  for (const MapProcedure& proc : proceduresFor(type)) {
    if (std::find(names.begin(), names.end(), proc.name) == names.end()) {
      names.push_back(proc.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> MfdController::procTransitions(
    ProcedureType type, const std::string& name) const {
  std::vector<std::string> transitions;
  if (navSource_ != nullptr && navSource_->ready() &&
      type == ProcedureType::Approach) {
    const std::vector<ApproachTransitionOption> options =
        navSource_->approachTransitionsFor(procAirportIcao(), name);
    if (!options.empty()) {
      transitions.reserve(options.size());
      for (const ApproachTransitionOption& opt : options) {
        transitions.push_back(opt.id);
      }
      return transitions;
    }
  }
  for (const MapProcedure& proc : proceduresFor(type)) {
    if (proc.name != name) continue;
    if (type == ProcedureType::Approach && proc.transition.size() >= 2 &&
        proc.transition[0] == 'R' && proc.transition[1] == 'W') {
      continue;
    }
    if (std::find(transitions.begin(), transitions.end(), proc.transition) ==
        transitions.end()) {
      transitions.push_back(proc.transition);
    }
  }
  std::sort(transitions.begin(), transitions.end());
  if (type == ProcedureType::Approach &&
      std::find(transitions.begin(), transitions.end(), "VECTORS") ==
          transitions.end()) {
    transitions.insert(transitions.begin(), "VECTORS");
  }
  return transitions;
}

std::vector<std::string> MfdController::procTransitionLabels(
    ProcedureType type, const std::string& name) const {
  if (navSource_ != nullptr && navSource_->ready() &&
      type == ProcedureType::Approach) {
    const std::vector<ApproachTransitionOption> options =
        navSource_->approachTransitionsFor(procAirportIcao(), name);
    if (!options.empty()) {
      std::vector<std::string> labels;
      labels.reserve(options.size());
      for (const ApproachTransitionOption& opt : options) {
        labels.push_back(opt.display);
      }
      return labels;
    }
  }
  return procTransitions(type, name);
}

std::vector<MapLeg> MfdController::procPreviewLegs() const {
  if (!procMenuOpen_ || navSource_ == nullptr || !navSource_->ready()) {
    return {};
  }
  const std::string icao = procAirportIcao();
  if (icao.empty()) return {};

  std::string name;
  std::string transition;
  if (procStep_ == ProcMenuStep::ProcedureList) {
    const std::vector<std::string> names = procProcedureNames(procCategory_);
    if (procSelected_ < 0 ||
        procSelected_ >= static_cast<int>(names.size())) {
      return {};
    }
    name = names[static_cast<std::size_t>(procSelected_)];
    const std::vector<std::string> transitions =
        procTransitions(procCategory_, name);
    if (transitions.empty()) return {};
    transition = transitions.front();
  } else {
    name = procSelectedName_;
    const std::vector<std::string> transitions =
        procTransitions(procCategory_, name);
    if (procSelected_ < 0 ||
        procSelected_ >= static_cast<int>(transitions.size())) {
      return {};
    }
    transition = transitions[static_cast<std::size_t>(procSelected_)];
  }
  return navSource_->expandProcedure(icao, procCategory_, name, transition);
}

bool MfdController::procBezelKey(BezelKey key) {
  if (isMapRangePanBezelKey(key)) return false;

  const std::vector<std::string> names = procProcedureNames(procCategory_);
  const std::vector<std::string> transitions =
      procStep_ == ProcMenuStep::TransitionList
          ? procTransitions(procCategory_, procSelectedName_)
          : std::vector<std::string>{};

  auto loadProcedure = [&](const std::string& name,
                           const std::string& transition) {
    if (navSource_ == nullptr || !navSource_->ready()) return;
    const std::string icao = procAirportIcao();
    std::vector<MapLeg> legs =
        navSource_->expandProcedure(icao, procCategory_, name, transition);
    if (legs.empty()) return;
    if (procCategory_ == ProcedureType::Approach) {
      fplApproachLegCount_ = static_cast<int>(legs.size());
    } else {
      fplApproachLegStart_ = 0;
      fplApproachLegCount_ = 0;
      fplLoadedApproach_ = {};
      fplApproachHeaderLabel_.clear();
    }
    insertProcedureLegs(procCategory_, fplLegs_, legs);
    if (procCategory_ == ProcedureType::Approach) {
      fplApproachLegStart_ =
          static_cast<int>(fplLegs_.size()) - fplApproachLegCount_;
      fplLoadedApproach_ = findProcedure(procCategory_, name, transition,
                                           proceduresFor(procCategory_));
      fplApproachHeaderLabel_ = formatApproachFplHeaderLabel(fplLoadedApproach_);
      persistedApproachRestore_ =
          persistedFromMapProcedure(fplLoadedApproach_, icao);
    }
    fplCursorRow_ = 0;
    fplPublishEdit();
    procLoadTarget_ = findProcedure(procCategory_, name, transition,
                                    proceduresFor(procCategory_));
    procLoadPending_ = true;
    procMenuOpen_ = false;
    procStep_ = ProcMenuStep::ProcedureList;
    procSelectedName_.clear();
  };

  switch (key) {
    case BezelKey::Ent:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        if (procSelected_ >= 0 &&
            procSelected_ < static_cast<int>(names.size())) {
          const std::string& name =
              names[static_cast<std::size_t>(procSelected_)];
          const std::vector<std::string> trans =
              procTransitions(procCategory_, name);
          if (trans.size() == 1) {
            loadProcedure(name, trans.front());
          } else if (trans.size() > 1) {
            procSelectedName_ = name;
            procStep_ = ProcMenuStep::TransitionList;
            procSelected_ = 0;
            for (int i = 0; i < static_cast<int>(trans.size()); ++i) {
              if (trans[static_cast<std::size_t>(i)] == "VECTORS") {
                procSelected_ = i;
                break;
              }
            }
          }
        }
      } else if (procSelected_ >= 0 &&
                 procSelected_ < static_cast<int>(transitions.size())) {
        loadProcedure(procSelectedName_,
                      transitions[static_cast<std::size_t>(procSelected_)]);
      }
      break;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      if (procStep_ == ProcMenuStep::TransitionList) {
        procStep_ = ProcMenuStep::ProcedureList;
        procSelectedName_.clear();
        procSelected_ = 0;
      } else {
        procMenuOpen_ = false;
      }
      break;
    case BezelKey::FmsInnerCw:
      if (procStep_ == ProcMenuStep::ProcedureList && !names.empty()) {
        procSelected_ = (procSelected_ + 1) % static_cast<int>(names.size());
      } else if (procStep_ == ProcMenuStep::TransitionList &&
                 !transitions.empty()) {
        procSelected_ =
            (procSelected_ + 1) % static_cast<int>(transitions.size());
      }
      break;
    case BezelKey::FmsInnerCcw:
      if (procStep_ == ProcMenuStep::ProcedureList && !names.empty()) {
        procSelected_ = (procSelected_ - 1 + static_cast<int>(names.size())) %
                        static_cast<int>(names.size());
      } else if (procStep_ == ProcMenuStep::TransitionList &&
                 !transitions.empty()) {
        procSelected_ = (procSelected_ - 1 +
                         static_cast<int>(transitions.size())) %
                        static_cast<int>(transitions.size());
      }
      break;
    case BezelKey::FmsOuterCw:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        procCategory_ = static_cast<ProcedureType>(
            (static_cast<int>(procCategory_) + 1) % 3);
        procSelected_ = 0;
        procSelectedName_.clear();
      }
      break;
    case BezelKey::FmsOuterCcw:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        procCategory_ = static_cast<ProcedureType>(
            (static_cast<int>(procCategory_) + 2) % 3);
        procSelected_ = 0;
        procSelectedName_.clear();
      }
      break;
    default:
      break;
  }
  return true;
}

}  // namespace avionics
