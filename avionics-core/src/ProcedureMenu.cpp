#include "avionics/ProcedureMenu.h"

#include <algorithm>

#include "avionics/ProcedureSupport.h"

namespace avionics {
namespace {

const std::string kEmptyString;

constexpr float kMinsStepFt = 100.0f;
constexpr float kMinsMaxFt = 16000.0f;

std::vector<MapProcedure> proceduresForAirport(const ProcedureMenuHost& host,
                                               const std::string& icao,
                                               ProcedureType type) {
  if (host.nav == nullptr || !host.nav->ready() || icao.empty()) return {};
  return host.nav->proceduresForAirport(icao, type);
}

void procedureMenuMoveMenu(ProcedureMenuHost& host, int dir) {
  const int n = static_cast<int>(host.state.menuItems.size());
  if (n == 0) return;
  for (int next = host.state.menuSel + dir; next >= 0 && next < n;
       next += dir) {
    if (host.state.menuItems[static_cast<std::size_t>(next)].enabled) {
      host.state.menuSel = next;
      break;
    }
  }
}

void procedureMenuApplyApproachDefaults(ProcedureMenuHost& host) {
  host.state.selectedName.clear();
  host.state.selectedTransition.clear();
  const std::string icao = procedureMenuAirportIcao(host);
  if (icao.empty()) return;

  const std::vector<std::string> names =
      procedureMenuProcedureNames(host, ProcedureType::Approach);
  if (names.empty()) return;
  host.state.selectedName = names.front();

  const std::vector<std::string> transitions = procedureMenuTransitions(
      host, ProcedureType::Approach, host.state.selectedName);
  host.state.selectedTransition = defaultProcedureTransition(transitions);
}

void procedureMenuCloseSubList(ProcedureMenuHost& host) {
  host.state.subListOpen = false;
  if (host.state.step == ProcStep::TransitionList ||
      host.state.step == ProcStep::AirportList) {
    host.state.step = ProcStep::ProcedureList;
  }
}

void procedureMenuFocusLoad(ProcedureMenuHost& host) {
  host.state.approachField = ProcApproachField::Load;
  host.state.sequenceFocused = false;
  host.state.loadArmed = true;
  host.state.activateArmed = false;
}

void procedureMenuFocusActivate(ProcedureMenuHost& host) {
  host.state.approachField = ProcApproachField::Activate;
  host.state.sequenceFocused = false;
  host.state.activateArmed = true;
  host.state.loadArmed = false;
}

bool procedureMenuApproachListLive(const ProcedureMenuHost& host) {
  return host.state.mode == ProcMode::Select &&
         host.state.category == ProcedureType::Approach &&
         host.state.subListOpen &&
         host.state.step == ProcStep::ProcedureList;
}

bool procedureMenuTransitionListLive(const ProcedureMenuHost& host) {
  return host.state.mode == ProcMode::Select &&
         host.state.category == ProcedureType::Approach &&
         host.state.subListOpen &&
         host.state.step == ProcStep::TransitionList;
}

std::string procedureMenuActiveApproachName(const ProcedureMenuHost& host) {
  if (host.state.mode != ProcMode::Select ||
      host.state.category != ProcedureType::Approach) {
    return host.state.selectedName;
  }
  if (procedureMenuApproachListLive(host)) {
    const std::vector<std::string> names =
        procedureMenuProcedureNames(host, host.state.category);
    if (host.state.selected < 0 ||
        host.state.selected >= static_cast<int>(names.size())) {
      return {};
    }
    return names[static_cast<std::size_t>(host.state.selected)];
  }
  if (!host.state.selectedName.empty()) return host.state.selectedName;
  if (host.state.step == ProcStep::ProcedureList) {
    const std::vector<std::string> names =
        procedureMenuProcedureNames(host, host.state.category);
    if (host.state.selected < 0 ||
        host.state.selected >= static_cast<int>(names.size())) {
      return {};
    }
    return names[static_cast<std::size_t>(host.state.selected)];
  }
  return {};
}

std::string procedureMenuActiveTransition(const ProcedureMenuHost& host,
                                          const std::string& approachName) {
  if (approachName.empty()) return {};
  if (procedureMenuTransitionListLive(host)) {
    const std::vector<std::string> transitions =
        procedureMenuTransitions(host, host.state.category, approachName);
    if (host.state.selected < 0 ||
        host.state.selected >= static_cast<int>(transitions.size())) {
      return {};
    }
    return transitions[static_cast<std::size_t>(host.state.selected)];
  }
  if (!host.state.selectedTransition.empty()) return host.state.selectedTransition;
  return defaultProcedureTransition(
      procedureMenuTransitions(host, host.state.category, approachName));
}

void procedureMenuPickApproach(ProcedureMenuHost& host, const std::string& name) {
  host.state.selectedName = name;
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  const std::vector<std::string> names =
      procedureMenuProcedureNames(host, host.state.category);
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    if (names[static_cast<std::size_t>(i)] == name) {
      host.state.selected = i;
      break;
    }
  }
  const std::vector<std::string> trans =
      procedureMenuTransitions(host, host.state.category, name);
  if (trans.size() <= 1) {
    host.state.selectedTransition = trans.empty() ? std::string() : trans.front();
    procedureMenuCloseSubList(host);
    procedureMenuFocusLoad(host);
    return;
  }
  host.state.step = ProcStep::TransitionList;
  host.state.selected = 0;
  const std::string preferred = defaultProcedureTransition(trans);
  for (int i = 0; i < static_cast<int>(trans.size()); ++i) {
    if (trans[static_cast<std::size_t>(i)] == preferred) {
      host.state.selected = i;
      break;
    }
  }
  host.state.selectedTransition =
      trans[static_cast<std::size_t>(host.state.selected)];
  host.state.subListOpen = true;
  host.state.approachField = ProcApproachField::Trans;
}

void procedureMenuBackToApproachList(ProcedureMenuHost& host) {
  host.state.step = ProcStep::ProcedureList;
  host.state.selectedTransition.clear();
  host.state.selected = 0;
  const std::vector<std::string> names =
      procedureMenuProcedureNames(host, host.state.category);
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    if (names[static_cast<std::size_t>(i)] == host.state.selectedName) {
      host.state.selected = i;
      break;
    }
  }
  host.state.subListOpen = true;
  host.state.approachField = ProcApproachField::Apr;
}

void procedureMenuCycleApproachField(ProcedureMenuHost& host, int dir) {
  if (host.state.approachField == ProcApproachField::MinsAlt) {
    host.state.approachField = ProcApproachField::Mins;
    host.state.loadArmed = false;
    host.state.activateArmed = false;
    return;
  }
  const int count = static_cast<int>(ProcApproachField::Count);
  const int field = static_cast<int>(host.state.approachField);
  // Clamp at the first/last field; the walk does not wrap. MinsAlt and Id are
  // skipped (reached via the small knob / display only).
  int next = field;
  for (int step = field + dir; step >= 0 && step < count; step += dir) {
    if (step == static_cast<int>(ProcApproachField::MinsAlt) ||
        step == static_cast<int>(ProcApproachField::Id)) {
      continue;
    }
    next = step;
    break;
  }
  host.state.approachField = static_cast<ProcApproachField>(next);
  host.state.loadArmed = host.state.approachField == ProcApproachField::Load;
  host.state.activateArmed =
      host.state.approachField == ProcApproachField::Activate;
}

// Large/small-knob field cursor walk that also descends into the Sequence box
// after the last form field. The on-screen order is Minimums, Primary
// Frequency, Sequence, then Load?/Activate?; the cursor enters the sequence
// list from Minimums (not from the footer buttons below it). Row selection
// clamps at the first/last leg; past the last row the cursor continues to
// Load, then Activate.
void procedureMenuMoveApproachCursor(ProcedureMenuHost& host, int dir) {
  const int legCount =
      static_cast<int>(procedureMenuPreviewLegs(host).size());
  if (host.state.sequenceFocused) {
    const int next = host.state.sequenceSelected + dir;
    if (next < 0) {
      host.state.sequenceFocused = false;
      host.state.sequenceSelected = 0;
      host.state.approachField = ProcApproachField::Mins;
      host.state.loadArmed = false;
      host.state.activateArmed = false;
    } else if (next < legCount) {
      host.state.sequenceSelected = next;
    } else if (dir > 0) {
      host.state.sequenceFocused = false;
      host.state.sequenceSelected = 0;
      host.state.approachField = ProcApproachField::Load;
      host.state.loadArmed = false;
      host.state.activateArmed = false;
    }
    return;
  }
  if (dir < 0 && legCount > 0 &&
      host.state.approachField == ProcApproachField::Load) {
    host.state.sequenceFocused = true;
    host.state.sequenceSelected = legCount - 1;
    host.state.loadArmed = false;
    host.state.activateArmed = false;
    return;
  }
  if (dir > 0 && legCount > 0 &&
      host.state.approachField == ProcApproachField::Mins) {
    host.state.sequenceFocused = true;
    host.state.sequenceSelected = 0;
    host.state.loadArmed = false;
    host.state.activateArmed = false;
    return;
  }
  procedureMenuCycleApproachField(host, dir);
}

void procedureMenuOpenAirportList(ProcedureMenuHost& host) {
  const std::vector<std::string> ids = host.nearestAirportIds();
  if (ids.empty()) return;
  host.state.step = ProcStep::AirportList;
  host.state.subListOpen = true;
  host.state.selected = 0;
  const std::string current = procedureMenuAirportIcao(host);
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    if (ids[static_cast<std::size_t>(i)] == current) {
      host.state.selected = i;
      break;
    }
  }
}

void procedureMenuOpenApproachList(ProcedureMenuHost& host) {
  host.state.step = ProcStep::ProcedureList;
  host.state.subListOpen = true;
  host.state.selected = 0;
  if (!host.state.selectedName.empty()) {
    const std::vector<std::string> names =
        procedureMenuProcedureNames(host, host.state.category);
    for (int i = 0; i < static_cast<int>(names.size()); ++i) {
      if (names[static_cast<std::size_t>(i)] == host.state.selectedName) {
        host.state.selected = i;
        break;
      }
    }
  }
}

void procedureMenuOpenTransitionList(ProcedureMenuHost& host) {
  const std::vector<std::string> trans = procedureMenuTransitions(
      host, host.state.category, host.state.selectedName);
  if (trans.size() <= 1) return;
  host.state.step = ProcStep::TransitionList;
  host.state.subListOpen = true;
  host.state.selected = 0;
  for (int i = 0; i < static_cast<int>(trans.size()); ++i) {
    if (trans[static_cast<std::size_t>(i)] == host.state.selectedTransition) {
      host.state.selected = i;
      break;
    }
  }
}

void procedureMenuCycleMins(ProcedureMenuHost& host) {
  host.setMinimumsBaro(!host.minimumsBaroEnabled());
  if (!host.minimumsBaroEnabled() &&
      host.state.approachField == ProcApproachField::MinsAlt) {
    host.state.approachField = ProcApproachField::Mins;
  }
}

void procedureMenuAdjustMinsAlt(ProcedureMenuHost& host, int step) {
  const float next = std::max(
      0.0f, std::min(kMinsMaxFt, host.minimumsAltitudeFt() + step * kMinsStepFt));
  host.setMinimumsAltitudeFt(next);
}

void procedureMenuLoadSelected(ProcedureMenuHost& host, const std::string& name,
                                 const std::string& transition) {
  if (host.nav == nullptr || !host.nav->ready()) return;
  const std::string icao = procedureMenuAirportIcao(host);
  std::vector<MapLeg> legs =
      host.nav->expandProcedure(icao, host.state.category, name, transition);
  if (legs.empty()) return;
  if (host.state.category == ProcedureType::Approach) {
    removeLoadedApproachLegs(host.fplLegs, host.approachLegStart,
                             host.approachLegCount);
    host.approachLegCount = static_cast<int>(legs.size());
  } else {
    host.approachLegStart = 0;
    host.approachLegCount = 0;
    host.loadedApproach = {};
    if (host.approachHeaderLabel != nullptr) host.approachHeaderLabel->clear();
  }
  insertProcedureLegs(host.state.category, host.fplLegs, legs);
  if (host.state.category == ProcedureType::Approach) {
    host.approachLegStart =
        static_cast<int>(host.fplLegs.size()) - host.approachLegCount;
    host.cursorRow = 0;
    host.loadedApproach =
        findProcedureInCatalog(host.state.category, name, transition,
                               proceduresForAirport(host, icao, host.state.category));
    host.persistedRestore =
        persistedFromMapProcedure(host.loadedApproach, icao);
    if (host.approachHeaderLabel != nullptr) {
      *host.approachHeaderLabel =
          formatApproachFplHeaderLabel(host.loadedApproach);
    }
  }
  host.publishFlightPlanEdit();
  host.state.loadTarget = host.loadedApproach;
  host.state.loadPending = true;
  host.closeProceduresMenu();
  host.state.mode = ProcMode::Menu;
  host.state.step = ProcStep::ProcedureList;
  host.state.selectedName.clear();
  host.state.selectedTransition.clear();
  host.state.subListOpen = false;
  host.state.loadArmed = false;
  host.state.activateArmed = false;
}

void procedureMenuActivateSelected(ProcedureMenuHost& host,
                                   const std::string& name,
                                   const std::string& transition) {
  procedureMenuLoadSelected(host, name, transition);
  if (host.state.category == ProcedureType::Approach && host.approachLegStart >= 0 &&
      host.approachLegStart < static_cast<int>(host.fplLegs.size())) {
    host.requestActivateLeg(host.approachLegStart);
  }
}

}  // namespace

void procedureMenuOpenApproachSelect(ProcedureMenuHost& host) {
  host.state.mode = ProcMode::Select;
  host.state.step = ProcStep::ProcedureList;
  host.state.selected = 0;
  host.state.selectedAirportIcao.clear();
  host.state.subListOpen = false;
  host.state.approachField = ProcApproachField::Airport;
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  host.state.loadArmed = false;
  host.state.activateArmed = false;
  procedureMenuApplyApproachDefaults(host);
}

void procedureMenuOpenApproachLoading(ProcedureMenuHost& host,
                                      const std::string& icao,
                                      const std::string& approachName,
                                      const std::string& transition,
                                      ProcLoadingList openList) {
  host.state.mode = ProcMode::Select;
  host.state.category = ProcedureType::Approach;
  host.state.step = ProcStep::ProcedureList;
  host.state.selected = 0;
  host.state.selectedAirportIcao = icao;
  host.state.selectedName = approachName;
  host.state.selectedTransition = transition;
  host.state.subListOpen = false;
  host.state.approachField = ProcApproachField::Activate;
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  host.state.loadArmed = false;
  host.state.activateArmed = true;
  if (openList == ProcLoadingList::Approach) {
    host.state.approachField = ProcApproachField::Apr;
    procedureMenuOpenApproachList(host);
  } else if (openList == ProcLoadingList::Transition) {
    host.state.approachField = ProcApproachField::Trans;
    procedureMenuOpenTransitionList(host);
  }
}

void procedureMenuBuild(ProcedureMenuHost& host) {
  const bool approachLoaded = host.approachLegCount > 0 &&
                              host.approachLegStart >= 0 &&
                              host.approachLegStart <
                                  static_cast<int>(host.fplLegs.size());
  const bool missedAvailable =
      approachLoaded && hasMissedApproachLegs(host.fplLegs);
  host.state.menuItems = {
      {"Activate Vector-to-Final", ProcMenuAction::ActivateVtf, false},
      {"Activate Approach", ProcMenuAction::ActivateApproach, approachLoaded},
      {"Activate Missed Approach", ProcMenuAction::ActivateMissed,
       missedAvailable},
      {"Select Approach", ProcMenuAction::SelectApproach, true},
      {"Select Arrival", ProcMenuAction::SelectArrival, true},
      {"Select Departure", ProcMenuAction::SelectDeparture, true},
  };
  host.state.mode = ProcMode::Menu;
  host.state.step = ProcStep::ProcedureList;
  host.state.selected = 0;
  host.state.selectedName.clear();
  host.state.selectedTransition.clear();
  host.state.selectedAirportIcao.clear();
  host.state.subListOpen = false;
  host.state.approachField = ProcApproachField::Airport;
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  host.state.loadArmed = false;
  host.state.activateArmed = false;
  host.state.menuSel = 0;
  for (int i = 0; i < static_cast<int>(host.state.menuItems.size()); ++i) {
    if (host.state.menuItems[static_cast<std::size_t>(i)].enabled) {
      host.state.menuSel = i;
      break;
    }
  }
}

const char* procedureMenuWindowTitle(const ProcedureMenuHost& host) {
  if (host.state.mode == ProcMode::Menu) return "Procedures";
  switch (host.state.category) {
    case ProcedureType::Departure:
      return "Select Departure";
    case ProcedureType::Arrival:
      return "Select Arrival";
    case ProcedureType::Approach:
    default:
      return "Select Approach";
  }
}

const std::string& procedureMenuItemText(const ProcedureMenuHost& host, int i) {
  if (i < 0 || i >= static_cast<int>(host.state.menuItems.size())) {
    return kEmptyString;
  }
  return host.state.menuItems[static_cast<std::size_t>(i)].text;
}

bool procedureMenuItemEnabled(const ProcedureMenuHost& host, int i) {
  if (i < 0 || i >= static_cast<int>(host.state.menuItems.size())) return false;
  return host.state.menuItems[static_cast<std::size_t>(i)].enabled;
}

std::string procedureMenuAirportIcao(const ProcedureMenuHost& host) {
  if (!host.state.selectedAirportIcao.empty()) {
    return host.state.selectedAirportIcao;
  }
  return host.defaultAirportIcao();
}

MapFeature procedureMenuAirportFeature(const ProcedureMenuHost& host) {
  if (host.nav == nullptr || !host.nav->ready()) return {};
  const std::string icao = procedureMenuAirportIcao(host);
  if (icao.empty()) return {};
  for (const MapFeature& f : host.nav->lookupIdent(icao, 8)) {
    if (f.type == MapFeatureType::Airport) return f;
  }
  return {};
}

std::vector<std::string> procedureMenuProcedureNames(const ProcedureMenuHost& host,
                                                       ProcedureType type) {
  return uniqueProcedureNames(
      proceduresForAirport(host, procedureMenuAirportIcao(host), type));
}

std::vector<std::string> procedureMenuTransitions(const ProcedureMenuHost& host,
                                                    ProcedureType type,
                                                    const std::string& name) {
  return procedureTransitionIds(host.nav, procedureMenuAirportIcao(host), type,
                                name);
}

std::vector<std::string> procedureMenuTransitionLabels(
    const ProcedureMenuHost& host, ProcedureType type, const std::string& name) {
  return procedureTransitionLabels(host.nav, procedureMenuAirportIcao(host), type,
                                   name);
}

std::vector<std::string> procedureMenuListItems(const ProcedureMenuHost& host) {
  if (host.state.step == ProcStep::AirportList) {
    return host.nearestAirportIds();
  }
  if (host.state.step == ProcStep::TransitionList) {
    return procedureMenuTransitionLabels(host, host.state.category,
                                         host.state.selectedName);
  }
  return procedureMenuProcedureNames(host, host.state.category);
}

std::string procedureMenuApproachDisplayName(const ProcedureMenuHost& host,
                                             int index) {
  const std::vector<std::string> names =
      procedureMenuProcedureNames(host, ProcedureType::Approach);
  if (index < 0 || index >= static_cast<int>(names.size())) return {};
  const std::string& name = names[static_cast<std::size_t>(index)];
  for (const MapProcedure& proc :
       proceduresForAirport(host, procedureMenuAirportIcao(host),
                            ProcedureType::Approach)) {
    if (proc.name == name) return formatApproachProcedureLabel(proc);
  }
  MapProcedure stub;
  stub.name = name;
  return formatApproachProcedureLabel(stub);
}

MapProcedure procedureMenuSelectedProcedure(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  if (host.nav == nullptr || !host.nav->ready()) return {};
  const std::string icao = procedureMenuAirportIcao(host);
  const std::string transition = procedureMenuActiveTransition(host, name);
  return findProcedureInCatalog(host.state.category, name, transition,
                                proceduresForAirport(host, icao, host.state.category));
}

std::string procedureMenuAirportCityLine(const ProcedureMenuHost& host) {
  const MapFeature f = procedureMenuAirportFeature(host);
  if (f.city.empty() && f.region.empty()) return {};
  if (f.city.empty()) return f.region;
  if (f.region.empty()) return f.city;
  return f.city + " " + f.region;
}

std::string procedureMenuAirportNameLine(const ProcedureMenuHost& host) {
  return procedureMenuAirportFeature(host).name;
}

std::string procedureMenuSelectedApproachDisplay(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  const MapProcedure proc = procedureMenuSelectedProcedure(host);
  if (!proc.name.empty()) return formatApproachProcedureLabel(proc);
  return name;
}

std::string procedureMenuSelectedTransitionDisplay(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  return procedureMenuActiveTransition(host, name);
}

float procedureMenuPrimaryFreqMhz(const ProcedureMenuHost& host) {
  const MapFeature airport = procedureMenuAirportFeature(host);
  const MapProcedure selected = procedureMenuSelectedProcedure(host);
  ProcPrimaryNav out =
      resolveProcPrimaryNav(host.nav, host.map, airport, selected);
  if (out.frequency > 0.0f) return out.frequency;

  const std::string runway = selected.runway;
  const std::string icao = procedureMenuAirportIcao(host);
  for (const MapProcedure& proc :
       proceduresForAirport(host, icao, ProcedureType::Approach)) {
    if (proc.frequencyMhz <= 0.0f) continue;
    if (proc.approachKind != "I" && proc.approachKind != "L") continue;
    if (!runway.empty() && !proc.runway.empty() && proc.runway != runway &&
        proc.runway.find(runway) == std::string::npos &&
        runway.find(proc.runway) == std::string::npos) {
      continue;
    }
    return proc.frequencyMhz;
  }
  return 0.0f;
}

bool procedureMenuPrimaryNavIsNdb(const ProcedureMenuHost& host) {
  return resolveProcPrimaryNav(host.nav, host.map,
                               procedureMenuAirportFeature(host),
                               procedureMenuSelectedProcedure(host))
      .isNdb;
}

bool procedureMenuShowsPrimaryNavFreq(const ProcedureMenuHost& host) {
  return procedureUsesNavPrimaryFrequency(procedureMenuSelectedProcedure(host));
}

std::string procedureMenuPrimaryIdent(const ProcedureMenuHost& host) {
  const MapFeature airport = procedureMenuAirportFeature(host);
  const MapProcedure selected = procedureMenuSelectedProcedure(host);
  ProcPrimaryNav out =
      resolveProcPrimaryNav(host.nav, host.map, airport, selected);
  if (!out.ident.empty()) return out.ident;

  const std::string runway = selected.runway;
  const std::string icao = procedureMenuAirportIcao(host);
  for (const MapProcedure& proc :
       proceduresForAirport(host, icao, ProcedureType::Approach)) {
    if (proc.frequencyMhz <= 0.0f) continue;
    if (proc.approachKind != "I" && proc.approachKind != "L") continue;
    if (!runway.empty() && !proc.runway.empty() && proc.runway != runway &&
        proc.runway.find(runway) == std::string::npos &&
        runway.find(proc.runway) == std::string::npos) {
      continue;
    }
    return proc.name;
  }
  return {};
}

std::vector<MapLeg> procedureMenuPreviewLegs(const ProcedureMenuHost& host) {
  if (host.state.mode != ProcMode::Select || host.nav == nullptr ||
      !host.nav->ready()) {
    return {};
  }
  const std::string icao = procedureMenuAirportIcao(host);
  if (icao.empty()) return {};

  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  const std::string transition = procedureMenuActiveTransition(host, name);
  if (transition.empty()) return {};
  return host.nav->expandProcedure(icao, host.state.category, name, transition);
}

bool procedureMenuBezelKey(ProcedureMenuHost& host, BezelKey key) {
  if (host.state.mode == ProcMode::Menu) {
    switch (key) {
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsOuterCw:
        procedureMenuMoveMenu(host, +1);
        return true;
      case BezelKey::FmsInnerCcw:
      case BezelKey::FmsOuterCcw:
        procedureMenuMoveMenu(host, -1);
        return true;
      case BezelKey::Ent: {
        if (host.state.menuSel < 0 ||
            host.state.menuSel >= static_cast<int>(host.state.menuItems.size())) {
          return true;
        }
        const ProcMenuItem& item =
            host.state.menuItems[static_cast<std::size_t>(host.state.menuSel)];
        if (!item.enabled) return true;
        switch (item.action) {
          case ProcMenuAction::SelectDeparture:
            host.state.category = ProcedureType::Departure;
            host.state.mode = ProcMode::Select;
            host.state.step = ProcStep::ProcedureList;
            host.state.selected = 0;
            host.state.selectedName.clear();
            host.state.selectedTransition.clear();
            host.state.subListOpen = false;
            return true;
          case ProcMenuAction::SelectArrival:
            host.state.category = ProcedureType::Arrival;
            host.state.mode = ProcMode::Select;
            host.state.step = ProcStep::ProcedureList;
            host.state.selected = 0;
            host.state.selectedName.clear();
            host.state.selectedTransition.clear();
            host.state.subListOpen = false;
            return true;
          case ProcMenuAction::SelectApproach:
            host.state.category = ProcedureType::Approach;
            procedureMenuOpenApproachSelect(host);
            return true;
          case ProcMenuAction::ActivateApproach:
            if (host.approachLegCount > 0) {
              host.requestActivateLeg(host.approachLegStart);
              host.closeProceduresMenu();
            }
            return true;
          case ProcMenuAction::ActivateMissed:
            host.requestActivateMissed();
            host.closeProceduresMenu();
            return true;
          default:
            return true;
        }
      }
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        host.closeProceduresMenu();
        return true;
      default:
        return false;
    }
  }

  const bool approachDetail =
      host.state.category == ProcedureType::Approach &&
      host.state.mode == ProcMode::Select;
  const std::vector<std::string> items = procedureMenuListItems(host);

  if (approachDetail && host.state.subListOpen) {
    switch (key) {
      case BezelKey::Ent:
        if (host.state.selected >= 0 &&
            host.state.selected < static_cast<int>(items.size())) {
          if (host.state.step == ProcStep::AirportList) {
            const std::vector<std::string> ids = host.nearestAirportIds();
            if (host.state.selected >= 0 &&
                host.state.selected < static_cast<int>(ids.size())) {
              host.state.selectedAirportIcao =
                  ids[static_cast<std::size_t>(host.state.selected)];
              procedureMenuApplyApproachDefaults(host);
            }
            procedureMenuCloseSubList(host);
          } else if (host.state.step == ProcStep::ProcedureList) {
            const std::vector<std::string> names =
                procedureMenuProcedureNames(host, host.state.category);
            if (host.state.selected < static_cast<int>(names.size())) {
              procedureMenuPickApproach(
                  host, names[static_cast<std::size_t>(host.state.selected)]);
            }
          } else {
            const std::vector<std::string> ids = procedureMenuTransitions(
                host, host.state.category, host.state.selectedName);
            if (host.state.selected < static_cast<int>(ids.size())) {
              host.state.selectedTransition =
                  ids[static_cast<std::size_t>(host.state.selected)];
            }
            procedureMenuCloseSubList(host);
            procedureMenuFocusLoad(host);
          }
        }
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        if (host.state.step == ProcStep::TransitionList) {
          procedureMenuBackToApproachList(host);
        } else {
          procedureMenuCloseSubList(host);
          if (host.state.step == ProcStep::ProcedureList) {
            host.state.selectedName.clear();
            host.state.selectedTransition.clear();
          }
        }
        return true;
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsOuterCw:
        if (!items.empty()) {
          host.state.selected = std::min(host.state.selected + 1,
                                         static_cast<int>(items.size()) - 1);
        }
        return true;
      case BezelKey::FmsInnerCcw:
      case BezelKey::FmsOuterCcw:
        if (!items.empty()) {
          host.state.selected = std::max(host.state.selected - 1, 0);
        }
        return true;
      default:
        return false;
    }
  }

  if (approachDetail) {
    switch (key) {
      case BezelKey::Ent:
        if (host.state.sequenceFocused) {
          return true;
        }
        if (host.state.approachField == ProcApproachField::Load) {
          if (host.state.loadArmed && !host.state.selectedName.empty()) {
            const std::string transition =
                host.state.selectedTransition.empty()
                    ? defaultProcedureTransition(procedureMenuTransitions(
                          host, host.state.category, host.state.selectedName))
                    : host.state.selectedTransition;
            procedureMenuLoadSelected(host, host.state.selectedName, transition);
          } else if (!host.state.selectedName.empty()) {
            procedureMenuFocusLoad(host);
          }
          return true;
        }
        if (host.state.approachField == ProcApproachField::Activate &&
            !host.state.selectedName.empty()) {
          if (host.state.activateArmed) {
            const std::string transition =
                host.state.selectedTransition.empty()
                    ? defaultProcedureTransition(procedureMenuTransitions(
                          host, host.state.category, host.state.selectedName))
                    : host.state.selectedTransition;
            procedureMenuActivateSelected(host, host.state.selectedName,
                                          transition);
          } else {
            procedureMenuFocusActivate(host);
          }
          return true;
        }
        procedureMenuFocusLoad(host);
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        host.state.mode = ProcMode::Menu;
        host.state.subListOpen = false;
        host.state.sequenceFocused = false;
        host.state.sequenceSelected = 0;
        host.state.loadArmed = false;
        host.state.activateArmed = false;
        return true;
      case BezelKey::FmsOuterCw:
        procedureMenuMoveApproachCursor(host, +1);
        return true;
      case BezelKey::FmsOuterCcw:
        procedureMenuMoveApproachCursor(host, -1);
        return true;
      case BezelKey::FmsInnerCw:
        if (host.state.sequenceFocused) {
          procedureMenuMoveApproachCursor(host, +1);
          return true;
        }
        if (host.state.approachField == ProcApproachField::MinsAlt) {
          procedureMenuAdjustMinsAlt(host, +1);
        } else if (host.state.approachField == ProcApproachField::Mins) {
          if (host.minimumsBaroEnabled()) {
            host.state.approachField = ProcApproachField::MinsAlt;
          } else {
            procedureMenuCycleMins(host);
          }
        } else if (host.state.approachField == ProcApproachField::Airport) {
          procedureMenuOpenAirportList(host);
        } else if (host.state.approachField == ProcApproachField::Apr) {
          procedureMenuOpenApproachList(host);
        } else if (host.state.approachField == ProcApproachField::Trans) {
          procedureMenuOpenTransitionList(host);
        } else if ((host.state.approachField == ProcApproachField::Activate ||
                    host.state.approachField == ProcApproachField::Load) &&
                   !host.state.selectedName.empty()) {
          procedureMenuMoveApproachCursor(host, +1);
        }
        return true;
      case BezelKey::FmsInnerCcw:
        if (host.state.sequenceFocused) {
          procedureMenuMoveApproachCursor(host, -1);
          return true;
        }
        if (host.state.approachField == ProcApproachField::MinsAlt) {
          procedureMenuAdjustMinsAlt(host, -1);
        } else if (host.state.approachField == ProcApproachField::Mins) {
          procedureMenuCycleMins(host);
        } else if (host.state.approachField == ProcApproachField::Airport) {
          procedureMenuOpenAirportList(host);
        } else if (host.state.approachField == ProcApproachField::Apr) {
          procedureMenuOpenApproachList(host);
        } else if (host.state.approachField == ProcApproachField::Trans) {
          procedureMenuOpenTransitionList(host);
        } else if ((host.state.approachField == ProcApproachField::Activate ||
                    host.state.approachField == ProcApproachField::Load) &&
                   !host.state.selectedName.empty()) {
          procedureMenuMoveApproachCursor(host, -1);
        }
        return true;
      default:
        return false;
    }
  }

  switch (key) {
    case BezelKey::Ent:
      if (host.state.step == ProcStep::ProcedureList) {
        if (host.state.selected >= 0 &&
            host.state.selected < static_cast<int>(items.size())) {
          const std::string& name =
              items[static_cast<std::size_t>(host.state.selected)];
          const std::vector<std::string> trans =
              procedureMenuTransitions(host, host.state.category, name);
          if (trans.size() == 1) {
            procedureMenuLoadSelected(host, name, trans.front());
          } else if (trans.size() > 1) {
            host.state.selectedName = name;
            host.state.step = ProcStep::TransitionList;
            host.state.selected = 0;
          }
        }
      } else if (host.state.selected >= 0 &&
                 host.state.selected < static_cast<int>(items.size())) {
        const std::vector<std::string> ids = procedureMenuTransitions(
            host, host.state.category, host.state.selectedName);
        if (host.state.selected < static_cast<int>(ids.size())) {
          procedureMenuLoadSelected(
              host, host.state.selectedName,
              ids[static_cast<std::size_t>(host.state.selected)]);
        }
      }
      return true;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      if (host.state.step == ProcStep::TransitionList) {
        host.state.step = ProcStep::ProcedureList;
        host.state.selectedName.clear();
        host.state.selected = 0;
      } else {
        host.state.mode = ProcMode::Menu;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsOuterCw:
      if (!items.empty()) {
        host.state.selected = std::min(host.state.selected + 1,
                                       static_cast<int>(items.size()) - 1);
      }
      return true;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsOuterCcw:
      if (!items.empty()) {
        host.state.selected = std::max(host.state.selected - 1, 0);
      }
      return true;
    default:
      return false;
  }
}

}  // namespace avionics
