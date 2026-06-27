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

bool procedureMenuIsArrDep(const ProcedureMenuHost& host) {
  return host.state.category == ProcedureType::Arrival ||
         host.state.category == ProcedureType::Departure;
}

void procedureMenuApplyApproachDefaults(ProcedureMenuHost& host) {
  host.state.selectedName.clear();
  host.state.selectedTransition.clear();
  host.state.selectedRunway.clear();
  const std::string icao = procedureMenuAirportIcao(host);
  if (icao.empty()) return;

  const ProcedureType type = host.state.category;
  const std::vector<std::string> names = procedureMenuProcedureNames(host, type);
  if (names.empty()) return;
  host.state.selectedName = names.front();

  if (procedureMenuIsArrDep(host)) {
    const std::vector<std::string> enroute = procedureEnrouteTransitions(
        host.nav, icao, type, host.state.selectedName);
    host.state.selectedTransition =
        enroute.empty() ? std::string() : defaultProcedureTransition(enroute);
    const std::vector<std::string> runways =
        procedureRunwayOptions(host.nav, icao, type, host.state.selectedName);
    host.state.selectedRunway = runways.empty() ? std::string() : runways.front();
    return;
  }

  const std::vector<std::string> transitions =
      procedureMenuTransitions(host, type, host.state.selectedName);
  host.state.selectedTransition = defaultProcedureTransition(transitions);
}

void procedureMenuCloseSubList(ProcedureMenuHost& host) {
  host.state.subListOpen = false;
  if (host.state.step == ProcStep::TransitionList ||
      host.state.step == ProcStep::AirportList ||
      host.state.step == ProcStep::RunwayList) {
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
  return host.state.mode == ProcMode::Select && host.state.subListOpen &&
         host.state.step == ProcStep::ProcedureList;
}

bool procedureMenuTransitionListLive(const ProcedureMenuHost& host) {
  return host.state.mode == ProcMode::Select && host.state.subListOpen &&
         host.state.step == ProcStep::TransitionList;
}

bool procedureMenuRunwayListLive(const ProcedureMenuHost& host) {
  return host.state.mode == ProcMode::Select && host.state.subListOpen &&
         host.state.step == ProcStep::RunwayList;
}

// Enroute transitions for an Arrival/Departure (named entry/exit fixes), or the
// full transition list for an Approach.
std::vector<std::string> procedureMenuEnrouteTransitions(
    const ProcedureMenuHost& host, const std::string& name) {
  if (procedureMenuIsArrDep(host)) {
    return procedureEnrouteTransitions(host.nav, procedureMenuAirportIcao(host),
                                       host.state.category, name);
  }
  return procedureMenuTransitions(host, host.state.category, name);
}

std::vector<std::string> procedureMenuRunwayOptions(const ProcedureMenuHost& host,
                                                    const std::string& name) {
  return procedureRunwayOptions(host.nav, procedureMenuAirportIcao(host),
                                host.state.category, name);
}

std::string procedureMenuActiveApproachName(const ProcedureMenuHost& host) {
  if (host.state.mode != ProcMode::Select) {
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
        procedureMenuEnrouteTransitions(host, approachName);
    if (host.state.selected < 0 ||
        host.state.selected >= static_cast<int>(transitions.size())) {
      return {};
    }
    return transitions[static_cast<std::size_t>(host.state.selected)];
  }
  if (!host.state.selectedTransition.empty()) return host.state.selectedTransition;
  if (procedureMenuIsArrDep(host)) {
    const std::vector<std::string> enroute =
        procedureMenuEnrouteTransitions(host, approachName);
    return enroute.empty() ? std::string() : defaultProcedureTransition(enroute);
  }
  return defaultProcedureTransition(
      procedureMenuTransitions(host, host.state.category, approachName));
}

// Runway transition label currently shown/active for an Arrival/Departure.
std::string procedureMenuActiveRunway(const ProcedureMenuHost& host,
                                      const std::string& name) {
  if (name.empty() || !procedureMenuIsArrDep(host)) return {};
  if (procedureMenuRunwayListLive(host)) {
    const std::vector<std::string> runways =
        procedureMenuRunwayOptions(host, name);
    if (host.state.selected < 0 ||
        host.state.selected >= static_cast<int>(runways.size())) {
      return {};
    }
    return runways[static_cast<std::size_t>(host.state.selected)];
  }
  if (!host.state.selectedRunway.empty()) return host.state.selectedRunway;
  const std::vector<std::string> runways = procedureMenuRunwayOptions(host, name);
  return runways.empty() ? std::string() : runways.front();
}

// Raise the "Fly Course Reversal at <fix>?" prompt for the currently selected
// approach + transition before the approach is committed to the flight plan.
// The NXi shows this prompt on the approach-loading window the moment a HILPT
// transition is chosen; the YES/NO answer is deferred and applied at
// Load/Activate. courseReversalLegIndex stays -1 to mark the not-yet-loaded
// (deferred) state. Any prior prompt/decision is reset so re-selecting a
// different transition starts clean.
void procedureMenuRaiseCourseReversalPrompt(ProcedureMenuHost& host) {
  host.state.courseReversalPromptActive = false;
  host.state.courseReversalFix.clear();
  host.state.courseReversalLegIndex = -1;
  host.state.courseReversalYes = false;
  host.state.courseReversalYesDirty = false;
  host.state.courseReversalAnswered = false;
  host.state.courseReversalAnswerFlyIt = false;
  host.state.courseReversalDecisionMade = false;
  host.state.courseReversalDecisionFlyIt = false;
  host.state.courseReversalDecisionFix.clear();
  if (host.state.category != ProcedureType::Approach) return;
  if (host.nav == nullptr || !host.nav->ready()) return;
  const std::string& transition = host.state.selectedTransition;
  if (transition.empty() || host.state.selectedName.empty()) return;
  const std::string icao = procedureMenuAirportIcao(host);
  if (icao.empty()) return;
  const std::vector<MapLeg> legs = host.nav->expandProcedure(
      icao, host.state.category, host.state.selectedName, transition);
  for (const MapLeg& leg : legs) {
    if (leg.id == transition && leg.hold.active && leg.hold.courseReversal) {
      host.state.courseReversalPromptActive = true;
      host.state.courseReversalFix = transition;
      host.state.courseReversalLegIndex = -1;
      break;
    }
  }
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
    procedureMenuRaiseCourseReversalPrompt(host);
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

// Begin inline ICAO entry on the Airport field, seeded with the current airport
// (whole ident highlighted, like the Direct-To default) so the first knob turn
// starts a fresh identifier (Pilot's Guide "Select Approach", airport entry).
void procedureMenuOpenAirportEntry(ProcedureMenuHost& host) {
  host.state.airportEntry.allowAirways = false;
  host.state.airportEntry.open(host.nav, host.map,
                               procedureMenuAirportIcao(host));
}

// Apply the typed airport identifier: switch the listed approaches to the new
// airport and refresh the default approach/transition.
void procedureMenuCommitAirportEntry(ProcedureMenuHost& host) {
  FmsWaypointEntry& entry = host.state.airportEntry;
  const std::string icao = entry.hasMatch && !entry.match.id.empty()
                               ? entry.match.id
                               : entry.ident();
  entry.reset();
  if (icao.empty()) return;
  host.state.selectedAirportIcao = icao;
  procedureMenuApplyApproachDefaults(host);
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

// Arrival pages walk Airport -> Arrival -> Transition -> Runway -> (Sequence) ->
// Load?; Departure pages swap Transition/Runway to Airport -> Departure ->
// Runway -> Transition -> (Sequence) -> Load?. The Sequence box is entered
// between the last picker field and Load (handled by the cursor mover).
std::vector<ProcApproachField> procedureMenuArrDepFieldOrder(
    const ProcedureMenuHost& host) {
  using Field = ProcApproachField;
  if (host.state.category == ProcedureType::Departure) {
    return {Field::Airport, Field::Apr, Field::Runway, Field::Trans,
            Field::Load};
  }
  return {Field::Airport, Field::Apr, Field::Trans, Field::Runway, Field::Load};
}

int procedureMenuArrDepFieldIndex(const std::vector<ProcApproachField>& order,
                                  ProcApproachField field) {
  for (int i = 0; i < static_cast<int>(order.size()); ++i) {
    if (order[static_cast<std::size_t>(i)] == field) return i;
  }
  return 0;
}

void procedureMenuMoveArrDepCursor(ProcedureMenuHost& host, int dir) {
  const std::vector<ProcApproachField> order =
      procedureMenuArrDepFieldOrder(host);
  const int legCount =
      static_cast<int>(procedureMenuPreviewLegs(host).size());
  // The last picker field (just before Load) is the gateway into the Sequence.
  const int lastPicker = static_cast<int>(order.size()) - 2;
  if (host.state.sequenceFocused) {
    const int next = host.state.sequenceSelected + dir;
    if (next < 0) {
      host.state.sequenceFocused = false;
      host.state.sequenceSelected = 0;
      host.state.approachField =
          order[static_cast<std::size_t>(std::max(0, lastPicker))];
      host.state.loadArmed = false;
    } else if (next < legCount) {
      host.state.sequenceSelected = next;
    } else if (dir > 0) {
      host.state.sequenceFocused = false;
      host.state.sequenceSelected = 0;
      host.state.approachField = ProcApproachField::Load;
      host.state.loadArmed = true;
    }
    return;
  }
  if (dir > 0 && legCount > 0 && lastPicker >= 0 &&
      host.state.approachField ==
          order[static_cast<std::size_t>(lastPicker)]) {
    host.state.sequenceFocused = true;
    host.state.sequenceSelected = 0;
    host.state.loadArmed = false;
    return;
  }
  if (dir < 0 && legCount > 0 &&
      host.state.approachField == ProcApproachField::Load) {
    host.state.sequenceFocused = true;
    host.state.sequenceSelected = legCount - 1;
    host.state.loadArmed = false;
    return;
  }
  const int idx =
      procedureMenuArrDepFieldIndex(order, host.state.approachField);
  const int next =
      std::max(0, std::min(idx + dir, static_cast<int>(order.size()) - 1));
  host.state.approachField = order[static_cast<std::size_t>(next)];
  host.state.loadArmed = host.state.approachField == ProcApproachField::Load;
  host.state.activateArmed = false;
}

void procedureMenuOpenEnrouteList(ProcedureMenuHost& host) {
  const std::vector<std::string> trans =
      procedureMenuEnrouteTransitions(host, host.state.selectedName);
  if (trans.empty()) return;
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

void procedureMenuOpenRunwayList(ProcedureMenuHost& host) {
  const std::vector<std::string> runways =
      procedureMenuRunwayOptions(host, host.state.selectedName);
  if (runways.empty()) return;
  host.state.step = ProcStep::RunwayList;
  host.state.subListOpen = true;
  host.state.selected = 0;
  for (int i = 0; i < static_cast<int>(runways.size()); ++i) {
    if (runways[static_cast<std::size_t>(i)] == host.state.selectedRunway) {
      host.state.selected = i;
      break;
    }
  }
}

// Choose an Arrival/Departure procedure from the open list, refresh the default
// enroute/runway transitions for it, and return the cursor to the procedure
// field (the pilot then steps down to Transition/Runway/Load).
void procedureMenuPickArrDepProcedure(ProcedureMenuHost& host,
                                      const std::string& name) {
  host.state.selectedName = name;
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  const std::vector<std::string> enroute =
      procedureMenuEnrouteTransitions(host, name);
  host.state.selectedTransition =
      enroute.empty() ? std::string() : defaultProcedureTransition(enroute);
  const std::vector<std::string> runways =
      procedureMenuRunwayOptions(host, name);
  host.state.selectedRunway = runways.empty() ? std::string() : runways.front();
  procedureMenuCloseSubList(host);
  host.state.approachField = ProcApproachField::Apr;
  host.state.loadArmed = false;
  host.state.activateArmed = false;
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
      procedureMenuIsArrDep(host)
          ? expandArrivalDepartureProcedure(host.nav, icao, host.state.category,
                                            name, transition,
                                            host.state.selectedRunway)
          : host.nav->expandProcedure(icao, host.state.category, name,
                                      transition);
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

  // When the approach is loaded via an IAF that has a HILPT course reversal,
  // resolve whether to fly it. If the pilot already answered a transition-select
  // prompt for this transition, apply that latched decision now; otherwise raise
  // the prompt here (covers single-transition / list-flow loads that never went
  // through the loading window). Only the selected transition's fix is used.
  const bool hadDecision = host.state.courseReversalDecisionMade &&
                           host.state.courseReversalDecisionFix == transition;
  const bool decisionFlyIt = host.state.courseReversalDecisionFlyIt;
  host.state.courseReversalPromptActive = false;
  host.state.courseReversalFix.clear();
  host.state.courseReversalLegIndex = -1;
  host.state.courseReversalYes = false;
  host.state.courseReversalDecisionMade = false;
  host.state.courseReversalDecisionFlyIt = false;
  host.state.courseReversalDecisionFix.clear();
  if (host.state.category == ProcedureType::Approach && !transition.empty()) {
    const int end = host.approachLegStart + host.approachLegCount;
    for (int i = host.approachLegStart;
         i >= 0 && i < end && i < static_cast<int>(host.fplLegs.size()); ++i) {
      MapLeg& leg = host.fplLegs[static_cast<std::size_t>(i)];
      if (leg.id == transition && leg.hold.active && leg.hold.courseReversal) {
        if (hadDecision) {
          if (!decisionFlyIt) {
            leg.hold = MapHoldPattern{};
            host.publishFlightPlanEdit();
          }
        } else {
          host.state.courseReversalPromptActive = true;
          host.state.courseReversalFix = leg.id;
          host.state.courseReversalLegIndex = i;
        }
        break;
      }
    }
  }

  host.closeProceduresMenu();
  host.state.mode = ProcMode::Menu;
  host.state.step = ProcStep::ProcedureList;
  host.state.selectedName.clear();
  host.state.selectedTransition.clear();
  host.state.selectedRunway.clear();
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

void procedureMenuAnswerCourseReversal(ProcedureMenuHost& host, bool flyIt) {
  if (!host.state.courseReversalPromptActive) return;
  host.state.courseReversalAnswerFlyIt = flyIt;
  host.state.courseReversalAnswered = true;
  host.state.courseReversalPromptActive = false;
  host.state.courseReversalYesDirty = false;

  // Pre-load prompt (raised at transition-select): there is nothing in the
  // flight plan to edit yet, so latch the decision and apply it when the
  // approach is loaded/activated (procedureMenuLoadSelected).
  if (host.state.courseReversalLegIndex < 0) {
    host.state.courseReversalDecisionMade = true;
    host.state.courseReversalDecisionFlyIt = flyIt;
    host.state.courseReversalDecisionFix = host.state.courseReversalFix;
    return;
  }

  // courseReversalFix / courseReversalLegIndex stay set until the engine clears
  // them after applying the choice on every GDU.
  if (flyIt) return;

  // Immediate local update so the answering display updates before peer sync.
  const int idx = host.state.courseReversalLegIndex;
  if (idx >= 0 && idx < static_cast<int>(host.fplLegs.size())) {
    MapLeg& leg = host.fplLegs[static_cast<std::size_t>(idx)];
    if (leg.hold.courseReversal) {
      leg.hold = MapHoldPattern{};
      host.publishFlightPlanEdit();
    }
  }
}

void procedureMenuOpenApproachSelect(ProcedureMenuHost& host) {
  host.state.mode = ProcMode::Select;
  host.state.step = ProcStep::ProcedureList;
  host.state.selected = 0;
  host.state.selectedAirportIcao.clear();
  host.state.subListOpen = false;
  host.state.approachField = ProcApproachField::Airport;
  host.state.airportEntry.reset();
  host.state.sequenceFocused = false;
  host.state.sequenceSelected = 0;
  host.state.loadArmed = false;
  host.state.activateArmed = false;
  procedureMenuApplyApproachDefaults(host);
}

void procedureMenuOpenArrDepSelect(ProcedureMenuHost& host,
                                   ProcedureType type) {
  host.state.category = type;
  host.state.mode = ProcMode::Select;
  host.state.step = ProcStep::ProcedureList;
  host.state.selected = 0;
  host.state.selectedAirportIcao.clear();
  host.state.subListOpen = false;
  host.state.approachField = ProcApproachField::Airport;
  host.state.airportEntry.reset();
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
  host.state.airportEntry.reset();
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
  host.state.airportEntry.reset();
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
  if (host.state.step == ProcStep::RunwayList) {
    return procedureRunwayOptions(host.nav, procedureMenuAirportIcao(host),
                                  host.state.category, host.state.selectedName);
  }
  if (host.state.step == ProcStep::TransitionList) {
    if (procedureMenuIsArrDep(host)) {
      return procedureEnrouteTransitions(host.nav,
                                         procedureMenuAirportIcao(host),
                                         host.state.category,
                                         host.state.selectedName);
    }
    return procedureMenuTransitionLabels(host, host.state.category,
                                         host.state.selectedName);
  }
  return procedureMenuProcedureNames(host, host.state.category);
}

std::string procedureMenuApproachDisplayName(const ProcedureMenuHost& host,
                                             int index) {
  const std::vector<std::string> names =
      procedureMenuProcedureNames(host, host.state.category);
  if (index < 0 || index >= static_cast<int>(names.size())) return {};
  const std::string& name = names[static_cast<std::size_t>(index)];
  // Arrival/Departure procedures display their published name verbatim
  // (e.g. "JOSFF5"); only approaches get the RNAV/ILS label formatting.
  if (host.state.category != ProcedureType::Approach) return name;
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

std::string procedureMenuAirportCityLineFor(const MapFeature& f) {
  if (f.city.empty() && f.region.empty()) return {};
  if (f.city.empty()) return f.region;
  if (f.region.empty()) return f.city;
  return f.city + " " + f.region;
}

std::string procedureMenuAirportCityLine(const ProcedureMenuHost& host) {
  return procedureMenuAirportCityLineFor(procedureMenuAirportFeature(host));
}

std::string procedureMenuAirportNameLine(const ProcedureMenuHost& host) {
  return procedureMenuAirportFeature(host).name;
}

std::string procedureMenuSelectedApproachDisplay(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  if (host.state.category != ProcedureType::Approach) return name;
  const MapProcedure proc = procedureMenuSelectedProcedure(host);
  if (!proc.name.empty()) return formatApproachProcedureLabel(proc);
  return name;
}

std::string procedureMenuSelectedTransitionDisplay(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  return procedureMenuActiveTransition(host, name);
}

std::string procedureMenuSelectedRunwayDisplay(const ProcedureMenuHost& host) {
  const std::string name = procedureMenuActiveApproachName(host);
  if (name.empty()) return {};
  return procedureMenuActiveRunway(host, name);
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
  if (procedureMenuIsArrDep(host)) {
    return expandArrivalDepartureProcedure(host.nav, icao, host.state.category,
                                           name, transition,
                                           procedureMenuActiveRunway(host, name));
  }
  if (transition.empty()) return {};
  std::vector<MapLeg> legs =
      host.nav->expandProcedure(icao, host.state.category, name, transition);
  // Honor a deferred "NO" course-reversal decision so the Sequence preview drops
  // the HOLD line immediately, matching the NXi loading window.
  if (host.state.courseReversalDecisionMade &&
      !host.state.courseReversalDecisionFlyIt &&
      host.state.courseReversalDecisionFix == transition) {
    for (MapLeg& leg : legs) {
      if (leg.id == transition && leg.hold.courseReversal) {
        leg.hold = MapHoldPattern{};
      }
    }
  }
  return legs;
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
            procedureMenuOpenArrDepSelect(host, ProcedureType::Departure);
            return true;
          case ProcMenuAction::SelectArrival:
            procedureMenuOpenArrDepSelect(host, ProcedureType::Arrival);
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

  const bool formDetail = host.state.mode == ProcMode::Select;
  const bool approachDetail =
      formDetail && host.state.category == ProcedureType::Approach;
  const bool arrDepDetail = formDetail && procedureMenuIsArrDep(host);
  const std::vector<std::string> items = procedureMenuListItems(host);

  // Inline Airport-field ICAO entry: small knob spells the identifier, large
  // knob moves the cursor, ENT commits, CLR / FMS-push cancels.
  if (formDetail && host.state.airportEntry.active) {
    switch (key) {
      case BezelKey::FmsInnerCw:
        host.state.airportEntry.turnChar(host.nav, host.map, +1);
        return true;
      case BezelKey::FmsInnerCcw:
        host.state.airportEntry.turnChar(host.nav, host.map, -1);
        return true;
      case BezelKey::FmsOuterCw:
        host.state.airportEntry.moveCursor(host.nav, host.map, +1);
        return true;
      case BezelKey::FmsOuterCcw:
        host.state.airportEntry.moveCursor(host.nav, host.map, -1);
        return true;
      case BezelKey::Ent:
        procedureMenuCommitAirportEntry(host);
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        host.state.airportEntry.reset();
        return true;
      default:
        return false;
    }
  }

  if (formDetail && host.state.subListOpen) {
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
              const std::string& picked =
                  names[static_cast<std::size_t>(host.state.selected)];
              if (arrDepDetail) {
                procedureMenuPickArrDepProcedure(host, picked);
              } else {
                procedureMenuPickApproach(host, picked);
              }
            }
          } else if (host.state.step == ProcStep::RunwayList) {
            const std::vector<std::string> runways =
                procedureMenuRunwayOptions(host, host.state.selectedName);
            if (host.state.selected < static_cast<int>(runways.size())) {
              host.state.selectedRunway =
                  runways[static_cast<std::size_t>(host.state.selected)];
            }
            procedureMenuCloseSubList(host);
            host.state.approachField = ProcApproachField::Runway;
          } else if (arrDepDetail) {
            const std::vector<std::string> ids =
                procedureMenuEnrouteTransitions(host, host.state.selectedName);
            if (host.state.selected < static_cast<int>(ids.size())) {
              host.state.selectedTransition =
                  ids[static_cast<std::size_t>(host.state.selected)];
            }
            procedureMenuCloseSubList(host);
            host.state.approachField = ProcApproachField::Trans;
          } else {
            const std::vector<std::string> ids = procedureMenuTransitions(
                host, host.state.category, host.state.selectedName);
            if (host.state.selected < static_cast<int>(ids.size())) {
              host.state.selectedTransition =
                  ids[static_cast<std::size_t>(host.state.selected)];
            }
            procedureMenuCloseSubList(host);
            procedureMenuFocusLoad(host);
            procedureMenuRaiseCourseReversalPrompt(host);
          }
        }
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        if (!arrDepDetail && host.state.step == ProcStep::TransitionList) {
          procedureMenuBackToApproachList(host);
        } else {
          procedureMenuCloseSubList(host);
          if (!arrDepDetail && host.state.step == ProcStep::ProcedureList) {
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
        host.state.airportEntry.reset();
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
          procedureMenuOpenAirportEntry(host);
          host.state.airportEntry.turnChar(host.nav, host.map, +1);
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
          procedureMenuOpenAirportEntry(host);
          host.state.airportEntry.turnChar(host.nav, host.map, -1);
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

  // Arrival / Departure loading form: Airport, procedure, enroute Transition and
  // Runway pickers, then the Sequence list and the Load? button. The on-screen
  // field order differs by category (handled by the cursor walk); the picker
  // dropdowns reuse the approach sub-list popup.
  if (arrDepDetail) {
    switch (key) {
      case BezelKey::Ent:
        if (host.state.sequenceFocused) return true;
        if (host.state.approachField == ProcApproachField::Load) {
          if (host.state.loadArmed && !host.state.selectedName.empty()) {
            const std::string transition =
                host.state.selectedTransition.empty()
                    ? defaultProcedureTransition(procedureMenuEnrouteTransitions(
                          host, host.state.selectedName))
                    : host.state.selectedTransition;
            procedureMenuLoadSelected(host, host.state.selectedName, transition);
          } else if (!host.state.selectedName.empty()) {
            procedureMenuFocusLoad(host);
          }
          return true;
        }
        if (host.state.approachField == ProcApproachField::Apr) {
          procedureMenuOpenApproachList(host);
        } else if (host.state.approachField == ProcApproachField::Trans) {
          procedureMenuOpenEnrouteList(host);
        } else if (host.state.approachField == ProcApproachField::Runway) {
          procedureMenuOpenRunwayList(host);
        } else if (host.state.approachField == ProcApproachField::Airport) {
          procedureMenuOpenAirportEntry(host);
        }
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        host.state.mode = ProcMode::Menu;
        host.state.subListOpen = false;
        host.state.airportEntry.reset();
        host.state.sequenceFocused = false;
        host.state.sequenceSelected = 0;
        host.state.loadArmed = false;
        host.state.activateArmed = false;
        return true;
      case BezelKey::FmsOuterCw:
        procedureMenuMoveArrDepCursor(host, +1);
        return true;
      case BezelKey::FmsOuterCcw:
        procedureMenuMoveArrDepCursor(host, -1);
        return true;
      case BezelKey::FmsInnerCw:
        if (host.state.sequenceFocused) {
          procedureMenuMoveArrDepCursor(host, +1);
        } else if (host.state.approachField == ProcApproachField::Airport) {
          procedureMenuOpenAirportEntry(host);
          host.state.airportEntry.turnChar(host.nav, host.map, +1);
        } else if (host.state.approachField == ProcApproachField::Apr) {
          procedureMenuOpenApproachList(host);
        } else if (host.state.approachField == ProcApproachField::Trans) {
          procedureMenuOpenEnrouteList(host);
        } else if (host.state.approachField == ProcApproachField::Runway) {
          procedureMenuOpenRunwayList(host);
        } else if (host.state.approachField == ProcApproachField::Load &&
                   !host.state.selectedName.empty()) {
          procedureMenuMoveArrDepCursor(host, +1);
        }
        return true;
      case BezelKey::FmsInnerCcw:
        if (host.state.sequenceFocused) {
          procedureMenuMoveArrDepCursor(host, -1);
        } else if (host.state.approachField == ProcApproachField::Airport) {
          procedureMenuOpenAirportEntry(host);
          host.state.airportEntry.turnChar(host.nav, host.map, -1);
        } else if (host.state.approachField == ProcApproachField::Apr) {
          procedureMenuOpenApproachList(host);
        } else if (host.state.approachField == ProcApproachField::Trans) {
          procedureMenuOpenEnrouteList(host);
        } else if (host.state.approachField == ProcApproachField::Runway) {
          procedureMenuOpenRunwayList(host);
        } else if (host.state.approachField == ProcApproachField::Load &&
                   !host.state.selectedName.empty()) {
          procedureMenuMoveArrDepCursor(host, -1);
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
