#pragma once

#include <algorithm>
#include <string>

#include <cctype>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// Last loaded terminal procedure (approach), saved across standalone restarts.
// Waypoint legs come from the sim FMS on reload; this restores PROC/FPL metadata.
struct PersistedLoadedApproach {
  bool active = false;
  std::string airportIcao;
  ProcedureType type = ProcedureType::Approach;
  std::string name;
  std::string transition;
  std::string runway;
  std::string approachKind;
  std::string levelOfService;
};

inline bool operator==(const PersistedLoadedApproach& a,
                       const PersistedLoadedApproach& b) {
  return a.active == b.active && a.airportIcao == b.airportIcao &&
         a.type == b.type && a.name == b.name && a.transition == b.transition &&
         a.runway == b.runway && a.approachKind == b.approachKind &&
         a.levelOfService == b.levelOfService;
}

inline bool operator!=(const PersistedLoadedApproach& a,
                       const PersistedLoadedApproach& b) {
  return !(a == b);
}

// Finds `sequence` as a contiguous subsequence of `plan` (id match). Prefers the
// last match (approach legs are usually at the end of the route).
bool findLegSequenceInPlan(const std::vector<MapLeg>& plan,
                           const std::vector<MapLeg>& sequence, int& startOut);

struct InferredProcedureBlock {
  int start = -1;
  int count = 0;
  bool valid() const { return start >= 0 && count > 0; }
};

// First contiguous procedure-leg block in `legs` (requires at least one leg
// before it). `minStart` skips any leading legs that belong to an earlier
// procedure block (e.g. a loaded arrival/STAR whose fixes carry procedureRole
// tags) so the approach is only inferred from the tail after that block.
InferredProcedureBlock inferProcedureBlockInPlan(const std::vector<MapLeg>& legs,
                                                 int minStart = 0);

// Destination airport leg immediately before a loaded approach (FPL display
// anchor, not a flyover fix on the navigation map). Returns -1 when absent.
int destinationAirportLegBeforeApproach(const std::vector<MapLeg>& legs,
                                          int minStart = 0);

// Plan legs for map route display: omits the destination airport row when an
// approach follows it so enroute connects to the IAF instead of the airport.
std::vector<MapLeg> mapRouteDisplayLegs(const std::vector<MapLeg>& legs,
                                          int minStart = 0);

// Map a leg index in `legs` to the display-route index after omitting the
// destination-airport row; returns -1 when that leg is omitted.
int mapRouteDisplayLegIndex(const std::vector<MapLeg>& legs, int planLegIndex,
                              int minStart = 0);

// Inverse of mapRouteDisplayLegIndex.
int mapRoutePlanLegIndex(const std::vector<MapLeg>& legs, int routeLegIndex,
                           int minStart = 0);

// Lowest index at which a loaded approach can begin. The approach is the
// procedure tail AFTER the destination airport, so it can never start at or
// before the last airport ident that is itself followed by procedure legs (that
// airport is the destination). Returned floor is never below `arrivalEnd` (a
// separately tracked arrival/STAR block) nor `departureEnd` (a separately
// tracked SID/departure block). This keeps a STAR's role-tagged fixes from being
// mistaken for the approach when the arrival block is not tracked (e.g. a route
// adopted from the sim/external FMS), and keeps a loaded SID's fixes from being
// swallowed by the approach inference's untagged-feeder walk-back when no
// destination-airport waypoint separates the SID/enroute legs from the approach
// (the sim drops the airport waypoint once an approach is loaded). So the
// departure, enroute, and approach stay distinct parent sections on the flight
// plan page.
int fplApproachInferenceFloor(const std::vector<MapLeg>& legs,
                              int arrivalEnd = 0, int departureEnd = 0);

// Copies procedureRole (and any future per-leg PROC fields) from expanded legs
// into the matching plan rows.
void mergeProcedureLegMetadata(std::vector<MapLeg>& plan, int start,
                               const std::vector<MapLeg>& procedureLegs);

// Erases a currently-loaded approach block so loading a new approach replaces it
// instead of appending a second copy (which would duplicate the missed-approach
// legs). No-op when the block is empty or out of range.
void removeLoadedApproachLegs(std::vector<MapLeg>& legs, int approachStart,
                              int approachCount);

// Heals a plan whose loaded approach was accidentally appended twice: removes a
// trailing leg block that exactly repeats (by ident) the block before it when
// that block carries procedure roles (i.e. is an approach). Returns true if any
// duplicate copy was removed.
bool collapseDuplicateApproachTail(std::vector<MapLeg>& legs);

MapProcedure mapProcedureFromPersisted(const PersistedLoadedApproach& saved);

PersistedLoadedApproach persistedFromMapProcedure(const MapProcedure& proc,
                                                    const std::string& icao);

// PFD/MFD FPL approach parent row (e.g. "RNAV_GPS 23 LPV").
std::string formatApproachFplHeaderLabel(const MapProcedure& proc);

// Loaded approach grouping shared between the PFD Active Flight Plan window
// and the MFD FPL page (Origin/Enroute template + approach parent header).
struct FlightPlanApproachState {
  int legStart = 0;
  int legCount = 0;
  MapProcedure loaded;
  std::string headerLabel;
  bool active() const { return legCount > 0; }
};

// Loaded SID/STAR grouping for the FPL parent row (Departure / Arrival headers).
struct FlightPlanTerminalProcedureState {
  int legStart = 0;
  int legCount = 0;
  MapProcedure loaded;
  std::string headerLabel;
  bool active() const { return legCount > 0 || !loaded.name.empty(); }
};

inline bool approachStateFitsPlan(const FlightPlanApproachState& state,
                                  const std::vector<MapLeg>& legs) {
  return state.legCount > 0 && state.legStart >= 0 &&
         state.legStart + state.legCount <= static_cast<int>(legs.size());
}

inline bool terminalProcedureStateFitsPlan(
    const FlightPlanTerminalProcedureState& state,
    const std::vector<MapLeg>& legs) {
  return state.legCount > 0 && state.legStart >= 0 &&
         state.legStart + state.legCount <= static_cast<int>(legs.size());
}

inline bool operator==(const FlightPlanApproachState& a,
                       const FlightPlanApproachState& b) {
  return a.legStart == b.legStart && a.legCount == b.legCount &&
         a.headerLabel == b.headerLabel && a.loaded.type == b.loaded.type &&
         a.loaded.name == b.loaded.name &&
         a.loaded.transition == b.loaded.transition &&
         a.loaded.runway == b.loaded.runway &&
         a.loaded.approachKind == b.loaded.approachKind &&
         a.loaded.levelOfService == b.loaded.levelOfService;
}

inline bool operator!=(const FlightPlanApproachState& a,
                       const FlightPlanApproachState& b) {
  return !(a == b);
}

inline bool operator==(const FlightPlanTerminalProcedureState& a,
                       const FlightPlanTerminalProcedureState& b) {
  return a.legStart == b.legStart && a.legCount == b.legCount &&
         a.headerLabel == b.headerLabel && a.loaded.type == b.loaded.type &&
         a.loaded.name == b.loaded.name &&
         a.loaded.transition == b.loaded.transition &&
         a.loaded.runway == b.loaded.runway;
}

inline bool operator!=(const FlightPlanTerminalProcedureState& a,
                       const FlightPlanTerminalProcedureState& b) {
  return !(a == b);
}

// Index of the MAPt leg in a loaded approach (-1 when none). Scans from the end
// so a repeated fix id in the missed segment does not win over the runway MAPt.
inline int findMaptLegIndex(const std::vector<MapLeg>& legs) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (legs[static_cast<std::size_t>(i)].procedureRole == "mapt") {
      return i;
    }
  }
  return -1;
}

// True when the plan carries at least one fix after the MAPt (missed segment).
inline bool hasMissedApproachLegs(const std::vector<MapLeg>& legs) {
  const int maptIdx = findMaptLegIndex(legs);
  return maptIdx >= 0 && maptIdx + 1 < static_cast<int>(legs.size());
}

// X-Plane FMS lat/lon entries use a coordinate-style id (e.g. "+27-81") when
// the waypoint is not stored as a nav-database ref.
bool isFmsLatLonIdent(const std::string& id);

// When the sim echoes coordinate idents, keep user/database idents from the
// last published plan for legs at the same position.
void preserveFlightPlanIdents(std::vector<MapLeg>& plan,
                              const std::vector<MapLeg>& published);

// Pilot-built route stored only in the avionics app until it is complete enough
// to program the simulator FMS (destination committed, at least two legs).
bool flightPlanReadyForSimulator(const std::vector<MapLeg>& legs,
                                 bool destinationFilled,
                                 bool requireDestinationFilled = true);

// Loaded approach with the airport duplicated as origin: hide the Origin row.
inline bool fplApproachBlankOriginSection(const std::vector<MapLeg>& legs,
                                          int approachStart,
                                          const std::string& approachAirport) {
  return approachStart <= 1 && !legs.empty() && !approachAirport.empty() &&
         legs.front().id == approachAirport;
}

// Enroute rows shown above a loaded approach. When the last enroute fix is also
// the first approach leg (e.g. VASES / VASES iaf), the approach copy wins.
inline int fplEnrouteDisplayLegCount(const std::vector<MapLeg>& legs,
                                     int approachStart) {
  if (approachStart <= 0) return 0;
  int count = approachStart;
  if (approachStart >= static_cast<int>(legs.size())) return count;
  const std::string& firstApproachId =
      legs[static_cast<std::size_t>(approachStart)].id;
  while (count > 0 &&
         legs[static_cast<std::size_t>(count - 1)].id == firstApproachId) {
    --count;
  }
  return count;
}

// When the same ident appears again later in the plan, only the last row is
// shown (matches trainer: enroute feeder vs IAF — bottom copy wins).
inline bool fplLegIdentsEqual(const std::string& a, const std::string& b) {
  if (a.empty() || b.empty()) return false;
  if (a == b) return true;
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (static_cast<char>(std::toupper(static_cast<unsigned char>(a[i]))) !=
        static_cast<char>(std::toupper(static_cast<unsigned char>(b[i])))) {
      return false;
    }
  }
  return true;
}

// When a loaded approach names an IAF transition, anchor the block at that
// fix if it appears at or before the inferred start (sim exports often omit
// procedureRole on the IAF while tagging only the FAF downstream).
inline int approachBlockStartFromTransition(const std::vector<MapLeg>& legs,
                                            const std::string& transition,
                                            int inferredStart) {
  if (transition.empty() || inferredStart < 0) return inferredStart;
  int start = inferredStart;
  for (int i = 0; i <= inferredStart && i < static_cast<int>(legs.size()); ++i) {
    if (fplLegIdentsEqual(legs[static_cast<std::size_t>(i)].id, transition)) {
      start = i;
    }
  }
  return start;
}

// Resolve approach grouping for FPL display: honor stored start/count when valid,
// otherwise infer from procedureRole tags, then anchor at the loaded transition.
inline InferredProcedureBlock resolveApproachBlockInPlan(
    const std::vector<MapLeg>& legs, int storedStart, int storedCount,
    const std::string& transition, int arrivalEnd = 0, int departureEnd = 0) {
  InferredProcedureBlock out;
  const int minStart =
      fplApproachInferenceFloor(legs, arrivalEnd, departureEnd);
  // A stored block that falls inside a loaded arrival/STAR block is stale
  // (its legs belong to the arrival, not an approach) - re-infer the tail.
  const bool storedUsable = storedCount > 0 && storedStart >= minStart &&
                            storedStart + storedCount <=
                                static_cast<int>(legs.size());
  if (storedUsable) {
    out.start = storedStart;
    out.count = storedCount;
  } else {
    out = inferProcedureBlockInPlan(legs, minStart);
  }
  if (!out.valid()) return out;
  out.start = approachBlockStartFromTransition(legs, transition, out.start);
  if (out.start < minStart) {
    return InferredProcedureBlock{};
  }
  out.count = static_cast<int>(legs.size()) - out.start;
  if (out.count <= 0) {
    out.start = -1;
    out.count = 0;
  }
  return out;
}

inline bool fplHideLegForDuplicateIdent(const std::vector<MapLeg>& legs,
                                         int legIndex) {
  if (legIndex < 0 || legIndex >= static_cast<int>(legs.size())) {
    return false;
  }
  const std::string& id = legs[static_cast<std::size_t>(legIndex)].id;
  if (id.empty()) return false;
  for (int j = legIndex + 1; j < static_cast<int>(legs.size()); ++j) {
    if (fplLegIdentsEqual(id, legs[static_cast<std::size_t>(j)].id)) {
      return true;
    }
  }
  return false;
}

// True when the filled destination leg is tagged as an airway exit (Load Airway
// merged the exit with the waypoint that was already next in the plan). Such a
// fix is listed under its "Airway - <name>.<exit>" header, not as a separate
// Destination section row.
inline bool fplDestinationIsAirwayExit(const std::vector<MapLeg>& legs,
                                         bool destinationFilled) {
  if (!destinationFilled || legs.size() < 2) return false;
  return !legs.back().viaAirway.empty();
}

// Active-leg highlight: prefer the last plan index (approach over enroute dupes).
inline int fplActiveLegIndexInPlan(const std::vector<MapLeg>& legs,
                                   const std::string& activeIdent) {
  if (activeIdent.empty()) return -1;
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (fplLegIdentsEqual(legs[static_cast<std::size_t>(i)].id, activeIdent)) {
      return i;
    }
  }
  return -1;
}

// Active-leg highlight: resolve the visible plan row for the active TO ident
// (last duplicate wins in FPL lists). Fall back to fmaActiveLegIndex when it
// points at a displayed leg.
inline int fplResolvedActiveLegIndex(const std::vector<MapLeg>& legs,
                                     const FlightData& d,
                                     const std::string& activeIdent) {
  const std::string& toIdent =
      !activeIdent.empty() ? activeIdent : d.fmaToWpt;
  if (!toIdent.empty()) {
    const int byIdent = fplActiveLegIndexInPlan(legs, toIdent);
    if (byIdent >= 0) return byIdent;
  }
  if (d.fmaActiveLegIndex >= 0 &&
      d.fmaActiveLegIndex < static_cast<int>(legs.size())) {
    const int idx = d.fmaActiveLegIndex;
    if (!fplHideLegForDuplicateIdent(legs, idx)) return idx;
    if (!toIdent.empty()) {
      const int visible = fplActiveLegIndexInPlan(legs, toIdent);
      if (visible >= 0) return visible;
    }
    return idx;
  }
  return -1;
}

// Leg index of the GPS Direct-To target in the displayed plan (last ident match).
inline int fplDirectToTargetLegIndex(const std::vector<MapLeg>& legs,
                                     const FlightData& d,
                                     const std::string& activeIdent,
                                     bool directToActive) {
  if (!directToActive) return -1;
  const std::string& toIdent =
      !d.fmaToWpt.empty() ? d.fmaToWpt : activeIdent;
  if (toIdent.empty()) return -1;
  return fplActiveLegIndexInPlan(legs, toIdent);
}

// True when this displayed row is the current navigation TO fix.
inline bool fplRowIsNavToTarget(const MapLeg& leg, const std::string& toIdent) {
  return !toIdent.empty() && fplLegIdentsEqual(leg.id, toIdent);
}

// True when navigation is flying (or Direct-To engaging) the published hold at
// this leg. The HOLD display row carries the magenta marker, not the fix row.
inline bool fplHoldNavActiveOnLeg(const FlightData& d, bool directToHold,
                                  bool directToActive, bool dtoNavActive,
                                  int legIdx, int activeLegIdx,
                                  const MapLeg& leg) {
  if (!leg.hold.active || legIdx < 0 || legIdx != activeLegIdx) return false;
  if (d.fmaLegIsHold) return true;
  return directToHold && (directToActive || dtoNavActive);
}

// Magenta ident flash on the active navigation row follows the list cursor:
// the arrow and DTK/DIS stay on the TO fix while another row is highlighted.
inline bool fplActiveNavRowBlink(int legIdx, int activeLegIdx, int cursorLegIdx,
                                 bool cursorOn, int listCursorRow,
                                 int activeSelectableRow) {
  if (cursorOn && cursorLegIdx >= 0 && legIdx == cursorLegIdx) return true;
  return !cursorOn && legIdx == activeLegIdx && activeSelectableRow >= 0 &&
         listCursorRow == activeSelectableRow;
}

// Magenta active-leg marker on a list row: always on the live TO ident; otherwise
// follow active-leg highlight state.
inline bool fplShowActiveNavRow(bool activeHighlight, int legIdx,
                                int activeLegIdx, const MapLeg& leg,
                                const std::string& toIdent) {
  if (fplRowIsNavToTarget(leg, toIdent)) return true;
  return activeLegIdx >= 0 && legIdx == activeLegIdx && activeHighlight;
}

// CIFP metadata may under-count; the approach tail includes all legs from start.
inline int fplNormalizedApproachCount(int approachStart, int approachCount,
                                      int legCount) {
  if (approachCount <= 0 || approachStart < 0) return 0;
  if (approachStart + approachCount < legCount) {
    return legCount - approachStart;
  }
  return approachCount;
}

// Resolve the approach leg span the FPL list renderer uses (stored grouping when
// valid, otherwise infer from procedureRole tags). Corrects stale blocks that
// start at the origin airport when the approach begins on the next leg.
inline FlightPlanApproachState fplResolvedApproachState(
    const std::vector<MapLeg>& legs, const FlightPlanApproachState& stored,
    const std::string& transition, int arrivalEnd = 0, int departureEnd = 0) {
  const InferredProcedureBlock block = resolveApproachBlockInPlan(
      legs, stored.legStart, stored.legCount, transition, arrivalEnd,
      departureEnd);
  if (!block.valid()) return FlightPlanApproachState{};
  FlightPlanApproachState out = stored;
  out.legStart = block.start;
  out.legCount = block.count;
  if (out.legCount > 0 && out.legStart >= 0 &&
      out.legStart + out.legCount < static_cast<int>(legs.size())) {
    out.legCount = fplNormalizedApproachCount(
        out.legStart, out.legCount, static_cast<int>(legs.size()));
  }
  return out;
}

// Active GPS Direct-To, saved across standalone restarts so the magenta
// course and FMA TO fix survive a relaunch.
struct PersistedDirectTo {
  bool active = false;
  MapLeg target;
  double originLat = 0.0;
  double originLon = 0.0;
  bool originValid = false;
  // When Direct-To is not active, the GPS active leg index in the loaded plan
  // (so a relaunch does not default back to the departure airport).
  int activeLegIndex = -1;
};

inline bool operator==(const PersistedDirectTo& a, const PersistedDirectTo& b) {
  return a.active == b.active && a.target.id == b.target.id &&
         a.target.lat == b.target.lat && a.target.lon == b.target.lon &&
         a.originLat == b.originLat && a.originLon == b.originLon &&
         a.originValid == b.originValid && a.activeLegIndex == b.activeLegIndex;
}

inline bool operator!=(const PersistedDirectTo& a, const PersistedDirectTo& b) {
  return !(a == b);
}

inline PersistedDirectTo persistedDirectToFromMap(const MapData& map) {
  PersistedDirectTo out;
  if (!map.directToActive || map.directTo.id.empty()) return out;
  out.active = true;
  out.target = map.directTo;
  out.originLat = map.directToOriginLat;
  out.originLon = map.directToOriginLon;
  out.originValid = map.directToOriginValid;
  out.activeLegIndex = -1;
  return out;
}

inline PersistedDirectTo persistedNavigationSnapshot(const MapData& map,
                                                     const FlightData& data) {
  PersistedDirectTo out = persistedDirectToFromMap(map);
  if (out.active) return out;
  if (data.fmaActiveLegIndex >= 0 &&
      data.fmaActiveLegIndex < static_cast<int>(map.flightPlan.size())) {
    out.activeLegIndex = data.fmaActiveLegIndex;
  }
  return out;
}

// Saved across standalone restarts (waypoint legs with id/lat/lon/procedureRole).
struct PersistedFlightPlan {
  bool active = false;
  bool destinationFilled = false;
  std::vector<MapLeg> legs;
  int approachLegStart = -1;
  int approachLegCount = 0;
  std::string approachAirportIcao;
  PersistedLoadedApproach approachMeta;
  int departureLegStart = -1;
  int departureLegCount = 0;
  PersistedLoadedApproach departureMeta;
  int arrivalLegStart = -1;
  int arrivalLegCount = 0;
  PersistedLoadedApproach arrivalMeta;
};

inline bool operator==(const PersistedFlightPlan& a,
                       const PersistedFlightPlan& b) {
  if (a.active != b.active || a.destinationFilled != b.destinationFilled ||
      a.approachLegStart != b.approachLegStart ||
      a.approachLegCount != b.approachLegCount ||
      a.approachAirportIcao != b.approachAirportIcao ||
      a.approachMeta != b.approachMeta ||
      a.departureLegStart != b.departureLegStart ||
      a.departureLegCount != b.departureLegCount ||
      a.departureMeta != b.departureMeta ||
      a.arrivalLegStart != b.arrivalLegStart ||
      a.arrivalLegCount != b.arrivalLegCount ||
      a.arrivalMeta != b.arrivalMeta ||
      a.legs.size() != b.legs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.legs.size(); ++i) {
    if (a.legs[i].id != b.legs[i].id || a.legs[i].lat != b.legs[i].lat ||
        a.legs[i].lon != b.legs[i].lon ||
        a.legs[i].procedureRole != b.legs[i].procedureRole ||
        a.legs[i].viaAirway != b.legs[i].viaAirway ||
        a.legs[i].altitudeConstraintFt != b.legs[i].altitudeConstraintFt ||
        a.legs[i].altitudeConstraint != b.legs[i].altitudeConstraint ||
        a.legs[i].altitudeDesignated != b.legs[i].altitudeDesignated) {
      return false;
    }
  }
  return true;
}

inline bool operator!=(const PersistedFlightPlan& a,
                       const PersistedFlightPlan& b) {
  return !(a == b);
}

class NavFeatureSource;

struct TerminalProcedureMetadataRestore {
  bool merged = false;
  // True when CIFP expansion succeeded but no stored block legs matched (stop
  // retrying every frame).
  bool stopRetrying = false;
  // Block span after matching the expanded procedure against the plan. May grow
  // beyond the passed-in block when the importer's heuristic excluded a fix that
  // CIFP says belongs to the procedure (e.g. a STAR transition-entry fix that
  // SimBrief tagged with the inbound airway). -1 when unchanged/unmatched.
  int correctedStart = -1;
  int correctedCount = 0;
  bool spanCorrected() const { return correctedStart >= 0; }
};

// Re-attaches CIFP SID/STAR metadata (altitude constraints, holds, roles) onto
// legs in a known departure/arrival block, and corrects the block span to cover
// every contiguous plan leg the expanded procedure claims. Used when a SimBrief
// import or a saved plan carries procedure headers but bare navlog legs.
TerminalProcedureMetadataRestore restoreTerminalProcedureMetadata(
    const NavFeatureSource* nav, const PersistedLoadedApproach& meta,
    std::vector<MapLeg>& plan, int blockStart, int blockCount);

// Infer procedure metadata (runway, RNAV name, LPV) from tagged approach legs.
bool inferApproachMetadataFromLegs(const std::vector<MapLeg>& legs,
                                   int approachStart,
                                   PersistedLoadedApproach& meta);

// Nearest airport symbol to the MAP/fix leg (destination ICAO for the header).
std::string inferApproachAirportFromProcedureLegs(
    const NavFeatureSource* nav, const std::vector<MapLeg>& legs,
    int approachStart, int approachCount);

// Fill approach grouping from legs when older settings lack approach* keys.
void enrichPersistedFlightPlanFromLegs(PersistedFlightPlan& plan);

// Serialize one persisted flight-plan leg for settings.txt (id/lat/lon/role and
// optional VNAV altitude fields). Older saves omit altitude; parsing accepts both.
bool parsePersistedFlightPlanLeg(const std::string& value, MapLeg& legOut);
std::string formatPersistedFlightPlanLeg(const MapLeg& leg);

}  // namespace avionics
