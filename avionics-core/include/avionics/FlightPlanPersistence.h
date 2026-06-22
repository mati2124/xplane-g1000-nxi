#pragma once

#include <string>

#include <cctype>

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

// First contiguous procedure-leg block in `legs` (requires at least one leg before it).
InferredProcedureBlock inferProcedureBlockInPlan(const std::vector<MapLeg>& legs);

// Copies procedureRole (and any future per-leg PROC fields) from expanded legs
// into the matching plan rows.
void mergeProcedureLegMetadata(std::vector<MapLeg>& plan, int start,
                               const std::vector<MapLeg>& procedureLegs);

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

inline bool approachStateFitsPlan(const FlightPlanApproachState& state,
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

// CIFP metadata may under-count; the approach tail includes all legs from start.
inline int fplNormalizedApproachCount(int approachStart, int approachCount,
                                      int legCount) {
  if (approachCount <= 0 || approachStart < 0) return 0;
  if (approachStart + approachCount < legCount) {
    return legCount - approachStart;
  }
  return approachCount;
}

// Saved across standalone restarts (waypoint legs with id/lat/lon).
struct PersistedFlightPlan {
  bool active = false;
  bool destinationFilled = false;
  std::vector<MapLeg> legs;
};

inline bool operator==(const PersistedFlightPlan& a,
                       const PersistedFlightPlan& b) {
  if (a.active != b.active || a.destinationFilled != b.destinationFilled ||
      a.legs.size() != b.legs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.legs.size(); ++i) {
    if (a.legs[i].id != b.legs[i].id || a.legs[i].lat != b.legs[i].lat ||
        a.legs[i].lon != b.legs[i].lon) {
      return false;
    }
  }
  return true;
}

inline bool operator!=(const PersistedFlightPlan& a,
                       const PersistedFlightPlan& b) {
  return !(a == b);
}

}  // namespace avionics
