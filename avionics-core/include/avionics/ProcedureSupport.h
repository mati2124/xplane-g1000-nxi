#pragma once

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Runway-dropdown sentinel for an Arrival/Departure procedure that has no
// runway-specific transition (the trainer shows "ALL").
inline constexpr const char* kProcRunwayAll = "ALL";

void insertProcedureLegs(ProcedureType type, std::vector<MapLeg>& fplLegs,
                         const std::vector<MapLeg>& legs);

MapProcedure findProcedureInCatalog(ProcedureType type, const std::string& name,
                                    const std::string& transition,
                                    const std::vector<MapProcedure>& catalog);

std::string formatApproachProcedureLabel(const MapProcedure& proc);

bool procedureUsesNavPrimaryFrequency(const MapProcedure& proc);

struct ProcPrimaryNav {
  float frequency = 0.0f;
  bool isNdb = false;
  std::string ident;
};

ProcPrimaryNav resolveProcPrimaryNav(const NavFeatureSource* nav,
                                     const MapData* map,
                                     const MapFeature& airport,
                                     const MapProcedure& proc);

// Merge the published primary navaid frequency into proc.frequencyMhz when the
// approach uses a ground-based navaid (ILS/LOC/VOR/NDB). Matches the NXi PRIM
// FREQ row and the standby tune on Load.
void mergeProcedurePrimaryNavFreq(const NavFeatureSource* nav,
                                  const MapData* map,
                                  const MapFeature& airport,
                                  MapProcedure& proc);

// Standby MHz to load into NAV1 when an approach is committed (0 = none).
float procedureApproachNavStandbyMhz(const MapProcedure& proc);

std::string defaultProcedureTransition(
    const std::vector<std::string>& transitions);

std::vector<std::string> procedureTransitionIds(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name);

std::vector<std::string> procedureTransitionLabels(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name);

// Arrival/Departure (STAR/SID) selection splits the published transitions into
// the enroute transition list (named entry/exit fixes) and the runway
// transition list. The runway list always offers at least the kProcRunwayAll
// sentinel when a procedure has no runway-specific transition.
std::vector<std::string> procedureEnrouteTransitions(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name);
std::vector<std::string> procedureRunwayOptions(const NavFeatureSource* nav,
                                                const std::string& icao,
                                                ProcedureType type,
                                                const std::string& name);

// Expand a STAR/SID for a chosen enroute transition combined with a runway
// transition, ordering the legs the way the unit sequences them (Arrival:
// enroute -> common body -> runway; Departure: runway -> common body ->
// enroute). A runwayLabel of kProcRunwayAll or empty means no runway segment.
std::vector<MapLeg> expandArrivalDepartureProcedure(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name, const std::string& enrouteTransition,
    const std::string& runwayLabel);

std::vector<std::string> uniqueProcedureNames(
    const std::vector<MapProcedure>& catalog);

// Departure FPL rows: RWxx threshold, climb-to-alt (<alt>FT), vectors (MANSEQ).
bool isRunwayTransitionId(const std::string& transition);
bool isRunwayDepartureLegId(const std::string& id);
bool isHeadingDepartureLeg(const MapLeg& leg);

// True for synthetic departure rows that are not real navigable fixes (runway
// threshold, climb-to-altitude, heading/vector legs). Direct-To cannot target
// these, so the cursor must resolve to the next real fix instead.
bool isNonFixDepartureLeg(const MapLeg& leg);

// Starting at legIndex, return the index of the first leg that is a real
// navigable fix (skipping non-fix departure legs such as RWxx/<alt>FT/MANSEQ).
// Returns legIndex unchanged when it already points at a fix, or -1 when no
// fix exists at or after legIndex.
int nextNavigableFixLegIndex(const std::vector<MapLeg>& legs, int legIndex);

}  // namespace avionics
