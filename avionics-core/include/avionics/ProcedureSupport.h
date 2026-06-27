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

}  // namespace avionics
