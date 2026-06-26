#pragma once

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

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

std::vector<std::string> uniqueProcedureNames(
    const std::vector<MapProcedure>& catalog);

}  // namespace avionics
