#pragma once

#include <istream>
#include <string>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// One leg row from an X-Plane CIFP file (XP-CIFP1101: SID/STAR/APPCH records).
struct CifpLeg {
  ProcedureType kind = ProcedureType::Approach;
  int sequence = 0;
  std::string routeType;
  std::string procedureName;
  std::string transition;
  std::string fixIdent;
  std::string pathTerminator;
  std::string approachKind;  // APPCH route-type column (I, R, ...)
};

// Parsed terminal procedures for one airport.
struct CifpAirportProcedures {
  std::string icao;
  std::vector<CifpLeg> legs;
  std::vector<MapProcedure> catalog;
};

// Parses an airport CIFP file (one ICAO per file). Returns empty on failure.
CifpAirportProcedures parseCifp(std::istream& in, const std::string& icao);

// Builds the FMS leg list for a selected procedure + transition. fixLookup
// resolves each published fix ident to coordinates (returns false when unknown).
using CifpFixLookup = bool (*)(const std::string& ident, double& lat,
                               double& lon, void* ctx);

std::vector<MapLeg> expandCifpProcedure(const CifpAirportProcedures& data,
                                        ProcedureType type,
                                        const std::string& name,
                                        const std::string& transition,
                                        CifpFixLookup lookup, void* ctx);

}  // namespace avionics
