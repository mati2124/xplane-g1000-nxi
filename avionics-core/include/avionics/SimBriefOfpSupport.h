#pragma once

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"

namespace avionics {

// Contiguous SID/STAR leg ranges inferred from SimBrief navlog via_airway tags.
struct SimBriefProcedureBlocks {
  int departureLegStart = -1;
  int departureLegCount = 0;
  int arrivalLegStart = -1;
  int arrivalLegCount = 0;
};

// Classifies navlog legs (indices 1 .. legCount-2; origin=0, destination=last)
// into departure (SID) and arrival (STAR) blocks by matching each leg's
// via_airway field to sidIdent / starIdent (case-insensitive).
SimBriefProcedureBlocks inferSimBriefProcedureBlocks(
    int legCount, const std::vector<std::string>& legViaAirways,
    const std::string& sidIdent, const std::string& starIdent);

// Trainer-style procedure suffix for the FPL parent row: "RW05.CSHEL6.LAL".
// runway is the bare runway designator (e.g. "05"); procedureName is the
// SID/STAR ident; transition is optional.
std::string formatTerminalProcedureFplHeaderLabel(const std::string& runway,
                                                  const std::string& procedureName,
                                                  const std::string& transition);

// Fields copied from a parsed SimBrief OFP into a stored catalog entry.
struct SimBriefOfpImport {
  std::vector<MapLeg> legs;
  std::string originIcao;
  std::string destinationIcao;
  std::string sidIdent;
  std::string sidTrans;
  std::string starIdent;
  std::string starTrans;
  std::string originRunway;
  std::string destRunway;
  int departureLegStart = -1;
  int departureLegCount = 0;
  int arrivalLegStart = -1;
  int arrivalLegCount = 0;
};

// Builds departure/arrival metadata on a PersistedFlightPlan from SimBrief.
PersistedFlightPlan persistedFlightPlanFromSimBriefImport(
    const SimBriefOfpImport& imp);

}  // namespace avionics
