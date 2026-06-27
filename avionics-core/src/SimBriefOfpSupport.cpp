#include "avionics/SimBriefOfpSupport.h"

#include <cctype>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"

namespace avionics {
namespace {

std::string upperCopy(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
}

bool identEqualInsensitive(const std::string& a, const std::string& b) {
  return !a.empty() && upperCopy(a) == upperCopy(b);
}

void appendSegment(std::string& out, const std::string& segment) {
  if (segment.empty()) return;
  if (!out.empty()) out += '.';
  out += segment;
}

}  // namespace

SimBriefProcedureBlocks inferSimBriefProcedureBlocks(
    int legCount, const std::vector<std::string>& legViaAirways,
    const std::string& sidIdent, const std::string& starIdent) {
  SimBriefProcedureBlocks blocks;
  if (legCount < 2 ||
      static_cast<int>(legViaAirways.size()) != legCount) {
    return blocks;
  }

  const int lastNav = legCount - 2;
  if (lastNav < 1) return blocks;

  if (!sidIdent.empty()) {
    int start = -1;
    int count = 0;
    for (int i = 1; i <= lastNav; ++i) {
      if (identEqualInsensitive(legViaAirways[static_cast<std::size_t>(i)],
                                 sidIdent)) {
        if (start < 0) start = i;
        ++count;
      } else if (start >= 0) {
        break;
      }
    }
    if (start >= 0) {
      blocks.departureLegStart = start;
      blocks.departureLegCount = count;
    }
  }

  if (!starIdent.empty()) {
    int start = -1;
    int count = 0;
    for (int i = lastNav; i >= 1; --i) {
      if (identEqualInsensitive(legViaAirways[static_cast<std::size_t>(i)],
                                 starIdent)) {
        if (start < 0) start = i;
        ++count;
      } else if (start >= 0) {
        break;
      }
    }
    if (start >= 0) {
      blocks.arrivalLegStart = start - count + 1;
      blocks.arrivalLegCount = count;
    }
  }

  return blocks;
}

std::string formatTerminalProcedureFplHeaderLabel(const std::string& runway,
                                                  const std::string& procedureName,
                                                  const std::string& transition) {
  std::string label;
  if (!runway.empty()) {
    appendSegment(label, runway.size() >= 2 && (runway[0] == 'R' || runway[0] == 'r') &&
                              (runway[1] == 'W' || runway[1] == 'w')
                      ? runway
                      : "RW" + runway);
  }
  appendSegment(label, procedureName);
  appendSegment(label, transition);
  return label;
}

PersistedFlightPlan persistedFlightPlanFromSimBriefImport(
    const SimBriefOfpImport& imp) {
  PersistedFlightPlan out;
  out.active = true;
  out.legs = imp.legs;
  out.destinationFilled = imp.legs.size() >= 2;
  out.departureLegStart = imp.departureLegStart;
  out.departureLegCount = imp.departureLegCount;
  out.arrivalLegStart = imp.arrivalLegStart;
  out.arrivalLegCount = imp.arrivalLegCount;

  if (!imp.sidIdent.empty()) {
    out.departureMeta.active = true;
    out.departureMeta.type = ProcedureType::Departure;
    out.departureMeta.airportIcao = imp.originIcao;
    out.departureMeta.name = imp.sidIdent;
    out.departureMeta.transition = imp.sidTrans;
    out.departureMeta.runway = imp.originRunway;
  }
  if (!imp.starIdent.empty()) {
    out.arrivalMeta.active = true;
    out.arrivalMeta.type = ProcedureType::Arrival;
    out.arrivalMeta.airportIcao = imp.destinationIcao;
    out.arrivalMeta.name = imp.starIdent;
    out.arrivalMeta.transition = imp.starTrans;
    out.arrivalMeta.runway = imp.destRunway;
  }

  enrichPersistedFlightPlanFromLegs(out);
  return out;
}

}  // namespace avionics
