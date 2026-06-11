#include "avionics/CifpParser.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>

namespace avionics {
namespace {

std::string trim(const std::string& s) {
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  for (char c : line) {
    if (c == ',') {
      out.push_back(field);
      field.clear();
    } else {
      field += c;
    }
  }
  out.push_back(field);
  return out;
}

ProcedureType kindFromRowCode(const std::string& code) {
  if (code == "SID") return ProcedureType::Departure;
  if (code == "STAR") return ProcedureType::Arrival;
  return ProcedureType::Approach;
}

bool navigableTerminator(const std::string& term) {
  static const char* kTerms[] = {"IF", "TF", "DF", "CF", "HM", "PI", "RF", "AF",
                                 "FA", "FC", "FD", "FM", "HA", "HF", "VA"};
  for (const char* t : kTerms) {
    if (term == t) return true;
  }
  return false;
}

bool looksLikeFix(const std::string& id) {
  if (id.empty()) return false;
  for (char c : id) {
    if (!std::isalnum(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

std::string runwayFromTransition(const std::string& transition) {
  if (transition.size() < 3) return {};
  if (transition[0] != 'R' || transition[1] != 'W') return {};
  return transition.substr(2);
}

void buildCatalog(CifpAirportProcedures& out) {
  std::unordered_set<std::string> seen;
  for (const CifpLeg& leg : out.legs) {
    std::string transition = trim(leg.transition);
    if (transition.empty() && leg.routeType != "5") continue;

    // Common-route rows are not user-selectable on their own.
    if (leg.kind == ProcedureType::Departure && leg.routeType == "5") continue;

    const std::string key = std::to_string(static_cast<int>(leg.kind)) + '|' +
                            leg.procedureName + '|' + transition;
    if (!seen.insert(key).second) continue;

    MapProcedure proc;
    proc.type = leg.kind;
    proc.name = leg.procedureName;
    proc.transition = transition;
    proc.runway = runwayFromTransition(transition);
    proc.approachKind = leg.approachKind;
    out.catalog.push_back(std::move(proc));
  }

  std::sort(out.catalog.begin(), out.catalog.end(),
            [](const MapProcedure& a, const MapProcedure& b) {
              if (a.type != b.type) {
                return static_cast<int>(a.type) < static_cast<int>(b.type);
              }
              if (a.name != b.name) return a.name < b.name;
              return a.transition < b.transition;
            });
}

bool legMatchesSelection(const CifpLeg& leg, ProcedureType type,
                         const std::string& name,
                         const std::string& transition) {
  if (leg.kind != type || leg.procedureName != name) return false;
  if (leg.routeType == "5") return true;  // common portion
  return trim(leg.transition) == transition;
}

}  // namespace

CifpAirportProcedures parseCifp(std::istream& in, const std::string& icao) {
  CifpAirportProcedures out;
  out.icao = icao;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) continue;

    const std::string rowCode = trim(line.substr(0, colon));
    if (rowCode != "SID" && rowCode != "STAR" && rowCode != "APPCH") continue;

    std::string payload = line.substr(colon + 1);
    if (!payload.empty() && payload.back() == ';') payload.pop_back();

    const std::vector<std::string> fields = splitCsv(payload);
    if (fields.size() < 4) continue;

    CifpLeg leg;
    leg.kind = kindFromRowCode(rowCode);
    leg.sequence = static_cast<int>(std::strtol(fields[0].c_str(), nullptr, 10));
    leg.routeType = trim(fields[1]);
    leg.procedureName = trim(fields[2]);
    leg.transition = trim(fields[3]);
    if (fields.size() > 4) leg.fixIdent = trim(fields[4]);
    if (fields.size() > 12) leg.pathTerminator = trim(fields[12]);
    if (leg.kind == ProcedureType::Approach) leg.approachKind = leg.routeType;

    if (leg.procedureName.empty()) continue;
    out.legs.push_back(std::move(leg));
  }

  buildCatalog(out);
  return out;
}

std::vector<MapLeg> expandCifpProcedure(const CifpAirportProcedures& data,
                                        ProcedureType type,
                                        const std::string& name,
                                        const std::string& transition,
                                        CifpFixLookup lookup, void* ctx) {
  std::vector<MapLeg> result;
  if (name.empty() || transition.empty() || lookup == nullptr) return result;

  std::vector<CifpLeg> selected;
  for (const CifpLeg& leg : data.legs) {
    if (legMatchesSelection(leg, type, name, transition)) {
      selected.push_back(leg);
    }
  }
  std::sort(selected.begin(), selected.end(),
            [](const CifpLeg& a, const CifpLeg& b) {
              return a.sequence < b.sequence;
            });

  std::unordered_set<std::string> added;
  for (const CifpLeg& leg : selected) {
    if (!navigableTerminator(leg.pathTerminator)) continue;
    if (!looksLikeFix(leg.fixIdent)) continue;
    if (!added.insert(leg.fixIdent).second) continue;

    double lat = 0.0;
    double lon = 0.0;
    if (!lookup(leg.fixIdent, lat, lon, ctx)) continue;

    MapLeg ml;
    ml.id = leg.fixIdent;
    ml.lat = lat;
    ml.lon = lon;
    result.push_back(std::move(ml));
  }
  return result;
}

}  // namespace avionics
