#include "avionics/CifpParser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
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

std::string upperCopy(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
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

// ARINC 424 semicolon field (e.g. N26345109, W081521218).
bool parseArincCoordinate(const std::string& raw, double& degOut) {
  const std::string s = trim(raw);
  if (s.size() < 8) return false;
  const char hemi =
      static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
  if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W') return false;
  const bool isLat = hemi == 'N' || hemi == 'S';

  const std::string digits = s.substr(1);
  const int degDigits = isLat ? 2 : 3;
  const int minStart = degDigits;
  const int secStart = degDigits + 2;
  if (static_cast<int>(digits.size()) < secStart + 2) return false;

  const double dd = std::strtod(digits.substr(0, degDigits).c_str(), nullptr);
  const double mm =
      std::strtod(digits.substr(minStart, 2).c_str(), nullptr);
  const std::string secRest = digits.substr(secStart);
  double ss = std::strtod(secRest.substr(0, 2).c_str(), nullptr);
  if (secRest.size() > 2) {
    const double frac = std::strtod(secRest.substr(2).c_str(), nullptr);
    const double divisor =
        std::pow(10.0, static_cast<double>(secRest.size() - 2));
    ss += frac / divisor;
  }

  double deg = dd + mm / 60.0 + ss / 3600.0;
  if (hemi == 'S' || hemi == 'W') deg = -deg;
  degOut = deg;
  return std::fabs(deg) <= (isLat ? 90.0 : 180.0);
}

bool parseRwyRecord(const std::string& payload,
                    std::unordered_map<std::string, std::pair<double, double>>&
                        runways) {
  const std::size_t semi = payload.find(';');
  const std::string runwayPart =
      semi == std::string::npos ? payload : payload.substr(0, semi);
  std::string coordPart =
      semi == std::string::npos ? std::string() : payload.substr(semi + 1);
  if (!coordPart.empty() && coordPart.back() == ';') coordPart.pop_back();

  const std::vector<std::string> fields = splitCsv(trim(runwayPart));
  if (fields.empty()) return false;
  const std::string rwyIdent = trim(fields[0]);
  if (rwyIdent.empty()) return false;

  const std::vector<std::string> coordFields = splitCsv(trim(coordPart));
  if (coordFields.size() < 2) return false;
  double lat = 0.0;
  double lon = 0.0;
  if (!parseArincCoordinate(coordFields[0], lat) ||
      !parseArincCoordinate(coordFields[1], lon)) {
    return false;
  }
  runways[rwyIdent] = {lat, lon};
  return true;
}

ProcedureType kindFromRowCode(const std::string& code) {
  if (code == "SID") return ProcedureType::Departure;
  if (code == "STAR") return ProcedureType::Arrival;
  return ProcedureType::Approach;
}

bool navigableTerminator(const std::string& term) {
  static const char* kTerms[] = {"IF", "TF", "DF", "CF", "HM", "PI", "RF", "AF",
                                 "FA", "FC", "FD", "FM", "HA", "HF", "VA", "CA",
                                 "VI", "VM", "CR", "CD", "CI"};
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

std::string roleFromWaypointDesc(const std::string& raw) {
  std::string s = trim(raw);
  if (s.empty()) return {};
  while (!s.empty() && s.back() == ' ') s.pop_back();
  if (s.empty()) return {};
  const char c = static_cast<char>(
      std::toupper(static_cast<unsigned char>(s.back())));
  switch (c) {
    case 'A':
      return "iaf";
    case 'F':
      return "faf";
    case 'M':
      return "mapt";
    case 'H':
      return "mahp";
    default:
      return {};
  }
}

std::string approachLegRole(const CifpLeg& leg) {
  const std::string fromDesc = roleFromWaypointDesc(leg.waypointDesc);
  if (!fromDesc.empty()) return fromDesc;

  auto roleFromToken = [](const std::string& raw) -> std::string {
    if (raw.empty()) return {};
    std::string q = trim(raw);
    std::string lower;
    for (char c : q) {
      lower += static_cast<char>(
          std::tolower(static_cast<unsigned char>(c)));
    }
    if (lower == "iaf" || lower == "iafc") return "iaf";
    if (lower == "faf" || lower == "fafc") return "faf";
    if (lower == "mapt" || lower == "map") return "mapt";
    if (lower == "mahp") return "mahp";
    const std::string upper = upperCopy(q);
    if (upper == "IAF") return "iaf";
    if (upper == "FAF") return "faf";
    if (upper == "MAP" || upper == "MAPT") return "mapt";
    if (upper == "MAHP") return "mahp";
    return {};
  };
  for (const std::string& raw :
       {leg.qualifier1, leg.qualifier2, leg.gpsFmsIndication}) {
    const std::string role = roleFromToken(raw);
    if (!role.empty()) return role;
  }
  return {};
}

std::string runwayFromTransition(const std::string& transition) {
  if (transition.size() < 3) return {};
  if (transition[0] != 'R' || transition[1] != 'W') return {};
  return transition.substr(2);
}

bool isRunwayToken(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) {
    if (!std::isdigit(static_cast<unsigned char>(c)) && c != 'L' && c != 'R') {
      return false;
    }
  }
  return true;
}

bool decodeAbbreviatedApproachName(const std::string& name, std::string& typeOut,
                                   std::string& runwayOut) {
  const std::string upper = upperCopy(name);
  if (upper.size() >= 6 && upper.substr(0, 6) == "VISUAL") {
    const std::string tail = upper.substr(6);
    if (isRunwayToken(tail)) {
      typeOut = "VISUAL";
      runwayOut = tail;
      return true;
    }
  }
  if (upper.size() >= 4 && upper.substr(0, 3) == "VOR" &&
      isRunwayToken(upper.substr(3))) {
    typeOut = "VOR";
    runwayOut = upper.substr(3);
    return true;
  }
  if (upper.size() >= 2 &&
      std::isalpha(static_cast<unsigned char>(upper[0]))) {
    const std::string tail = upper.substr(1);
    if (!isRunwayToken(tail)) return false;
    switch (upper[0]) {
      case 'I':
        typeOut = "ILS";
        runwayOut = tail;
        return true;
      case 'L':
        typeOut = "LOC";
        runwayOut = tail;
        return true;
      case 'R':
        typeOut = "RNAV";
        runwayOut = tail;
        return true;
      case 'V':
        typeOut = "VOR";
        runwayOut = tail;
        return true;
      case 'N':
        typeOut = "NDB";
        runwayOut = tail;
        return true;
      default:
        break;
    }
  }
  return false;
}

std::string inferTransitionForApproach(const CifpLeg& leg) {
  std::string type;
  std::string runway;
  if (decodeAbbreviatedApproachName(leg.procedureName, type, runway)) {
    return "RW" + runway;
  }
  const std::string upper = upperCopy(leg.procedureName);
  if (upper.size() >= 3 && upper.substr(0, 3) == "VOR" && upper.size() > 3) {
    return "RW" + upper.substr(3);
  }
  return "VECTORS";
}

std::string normalizeLevelOfService(const std::string& raw) {
  const std::string upper = upperCopy(trim(raw));
  if (upper.empty()) return {};
  if (upper.find("LPV") != std::string::npos) return "LPV";
  if (upper.find("LNAV/VNAV") != std::string::npos) return "LNAV/VNAV";
  if (upper.find("LNAV") != std::string::npos) return "LNAV";
  return upper;
}

std::string inferLevelOfService(const CifpLeg& leg) {
  const std::string fromField = normalizeLevelOfService(leg.levelOfService);
  if (!fromField.empty()) return fromField;

  const std::string gps = upperCopy(leg.gpsFmsIndication);
  if (gps.find("LPV") != std::string::npos) return "LPV";
  if (gps.find("LNAV") != std::string::npos) return "LNAV";

  const std::string q1 = upperCopy(leg.qualifier1);
  const std::string q2 = upperCopy(leg.qualifier2);
  if (q1.find("VISUAL") != std::string::npos ||
      q2.find("VISUAL") != std::string::npos) {
    return "VISUAL";
  }

  const char kind =
      leg.approachKind.empty() ? '\0' : upperCopy(leg.approachKind)[0];
  if (kind == 'R' && leg.rnp > 0.0f && leg.rnp <= 0.35f) return "LPV";
  return {};
}

int levelOfServiceRank(const std::string& los) {
  if (los == "LPV") return 0;
  if (los == "LNAV/VNAV") return 1;
  if (los == "LNAV") return 2;
  return 3;
}

void mergeLevelOfService(std::string& into, const std::string& candidate) {
  const std::string norm = normalizeLevelOfService(candidate);
  if (norm.empty()) return;
  if (into.empty() || levelOfServiceRank(norm) < levelOfServiceRank(into)) {
    into = norm;
  }
}

void absorbPrdatLevelOfService(const std::vector<std::string>& fields,
                               std::unordered_map<std::string, std::string>& losByProc) {
  std::string procHint;
  if (fields.size() > 15) {
    procHint = trim(fields[14]) + trim(fields[15]);
  }
  std::string losFound;
  for (const std::string& field : fields) {
    mergeLevelOfService(losFound, field);
  }
  if (!losFound.empty() && !procHint.empty()) {
    mergeLevelOfService(losByProc[procHint], losFound);
  }
}

void buildCatalog(CifpAirportProcedures& out,
                  const std::unordered_map<std::string, std::string>& losByProc) {
  std::unordered_set<std::string> seen;
  std::unordered_map<std::string, std::string> losByName;

  for (const CifpLeg& leg : out.legs) {
    mergeLevelOfService(losByName[leg.procedureName], inferLevelOfService(leg));
    const std::string losKey = leg.approachKind + leg.qualifier1 + leg.qualifier2;
    if (!losKey.empty()) {
      auto it = losByProc.find(losKey);
      if (it != losByProc.end()) {
        mergeLevelOfService(losByName[leg.procedureName], it->second);
      }
    }
  }

  for (const CifpLeg& leg : out.legs) {
    std::string transition = trim(leg.transition);
    if (transition.empty() && leg.routeType != "5") {
      if (leg.kind == ProcedureType::Approach) {
        transition = inferTransitionForApproach(leg);
      } else {
        continue;
      }
    }

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
    if (proc.runway.empty()) {
      std::string ignored;
      std::string rwy;
      if (decodeAbbreviatedApproachName(leg.procedureName, ignored, rwy)) {
        proc.runway = rwy;
      }
    }
    proc.approachKind = leg.approachKind;
    proc.levelOfService = losByName[leg.procedureName];
    if (proc.levelOfService.empty() && proc.approachKind == "R") {
      proc.levelOfService = "LPV";
    }
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

int approachRouteRank(const std::string& routeType) {
  if (routeType == "A") return 0;
  if (routeType == "B") return 1;
  if (routeType == "R" || routeType == "I") return 2;
  if (routeType == "M") return 3;
  return 4;
}

bool legMatchesSelection(const CifpLeg& leg, ProcedureType type,
                         const std::string& name,
                         const std::string& transition) {
  if (leg.kind != type || leg.procedureName != name) return false;

  // Common feeder (route type 5) applies to every transition selection.
  if (leg.routeType == "5") return true;

  // Missed approach segment (route type M) is always part of a loaded approach
  // on the trainer FPL (HADMO, MAP hold, MAHP, etc.), regardless of IAF choice.
  if (leg.routeType == "M") return true;

  const std::string legTransition = trim(leg.transition);
  if (legTransition == transition) return true;
  if (leg.kind == ProcedureType::Approach && legTransition.empty()) {
    if (inferTransitionForApproach(leg) == transition) return true;
    // FAA CIFP encodes the final approach course and missed approach as route
    // type R (RNAV) or I (ILS/LOC) with an empty transition column. Those legs
    // apply regardless of which IAF feeder was selected (e.g. CITAG → UZAWO →
    // GRAMS → RW05).
    if (leg.routeType == "R" || leg.routeType == "I") return true;
  }

  // Some databases tag the runway transition explicitly in the transition column.
  if (leg.kind == ProcedureType::Approach) {
    std::string abbrevType;
    std::string runway;
    if (decodeAbbreviatedApproachName(name, abbrevType, runway) &&
        !runway.empty() && legTransition == "RW" + runway) {
      return true;
    }
  }
  return false;
}

}  // namespace

CifpAirportProcedures parseCifp(std::istream& in, const std::string& icao) {
  CifpAirportProcedures out;
  out.icao = icao;
  std::unordered_map<std::string, std::string> losByProc;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) continue;

    const std::string rowCode = trim(line.substr(0, colon));
    std::string payload = line.substr(colon + 1);
    if (!payload.empty() && payload.back() == ';') payload.pop_back();

    const std::vector<std::string> fields = splitCsv(payload);
    if (fields.empty()) continue;

    if (rowCode == "PRDAT") {
      absorbPrdatLevelOfService(fields, losByProc);
      continue;
    }
    if (rowCode == "RWY") {
      parseRwyRecord(payload, out.runways);
      continue;
    }
    if (rowCode != "SID" && rowCode != "STAR" && rowCode != "APPCH") continue;
    if (fields.size() < 4) continue;

    CifpLeg leg;
    leg.kind = kindFromRowCode(rowCode);
    leg.sequence = static_cast<int>(std::strtol(fields[0].c_str(), nullptr, 10));
    leg.routeType = trim(fields[1]);
    leg.procedureName = trim(fields[2]);
    leg.transition = trim(fields[3]);
    if (fields.size() > 4) leg.fixIdent = trim(fields[4]);
    if (fields.size() > 8) leg.waypointDesc = trim(fields[8]);
    if (fields.size() > 11) leg.pathTerminator = trim(fields[11]);
    if (fields.size() > 10) {
      leg.rnp = static_cast<float>(std::strtod(fields[10].c_str(), nullptr));
    }
    if (fields.size() > 35) leg.gpsFmsIndication = trim(fields[35]);
    if (fields.size() > 36) leg.qualifier1 = trim(fields[36]);
    if (fields.size() > 37) leg.qualifier2 = trim(fields[37]);
    if (leg.kind == ProcedureType::Approach) leg.approachKind = leg.routeType;

    if (leg.procedureName.empty()) continue;
    out.legs.push_back(std::move(leg));
  }

  buildCatalog(out, losByProc);
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
              const int ra = approachRouteRank(a.routeType);
              const int rb = approachRouteRank(b.routeType);
              if (ra != rb) return ra < rb;
              return a.sequence < b.sequence;
            });

  std::unordered_set<std::string> added;
  std::string lastRouteType;
  for (const CifpLeg& leg : selected) {
    if (leg.routeType != lastRouteType) {
      // Same fix may repeat in the missed segment (route type M), but feeder IF
      // and final-segment IF (e.g. UZAWO) share a fix and should not duplicate.
      if (leg.routeType == "M") {
        added.clear();
      }
      lastRouteType = leg.routeType;
    }
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
    ml.procedureRole = approachLegRole(leg);
    result.push_back(std::move(ml));
  }
  return result;
}

}  // namespace avionics
