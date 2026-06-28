#include "avionics/CifpParser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>

#include "avionics/NavMath.h"

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

bool isRunwayFixIdent(const std::string& id) {
  if (id.size() < 3) return false;
  if (id[0] != 'R' || id[1] != 'W') return false;
  for (std::size_t i = 2; i < id.size(); ++i) {
    const char c = id[i];
    if (!std::isdigit(static_cast<unsigned char>(c)) && c != 'L' && c != 'R') {
      return false;
    }
  }
  return true;
}

int parseAltitudeFtField(const std::string& raw) {
  const std::string s = trim(raw);
  if (s.empty()) return 0;
  return static_cast<int>(std::strtol(s.c_str(), nullptr, 10));
}

float parseVerticalAngleDegField(const std::string& raw) {
  const std::string s = trim(raw);
  if (s.empty()) return 0.0f;
  double v = std::strtod(s.c_str(), nullptr);
  if (std::fabs(v) >= 100.0) v /= 100.0;
  return static_cast<float>(std::fabs(v));
}

// ARINC APPCH magnetic course (DDD.d): 0510 -> 51.0 deg, 1740 -> 174.0 deg.
float parseMagneticCourseField(const std::string& raw) {
  const std::string s = trim(raw);
  if (s.empty()) return 0.0f;
  const double v = std::strtod(s.c_str(), nullptr);
  if (v <= 0.0) return 0.0f;
  return static_cast<float>(v / 10.0);
}

// Leg length in NM (0040 -> 4.0) or time in minutes (T010 -> 1.0).
void parseLegLengthOrTimeField(const std::string& raw, float& legLengthNm,
                               float& legTimeMin) {
  const std::string s = trim(raw);
  if (s.empty()) return;
  if (s.size() >= 2 &&
      (s[0] == 'T' || s[0] == 't')) {
    const double v = std::strtod(s.substr(1).c_str(), nullptr);
    if (v > 0.0) legTimeMin = static_cast<float>(v / 10.0);
    return;
  }
  const double v = std::strtod(s.c_str(), nullptr);
  if (v > 0.0) legLengthNm = static_cast<float>(v / 10.0);
}

float parseScaledRadiusNmField(const std::string& raw, double divisor) {
  const std::string s = trim(raw);
  if (s.empty()) return 0.0f;
  const double v = std::strtod(s.c_str(), nullptr);
  if (v <= 0.0) return 0.0f;
  return static_cast<float>(v / divisor);
}

HoldTurnDirection parseHoldTurnDirection(const CifpLeg& leg) {
  const std::string turn = upperCopy(trim(leg.turnDirection));
  if (turn == "L") return HoldTurnDirection::Left;
  if (turn == "R") return HoldTurnDirection::Right;
  const std::string desc = upperCopy(trim(leg.waypointDesc));
  if (!desc.empty()) {
    const char c = desc.back();
    if (c == 'L') return HoldTurnDirection::Left;
    if (c == 'R') return HoldTurnDirection::Right;
  }
  return HoldTurnDirection::None;
}

bool isHoldTerminator(const std::string& term) {
  return term == "HM" || term == "HA" || term == "HF";
}

bool isCourseLegWithoutFix(const CifpLeg& leg) {
  if (looksLikeFix(leg.fixIdent)) return false;
  return leg.pathTerminator == "CA" || leg.pathTerminator == "FM" ||
         leg.pathTerminator == "VM" || leg.pathTerminator == "VI" ||
         leg.pathTerminator == "VA";
}

bool isRunwayTransitionId(const std::string& transition) {
  return transition.size() >= 3 &&
         (transition[0] == 'R' || transition[0] == 'r') &&
         (transition[1] == 'W' || transition[1] == 'w');
}

bool isDepartureClimbTerminator(const std::string& term) {
  return term == "CA" || term == "VA";
}

bool isDepartureVectorsTerminator(const std::string& term) {
  return term == "VM" || term == "FM" || term == "VI";
}

void applyPathTerminatorFields(const CifpLeg& leg, MapLeg& ml) {
  ml.pathTerminator = leg.pathTerminator;
  if (leg.magneticCourseDeg > 0.0f) {
    ml.legCourseDeg = normalizeHeadingDeg(leg.magneticCourseDeg);
  }
}

void applyProcedureArcFromCifpLeg(const CifpLeg& leg, MapLeg& ml,
                                  CifpFixLookup lookup, void* ctx) {
  if ((leg.pathTerminator != "RF" && leg.pathTerminator != "AF") ||
      leg.arcCenterIdent.empty()) {
    return;
  }
  double lat = 0.0;
  double lon = 0.0;
  if (!lookup(leg.arcCenterIdent, lat, lon, ctx)) return;
  ml.procedureArc.active = true;
  ml.procedureArc.centerIdent = leg.arcCenterIdent;
  ml.procedureArc.centerLat = lat;
  ml.procedureArc.centerLon = lon;
  ml.procedureArc.radiusNm = leg.arcRadiusNm;
  ml.procedureArc.turn = parseHoldTurnDirection(leg);
}

// ARINC 424 HM/HA/HF magnetic course is the inbound course to the holding fix.
void applyHoldFromCifpLeg(const CifpLeg& leg, MapLeg& ml) {
  if (!isHoldTerminator(leg.pathTerminator)) return;
  const HoldTurnDirection turn = parseHoldTurnDirection(leg);
  if (turn == HoldTurnDirection::None) return;

  ml.hold.active = true;
  ml.hold.turn = turn;
  // HF = hold-to-fix: a single-circuit HILPT course reversal at the IAF. HM
  // (hold-to-manual) is a missed-approach hold and is not a course reversal.
  ml.hold.courseReversal = (leg.pathTerminator == "HF");
  ml.hold.legLengthNm = leg.legLengthNm;
  ml.hold.legTimeMin = leg.legTimeMin;
  if (ml.hold.legLengthNm <= 0.0f && ml.hold.legTimeMin <= 0.0f) {
    ml.hold.legLengthNm = 4.0f;
  }
  if (leg.magneticCourseDeg > 0.0f) {
    ml.hold.inboundCourseDeg = normalizeHeadingDeg(leg.magneticCourseDeg);
  }
}

void applyArincAltitudeConstraint(const CifpLeg& leg, MapLeg& ml) {
  const int alt1 = leg.altitude1Ft;
  const int alt2 = leg.altitude2Ft;
  const std::string desc = upperCopy(trim(leg.altitudeDescription));
  const char code = desc.empty() ? '\0' : desc[0];

  switch (code) {
    case '+':
      if (alt1 > 0) {
        ml.altitudeConstraintFt = alt1;
        ml.altitudeConstraint = AltConstraintType::AtOrAbove;
      }
      break;
    case '-':
      if (alt1 > 0) {
        ml.altitudeConstraintFt = alt1;
        ml.altitudeConstraint = AltConstraintType::AtOrBelow;
      }
      break;
    case 'B':
    case 'H':
      if (alt2 > 0) {
        ml.altitudeConstraintFt = alt2;
        ml.altitudeConstraint = AltConstraintType::At;
      } else if (alt1 > 0) {
        ml.altitudeConstraintFt = alt1;
        ml.altitudeConstraint = AltConstraintType::AtOrAbove;
      }
      break;
    case 'J':
      if (alt1 > 0) {
        ml.altitudeConstraintFt = alt1;
        ml.altitudeConstraint = AltConstraintType::AtOrAbove;
      }
      break;
  default:
      if (alt1 > 0) {
        ml.altitudeConstraintFt = alt1;
        ml.altitudeConstraint = AltConstraintType::At;
      }
      break;
  }
  ml.altitudeDesignated = false;
}

void applyMissedInitialFromCifpLeg(const CifpLeg& leg, MapLeg& mapt) {
  mapt.missedInitial.active = true;
  mapt.missedInitial.pathTerminator = leg.pathTerminator;
  if (leg.magneticCourseDeg > 0.0f) {
    mapt.missedInitial.courseDeg = normalizeHeadingDeg(leg.magneticCourseDeg);
  }
  MapLeg temp;
  if (leg.kind == ProcedureType::Approach) {
    applyArincAltitudeConstraint(leg, temp);
    if (temp.altitudeConstraintFt > 0) {
      mapt.missedInitial.altitudeFt = temp.altitudeConstraintFt;
      mapt.missedInitial.altitudeConstraint = temp.altitudeConstraint;
    }
  }
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
    case 'I':
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

// Reciprocal runway key ("RW13" -> "RW31"), used for departure centerline course.
std::string oppositeRunwayKey(const std::string& runwayKey) {
  const std::string rw = upperCopy(runwayKey);
  std::size_t digits = 2;
  if (rw.size() <= 2) return {};
  while (digits < rw.size() && rw[digits] >= '0' && rw[digits] <= '9') {
    ++digits;
  }
  if (digits <= 2) return {};
  const int num = std::stoi(rw.substr(2, digits - 2));
  const int opp = ((num + 18 - 1) % 36) + 1;
  std::string suffix = rw.substr(digits);
  if (suffix == "L") suffix = "R";
  else if (suffix == "R") suffix = "L";
  char numbuf[8];
  std::snprintf(numbuf, sizeof(numbuf), "%02d", opp);
  return "RW" + std::string(numbuf) + suffix;
}

float runwayDepartureCourseDeg(const CifpAirportProcedures& data,
                               const std::string& rwKey) {
  const auto thrIt = data.runways.find(upperCopy(rwKey));
  if (thrIt == data.runways.end()) return 0.0f;
  const auto oppIt = data.runways.find(oppositeRunwayKey(rwKey));
  if (oppIt != data.runways.end()) {
    return static_cast<float>(navBearingDeg(
        thrIt->second.first, thrIt->second.second, oppIt->second.first,
        oppIt->second.second));
  }
  const std::string rw = runwayFromTransition(rwKey);
  int num = 0;
  for (char ch : rw) {
    if (ch < '0' || ch > '9') break;
    num = num * 10 + (ch - '0');
  }
  return num > 0 ? static_cast<float>(num) * 10.0f : 0.0f;
}

MapLeg makeRunwayDepartureLeg(const std::string& rwTransition, double lat,
                              double lon) {
  MapLeg ml;
  ml.id = upperCopy(rwTransition);
  ml.lat = lat;
  ml.lon = lon;
  return ml;
}

MapLeg makeDepartureManeuverLeg(const CifpLeg& leg, double anchorLat,
                                double anchorLon, float defaultCourse) {
  MapLeg ml;
  const std::string& term = leg.pathTerminator;
  if (!isDepartureClimbTerminator(term) &&
      !isDepartureVectorsTerminator(term)) {
    return ml;
  }
  ml.pathTerminator = term;
  if (leg.magneticCourseDeg > 0.0f) {
    ml.legCourseDeg = normalizeHeadingDeg(leg.magneticCourseDeg);
  } else if (defaultCourse > 0.0f) {
    ml.legCourseDeg = defaultCourse;
  }
  if (isDepartureClimbTerminator(term)) {
    applyArincAltitudeConstraint(leg, ml);
    const int altFt =
        leg.altitude1Ft > 0 ? leg.altitude1Ft : ml.altitudeConstraintFt;
    if (altFt <= 0) return MapLeg{};
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%dFT", altFt);
    ml.id = buf;
  } else {
    ml.id = "MANSEQ";
  }
  const float course =
      ml.legCourseDeg > 0.0f ? ml.legCourseDeg : defaultCourse;
  if (course > 0.0f) {
    navOffsetPoint(anchorLat, anchorLon, course, 0.05, ml.lat, ml.lon);
  } else {
    ml.lat = anchorLat;
    ml.lon = anchorLon;
  }
  return ml;
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

bool runwaySuffixFromVariantName(const std::string& s, std::string& runwayOut) {
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) continue;
    std::size_t j = i;
    while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j]))) ++j;
    if (j - i < 2 || j - i > 3) continue;
    if (j < s.size() && (s[j] == 'L' || s[j] == 'R' || s[j] == 'C')) ++j;
    runwayOut = s.substr(i, j - i);
    return isRunwayToken(runwayOut);
  }
  return false;
}

bool decodeAbbreviatedApproachName(const std::string& name, std::string& typeOut,
                                   std::string& runwayOut) {
  typeOut.clear();
  runwayOut.clear();
  const std::string upper = upperCopy(name);
  if (upper.size() >= 6 && upper.substr(0, 6) == "VISUAL") {
    const std::string tail = upper.substr(6);
    if (isRunwayToken(tail)) {
      typeOut = "VISUAL";
      runwayOut = tail;
      return true;
    }
  }

  struct Prefix {
    const char* text;
    const char* type;
  };
  static constexpr Prefix kPrefixes[] = {{"RNAV", "RNAV"}, {"RNP", "RNAV"},
                                         {"RNV", "RNAV"},  {"ILS", "ILS"},
                                         {"LOC", "LOC"},   {"VOR", "VOR"},
                                         {"NDB", "NDB"},   {"D", "VOR"}};
  for (const Prefix& prefix : kPrefixes) {
    const std::string p = prefix.text;
    if (upper.rfind(p, 0) == 0 &&
        runwaySuffixFromVariantName(upper.substr(p.size()), runwayOut)) {
      typeOut = prefix.type;
      return true;
    }
  }

  if (upper.size() >= 2 &&
      std::isalpha(static_cast<unsigned char>(upper[0]))) {
    const std::string tail = upper.substr(1);
    if (!isRunwayToken(tail) && !runwaySuffixFromVariantName(tail, runwayOut)) {
      return false;
    }
    switch (upper[0]) {
      case 'I':
        typeOut = "ILS";
        if (runwayOut.empty()) runwayOut = tail;
        return true;
      case 'L':
        typeOut = "LOC";
        if (runwayOut.empty()) runwayOut = tail;
        return true;
      case 'R':
        typeOut = "RNAV";
        if (runwayOut.empty()) runwayOut = tail;
        return true;
      case 'V':
        typeOut = "VOR";
        if (runwayOut.empty()) runwayOut = tail;
        return true;
      case 'N':
        typeOut = "NDB";
        if (runwayOut.empty()) runwayOut = tail;
        return true;
      case 'D':
        typeOut = "VOR";
        if (runwayOut.empty()) runwayOut = tail;
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
  if (routeType != "M" && routeType != "5") return 2;
  if (routeType == "M") return 3;
  return 4;
}

bool isApproachFeederRouteType(const std::string& routeType) {
  return routeType == "A" || routeType == "B" || routeType == "5";
}

bool isApproachFinalRouteType(const std::string& routeType) {
  return !isApproachFeederRouteType(routeType) && routeType != "M";
}

bool legMatchesSelection(const CifpLeg& leg, ProcedureType type,
                         const std::string& name,
                         const std::string& transition) {
  if (leg.kind != type || leg.procedureName != name) return false;

  if (leg.kind == ProcedureType::Approach &&
      upperCopy(transition) == "VECTORS") {
    if (isApproachFeederRouteType(leg.routeType)) {
      return false;
    }
    if (leg.routeType == "M") return true;
    const std::string legTransition = trim(leg.transition);
    if (legTransition.empty() && isApproachFinalRouteType(leg.routeType)) {
      return true;
    }
    return false;
  }

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
    // types such as R (RNAV), I (ILS/LOC), D (VOR/DME), or G (GLS) with an
    // empty transition column. Those legs apply regardless of which IAF feeder
    // was selected (e.g. CITAG → UZAWO → GRAMS → RW05).
    if (isApproachFinalRouteType(leg.routeType)) return true;
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
    if (fields.size() > 9) leg.turnDirection = trim(fields[9]);
    if (fields.size() > 11) leg.pathTerminator = trim(fields[11]);
    if (fields.size() > 10) {
      leg.rnp = static_cast<float>(std::strtod(fields[10].c_str(), nullptr));
    }
    if (fields.size() > 20) {
      leg.magneticCourseDeg = parseMagneticCourseField(fields[20]);
    }
    if (fields.size() > 21) {
      parseLegLengthOrTimeField(fields[21], leg.legLengthNm, leg.legTimeMin);
    }
    if (leg.pathTerminator == "RF") {
      if (fields.size() > 17) {
        leg.arcRadiusNm = parseScaledRadiusNmField(fields[17], 1000.0);
      }
      if (fields.size() > 30) leg.arcCenterIdent = trim(fields[30]);
    } else if (leg.pathTerminator == "AF") {
      if (fields.size() > 19) {
        leg.arcRadiusNm = parseScaledRadiusNmField(fields[19], 10.0);
      }
      if (fields.size() > 13) leg.arcCenterIdent = trim(fields[13]);
    }
    if (fields.size() > 35) leg.gpsFmsIndication = trim(fields[35]);
    if (fields.size() > 36) leg.qualifier1 = trim(fields[36]);
    if (fields.size() > 37) leg.qualifier2 = trim(fields[37]);
    if (fields.size() > 22) leg.altitudeDescription = trim(fields[22]);
    if (fields.size() > 23) leg.altitude1Ft = parseAltitudeFtField(fields[23]);
    if (fields.size() > 24) leg.altitude2Ft = parseAltitudeFtField(fields[24]);
    if (fields.size() > 28) {
      leg.verticalAngleDeg = parseVerticalAngleDegField(fields[28]);
    }
    if (leg.kind == ProcedureType::Approach) leg.approachKind = leg.routeType;

    if (leg.procedureName.empty()) continue;
    if (fields.size() > 18 && !leg.fixIdent.empty()) {
      double fixLat = 0.0;
      double fixLon = 0.0;
      if (parseArincCoordinate(fields[17], fixLat) &&
          parseArincCoordinate(fields[18], fixLon)) {
        out.fixes[leg.fixIdent] = {fixLat, fixLon};
      }
    }
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

  // When `transition` names a published IAF that lives inside a route-A/B
  // feeder (the chart keys the transition off the IAF, not the CIFP feeder
  // entry fix), include that feeder from the IAF leg onward, dropping the
  // pre-IAF enroute entry fix (e.g. selecting BUTLY drops CITAG on KFMY R05).
  std::unordered_map<std::string, int> feederFromSeq;
  for (const CifpLeg& leg : data.legs) {
    if (leg.kind != type || leg.procedureName != name) continue;
    if (leg.routeType != "A" && leg.routeType != "B") continue;
    if (leg.fixIdent != transition || approachLegRole(leg) != "iaf") continue;
    const std::string feeder = trim(leg.transition);
    if (feeder.empty()) continue;
    const auto it = feederFromSeq.find(feeder);
    if (it == feederFromSeq.end() || leg.sequence < it->second) {
      feederFromSeq[feeder] = leg.sequence;
    }
  }

  std::vector<CifpLeg> selected;
  for (const CifpLeg& leg : data.legs) {
    bool match = legMatchesSelection(leg, type, name, transition);
    // Same feeder name can recur on other approaches (e.g. a PINTS feeder on
    // both KFMY R05 and R13), so the feeder branch must stay within this
    // procedure or sibling-runway legs leak in (QUZSY from R13 PINTS).
    if (!match && leg.kind == type && leg.procedureName == name &&
        (leg.routeType == "A" || leg.routeType == "B")) {
      const auto it = feederFromSeq.find(trim(leg.transition));
      if (it != feederFromSeq.end() && leg.sequence >= it->second) match = true;
    }
    if (match) selected.push_back(leg);
  }
  std::sort(selected.begin(), selected.end(),
            [](const CifpLeg& a, const CifpLeg& b) {
              const int ra = approachRouteRank(a.routeType);
              const int rb = approachRouteRank(b.routeType);
              if (ra != rb) return ra < rb;
              return a.sequence < b.sequence;
            });

  double anchorLat = 0.0;
  double anchorLon = 0.0;
  float defaultCourse = 0.0f;
  bool haveAnchor = false;
  if (type == ProcedureType::Departure && isRunwayTransitionId(transition)) {
    const std::string rwKey = upperCopy(transition);
    double rwLat = 0.0;
    double rwLon = 0.0;
    if (lookup(rwKey, rwLat, rwLon, ctx)) {
      defaultCourse = runwayDepartureCourseDeg(data, rwKey);
      result.push_back(makeRunwayDepartureLeg(rwKey, rwLat, rwLon));
      anchorLat = rwLat;
      anchorLon = rwLon;
      haveAnchor = true;
    }
  }

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
    if (isCourseLegWithoutFix(leg)) {
      if (type == ProcedureType::Departure) {
        if (!haveAnchor && !result.empty()) {
          anchorLat = result.back().lat;
          anchorLon = result.back().lon;
          haveAnchor = true;
          if (result.back().legCourseDeg > 0.0f) {
            defaultCourse = result.back().legCourseDeg;
          }
        }
        if (!haveAnchor) continue;
        MapLeg ml = makeDepartureManeuverLeg(leg, anchorLat, anchorLon,
                                             defaultCourse);
        if (ml.id.empty()) continue;
        anchorLat = ml.lat;
        anchorLon = ml.lon;
        haveAnchor = true;
        if (ml.legCourseDeg > 0.0f) defaultCourse = ml.legCourseDeg;
        result.push_back(std::move(ml));
        continue;
      }
      if (!result.empty()) {
        applyMissedInitialFromCifpLeg(leg, result.back());
      }
      continue;
    }
    if (!looksLikeFix(leg.fixIdent)) continue;
    const std::string role = approachLegRole(leg);
    const bool holdLeg = isHoldTerminator(leg.pathTerminator);
    if (!added.insert(leg.fixIdent).second) {
      if (holdLeg) {
        for (MapLeg& existing : result) {
          if (existing.id != leg.fixIdent) continue;
          applyHoldFromCifpLeg(leg, existing);
          if (!role.empty()) existing.procedureRole = role;
          break;
        }
        continue;
      }
      // Feeder holds (route A/B, HF/HA) often share a fix with the final-segment
      // IAF (route R/I, IF). Keep one leg; the final-segment row carries IAF/FAF
      // roles and altitude constraints (e.g. KPGD RNAV R04 BULOW transition).
      if (isApproachFinalRouteType(leg.routeType) && !role.empty()) {
        for (MapLeg& existing : result) {
          if (existing.id != leg.fixIdent) continue;
          existing.procedureRole = role;
          applyArincAltitudeConstraint(leg, existing);
          if (leg.kind == ProcedureType::Approach &&
              leg.verticalAngleDeg > 0.0f) {
            existing.glidePathAngleDeg = leg.verticalAngleDeg;
          }
          break;
        }
      }
      continue;
    }

    double lat = 0.0;
    double lon = 0.0;
    if (!lookup(leg.fixIdent, lat, lon, ctx)) continue;

    MapLeg ml;
    ml.id = leg.fixIdent;
    ml.lat = lat;
    ml.lon = lon;
    ml.procedureRole = role;
    applyPathTerminatorFields(leg, ml);
    applyProcedureArcFromCifpLeg(leg, ml, lookup, ctx);
    // Published altitude restrictions apply to SID/STAR/approach legs alike; the
    // FPL ALT column reads altitudeConstraintFt from each MapLeg.
    applyArincAltitudeConstraint(leg, ml);
    if (leg.kind == ProcedureType::Approach && leg.verticalAngleDeg > 0.0f) {
      ml.glidePathAngleDeg = leg.verticalAngleDeg;
    }
    if (holdLeg) {
      applyHoldFromCifpLeg(leg, ml);
    }
    result.push_back(std::move(ml));
  }

  // Transition-segment holds (HF/HA on route A/B) apply at an IAF that also
  // appears on the final approach segment. When the pilot selects that IAF
  // directly instead of the named feeder (KCMI RNAV 04 "BOSTN iaf" vs "CMI"),
  // the feeder legs are omitted but the HILPT must still attach to BOSTN.
  for (const CifpLeg& leg : data.legs) {
    if (leg.kind != type || leg.procedureName != name) continue;
    if (!isHoldTerminator(leg.pathTerminator)) continue;
    if (!looksLikeFix(leg.fixIdent)) continue;
    for (MapLeg& existing : result) {
      if (existing.id != leg.fixIdent) continue;
      applyHoldFromCifpLeg(leg, existing);
      break;
    }
  }

  return result;
}

bool isNamedApproachTransition(const std::string& trans) {
  if (trans.empty()) return false;
  if (trans.size() >= 2 && trans[0] == 'R' && trans[1] == 'W') return false;
  return true;
}

bool isDirectIafRouteType(const std::string& routeType) {
  return routeType == "R" || routeType == "I" || routeType == "H" ||
         routeType == "L" || routeType == "V";
}

std::vector<ApproachTransitionOption> listApproachTransitions(
    const CifpAirportProcedures& data, const std::string& approachName) {
  std::unordered_set<std::string> transitionNames;
  std::unordered_set<std::string> iafNamesSet;
  // FAA CIFP names a route-A/B feeder after its enroute entry fix (e.g. KFMY
  // R05 "CITAG"/"PINTS" on V225/V579). Both the named feeder and the published
  // IAF inside it (BUTLY, AZOMY) are selectable transitions on the real
  // avionics, so offer both; only the IAF fix carries the "iaf" suffix.
  for (const CifpLeg& leg : data.legs) {
    if (leg.kind != ProcedureType::Approach || leg.procedureName != approachName) {
      continue;
    }
    const std::string trans = trim(leg.transition);
    if (leg.routeType == "A" || leg.routeType == "B") {
      if (isNamedApproachTransition(trans)) transitionNames.insert(trans);
      if (approachLegRole(leg) == "iaf" && looksLikeFix(leg.fixIdent)) {
        transitionNames.insert(leg.fixIdent);
        iafNamesSet.insert(leg.fixIdent);
      }
      continue;
    }
    // Final-segment IAF (e.g. ZEPIG on ILS I04) with no named feeder route.
    if (trans.empty() && isDirectIafRouteType(leg.routeType) &&
        approachLegRole(leg) == "iaf" && looksLikeFix(leg.fixIdent)) {
      transitionNames.insert(leg.fixIdent);
      iafNamesSet.insert(leg.fixIdent);
    }
  }

  std::vector<std::string> names(transitionNames.begin(), transitionNames.end());
  std::sort(names.begin(), names.end());

  std::vector<ApproachTransitionOption> options;
  options.reserve(names.size() + 1);
  options.push_back({"VECTORS", "VECTORS"});
  for (const std::string& name : names) {
    const bool isIaf = iafNamesSet.count(name) > 0;
    options.push_back({name, isIaf ? name + " iaf" : name});
  }
  return options;
}

}  // namespace avionics
