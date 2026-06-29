#include "avionics/FlightPlanPersistence.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <unordered_set>
#include <vector>

#include "avionics/FplRouteEdit.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {
namespace {

std::vector<std::string> splitPipeFields(const std::string& value) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  for (;;) {
    const std::size_t sep = value.find('|', start);
    if (sep == std::string::npos) {
      fields.push_back(value.substr(start));
      break;
    }
    fields.push_back(value.substr(start, sep - start));
    start = sep + 1;
  }
  return fields;
}

bool parseAltConstraintKind(const std::string& raw, AltConstraintType& out) {
  try {
    const int v = std::stoi(raw);
    if (v < static_cast<int>(AltConstraintType::None) ||
        v > static_cast<int>(AltConstraintType::AtOrBelow)) {
      return false;
    }
    out = static_cast<AltConstraintType>(v);
    return true;
  } catch (...) {
    return false;
  }
}

void applyPersistedLegAltitudeFields(const std::vector<std::string>& fields,
                                     std::size_t offset, MapLeg& legOut) {
  if (fields.size() < offset + 3) return;
  try {
    legOut.altitudeConstraintFt = std::stoi(fields[offset]);
    AltConstraintType kind = AltConstraintType::None;
    if (parseAltConstraintKind(fields[offset + 1], kind)) {
      legOut.altitudeConstraint = kind;
    }
    legOut.altitudeDesignated = fields[offset + 2] == "1";
  } catch (...) {
  }
}

bool legHasPersistedAltitude(const MapLeg& leg) {
  return leg.altitudeConstraintFt > 0 ||
         leg.altitudeConstraint != AltConstraintType::None ||
         leg.altitudeDesignated;
}

}  // namespace

bool parsePersistedFlightPlanLeg(const std::string& value, MapLeg& legOut) {
  const std::vector<std::string> fields = splitPipeFields(value);
  if (fields.size() < 3) return false;
  try {
    legOut = MapLeg{};
    legOut.id = fields[0];
    legOut.lat = std::stod(fields[1]);
    legOut.lon = std::stod(fields[2]);
    if (fields.size() == 3) {
      return !legOut.id.empty();
    }
    if (fields.size() == 4) {
      legOut.procedureRole = fields[3];
      return !legOut.id.empty();
    }
    if (fields.size() >= 7) {
      legOut.procedureRole = fields[3];
      applyPersistedLegAltitudeFields(fields, 4, legOut);
      // Optional trailing airway tag (Load Airway grouping), appended after the
      // VNAV altitude fields so older saves still parse.
      if (fields.size() >= 8) legOut.viaAirway = fields[7];
      return !legOut.id.empty();
    }
    return false;
  } catch (...) {
    return false;
  }
}

std::string formatPersistedFlightPlanLeg(const MapLeg& leg) {
  char buf[256];
  const bool hasAlt = legHasPersistedAltitude(leg);
  if (!leg.viaAirway.empty()) {
    // Full form with a trailing airway tag so Load Airway grouping survives a
    // restart (altitude fields default to 0 when the leg has no constraint).
    const int kind = static_cast<int>(leg.altitudeConstraint);
    std::snprintf(buf, sizeof(buf), "%s|%.6f|%.6f|%s|%d|%d|%d|%s",
                  leg.id.c_str(), leg.lat, leg.lon, leg.procedureRole.c_str(),
                  leg.altitudeConstraintFt, kind,
                  leg.altitudeDesignated ? 1 : 0, leg.viaAirway.c_str());
  } else if (!leg.procedureRole.empty() && !hasAlt) {
    std::snprintf(buf, sizeof(buf), "%s|%.6f|%.6f|%s", leg.id.c_str(), leg.lat,
                  leg.lon, leg.procedureRole.c_str());
  } else if (hasAlt) {
    const int kind = static_cast<int>(leg.altitudeConstraint);
    std::snprintf(buf, sizeof(buf), "%s|%.6f|%.6f|%s|%d|%d|%d",
                  leg.id.c_str(), leg.lat, leg.lon, leg.procedureRole.c_str(),
                  leg.altitudeConstraintFt, kind,
                  leg.altitudeDesignated ? 1 : 0);
  } else {
    std::snprintf(buf, sizeof(buf), "%s|%.6f|%.6f", leg.id.c_str(), leg.lat,
                  leg.lon);
  }
  return std::string(buf);
}

bool isFmsLatLonIdent(const std::string& id) {
  if (id.empty()) return false;
  const unsigned char first = static_cast<unsigned char>(id.front());
  if (first == '+') return true;
  if (first == '-' && id.size() > 1 &&
      std::isdigit(static_cast<unsigned char>(id[1]))) {
    return true;
  }
  return false;
}

void preserveFlightPlanIdents(std::vector<MapLeg>& plan,
                              const std::vector<MapLeg>& published) {
  if (published.empty()) return;
  constexpr double kLatLonMatchDeg = 0.03;
  for (std::size_t i = 0; i < plan.size() && i < published.size(); ++i) {
    MapLeg& leg = plan[i];
    const MapLeg& pub = published[static_cast<std::size_t>(i)];
    const bool samePosition =
        std::fabs(leg.lat - pub.lat) <= kLatLonMatchDeg &&
        std::fabs(leg.lon - pub.lon) <= kLatLonMatchDeg;
    // The sim FMS does not carry the airway tag; restore it from the last
    // published plan so the FPL "Airway -" grouping survives the round trip.
    if (leg.viaAirway.empty() && !pub.viaAirway.empty() && samePosition &&
        (leg.id == pub.id || isFmsLatLonIdent(leg.id))) {
      leg.viaAirway = pub.viaAirway;
    }
    if (leg.id == pub.id) continue;
    if (!isFmsLatLonIdent(leg.id) || isFmsLatLonIdent(pub.id)) continue;
    if (!samePosition) continue;
    leg.id = pub.id;
  }
}

bool flightPlanReadyForSimulator(const std::vector<MapLeg>& legs,
                                 bool destinationFilled,
                                 bool requireDestinationFilled) {
  if (legs.size() < 2) return false;
  return requireDestinationFilled ? destinationFilled : true;
}

bool findLegSequenceInPlan(const std::vector<MapLeg>& plan,
                           const std::vector<MapLeg>& sequence, int& startOut) {
  if (sequence.empty() || plan.size() < sequence.size()) return false;
  int found = -1;
  for (std::size_t i = 0; i + sequence.size() <= plan.size(); ++i) {
    bool match = true;
    for (std::size_t j = 0; j < sequence.size(); ++j) {
      if (plan[i + j].id != sequence[j].id) {
        match = false;
        break;
      }
    }
    if (match) found = static_cast<int>(i);
  }
  if (found < 0) return false;
  startOut = found;
  return true;
}

InferredProcedureBlock inferProcedureBlockInPlan(const std::vector<MapLeg>& legs,
                                                 int minStart) {
  InferredProcedureBlock out;
  const int scanFrom = std::max(0, minStart);
  for (int i = scanFrom; i < static_cast<int>(legs.size()); ++i) {
    if (!legs[static_cast<std::size_t>(i)].procedureRole.empty()) {
      out.start = i;
      break;
    }
  }
  if (out.start < 0) return out;
  // Untagged approach fixes may precede the first tagged IAF/FAF (e.g. WADOR
  // before AMXUQ when only GRRDN carries procedureRole after a sim re-import).
  // Walk back through those intermediates but stop at the destination airport
  // (KJAX) — it belongs in the approach header, not the approach leg block.
  // Never walk back past `minStart`, which marks the end of an earlier
  // procedure block (e.g. a loaded arrival/STAR) whose fixes are not approach
  // legs.
  while (out.start > scanFrom &&
         legs[static_cast<std::size_t>(out.start - 1)].procedureRole.empty()) {
    if (isAirportIdent(legs[static_cast<std::size_t>(out.start - 1)].id)) {
      break;
    }
    const std::string& taggedRole =
        legs[static_cast<std::size_t>(out.start)].procedureRole;
    if (taggedRole.empty() || taggedRole == "iaf" || taggedRole == "faf") {
      --out.start;
      continue;
    }
    break;
  }
  // CIFP only tags IAF/FAF/MAP legs with procedureRole; the intermediate
  // fixes on a loaded approach are still part of the same tail block.
  out.count = static_cast<int>(legs.size()) - out.start;
  if (out.count <= 0) {
    out.start = -1;
  }
  return out;
}

int destinationAirportLegBeforeApproach(const std::vector<MapLeg>& legs,
                                          int minStart) {
  const int approachStart = fplApproachInferenceFloor(legs, minStart);
  if (approachStart <= 0 ||
      approachStart >= static_cast<int>(legs.size())) {
    return -1;
  }
  const int destIdx = approachStart - 1;
  if (destIdx < minStart) return -1;
  if (!isAirportIdent(legs[static_cast<std::size_t>(destIdx)].id)) return -1;
  return destIdx;
}

std::vector<MapLeg> mapRouteDisplayLegs(const std::vector<MapLeg>& legs,
                                          int minStart) {
  const int destIdx = destinationAirportLegBeforeApproach(legs, minStart);
  if (destIdx < 0) return legs;
  std::vector<MapLeg> out;
  out.reserve(legs.size() - 1);
  out.insert(out.end(), legs.begin(),
             legs.begin() + static_cast<std::size_t>(destIdx));
  out.insert(out.end(), legs.begin() + static_cast<std::size_t>(destIdx + 1),
             legs.end());
  return out;
}

int mapRouteDisplayLegIndex(const std::vector<MapLeg>& legs, int planLegIndex,
                              int minStart) {
  if (planLegIndex < 0 ||
      planLegIndex >= static_cast<int>(legs.size())) {
    return -1;
  }
  const int destIdx = destinationAirportLegBeforeApproach(legs, minStart);
  if (destIdx < 0) return planLegIndex;
  if (planLegIndex == destIdx) return -1;
  if (planLegIndex > destIdx) return planLegIndex - 1;
  return planLegIndex;
}

int mapRoutePlanLegIndex(const std::vector<MapLeg>& legs, int routeLegIndex,
                           int minStart) {
  if (routeLegIndex < 0) return -1;
  const int destIdx = destinationAirportLegBeforeApproach(legs, minStart);
  if (destIdx < 0) return routeLegIndex;
  if (routeLegIndex >= destIdx) return routeLegIndex + 1;
  return routeLegIndex;
}

int fplApproachInferenceFloor(const std::vector<MapLeg>& legs, int arrivalEnd) {
  int floor = std::max(0, arrivalEnd);
  // The destination airport separates the STAR (before it) from the approach
  // (after it). Find the last airport ident that is followed by at least one
  // procedure-role leg: that airport is the destination, and the approach can
  // only begin after it. Requiring a trailing role leg avoids treating a 4-char
  // missed-approach fix (which isAirportIdent() also matches) as the boundary.
  const int n = static_cast<int>(legs.size());
  for (int i = n - 1; i >= 0; --i) {
    if (!isAirportIdent(legs[static_cast<std::size_t>(i)].id)) continue;
    bool roleAfter = false;
    for (int j = i + 1; j < n; ++j) {
      if (!legs[static_cast<std::size_t>(j)].procedureRole.empty()) {
        roleAfter = true;
        break;
      }
    }
    if (roleAfter) {
      floor = std::max(floor, i + 1);
      break;
    }
  }
  return floor;
}

void mergeProcedureLegFields(MapLeg& dst, const MapLeg& src) {
  dst.procedureRole = src.procedureRole;
  dst.hold = src.hold;
  dst.pathTerminator = src.pathTerminator;
  if (src.legCourseDeg > 0.0f) {
    dst.legCourseDeg = src.legCourseDeg;
  }
  if (src.missedInitial.active) {
    dst.missedInitial = src.missedInitial;
  }
  if (src.altitudeConstraintFt > 0) {
    dst.altitudeConstraintFt = src.altitudeConstraintFt;
    dst.altitudeConstraint = src.altitudeConstraint;
  }
  if (src.glidePathAngleDeg > 0.0f) {
    dst.glidePathAngleDeg = src.glidePathAngleDeg;
  }
}

void mergeProcedureLegMetadata(std::vector<MapLeg>& plan, int start,
                               const std::vector<MapLeg>& procedureLegs) {
  for (std::size_t i = 0; i < procedureLegs.size(); ++i) {
    const std::size_t idx = static_cast<std::size_t>(start) + i;
    if (idx >= plan.size()) break;
    if (plan[idx].id != procedureLegs[i].id) continue;
    mergeProcedureLegFields(plan[idx], procedureLegs[i]);
  }
}

TerminalProcedureMetadataRestore restoreTerminalProcedureMetadata(
    const NavFeatureSource* nav, const PersistedLoadedApproach& meta,
    std::vector<MapLeg>& plan, int blockStart, int blockCount) {
  TerminalProcedureMetadataRestore result;
  if (!meta.active || meta.name.empty() || meta.airportIcao.empty()) {
    result.stopRetrying = true;
    return result;
  }
  if (nav == nullptr || !nav->ready() || plan.empty()) {
    return result;
  }
  if (blockStart < 0 || blockCount <= 0 ||
      blockStart + blockCount > static_cast<int>(plan.size())) {
    result.stopRetrying = true;
    return result;
  }

  const std::vector<MapLeg> expanded =
      nav->expandProcedure(meta.airportIcao, meta.type, meta.name,
                           meta.transition);
  if (expanded.empty()) return result;

  // Fix idents the procedure claims. The importer's via_airway heuristic can
  // exclude a transition-entry/exit fix that CIFP includes (it carries the
  // procedure's altitude restriction), so grow the block outward across any
  // contiguous plan leg whose ident the procedure also names.
  std::unordered_set<std::string> procedureIds;
  for (const MapLeg& src : expanded) procedureIds.insert(src.id);

  const int planSize = static_cast<int>(plan.size());
  int start = blockStart;
  int end = blockStart + blockCount;  // one past the last block leg
  while (start - 1 >= 0) {
    const MapLeg& prev = plan[static_cast<std::size_t>(start - 1)];
    if (isAirportIdent(prev.id)) break;
    if (procedureIds.find(prev.id) == procedureIds.end()) break;
    --start;
  }
  while (end < planSize) {
    const MapLeg& next = plan[static_cast<std::size_t>(end)];
    if (isAirportIdent(next.id)) break;
    if (procedureIds.find(next.id) == procedureIds.end()) break;
    ++end;
  }

  int mergedCount = 0;
  for (const MapLeg& src : expanded) {
    for (int i = start; i < end; ++i) {
      if (plan[static_cast<std::size_t>(i)].id != src.id) continue;
      mergeProcedureLegFields(plan[static_cast<std::size_t>(i)], src);
      ++mergedCount;
      break;
    }
  }
  if (mergedCount == 0) {
    result.stopRetrying = true;
    return result;
  }
  result.merged = true;
  if (start != blockStart || end != blockStart + blockCount) {
    result.correctedStart = start;
    result.correctedCount = end - start;
  }
  return result;
}

void removeLoadedApproachLegs(std::vector<MapLeg>& legs, int approachStart,
                              int approachCount) {
  if (approachCount <= 0 || approachStart < 0) return;
  const int end = approachStart + approachCount;
  if (end > static_cast<int>(legs.size())) return;
  legs.erase(legs.begin() + approachStart, legs.begin() + end);
}

bool collapseDuplicateApproachTail(std::vector<MapLeg>& legs) {
  bool changed = false;
  for (;;) {
    const int n = static_cast<int>(legs.size());
    bool removed = false;
    // Largest repeated tail block first so a fully duplicated approach collapses
    // in one step rather than fix-by-fix.
    for (int block = n / 2; block >= 2; --block) {
      bool idsMatch = true;
      for (int i = 0; i < block; ++i) {
        if (legs[static_cast<std::size_t>(n - 2 * block + i)].id !=
            legs[static_cast<std::size_t>(n - block + i)].id) {
          idsMatch = false;
          break;
        }
      }
      if (!idsMatch) continue;
      // Only collapse when the repeated tail is an approach (carries procedure
      // roles); enroute fixes can legitimately repeat (e.g. holds, airways).
      bool tailHasProcedureRole = false;
      for (int i = n - block; i < n; ++i) {
        if (!legs[static_cast<std::size_t>(i)].procedureRole.empty()) {
          tailHasProcedureRole = true;
          break;
        }
      }
      if (!tailHasProcedureRole) continue;
      legs.erase(legs.begin() + (n - block), legs.end());
      removed = true;
      break;
    }
    if (!removed) break;
    changed = true;
  }
  return changed;
}

MapProcedure mapProcedureFromPersisted(const PersistedLoadedApproach& saved) {
  MapProcedure proc;
  proc.type = saved.type;
  proc.name = saved.name;
  proc.transition = saved.transition;
  proc.runway = saved.runway;
  proc.approachKind = saved.approachKind;
  proc.levelOfService = saved.levelOfService;
  return proc;
}

PersistedLoadedApproach persistedFromMapProcedure(const MapProcedure& proc,
                                                    const std::string& icao) {
  PersistedLoadedApproach out;
  out.active = !proc.name.empty();
  out.airportIcao = icao;
  out.type = proc.type;
  out.name = proc.name;
  out.transition = proc.transition;
  out.runway = proc.runway;
  out.approachKind = proc.approachKind;
  out.levelOfService = proc.levelOfService;
  return out;
}

namespace {

std::string upperCopy(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
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

bool containsInsensitive(const std::string& haystack, const std::string& needle) {
  if (needle.empty()) return false;
  const std::string h = upperCopy(haystack);
  const std::string n = upperCopy(needle);
  return h.find(n) != std::string::npos;
}

std::string approachTypeFromLetter(char kind) {
  switch (kind) {
    case 'I':
      return "ILS";
    case 'L':
      return "LOC";
    case 'R':
      return "RNAV";
    case 'V':
      return "VOR";
    case 'N':
      return "NDB";
    default:
      return {};
  }
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
    const std::string type = approachTypeFromLetter(upper[0]);
    if (type.empty()) return false;
    const std::string tail = upper.substr(1);
    if (!isRunwayToken(tail)) return false;
    typeOut = type;
    runwayOut = tail;
    return true;
  }
  return false;
}

bool isRnavGps(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevType == "RNAV";
  }
  const std::string prefix = upperCopy(proc.name).substr(0, 4);
  return prefix == "RNAV" || proc.approachKind == "R";
}

std::string approachSuffix(const MapProcedure& proc) {
  const std::string los = upperCopy(proc.levelOfService);
  if (los == "LPV") return " LPV";
  if (los == "LNAV/VNAV") return " LNAV/VNAV";
  if (los == "LNAV") return " LNAV";
  if (containsInsensitive(proc.name, "LPV")) return " LPV";
  if (containsInsensitive(proc.name, "LNAV")) return " LNAV";
  if (containsInsensitive(proc.transition, "LPV")) return " LPV";
  if (containsInsensitive(proc.transition, "LNAV")) return " LNAV";
  return {};
}

std::string approachTypePrefix(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevType;
  }

  const std::string name = upperCopy(proc.name);
  if (name.size() >= 3) {
    const std::string prefix = name.substr(0, 3);
    if (prefix == "ILS" || prefix == "LOC" || prefix == "VOR" ||
        prefix == "NDB" || prefix == "RNA") {
      return prefix == "RNA" ? "RNAV" : prefix;
    }
    if (name.size() >= 6 && name.substr(0, 6) == "VISUAL") return "VISUAL";
  }
  if (!proc.approachKind.empty()) {
    const std::string fromKind = approachTypeFromLetter(proc.approachKind[0]);
    if (!fromKind.empty()) return fromKind;
  }
  return proc.name;
}

}  // namespace

std::string formatApproachFplHeaderLabel(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    std::string label = abbrevType == "RNAV" ? std::string("RNAV_GPS") : abbrevType;
    if (!abbrevRunway.empty()) label += " " + abbrevRunway;
    label += approachSuffix(proc);
    return label;
  }

  std::string runway = proc.runway;
  if (isRnavGps(proc)) {
    std::string label = "RNAV_GPS";
    if (!runway.empty()) label += " " + runway;
    label += approachSuffix(proc);
    return label;
  }
  std::string label = approachTypePrefix(proc);
  if (!runway.empty()) {
    label += " " + runway;
  }
  label += approachSuffix(proc);
  return label;
}

bool inferApproachMetadataFromLegs(const std::vector<MapLeg>& legs,
                                   int approachStart,
                                   PersistedLoadedApproach& meta) {
  if (approachStart < 0 ||
      approachStart >= static_cast<int>(legs.size())) {
    return false;
  }
  meta.active = true;
  meta.type = ProcedureType::Approach;
  for (int i = approachStart; i < static_cast<int>(legs.size()); ++i) {
    const MapLeg& leg = legs[static_cast<std::size_t>(i)];
    const std::string& id = leg.id;
    if (id.size() >= 3 && (id[0] == 'R' || id[0] == 'r') && id[1] == 'W' &&
        leg.procedureRole == "mapt") {
      meta.runway = id.substr(2);
      meta.name = "R" + meta.runway;
    }
    if (leg.procedureRole == "mahp") {
      meta.levelOfService = "LPV";
      meta.approachKind = "LPV";
    }
  }
  std::string abbrevType;
  std::string abbrevRunway;
  if (!meta.name.empty() &&
      decodeAbbreviatedApproachName(meta.name, abbrevType, abbrevRunway)) {
    if (meta.runway.empty()) meta.runway = abbrevRunway;
  }
  return !meta.name.empty() || !meta.runway.empty();
}

std::string inferApproachAirportFromProcedureLegs(
    const NavFeatureSource* nav, const std::vector<MapLeg>& legs,
    int approachStart, int approachCount) {
  if (nav == nullptr || !nav->ready() || approachCount <= 0 ||
      approachStart < 0 ||
      approachStart + approachCount > static_cast<int>(legs.size())) {
    return {};
  }
  const MapLeg* ref = nullptr;
  for (int i = approachStart; i < approachStart + approachCount; ++i) {
    if (legs[static_cast<std::size_t>(i)].procedureRole == "mapt") {
      ref = &legs[static_cast<std::size_t>(i)];
      break;
    }
  }
  if (ref == nullptr) {
    ref = &legs[static_cast<std::size_t>(approachStart + approachCount - 1)];
  }
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  const std::vector<MapFeature> features =
      nav->nearby(ref->lat, ref->lon, 15.0f, 16);
  std::string best;
  double bestDist2 = 1e30;
  const double cosLat = std::cos(ref->lat * kDegToRad);
  for (const MapFeature& f : features) {
    if (f.type != MapFeatureType::Airport || !isAirportIdent(f.id)) continue;
    const double dLat = f.lat - ref->lat;
    const double dLon = (f.lon - ref->lon) * cosLat;
    const double d2 = dLat * dLat + dLon * dLon;
    if (d2 < bestDist2) {
      bestDist2 = d2;
      best = f.id;
    }
  }
  return best;
}

void enrichPersistedFlightPlanFromLegs(PersistedFlightPlan& plan) {
  if (!plan.active || plan.legs.empty()) return;
  if (collapseDuplicateApproachTail(plan.legs)) {
    // The stored grouping spanned both copies of the duplicated approach; force
    // a re-inference below from the cleaned (single-copy) leg list.
    plan.approachLegStart = -1;
    plan.approachLegCount = 0;
  }
  // The approach is the procedure tail after any loaded arrival/STAR block; its
  // fixes are not approach legs even when they carry procedureRole tags.
  const int arrivalEnd = plan.arrivalLegCount > 0
                             ? plan.arrivalLegStart + plan.arrivalLegCount
                             : 0;
  InferredProcedureBlock block =
      inferProcedureBlockInPlan(plan.legs, arrivalEnd);
  if (!block.valid()) return;
  if (plan.approachLegCount <= 0 || plan.approachLegStart < arrivalEnd) {
    plan.approachLegStart = block.start;
    plan.approachLegCount = block.count;
  }
  if (plan.approachMeta.name.empty()) {
    inferApproachMetadataFromLegs(plan.legs, plan.approachLegStart,
                                  plan.approachMeta);
  }
  if (plan.approachAirportIcao.empty() &&
      !plan.approachMeta.airportIcao.empty()) {
    plan.approachAirportIcao = plan.approachMeta.airportIcao;
  }
  if (!plan.destinationFilled && plan.approachLegCount > 0) {
    plan.destinationFilled = true;
  }
}

}  // namespace avionics
