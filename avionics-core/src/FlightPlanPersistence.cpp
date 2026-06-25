#include "avionics/FlightPlanPersistence.h"

#include <cmath>
#include <cctype>

#include "avionics/FplRouteEdit.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

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
    if (leg.id == pub.id) continue;
    if (!isFmsLatLonIdent(leg.id) || isFmsLatLonIdent(pub.id)) continue;
    if (std::fabs(leg.lat - pub.lat) > kLatLonMatchDeg ||
        std::fabs(leg.lon - pub.lon) > kLatLonMatchDeg) {
      continue;
    }
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

InferredProcedureBlock inferProcedureBlockInPlan(const std::vector<MapLeg>& legs) {
  InferredProcedureBlock out;
  for (int i = 0; i < static_cast<int>(legs.size()); ++i) {
    if (!legs[static_cast<std::size_t>(i)].procedureRole.empty()) {
      out.start = i;
      break;
    }
  }
  if (out.start < 0) return out;
  // Untagged feeder/hold fixes may sit immediately before the first tagged IAF/FAF
  // when a route A/B hold duplicates the final-segment IAF fix (BULOW on R04).
  while (out.start > 0 &&
         legs[static_cast<std::size_t>(out.start - 1)].procedureRole.empty()) {
    const std::string& taggedRole =
        legs[static_cast<std::size_t>(out.start)].procedureRole;
    if (taggedRole != "iaf" && taggedRole != "faf") break;
    --out.start;
  }
  // CIFP only tags IAF/FAF/MAP legs with procedureRole; the intermediate
  // fixes on a loaded approach are still part of the same tail block.
  out.count = static_cast<int>(legs.size()) - out.start;
  if (out.count <= 0) {
    out.start = -1;
  }
  return out;
}

void mergeProcedureLegMetadata(std::vector<MapLeg>& plan, int start,
                               const std::vector<MapLeg>& procedureLegs) {
  for (std::size_t i = 0; i < procedureLegs.size(); ++i) {
    const std::size_t idx = static_cast<std::size_t>(start) + i;
    if (idx >= plan.size()) break;
    if (plan[idx].id != procedureLegs[i].id) continue;
    plan[idx].procedureRole = procedureLegs[i].procedureRole;
    plan[idx].hold = procedureLegs[i].hold;
    plan[idx].pathTerminator = procedureLegs[i].pathTerminator;
    if (procedureLegs[i].legCourseDeg > 0.0f) {
      plan[idx].legCourseDeg = procedureLegs[i].legCourseDeg;
    }
    if (procedureLegs[i].missedInitial.active) {
      plan[idx].missedInitial = procedureLegs[i].missedInitial;
    }
    if (procedureLegs[i].altitudeConstraintFt > 0) {
      plan[idx].altitudeConstraintFt = procedureLegs[i].altitudeConstraintFt;
      plan[idx].altitudeConstraint = procedureLegs[i].altitudeConstraint;
    }
    if (procedureLegs[i].glidePathAngleDeg > 0.0f) {
      plan[idx].glidePathAngleDeg = procedureLegs[i].glidePathAngleDeg;
    }
  }
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
  InferredProcedureBlock block = inferProcedureBlockInPlan(plan.legs);
  if (!block.valid()) return;
  if (plan.approachLegCount <= 0) {
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
