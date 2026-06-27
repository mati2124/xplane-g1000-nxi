#include "avionics/ProcedureSupport.h"

#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include <cctype>
#include <cmath>

namespace avionics {
namespace {

std::string upperCopy(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
}

bool containsInsensitive(const std::string& haystack, const std::string& needle) {
  if (needle.empty()) return false;
  const std::string h = upperCopy(haystack);
  const std::string n = upperCopy(needle);
  return h.find(n) != std::string::npos;
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
    case 'D':
      return "VOR";
    default:
      return {};
  }
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
    const std::string type = approachTypeFromLetter(upper[0]);
    if (type.empty()) return false;
    const std::string tail = upper.substr(1);
    if (!isRunwayToken(tail) && !runwaySuffixFromVariantName(tail, runwayOut)) {
      return false;
    }
    typeOut = type;
    if (runwayOut.empty()) runwayOut = tail;
    return true;
  }
  return false;
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

bool isRnavGps(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevType == "RNAV";
  }
  const std::string prefix = approachTypePrefix(proc);
  return prefix == "RNAV" || proc.approachKind == "R";
}

std::string procedureRunwayLabel(const MapProcedure& proc) {
  if (!proc.runway.empty()) return proc.runway;
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevRunway;
  }
  const std::string trans = upperCopy(proc.transition);
  if (trans.size() > 2 && trans.compare(0, 2, "RW") == 0) {
    return proc.transition.substr(2);
  }
  return {};
}

bool runwayLabelsMatch(const std::string& a, const std::string& b) {
  if (a.empty() || b.empty()) return false;
  if (a == b) return true;
  if (a.size() < b.size() && b.compare(0, a.size(), a) == 0) return true;
  if (b.size() < a.size() && a.compare(0, b.size(), b) == 0) return true;
  return false;
}

}  // namespace

void insertProcedureLegs(ProcedureType type, std::vector<MapLeg>& fplLegs,
                         const std::vector<MapLeg>& legs) {
  if (legs.empty()) return;
  int row = 0;
  if (type == ProcedureType::Departure) {
    row = fplLegs.empty() ? 0 : 1;
  } else if (type == ProcedureType::Arrival) {
    row = std::max(0, static_cast<int>(fplLegs.size()) - 1);
  } else {
    row = static_cast<int>(fplLegs.size());
    if (!fplLegs.empty() &&
        fplLegIdentsEqual(fplLegs.back().id, legs.front().id)) {
      fplLegs.pop_back();
      row = static_cast<int>(fplLegs.size());
    }
  }
  fplLegs.insert(fplLegs.begin() + row, legs.begin(), legs.end());
}

MapProcedure findProcedureInCatalog(ProcedureType type, const std::string& name,
                                    const std::string& transition,
                                    const std::vector<MapProcedure>& catalog) {
  for (const MapProcedure& proc : catalog) {
    if (proc.type == type && proc.name == name &&
        proc.transition == transition) {
      return proc;
    }
  }
  MapProcedure fallback;
  fallback.type = type;
  fallback.name = name;
  fallback.transition = transition;
  return fallback;
}

std::string formatApproachProcedureLabel(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    std::string label =
        abbrevType == "RNAV" ? std::string("RNAV_GPS") : abbrevType;
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
  if (!runway.empty()) label += " " + runway;
  label += approachSuffix(proc);
  return label;
}

bool procedureUsesNavPrimaryFrequency(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevType == "ILS" || abbrevType == "LOC" || abbrevType == "VOR" ||
           abbrevType == "NDB";
  }
  if (!proc.approachKind.empty()) {
    const char kind = upperCopy(proc.approachKind)[0];
    return kind == 'I' || kind == 'L' || kind == 'V' || kind == 'N';
  }
  const std::string name = upperCopy(proc.name);
  if (name.size() >= 3) {
    const std::string prefix = name.substr(0, 3);
    return prefix == "ILS" || prefix == "LOC" || prefix == "VOR" ||
           prefix == "NDB";
  }
  return false;
}

ProcPrimaryNav resolveProcPrimaryNav(const NavFeatureSource* nav,
                                     const MapData* map,
                                     const MapFeature& airport,
                                     const MapProcedure& proc) {
  ProcPrimaryNav out;
  if (!procedureUsesNavPrimaryFrequency(proc)) return out;

  const std::string runway = procedureRunwayLabel(proc);
  std::string abbrevType;
  std::string abbrevRunway;
  const bool abbreviated =
      decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway);

  if (proc.frequencyMhz > 0.0f) {
    out.frequency = proc.frequencyMhz;
    if (!abbreviated) out.ident = proc.name;
  }

  if (nav != nullptr && nav->ready() && !airport.id.empty()) {
    const std::string icao = airport.id;
    const bool ilsLike =
        (abbreviated && (abbrevType == "ILS" || abbrevType == "LOC")) ||
        proc.approachKind == "I" || proc.approachKind == "L";
    if (ilsLike) {
      for (const MapApproach& ap : nav->approachesForAirport(icao)) {
        if (!runway.empty() && runwayLabelsMatch(runway, ap.runway)) {
          if (ap.frequencyMhz > 0.0f) out.frequency = ap.frequencyMhz;
          if (!ap.ident.empty()) out.ident = ap.ident;
          out.isNdb = false;
          return out;
        }
        if (ap.ident == proc.name) {
          if (ap.frequencyMhz > 0.0f) out.frequency = ap.frequencyMhz;
          out.ident = ap.ident;
          out.isNdb = false;
          return out;
        }
      }
      if (out.frequency <= 0.0f && icao.size() == 4 && icao[0] == 'K') {
        const std::string locIdent = std::string("I") + icao.substr(1);
        for (const MapFeature& match : nav->lookupIdent(locIdent, 8)) {
          if (match.type == MapFeatureType::Vor && match.frequency > 0.0f) {
            out.frequency = match.frequency;
            out.ident = match.id;
            out.isNdb = false;
            return out;
          }
        }
      }
    }

    if (!abbreviated) {
      for (const MapFeature& match : nav->lookupIdent(proc.name, 8)) {
        if (match.type == MapFeatureType::Vor && match.frequency > 0.0f) {
          out.frequency = match.frequency;
          out.ident = match.id;
          out.isNdb = false;
          return out;
        }
        if (match.type == MapFeatureType::Ndb && match.frequency > 0.0f) {
          out.frequency = match.frequency;
          out.ident = match.id;
          out.isNdb = true;
          return out;
        }
      }
    }

    if (abbreviated && (abbrevType == "VOR" || abbrevType == "NDB") &&
        map != nullptr && airport.lat != 0.0 && airport.lon != 0.0) {
      const MapFeatureType want =
          abbrevType == "NDB" ? MapFeatureType::Ndb : MapFeatureType::Vor;
      const MapFeature* best = nullptr;
      double bestSq = 0.0;
      bool haveBest = false;
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      const double cosLat = std::max(0.05, std::cos(airport.lat * kDegToRad));
      for (const MapFeature& f : map->features) {
        if (f.type != want || f.frequency <= 0.0f) continue;
        const double dLat = f.lat - airport.lat;
        const double dLon = (f.lon - airport.lon) * cosLat;
        const double dSq = dLat * dLat + dLon * dLon;
        if (!haveBest || dSq < bestSq) {
          best = &f;
          bestSq = dSq;
          haveBest = true;
        }
      }
      if (best != nullptr) {
        out.frequency = best->frequency;
        out.ident = best->id;
        out.isNdb = best->type == MapFeatureType::Ndb;
        return out;
      }
    }
  }

  return out;
}

std::string defaultProcedureTransition(
    const std::vector<std::string>& transitions) {
  for (const std::string& t : transitions) {
    if (upperCopy(t) == "VECTORS") return t;
  }
  return transitions.empty() ? std::string() : transitions.front();
}

std::vector<std::string> procedureTransitionIds(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name) {
  std::vector<std::string> transitions;
  if (nav == nullptr || !nav->ready() || icao.empty()) return transitions;

  if (type == ProcedureType::Approach) {
    const std::vector<ApproachTransitionOption> options =
        nav->approachTransitionsFor(icao, name);
    if (!options.empty()) {
      transitions.reserve(options.size());
      for (const ApproachTransitionOption& opt : options) {
        transitions.push_back(opt.id);
      }
      return transitions;
    }
  }

  std::string abbrevType;
  std::string abbrevRunway;
  const bool namedRunwayApproach =
      decodeAbbreviatedApproachName(name, abbrevType, abbrevRunway);
  for (const MapProcedure& proc : nav->proceduresForAirport(icao, type)) {
    if (proc.name != name) continue;
    if (type == ProcedureType::Approach && namedRunwayApproach &&
        proc.transition.size() >= 2 && proc.transition[0] == 'R' &&
        proc.transition[1] == 'W') {
      continue;
    }
    if (std::find(transitions.begin(), transitions.end(), proc.transition) ==
        transitions.end()) {
      transitions.push_back(proc.transition);
    }
  }
  std::sort(transitions.begin(), transitions.end());
  if (type == ProcedureType::Approach &&
      std::find(transitions.begin(), transitions.end(), "VECTORS") ==
          transitions.end()) {
    transitions.insert(transitions.begin(), "VECTORS");
  }
  return transitions;
}

std::vector<std::string> procedureTransitionLabels(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name) {
  if (nav != nullptr && nav->ready() && type == ProcedureType::Approach) {
    const std::vector<ApproachTransitionOption> options =
        nav->approachTransitionsFor(icao, name);
    if (!options.empty()) {
      std::vector<std::string> labels;
      labels.reserve(options.size());
      for (const ApproachTransitionOption& opt : options) {
        labels.push_back(opt.display);
      }
      return labels;
    }
  }
  return procedureTransitionIds(nav, icao, type, name);
}

namespace {

bool isRunwayTransitionId(const std::string& transition) {
  return transition.size() >= 3 && (transition[0] == 'R' || transition[0] == 'r') &&
         (transition[1] == 'W' || transition[1] == 'w');
}

}  // namespace

std::vector<std::string> procedureEnrouteTransitions(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name) {
  std::vector<std::string> enroute;
  for (const std::string& t : procedureTransitionIds(nav, icao, type, name)) {
    if (t.empty() || isRunwayTransitionId(t)) continue;
    enroute.push_back(t);
  }
  return enroute;
}

std::vector<std::string> procedureRunwayOptions(const NavFeatureSource* nav,
                                                const std::string& icao,
                                                ProcedureType type,
                                                const std::string& name) {
  std::vector<std::string> runways;
  for (const std::string& t : procedureTransitionIds(nav, icao, type, name)) {
    if (!isRunwayTransitionId(t)) continue;
    if (std::find(runways.begin(), runways.end(), t) == runways.end()) {
      runways.push_back(t);
    }
  }
  if (runways.empty()) runways.push_back(kProcRunwayAll);
  return runways;
}

std::vector<MapLeg> expandArrivalDepartureProcedure(
    const NavFeatureSource* nav, const std::string& icao, ProcedureType type,
    const std::string& name, const std::string& enrouteTransition,
    const std::string& runwayLabel) {
  if (nav == nullptr || !nav->ready() || name.empty()) return {};

  const bool hasRunway =
      !runwayLabel.empty() && runwayLabel != kProcRunwayAll;
  const std::string runwayTransition = hasRunway ? runwayLabel : std::string();
  const std::string& enroute = enrouteTransition;

  // Arrival sequences enroute -> body -> runway; Departure runway -> body ->
  // enroute. The leg common to both expansions (the procedure body, route type
  // 5) is de-duplicated by fix ident when merging the two segments.
  const std::string primary =
      type == ProcedureType::Departure ? runwayTransition : enroute;
  const std::string secondary =
      type == ProcedureType::Departure ? enroute : runwayTransition;

  auto expand = [&](const std::string& transition) {
    if (transition.empty()) return std::vector<MapLeg>{};
    return nav->expandProcedure(icao, type, name, transition);
  };

  std::vector<MapLeg> legs = expand(primary);
  if (legs.empty()) legs = expand(secondary);
  if (legs.empty()) {
    // Neither axis selected: fall back to whatever single transition exists.
    const std::vector<std::string> all =
        procedureTransitionIds(nav, icao, type, name);
    if (!all.empty()) legs = expand(all.front());
    return legs;
  }

  const std::vector<MapLeg> rest =
      (primary == secondary || secondary.empty()) ? std::vector<MapLeg>{}
                                                  : expand(secondary);
  for (const MapLeg& leg : rest) {
    bool present = false;
    for (const MapLeg& have : legs) {
      if (have.id == leg.id) {
        present = true;
        break;
      }
    }
    if (!present) legs.push_back(leg);
  }
  return legs;
}

std::vector<std::string> uniqueProcedureNames(
    const std::vector<MapProcedure>& catalog) {
  std::vector<std::string> names;
  for (const MapProcedure& proc : catalog) {
    if (std::find(names.begin(), names.end(), proc.name) == names.end()) {
      names.push_back(proc.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace avionics
