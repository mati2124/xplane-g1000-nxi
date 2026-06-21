#include <algorithm>
#include <cctype>

#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8). Mirrors the real
// unit's Procedures menu (Working Title PFDProc): a top-level menu of
// approach-activation and procedure-selection items, then a selection
// sub-window that loads the chosen departure / arrival / approach into the
// active flight plan (reusing the PFD flight-plan working copy and its
// shell-publish path). The selection/leg-expansion logic mirrors the MFD's
// MfdControllerProcedures.
namespace avionics {
namespace {

const std::string kEmptyString;

bool isAirportIdent(const std::string& id) {
  if (id.size() != 4) return false;
  for (char c : id) {
    if (c < 'A' || c > 'Z') return false;
  }
  return true;
}

// Airport ICAO for procedure pickers: enroute/destination airport, not the
// active approach fix (CITAG) or other 5-letter waypoint idents.
std::string airportIcaoBeforeIndex(const std::vector<MapLeg>& legs, int before) {
  for (int i = std::min(before, static_cast<int>(legs.size())) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

std::string directToAirportIcao(const MapData* map) {
  if (map == nullptr || !map->directToActive) return {};
  return isAirportIdent(map->directTo.id) ? map->directTo.id : std::string();
}

std::string lastAirportInPlan(const std::vector<MapLeg>& legs) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

// Insert a procedure's legs at the conventional place in the plan: a departure
// after the origin, an arrival before the destination, an approach at the end
// (mirrors MfdControllerProcedures::insertProcedureLegs).
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
  }
  fplLegs.insert(fplLegs.begin() + row, legs.begin(), legs.end());
}

MapProcedure findProcedure(ProcedureType type, const std::string& name,
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

// CIFP catalog names are often abbreviated: I05 = ILS runway 05, R13 = RNAV, etc.
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
    const std::string fromKind =
        approachTypeFromLetter(proc.approachKind[0]);
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

struct ProcPrimaryNav {
  float frequency = 0.0f;
  bool isNdb = false;
  std::string ident;
};

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
      const double cosLat =
          std::max(0.05, std::cos(airport.lat * kDegToRad));
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

bool isRnavGps(const MapProcedure& proc) {
  std::string abbrevType;
  std::string abbrevRunway;
  if (decodeAbbreviatedApproachName(proc.name, abbrevType, abbrevRunway)) {
    return abbrevType == "RNAV";
  }
  const std::string prefix = approachTypePrefix(proc);
  return prefix == "RNAV" || proc.approachKind == "R";
}

std::string defaultTransition(const std::vector<std::string>& transitions) {
  for (const std::string& t : transitions) {
    if (upperCopy(t) == "VECTORS") return t;
  }
  return transitions.empty() ? std::string() : transitions.front();
}

}  // namespace

void SoftkeyController::buildProcMenu() {
  procMenuItems_ = {
      {"Activate Vector-to-Final", ProcMenuAction::ActivateVtf, false},
      {"Activate Approach", ProcMenuAction::ActivateApproach, false},
      {"Activate Missed Approach", ProcMenuAction::ActivateMissed, false},
      {"Select Approach", ProcMenuAction::SelectApproach, true},
      {"Select Arrival", ProcMenuAction::SelectArrival, true},
      {"Select Departure", ProcMenuAction::SelectDeparture, true},
  };
  procMode_ = ProcMode::Menu;
  procStep_ = ProcStep::ProcedureList;
  procSelected_ = 0;
  procSelectedName_.clear();
  procSelectedTransition_.clear();
  procSubListOpen_ = false;
  procApproachField_ = ProcApproachField::Apr;
  procLoadArmed_ = false;
  procActivateArmed_ = false;
  // Land the cursor on the first enabled row (the activate items are disabled).
  procMenuSel_ = 0;
  for (int i = 0; i < static_cast<int>(procMenuItems_.size()); ++i) {
    if (procMenuItems_[static_cast<std::size_t>(i)].enabled) {
      procMenuSel_ = i;
      break;
    }
  }
}

const char* SoftkeyController::procWindowTitle() const {
  if (procMode_ == ProcMode::Menu) return "Procedures";
  switch (procCategory_) {
    case ProcedureType::Departure:
      return "Select Departure";
    case ProcedureType::Arrival:
      return "Select Arrival";
    case ProcedureType::Approach:
    default:
      return "Select Approach";
  }
}

const std::string& SoftkeyController::procMenuItemText(int i) const {
  if (i < 0 || i >= static_cast<int>(procMenuItems_.size())) return kEmptyString;
  return procMenuItems_[static_cast<std::size_t>(i)].text;
}

bool SoftkeyController::procMenuItemEnabled(int i) const {
  if (i < 0 || i >= static_cast<int>(procMenuItems_.size())) return false;
  return procMenuItems_[static_cast<std::size_t>(i)].enabled;
}

std::string SoftkeyController::procAirportIcao() const {
  if (procCategory_ == ProcedureType::Departure && !fplLegs_.empty() &&
      isAirportIdent(fplLegs_.front().id)) {
    return fplLegs_.front().id;
  }

  const int approachEnd =
      fplApproachLegCount_ > 0 ? fplApproachLegStart_ : static_cast<int>(fplLegs_.size());
  std::string icao = airportIcaoBeforeIndex(fplLegs_, approachEnd);
  if (!icao.empty()) return icao;

  icao = directToAirportIcao(mapData_);
  if (!icao.empty()) return icao;

  if (mapData_ != nullptr) {
    icao = lastAirportInPlan(mapData_->flightPlan);
    if (!icao.empty()) return icao;
  }

  icao = lastAirportInPlan(fplLegs_);
  if (!icao.empty()) return icao;

  if (isAirportIdent(activeWaypoint_)) return activeWaypoint_;
  return {};
}

MapFeature SoftkeyController::procAirportFeature() const {
  if (navSource_ == nullptr || !navSource_->ready()) return {};
  const std::string icao = procAirportIcao();
  if (icao.empty()) return {};
  for (const MapFeature& f : navSource_->lookupIdent(icao, 8)) {
    if (f.type == MapFeatureType::Airport) return f;
  }
  return {};
}

std::vector<std::string> SoftkeyController::procProcedureNames(
    ProcedureType type) const {
  std::vector<std::string> names;
  if (navSource_ == nullptr || !navSource_->ready()) return names;
  const std::string icao = procAirportIcao();
  if (icao.empty()) return names;
  for (const MapProcedure& proc : navSource_->proceduresForAirport(icao, type)) {
    if (std::find(names.begin(), names.end(), proc.name) == names.end()) {
      names.push_back(proc.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> SoftkeyController::procTransitions(
    ProcedureType type, const std::string& name) const {
  std::vector<std::string> transitions;
  if (navSource_ == nullptr || !navSource_->ready()) return transitions;
  const std::string icao = procAirportIcao();
  if (icao.empty()) return transitions;
  for (const MapProcedure& proc : navSource_->proceduresForAirport(icao, type)) {
    if (proc.name != name) continue;
    if (std::find(transitions.begin(), transitions.end(), proc.transition) ==
        transitions.end()) {
      transitions.push_back(proc.transition);
    }
  }
  std::sort(transitions.begin(), transitions.end());
  return transitions;
}

std::string SoftkeyController::formatApproachLabel(const MapProcedure& proc) const {
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

std::string SoftkeyController::procApproachDisplayName(int index) const {
  const std::vector<std::string> names = procProcedureNames(ProcedureType::Approach);
  if (index < 0 || index >= static_cast<int>(names.size())) return {};
  const std::string& name = names[static_cast<std::size_t>(index)];
  if (navSource_ == nullptr || !navSource_->ready()) {
    MapProcedure stub;
    stub.name = name;
    return formatApproachLabel(stub);
  }
  const std::string icao = procAirportIcao();
  for (const MapProcedure& proc :
       navSource_->proceduresForAirport(icao, ProcedureType::Approach)) {
    if (proc.name == name) return formatApproachLabel(proc);
  }
  MapProcedure stub;
  stub.name = name;
  return formatApproachLabel(stub);
}

MapProcedure SoftkeyController::procSelectedProcedure() const {
  if (procSelectedName_.empty()) return {};
  if (navSource_ == nullptr || !navSource_->ready()) return {};
  const std::string icao = procAirportIcao();
  const std::string transition =
      procSelectedTransition_.empty() ? procSelectedTransitionDisplay()
                                      : procSelectedTransition_;
  return findProcedure(procCategory_, procSelectedName_, transition,
                       navSource_->proceduresForAirport(icao, procCategory_));
}

std::string SoftkeyController::procAirportCityLine() const {
  const MapFeature f = procAirportFeature();
  if (f.city.empty() && f.region.empty()) return {};
  if (f.city.empty()) return f.region;
  if (f.region.empty()) return f.city;
  return f.city + " " + f.region;
}

std::string SoftkeyController::procAirportNameLine() const {
  return procAirportFeature().name;
}

std::string SoftkeyController::procSelectedApproachDisplay() const {
  if (procSelectedName_.empty()) return {};
  const MapProcedure proc = procSelectedProcedure();
  if (!proc.name.empty()) return formatApproachLabel(proc);
  return procSelectedName_;
}

std::string SoftkeyController::procSelectedTransitionDisplay() const {
  if (!procSelectedTransition_.empty()) return procSelectedTransition_;
  if (procSelectedName_.empty()) return {};
  const std::vector<std::string> transitions =
      procTransitions(procCategory_, procSelectedName_);
  return defaultTransition(transitions);
}

float SoftkeyController::procPrimaryFreqMhz() const {
  const ProcPrimaryNav nav =
      resolveProcPrimaryNav(navSource_, mapData_, procAirportFeature(),
                            procSelectedProcedure());
  return nav.frequency;
}

bool SoftkeyController::procPrimaryNavIsNdb() const {
  const ProcPrimaryNav nav =
      resolveProcPrimaryNav(navSource_, mapData_, procAirportFeature(),
                            procSelectedProcedure());
  return nav.isNdb;
}

bool SoftkeyController::procShowsPrimaryNavFreq() const {
  return procedureUsesNavPrimaryFrequency(procSelectedProcedure());
}

std::string SoftkeyController::procPrimaryIdent() const {
  const ProcPrimaryNav nav =
      resolveProcPrimaryNav(navSource_, mapData_, procAirportFeature(),
                            procSelectedProcedure());
  return nav.ident;
}

std::vector<std::string> SoftkeyController::procListItems() const {
  if (procStep_ == ProcStep::TransitionList) {
    return procTransitions(procCategory_, procSelectedName_);
  }
  return procProcedureNames(procCategory_);
}

bool SoftkeyController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procLoadPending_) return false;
  procLoadPending_ = false;
  out = procLoadTarget_;
  return true;
}

void SoftkeyController::procMoveMenu(int dir) {
  const int n = static_cast<int>(procMenuItems_.size());
  if (n == 0) return;
  for (int step = 0; step < n; ++step) {
    procMenuSel_ = (procMenuSel_ + dir + n) % n;
    if (procMenuItems_[static_cast<std::size_t>(procMenuSel_)].enabled) break;
  }
}

void SoftkeyController::procOpenApproachSelect() {
  procMode_ = ProcMode::Select;
  procStep_ = ProcStep::ProcedureList;
  procSelected_ = 0;
  procSelectedName_.clear();
  procSelectedTransition_.clear();
  procSubListOpen_ = true;
  procApproachField_ = ProcApproachField::Apr;
  procLoadArmed_ = false;
  procActivateArmed_ = false;
}

void SoftkeyController::procPickApproach(const std::string& name) {
  procSelectedName_ = name;
  const std::vector<std::string> trans =
      procTransitions(procCategory_, name);
  if (trans.size() <= 1) {
    procSelectedTransition_ = trans.empty() ? std::string() : trans.front();
    procCloseSubList();
    procFocusLoad();
    return;
  }
  procStep_ = ProcStep::TransitionList;
  procSelected_ = 0;
  for (int i = 0; i < static_cast<int>(trans.size()); ++i) {
    if (upperCopy(trans[static_cast<std::size_t>(i)]) == "VECTORS") {
      procSelected_ = i;
      break;
    }
  }
  procSelectedTransition_ = trans[static_cast<std::size_t>(procSelected_)];
  procSubListOpen_ = true;
  procApproachField_ = ProcApproachField::Trans;
}

void SoftkeyController::procBackToApproachList() {
  procStep_ = ProcStep::ProcedureList;
  procSelectedTransition_.clear();
  procSelected_ = 0;
  const std::vector<std::string> names = procProcedureNames(procCategory_);
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    if (names[static_cast<std::size_t>(i)] == procSelectedName_) {
      procSelected_ = i;
      break;
    }
  }
  procSubListOpen_ = true;
  procApproachField_ = ProcApproachField::Apr;
}

void SoftkeyController::procCloseSubList() {
  procSubListOpen_ = false;
  if (procStep_ == ProcStep::TransitionList) {
    procStep_ = ProcStep::ProcedureList;
  }
}

void SoftkeyController::procCycleApproachField(int dir) {
  if (procApproachField_ == ProcApproachField::MinsAlt) {
    procApproachField_ = ProcApproachField::Mins;
    procLoadArmed_ = false;
    procActivateArmed_ = false;
    return;
  }
  const int count = static_cast<int>(ProcApproachField::Count);
  int field = static_cast<int>(procApproachField_);
  do {
    field = (field + dir + count) % count;
  } while (field == static_cast<int>(ProcApproachField::MinsAlt));
  procApproachField_ = static_cast<ProcApproachField>(field);
  procLoadArmed_ = procApproachField_ == ProcApproachField::Load;
  procActivateArmed_ = procApproachField_ == ProcApproachField::Activate;
}

void SoftkeyController::procFocusLoad() {
  procApproachField_ = ProcApproachField::Load;
  procLoadArmed_ = true;
  procActivateArmed_ = false;
}

void SoftkeyController::procOpenApproachList() {
  procStep_ = ProcStep::ProcedureList;
  procSubListOpen_ = true;
  procSelected_ = 0;
  if (!procSelectedName_.empty()) {
    const std::vector<std::string> names =
        procProcedureNames(procCategory_);
    for (int i = 0; i < static_cast<int>(names.size()); ++i) {
      if (names[static_cast<std::size_t>(i)] == procSelectedName_) {
        procSelected_ = i;
        break;
      }
    }
  }
}

void SoftkeyController::procOpenTransitionList() {
  const std::vector<std::string> trans =
      procTransitions(procCategory_, procSelectedName_);
  if (trans.size() <= 1) return;
  procStep_ = ProcStep::TransitionList;
  procSubListOpen_ = true;
  procSelected_ = 0;
  for (int i = 0; i < static_cast<int>(trans.size()); ++i) {
    if (trans[static_cast<std::size_t>(i)] == procSelectedTransition_) {
      procSelected_ = i;
      break;
    }
  }
}

void SoftkeyController::procCycleApproachMins(int dir) {
  (void)dir;
  if (minsMode_ == MinimumsMode::Off) {
    minsMode_ = MinimumsMode::Baro;
  } else {
    minsMode_ = MinimumsMode::Off;
    if (procApproachField_ == ProcApproachField::MinsAlt) {
      procApproachField_ = ProcApproachField::Mins;
    }
  }
}

void SoftkeyController::procAdjustApproachMinsAlt(int step) {
  constexpr float kStepFt = 100.0f;
  constexpr float kMaxFt = 16000.0f;
  minsAltFt_ = std::max(0.0f, std::min(kMaxFt, minsAltFt_ + step * kStepFt));
}

void SoftkeyController::procLoadSelected(const std::string& name,
                                         const std::string& transition) {
  if (navSource_ == nullptr || !navSource_->ready()) return;
  const std::string icao = procAirportIcao();
  std::vector<MapLeg> legs =
      navSource_->expandProcedure(icao, procCategory_, name, transition);
  if (legs.empty()) return;
  if (procCategory_ == ProcedureType::Approach) {
    fplApproachLegStart_ = static_cast<int>(fplLegs_.size());
    fplApproachLegCount_ = static_cast<int>(legs.size());
  } else {
    fplApproachLegStart_ = 0;
    fplApproachLegCount_ = 0;
    fplLoadedApproach_ = {};
  }
  insertProcedureLegs(procCategory_, fplLegs_, legs);
  if (procCategory_ == ProcedureType::Approach) {
    fplCursorRow_ = fplApproachLegStart_ + fplApproachLegCount_ - 1;
    fplLoadedApproach_ = findProcedure(procCategory_, name, transition,
                                       navSource_->proceduresForAirport(
                                           icao, procCategory_));
  }
  flightPlanPublishEdit();
  procLoadTarget_ = fplLoadedApproach_;
  procLoadPending_ = true;
  // Loading closes the Procedures window; the new legs show on the FPL window.
  window_ = PfdWindow::None;
  procMode_ = ProcMode::Menu;
  procStep_ = ProcStep::ProcedureList;
  procSelectedName_.clear();
  procSelectedTransition_.clear();
  procSubListOpen_ = false;
  procLoadArmed_ = false;
  procActivateArmed_ = false;
}

std::string SoftkeyController::flightPlanApproachAirportIcao() const {
  if (fplApproachLegCount_ <= 0) return {};
  std::string icao = airportIcaoBeforeIndex(fplLegs_, fplApproachLegStart_);
  if (!icao.empty()) return icao;
  icao = directToAirportIcao(mapData_);
  if (!icao.empty()) return icao;
  if (mapData_ != nullptr) {
    icao = lastAirportInPlan(mapData_->flightPlan);
    if (!icao.empty()) return icao;
  }
  return {};
}

std::string SoftkeyController::flightPlanApproachHeaderLabel() const {
  if (fplApproachLegCount_ <= 0 || fplLoadedApproach_.name.empty()) return {};
  return formatApproachLabel(fplLoadedApproach_);
}

bool SoftkeyController::procBezelKey(BezelKey key) {
  // FPL / PROC / MENU switch windows on the real unit even while Procedures is
  // open (see isPageNavigationBezelKey in BezelKeys.h).
  if (isPageNavigationBezelKey(key)) {
    pageMenuOpen_ = false;
    return false;
  }

  // Top-level menu: the knob moves between enabled rows, ENT opens the
  // selection sub-window for a "Select ..." item, CLR / knob push closes.
  if (procMode_ == ProcMode::Menu) {
    switch (key) {
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsOuterCw:
        procMoveMenu(+1);
        return true;
      case BezelKey::FmsInnerCcw:
      case BezelKey::FmsOuterCcw:
        procMoveMenu(-1);
        return true;
      case BezelKey::Ent: {
        if (procMenuSel_ < 0 ||
            procMenuSel_ >= static_cast<int>(procMenuItems_.size())) {
          return true;
        }
        const ProcMenuItem& item =
            procMenuItems_[static_cast<std::size_t>(procMenuSel_)];
        if (!item.enabled) return true;
        switch (item.action) {
          case ProcMenuAction::SelectDeparture:
            procCategory_ = ProcedureType::Departure;
            procMode_ = ProcMode::Select;
            procStep_ = ProcStep::ProcedureList;
            procSelected_ = 0;
            procSelectedName_.clear();
            procSelectedTransition_.clear();
            procSubListOpen_ = false;
            return true;
          case ProcMenuAction::SelectArrival:
            procCategory_ = ProcedureType::Arrival;
            procMode_ = ProcMode::Select;
            procStep_ = ProcStep::ProcedureList;
            procSelected_ = 0;
            procSelectedName_.clear();
            procSelectedTransition_.clear();
            procSubListOpen_ = false;
            return true;
          case ProcMenuAction::SelectApproach:
            procCategory_ = ProcedureType::Approach;
            procOpenApproachSelect();
            return true;
          default:
            return true;  // activate items are inert in this suite
        }
      }
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        window_ = PfdWindow::None;
        return true;
      default:
        return false;
    }
  }

  const bool approachDetail =
      procCategory_ == ProcedureType::Approach && procSelectMode();
  const std::vector<std::string> items = procListItems();

  if (approachDetail && procSubListOpen_) {
    switch (key) {
      case BezelKey::Ent:
        if (procSelected_ >= 0 && procSelected_ < static_cast<int>(items.size())) {
          if (procStep_ == ProcStep::ProcedureList) {
            procPickApproach(items[static_cast<std::size_t>(procSelected_)]);
          } else {
            procSelectedTransition_ =
                items[static_cast<std::size_t>(procSelected_)];
            procCloseSubList();
            procFocusLoad();
          }
        }
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        if (procStep_ == ProcStep::TransitionList) {
          procBackToApproachList();
        } else {
          procCloseSubList();
          procSelectedName_.clear();
          procSelectedTransition_.clear();
        }
        return true;
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsOuterCw:
        if (!items.empty()) {
          procSelected_ = (procSelected_ + 1) % static_cast<int>(items.size());
        }
        return true;
      case BezelKey::FmsInnerCcw:
      case BezelKey::FmsOuterCcw:
        if (!items.empty()) {
          procSelected_ = (procSelected_ - 1 + static_cast<int>(items.size())) %
                          static_cast<int>(items.size());
        }
        return true;
      default:
        return false;
    }
  }

  if (approachDetail) {
    switch (key) {
      case BezelKey::Ent:
        if (procApproachField_ == ProcApproachField::Load) {
          if (procLoadArmed_ && !procSelectedName_.empty()) {
            procLoadSelected(procSelectedName_,
                             procSelectedTransitionDisplay());
          } else if (!procSelectedName_.empty()) {
            procFocusLoad();
          }
          return true;
        }
        if (procApproachField_ == ProcApproachField::Activate &&
            !procSelectedName_.empty()) {
          procApproachField_ = ProcApproachField::Activate;
          procActivateArmed_ = true;
          procLoadArmed_ = false;
          return true;
        }
        procFocusLoad();
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        procMode_ = ProcMode::Menu;
        procSubListOpen_ = false;
        procLoadArmed_ = false;
        procActivateArmed_ = false;
        return true;
      case BezelKey::FmsOuterCw:
        procCycleApproachField(+1);
        return true;
      case BezelKey::FmsOuterCcw:
        procCycleApproachField(-1);
        return true;
      case BezelKey::FmsInnerCw:
        if (procApproachField_ == ProcApproachField::MinsAlt) {
          procAdjustApproachMinsAlt(+1);
        } else if (procApproachField_ == ProcApproachField::Mins) {
          if (minsMode_ == MinimumsMode::Baro) {
            procApproachField_ = ProcApproachField::MinsAlt;
          } else {
            procCycleApproachMins(+1);
          }
        } else if (procApproachField_ == ProcApproachField::Apr) {
          procOpenApproachList();
        } else if (procApproachField_ == ProcApproachField::Trans) {
          procOpenTransitionList();
        } else if (procApproachField_ == ProcApproachField::Activate &&
                   !procSelectedName_.empty()) {
          procApproachField_ = ProcApproachField::Activate;
          procActivateArmed_ = true;
          procLoadArmed_ = false;
        } else if (procApproachField_ == ProcApproachField::Load &&
                   !procSelectedName_.empty()) {
          procFocusLoad();
        }
        return true;
      case BezelKey::FmsInnerCcw:
        if (procApproachField_ == ProcApproachField::MinsAlt) {
          procAdjustApproachMinsAlt(-1);
        } else if (procApproachField_ == ProcApproachField::Mins) {
          procCycleApproachMins(-1);
        } else if (procApproachField_ == ProcApproachField::Apr) {
          procOpenApproachList();
        } else if (procApproachField_ == ProcApproachField::Trans) {
          procOpenTransitionList();
        } else if (procApproachField_ == ProcApproachField::Activate &&
                   !procSelectedName_.empty()) {
          procApproachField_ = ProcApproachField::Activate;
          procActivateArmed_ = true;
          procLoadArmed_ = false;
        } else if (procApproachField_ == ProcApproachField::Load &&
                   !procSelectedName_.empty()) {
          procFocusLoad();
        }
        return true;
      default:
        return false;
    }
  }

  // Departure / arrival selection sub-window (simple list).
  switch (key) {
    case BezelKey::Ent:
      if (procStep_ == ProcStep::ProcedureList) {
        if (procSelected_ >= 0 &&
            procSelected_ < static_cast<int>(items.size())) {
          const std::string& name = items[static_cast<std::size_t>(procSelected_)];
          const std::vector<std::string> trans =
              procTransitions(procCategory_, name);
          if (trans.size() == 1) {
            procLoadSelected(name, trans.front());
          } else if (trans.size() > 1) {
            procSelectedName_ = name;
            procStep_ = ProcStep::TransitionList;
            procSelected_ = 0;
          }
        }
      } else if (procSelected_ >= 0 &&
                 procSelected_ < static_cast<int>(items.size())) {
        procLoadSelected(procSelectedName_,
                         items[static_cast<std::size_t>(procSelected_)]);
      }
      return true;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      if (procStep_ == ProcStep::TransitionList) {
        procStep_ = ProcStep::ProcedureList;
        procSelectedName_.clear();
        procSelected_ = 0;
      } else {
        procMode_ = ProcMode::Menu;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsOuterCw:
      if (!items.empty()) {
        procSelected_ = (procSelected_ + 1) % static_cast<int>(items.size());
      }
      return true;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsOuterCcw:
      if (!items.empty()) {
        procSelected_ = (procSelected_ - 1 + static_cast<int>(items.size())) %
                        static_cast<int>(items.size());
      }
      return true;
    default:
      return false;
  }
}

}  // namespace avionics
