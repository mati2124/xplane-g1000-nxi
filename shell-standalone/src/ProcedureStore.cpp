#include "ProcedureStore.h"

#include "AptDatStore.h"
#include "NavData.h"
#include "XPlaneInstall.h"

#include "avionics/NavMath.h"

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include <fstream>

namespace avionics {
namespace {

std::string join(const std::string& dir, const std::string& leaf) {
  if (dir.empty()) return leaf;
  if (dir.back() == '/' || dir.back() == '\\') return dir + leaf;
  return dir + '/' + leaf;
}

std::string cifpDirForRoot(const std::string& root) {
  const std::string candidates[] = {join(root, "Custom Data/CIFP"),
                                    join(root, "Resources/default data/CIFP")};
  for (const std::string& dir : candidates) {
    std::ifstream probe(join(dir, "KSEA.dat"));
    if (probe.good()) return dir;
  }
  return std::string();
}

struct LookupCtx {
  const NavFeatureSource* nav = nullptr;
  const std::unordered_map<std::string, std::pair<double, double>>* runways =
      nullptr;
  const std::unordered_map<std::string, std::pair<double, double>>* cifpFixes =
      nullptr;
  double refLat = 0.0;
  double refLon = 0.0;
  bool haveRef = false;
};

bool fixLookup(const std::string& ident, double& lat, double& lon, void* ctx) {
  auto* c = static_cast<LookupCtx*>(ctx);
  if (c == nullptr) return false;

  if (c->cifpFixes != nullptr) {
    const auto fixIt = c->cifpFixes->find(ident);
    if (fixIt != c->cifpFixes->end()) {
      lat = fixIt->second.first;
      lon = fixIt->second.second;
      return true;
    }
  }

  if (c->runways != nullptr) {
    const auto it = c->runways->find(ident);
    if (it != c->runways->end()) {
      lat = it->second.first;
      lon = it->second.second;
      return true;
    }
  }

  if (c->nav == nullptr || !c->nav->ready()) return false;
  std::vector<MapFeature> matches;
  if (c->haveRef) {
    matches = c->nav->lookupIdentNear(ident, c->refLat, c->refLon, 16);
  }
  if (matches.empty()) {
    matches = c->nav->lookupIdent(ident, 16);
  }
  if (matches.empty()) return false;

  const MapFeature* best = &matches.front();
  if (c->haveRef && matches.size() > 1) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double cosLat =
        std::max(0.05, std::cos(c->refLat * kDegToRad));
    double bestSq = 0.0;
    bool first = true;
    for (const MapFeature& f : matches) {
      const double dLat = f.lat - c->refLat;
      const double dLon = (f.lon - c->refLon) * cosLat;
      const double dSq = dLat * dLat + dLon * dLon;
      if (first || dSq < bestSq) {
        best = &f;
        bestSq = dSq;
        first = false;
      }
    }
  }
  lat = best->lat;
  lon = best->lon;
  return true;
}

void mergeIlsApproaches(const NavDataStore& nav, const std::string& icao,
                        std::vector<MapProcedure>& list) {
  for (const MapApproach& ap : nav.approachesForAirport(icao)) {
    MapProcedure proc;
    proc.type = ProcedureType::Approach;
    proc.name = ap.ident;
    proc.transition = ap.runway.empty() ? ap.ident : ap.runway;
    proc.runway = ap.runway;
    proc.approachKind = ap.hasGlideslope ? "I" : "L";
    proc.frequencyMhz = ap.frequencyMhz;
    list.push_back(std::move(proc));
  }
}

bool hasApproachName(const std::vector<MapProcedure>& list,
                     const std::string& name) {
  for (const MapProcedure& proc : list) {
    if (proc.name == name) return true;
  }
  return false;
}

void addSyntheticVisualApproaches(const AptDatStore* aptData,
                                  const std::string& icao,
                                  std::vector<MapProcedure>& list) {
  std::unordered_set<std::string> runways;
  // apt.dat designations are the paired form ("05-23"); each end gets its own
  // visual approach (the unit lists VISUAL 05, VISUAL 23, ... not "05-23").
  const auto addRunwayEnds = [&](const std::string& designation) {
    if (designation.empty()) return;
    const std::size_t dash = designation.find('-');
    if (dash == std::string::npos) {
      runways.insert(designation);
      return;
    }
    const std::string first = designation.substr(0, dash);
    const std::string second = designation.substr(dash + 1);
    if (!first.empty()) runways.insert(first);
    if (!second.empty()) runways.insert(second);
  };
  if (aptData != nullptr && aptData->loaded()) {
    const AirportMeta* meta = aptData->airportMeta(icao);
    if (meta != nullptr) {
      for (const AirportRunwayInfo& rwy : meta->runways) {
        addRunwayEnds(rwy.designation);
      }
    }
  }
  for (const MapProcedure& proc : list) {
    if (proc.type != ProcedureType::Approach) continue;
    addRunwayEnds(proc.runway);
  }
  for (const std::string& runway : runways) {
    const std::string name = "VISUAL" + runway;
    if (hasApproachName(list, name)) continue;
    MapProcedure visual;
    visual.type = ProcedureType::Approach;
    visual.name = name;
    visual.transition = "RW" + runway;
    visual.runway = runway;
    visual.levelOfService = "VISUAL";
    list.push_back(std::move(visual));
  }
}

void mergeProcedureLevelOfService(std::vector<MapProcedure>& list) {
  std::unordered_map<std::string, std::string> bestLos;
  for (const MapProcedure& proc : list) {
    if (proc.levelOfService.empty()) continue;
    auto it = bestLos.find(proc.name);
    if (it == bestLos.end()) {
      bestLos[proc.name] = proc.levelOfService;
    }
  }
  for (MapProcedure& proc : list) {
    if (proc.levelOfService.empty()) {
      auto it = bestLos.find(proc.name);
      if (it != bestLos.end()) proc.levelOfService = it->second;
    }
  }
}

// Reciprocal runway key ("RW13" -> "RW31", swapping L/R), used to derive the
// runway centerline course when only CIFP thresholds are available.
std::string oppositeRunwayKey(const std::string& runway) {
  std::size_t digits = 0;
  while (digits < runway.size() && runway[digits] >= '0' &&
         runway[digits] <= '9') {
    ++digits;
  }
  if (digits == 0) return {};
  const int num = std::stoi(runway.substr(0, digits));
  const int opp = ((num + 18 - 1) % 36) + 1;
  std::string suffix = runway.substr(digits);
  if (suffix == "L") suffix = "R";
  else if (suffix == "R") suffix = "L";
  char numbuf[8];
  std::snprintf(numbuf, sizeof(numbuf), "%02d", opp);
  return "RW" + std::string(numbuf) + suffix;
}

// Garmin synthesizes a straight-in visual approach along the runway centerline:
// STRGHT (3.5 NM final), FINAL (FAF, 1.0 NM), the runway threshold (MAP), and
// MANSEQ (5.0 NM straight-ahead missed sequence). Distances/roles match the
// trainer's VISUAL approach preview.
std::vector<MapLeg> synthesizeVisualApproach(const AptDatStore* aptData,
                                             const CifpAirportProcedures& airport,
                                             double refLat, double refLon,
                                             bool haveRef,
                                             const std::string& runway) {
  if (runway.empty()) return {};
  double thrLat = 0.0;
  double thrLon = 0.0;
  double courseDeg = 0.0;
  bool resolved = false;

  // Prefer apt.dat geometry: both thresholds give the exact centerline course.
  if (aptData != nullptr && aptData->loaded() && haveRef) {
    for (const MapRunway& rw : aptData->nearby(refLat, refLon, 6.0f, 64)) {
      if (rw.idA == runway) {
        thrLat = rw.a.lat;
        thrLon = rw.a.lon;
        courseDeg = navBearingDeg(rw.a.lat, rw.a.lon, rw.b.lat, rw.b.lon);
        resolved = true;
        break;
      }
      if (rw.idB == runway) {
        thrLat = rw.b.lat;
        thrLon = rw.b.lon;
        courseDeg = navBearingDeg(rw.b.lat, rw.b.lon, rw.a.lat, rw.a.lon);
        resolved = true;
        break;
      }
    }
  }

  // Fall back to the CIFP runway thresholds.
  if (!resolved) {
    const auto thrIt = airport.runways.find("RW" + runway);
    if (thrIt == airport.runways.end()) return {};
    thrLat = thrIt->second.first;
    thrLon = thrIt->second.second;
    const auto oppIt = airport.runways.find(oppositeRunwayKey(runway));
    if (oppIt != airport.runways.end()) {
      courseDeg = navBearingDeg(thrLat, thrLon, oppIt->second.first,
                                oppIt->second.second);
    } else {
      int num = 0;
      for (char ch : runway) {
        if (ch < '0' || ch > '9') break;
        num = num * 10 + (ch - '0');
      }
      courseDeg = static_cast<double>(num) * 10.0;
    }
  }

  const double backDeg = courseDeg + 180.0;
  const auto makeLeg = [](const std::string& id, double lat, double lon,
                          const std::string& role) {
    MapLeg leg;
    leg.id = id;
    leg.lat = lat;
    leg.lon = lon;
    leg.procedureRole = role;
    return leg;
  };

  std::vector<MapLeg> legs;
  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(thrLat, thrLon, backDeg, 3.5, lat, lon);
  legs.push_back(makeLeg("STRGHT", lat, lon, ""));
  navOffsetPoint(thrLat, thrLon, backDeg, 1.0, lat, lon);
  legs.push_back(makeLeg("FINAL", lat, lon, "faf"));
  legs.push_back(makeLeg("RW" + runway, thrLat, thrLon, "map"));
  navOffsetPoint(thrLat, thrLon, courseDeg, 5.0, lat, lon);
  legs.push_back(makeLeg("MANSEQ", lat, lon, ""));
  return legs;
}

}  // namespace

ProcedureStore::ProcedureStore(const NavDataStore& navData,
                               const AptDatStore* aptData)
    : navData_(navData), aptData_(aptData) {
  for (const std::string& root : xplane_install::readInstallRoots()) {
    sourceDir_ = cifpDirForRoot(root);
    if (!sourceDir_.empty()) break;
  }
}

const CifpAirportProcedures& ProcedureStore::loadAirport(
    const std::string& icao) const {
  const std::string key = icao;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
  }

  CifpAirportProcedures parsed;
  parsed.icao = icao;
  if (!sourceDir_.empty() && !icao.empty()) {
    std::ifstream in(join(sourceDir_, icao + ".dat"));
    if (in.good()) parsed = parseCifp(in, icao);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.emplace(key, std::move(parsed)).first->second;
}

std::vector<MapProcedure> ProcedureStore::proceduresForAirport(
    const std::string& icao, ProcedureType type) const {
  std::vector<MapProcedure> result;
  if (icao.empty()) return result;

  for (const MapProcedure& p : loadAirport(icao).catalog) {
    if (p.type == type) result.push_back(p);
  }

  if (type == ProcedureType::Approach) {
    mergeIlsApproaches(navData_, icao, result);
    mergeProcedureLevelOfService(result);
    addSyntheticVisualApproaches(aptData_, icao, result);
    std::sort(result.begin(), result.end(),
              [](const MapProcedure& a, const MapProcedure& b) {
                if (a.name != b.name) return a.name < b.name;
                return a.transition < b.transition;
              });
  }
  return result;
}

std::vector<MapLeg> ProcedureStore::expandProcedure(
    const std::string& icao, ProcedureType type, const std::string& name,
    const std::string& transition, const NavFeatureSource* lookup) const {
  const CifpAirportProcedures& airport = loadAirport(icao);
  LookupCtx ctx;
  ctx.nav = lookup;
  ctx.runways = &airport.runways;
  ctx.cifpFixes = &airport.fixes;
  if (lookup != nullptr && lookup->ready() && !icao.empty()) {
    const std::vector<MapFeature> apt = lookup->lookupIdent(icao, 1);
    if (!apt.empty()) {
      ctx.refLat = apt[0].lat;
      ctx.refLon = apt[0].lon;
      ctx.haveRef = true;
    }
  }
  const std::vector<MapLeg> cifpLegs = expandCifpProcedure(
      airport, type, name, transition, fixLookup, &ctx);
  if (!cifpLegs.empty()) return cifpLegs;

  if (type != ProcedureType::Approach) return {};

  // Synthetic visual approaches have no CIFP legs; build the straight-in course
  // (STRGHT/FINAL/RW/MANSEQ) from the runway centerline geometry.
  if (name.compare(0, 6, "VISUAL") == 0) {
    const std::string runway = name.substr(6);
    return synthesizeVisualApproach(aptData_, airport, ctx.refLat, ctx.refLon,
                                    ctx.haveRef, runway);
  }

  for (const MapApproach& ap : navData_.approachesForAirport(icao)) {
    if (ap.ident != name && ap.runway != transition) continue;
    if (lookup == nullptr || !lookup->ready()) return {};
    const std::vector<MapFeature> matches = lookup->lookupIdent(ap.ident, 1);
    if (matches.empty()) continue;
    MapLeg leg;
    leg.id = matches[0].id;
    leg.lat = matches[0].lat;
    leg.lon = matches[0].lon;
    return {leg};
  }
  return {};
}

std::vector<ApproachTransitionOption> ProcedureStore::approachTransitionsFor(
    const std::string& icao, const std::string& approachName) const {
  if (icao.empty() || approachName.empty()) return {};
  return listApproachTransitions(loadAirport(icao), approachName);
}

}  // namespace avionics
