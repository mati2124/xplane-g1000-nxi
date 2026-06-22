#include "ProcedureStore.h"

#include "AptDatStore.h"
#include "NavData.h"
#include "XPlaneInstall.h"

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
  double refLat = 0.0;
  double refLon = 0.0;
  bool haveRef = false;
};

bool fixLookup(const std::string& ident, double& lat, double& lon, void* ctx) {
  auto* c = static_cast<LookupCtx*>(ctx);
  if (c == nullptr) return false;

  if (c->runways != nullptr) {
    const auto it = c->runways->find(ident);
    if (it != c->runways->end()) {
      lat = it->second.first;
      lon = it->second.second;
      return true;
    }
  }

  if (c->nav == nullptr || !c->nav->ready()) return false;
  const std::vector<MapFeature> matches = c->nav->lookupIdent(ident, 16);
  if (matches.empty()) return false;

  const MapFeature* best = &matches.front();
  if (c->haveRef) {
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
  if (aptData != nullptr && aptData->loaded()) {
    const AirportMeta* meta = aptData->airportMeta(icao);
    if (meta != nullptr) {
      for (const AirportRunwayInfo& rwy : meta->runways) {
        if (!rwy.designation.empty()) runways.insert(rwy.designation);
      }
    }
  }
  for (const MapProcedure& proc : list) {
    if (proc.type != ProcedureType::Approach) continue;
    if (!proc.runway.empty()) runways.insert(proc.runway);
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
