#include "AptDatStore.h"

#include "XPlaneInstall.h"
#include "avionics/AptDatGeometryCache.h"
#include "avionics/AptDatParser.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace avionics {
namespace {

char pathSep() {
#if defined(_WIN32)
  return '\\';
#else
  return '/';
#endif
}

std::string join(const std::string& dir, const std::string& leaf) {
  if (dir.empty()) return leaf;
  if (dir.back() == '/' || dir.back() == '\\') return dir + leaf;
  return dir + pathSep() + leaf;
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

std::string cacheDir() {
#if defined(_WIN32)
  const char* base = std::getenv("LOCALAPPDATA");
  return base ? std::string(base) + "\\xplane-g1000-nxi" : std::string();
#elif defined(__APPLE__)
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/Library/Application Support/xplane-g1000-nxi"
              : std::string();
#else
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/.local/share/xplane-g1000-nxi"
              : std::string();
#endif
}

std::string aptGeometryCachePath() {
  const std::string dir = cacheDir();
  return dir.empty() ? std::string() : join(dir, "apt_geometry.cache");
}

}  // namespace

AptDatStore::AptDatStore() {
  thread_ = std::thread([this] { load(); });
}

AptDatStore::~AptDatStore() {
  if (thread_.joinable()) thread_.join();
}

void AptDatStore::load() {
  std::string path;
  for (const std::string& root : xplane_install::readInstallRoots()) {
    const std::string candidates[] = {
        join(join(join(root, "Global Scenery"), "Global Airports"),
             join("Earth nav data", "apt.dat")),
        join(join(join(root, "Custom Scenery"), "Global Airports"),
             join("Earth nav data", "apt.dat")),
    };
    for (const std::string& candidate : candidates) {
      if (fileExists(candidate)) {
        path = candidate;
        break;
      }
    }
    if (!path.empty()) break;
  }
  if (path.empty()) {
    loaded_.store(true, std::memory_order_release);
    return;
  }

  std::ifstream in(path);
  if (in.good()) {
    AptDatParseResult parsed;
    const std::string cachePath = aptGeometryCachePath();
    if (!cachePath.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(cacheDir(), ec);
    }
    const bool cached =
        !cachePath.empty() &&
        loadAptDatGeometryCache(cachePath, path, parsed);
    if (!cached) {
      in.close();
      in.open(path);
      if (in.good()) parsed = parseAptDat(in);
      if (!cachePath.empty()) {
        saveAptDatGeometryCache(cachePath, path, parsed);
      }
    }
    cells_ = std::move(parsed.runwayCells);
    pavementCells_ = std::move(parsed.pavementCells);
    taxiwayLabelCells_ = std::move(parsed.taxiwayLabelCells);
    metaByIcao_ = std::move(parsed.metaByIcao);
    sourcePath_ = path;
  }
  loaded_.store(true, std::memory_order_release);
}

std::vector<MapRunway> AptDatStore::nearby(double lat, double lon,
                                           float rangeNm,
                                           std::size_t maxCount) const {
  if (!loaded()) return {};
  return nearbyRunwaysFromCells(cells_, lat, lon, rangeNm, maxCount);
}

std::vector<MapPavement> AptDatStore::nearbyTaxiways(double lat, double lon,
                                                     float rangeNm,
                                                     std::size_t maxCount) const {
  if (!loaded()) return {};
  return nearbyPavementFromCells(pavementCells_, lat, lon, rangeNm, maxCount);
}

std::vector<MapTaxiwayLabel> AptDatStore::nearbyTaxiwayLabels(
    double lat, double lon, float rangeNm, std::size_t maxCount) const {
  if (!loaded()) return {};
  return nearbyTaxiwayLabelsFromCells(taxiwayLabelCells_, lat, lon, rangeNm,
                                      maxCount);
}

const AirportMeta* AptDatStore::airportMeta(const std::string& icao) const {
  std::string key = icao;
  for (char& c : key) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  const auto it = metaByIcao_.find(key);
  return it == metaByIcao_.end() ? nullptr : &it->second;
}

std::vector<MapAirportFrequency> AptDatStore::frequenciesForAirport(
    const std::string& icao) const {
  if (!loaded() || icao.empty()) return {};
  const auto it = metaByIcao_.find(icao);
  if (it == metaByIcao_.end()) return {};
  return it->second.frequencies;
}

void AptDatStore::enrichAirport(MapFeature& feature) const {
  enrichAirportFromMeta(feature, metaByIcao_);
}

}  // namespace avionics
