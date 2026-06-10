#include "AirspaceStore.h"

#include <cstdlib>
#include <fstream>
#include <string>

#include "avionics/OpenAirParser.h"

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

std::string installListDir() {
#if defined(_WIN32)
  const char* base = std::getenv("LOCALAPPDATA");
  return base ? std::string(base) + "\\" : std::string();
#elif defined(__APPLE__)
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/Library/Preferences/" : std::string();
#else
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/.x-plane/" : std::string();
#endif
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

std::vector<std::string> readInstallRoots() {
  std::vector<std::string> roots;
  const std::string dir = installListDir();
  if (dir.empty()) return roots;
  for (const char* version : {"12", "11"}) {
    const std::string listPath =
        join(dir, std::string("x-plane_install_") + version + ".txt");
    std::ifstream in(listPath);
    if (!in.good()) continue;
    std::string line;
    while (std::getline(in, line)) {
      while (!line.empty() &&
             (line.back() == '\r' || line.back() == '\n' ||
              line.back() == ' ' || line.back() == '\t')) {
        line.pop_back();
      }
      if (!line.empty()) roots.push_back(line);
    }
  }
  return roots;
}

// Within an install root, the OpenAir airspace file. User data in
// Custom Data/Airspaces wins over the bundled default, matching X-Plane.
std::string airspaceFileForRoot(const std::string& root) {
  const std::string candidates[] = {
      join(join(root, "Custom Data"), join("Airspaces", "airspace.txt")),
      join(join(root, "Resources"), join("default data",
                                         join("airspaces", "airspace.txt"))),
  };
  for (const std::string& path : candidates) {
    if (fileExists(path)) return path;
  }
  return std::string();
}

}  // namespace

AirspaceStore::AirspaceStore() {
  thread_ = std::thread([this] { load(); });
}

AirspaceStore::~AirspaceStore() {
  if (thread_.joinable()) thread_.join();
}

void AirspaceStore::load() {
  std::string path;
  for (const std::string& root : readInstallRoots()) {
    path = airspaceFileForRoot(root);
    if (!path.empty()) break;
  }
  if (path.empty()) {
    loaded_.store(true, std::memory_order_release);
    return;
  }

  std::ifstream in(path);
  if (in.good()) {
    airspaces_ = parseOpenAir(in);
    sourcePath_ = path;
  }
  loaded_.store(true, std::memory_order_release);
}

std::vector<MapAirspace> AirspaceStore::nearby(double lat, double lon,
                                               float rangeNm,
                                               std::size_t maxCount) const {
  if (!loaded()) return {};
  return airspacesNear(airspaces_, lat, lon, rangeNm, maxCount);
}

}  // namespace avionics
