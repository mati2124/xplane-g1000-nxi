#include "NavData.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace avionics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

// XP-NAV row codes we surface on the map; the rest (ILS/markers/DME/etc.) are
// skipped to avoid clutter at inset-map scale.
constexpr long kRowNdb = 2;
constexpr long kRowVor = 3;
constexpr long kRowTerminator = 99;

// Per-OS directory holding x-plane_install_*.txt. Each line in that file is the
// full path to an X-Plane install root.
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

// Read the install-list files (XP 12 first, then 11) and return each install
// root listed inside, in order.
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
      // Trim trailing whitespace / CR (Windows line endings on POSIX).
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

// Given an install root, return the directory that contains earth_nav.dat and
// earth_fix.dat, preferring user-updated "Custom Data" over the bundled
// "Resources/default data". Returns empty if neither has the files.
std::string navDataDirForRoot(const std::string& root) {
  const std::string candidates[] = {join(root, "Custom Data"),
                                     join(join(root, "Resources"),
                                          "default data")};
  for (const std::string& dir : candidates) {
    if (fileExists(join(dir, "earth_nav.dat")) &&
        fileExists(join(dir, "earth_fix.dat"))) {
      return dir;
    }
  }
  return std::string();
}

// Skip leading blanks then advance past one whitespace-delimited token.
const char* skipToken(const char* p) {
  while (*p == ' ' || *p == '\t') ++p;
  while (*p && *p != ' ' && *p != '\t') ++p;
  return p;
}

const char* skipSpaces(const char* p) {
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

std::string readToken(const char*& p) {
  p = skipSpaces(p);
  const char* start = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  return std::string(start, static_cast<std::size_t>(p - start));
}

// earth_nav.dat: "<code> <lat> <lon> <elev> <freq> <range> <extra> <ident> ...".
// We keep only NDB (2) and VOR (3); ident is the 8th token (4 numeric fields
// follow the longitude before it).
void parseNav(const std::string& path, std::vector<MapFeature>& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    char* end = nullptr;
    const long code = std::strtol(p, &end, 10);
    if (end == p) continue;  // header / blank line
    if (code == kRowTerminator) break;
    if (code != kRowNdb && code != kRowVor) continue;

    p = end;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;

    // Skip elevation, frequency, range, and the bias/variation field.
    for (int i = 0; i < 4; ++i) p = skipToken(p);
    const std::string ident = readToken(p);
    if (ident.empty()) continue;

    out.push_back({code == kRowVor ? MapFeatureType::Vor : MapFeatureType::Ndb,
                   lat, lon, ident});
  }
}

// earth_fix.dat: "<lat> <lon> <ident> ...". No row codes (one data type), so
// header lines are rejected by failing to parse a latitude/longitude pair.
void parseFix(const std::string& path, std::vector<MapFeature>& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    char* end = nullptr;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;  // "I"/"A" or other header line
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;  // version line ("1200 Version ...")
    p = end;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;  // e.g. "99"
    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    out.push_back({MapFeatureType::Fix, lat, lon, ident});
  }
}

}  // namespace

NavDataStore::NavDataStore() {
  thread_ = std::thread([this] { load(); });
}

NavDataStore::~NavDataStore() {
  if (thread_.joinable()) thread_.join();
}

void NavDataStore::load() {
  std::string dir;
  for (const std::string& root : readInstallRoots()) {
    dir = navDataDirForRoot(root);
    if (!dir.empty()) break;
  }
  if (dir.empty()) {
    loaded_.store(true, std::memory_order_release);  // nothing to load
    return;
  }

  parseNav(join(dir, "earth_nav.dat"), navaids_);
  parseFix(join(dir, "earth_fix.dat"), fixes_);
  sourceDir_ = dir;

  loaded_.store(true, std::memory_order_release);
}

std::vector<MapFeature> NavDataStore::nearby(double lat, double lon,
                                             float rangeNm,
                                             std::size_t maxCount) const {
  std::vector<MapFeature> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  // Generous bounding box (range + 20% margin) for a cheap first-pass reject.
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  struct Scored {
    MapFeature feature;
    double distSq;
  };
  std::vector<Scored> scored;

  auto consider = [&](const std::vector<MapFeature>& src) {
    for (const MapFeature& f : src) {
      if (std::fabs(f.lat - lat) > dLat) continue;
      if (std::fabs(f.lon - lon) > dLon) continue;
      const double north = (f.lat - lat) * kNmPerDeg;
      const double east = (f.lon - lon) * kNmPerDeg * cosLat;
      scored.push_back({f, north * north + east * east});
    }
  };
  consider(navaids_);
  consider(fixes_);

  std::sort(scored.begin(), scored.end(),
            [](const Scored& a, const Scored& b) { return a.distSq < b.distSq; });
  if (scored.size() > maxCount) scored.resize(maxCount);

  result.reserve(scored.size());
  for (const Scored& s : scored) result.push_back(s.feature);
  return result;
}

}  // namespace avionics
