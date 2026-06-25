#include "NavData.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include "avionics/nav/NearbyFeatureSelect.h"

namespace avionics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

// XP-NAV row codes we surface on the map; the rest (markers/DME/etc.) are
// skipped to avoid clutter at inset-map scale.
constexpr long kRowNdb = 2;
constexpr long kRowVor = 3;
constexpr long kRowIls = 4;
constexpr long kRowLoc = 5;
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

// Given an install root, return the path to earth_aptmeta.dat (airport
// lat/lon/ICAO index), preferring Custom Data over Resources/default data.
std::string aptMetaPathForRoot(const std::string& root) {
  const std::string candidates[] = {join(root, "Custom Data/earth_aptmeta.dat"),
                                      join(join(root, "Resources"),
                                           "default data/earth_aptmeta.dat")};
  for (const std::string& path : candidates) {
    if (fileExists(path)) return path;
  }
  return std::string();
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

// earth_aptmeta.dat: "<icao> <region> <lat> <lon> <elev> ..." per airport.
// XP12 no longer lists airports in earth_nav.dat; this compact index is the
// practical source for nearest-airport queries.
void parseAptMeta(const std::string& path, std::vector<MapFeature>& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == 'I') continue;  // header line
    const char* p = line.c_str();
    while (*p == ' ') ++p;
    if (!*p) continue;

    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    const std::string region = readToken(p);  // FAA region code (e.g. "K2")
    p = skipSpaces(p);
    char* end = nullptr;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;
    p = end;
    // Remaining columns: elevation (ft), usage type, longest runway (ft), ...
    const double elevFt = std::strtod(p, &end);
    if (end != p) p = end;
    const std::string usage = readToken(p);
    char* rwyEnd = nullptr;
    const double longestRwyFt = std::strtod(p, &rwyEnd);

    MapFeature apt;
    apt.type = MapFeatureType::Airport;
    apt.lat = lat;
    apt.lon = lon;
    apt.id = ident;
    apt.region = region;
    apt.elevationFt = static_cast<float>(elevFt);
    apt.longestRunwayFt =
        rwyEnd != p ? static_cast<int>(longestRwyFt) : 0;
    if (!usage.empty()) {
      const char u = static_cast<char>(
          std::toupper(static_cast<unsigned char>(usage[0])));
      if (u == 'P' || u == 'R') {
        apt.airportKind = AirportFacilityKind::Private;
      } else if (u == 'H') {
        apt.airportKind = AirportFacilityKind::Heliport;
      } else if (u == 'S' || u == 'W') {
        apt.airportKind = AirportFacilityKind::Seaplane;
      }
    }
    out.push_back(apt);
  }
}

// earth_nav.dat navaid rows: "<code> <lat> <lon> <elev> <freq> <range>
// <magvar> <ident> <airport|ENRT> <region> <name...>". We keep only NDB (2)
// and VOR (3).
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

    // Skip elevation, then read the frequency column: VOR rows carry it in
    // 10 kHz units (11390 == 113.90 MHz), NDB rows directly in kHz.
    p = skipToken(p);
    const double rawFreq = std::strtod(skipSpaces(p), &end);
    if (end != p) p = end;
    // Service-volume range (nm) and the slaved variation (+E/-W degrees).
    const double rangeNm = std::strtod(skipSpaces(p), &end);
    const bool hasRange = end != p;
    if (hasRange) p = end;
    const double magvar = std::strtod(skipSpaces(p), &end);
    const bool hasMagvar = end != p;
    if (hasMagvar) p = end;
    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    readToken(p);  // terminal airport or ENRT
    const std::string region = readToken(p);
    p = skipSpaces(p);
    const char* nameStart = p;
    while (*p && *p != '\r' && *p != '\n') ++p;
    std::string name(nameStart, static_cast<std::size_t>(p - nameStart));
    while (!name.empty() && (name.back() == ' ' || name.back() == '\t')) {
      name.pop_back();
    }

    MapFeature navaid;
    navaid.type = code == kRowVor ? MapFeatureType::Vor : MapFeatureType::Ndb;
    navaid.lat = lat;
    navaid.lon = lon;
    navaid.id = ident;
    navaid.region = region;
    navaid.frequency = code == kRowVor ? static_cast<float>(rawFreq / 100.0)
                                       : static_cast<float>(rawFreq);
    navaid.navaidType = splitNavaidTypeSuffix(name);
    if (navaid.navaidType.empty()) {
      navaid.navaidType = code == kRowVor ? "VOR" : "NDB";
    }
    navaid.name = name;
    navaid.rangeNm = hasRange ? static_cast<int>(rangeNm) : 0;
    if (code == kRowVor && hasMagvar) {
      navaid.magvarDeg = static_cast<float>(magvar);
      navaid.hasMagvar = true;
    }
    out.push_back(navaid);
  }
}

// earth_nav.dat ILS (4) and localizer (5) rows carry the published approach
// list for an airport: frequency, course, localizer ident, and runway.
void parseApproaches(
    const std::string& path,
    std::unordered_map<std::string, std::vector<MapApproach>>& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    char* end = nullptr;
    const long code = std::strtol(p, &end, 10);
    if (end == p) continue;
    if (code == kRowTerminator) break;
    if (code != kRowIls && code != kRowLoc) continue;

    p = end;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;

    p = skipToken(p);  // elevation
    const double rawFreq = std::strtod(skipSpaces(p), &end);
    if (end != p) p = end;
    p = skipToken(p);  // range
    const double course = std::strtod(skipSpaces(p), &end);
    if (end != p) p = end;
    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    readToken(p);  // associated approach id
    const std::string airport = readToken(p);
    if (airport.empty()) continue;
    readToken(p);  // region
    const std::string runway = readToken(p);
    if (runway.empty()) continue;

    MapApproach ap;
    ap.ident = ident;
    ap.airportIcao = airport;
    ap.runway = runway;
    ap.frequencyMhz = static_cast<float>(rawFreq / 100.0);
    ap.courseDeg = static_cast<float>(course);
    ap.hasGlideslope = (code == kRowIls);

    std::vector<MapApproach>& list = out[airport];
    // Prefer the ILS row when both ILS and LOC exist for the same runway.
    bool merged = false;
    for (MapApproach& existing : list) {
      if (existing.runway != runway) continue;
      if (code == kRowIls || !existing.hasGlideslope) {
        existing = ap;
      }
      merged = true;
      break;
    }
    if (!merged) list.push_back(std::move(ap));
  }
}

// earth_fix.dat: "<lat> <lon> <ident> <airport|ENRT> <region> ...". No row
// codes (one data type), so header lines are rejected by failing to parse a
// latitude/longitude pair.
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
    readToken(p);  // terminal airport or ENRT
    const std::string region = readToken(p);
    MapFeature fix;
    fix.type = MapFeatureType::Fix;
    fix.lat = lat;
    fix.lon = lon;
    fix.id = ident;
    fix.region = region;
    out.push_back(std::move(fix));
  }
}

// The version header of the XP-NAV files carries the AIRAC cycle, e.g.
// "1200 Version - data cycle 2506, build 20250504, metadata NavXP1200...".
// Returns the YYCC cycle number, or 0 when no header line carries one.
int parseDataCycle(const std::string& path) {
  std::ifstream in(path);
  if (!in.good()) return 0;
  constexpr const char* kTag = "data cycle ";
  std::string line;
  for (int i = 0; i < 4 && std::getline(in, line); ++i) {
    const std::size_t pos = line.find(kTag);
    if (pos == std::string::npos) continue;
    return static_cast<int>(
        std::strtol(line.c_str() + pos + std::strlen(kTag), nullptr, 10));
  }
  return 0;
}

}  // namespace

NavDataStore::NavDataStore() {
  thread_ = std::thread([this] { load(); });
}

NavDataStore::~NavDataStore() {
  if (thread_.joinable()) thread_.join();
}

void NavDataStore::load() {
  std::string root;
  std::string dir;
  for (const std::string& r : readInstallRoots()) {
    dir = navDataDirForRoot(r);
    if (!dir.empty()) {
      root = r;
      break;
    }
  }
  if (dir.empty()) {
    loaded_.store(true, std::memory_order_release);  // nothing to load
    return;
  }

  const std::string navPath = join(dir, "earth_nav.dat");
  parseNav(navPath, navaids_);
  parseApproaches(navPath, approachesByAirport_);
  parseFix(join(dir, "earth_fix.dat"), fixes_);
  dbInfo_ = navDatabaseInfoForCycle(parseDataCycle(navPath));

  const std::string aptMeta = aptMetaPathForRoot(root);
  if (!aptMeta.empty()) parseAptMeta(aptMeta, airports_);

  // Ident-sorted index over every feature for the FMS entry lookups. Stable
  // sort keeps airports and navaids ahead of same-ident fixes (the order they
  // are appended), so a spelled "OAK" resolves to the VOR before the fix.
  identIndex_.reserve(airports_.size() + navaids_.size() + fixes_.size());
  for (const std::vector<MapFeature>* src : {&airports_, &navaids_, &fixes_}) {
    for (const MapFeature& f : *src) identIndex_.push_back(&f);
  }
  std::stable_sort(identIndex_.begin(), identIndex_.end(),
                   [](const MapFeature* a, const MapFeature* b) {
                     return a->id < b->id;
                   });

  sourceDir_ = dir;
  loaded_.store(true, std::memory_order_release);
}

std::vector<MapFeature> NavDataStore::lookupIdent(const std::string& ident,
                                                  std::size_t maxCount) const {
  std::vector<MapFeature> result;
  if (!loaded() || ident.empty() || maxCount == 0) return result;
  auto it = std::lower_bound(identIndex_.begin(), identIndex_.end(), ident,
                             [](const MapFeature* f, const std::string& id) {
                               return f->id < id;
                             });
  for (; it != identIndex_.end() && (*it)->id == ident; ++it) {
    if (result.size() >= maxCount) break;
    result.push_back(**it);
  }
  return result;
}

std::vector<MapFeature> NavDataStore::lookupIdentNear(
    const std::string& ident, double refLat, double refLon,
    std::size_t maxCount) const {
  std::vector<MapFeature> matches = lookupIdent(ident, maxCount);
  if (matches.size() <= 1) return matches;

  const double cosLat = std::max(0.05, std::cos(refLat * kDegToRad));
  std::sort(matches.begin(), matches.end(),
            [&](const MapFeature& a, const MapFeature& b) {
              const double dLatA = a.lat - refLat;
              const double dLonA = (a.lon - refLon) * cosLat;
              const double dLatB = b.lat - refLat;
              const double dLonB = (b.lon - refLon) * cosLat;
              return (dLatA * dLatA + dLonA * dLonA) <
                     (dLatB * dLatB + dLonB * dLonB);
            });
  return matches;
}

std::vector<MapApproach> NavDataStore::approachesForAirport(
    const std::string& icao) const {
  if (!loaded() || icao.empty()) return {};
  const auto it = approachesByAirport_.find(icao);
  if (it == approachesByAirport_.end()) return {};
  return it->second;
}

std::string NavDataStore::firstIdentWithPrefix(const std::string& prefix) const {
  if (!loaded() || prefix.empty()) return {};
  auto it = std::lower_bound(identIndex_.begin(), identIndex_.end(), prefix,
                             [](const MapFeature* f, const std::string& id) {
                               return f->id < id;
                             });
  if (it == identIndex_.end()) return {};
  const std::string& id = (*it)->id;
  if (id.compare(0, prefix.size(), prefix) != 0) return {};
  return id;
}

std::vector<MapFeature> NavDataStore::nearby(double lat, double lon,
                                             float rangeNm,
                                             std::size_t maxCount) const {
  if (!loaded() || maxCount == 0) return {};
  return assembleNearbyMapFeatures(airports_, navaids_, fixes_, lat, lon,
                                   rangeNm, maxCount);
}

}  // namespace avionics
