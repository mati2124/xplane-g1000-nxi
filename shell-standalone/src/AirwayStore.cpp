#include "AirwayStore.h"

#include "XPlaneInstall.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace avionics {
namespace {

constexpr double kNmPerDeg = 60.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// earth_awy.dat row terminator code.
constexpr long kRowTerminator = 99;

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

struct LatLon {
  double lat = 0.0;
  double lon = 0.0;
};

using PointTable = std::unordered_map<std::string, LatLon>;

// Composite key for the fix/navaid coordinate table. Airway rows identify an
// endpoint by ident + ICAO region, which is unique enough in practice.
std::string pointKey(const std::string& ident, const std::string& region) {
  return ident + '|' + region;
}

// earth_fix.dat: "<lat> <lon> <ident> <terminal area> <region> ...".
void indexFixes(const std::string& path, PointTable& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    char* end = nullptr;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;  // header line
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;  // "99"
    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    readToken(p);  // terminal area ("ENRT" for enroute fixes)
    const std::string region = readToken(p);
    out.emplace(pointKey(ident, region), LatLon{lat, lon});
  }
}

// earth_nav.dat: "<code> <lat> <lon> <elev> <freq> <range> <bias> <ident>
// <terminal area> <region> <name...>". All navaid classes are indexed; airways
// reference VORs and NDBs.
void indexNavaids(const std::string& path, PointTable& out) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    char* end = nullptr;
    const long code = std::strtol(p, &end, 10);
    if (end == p) continue;
    if (code == kRowTerminator) break;
    if (code < 2 || code > 13) continue;
    p = end;
    const double lat = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;
    // Skip elevation, frequency, range, bias to reach the ident.
    for (int i = 0; i < 4; ++i) readToken(p);
    const std::string ident = readToken(p);
    if (ident.empty()) continue;
    readToken(p);  // terminal area
    const std::string region = readToken(p);
    out.emplace(pointKey(ident, region), LatLon{lat, lon});
  }
}

static std::string airwayPrimaryName(const std::string& names) {
  const std::size_t dash = names.find('-');
  return dash == std::string::npos ? names : names.substr(0, dash);
}

static void addGraphEdge(
    std::unordered_map<std::string,
                       std::vector<std::pair<std::string, std::string>>>& graph,
    std::unordered_set<std::string>& airwayNames, const std::string& fromKey,
    const std::string& toKey, const std::string& airwayName) {
  if (fromKey.empty() || toKey.empty() || airwayName.empty()) return;
  airwayNames.insert(airwayName);
  graph[fromKey].push_back({toKey, airwayName});
}

// earth_awy.dat 1100-format rows:
//   <id1> <region1> <type1> <id2> <region2> <type2> <dir> <class> <base>
//   <top> <name(s)>
// where class is 1 = low/victor, 2 = high/jet, and name(s) may list several
// dash-separated airways sharing the segment (e.g. "J15-J52").
void parseAirways(
    const std::string& path, const PointTable& points,
    std::vector<MapAirwaySegment>& out,
    std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>&
        graph,
    std::unordered_set<std::string>& airwayNames) {
  std::ifstream in(path);
  if (!in.good()) return;
  std::string line;
  while (std::getline(in, line)) {
    const char* p = line.c_str();

    const std::string id1 = readToken(p);
    if (id1.empty() || id1 == "I" || id1 == "A") continue;  // header lines
    if (id1 == "99") break;
    // Version line ("1100 Version - data cycle ...").
    if (id1.size() == 4 && std::isdigit(static_cast<unsigned char>(id1[0])) &&
        line.find("Version") != std::string::npos) {
      continue;
    }

    const std::string region1 = readToken(p);
    readToken(p);  // type1
    const std::string id2 = readToken(p);
    const std::string region2 = readToken(p);
    readToken(p);  // type2
    const std::string dir = readToken(p);  // directionality (N/F/B)
    const std::string cls = readToken(p);
    readToken(p);  // base FL
    readToken(p);  // top FL
    std::string names = readToken(p);
    if (id2.empty() || cls.empty()) continue;

    const auto a = points.find(pointKey(id1, region1));
    const auto b = points.find(pointKey(id2, region2));
    if (a == points.end() || b == points.end()) continue;

      const std::string primary = airwayPrimaryName(names);

    const std::string keyA = pointKey(id1, region1);
    const std::string keyB = pointKey(id2, region2);
    if (dir != "B") addGraphEdge(graph, airwayNames, keyA, keyB, primary);
    if (dir != "F") addGraphEdge(graph, airwayNames, keyB, keyA, primary);

    MapAirwaySegment seg;
    seg.level = cls == "2" ? AirwayLevel::High : AirwayLevel::Low;
    seg.name = primary;
    seg.a = {a->second.lat, a->second.lon};
    seg.b = {b->second.lat, b->second.lon};
    out.push_back(std::move(seg));
  }
}

static bool looksLikeAirwayIdent(const std::string& name) {
  if (name.size() < 2) return false;
  const char lead = static_cast<char>(std::toupper(
      static_cast<unsigned char>(name[0])));
  if (lead != 'V' && lead != 'J' && lead != 'Q' && lead != 'T' &&
      lead != 'A') {
    return false;
  }
  return std::isdigit(static_cast<unsigned char>(name[1])) != 0;
}

}  // namespace

AirwayStore::AirwayStore() {
  thread_ = std::thread([this] { load(); });
}

AirwayStore::~AirwayStore() {
  if (thread_.joinable()) thread_.join();
}

void AirwayStore::load() {
  std::string dir;
  for (const std::string& root : xplane_install::readInstallRoots()) {
    const std::string candidate = xplane_install::navDataDirForRoot(root);
    if (!candidate.empty() && fileExists(join(candidate, "earth_awy.dat"))) {
      dir = candidate;
      break;
    }
  }
  if (dir.empty()) {
    loaded_.store(true, std::memory_order_release);  // nothing to load
    return;
  }

  // Coordinate table first (fixes + navaids), then resolve the airway graph.
  PointTable points;
  indexFixes(join(dir, "earth_fix.dat"), points);
  indexNavaids(join(dir, "earth_nav.dat"), points);
  for (const auto& kv : points) {
    points_[kv.first] = {kv.second.lat, kv.second.lon};
  }
  parseAirways(join(dir, "earth_awy.dat"), points, segments_, graph_,
               airwayNames_);

  sourcePath_ = join(dir, "earth_awy.dat");
  loaded_.store(true, std::memory_order_release);
}

std::vector<MapAirwaySegment> AirwayStore::nearby(double lat, double lon,
                                                  float rangeNm,
                                                  std::size_t maxCount) const {
  std::vector<MapAirwaySegment> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  for (const MapAirwaySegment& seg : segments_) {
    // Keep the segment when its bounding box overlaps the query box, so long
    // edges crossing the view without an endpoint inside still draw.
    const double minLat = std::min(seg.a.lat, seg.b.lat) - dLat;
    const double maxLat = std::max(seg.a.lat, seg.b.lat) + dLat;
    const double minLon = std::min(seg.a.lon, seg.b.lon) - dLon;
    const double maxLon = std::max(seg.a.lon, seg.b.lon) + dLon;
    if (lat < minLat || lat > maxLat || lon < minLon || lon > maxLon) continue;
    result.push_back(seg);
    if (result.size() >= maxCount) break;
  }
  return result;
}

bool AirwayStore::isAirwayName(const std::string& name) const {
  if (!loaded() || name.empty()) return false;
  if (airwayNames_.count(name) != 0) return true;
  return looksLikeAirwayIdent(name);
}

std::vector<MapLeg> AirwayStore::expandAirway(const std::string& airwayName,
                                              const std::string& fromIdent,
                                              const std::string& toIdent) const {
  std::vector<MapLeg> result;
  if (!loaded() || airwayName.empty() || fromIdent.empty() || toIdent.empty()) {
    return result;
  }

  auto keysForIdent = [&](const std::string& ident) {
    std::vector<std::string> keys;
    const std::string prefix = ident + '|';
    for (const auto& kv : points_) {
      if (kv.first.compare(0, prefix.size(), prefix) == 0) keys.push_back(kv.first);
    }
    return keys;
  };

  const std::vector<std::string> fromKeys = keysForIdent(fromIdent);
  const std::vector<std::string> toKeys = keysForIdent(toIdent);
  if (fromKeys.empty() || toKeys.empty()) return result;

  std::unordered_set<std::string> goal;
  for (const std::string& k : toKeys) goal.insert(k);

  std::unordered_set<std::string> fromSet(fromKeys.begin(), fromKeys.end());
  std::unordered_map<std::string, std::string> parent;
  std::queue<std::string> q;
  for (const std::string& k : fromKeys) {
    parent.emplace(k, std::string{});
    q.push(k);
  }

  std::string endKey;
  bool found = false;
  while (!q.empty() && !found) {
    const std::string cur = q.front();
    q.pop();
    const auto it = graph_.find(cur);
    if (it == graph_.end()) continue;
    for (const auto& edge : it->second) {
      if (edge.second != airwayName) continue;
      if (parent.count(edge.first) != 0) continue;
      parent[edge.first] = cur;
      if (goal.count(edge.first) != 0) {
        endKey = edge.first;
        found = true;
        break;
      }
      q.push(edge.first);
    }
  }
  if (!found) return result;

  std::vector<std::string> pathKeys;
  for (std::string k = endKey;;) {
    pathKeys.push_back(k);
    if (fromSet.count(k) != 0) break;
    const auto pit = parent.find(k);
    if (pit == parent.end() || pit->second.empty()) return {};
    k = pit->second;
  }
  std::reverse(pathKeys.begin(), pathKeys.end());

  // Drop the from-ident endpoint; keep interior fixes and the to-ident.
  if (pathKeys.size() < 2) return result;
  for (std::size_t i = 1; i < pathKeys.size(); ++i) {
    const auto pt = points_.find(pathKeys[i]);
    if (pt == points_.end()) continue;
    const std::size_t sep = pathKeys[i].find('|');
    MapLeg leg;
    leg.id = sep == std::string::npos ? pathKeys[i] : pathKeys[i].substr(0, sep);
    leg.lat = pt->second.first;
    leg.lon = pt->second.second;
    result.push_back(std::move(leg));
  }
  return result;
}

}  // namespace avionics
