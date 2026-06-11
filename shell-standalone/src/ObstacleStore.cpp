#include "ObstacleStore.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <utility>

namespace avionics {
namespace {

constexpr double kNmPerDeg = 60.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

int cellKey(double lat, double lon) {
  const int la = static_cast<int>(std::floor(lat)) + 90;
  const int lo = static_cast<int>(std::floor(lon)) + 180;
  return la * 360 + lo;
}

std::string trimUpper(const std::string& s) {
  std::size_t b = 0;
  std::size_t e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  std::string out = s.substr(b, e - b);
  for (char& c : out) c = static_cast<char>(std::toupper(
      static_cast<unsigned char>(c)));
  return out;
}

// Splits a CSV row. The FAA strips embedded commas from the free-text fields
// ("Extra commas have been removed from the city name", DDOF README), so a
// plain comma split is sufficient.
void splitCsv(const std::string& line, std::vector<std::string>& out) {
  out.clear();
  std::size_t start = 0;
  for (;;) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      out.push_back(line.substr(start));
      return;
    }
    out.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
}

}  // namespace

ObstacleStore::ObstacleStore(std::string path) : path_(std::move(path)) {
  thread_ = std::thread([this] { load(); });
}

ObstacleStore::~ObstacleStore() {
  if (thread_.joinable()) thread_.join();
}

void ObstacleStore::load() {
  if (path_.empty()) {
    loaded_.store(true, std::memory_order_release);
    return;
  }
  std::ifstream in(path_);
  std::string line;
  std::vector<std::string> fields;

  // Header row: locate the columns we need by name, so reordered or extended
  // future DOF revisions keep working.
  int colLat = -1, colLon = -1, colAgl = -1, colAmsl = -1;
  if (std::getline(in, line)) {
    splitCsv(line, fields);
    for (std::size_t i = 0; i < fields.size(); ++i) {
      const std::string name = trimUpper(fields[i]);
      const int idx = static_cast<int>(i);
      if (name == "LATDEC") colLat = idx;
      if (name == "LONDEC") colLon = idx;
      if (name == "AGL") colAgl = idx;
      if (name == "AMSL") colAmsl = idx;
    }
  }
  if (colLat < 0 || colLon < 0 || colAgl < 0 || colAmsl < 0) {
    loaded_.store(true, std::memory_order_release);  // not a DOF CSV
    return;
  }

  const int needed = std::max(std::max(colLat, colLon),
                              std::max(colAgl, colAmsl));
  while (std::getline(in, line)) {
    splitCsv(line, fields);
    if (static_cast<int>(fields.size()) <= needed) continue;
    char* end = nullptr;
    const double lat = std::strtod(fields[colLat].c_str(), &end);
    if (end == fields[colLat].c_str()) continue;
    const double lon = std::strtod(fields[colLon].c_str(), &end);
    if (end == fields[colLon].c_str()) continue;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;

    MapObstacle ob;
    ob.lat = lat;
    ob.lon = lon;
    ob.aglFt = static_cast<float>(std::atof(fields[colAgl].c_str()));
    ob.mslFt = static_cast<float>(std::atof(fields[colAmsl].c_str()));
    cells_[cellKey(lat, lon)].push_back(ob);
  }

  loaded_.store(true, std::memory_order_release);
}

std::vector<MapObstacle> ObstacleStore::nearby(double lat, double lon,
                                               float rangeNm,
                                               std::size_t maxCount) const {
  std::vector<MapObstacle> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  const int la0 = static_cast<int>(std::floor(lat - dLat));
  const int la1 = static_cast<int>(std::floor(lat + dLat));
  const int lo0 = static_cast<int>(std::floor(lon - dLon));
  const int lo1 = static_cast<int>(std::floor(lon + dLon));

  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      const auto it = cells_.find((la + 90) * 360 + (lo + 180));
      if (it == cells_.end()) continue;
      for (const MapObstacle& ob : it->second) {
        if (std::fabs(ob.lat - lat) > dLat || std::fabs(ob.lon - lon) > dLon) {
          continue;
        }
        result.push_back(ob);
        if (result.size() >= maxCount) return result;
      }
    }
  }
  return result;
}

}  // namespace avionics
