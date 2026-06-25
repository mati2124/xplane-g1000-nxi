#include "avionics/nav/NearbyFeatureSelect.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace avionics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

struct ScoredFeature {
  MapFeature feature;
  double distSq = 0.0;
};

std::vector<ScoredFeature> scoreFeaturesInRange(
    const std::vector<MapFeature>& src, double lat, double lon, float rangeNm) {
  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  std::vector<ScoredFeature> scored;
  scored.reserve(src.size());
  for (const MapFeature& f : src) {
    if (std::fabs(f.lat - lat) > dLat) continue;
    if (std::fabs(f.lon - lon) > dLon) continue;
    const double north = (f.lat - lat) * kNmPerDeg;
    const double east = (f.lon - lon) * kNmPerDeg * cosLat;
    scored.push_back({f, north * north + east * east});
  }
  std::sort(scored.begin(), scored.end(),
            [](const ScoredFeature& a, const ScoredFeature& b) {
              return a.distSq < b.distSq;
            });
  return scored;
}

void appendNearest(std::vector<MapFeature>& result,
                   const std::vector<ScoredFeature>& scored, std::size_t cap,
                   std::size_t maxCount) {
  for (const ScoredFeature& s : scored) {
    if (result.size() >= maxCount || cap == 0) return;
    result.push_back(s.feature);
    --cap;
  }
}

}  // namespace

std::vector<MapFeature> selectNearbyFixes(
    const std::vector<MapFeature>& fixes, double lat, double lon, float rangeNm,
    std::size_t maxCount) {
  std::vector<MapFeature> result;
  if (maxCount == 0 || fixes.empty()) return result;

  const std::vector<ScoredFeature> scored =
      scoreFeaturesInRange(fixes, lat, lon, rangeNm);
  result.reserve(std::min(scored.size(), maxCount));
  for (const ScoredFeature& s : scored) {
    if (s.feature.type != MapFeatureType::Fix &&
        s.feature.type != MapFeatureType::Waypoint) {
      continue;
    }
    result.push_back(s.feature);
    if (result.size() >= maxCount) break;
  }
  return result;
}

std::vector<MapFeature> assembleNearbyMapFeatures(
    const std::vector<MapFeature>& airports,
    const std::vector<MapFeature>& navaids,
    const std::vector<MapFeature>& fixes, double lat, double lon, float rangeNm,
    std::size_t maxCount) {
  std::vector<MapFeature> result;
  if (maxCount == 0) return result;

  // Reserve airports and navaids first so dense fix databases do not crowd
  // them out of the map feature budget (important for NRST lists). The airport
  // reserve must be deep enough that a wide MFD MAP view (out to 150 NM) still
  // receives the far airports, not just the nearest cluster -- the renderer
  // then declutters them per-size against the Map Setup ranges.
  constexpr std::size_t kMaxAirports = 200;
  constexpr std::size_t kMaxNavaids = 100;

  appendNearest(result, scoreFeaturesInRange(airports, lat, lon, rangeNm),
                kMaxAirports, maxCount);
  appendNearest(result, scoreFeaturesInRange(navaids, lat, lon, rangeNm),
                kMaxNavaids, maxCount);

  const std::size_t fixBudget =
      result.size() < maxCount ? maxCount - result.size() : 0;
  if (fixBudget == 0) return result;

  // Every fix in range gets an icon; nearest-first so a dense database trims
  // the farthest fixes if it overruns the budget. The renderer declutters the
  // idents, not the symbols.
  std::vector<MapFeature> nearbyFixes =
      selectNearbyFixes(fixes, lat, lon, rangeNm, fixBudget);
  for (const MapFeature& f : nearbyFixes) {
    result.push_back(f);
  }
  return result;
}

std::vector<MapFeature> assembleNearbyMapFeaturesMixed(
    const std::vector<MapFeature>& src, double lat, double lon, float rangeNm,
    std::size_t maxCount) {
  std::vector<MapFeature> airports;
  std::vector<MapFeature> navaids;
  std::vector<MapFeature> fixes;
  airports.reserve(256);
  navaids.reserve(128);
  fixes.reserve(512);
  for (const MapFeature& f : src) {
    switch (f.type) {
      case MapFeatureType::Airport:
        airports.push_back(f);
        break;
      case MapFeatureType::Vor:
      case MapFeatureType::Ndb:
        navaids.push_back(f);
        break;
      case MapFeatureType::Fix:
      case MapFeatureType::Waypoint:
        fixes.push_back(f);
        break;
      default:
        break;
    }
  }
  return assembleNearbyMapFeatures(airports, navaids, fixes, lat, lon, rangeNm,
                                   maxCount);
}

}  // namespace avionics
