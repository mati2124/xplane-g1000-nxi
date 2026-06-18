#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "avionics/MapRange.h"
#include "avionics/render/MapSymbols.h"

namespace avionics::mapview {
namespace {

constexpr float kFeatureRangeVorNm = 100.0f;
constexpr float kFeatureRangeNdbNm = 40.0f;
constexpr float kFeatureRangeFixNm = 7.5f;
constexpr int kMaxFixesDrawn = 40;
constexpr int kLargeAirportRunwayFt = 8100;
constexpr int kMediumAirportRunwayFt = 5000;

bool visibleAtRange(MapFeatureType type, float rangeNm) {
  switch (type) {
    case MapFeatureType::Airport:
      return true;
    case MapFeatureType::Vor:
      return rangeNm <= kFeatureRangeVorNm;
    case MapFeatureType::Ndb:
      return rangeNm <= kFeatureRangeNdbNm;
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      return rangeNm <= kFeatureRangeFixNm;
  }
  return true;
}

bool airportVisible(const MapFeature& f, const MapViewConfig& config,
                    float rangeNm) {
  if (f.longestRunwayFt >= kLargeAirportRunwayFt) {
    return config.style.showLargeAirports &&
           rangeNm <= config.style.largeAirportRangeNm;
  }
  if (f.longestRunwayFt >= kMediumAirportRunwayFt || f.airportTowered) {
    return config.style.showMediumAirports &&
           rangeNm <= config.style.mediumAirportRangeNm;
  }
  return config.style.showSmallAirports &&
         rangeNm <= config.style.smallAirportRangeNm;
}

int airportImportance(const MapFeature& f) {
  int score = f.longestRunwayFt;
  if (f.airportTowered) score += 2000;
  if (f.airportServiced) score += 500;
  return score;
}

// At regional scale the NXi draws only the highest-priority airports that pass
// the Map Setup size-class gates, not every towered field in the query radius.
std::unordered_set<std::size_t> rankedAirportDrawSet(
    const MapData& map, const MapViewConfig& config, float rangeNm) {
  struct Cand {
    int score = 0;
    std::size_t idx = 0;
  };
  std::vector<Cand> candidates;
  candidates.reserve(64);
  for (std::size_t i = 0; i < map.features.size(); ++i) {
    const MapFeature& f = map.features[i];
    if (f.type != MapFeatureType::Airport) continue;
    if (!airportVisible(f, config, rangeNm)) continue;
    candidates.push_back({airportImportance(f), i});
  }

  std::unordered_set<std::size_t> allowed;
  allowed.reserve(candidates.size());
  if (rangeNm < kAirportImportanceBudgetMinNm ||
      static_cast<int>(candidates.size()) <= kAirportImportanceBudget) {
    for (const Cand& c : candidates) allowed.insert(c.idx);
    return allowed;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Cand& a, const Cand& b) {
              if (a.score != b.score) return a.score > b.score;
              return a.idx < b.idx;
            });
  const std::size_t keep =
      std::min(candidates.size(),
               static_cast<std::size_t>(kAirportImportanceBudget));
  for (std::size_t i = 0; i < keep; ++i) allowed.insert(candidates[i].idx);
  return allowed;
}

bool featureVisible(const MapFeature& f, const MapViewConfig& config,
                    float rangeNm) {
  if (f.type == MapFeatureType::Airport) {
    return airportVisible(f, config, rangeNm);
  }
  return visibleAtRange(f.type, rangeNm);
}

bool drawFeature(const MapFeature& f, std::size_t featureIdx,
                 const std::unordered_set<std::size_t>& airportDrawSet,
                 const MapViewConfig& config, float rangeNm) {
  if (!featureVisible(f, config, rangeNm)) return false;
  if (f.type == MapFeatureType::Airport &&
      airportDrawSet.find(featureIdx) == airportDrawSet.end()) {
    return false;
  }
  return true;
}

}  // namespace

void drawNavFeatures(Renderer& r, const MapData& map, const Proj& proj,
                     const MapViewConfig& config, float rangeNm, float symSize) {
  if (rangeNm > kContinentalChartRangeNm) return;

  const std::unordered_set<std::size_t> airportDrawSet =
      rankedAirportDrawSet(map, config, rangeNm);

  int fixesDrawn = 0;
  for (std::size_t i = 0; i < map.features.size(); ++i) {
    const MapFeature& f = map.features[i];
    if (!drawFeature(f, i, airportDrawSet, config, rangeNm)) continue;
    const bool isFix =
        f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
    if (isFix && (!config.style.showFixes || fixesDrawn >= kMaxFixesDrawn)) {
      continue;
    }

    float x = 0.0f, y = 0.0f;
    proj.toPx(f.lat, f.lon, x, y);
    if (x < config.x - symSize || x > config.x + config.w + symSize ||
        y < config.y - symSize || y > config.y + config.h + symSize) {
      continue;
    }
    if (isFix) ++fixesDrawn;
    drawMapFeatureSymbol(r, f, x, y, symSize);
  }
}

void drawNavFeatureLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, float rangeNm,
                          float symSize, float labelSize) {
  if (rangeNm > kContinentalChartRangeNm || !config.style.showLabels) return;

  const std::unordered_set<std::size_t> airportDrawSet =
      rankedAirportDrawSet(map, config, rangeNm);

  int fixesDrawn = 0;
  for (std::size_t i = 0; i < map.features.size(); ++i) {
    const MapFeature& f = map.features[i];
    if (!drawFeature(f, i, airportDrawSet, config, rangeNm)) continue;
    const bool isFix =
        f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
    if (isFix && (!config.style.showFixes || fixesDrawn >= kMaxFixesDrawn)) {
      continue;
    }
    if (f.id.empty()) continue;

    float x = 0.0f, y = 0.0f;
    proj.toPx(f.lat, f.lon, x, y);
    if (x < config.x - symSize || x > config.x + config.w + symSize ||
        y < config.y - symSize || y > config.y + config.h + symSize) {
      continue;
    }
    if (isFix) ++fixesDrawn;
    const float textSize = labelSize * kMapIdentLabelScale;
    const float labelY =
        (f.type == MapFeatureType::Airport ? y - symSize * 1.25f
                                           : y - symSize * 1.12f) -
        kMapLabelLiftPx;
    r.fillText(x, labelY, f.id, textSize, TextAlign::Center, colors::kWhite,
               kMapLabelFace);
  }
}

}  // namespace avionics::mapview
