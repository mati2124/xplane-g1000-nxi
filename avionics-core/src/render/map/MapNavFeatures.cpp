#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "avionics/MapRange.h"
#include "avionics/render/MapSymbols.h"

namespace avionics::mapview {
namespace {

constexpr float kFeatureRangeVorNm = 100.0f;
constexpr float kFeatureRangeNdbNm = 40.0f;
constexpr float kFeatureRangeFixNm = kFixMaxRangeNm;
constexpr int kLargeAirportRunwayFt = 8100;
constexpr int kMediumAirportRunwayFt = 5000;

bool textRectsOverlap(const TextRect& a, const TextRect& b, float pad) {
  return !(a.right + pad < b.left || b.right + pad < a.left ||
           a.bottom + pad < b.top || b.bottom + pad < a.top);
}

TextRect expandTextRect(const TextRect& tr, float pad) {
  return {tr.left - pad, tr.top - pad, tr.right + pad, tr.bottom + pad};
}

struct FixLabelCandidate {
  std::string id;
  float x = 0.0f;
  float y = 0.0f;
  float distSq = 0.0f;
};

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

// Flight-plan, direct-to, and procedure-preview layers draw their own idents
// on top of nav symbology; skip the underlying nav symbol and white label when
// one of those overlays already marks the same fix.
std::unordered_set<std::string> routeOverlayLabelIds(
    const MapData& map, const MapViewConfig& config) {
  std::unordered_set<std::string> ids;
  if (config.style.showFlightPlan) {
    if (map.flightPlan.size() >= 2) {
      for (const MapLeg& leg : map.flightPlan) {
        if (!leg.id.empty()) ids.insert(leg.id);
      }
    }
    if (map.directToActive && map.positionValid && !map.directTo.id.empty()) {
      ids.insert(map.directTo.id);
    }
  }
  if (config.procedurePreview != nullptr &&
      config.procedurePreview->size() >= 2) {
    for (const MapLeg& leg : *config.procedurePreview) {
      if (!leg.id.empty()) ids.insert(leg.id);
    }
  }
  return ids;
}

}  // namespace

void drawNavFeatures(Renderer& r, const MapData& map, const Proj& proj,
                     const MapViewConfig& config, float rangeNm, float symSize) {
  if (rangeNm > kContinentalChartRangeNm) return;

  const std::unordered_set<std::size_t> airportDrawSet =
      rankedAirportDrawSet(map, config, rangeNm);
  const std::unordered_set<std::string> routeOverlayIds =
      routeOverlayLabelIds(map, config);

  for (std::size_t i = 0; i < map.features.size(); ++i) {
    const MapFeature& f = map.features[i];
    if (!drawFeature(f, i, airportDrawSet, config, rangeNm)) continue;
    if (!f.id.empty() && routeOverlayIds.find(f.id) != routeOverlayIds.end()) {
      continue;
    }
    const bool isFix =
        f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
    if (isFix && !config.style.showFixes) continue;
    const bool isNavaid =
        f.type == MapFeatureType::Vor || f.type == MapFeatureType::Ndb;
    if (isNavaid && !config.style.showNavaids) continue;

    float x = 0.0f, y = 0.0f;
    proj.toPx(f.lat, f.lon, x, y);
    if (x < config.x - symSize || x > config.x + config.w + symSize ||
        y < config.y - symSize || y > config.y + config.h + symSize) {
      continue;
    }
    drawMapFeatureSymbol(r, f, x, y, symSize);
  }
}

void drawNavFeatureLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, float rangeNm,
                          float symSize, float labelSize) {
  if (rangeNm > kContinentalChartRangeNm || !config.style.showLabels) return;

  const std::unordered_set<std::size_t> airportDrawSet =
      rankedAirportDrawSet(map, config, rangeNm);
  const std::unordered_set<std::string> routeLabelIds =
      routeOverlayLabelIds(map, config);

  const float textSize = labelSize * kMapIdentLabelScale;
  std::vector<TextRect> placedFixLabels;
  std::vector<FixLabelCandidate> fixLabels;
  fixLabels.reserve(128);

  for (std::size_t i = 0; i < map.features.size(); ++i) {
    const MapFeature& f = map.features[i];
    if (!drawFeature(f, i, airportDrawSet, config, rangeNm)) continue;
    const bool isFix =
        f.type == MapFeatureType::Fix || f.type == MapFeatureType::Waypoint;
    if (isFix && !config.style.showFixes) continue;
    const bool isNavaid =
        f.type == MapFeatureType::Vor || f.type == MapFeatureType::Ndb;
    if (isNavaid && !config.style.showNavaids) continue;
    if (f.id.empty()) continue;
    if (routeLabelIds.find(f.id) != routeLabelIds.end()) continue;

    float x = 0.0f, y = 0.0f;
    proj.toPx(f.lat, f.lon, x, y);
    if (x < config.x - symSize || x > config.x + config.w + symSize ||
        y < config.y - symSize || y > config.y + config.h + symSize) {
      continue;
    }

    if (isFix) {
      const float dx = x - proj.cx;
      const float dy = y - proj.cy;
      fixLabels.push_back({f.id, x, y, dx * dx + dy * dy});
      continue;
    }

    const float labelY =
        (f.type == MapFeatureType::Airport ? y - symSize * 1.25f
                                           : y - symSize * 1.12f) -
        kMapLabelLiftPx;
    r.fillText(x, labelY, f.id, textSize, TextAlign::Center, colors::kWhite,
               kMapLabelFace);
  }

  // The PC Trainer labels only a subset of fix symbols: idents that fit without
  // overlapping other fix labels (flight-plan idents are drawn separately).
  std::sort(fixLabels.begin(), fixLabels.end(),
            [](const FixLabelCandidate& a, const FixLabelCandidate& b) {
              return a.distSq < b.distSq;
            });
  placedFixLabels.reserve(fixLabels.size());
  const float labelPad = textSize * 0.18f;
  for (const FixLabelCandidate& fix : fixLabels) {
    const float labelY = fix.y - symSize * 1.12f - kMapLabelLiftPx;
    const TextRect tr = r.measureTextRect(fix.x, labelY, fix.id, textSize,
                                          TextAlign::Center, kMapLabelFace);
    bool clash = false;
    for (const TextRect& placed : placedFixLabels) {
      if (textRectsOverlap(tr, placed, labelPad)) {
        clash = true;
        break;
      }
    }
    if (clash) continue;
    r.fillText(fix.x, labelY, fix.id, textSize, TextAlign::Center,
               colors::kWhite, kMapLabelFace);
    placedFixLabels.push_back(expandTextRect(tr, labelPad));
  }
}

}  // namespace avionics::mapview
