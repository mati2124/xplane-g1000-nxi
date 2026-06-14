#include "render/map/MapViewInternal.h"

#include "avionics/render/MapSymbols.h"

namespace avionics::mapview {
namespace {

// Range-based feature declutter (NM): each class is hidden once the map range
// exceeds its threshold, matching the G1000's progressive decluttering. Fixes
// (intersections) are the densest, so they only appear when zoomed well in, and
// at most kMaxFixesDrawn of the nearest ones are drawn. Airports are not listed
// here: they declutter per-size against the Map Setup "Aviation" ranges carried
// in MapViewStyle (see kLargeAirportRunwayFt below).
constexpr float kFeatureRangeVorNm = 100.0f;
constexpr float kFeatureRangeNdbNm = 40.0f;
constexpr float kFeatureRangeFixNm = 7.5f;
constexpr int kMaxFixesDrawn = 40;

// Airport size classification by longest runway, mirroring Garmin's
// AirportWaypoint: a hard-surface runway >= 8100 ft is a Large airport, >= 5000
// ft (or any towered field) is Medium, and everything else is Small. Each size
// then declutters against its own Map Setup "Aviation" max range.
constexpr int kLargeAirportRunwayFt = 8100;
constexpr int kMediumAirportRunwayFt = 5000;

}  // namespace

void drawNavFeatures(Renderer& r, const MapData& map, const Proj& proj,
                     const MapViewConfig& config, float rangeNm, float symSize,
                     float labelSize) {
  // Range-based declutter, mirroring the G1000: drop the densest feature
  // classes as the range opens up so the map stays readable, and cap the
  // number of intersections drawn (the list is nearest-first, so the closest
  // ones win) since terminal areas hold hundreds of them.
  auto visibleAtRange = [&](MapFeatureType type) {
    switch (type) {
      case MapFeatureType::Airport:
        return true;  // airports declutter by size (airportVisible)
      case MapFeatureType::Vor:
        return rangeNm <= kFeatureRangeVorNm;
      case MapFeatureType::Ndb:
        return rangeNm <= kFeatureRangeNdbNm;
      case MapFeatureType::Fix:
      case MapFeatureType::Waypoint:
        return rangeNm <= kFeatureRangeFixNm;
    }
    return true;
  };

  // Airports declutter per-size against the Map Setup "Aviation" ranges, so a
  // wide view keeps the major (Large) fields long after the small ones drop.
  auto airportVisible = [&](const MapFeature& f) {
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
  };

  int fixesDrawn = 0;
  for (const MapFeature& f : map.features) {
    if (f.type == MapFeatureType::Airport) {
      if (!airportVisible(f)) continue;
    } else if (!visibleAtRange(f.type)) {
      continue;
    }
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
    const Color labelC = mapFeatureColor(f);
    if (config.style.showLabels && !f.id.empty()) {
      r.fillText(x + symSize, y - symSize * 0.2f, f.id, labelSize * 0.85f,
                 TextAlign::Left, labelC);
    }
  }
}

}  // namespace avionics::mapview
