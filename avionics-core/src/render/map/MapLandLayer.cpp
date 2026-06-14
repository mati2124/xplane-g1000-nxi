#include "render/map/MapViewInternal.h"

#include <vector>

namespace avionics::mapview {
namespace {

// Layer range declutter (NM), mirroring the G1000 Map Setup maximum-range
// defaults: each layer disappears once the range opens past its threshold.
constexpr float kRoadMaxRangeNm = 60.0f;
constexpr float kRiverMaxRangeNm = 150.0f;
constexpr float kLakeMaxRangeNm = 150.0f;
// Railroads are a close-in detail feature on the NXi Land group; declutter
// past short range so their crosstie ticks don't clutter a wide view.
constexpr float kRailroadMaxRangeNm = 30.0f;
// State/province lines declutter past continental scale so a near-global view
// shows only nation borders and coastlines; nation borders have no cutoff.
constexpr float kStateBorderMaxRangeNm = 1000.0f;
// City label tiers by Natural Earth rank (higher rank = larger city).
constexpr float kCityLargeMaxRangeNm = 150.0f;
constexpr float kCityMediumMaxRangeNm = 50.0f;
constexpr float kCitySmallMaxRangeNm = 20.0f;
constexpr float kCityCapitalMaxRangeNm = 500.0f;

// Land-data styling to sit under the white/cyan/magenta symbology like the
// Garmin base map: hydro features are medium blue (Fig 5-13/5-14, ~35/61/137),
// roads dark brown, borders gray.
constexpr Color kWaterFill{0.137f, 0.239f, 0.537f, 1.0f};
constexpr Color kRiverStroke{0.22f, 0.42f, 0.68f, 1.0f};
constexpr Color kRoadStroke{0.38f, 0.30f, 0.20f, 1.0f};
constexpr Color kBorderStroke{0.55f, 0.55f, 0.55f, 0.8f};
// State/province boundaries: dimmer than nation borders so the political
// hierarchy reads at a glance (nation lines dominate).
constexpr Color kStateBorderStroke{0.42f, 0.42f, 0.42f, 0.7f};
constexpr Color kCoastStroke{0.35f, 0.42f, 0.52f, 0.85f};
constexpr Color kRailroadStroke{0.62f, 0.62f, 0.62f, 0.8f};
constexpr Color kCityDot{0.85f, 0.78f, 0.45f, 1.0f};

}  // namespace

void drawLandData(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm) {
  const ClipBounds clip{proj.minX, proj.minY, proj.maxX, proj.maxY};
  std::vector<Point> pts;
  for (const MapLandLine& line : map.landLines) {
    if (line.points.size() < 2) continue;
    if (line.landClass == LandClass::Road && rangeNm > kRoadMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::River && rangeNm > kRiverMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::Lake && rangeNm > kLakeMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::StateBorder &&
        rangeNm > kStateBorderMaxRangeNm) {
      continue;
    }
    if (line.landClass == LandClass::Railroad &&
        rangeNm > kRailroadMaxRangeNm) {
      continue;
    }

    pts.clear();
    pts.reserve(line.points.size());
    bool anyVisible = false;
    for (const GeoPoint& g : line.points) {
      float x = 0.0f, y = 0.0f;
      proj.toPx(g.lat, g.lon, x, y);
      anyVisible = anyVisible || proj.onScreen(x, y, 80.0f);
      pts.push_back({x, y});
    }
    if (!anyVisible) continue;

    const int n = static_cast<int>(pts.size());
    switch (line.landClass) {
      case LandClass::Lake:
        r.fillPolygon(pts.data(), n, kWaterFill);
        break;
      case LandClass::River:
        r.strokePolyline(pts.data(), n, 1.2f, kRiverStroke);
        break;
      case LandClass::Road:
        r.strokePolyline(pts.data(), n, 1.2f, kRoadStroke);
        break;
      case LandClass::Border:
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kBorderStroke, &clip);
        break;
      case LandClass::StateBorder:
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kStateBorderStroke, &clip);
        break;
      case LandClass::Coast:
        r.strokePolyline(pts.data(), n, 1.0f, kCoastStroke);
        break;
      case LandClass::Railroad:
        drawRailroad(r, pts.data(), n, kRailroadStroke, &clip);
        break;
      case LandClass::City:
        break;  // cities are point features (MapData::cities)
    }
  }
}

void drawCities(Renderer& r, const MapData& map, const Proj& proj,
                float rangeNm, float symSize, float labelSize) {
  for (const MapLandCity& city : map.cities) {
    const float maxRange = city.rank >= 9   ? kCityCapitalMaxRangeNm
                           : city.rank >= 8 ? kCityLargeMaxRangeNm
                           : city.rank >= 4 ? kCityMediumMaxRangeNm
                                            : kCitySmallMaxRangeNm;
    if (rangeNm > maxRange) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(city.lat, city.lon, x, y);
    if (!proj.onScreen(x, y, symSize)) continue;
    r.fillCircle(x, y, symSize * 0.28f, kCityDot);
    r.fillText(x + symSize * 0.6f, y - symSize * 0.2f, city.name,
               labelSize * 0.8f, TextAlign::Left, kCityDot);
  }
}

}  // namespace avionics::mapview
