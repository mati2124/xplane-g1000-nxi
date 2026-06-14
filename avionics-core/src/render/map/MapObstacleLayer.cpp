#include "render/map/MapViewInternal.h"

namespace avionics::mapview {
namespace {

constexpr float kObstacleMaxRangeNm = 20.0f;
// Obstacle proximity coloring (same bands as relative terrain).
constexpr float kObstacleRedBelowFt = 100.0f;
constexpr float kObstacleYellowBelowFt = 1000.0f;

}  // namespace

void drawObstacles(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, float ownAltFt, bool altValid,
                   float symSize) {
  if (rangeNm > kObstacleMaxRangeNm) return;
  for (const MapObstacle& ob : map.obstacles) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(ob.lat, ob.lon, x, y);
    if (!proj.onScreen(x, y, symSize)) continue;

    Color c = colors::kWhite;
    if (altValid) {
      const float rel = ob.mslFt - ownAltFt;
      if (rel >= -kObstacleRedBelowFt) {
        c = colors::kBandRed;
      } else if (rel >= -kObstacleYellowBelowFt) {
        c = colors::kBandYellow;
      }
    }
    // Slender obstacle/tower glyph: tall open triangle with a tip dot.
    const float s = symSize * 0.9f;
    const Point tri[4] = {{x, y - s},
                          {x - s * 0.45f, y + s * 0.5f},
                          {x + s * 0.45f, y + s * 0.5f},
                          {x, y - s}};
    r.strokePolyline(tri, 4, 1.5f, c);
    r.fillCircle(x, y - s, 1.5f, c);
  }
}

}  // namespace avionics::mapview
