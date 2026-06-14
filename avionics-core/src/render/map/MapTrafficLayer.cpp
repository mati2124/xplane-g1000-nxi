#include "render/map/MapViewInternal.h"

#include <cmath>
#include <cstdio>

namespace avionics::mapview {
namespace {

// Traffic advisory / vertical trend thresholds for the TIS symbology.
constexpr float kTrafficTrendFpm = 500.0f;

}  // namespace

void drawTraffic(Renderer& r, const MapData& map, const Proj& proj,
                 float symSize, float labelSize) {
  char tag[8];
  for (const MapTraffic& t : map.traffic) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(t.lat, t.lon, x, y);
    if (!proj.onScreen(x, y, symSize * 2.0f)) continue;

    const Color c = t.trafficAdvisory ? colors::kBandYellow : colors::kWhite;
    const float s = symSize * 0.75f;
    if (t.trafficAdvisory) {
      r.fillCircle(x, y, s * 0.8f, c);
    } else {
      const Point diamond[5] = {
          {x, y - s}, {x + s, y}, {x, y + s}, {x - s, y}, {x, y - s}};
      r.strokePolyline(diamond, 5, 1.8f, c);
    }

    // Relative altitude tag in hundreds of feet, above the symbol when the
    // target is above ownship, below when below (TIS display convention).
    const int relHundreds =
        static_cast<int>(std::lround(t.relAltFt / 100.0f));
    std::snprintf(tag, sizeof(tag), "%+03d", relHundreds);
    const float tagY = t.relAltFt >= 0.0f ? y - s - labelSize * 0.55f
                                          : y + s + labelSize * 0.55f;
    r.fillText(x, tagY, tag, labelSize * 0.85f, TextAlign::Center, c);

    // Vertical trend arrow beside the symbol.
    if (std::fabs(t.verticalSpeedFpm) >= kTrafficTrendFpm) {
      const bool up = t.verticalSpeedFpm > 0.0f;
      const float axCol = x + s + labelSize * 0.4f;
      const float a0 = up ? y + s * 0.6f : y - s * 0.6f;
      const float a1 = up ? y - s * 0.6f : y + s * 0.6f;
      r.strokeLine(axCol, a0, axCol, a1, 1.5f, c);
      const float head = up ? 1.0f : -1.0f;
      r.strokeLine(axCol, a1, axCol - s * 0.3f, a1 + head * s * 0.4f, 1.5f, c);
      r.strokeLine(axCol, a1, axCol + s * 0.3f, a1 + head * s * 0.4f, 1.5f, c);
    }
  }
}

}  // namespace avionics::mapview
