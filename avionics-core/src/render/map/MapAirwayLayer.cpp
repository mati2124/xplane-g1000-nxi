#include "render/map/MapViewInternal.h"

namespace avionics::mapview {
namespace {

// Airways declutter above this range; labels only appear when zoomed further in.
constexpr float kAirwayMaxRangeNm = 80.0f;
constexpr float kAirwayLabelMaxRangeNm = 30.0f;

// Low-altitude Victor/T-routes: gray, same shade as roads (Fig 5-15).
constexpr Color kAirwayLowStroke{0.55f, 0.55f, 0.55f, 0.75f};

bool airwayLevelVisible(AirwayDisplay display, AirwayLevel level) {
  switch (display) {
    case AirwayDisplay::Off:
      return false;
    case AirwayDisplay::All:
      return true;
    case AirwayDisplay::Low:
      return level == AirwayLevel::Low;
    case AirwayDisplay::High:
      return level == AirwayLevel::High;
  }
  return false;
}

constexpr Color airwayStrokeFor(AirwayLevel level) {
  return level == AirwayLevel::High ? colors::kGreen : kAirwayLowStroke;
}

// Boxed airway ident centered on the segment (Fig 5-15).
void drawAirwayLabel(Renderer& r, float cx, float cy, const char* text,
                     float size) {
  const float padX = size * 0.35f;
  const float w = r.measureTextWidth(text, size) + 2.0f * padX;
  const float h = size * 1.35f;
  const float x = cx - w * 0.5f;
  const float y = cy - h * 0.5f;
  drawChromeBox(r, x, y, w, h);
  r.fillText(cx, y + h * 0.52f, text, size, TextAlign::Center, colors::kWhite);
}

// One altitude class: gray (low) or green (high) edges with a boxed ident at
// each segment midpoint (Fig 5-15), filtered by the AWY softkey state.
void drawAirwaySegments(Renderer& r, const MapData& map, const Proj& proj,
                        AirwayDisplay display, AirwayLevel level, float rangeNm,
                        float labelSize) {
  if (!airwayLevelVisible(display, level)) return;
  const Color stroke = airwayStrokeFor(level);
  const float textSize = labelSize * 0.8f;
  for (const MapAirwaySegment& seg : map.airways) {
    if (seg.level != level) continue;
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    proj.toPx(seg.a.lat, seg.a.lon, ax, ay);
    proj.toPx(seg.b.lat, seg.b.lon, bx, by);
    if (!proj.onScreen(ax, ay, 120.0f) && !proj.onScreen(bx, by, 120.0f)) {
      continue;
    }
    r.strokeLine(ax, ay, bx, by, 1.0f, stroke);

    if (rangeNm <= kAirwayLabelMaxRangeNm && !seg.name.empty()) {
      const float mx = (ax + bx) * 0.5f;
      const float my = (ay + by) * 0.5f;
      if (!proj.onScreen(mx, my, 0.0f)) continue;
      drawAirwayLabel(r, mx, my, seg.name.c_str(), textSize);
    }
  }
}

}  // namespace

void drawAirways(Renderer& r, const MapData& map, const Proj& proj,
                 AirwayDisplay display, float rangeNm, float labelSize) {
  if (display == AirwayDisplay::Off || rangeNm > kAirwayMaxRangeNm) return;
  drawAirwaySegments(r, map, proj, display, AirwayLevel::Low, rangeNm,
                     labelSize);
  drawAirwaySegments(r, map, proj, display, AirwayLevel::High, rangeNm,
                     labelSize);
}

}  // namespace avionics::mapview
