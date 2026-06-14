#include "render/map/MapViewInternal.h"

#include <cmath>
#include <cstddef>
#include <string>

#include "avionics/MapRange.h"

namespace avionics::mapview {

void drawNorthArrow(Renderer& r, float cx, float cy, float size,
                    float rotation) {
  // A white compass arrow with "N" near its head, below the orientation label
  // (Fig 5-26).
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotation);
  const float h = size;          // half height
  const float w = size * 0.45f;  // half width
  // Two mirrored halves so the arrow reads like the printed compass glyph.
  const Point left[3] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}};
  const Point right[3] = {{0.0f, -h}, {w, h}, {0.0f, h * 0.45f}};
  r.fillPolygon(left, 3, colors::kWhite);
  r.fillPolygon(right, 3, Color{0.62f, 0.62f, 0.62f, 1.0f});
  const Point outline[5] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}, {w, h},
                            {0.0f, -h}};
  r.strokePolyline(outline, 5, 1.0f, Color{0.2f, 0.2f, 0.2f, 1.0f});
  r.fillText(0.0f, 0.0f, "N", size * 0.8f, TextAlign::Center,
             Color{0.1f, 0.1f, 0.1f, 1.0f});
  r.restore();
}

void drawRangeLabel(Renderer& r, float x, float y, float rangeNm,
                    float labelSize, bool centerOnPoint) {
  // Cyan value with the smaller unit suffix on a chrome plate, centered on the
  // ring's upper-left (Fig 5-2 "15NM").
  char buf[16];
  formatMapRange(buf, sizeof(buf), rangeNm);
  // Split the value digits from the unit suffix so the unit renders smaller.
  std::size_t unitStart = 0;
  while (buf[unitStart] != '\0' &&
         ((buf[unitStart] >= '0' && buf[unitStart] <= '9') ||
          buf[unitStart] == '.')) {
    ++unitStart;
  }
  const std::string value(buf, unitStart);
  const std::string unit(buf + unitStart);
  const float unitSize = labelSize * 0.72f;
  const float padX = labelSize * 0.45f;
  const float textW = r.measureTextWidth(value, labelSize) +
                      r.measureTextWidth(unit, unitSize);
  const float w = textW + 2.0f * padX;
  const float h = labelSize * 1.5f;
  const float bx = centerOnPoint ? x - w * 0.5f : x - w;
  const float by = centerOnPoint ? y - h * 0.5f : y - h;
  drawChromeBox(r, bx, by, w, h);
  r.fillText(bx + padX, by + h * 0.52f, value, labelSize, TextAlign::Left,
             colors::kCyan);
  r.fillText(bx + padX + r.measureTextWidth(value, labelSize), by + h * 0.52f,
             unit, unitSize, TextAlign::Left, colors::kCyan);
}

void drawRelTerrainLegend(Renderer& r, const MapViewConfig& config,
                          float labelSize) {
  // Shown while TER REL is enabled (NXi Pilot's Guide, Hazard Avoidance). Two
  // color bands: red within 100 ft below ownship, yellow within 1000 ft.
  const float pad = labelSize * 0.45f;
  const float rowH = labelSize * 1.25f;
  const float chipW = labelSize * 1.5f;
  const float w = labelSize * 7.2f;
  const float h = rowH * 3.0f + pad;
  const float x = config.x + config.w * 0.02f;
  const float y = config.y + config.h - h - config.h * 0.02f;

  r.fillRect(x, y, w, h, Color{0.0f, 0.0f, 0.0f, 0.78f});
  r.strokeLine(x, y, x + w, y, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y + h, x + w, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y, x, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x + w, y, x + w, y + h, 1.0f, colors::kPanelBorder);

  r.fillText(x + w * 0.5f, y + pad + rowH * 0.35f, "TERRAIN",
             labelSize * 0.85f, TextAlign::Center, colors::kWhite);

  struct Row {
    Color chip;
    const char* label;
  };
  const Row rows[2] = {{colors::kBandRed, "-100 FT"},
                       {colors::kBandYellow, "-1000 FT"}};
  for (int i = 0; i < 2; ++i) {
    const float rowY = y + pad + rowH * (0.9f + static_cast<float>(i));
    r.fillRect(x + pad, rowY, chipW, rowH * 0.6f, rows[i].chip);
    r.fillText(x + pad + chipW + pad, rowY + rowH * 0.3f, rows[i].label,
               labelSize * 0.8f, TextAlign::Left, colors::kLabelText);
  }
}

void drawRangeRing(Renderer& r, float cx, float cy, float radiusPx,
                   const Color& c) {
  constexpr int kSeg = 48;
  Point ring[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a =
        static_cast<float>(i) / static_cast<float>(kSeg) * 2.0f * 3.14159265f;
    ring[i] = {cx + radiusPx * std::cos(a), cy + radiusPx * std::sin(a)};
  }
  r.strokePolyline(ring, kSeg + 1, 1.5f, c);
}

}  // namespace avionics::mapview
