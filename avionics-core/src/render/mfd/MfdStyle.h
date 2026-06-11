#pragma once

#include <cmath>
#include <string>

#include "avionics/Color.h"
#include "avionics/Renderer.h"
#include "avionics/render/CursorHighlight.h"

// Shared layout helpers for the G1000 NXi MFD pages, matching the Working
// Title NXi chrome: WPT/NRST/FPL pages put a full-height map on the left and a
// gray data panel on the right, built from rounded "group boxes" whose white
// title sits on (and breaks) the top border line. Sizes are CSS pixels on the
// WT 1024x768 reference frame, scaled by displayH.
namespace avionics::mfd {

inline constexpr float kCanvasHeight = 768.0f;

// Font weights (px on the 768 canvas), from the WT NXi CSS: 14px group-box
// titles, 20px box content rows, field labels at 0.75em of the row text,
// 14px column headers (.smallText), large facility idents.
inline constexpr float kWtBoxTitle = 14.0f;
inline constexpr float kWtRow = 20.0f;
inline constexpr float kWtFieldLabel = 15.0f;
inline constexpr float kWtFieldValue = 20.0f;
inline constexpr float kWtHeader = 14.0f;
inline constexpr float kWtIdentLarge = 26.0f;
// Unit suffixes render at 0.7em of their value (.numberunit-unit-small).
inline constexpr float kUnitEm = 0.7f;

// Right data panel width fractions of the page body (WT: 298px / 436px of the
// 876px page area).
inline constexpr float kPanelWFrac = 298.0f / 876.0f;
inline constexpr float kPanelWideWFrac = 436.0f / 876.0f;

inline float mfdFontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kCanvasHeight);
}

inline Color mfdAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

struct Rect {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
};

// ---- rounded-rect / circle geometry (the Renderer has no arc primitive) ----

inline constexpr int kCornerSegments = 5;
// Points for a closed rounded rectangle: 4 corners x (kCornerSegments + 1)
// vertices, plus one to close the outline when stroked.
inline constexpr int kRoundedRectPoints = 4 * (kCornerSegments + 1);

// Fills `pts` (size >= kRoundedRectPoints + 1) with a rounded-rect outline,
// clockwise from the top-left corner. Returns the closed point count
// (kRoundedRectPoints + 1); pass kRoundedRectPoints to fillPolygon.
inline int buildRoundedRect(Point* pts, const Rect& box, float radius) {
  const float r = std::min(radius, std::min(box.w, box.h) * 0.5f);
  // Corner centers in clockwise order starting top-left, with the arc start
  // angle (radians, 0 = +x, y down).
  struct Corner {
    float cx, cy, a0;
  };
  const Corner corners[4] = {
      {box.x + r, box.y + r, 3.14159265f},                      // TL: 180->270
      {box.x + box.w - r, box.y + r, 4.71238898f},              // TR: 270->360
      {box.x + box.w - r, box.y + box.h - r, 0.0f},             // BR: 0->90
      {box.x + r, box.y + box.h - r, 1.57079633f},              // BL: 90->180
  };
  int n = 0;
  for (const Corner& c : corners) {
    for (int s = 0; s <= kCornerSegments; ++s) {
      const float a =
          c.a0 + 1.57079633f * static_cast<float>(s) / kCornerSegments;
      pts[n++] = {c.cx + r * std::cos(a), c.cy + r * std::sin(a)};
    }
  }
  pts[n] = pts[0];  // close for strokePolyline
  return n + 1;
}

// Stroked circle via a tessellated polyline.
inline void strokeCircle(Renderer& r, float cx, float cy, float radius,
                         float widthPx, const Color& c) {
  constexpr int kSegs = 28;
  Point pts[kSegs + 1];
  for (int i = 0; i <= kSegs; ++i) {
    const float a = 6.2831853f * static_cast<float>(i) / kSegs;
    pts[i] = {cx + radius * std::cos(a), cy + radius * std::sin(a)};
  }
  r.strokePolyline(pts, kSegs + 1, widthPx, c);
}

// ---- group box ----

// Draws an NXi group box inside `slot` and returns the inner content rect.
// WT GroupBox.css: 1px rgb(120,120,120) border, 10px radius, black body with
// a faint steel-blue gradient over the first 10px, and a white 14px title at
// left 15px sitting across the top border on a panel-gray patch. The top half
// of the title extends above the container, so the container starts a half
// title-height below the slot top.
inline Rect drawGroupBox(Renderer& r, const Rect& slot, const char* title,
                         float displayH) {
  const float titleSize = mfdFontPx(kWtBoxTitle, displayH);
  const float topInset = titleSize * 0.55f;
  const Rect box{slot.x, slot.y + topInset, slot.w, slot.h - topInset};
  const float radius = mfdFontPx(10.0f, displayH);

  Point pts[kRoundedRectPoints + 1];
  const int closed = buildRoundedRect(pts, box, radius);
  r.fillPolygon(pts, kRoundedRectPoints, colors::kBlack);
  r.fillRectVerticalGradient(box.x + radius, box.y + 1.0f, box.w - 2.0f * radius,
                             radius, box.y, box.y + radius,
                             colors::kGroupBoxSheen, colors::kGroupBoxSheenEnd);
  r.strokePolyline(pts, closed, 1.0f, colors::kGroupBoxBorder);

  if (title != nullptr && title[0] != '\0') {
    const float tx = box.x + mfdFontPx(15.0f, displayH);
    const float tw = r.measureTextWidth(title, titleSize);
    const float padX = titleSize * 0.35f;
    // Opaque patch matching the panel gray behind the title "breaks" the
    // border line, like the WT title's background-color.
    r.fillRect(tx - padX, box.y - titleSize * 0.62f, tw + 2.0f * padX,
               titleSize * 1.24f, colors::kMfdPanelGray);
    r.fillText(tx, box.y, title, titleSize, TextAlign::Left, colors::kWhite);
  }

  const float padX = mfdFontPx(6.0f, displayH);
  const float padTop = mfdFontPx(10.0f, displayH);
  const float padBot = mfdFontPx(8.0f, displayH);
  return Rect{box.x + padX, box.y + padTop, box.w - 2.0f * padX,
              box.h - padTop - padBot};
}

// ---- dialogs (page menu / entry / confirmation popouts) ----

// WT .popout-dialog: 10px radius, 4px rgb(150,150,150) border, gray #323232
// body, centered title. Returns the inner content rect.
inline Rect drawDialog(Renderer& r, const Rect& box, const char* title,
                       float displayH) {
  const float radius = mfdFontPx(10.0f, displayH);
  Point pts[kRoundedRectPoints + 1];
  const int closed = buildRoundedRect(pts, box, radius);
  r.fillPolygon(pts, kRoundedRectPoints, colors::kMfdPanelGray);
  r.strokePolyline(pts, closed, mfdFontPx(4.0f, displayH),
                   colors::kPanelBorder);

  const float titleSize = mfdFontPx(16.0f, displayH);
  float top = box.y + mfdFontPx(6.0f, displayH);
  if (title != nullptr && title[0] != '\0') {
    r.fillText(box.x + box.w * 0.5f, top + titleSize * 0.7f, title, titleSize,
               TextAlign::Center, colors::kWhite);
    top += titleSize * 1.6f;
  }
  const float pad = mfdFontPx(10.0f, displayH);
  return Rect{box.x + pad, top, box.w - 2.0f * pad,
              box.y + box.h - top - pad};
}

// ---- fields and values ----

// One field row: gray label (--title-gray, 0.75em) on the left, the value
// right-aligned in its semantic color (whitesmoke = computed, cyan = pilot
// editable, magenta = GPS derived). Returns the next row's y.
inline float drawField(Renderer& r, const Rect& area, float y, float rowH,
                       const char* label, const std::string& value,
                       float displayH, const Color& valueColor) {
  const float cy = y + rowH * 0.5f;
  r.fillText(area.x, cy, label, mfdFontPx(kWtFieldLabel, displayH),
             TextAlign::Left, colors::kTitleGray);
  r.fillText(area.x + area.w, cy, value, mfdFontPx(kWtFieldValue, displayH),
             TextAlign::Right, valueColor);
  return y + rowH;
}

// Right-aligned value with a smaller unit suffix (NumberUnitDisplay: unit at
// 0.7em of the value size). xRight is the right edge of the unit text.
inline void drawValueWithUnit(Renderer& r, float xRight, float cy,
                              const std::string& value, const char* unit,
                              float valueSize, const Color& c) {
  const float unitSize = valueSize * kUnitEm;
  const float unitW = r.measureTextWidth(unit, unitSize);
  r.fillText(xRight, cy, unit, unitSize, TextAlign::Right, c);
  r.fillText(xRight - unitW, cy, value, valueSize, TextAlign::Right, c);
}

// Steady highlight-active cursor (solid cyan plate, black text) while a field
// is actively receiving input.
inline void drawCursorText(Renderer& r, float x, float cy,
                           const std::string& text, float sizePx,
                           TextAlign align) {
  render::drawCursorActive(r, x, cy, text, sizePx, align);
}

// Pulsing highlight-select cursor for a parked field the pilot can edit next.
inline void drawCursorSelect(Renderer& r, float x, float cy,
                             const std::string& text, float sizePx,
                             TextAlign align, bool blinkOn) {
  render::drawCursorSelect(r, x, cy, text, sizePx, align, blinkOn);
}

// Frequency "pill": centered text inside a 1px gray rounded border
// (FrequenciesGroup.css: border-radius 16px, border 1px solid gray).
inline void drawPill(Renderer& r, const Rect& box, const std::string& text,
                     float sizePx, const Color& textColor) {
  Point pts[kRoundedRectPoints + 1];
  const int closed = buildRoundedRect(pts, box, box.h * 0.5f);
  r.strokePolyline(pts, closed, 1.0f, colors::kGroupBoxBorder);
  r.fillText(box.x + box.w * 0.5f, box.y + box.h * 0.5f, text, sizePx,
             TextAlign::Center, textColor);
}

}  // namespace avionics::mfd
