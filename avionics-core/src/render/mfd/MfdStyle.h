#pragma once

#include "avionics/Color.h"
#include "avionics/Renderer.h"

// Shared layout helpers for the G1000 NXi-style MFD pages: the bordered "window"
// boxes (cyan title header over a dark body) that every WPT/AUX/NRST page is
// built from, plus font scaling against the shared 1024x768 GDU canvas.
namespace avionics::mfd {

inline constexpr float kCanvasHeight = 768.0f;

// Font weights (px on the 768 canvas) tuned to the NXi field text sizes.
inline constexpr float kWtWindowTitle = 15.0f;
inline constexpr float kWtFieldLabel = 14.0f;
inline constexpr float kWtFieldValue = 18.0f;
inline constexpr float kWtIdentLarge = 30.0f;
inline constexpr float kWtRow = 16.0f;
inline constexpr float kWtHeader = 13.0f;

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

// Draws a bordered window with a cyan title strip and returns the inner content
// rect (below the title, inset by a small margin). The window body is a near
// black so map/terrain and white text read cleanly, matching the NXi field box.
inline Rect drawWindow(Renderer& r, const Rect& box, const char* title,
                       float displayH) {
  const float titleH = mfdFontPx(kWtWindowTitle, displayH) * 1.6f;

  // Body.
  r.fillRect(box.x, box.y, box.w, box.h, Color{0.04f, 0.05f, 0.07f, 0.96f});
  // Title strip.
  r.fillRectVerticalGradient(box.x, box.y, box.w, titleH, box.y, box.y + titleH,
                             colors::kPanelBackgroundBottom,
                             colors::kPanelBackground);
  r.strokeLine(box.x, box.y + titleH, box.x + box.w, box.y + titleH, 1.0f,
               colors::kPanelSeparator);

  // Outer border.
  const Point border[5] = {{box.x, box.y},
                           {box.x + box.w, box.y},
                           {box.x + box.w, box.y + box.h},
                           {box.x, box.y + box.h},
                           {box.x, box.y}};
  r.strokePolyline(border, 5, 1.5f, colors::kPanelBorder);

  if (title != nullptr && title[0] != '\0') {
    r.fillText(box.x + box.w * 0.04f, box.y + titleH * 0.5f, title,
               mfdFontPx(kWtWindowTitle, displayH), TextAlign::Left,
               colors::kCyan);
  }

  const float pad = box.w * 0.035f;
  return Rect{box.x + pad, box.y + titleH + pad * 0.5f, box.w - 2.0f * pad,
              box.h - titleH - pad};
}

// One "LABEL ........ value" row inside a window body, label grey on the left,
// value (white or cyan) right-aligned. Returns the next row's y.
inline float drawField(Renderer& r, const Rect& area, float y, float rowH,
                       const char* label, const std::string& value,
                       float displayH, const Color& valueColor) {
  const float cy = y + rowH * 0.5f;
  r.fillText(area.x, cy, label, mfdFontPx(kWtFieldLabel, displayH),
             TextAlign::Left, colors::kLabelText);
  r.fillText(area.x + area.w, cy, value, mfdFontPx(kWtFieldValue, displayH),
             TextAlign::Right, valueColor);
  return y + rowH;
}

}  // namespace avionics::mfd
