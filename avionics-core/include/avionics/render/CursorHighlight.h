#pragma once

#include <string>

#include "avionics/Color.h"
#include "avionics/Renderer.h"

namespace avionics::render {

inline Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Steady highlight-active (WT .highlight-active): solid cyan plate, black text.
// Used while a field is actively receiving knob/keyboard input.
inline void drawCursorActive(Renderer& r, float x, float cy,
                             const std::string& text, float sizePx,
                             TextAlign align, float alpha = 1.0f) {
  const float tw = r.measureTextWidth(text, sizePx);
  const float padX = sizePx * 0.25f;
  float left = x;
  if (align == TextAlign::Right) left = x - tw;
  if (align == TextAlign::Center) left = x - tw * 0.5f;
  r.fillRect(left - padX, cy - sizePx * 0.62f, tw + 2.0f * padX,
             sizePx * 1.24f, withAlpha(colors::kPopoutCyan, alpha));
  r.fillText(x, cy, text, sizePx, align, withAlpha(colors::kBlack, alpha));
}

// Pulsing highlight-select (WT .highlight-select @keyframes pulse, 1 Hz):
// cyan plate + black text for the first half of each second, plain cyan text
// for the second half. `blinkOn` is true during the on half.
inline void drawCursorSelect(Renderer& r, float x, float cy,
                             const std::string& text, float sizePx,
                             TextAlign align, bool blinkOn, float alpha = 1.0f) {
  if (blinkOn) {
    drawCursorActive(r, x, cy, text, sizePx, align, alpha);
  } else {
    r.fillText(x, cy, text, sizePx, align, withAlpha(colors::kPopoutCyan, alpha));
  }
}

// Full-width row highlight that pulses with highlight-select.
inline void drawCursorRowSelect(Renderer& r, float x, float y, float w,
                                float h, bool blinkOn, float alpha = 1.0f) {
  if (blinkOn) {
    r.fillRect(x, y, w, h, withAlpha(colors::kPopoutCyan, alpha));
  }
}

}  // namespace avionics::render
