#include "avionics/render/BezelKeys.h"

#include <algorithm>

#include "avionics/Color.h"

namespace avionics {
namespace {

constexpr float kCanvasHeight = 768.0f;

// Layout fractions of the bezel-strip rectangle.
constexpr float kPadXFrac = 0.12f;   // side padding (of strip width)
constexpr float kPadYFrac = 0.015f;  // top/bottom padding (of strip height)
constexpr float kGapFrac = 0.012f;   // gap between keys (of strip height)

// Label/symbol sizes, in 768-px-canvas units.
constexpr float kLabelWt = 18.0f;
constexpr float kSymbolWt = 30.0f;
constexpr float kCaptionWt = 12.0f;

float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kCanvasHeight);
}

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

struct Cell {
  float x, y, w, h;
};

Cell cellRect(int index, float x, float y, float w, float h) {
  const float padX = w * kPadXFrac;
  const float padY = h * kPadYFrac;
  const float gap = h * kGapFrac;
  const float cellH =
      (h - 2.0f * padY - gap * (kBezelKeyCount - 1)) / kBezelKeyCount;
  return {x + padX, y + padY + static_cast<float>(index) * (cellH + gap),
          w - 2.0f * padX, cellH};
}

const char* keyLabel(BezelKey key) {
  switch (key) {
    case BezelKey::DirectTo:
      return "D";  // drawn with an arrow alongside
    case BezelKey::Menu:
      return "MENU";
    case BezelKey::Proc:
      return "PROC";
    case BezelKey::Clr:
      return "CLR";
    case BezelKey::Ent:
      return "ENT";
    case BezelKey::RangeUp:
      return "+";
    case BezelKey::RangeDown:
      return "\xE2\x88\x92";  // U+2212 minus sign
    case BezelKey::Count:
      break;
  }
  return "";
}

void drawDirectToGlyph(Renderer& r, const Cell& c, float displayH,
                       const Color& color) {
  const float cx = c.x + c.w * 0.5f;
  const float cy = c.y + c.h * 0.5f;
  const float size = fontPx(kSymbolWt, displayH);
  r.fillText(cx - c.w * 0.04f, cy, "D", size, TextAlign::Right, color);
  const float ax = cx;
  const float ay = cy;
  const float len = c.w * 0.22f;
  const float head = c.h * 0.15f;
  r.strokeLine(ax, ay, ax + len, ay, 2.0f, color);
  const Point tri[3] = {{ax + len + head * 0.2f, ay},
                        {ax + len - head * 0.4f, ay - head},
                        {ax + len - head * 0.4f, ay + head}};
  r.fillPolygon(tri, 3, color);
}

}  // namespace

void BezelKeyPanel::render(Renderer& r, float x, float y, float w, float h,
                           float displayH, const float* pressLevels) {
  // Bezel face: a dark metallic vertical gradient with an inner border, so the
  // strip reads as the physical frame around the screen rather than an overlay.
  r.fillRectVerticalGradient(x, y, w, h, y, y + h,
                             Color{0.13f, 0.14f, 0.16f, 1.0f},
                             Color{0.06f, 0.065f, 0.08f, 1.0f});
  r.strokeLine(x, y, x, y + h, 2.0f, colors::kPanelBorder);

  const float labelSize = fontPx(kLabelWt, displayH);
  const float symbolSize = fontPx(kSymbolWt, displayH);
  const float captionSize = fontPx(kCaptionWt, displayH);

  const Color faceTop{0.20f, 0.21f, 0.24f, 1.0f};
  const Color faceBottom{0.09f, 0.095f, 0.11f, 1.0f};

  for (int i = 0; i < kBezelKeyCount; ++i) {
    const Cell cell = cellRect(i, x, y, w, h);
    const BezelKey key = static_cast<BezelKey>(i);
    const float press = pressLevels ? std::max(0.0f, pressLevels[i]) : 0.0f;

    // Raised key face with a light top edge and a 1 px border.
    r.fillRectVerticalGradient(cell.x, cell.y, cell.w, cell.h, cell.y,
                               cell.y + cell.h, faceTop, faceBottom);
    const Point border[5] = {{cell.x, cell.y},
                             {cell.x + cell.w, cell.y},
                             {cell.x + cell.w, cell.y + cell.h},
                             {cell.x, cell.y + cell.h},
                             {cell.x, cell.y}};
    r.strokePolyline(border, 5, 1.0f, colors::kPanelBorder);

    // Press flash: cyan wash + brightened border, like the softkey feedback.
    if (press > 0.0f) {
      r.fillRect(cell.x, cell.y, cell.w, cell.h,
                 withAlpha(colors::kCyan, press * 0.40f));
      r.strokePolyline(border, 5, 2.0f, withAlpha(colors::kCyan, press));
    }

    const float cx = cell.x + cell.w * 0.5f;
    const float cy = cell.y + cell.h * 0.5f;
    const bool isRange =
        key == BezelKey::RangeUp || key == BezelKey::RangeDown;

    if (key == BezelKey::DirectTo) {
      drawDirectToGlyph(r, cell, displayH, colors::kWhite);
    } else if (isRange) {
      if (key == BezelKey::RangeUp) {
        r.fillText(cx, cell.y + cell.h * 0.24f, "RNG", captionSize,
                   TextAlign::Center, colors::kLabelText);
      }
      r.fillText(cx, cy + cell.h * 0.06f, keyLabel(key), symbolSize,
                 TextAlign::Center, colors::kCyan);
    } else {
      r.fillText(cx, cy, keyLabel(key), labelSize, TextAlign::Center,
                 colors::kWhite);
    }
  }
}

BezelKey BezelKeyPanel::hitTest(float xPx, float yPx, float x, float y, float w,
                                float h) {
  for (int i = 0; i < kBezelKeyCount; ++i) {
    const Cell c = cellRect(i, x, y, w, h);
    if (xPx >= c.x && xPx <= c.x + c.w && yPx >= c.y && yPx <= c.y + c.h) {
      return static_cast<BezelKey>(i);
    }
  }
  return BezelKey::Count;
}

}  // namespace avionics
