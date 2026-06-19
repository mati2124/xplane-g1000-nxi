#include "avionics/render/BezelKeys.h"

#include <algorithm>
#include <cmath>

#include "avionics/Color.h"
#include "render/BezelStyle.h"

namespace avionics {
namespace {

constexpr float kCanvasHeight = 768.0f;
constexpr float kPi = 3.14159265f;

// Layout fractions of the bezel-strip rectangle, stacked top-to-bottom to match
// the GDU bezel (G1000 NXi Pilot's Guide Figure 1-2 "PFD/MFD Controls"). The
// right strip carries (top→bottom): the COM VOL/SQ knob + COM transfer key, the
// COM knob, the CRS/BARO knob, the RANGE joystick, the 2×3 key grid and the FMS
// knob. The left strip carries the NAV VOL/ID knob + NAV transfer key, the NAV
// knob and the HDG knob.
constexpr float kPadXFrac = 0.10f;
constexpr float kPadTopFrac = 0.012f;
constexpr float kSlotGapFrac = 0.012f;

// Right-strip slot heights, as fractions of the strip height (sum < 1).
constexpr float kVolRowFrac = 0.085f;     // VOL/SQ knob + transfer key row
constexpr float kRadioKnobFrac = 0.135f;  // COM / NAV tuning knob
constexpr float kBaroKnobFrac = 0.135f;   // CRS/BARO knob
constexpr float kRangeAreaFrac = 0.135f;  // RANGE joystick
constexpr float kKeyGridAreaFrac = 0.235f;
constexpr float kKnobAreaFrac = 0.155f;   // FMS knob (bottom)

constexpr int kKeyGridCols = 2;
constexpr int kKeyGridRows = 3;

// The drawn RANGE zoom ring is a thin annulus (rInner..rOuter), so the +/- zoom
// targets are tiny. Extend the clickable zoom region past the drawn rOuter to
// make them easy to hit. The key grid below is hit-tested first, so this never
// steals clicks from the keys, and the area above the knob is plain face.
constexpr float kRangeZoomHitMargin = 1.4f;

// Label/symbol sizes, in 768-px-canvas units.
constexpr float kLabelWt = 13.0f;
constexpr float kSymbolWt = 20.0f;
constexpr float kCaptionWt = 11.0f;
constexpr float kKnobCaptionWt = 8.5f;
constexpr float kLegendWt = 7.0f;

float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kCanvasHeight);
}

struct Cell {
  float x, y, w, h;
};

// Top/height of each stacked slot in the right bezel strip.
struct ClusterLayout {
  float volRowTop, volRowH;
  float radioTop, radioH;
  float baroTop, baroH;
  float rangeTop, rangeH;
  float keyTop, keyH;
  float knobTop, knobH;
};

ClusterLayout layoutCluster(float x, float y, float w, float h) {
  (void)x;
  const float gap = h * kSlotGapFrac;
  float top = y + h * kPadTopFrac;
  ClusterLayout cl{};
  cl.volRowTop = top;
  cl.volRowH = h * kVolRowFrac;
  top += cl.volRowH + gap;
  cl.radioTop = top;
  cl.radioH = h * kRadioKnobFrac;
  top += cl.radioH + gap;
  cl.baroTop = top;
  cl.baroH = h * kBaroKnobFrac;
  top += cl.baroH + gap;
  cl.rangeTop = top;
  cl.rangeH = h * kRangeAreaFrac;
  top += cl.rangeH + gap;
  cl.keyTop = top;
  cl.keyH = h * kKeyGridAreaFrac;
  top += cl.keyH + gap;
  cl.knobTop = top;
  cl.knobH = h * kKnobAreaFrac;
  return cl;
}

// Top/height of each stacked slot in the left bezel strip (NAV + HDG). The
// lower portion carries the red DISPLAY BACKUP key (GMA audio panel).
struct LeftLayout {
  float volRowTop, volRowH;
  float navTop, navH;
  float hdgTop, hdgH;
  float backupTop, backupH;
};

LeftLayout layoutLeft(float x, float y, float w, float h) {
  (void)x;
  (void)w;
  const float gap = h * kSlotGapFrac;
  float top = y + h * kPadTopFrac;
  LeftLayout cl{};
  cl.volRowTop = top;
  cl.volRowH = h * kVolRowFrac;
  top += cl.volRowH + gap;
  cl.navTop = top;
  cl.navH = h * kRadioKnobFrac;
  top += cl.navH + gap;
  cl.hdgTop = top;
  cl.hdgH = h * kBaroKnobFrac;
  top += cl.hdgH + gap;
  cl.backupTop = top;
  cl.backupH = std::max(h * 0.10f, y + h * (1.0f - kPadTopFrac) - top);
  return cl;
}

// BezelKey indices 0..5 map to the real 2×3 grid (left column, then right):
//   D→  MENU
//   FPL PROC
//   CLR ENT
Cell gridCell(int index, float x, float w, const ClusterLayout& cl) {
  const int row = index / kKeyGridCols;
  const int col = index % kKeyGridCols;
  const float padX = w * kPadXFrac;
  const float colGap = w * 0.05f;
  const float rowGap = cl.keyH * 0.055f;
  const float cellW = (w - 2.0f * padX - colGap) * 0.5f;
  const float cellH =
      (cl.keyH - rowGap * static_cast<float>(kKeyGridRows + 1)) /
      static_cast<float>(kKeyGridRows);
  return {x + padX + static_cast<float>(col) * (cellW + colGap),
          cl.keyTop + rowGap + static_cast<float>(row) * (cellH + rowGap),
          cellW, cellH};
}

struct Knob {
  float cx, cy;
  float rOuter;
  float rInner;
  float rCenter;
};

// A dual-concentric knob centered in a stacked slot [top, top+slotH], leaving
// room above for a caption.
Knob knobInSlot(float x, float w, float top, float slotH) {
  const float cx = x + w * 0.5f;
  const float cy = top + slotH * 0.56f;
  const float rOuter = std::min(w * 0.42f, slotH * 0.40f);
  return {cx, cy, rOuter, rOuter * 0.62f, rOuter * 0.28f};
}

Knob fmsKnobRect(float x, float w, const ClusterLayout& cl) {
  return knobInSlot(x, w, cl.knobTop, cl.knobH);
}

Knob rangeJoyRect(float x, float w, const ClusterLayout& cl) {
  const float cx = x + w * 0.5f;
  const float cy = cl.rangeTop + cl.rangeH * 0.56f;
  const float rOuter = std::min(w * 0.42f, cl.rangeH * 0.40f);
  return {cx, cy, rOuter, rOuter * 0.62f, rOuter * 0.30f};
}

Knob comKnobRect(float x, float w, const ClusterLayout& cl) {
  return knobInSlot(x, w, cl.radioTop, cl.radioH);
}

Knob baroKnobRect(float x, float w, const ClusterLayout& cl) {
  return knobInSlot(x, w, cl.baroTop, cl.baroH);
}

// A small single-rotary knob (the VOL/SQ or VOL/ID knob) sitting on one side of
// the top row, with the transfer key on the other.
Knob volKnobIn(float x, float w, float top, float slotH, bool leftSide) {
  const float r = std::min(w * 0.20f, slotH * 0.42f);
  const float cx = leftSide ? x + w * 0.30f : x + w * 0.70f;
  const float cy = top + slotH * 0.60f;
  return {cx, cy, r, r * 0.55f, r * 0.55f};
}

Cell transferCellIn(float x, float w, float top, float slotH, bool leftSide) {
  const float cw = w * 0.30f;
  const float ch = slotH * 0.52f;
  const float cx = leftSide ? x + w * 0.70f - cw * 0.5f : x + w * 0.30f - cw * 0.5f;
  const float cy = top + slotH * 0.34f;
  return {cx, cy, cw, ch};
}

Cell backupCell(float x, float w, const LeftLayout& cl) {
  const float padX = w * kPadXFrac;
  const float keyW = w - 2.0f * padX;
  const float keyH = cl.backupH * 0.82f;
  return {x + padX, cl.backupTop + (cl.backupH - keyH) * 0.5f, keyW, keyH};
}

// Red GMA audio-panel DISPLAY BACKUP key (Pilot's Guide Fig. 1-7).
void drawDisplayBackupKey(Renderer& r, const Cell& c, float displayH,
                          float press) {
  const float lit = std::min(1.0f, press + 0.35f);
  const Color faceTop{0.78f + 0.12f * lit, 0.08f + 0.06f * lit,
                      0.08f + 0.06f * lit, 1.0f};
  const Color faceBottom{0.48f + 0.10f * lit, 0.02f + 0.04f * lit,
                         0.02f + 0.04f * lit, 1.0f};
  r.fillRectVerticalGradient(c.x, c.y, c.w, c.h, c.y, c.y + c.h, faceTop,
                             faceBottom);
  r.strokeRoundedRect(c.x, c.y, c.w, c.h, c.h * 0.12f, 1.5f,
                      Color{0.18f, 0.02f, 0.02f, 1.0f});

  const float size = std::min(fontPx(kCaptionWt, displayH), c.w * 0.16f);
  const float lineGap = size * 0.95f;
  const float cy = c.y + c.h * 0.52f;
  r.fillText(c.x + c.w * 0.5f, cy - lineGap * 0.5f, "DISPLAY", size,
             TextAlign::Center, colors::kWhite);
  r.fillText(c.x + c.w * 0.5f, cy + lineGap * 0.5f, "BACKUP", size,
             TextAlign::Center, colors::kWhite);
}

const char* keyLabel(BezelKey key) {
  switch (key) {
    case BezelKey::DirectTo:
      return "D";
    case BezelKey::Menu:
      return "MENU";
    case BezelKey::Fpl:
      return "FPL";
    case BezelKey::Proc:
      return "PROC";
    case BezelKey::Clr:
      return "CLR";
    case BezelKey::Ent:
      return "ENT";
    default:
      break;
  }
  return "";
}

void drawDirectToGlyph(Renderer& r, const Cell& c, float displayH,
                       const Color& color) {
  const float cx = c.x + c.w * 0.5f;
  const float cy = c.y + c.h * 0.5f;
  const float size = std::min(fontPx(kSymbolWt, displayH), c.h * 0.55f);
  r.fillText(cx - c.w * 0.06f, cy, "D", size, TextAlign::Right, color);
  const float len = c.w * 0.20f;
  const float head = c.h * 0.14f;
  r.strokeLine(cx, cy, cx + len, cy, 1.8f, color);
  const Point tri[3] = {{cx + len + head * 0.15f, cy},
                        {cx + len - head * 0.35f, cy - head},
                        {cx + len - head * 0.35f, cy + head}};
  r.fillPolygon(tri, 3, color);
}

float pressLevel(const float* levels, BezelKey key) {
  return levels != nullptr
             ? std::max(0.0f, levels[static_cast<int>(key)])
             : 0.0f;
}

void drawKnurledRing(Renderer& r, float cx, float cy, float rInner,
                     float rOuter) {
  constexpr int kGrooves = 36;
  const Color light{0.26f, 0.27f, 0.30f, 0.30f};
  const Color dark{0.05f, 0.06f, 0.07f, 0.55f};
  const float rMid = (rInner + rOuter) * 0.5f;
  const float half = (rOuter - rInner) * 0.38f;
  for (int i = 0; i < kGrooves; ++i) {
    const float a = static_cast<float>(i) / static_cast<float>(kGrooves) *
                    2.0f * kPi;
    const float ca = std::cos(a);
    const float sa = std::sin(a);
    r.strokeLine(cx + ca * (rMid - half), cy + sa * (rMid - half),
                 cx + ca * (rMid + half), cy + sa * (rMid + half), 1.0f,
                 (i % 2 == 0) ? light : dark);
  }
}

void drawRingOutline(Renderer& r, float cx, float cy, float radius,
                     float width, const Color& c) {
  constexpr int kSegs = 48;
  Point pts[kSegs + 1];
  for (int i = 0; i <= kSegs; ++i) {
    const float a = static_cast<float>(i) / static_cast<float>(kSegs) * 2.0f *
                    kPi;
    pts[i] = {cx + std::cos(a) * radius, cy + std::sin(a) * radius};
  }
  r.strokePolyline(pts, kSegs + 1, width, c);
}

void drawConcentricKnobBody(Renderer& r, const Knob& k, float push) {
  // Outer flange with knurled grip (Figure 1-2 inset).
  r.fillCircle(k.cx, k.cy, k.rOuter, Color{0.11f, 0.12f, 0.14f, 1.0f});
  drawKnurledRing(r, k.cx, k.cy, k.rInner + (k.rOuter - k.rInner) * 0.18f,
                  k.rOuter * 0.98f);
  drawRingOutline(r, k.cx, k.cy, k.rOuter * 0.98f, 1.2f,
                  Color{0.04f, 0.04f, 0.05f, 1.0f});

  // Inner dome with a soft top sheen.
  r.fillCircle(k.cx, k.cy, k.rInner, Color{0.17f, 0.18f, 0.20f, 1.0f});
  r.fillRectVerticalGradient(k.cx - k.rInner * 0.55f, k.cy - k.rInner * 0.65f,
                             k.rInner * 1.1f, k.rInner * 0.55f,
                             k.cy - k.rInner * 0.65f, k.cy,
                             Color{1.0f, 1.0f, 1.0f, 0.10f},
                             Color{1.0f, 1.0f, 1.0f, 0.0f});
  drawRingOutline(r, k.cx, k.cy, k.rInner, 1.0f,
                  Color{0.06f, 0.07f, 0.08f, 1.0f});

  // Center push cap.
  r.fillCircle(k.cx, k.cy, k.rCenter,
               Color{0.19f + 0.24f * push, 0.20f + 0.24f * push,
                     0.22f + 0.24f * push, 1.0f});
  drawRingOutline(r, k.cx, k.cy, k.rCenter, 0.8f,
                  Color{0.05f, 0.05f, 0.06f, 0.9f});
}

void drawRingGlow(Renderer& r, const Knob& k, float ringR, float glowR,
                  float dirX, float level) {
  if (level <= 0.0f) return;
  r.fillCircle(k.cx + dirX * ringR, k.cy, glowR,
               Color{1.0f, 1.0f, 1.0f, 0.55f * level});
}

void drawZoomArc(Renderer& r, float cx, float cy, float radius, bool ccw,
                 const Color& c) {
  constexpr int kSegs = 10;
  Point pts[kSegs + 1];
  const float start = ccw ? kPi * 0.55f : -kPi * 0.05f;
  const float end = ccw ? kPi * 1.05f : kPi * 0.45f;
  for (int i = 0; i <= kSegs; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kSegs);
    const float a = start + (end - start) * t;
    pts[i] = {cx + std::cos(a) * radius, cy + std::sin(a) * radius};
  }
  r.strokePolyline(pts, kSegs + 1, 1.4f, c);
  const float tipA = end;
  const float tx = cx + std::cos(tipA) * radius;
  const float ty = cy + std::sin(tipA) * radius;
  const float px = -std::sin(tipA);
  const float py = std::cos(tipA);
  const float s = radius * 0.12f;
  const Point tri[3] = {{tx, ty},
                        {tx - std::cos(tipA) * s + px * s * 0.55f,
                         ty - std::sin(tipA) * s + py * s * 0.55f},
                        {tx - std::cos(tipA) * s - px * s * 0.55f,
                         ty - std::sin(tipA) * s - py * s * 0.55f}};
  r.fillPolygon(tri, 3, c);
}

void drawPanChevron(Renderer& r, const Knob& k, float dirX, float dirY,
                    float level) {
  const float mid = (k.rInner + k.rCenter) * 0.52f;
  const float ax = k.cx + dirX * mid;
  const float ay = k.cy + dirY * mid;
  const float s = (k.rInner - k.rCenter) * 0.42f;
  const float px = -dirY;
  const float py = dirX;
  const Point tri[3] = {{ax + dirX * s, ay + dirY * s},
                        {ax - dirX * s * 0.35f + px * s,
                         ay - dirY * s * 0.35f + py * s},
                        {ax - dirX * s * 0.35f - px * s,
                         ay - dirY * s * 0.35f - py * s}};
  const float bright = 0.50f + 0.50f * level;
  r.fillPolygon(tri, 3, Color{bright, bright, bright, 1.0f});
}

void drawDefaultMapLegend(Renderer& r, const Cell& clrCell, float displayH) {
  const float cap = fontPx(kLegendWt, displayH);
  const float cx = clrCell.x + clrCell.w * 0.5f;
  const float textY = clrCell.y + clrCell.h + cap * 1.05f;
  r.fillText(cx, textY, "DEFAULT", cap, TextAlign::Center, colors::kLabelText);
  r.fillText(cx, textY + cap * 1.05f, "MAP", cap, TextAlign::Center,
             colors::kLabelText);
  const float arrowY = clrCell.y + clrCell.h + cap * 0.35f;
  const Point tri[3] = {{cx, arrowY},
                        {cx - cap * 0.20f, arrowY + cap * 0.28f},
                        {cx + cap * 0.20f, arrowY + cap * 0.28f}};
  r.fillPolygon(tri, 3, colors::kLabelText);
}

void drawFmsKnob(Renderer& r, const Knob& k, float displayH,
                 const float* levels) {
  const float outerCcw = pressLevel(levels, BezelKey::FmsOuterCcw);
  const float outerCw = pressLevel(levels, BezelKey::FmsOuterCw);
  const float innerCcw = pressLevel(levels, BezelKey::FmsInnerCcw);
  const float innerCw = pressLevel(levels, BezelKey::FmsInnerCw);
  const float push = pressLevel(levels, BezelKey::FmsPush);

  r.fillText(k.cx, k.cy - k.rOuter - fontPx(kCaptionWt, displayH) * 0.85f,
             "FMS", fontPx(kCaptionWt, displayH), TextAlign::Center,
             colors::kLabelText);

  drawConcentricKnobBody(r, k, push);

  const float ringR = (k.rOuter + k.rInner) * 0.5f;
  const float glowR = (k.rOuter - k.rInner) * 0.40f;
  drawRingGlow(r, k, ringR, glowR, -1.0f, outerCcw);
  drawRingGlow(r, k, ringR, glowR, 1.0f, outerCw);
  const float innerRingR = (k.rInner + k.rCenter) * 0.5f;
  const float innerGlowR = (k.rInner - k.rCenter) * 0.40f;
  drawRingGlow(r, k, innerRingR, innerGlowR, -1.0f, innerCcw);
  drawRingGlow(r, k, innerRingR, innerGlowR, 1.0f, innerCw);

  r.fillText(k.cx, k.cy + k.rOuter + fontPx(kKnobCaptionWt, displayH) * 0.95f,
             "PUSH CRSR", fontPx(kKnobCaptionWt, displayH), TextAlign::Center,
             colors::kLabelText);
}

void drawRangeJoystick(Renderer& r, const Knob& k, float displayH,
                       const float* levels) {
  const float rangeUp = pressLevel(levels, BezelKey::RangeUp);
  const float rangeDown = pressLevel(levels, BezelKey::RangeDown);
  const float push = pressLevel(levels, BezelKey::PanPush);

  r.fillText(k.cx, k.cy - k.rOuter - fontPx(kCaptionWt, displayH) * 0.85f,
             "RANGE", fontPx(kCaptionWt, displayH), TextAlign::Center,
             colors::kLabelText);

  drawConcentricKnobBody(r, k, push);

  const float ringR = (k.rOuter + k.rInner) * 0.5f;
  const float glowR = (k.rOuter - k.rInner) * 0.40f;
  drawRingGlow(r, k, ringR, glowR, -1.0f, rangeDown);
  drawRingGlow(r, k, ringR, glowR, 1.0f, rangeUp);

  const float zoomSize = fontPx(kCaptionWt, displayH);
  const float arcR = k.rOuter * 0.88f;
  r.fillText(k.cx - ringR, k.cy, "\xE2\x88\x92", zoomSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(k.cx + ringR, k.cy, "+", zoomSize, TextAlign::Center,
             colors::kLabelText);
  drawZoomArc(r, k.cx, k.cy, arcR, true, colors::kLabelText);
  drawZoomArc(r, k.cx, k.cy, arcR, false, colors::kLabelText);

  drawPanChevron(r, k, 0.0f, -1.0f, pressLevel(levels, BezelKey::PanUp));
  drawPanChevron(r, k, 0.0f, 1.0f, pressLevel(levels, BezelKey::PanDown));
  drawPanChevron(r, k, -1.0f, 0.0f, pressLevel(levels, BezelKey::PanLeft));
  drawPanChevron(r, k, 1.0f, 0.0f, pressLevel(levels, BezelKey::PanRight));

  const float cap = fontPx(kKnobCaptionWt, displayH);
  const float legendY = k.cy + k.rOuter + cap * 0.95f;
  r.fillText(k.cx, legendY, "PUSH PAN", cap, TextAlign::Center,
             colors::kLabelText);
  const float panW = cap * 1.6f;
  r.strokeLine(k.cx - panW * 0.5f, legendY + cap * 1.05f,
               k.cx - panW * 0.15f, legendY + cap * 1.05f, 1.2f,
               colors::kLabelText);
  r.strokeLine(k.cx + panW * 0.15f, legendY + cap * 1.05f,
               k.cx + panW * 0.5f, legendY + cap * 1.05f, 1.2f,
               colors::kLabelText);
}

// Generic dual-concentric tuning knob (COM, NAV, CRS/BARO). `topCaption` is
// drawn above the knob and `bottomCaption` below it (e.g. "1/2" or "CRS"); the
// inner/outer rings glow toward the side that was last clicked.
void drawTuningKnob(Renderer& r, const Knob& k, float displayH,
                    const char* topCaption, const char* bottomCaption,
                    const float* levels, BezelKey outerCcw, BezelKey outerCw,
                    BezelKey innerCcw, BezelKey innerCw, BezelKey push) {
  if (topCaption != nullptr) {
    r.fillText(k.cx, k.cy - k.rOuter - fontPx(kCaptionWt, displayH) * 0.85f,
               topCaption, fontPx(kCaptionWt, displayH), TextAlign::Center,
               colors::kLabelText);
  }

  drawConcentricKnobBody(r, k, pressLevel(levels, push));

  const float ringR = (k.rOuter + k.rInner) * 0.5f;
  const float glowR = (k.rOuter - k.rInner) * 0.40f;
  drawRingGlow(r, k, ringR, glowR, -1.0f, pressLevel(levels, outerCcw));
  drawRingGlow(r, k, ringR, glowR, 1.0f, pressLevel(levels, outerCw));
  const float innerRingR = (k.rInner + k.rCenter) * 0.5f;
  const float innerGlowR = (k.rInner - k.rCenter) * 0.40f;
  drawRingGlow(r, k, innerRingR, innerGlowR, -1.0f, pressLevel(levels, innerCcw));
  drawRingGlow(r, k, innerRingR, innerGlowR, 1.0f, pressLevel(levels, innerCw));

  if (bottomCaption != nullptr) {
    r.fillText(k.cx, k.cy + k.rOuter + fontPx(kKnobCaptionWt, displayH) * 0.95f,
               bottomCaption, fontPx(kKnobCaptionWt, displayH),
               TextAlign::Center, colors::kLabelText);
  }
}

// Single-rotary knob with a push cap (the HDG knob, and the small VOL/SQ and
// VOL/ID knobs). The body reuses the concentric look but only the outer ring
// glows on rotation.
void drawSingleKnob(Renderer& r, const Knob& k, float displayH,
                    const char* topCaption, const char* bottomCaption,
                    const float* levels, BezelKey ccw, BezelKey cw,
                    BezelKey push) {
  if (topCaption != nullptr) {
    r.fillText(k.cx, k.cy - k.rOuter - fontPx(kKnobCaptionWt, displayH) * 0.85f,
               topCaption, fontPx(kKnobCaptionWt, displayH), TextAlign::Center,
               colors::kLabelText);
  }

  const float push01 = pressLevel(levels, push);
  r.fillCircle(k.cx, k.cy, k.rOuter, Color{0.11f, 0.12f, 0.14f, 1.0f});
  drawKnurledRing(r, k.cx, k.cy, k.rInner + (k.rOuter - k.rInner) * 0.18f,
                  k.rOuter * 0.98f);
  drawRingOutline(r, k.cx, k.cy, k.rOuter * 0.98f, 1.0f,
                  Color{0.04f, 0.04f, 0.05f, 1.0f});
  r.fillCircle(k.cx, k.cy, k.rInner,
               Color{0.18f + 0.24f * push01, 0.19f + 0.24f * push01,
                     0.21f + 0.24f * push01, 1.0f});
  drawRingOutline(r, k.cx, k.cy, k.rInner, 0.8f,
                  Color{0.05f, 0.05f, 0.06f, 0.9f});

  const float ringR = (k.rOuter + k.rInner) * 0.5f;
  const float glowR = (k.rOuter - k.rInner) * 0.40f;
  drawRingGlow(r, k, ringR, glowR, -1.0f, pressLevel(levels, ccw));
  drawRingGlow(r, k, ringR, glowR, 1.0f, pressLevel(levels, cw));

  if (bottomCaption != nullptr) {
    r.fillText(k.cx, k.cy + k.rOuter + fontPx(kKnobCaptionWt, displayH) * 0.95f,
               bottomCaption, fontPx(kKnobCaptionWt, displayH),
               TextAlign::Center, colors::kLabelText);
  }
}

// Frequency transfer key: a small cap with a double-headed horizontal arrow
// (the flip-flop symbol next to the COM/NAV knobs).
void drawTransferKey(Renderer& r, const Cell& c, float press) {
  bezel::drawKeyFace(r, c.x, c.y, c.w, c.h, press);
  const float cy = c.y + c.h * 0.5f;
  const float len = c.w * 0.30f;
  const float cx = c.x + c.w * 0.5f;
  const float head = c.h * 0.16f;
  r.strokeLine(cx - len, cy, cx + len, cy, 1.8f, colors::kWhite);
  const Point left[3] = {{cx - len - head * 0.55f, cy},
                         {cx - len + head * 0.45f, cy - head},
                         {cx - len + head * 0.45f, cy + head}};
  r.fillPolygon(left, 3, colors::kWhite);
  const Point right[3] = {{cx + len + head * 0.55f, cy},
                          {cx + len - head * 0.45f, cy - head},
                          {cx + len - head * 0.45f, cy + head}};
  r.fillPolygon(right, 3, colors::kWhite);
}

// Hit-test a dual-concentric knob: center push, then inner ring (left/right =
// ccw/cw), then outer ring. Returns BezelKey::Count when outside.
BezelKey hitConcentric(const Knob& k, float xPx, float yPx, BezelKey outerCcw,
                       BezelKey outerCw, BezelKey innerCcw, BezelKey innerCw,
                       BezelKey push) {
  const float dx = xPx - k.cx;
  const float dy = yPx - k.cy;
  const float d = std::sqrt(dx * dx + dy * dy);
  if (d <= k.rCenter) return push;
  if (d <= k.rInner) return dx < 0.0f ? innerCcw : innerCw;
  if (d <= k.rOuter) return dx < 0.0f ? outerCcw : outerCw;
  return BezelKey::Count;
}

// Hit-test a single-rotary knob: center push, then the ring (left/right).
BezelKey hitSingle(const Knob& k, float xPx, float yPx, BezelKey ccw,
                   BezelKey cw, BezelKey push) {
  const float dx = xPx - k.cx;
  const float dy = yPx - k.cy;
  const float d = std::sqrt(dx * dx + dy * dy);
  if (d <= k.rInner) return push;
  if (d <= k.rOuter) return dx < 0.0f ? ccw : cw;
  return BezelKey::Count;
}

bool inCell(const Cell& c, float xPx, float yPx) {
  return xPx >= c.x && xPx <= c.x + c.w && yPx >= c.y && yPx <= c.y + c.h;
}

}  // namespace

void BezelKeyPanel::render(Renderer& r, float x, float y, float w, float h,
                           float displayH, const float* pressLevels) {
  r.fillRectVerticalGradient(x, y, w, h, y, y + h, bezel::kFaceTop,
                             bezel::kFaceBottom);
  r.strokeLine(x, y, x, y + h, 2.0f, colors::kPanelBorder);

  const ClusterLayout cluster = layoutCluster(x, y, w, h);
  const float labelSize = fontPx(kLabelWt, displayH);
  Cell clrCell{};

  for (int i = 0; i < kBezelButtonCount; ++i) {
    const Cell cell = gridCell(i, x, w, cluster);
    const BezelKey key = static_cast<BezelKey>(i);
    const float press = pressLevels ? std::max(0.0f, pressLevels[i]) : 0.0f;

    bezel::drawKeyFace(r, cell.x, cell.y, cell.w, cell.h, press);

    const float cx = cell.x + cell.w * 0.5f;
    const float cy = cell.y + cell.h * 0.5f;

    if (key == BezelKey::DirectTo) {
      drawDirectToGlyph(r, cell, displayH, colors::kWhite);
    } else {
      r.fillText(cx, cy, keyLabel(key), labelSize, TextAlign::Center,
                 colors::kWhite);
    }

    if (key == BezelKey::Clr) clrCell = cell;
  }

  if (clrCell.w > 0.0f) drawDefaultMapLegend(r, clrCell, displayH);

  // Top row: COM VOL/SQ knob and the COM frequency transfer key.
  drawSingleKnob(r, volKnobIn(x, w, cluster.volRowTop, cluster.volRowH, false),
                 displayH, "VOL SQ", nullptr, pressLevels, BezelKey::ComVolCcw,
                 BezelKey::ComVolCw, BezelKey::ComVolPush);
  drawTransferKey(r, transferCellIn(x, w, cluster.volRowTop, cluster.volRowH,
                                    false),
                  pressLevel(pressLevels, BezelKey::ComTransfer));

  // COM tuning knob (1/2 select on push) and the CRS/BARO knob (large = BARO,
  // small = CRS, push = standard baro).
  drawTuningKnob(r, comKnobRect(x, w, cluster), displayH, "COM", "PUSH 1-2",
                 pressLevels, BezelKey::ComOuterCcw, BezelKey::ComOuterCw,
                 BezelKey::ComInnerCcw, BezelKey::ComInnerCw, BezelKey::ComPush);
  drawTuningKnob(r, baroKnobRect(x, w, cluster), displayH, "BARO", "PUSH STD",
                 pressLevels, BezelKey::BaroCcw, BezelKey::BaroCw,
                 BezelKey::CrsCcw, BezelKey::CrsCw, BezelKey::CrsPush);

  drawRangeJoystick(r, rangeJoyRect(x, w, cluster), displayH, pressLevels);
  drawFmsKnob(r, fmsKnobRect(x, w, cluster), displayH, pressLevels);
}

void BezelKeyPanel::renderLeft(Renderer& r, float x, float y, float w, float h,
                               float displayH, const float* pressLevels) {
  r.fillRectVerticalGradient(x, y, w, h, y, y + h, bezel::kFaceTop,
                             bezel::kFaceBottom);
  r.strokeLine(x + w, y, x + w, y + h, 2.0f, colors::kPanelBorder);

  const LeftLayout cluster = layoutLeft(x, y, w, h);

  // Top row: NAV VOL/ID knob and the NAV frequency transfer key.
  drawSingleKnob(r, volKnobIn(x, w, cluster.volRowTop, cluster.volRowH, true),
                 displayH, "VOL ID", nullptr, pressLevels, BezelKey::NavVolCcw,
                 BezelKey::NavVolCw, BezelKey::NavVolPush);
  drawTransferKey(r, transferCellIn(x, w, cluster.volRowTop, cluster.volRowH,
                                    true),
                  pressLevel(pressLevels, BezelKey::NavTransfer));

  // NAV tuning knob (1/2 select on push) and the HDG knob.
  drawTuningKnob(r, knobInSlot(x, w, cluster.navTop, cluster.navH), displayH,
                 "NAV", "PUSH 1-2", pressLevels, BezelKey::NavOuterCcw,
                 BezelKey::NavOuterCw, BezelKey::NavInnerCcw,
                 BezelKey::NavInnerCw, BezelKey::NavPush);
  drawSingleKnob(r, knobInSlot(x, w, cluster.hdgTop, cluster.hdgH), displayH,
                 "HDG", "PUSH HDG SYNC", pressLevels, BezelKey::HdgCcw,
                 BezelKey::HdgCw, BezelKey::HdgPush);

  const Cell backup = backupCell(x, w, cluster);
  drawDisplayBackupKey(r, backup, displayH,
                       pressLevel(pressLevels, BezelKey::DisplayBackup));
}

BezelKey BezelKeyPanel::hitTest(float xPx, float yPx, float x, float y, float w,
                                float h) {
  const ClusterLayout cluster = layoutCluster(x, y, w, h);

  for (int i = 0; i < kBezelButtonCount; ++i) {
    const Cell c = gridCell(i, x, w, cluster);
    if (xPx >= c.x && xPx <= c.x + c.w && yPx >= c.y && yPx <= c.y + c.h) {
      return static_cast<BezelKey>(i);
    }
  }

  const Knob rj = rangeJoyRect(x, w, cluster);
  {
    const float dx = xPx - rj.cx;
    const float dy = yPx - rj.cy;
    const float d = std::sqrt(dx * dx + dy * dy);
    if (d <= rj.rCenter) return BezelKey::PanPush;
    if (d <= rj.rInner) {
      if (std::fabs(dx) > std::fabs(dy)) {
        return dx < 0.0f ? BezelKey::PanLeft : BezelKey::PanRight;
      }
      return dy < 0.0f ? BezelKey::PanUp : BezelKey::PanDown;
    }
    if (d <= rj.rOuter * kRangeZoomHitMargin) {
      return dx < 0.0f ? BezelKey::RangeDown : BezelKey::RangeUp;
    }
  }

  const BezelKey fms = hitConcentric(
      fmsKnobRect(x, w, cluster), xPx, yPx, BezelKey::FmsOuterCcw,
      BezelKey::FmsOuterCw, BezelKey::FmsInnerCcw, BezelKey::FmsInnerCw,
      BezelKey::FmsPush);
  if (fms != BezelKey::Count) return fms;

  const BezelKey com = hitConcentric(
      comKnobRect(x, w, cluster), xPx, yPx, BezelKey::ComOuterCcw,
      BezelKey::ComOuterCw, BezelKey::ComInnerCcw, BezelKey::ComInnerCw,
      BezelKey::ComPush);
  if (com != BezelKey::Count) return com;

  const BezelKey baro = hitConcentric(
      baroKnobRect(x, w, cluster), xPx, yPx, BezelKey::BaroCcw, BezelKey::BaroCw,
      BezelKey::CrsCcw, BezelKey::CrsCw, BezelKey::CrsPush);
  if (baro != BezelKey::Count) return baro;

  const BezelKey vol =
      hitSingle(volKnobIn(x, w, cluster.volRowTop, cluster.volRowH, false), xPx,
                yPx, BezelKey::ComVolCcw, BezelKey::ComVolCw,
                BezelKey::ComVolPush);
  if (vol != BezelKey::Count) return vol;
  if (inCell(transferCellIn(x, w, cluster.volRowTop, cluster.volRowH, false),
             xPx, yPx)) {
    return BezelKey::ComTransfer;
  }

  return BezelKey::Count;
}

BezelKey BezelKeyPanel::hitTestLeft(float xPx, float yPx, float x, float y,
                                    float w, float h) {
  const LeftLayout cluster = layoutLeft(x, y, w, h);

  const BezelKey nav = hitConcentric(
      knobInSlot(x, w, cluster.navTop, cluster.navH), xPx, yPx,
      BezelKey::NavOuterCcw, BezelKey::NavOuterCw, BezelKey::NavInnerCcw,
      BezelKey::NavInnerCw, BezelKey::NavPush);
  if (nav != BezelKey::Count) return nav;

  const BezelKey hdg =
      hitSingle(knobInSlot(x, w, cluster.hdgTop, cluster.hdgH), xPx, yPx,
                BezelKey::HdgCcw, BezelKey::HdgCw, BezelKey::HdgPush);
  if (hdg != BezelKey::Count) return hdg;

  const BezelKey vol =
      hitSingle(volKnobIn(x, w, cluster.volRowTop, cluster.volRowH, true), xPx,
                yPx, BezelKey::NavVolCcw, BezelKey::NavVolCw,
                BezelKey::NavVolPush);
  if (vol != BezelKey::Count) return vol;
  if (inCell(transferCellIn(x, w, cluster.volRowTop, cluster.volRowH, true),
             xPx, yPx)) {
    return BezelKey::NavTransfer;
  }

  if (inCell(backupCell(x, w, cluster), xPx, yPx)) {
    return BezelKey::DisplayBackup;
  }

  return BezelKey::Count;
}

}  // namespace avionics
