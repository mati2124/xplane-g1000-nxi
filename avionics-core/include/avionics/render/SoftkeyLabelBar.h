#pragma once

#include <algorithm>
#include <string>

#include "avionics/Color.h"
#include "avionics/Renderer.h"
#include "avionics/render/CursorHighlight.h"  // withAlpha

namespace avionics::render {

// Shared on-screen softkey label bar, modeled on the G1000 NXi Pilot's Guide
// Fig. 1-9 (and the real-unit photo). Every cell is a top-rounded cap filled
// with a dark blue-grey vertical gradient and edged along its top by a bright
// horizontal highlight; the caps sit on a near-black bar and are separated by
// thin dark grooves. A selected/pressed cap fills with the light "Softkey
// Function" gradient and the label flips to black. Used by both the PFD and
// MFD so the footer looks identical on each display.

// Cap geometry shared by the cells. The real caps have only a slight top
// rounding (~12% of bar height), not a pill-shaped top.
inline float softkeyCapRadius(float barH) { return barH * 0.12f; }

// Draws the bar base (near-black) across [0, w] at `top` with height `barH`.
// The dark base shows through the thin grooves between the per-cell caps.
inline void drawSoftkeyBarBackground(Renderer& r, float w, float top,
                                     float barH, int /*cellCount*/) {
  r.fillRect(0.0f, top, w, barH, colors::kSoftkeyBarBase);
}

// Draws one softkey cap and its label. The cell spans [x, x + cellW] of the
// bar at `top` with height `barH`. `level` (0..1) is the selected/press
// highlight; otherwise the label is drawn in `labelColor`.
inline void drawSoftkeyCell(Renderer& r, float x, float top, float cellW,
                            float barH, const std::string& label, float level,
                            const Color& labelColor, float fontPx) {
  const float gap = cellW * 0.012f;          // thin dark groove between caps
  const float topMargin = barH * 0.04f;
  const float bx = x + gap;
  const float by = top + topMargin;
  const float bw = cellW - 2.0f * gap;
  const float bh = barH - topMargin;         // square bottom flush with bar
  const float radius = softkeyCapRadius(barH);

  // Dark cap (always present on the real unit).
  r.fillTopRoundedRectVerticalGradient(bx, by, bw, bh, radius,
                                       colors::kSoftkeyCapTop,
                                       colors::kSoftkeyCapBottom);
  // Selected/pressed: overlay the light gradient, fading in with the press.
  if (level > 0.0f) {
    r.fillTopRoundedRectVerticalGradient(
        bx, by, bw, bh, radius, withAlpha(colors::kSoftkeySelectedTop, level),
        withAlpha(colors::kSoftkeySelected, level));
  }
  // Bright top-edge highlight (the horizontal line across each cap top), inset
  // by the corner radius so it tracks the flat part of the rounded top.
  r.strokeLine(bx + radius, by + 0.5f, bx + bw - radius, by + 0.5f, 1.0f,
               colors::kSoftkeyCapHighlight);

  if (!label.empty()) {
    const Color c = level > 0.5f ? colors::kBlack : labelColor;
    // The real GDU renders softkey labels in a bold weight.
    r.fillText(x + cellW * 0.5f, top + barH * 0.5f, label, fontPx,
               TextAlign::Center, c, FontFace::RobotoBold);
  }
}

}  // namespace avionics::render
