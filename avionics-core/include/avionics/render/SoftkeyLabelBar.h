#pragma once

#include <algorithm>
#include <cstring>
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

// Cap geometry shared by the cells. WT SoftKey.css uses 5px top radii on a
// 34px bar (~15%); the caps are nearly full-height with a thin top margin.
inline float softkeyCapRadius(float barH) {
  return std::min(barH * 0.147f, 5.0f);
}

// Draws the bar base (near-black) across [0, w] at `top` with height `barH`.
// The dark base shows through the thin grooves between the per-cell caps.
inline void drawSoftkeyBarBackground(Renderer& r, float w, float top,
                                     float barH, int /*cellCount*/) {
  r.fillRect(0.0f, top, w, barH, colors::kSoftkeyBarBase);
}

// Draws one softkey cap and its label. The cell spans [x, x + cellW] of the
// bar at `top` with height `barH`. `level` (0..1) is the selected/press
// highlight; otherwise the label is drawn in `labelColor`. When `valueSuffix`
// is non-empty the main label is left-aligned and the suffix is right-aligned
// in `valueColor` (WT SoftKey.css: --value-color cyan).
inline void drawSoftkeyCell(Renderer& r, float x, float top, float cellW,
                            float barH, const std::string& label, float level,
                            const Color& labelColor, float fontPx,
                            const std::string& valueSuffix = {},
                            const Color& valueColor = colors::kCyan) {
  const float gap = std::max(1.0f, cellW * 0.012f);  // ~1px groove between caps
  const float topMargin = std::max(1.0f, barH * 0.03f);
  const float bx = x + gap;
  const float by = top + topMargin;
  const float bw = cellW - 2.0f * gap;
  const float bh = barH - topMargin;  // square bottom flush with the bar base
  const float radius = softkeyCapRadius(barH);
  const float labelCy = by + bh * 0.52f;  // WT softkey-tab padding-top ~6px

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

  if (label.empty() && valueSuffix.empty()) return;

  const bool pressed = level > 0.5f;
  const Color mainColor = pressed ? colors::kBlack : labelColor;
  const Color suffixColor = pressed ? colors::kBlack : valueColor;
  const float pad = std::max(3.0f, cellW * 0.04f);  // WT softkey-tab margin ~4px

  if (!valueSuffix.empty()) {
    if (!label.empty()) {
      r.fillText(x + gap + pad, labelCy, label, fontPx, TextAlign::Left,
                 mainColor, FontFace::RobotoBold);
    }
    r.fillText(x + cellW - gap - pad, labelCy, valueSuffix, fontPx,
               TextAlign::Right, suffixColor, FontFace::RobotoBold);
  } else {
    r.fillText(x + cellW * 0.5f, labelCy, label, fontPx, TextAlign::Center,
               mainColor, FontFace::RobotoBold);
  }
}

// Splits a state-carrying softkey label ("Detail All", "TER Topo", ...) into
// the fixed function name and the cyan state suffix shown on the real unit.
inline bool splitStateSoftkeyLabel(const std::string& label,
                                   std::string& mainOut,
                                   std::string& valueOut) {
  static constexpr const char* kPrefixes[] = {"Detail ", "TER ", "AWY "};
  for (const char* prefix : kPrefixes) {
    if (label.rfind(prefix, 0) != 0) continue;
    mainOut.assign(prefix, prefix + std::strlen(prefix) - 1);
    valueOut = label.substr(std::strlen(prefix));
    return true;
  }
  return false;
}

}  // namespace avionics::render
