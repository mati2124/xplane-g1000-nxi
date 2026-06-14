#include <algorithm>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// On-screen softkey label bar. Display only, like the real unit: the labels
// name the functions of the physical keys on the bezel directly below the
// screen, and all interaction happens through those keys. Each cell is a
// thin-framed label box; a selected key shows black text on a gray background
// (G1000 Pilot's Guide for the Diamond DA40, "Softkey Function"), and a press
// flashes the same look momentarily.
void drawSoftkeyBar(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float top = h - L.bottomBarH;
  r.fillRect(0.0f, top, w, L.bottomBarH, colors::kSoftkeyBackground);

  const float cellW = w / static_cast<float>(kSoftkeyCount);
  const float cy = top + L.bottomBarH * 0.5f;
  const float size = fontPx(wt::kSoftkey, h);
  const float insetX = cellW * 0.055f;
  const float insetY = L.bottomBarH * 0.13f;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    const float bx = static_cast<float>(i) * cellW + insetX;
    const float by = top + insetY;
    const float bw = cellW - 2.0f * insetX;
    const float bh = L.bottomBarH - 2.0f * insetY;
    // Real GDU softkey labels sit in caps whose top corners are rounded while
    // the bottom corners stay square; keep the radius modest vs. box height.
    const float radius = bh * 0.30f;

    // Selected = steady black-on-gray; a key press flashes the same fill,
    // decaying back to white-on-black. The cap is lighter at the top fading to
    // the base gray, like the real unit.
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 1.0f : 0.0f);
    if (level > 0.0f) {
      r.fillTopRoundedRectVerticalGradient(
          bx, by, bw, bh, radius, withAlpha(colors::kSoftkeySelectedTop, level),
          withAlpha(colors::kSoftkeySelected, level));
    }
    r.strokeTopRoundedRect(bx, by, bw, bh, radius, 1.0f,
                           colors::kPanelSeparator);

    if (ui.label(i)[0] != '\0') {
      Color labelColor = level > 0.5f ? colors::kBlack : colors::kWhite;
      // The Alerts cell relabels and flashes by severity when alerts are
      // unacknowledged (warning red, caution amber, message advisory white).
      if (ui.softkeyFlashing(i)) {
        labelColor = (ui.alertSoftkeyLevel() == AlertLevel::Warning)
                         ? colors::kBandRed
                     : (ui.alertSoftkeyLevel() == AlertLevel::Caution)
                         ? colors::kBandYellow
                         : colors::kWhite;
        if (!ui.blinkOn()) labelColor = withAlpha(labelColor, 0.0f);
      }
      r.fillText(bx + bw * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 labelColor);
    }
  }
}

}  // namespace avionics::pfd
