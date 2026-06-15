#include <algorithm>
#include <string>

#include "avionics/render/SoftkeyLabelBar.h"
#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// On-screen softkey label bar. Display only, like the real unit: the labels
// name the functions of the physical keys on the bezel directly below the
// screen, and all interaction happens through those keys. The bar is the
// shared NXi gradient strip with notched separators (no per-cell boxes); a
// selected key shows black text on a light gradient (G1000 NXi Pilot's Guide
// Fig. 1-9, "Softkey Function"), and a press flashes the same look momentarily.
void drawSoftkeyBar(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float top = h - L.bottomBarH;
  render::drawSoftkeyBarBackground(r, w, top, L.bottomBarH, kSoftkeyCount);

  const float cellW = w / static_cast<float>(kSoftkeyCount);
  const float size = fontPx(wt::kSoftkey, h);
  for (int i = 0; i < kSoftkeyCount; ++i) {
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 1.0f : 0.0f);

    Color labelColor = colors::kWhite;
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
    render::drawSoftkeyCell(r, static_cast<float>(i) * cellW, top, cellW,
                            L.bottomBarH, ui.label(i), level, labelColor, size);
  }
}

}  // namespace avionics::pfd
