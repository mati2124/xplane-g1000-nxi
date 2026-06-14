#include <algorithm>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// Always-on Crew Alerting System annunciation window on the main PFD. Per the
// G1000 Pilot's Guide (Fig. A-1) it sits to the right of the Vertical Speed
// Indicator at mid-display height, in a bordered box, with left-aligned text
// colored by severity (warning red, caution amber, advisory white). Higher
// priority messages are at the top; the list grows downward.
void drawCasAnnunciations(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
  const auto& msgs = ui.annunciations();
  if (msgs.empty()) return;

  const float msgSize = fontPx(wt::kInfoLabel, h);
  const float lineH = msgSize * 1.55f;
  const float padX = msgSize * 0.35f;
  const float padY = msgSize * 0.30f;

  // Clip the number of rows to what fits between the fixed top and the info
  // panel (matches the G1000's ~14-message capacity at this scale).
  const float maxBottom = L.infoPanelTop - lineH * 0.2f;
  const float boxW = L.casAnnunW;
  const float boxH = std::min(maxBottom - L.casAnnunTop,
                              padY * 2.0f + lineH * static_cast<float>(msgs.size()));

  // Faint plate + thin border, as the annunciation window is drawn in Fig. A-1.
  r.fillRect(L.casAnnunLeft, L.casAnnunTop, boxW, boxH,
             withAlpha(colors::kBlack, 0.55f));
  const Point border[5] = {{L.casAnnunLeft, L.casAnnunTop},
                           {L.casAnnunLeft + boxW, L.casAnnunTop},
                           {L.casAnnunLeft + boxW, L.casAnnunTop + boxH},
                           {L.casAnnunLeft, L.casAnnunTop + boxH},
                           {L.casAnnunLeft, L.casAnnunTop}};
  r.strokePolyline(border, 5, 1.5f, colors::kPanelBorder);

  const float textX = L.casAnnunLeft + padX;
  float y = L.casAnnunTop + padY + lineH * 0.5f;
  for (const AlertMessage& m : msgs) {
    if (y > L.casAnnunTop + boxH - lineH * 0.3f) break;
    const Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
                    : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                     : colors::kWhite;
    r.fillText(textX, y, m.text, msgSize, TextAlign::Left, c);
    y += lineH;
  }
}

}  // namespace avionics::pfd
