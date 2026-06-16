#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// Pop-up Alerts/messages window (under the "Alerts" key).
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // Same taller popout as Direct-To / Page Menu / PFD Setup (Pilot's Guide Fig.
  // 1-17 shows the Alerts window as the same lower-right box, with messages
  // listed from the top over an otherwise empty panel).
  float panelW = 0.0f;
  float panelH = 0.0f;
  tallPopoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Alerts), "Alerts",
                      panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  // Message list (or an empty-state line). Standard popout body size, shared
  // with the other PFD popups.
  const float msgSize = fontPx(wt::kInfoValue, h);
  const float lineH = msgSize * 1.6f;
  const float textX = f.x + f.w * 0.04f;
  float y = f.contentTop + lineH * 0.75f;

  const auto& msgs = ui.alerts();
  if (msgs.empty()) {
    r.fillText(f.x + f.w * 0.5f, f.top + f.h * 0.55f, "NO ACTIVE ALERTS",
               msgSize, TextAlign::Center, withAlpha(colors::kLabelText, a));
    return;
  }
  for (const AlertMessage& m : msgs) {
    if (y > f.top + f.h - lineH * 0.4f) break;  // clip overflow
    Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
              : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                 : colors::kWhite;
    r.fillText(textX, y, m.text, msgSize, TextAlign::Left, withAlpha(c, a));
    y += lineH;
  }
}

}  // namespace avionics::pfd
