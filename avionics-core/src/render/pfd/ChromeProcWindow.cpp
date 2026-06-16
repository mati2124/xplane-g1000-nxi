#include <algorithm>
#include <string>
#include <vector>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {
namespace {

// One selectable row: highlighted rows pulse as a cyan plate with black text
// (~1 Hz, like the other PFD menus); disabled rows are grey, the rest white.
void drawMenuRow(Renderer& r, float textX, float plateX, float plateW, float cy,
                 const std::string& text, float size, bool selected,
                 bool enabled, bool blinkOn, float a) {
  if (selected && blinkOn) {
    r.fillRect(plateX, cy - size * 0.62f, plateW, size * 1.24f,
               withAlpha(colors::kCyan, a));
    r.fillText(textX, cy, text, size, TextAlign::Left,
               withAlpha(colors::kBlack, a));
    return;
  }
  const Color color = selected    ? colors::kCyan
                      : enabled    ? colors::kWhite
                                   : colors::kDisabledGray;
  r.fillText(textX, cy, text, size, TextAlign::Left, withAlpha(color, a));
}

}  // namespace

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8 "Procedures"): the
// lower-right popout the other PFD windows share. The top-level menu lists the
// approach-activation items (disabled in this suite) and the Select Approach /
// Arrival / Departure items; selecting one switches the window to the
// procedure-selection list (procedure names, then transitions) that loads the
// chosen procedure into the active flight plan.
void drawProcWindow(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float rawAnim = ui.windowAnim(PfdWindow::Procedures);
  if (rawAnim <= 0.0f) return;

  const FontScope fs(r, FontFace::DejaVuSemiBold);
  float panelW = 0.0f;
  float panelH = 0.0f;
  tallPopoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f = drawWindowFrame(r, w, h, L, rawAnim,
                                        ui.procWindowTitle(), panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const float rowSize = fontPx(wt::kInfoValue, h);
  const float rowH = h * (kWtPageMenuRowHeightPx / kWtCanvasHeightPx);
  const float pad = f.w * 0.05f;
  const float textX = f.x + pad;
  const float plateX = f.x + pad * 0.5f;
  const float plateW = f.w - pad;
  const bool blinkOn = ui.blinkOn();
  float listTop = f.contentTop + rowSize * 0.9f;

  if (!ui.procSelectMode()) {
    // Top-level menu: the six Procedures items.
    const int n = ui.procMenuItemCount();
    for (int i = 0; i < n; ++i) {
      const float cy = listTop + rowH * (static_cast<float>(i) + 0.5f);
      drawMenuRow(r, textX, plateX, plateW, cy, ui.procMenuItemText(i), rowSize,
                  i == ui.procMenuSelected(), ui.procMenuItemEnabled(i), blinkOn,
                  a);
    }
    return;
  }

  // Selection sub-window: the airport header, then the procedure / transition
  // list. On the transition step the chosen procedure name heads the list.
  const std::string icao = ui.procAirportIcao();
  r.fillText(textX, listTop, icao.empty() ? "_____" : icao, rowSize,
             TextAlign::Left, withAlpha(colors::kCyan, a));
  if (ui.procStep() == SoftkeyController::ProcStep::TransitionList) {
    r.fillText(f.x + f.w - pad, listTop, ui.procSelectedName(), rowSize,
               TextAlign::Right, withAlpha(colors::kCyan, a));
  }
  const float sepY = listTop + rowSize * 0.45f;
  r.strokeLine(f.x + pad * 0.5f, sepY, f.x + f.w - pad * 0.5f, sepY, 1.0f,
               withAlpha(colors::kPanelSeparator, a));
  listTop = sepY + rowSize * 0.55f;

  const std::vector<std::string> items = ui.procListItems();
  if (items.empty()) {
    r.fillText(f.x + f.w * 0.5f, listTop + rowH, "NO PROCEDURES", rowSize,
               TextAlign::Center, withAlpha(colors::kTitleGray, a));
    return;
  }

  const float listH = f.top + f.h - listTop - pad;
  const int maxRows = std::max(1, static_cast<int>(listH / rowH));
  const int total = static_cast<int>(items.size());
  const int selected = std::min(std::max(0, ui.procListSelected()), total - 1);
  int first = 0;
  if (total > maxRows) {
    first = std::max(0, std::min(selected - maxRows / 2, total - maxRows));
  }
  const int end = std::min(total, first + maxRows);
  for (int i = first; i < end; ++i) {
    const float cy = listTop + rowH * (static_cast<float>(i - first) + 0.5f);
    drawMenuRow(r, textX, plateX, plateW, cy, items[static_cast<std::size_t>(i)],
                rowSize, i == selected, true, blinkOn, a);
  }

  // Scroll bar when the list overflows the viewport.
  if (total > maxRows) {
    const float barX = f.x + f.w - pad * 0.4f;
    const float barTop = listTop;
    const float barH = static_cast<float>(maxRows) * rowH;
    r.strokeLine(barX, barTop, barX, barTop + barH, 1.0f,
                 withAlpha(colors::kTitleGray, a));
    const float thumbH = barH * static_cast<float>(maxRows) /
                         static_cast<float>(total);
    const float thumbY = barTop + (barH - thumbH) * static_cast<float>(first) /
                                      static_cast<float>(total - maxRows);
    r.fillRect(barX - 1.0f, thumbY, 2.0f, thumbH, withAlpha(colors::kCyan, a));
  }
}

}  // namespace avionics::pfd
