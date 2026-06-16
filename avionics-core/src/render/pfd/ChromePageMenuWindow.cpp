#include "render/pfd/ChromeInternal.h"

#include <algorithm>

namespace avionics::pfd {
namespace {

void drawOptionsGroup(Renderer& r, float x, float y, float w, float h,
                      const char* title, float titleSize, float alpha) {
  const Point box[5] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}};
  r.strokePolyline(box, 5, 1.0f, withAlpha(colors::kGroupBoxBorder, alpha));
  if (title != nullptr && title[0] != '\0') {
    const float tx = x + titleSize * 0.35f;
    const float tw = r.measureTextWidth(title, titleSize);
    const float padX = titleSize * 0.35f;
    r.fillRect(tx - padX, y - titleSize * 0.62f, tw + 2.0f * padX,
               titleSize * 1.24f, withAlpha(colors::kBlack, alpha));
    r.fillText(tx, y, title, titleSize, TextAlign::Left,
               withAlpha(colors::kWhite, alpha));
  }
}

}  // namespace

// Page Menu (MENU key on an open PFD popout, Pilot's Guide Fig. 1-10): the
// 310x220 lower-right shell with an "Options" group box and up to three visible
// rows (72 px list viewport on the 768-tall GDU canvas).
void drawPageMenuWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const float rawAnim = ui.pageMenuAnim();
  if (rawAnim <= 0.0f) return;

  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // Same taller 310-wide popout as the Direct-To window; the Options group box
  // fills the body, matching the populated menu in Pilot's Guide Fig. 1-10.
  float panelW = 0.0f;
  float panelH = 0.0f;
  tallPopoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, rawAnim, "Page Menu", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const float rowSize = fontPx(wt::kInfoValue, h);
  const float rowH = h * (kWtPageMenuRowHeightPx / kWtCanvasHeightPx);
  const float pad = f.w * 0.04f;
  const float groupX = f.x + pad;
  const float groupW = f.w - 2.0f * pad;
  const float groupY = f.contentTop + fontPx(4.0f, h);
  const float groupBottom = f.top + f.h - pad;
  const float groupH = groupBottom - groupY;

  drawOptionsGroup(r, groupX, groupY, groupW, groupH, "Options", rowSize, a);

  const int n = ui.pageMenuItemCount();
  const float textX = groupX + pad * 0.5f;
  const float listTop = groupY + rowSize * 0.9f;
  const float listH = groupBottom - listTop - rowSize * 0.4f;

  if (n <= 0) {
    r.fillText(groupX + groupW * 0.5f, listTop + listH * 0.5f, "No Options",
               rowSize, TextAlign::Center, withAlpha(colors::kWhite, a));
    return;
  }

  const int maxRows = std::max(1, static_cast<int>(listH / rowH));
  const int scroll = ui.pageMenuScrollOffset();
  const int visible = std::min(n - scroll, maxRows);

  for (int row = 0; row < visible; ++row) {
    const int i = scroll + row;
    const float cy = listTop + rowH * 0.5f + row * rowH;
    const std::string& text = ui.pageMenuItemText(i);
    const bool enabled = ui.pageMenuItemEnabled(i);
    if (i == ui.pageMenuSelected() && enabled) {
      if (ui.blinkOn()) {
        r.fillRect(textX - pad * 0.25f, cy - rowSize * 0.62f,
                   groupW - pad * 0.5f, rowSize * 1.24f,
                   withAlpha(colors::kCyan, a));
        r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                   withAlpha(colors::kBlack, a));
      } else {
        r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                   withAlpha(colors::kCyan, a));
      }
    } else {
      r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                 withAlpha(enabled ? colors::kWhite : colors::kDisabledGray,
                           a));
    }
  }

  if (n > visible) {
    const float barX = groupX + groupW - pad * 0.35f;
    const float barTop = listTop;
    const float barH = listH;
    r.strokeLine(barX, barTop, barX, barTop + barH, 1.0f,
                 withAlpha(colors::kTitleGray, a));
    const float thumbH = barH * (static_cast<float>(visible) / n);
    const float thumbY =
        barTop + (barH - thumbH) * (static_cast<float>(scroll) / (n - visible));
    r.fillRect(barX - 1.0f, thumbY, 2.0f, thumbH, withAlpha(colors::kCyan, a));
  }
}

}  // namespace avionics::pfd
