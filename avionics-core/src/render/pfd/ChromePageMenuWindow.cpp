#include "render/pfd/ChromeInternal.h"

#include <algorithm>

namespace avionics::pfd {
namespace {

void drawOptionsGroup(Renderer& r, float x, float y, float w, float h,
                      const char* title, float labelSize, float displayH,
                      float alpha) {
  // Black interior list area inside the gray menu body (trainer: rgb(0,0,0)
  // behind the option rows), with rounded corners and a thin gray border.
  const float radius = fontPx(7.0f, displayH);
  r.fillRoundedRect(x, y, w, h, radius, withAlpha(colors::kBlack, alpha));
  r.strokeRoundedRect(x + 0.5f, y + 0.5f, w - 1.0f, h - 1.0f, radius, 1.0f,
                      withAlpha(colors::kGroupBoxBorder, alpha));
  if (title != nullptr && title[0] != '\0') {
    const float tx = x + radius + labelSize * 0.25f;
    const float tw = r.measureTextWidth(title, labelSize);
    const float padX = labelSize * 0.3f;
    // Patch matching the menu-body gray "breaks" the group border behind the
    // "Options" label, the way the WT title's background-color does.
    r.fillRect(tx - padX, y - labelSize * 0.6f, tw + 2.0f * padX,
               labelSize * 1.2f, withAlpha(colors::kPageMenuBodyTop, alpha));
    r.fillText(tx, y, title, labelSize, TextAlign::Left,
               withAlpha(colors::kWhite, alpha));
  }
}

}  // namespace

// Page Menu (MENU key on an open PFD popout, Pilot's Guide Fig. 1-10): a compact
// gray lower-right shell (shorter than the .popout-dialog windows) with an
// "Options" group box showing up to three 24 px rows; longer menus scroll.
void drawPageMenuWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const float rawAnim = ui.pageMenuAnim();
  if (rawAnim <= 0.0f) return;

  float panelW = 0.0f;
  float fullH = 0.0f;
  popoutPanelSize(w, h, panelW, fullH);  // full popout footprint

  const float rowSize = fontPx(20.0f, h);          // option-row text
  const float optionsLabelSize = fontPx(14.0f, h);  // small "Options" caption
  const float rowH = fontPx(26.0f, h);             // row pitch (fits rowSize)
  const int n = ui.pageMenuItemCount();
  const int maxRows =
      std::min(std::max(1, n), kWtPageMenuVisibleRows);  // viewport rows

  // The gray menu rises almost to the Flight Plan window behind it, leaving just
  // its title row (+ separator) peeking above. Lots of vertical padding wraps
  // the "Page Menu" title: titleTopPad above it, then a generous gap below the
  // separator before the "Options" box.
  const float titleTopPad = fontPx(14.0f, h);
  const float panelH = fullH - fontPx(18.0f, h);

  const WindowFrame f = drawWindowFrame(
      r, w, h, L, rawAnim, "Page Menu", panelW, panelH,
      colors::kPageMenuBodyTop, colors::kPageMenuBodyBottom, colors::kWhite,
      titleTopPad);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  // The "Options" box hugs its rows below the title, leaving gray padding above
  // and below (trainer Fig. 1-10): it is NOT stretched to the panel bottom.
  const float pad = f.w * 0.06f;
  const float groupX = f.x + pad;
  const float groupW = f.w - 2.0f * pad;
  const float groupY = f.contentTop + fontPx(18.0f, h);
  const float listTop0 = groupY + rowSize * 0.95f;
  const float groupH =
      (listTop0 - groupY) + static_cast<float>(maxRows) * rowH + rowSize * 0.5f;
  const float groupBottom = groupY + groupH;

  drawOptionsGroup(r, groupX, groupY, groupW, groupH, "Options",
                   optionsLabelSize, h, a);

  const float listTop = listTop0;
  const float listH = groupBottom - listTop - rowSize * 0.4f;

  // Scrollbar lane (only when the menu overflows): the cyan select bar runs up
  // to and touches its left edge rather than passing behind it.
  const bool hasScroll = n > maxRows;
  const float scrollW = wtScrollBarLane(h);
  const float trackX = groupX + groupW - pad * 0.35f - scrollW;
  const float hlLeft = groupX + fontPx(5.0f, h);
  const float hlRight = hasScroll ? trackX : (groupX + groupW - fontPx(5.0f, h));
  const float textX = hlLeft + fontPx(10.0f, h);

  if (n <= 0) {
    r.fillText(groupX + groupW * 0.5f, listTop + listH * 0.5f, "No Options",
               rowSize, TextAlign::Center, withAlpha(colors::kWhite, a));
    return;
  }

  const int scroll = ui.pageMenuScrollOffset();
  const int visible = std::min(n - scroll, maxRows);

  for (int row = 0; row < visible; ++row) {
    const int i = scroll + row;
    const float cy = listTop + rowH * 0.5f + row * rowH;
    const std::string& text = ui.pageMenuItemText(i);
    const bool enabled = ui.pageMenuItemEnabled(i);
    if (i == ui.pageMenuSelected() && enabled) {
      if (ui.blinkOn()) {
        r.fillRect(hlLeft, cy - rowSize * 0.62f, hlRight - hlLeft,
                   rowSize * 1.24f, withAlpha(colors::kPopoutCyan, a));
        r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                   withAlpha(colors::kBlack, a));
      } else {
        r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                   withAlpha(colors::kPopoutCyan, a));
      }
    } else {
      r.fillText(textX, cy, text, rowSize, TextAlign::Left,
                 withAlpha(enabled ? colors::kWhite : colors::kDisabledGray,
                           a));
    }
  }

  if (hasScroll) {
    drawWtScrollBar(r, h, trackX, listTop, listH, n, maxRows, scroll, a);
  }
}

}  // namespace avionics::pfd
