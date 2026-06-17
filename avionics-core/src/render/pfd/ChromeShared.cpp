#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

WindowFrame drawWindowFrame(Renderer& r, float w, float h, const Layout& L,
                            float rawAnim, const char* title, float panelW,
                            float panelH) {
  WindowFrame f;
  if (rawAnim <= 0.0f) return f;
  const float a = smoothstep(rawAnim);

  const float margin = fontPx(kWtPopoutRightMarginPx, h);
  const float panelX = w - panelW - margin;
  const float panelBottom =
      (h - L.bottomBarH) - fontPx(kWtPopoutBottomMarginPx, h);
  // Slide up into place as it fades in.
  const float slide = (1.0f - a) * panelH * 0.22f;
  const float panelTop = panelBottom - panelH + slide;

  // Shared popout chrome (WT .popout-dialog): translucent-black vertical
  // gradient body, 10 px rounded corners, 3 px rgb(150,150,150) border.
  const float radius = fontPx(kWtPopoutBorderRadiusPx, h);
  const float borderW = fontPx(kWtPopoutBorderPx, h);
  r.fillRoundedRectVerticalGradient(
      panelX, panelTop, panelW, panelH, radius, panelTop, panelTop + panelH,
      withAlpha(colors::kPopoutBodyTop, a),
      withAlpha(colors::kPopoutBodyBottom, a));
  r.strokeRoundedRect(panelX + borderW * 0.5f, panelTop + borderW * 0.5f,
                      panelW - borderW, panelH - borderW, radius, borderW,
                      withAlpha(colors::kPopoutBorder, a));

  // Cyan centered title (WT h1, 16 px Roboto) with a 1 px grey rule beneath.
  const float titleSize = fontPx(wt::kInfoLabel, h);
  const float titleCy = panelTop + borderW + titleSize * 0.72f;
  const float sepY = titleCy + titleSize * 0.62f;
  r.fillText(panelX + panelW * 0.5f, titleCy, title, titleSize,
             TextAlign::Center, withAlpha(colors::kPopoutCyan, a));
  const float sepInset = borderW + panelW * 0.01f;
  r.strokeLine(panelX + sepInset, sepY, panelX + panelW - sepInset, sepY, 1.0f,
               withAlpha(colors::kPopoutBorder, a));

  f.a = a;
  f.x = panelX;
  f.top = panelTop;
  f.w = panelW;
  f.h = panelH;
  f.contentTop = sepY + titleSize * 0.45f;
  return f;
}

float putField(Renderer& r, float x, float cy, const std::string& text,
               float size, const Color& color, bool highlighted, float alpha,
               float trailingGapFrac, FontFace face) {
  const float tw = r.measureTextWidth(text, size, face);
  if (highlighted) {
    const float padX = size * 0.25f;
    const float padY = size * 0.18f;
    r.fillRect(x - padX, cy - size * 0.5f - padY, tw + 2.0f * padX,
               size + 2.0f * padY, withAlpha(colors::kPopoutCyan, alpha));
  }
  const Color textColor = highlighted ? colors::kBlack : color;
  r.fillText(x, cy, text, size, TextAlign::Left, withAlpha(textColor, alpha),
             face);
  return x + tw + size * trailingGapFrac;
}

}  // namespace avionics::pfd
