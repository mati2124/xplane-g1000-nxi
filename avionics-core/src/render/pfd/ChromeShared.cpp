#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

WindowFrame drawWindowFrame(Renderer& r, float w, float h, const Layout& L,
                            float rawAnim, const char* title, float panelW,
                            float panelH) {
  WindowFrame f;
  if (rawAnim <= 0.0f) return f;
  const float a = smoothstep(rawAnim);

  const float margin = w * 0.012f;
  const float panelX = w - panelW - margin;
  const float panelBottom = (h - L.bottomBarH) - h * 0.012f;
  // Slide up into place as it fades in.
  const float slide = (1.0f - a) * panelH * 0.22f;
  const float panelTop = panelBottom - panelH + slide;

  // Shared menu chrome (matches the PFD Setup Menu / real unit, Fig. 1-18):
  // opaque black rounded body with a thick light-grey rounded border drawn
  // inset by half its width so the stroke sits fully inside the panel.
  const float radius = panelH * 0.06f;
  const float borderW = 3.0f * (h / 768.0f);
  r.fillRoundedRect(panelX, panelTop, panelW, panelH, radius,
                    withAlpha(colors::kBlack, a));
  r.strokeRoundedRect(panelX + borderW * 0.5f, panelTop + borderW * 0.5f,
                      panelW - borderW, panelH - borderW, radius, borderW,
                      withAlpha(colors::kMenuBorderGray, a));

  // Cyan centered title with a white separator rule beneath it. Sized to match
  // the PFD Setup Menu's title/text (wt::kInfoLabel) so every pop-up shares the
  // same type size.
  const float titleSize = fontPx(wt::kInfoLabel, h);
  const float sepY = panelTop + titleSize * 1.5f;
  r.fillText(panelX + panelW * 0.5f, panelTop + titleSize * 0.85f, title,
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a));
  const float sepInset = borderW + panelW * 0.01f;
  r.strokeLine(panelX + sepInset, sepY, panelX + panelW - sepInset, sepY, 1.5f,
               withAlpha(colors::kWhitesmoke, a));

  f.a = a;
  f.x = panelX;
  f.top = panelTop;
  f.w = panelW;
  f.h = panelH;
  f.contentTop = sepY + titleSize * 0.35f;
  return f;
}

float putField(Renderer& r, float x, float cy, const std::string& text,
               float size, const Color& color, bool highlighted, float alpha,
               bool blinkOn, float trailingGapFrac, FontFace face) {
  const float tw = r.measureTextWidth(text, size, face);
  if (highlighted && blinkOn) {
    const float padX = size * 0.25f;
    const float padY = size * 0.18f;
    r.fillRect(x - padX, cy - size * 0.5f - padY, tw + 2.0f * padX,
               size + 2.0f * padY, withAlpha(colors::kCyan, alpha));
  }
  const Color textColor = highlighted
                              ? (blinkOn ? colors::kBlack : colors::kCyan)
                              : color;
  r.fillText(x, cy, text, size, TextAlign::Left, withAlpha(textColor, alpha),
             face);
  return x + tw + size * trailingGapFrac;
}

}  // namespace avionics::pfd
