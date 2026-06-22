#include <algorithm>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

float drawDirectToIcon(Renderer& r, float x, float cy, float size,
                       const Color& color) {
  // The font has no bold weight, so the "D" is over-drawn at a small plus-
  // shaped halo of offsets to fatten its stroke toward the Garmin glyph.
  const float bold = std::max(1.0f, size * 0.03f);
  const float off[5][2] = {{0, 0}, {-bold, 0}, {bold, 0}, {0, -bold}, {0, bold}};
  for (const auto& o : off) {
    r.fillText(x + o[0], cy + o[1], "D", size, TextAlign::Left, color);
  }
  // Capital text is middle-aligned, so the visual center of the "D" sits a touch
  // above cy; pierce the arrow through there so it reads as centered on the D.
  const float ay = cy - size * 0.035f;
  const float dW = r.measureTextWidth("D", size);
  const float shaftL = x + dW * 0.42f;          // pierce through the D's bowl
  const float shaftR = x + dW + size * 0.26f;   // exit to the right of the D
  const float head = size * 0.34f;              // chunky arrowhead (the "carrot")
  r.strokeLine(shaftL, ay, shaftR, ay, std::max(2.5f, size * 0.13f), color);
  const Point tri[3] = {{shaftR + head * 0.55f, ay},
                        {shaftR - head * 0.30f, ay - head},
                        {shaftR - head * 0.30f, ay + head}};
  r.fillPolygon(tri, 3, color);
  return shaftR + head * 0.7f;
}

float drawNavDirectToHeader(Renderer& r, float x, float cy,
                            const std::string& ident, float size,
                            const Color& color) {
  float hx = drawDirectToIcon(r, x, cy, size, color);
  hx += size * 0.18f;
  if (!ident.empty()) {
    r.fillText(hx, cy, ident, size, TextAlign::Left, color);
  }
  return hx;
}

// Direct-To target row under Enroute (D→ glyph + ident), shared by PFD/MFD FPL.
float drawFplDirectToTargetRow(Renderer& r, float x, float cy,
                               const std::string& ident, float size,
                               const Color& textColor) {
  float hx = drawDirectToIcon(r, x, cy, size, colors::kMagenta);
  hx += size * 0.10f;
  if (!ident.empty()) {
    r.fillText(hx, cy, ident, size, TextAlign::Left, textColor);
  }
  return hx;
}

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

void drawWtScrollBar(Renderer& r, float displayH, float trackX, float trackTop,
                     float trackH, int total, int visible, int first, float a) {
  if (total <= visible || trackH <= 0.0f) return;

  const float laneW = wtScrollBarLane(displayH);
  const float trackW = std::max(1.0f, fontPx(kWtScrollBarTrackPx, displayH));
  const float thumbW =
      std::max(trackW + 1.0f, fontPx(kWtScrollBarThumbPx, displayH));
  const float carrotHalfW = fontPx(kWtScrollBarCarrotHalfWPx, displayH);
  const float carrotHalfH = fontPx(kWtScrollBarCarrotHalfHPx, displayH);
  const float trackCenterX = trackX + laneW * 0.5f;
  const float trackBottom = trackTop + trackH;

  const Color carrotColor = withAlpha(colors::kWhite, a);
  const Color trackColor = withAlpha(colors::kWhite, a);
  const Color thumbColor = withAlpha(colors::kMenuBorderGray, a);

  const Point upCarrot[3] = {{trackCenterX, trackTop},
                             {trackCenterX - carrotHalfW, trackTop + carrotHalfH},
                             {trackCenterX + carrotHalfW, trackTop + carrotHalfH}};
  r.fillPolygon(upCarrot, 3, carrotColor);

  const Point downCarrot[3] = {
      {trackCenterX, trackBottom},
      {trackCenterX - carrotHalfW, trackBottom - carrotHalfH},
      {trackCenterX + carrotHalfW, trackBottom - carrotHalfH}};
  r.fillPolygon(downCarrot, 3, carrotColor);

  const float lineTop = trackTop + carrotHalfH;
  const float lineBottom = trackBottom - carrotHalfH;
  const float lineH = lineBottom - lineTop;
  if (lineH > 0.0f) {
    r.fillRect(trackCenterX - trackW * 0.5f, lineTop, trackW, lineH, trackColor);
  }

  const float thumbH =
      std::max(fontPx(kWtScrollBarMinThumbPx, displayH),
               lineH * static_cast<float>(visible) / static_cast<float>(total));
  const float maxScroll = static_cast<float>(total - visible);
  const float thumbTop =
      lineTop + (lineH - thumbH) * static_cast<float>(first) / maxScroll;
  r.fillRect(trackCenterX - thumbW * 0.5f, thumbTop, thumbW, thumbH, thumbColor);
}

}  // namespace avionics::pfd
