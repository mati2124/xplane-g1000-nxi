#include "render/pfd/PfdInternal.h"

#include <algorithm>

namespace avionics::pfd {
namespace {

// VSI pointer: navy tag on the VSI tape with a small left caret (mirroring the
// airspeed/altitude readout carets). The tip sits on the altimeter boundary so
// the body and digits never cover the altitude black pointer (G1000 NXi).
void drawVsiPointer(Renderer& r, float x, float vw, float pointerY,
                    float displayH, const std::string& text) {
  const float fontSize = fontPx(wt::kVsi, displayH);
  const float h = fontSize * 1.5f;
  const float w = vw * 1.5f;
  const float top = pointerY - h * 0.5f;
  const float bottom = pointerY + h * 0.5f;
  const float caretDepth = h * 0.14f;
  const float caretHalfH = h * 0.20f;
  const float tipX = x;
  const float bodyLeft = x + caretDepth;
  const float right = bodyLeft + w;

  const Point body[5] = {{bodyLeft, top},
                         {right, top},
                         {right, bottom},
                         {bodyLeft, bottom},
                         {tipX, pointerY}};
  r.fillPolygon(body, 5, colors::kVsiBox);
  const Point outline[8] = {{bodyLeft, top},
                            {right, top},
                            {right, bottom},
                            {bodyLeft, bottom},
                            {bodyLeft, pointerY + caretHalfH},
                            {tipX, pointerY},
                            {bodyLeft, pointerY - caretHalfH},
                            {bodyLeft, top}};
  r.strokePolyline(outline, 8, 2.0f, colors::kWhite);

  if (!text.empty()) {
    r.fillText(bodyLeft + w * 0.5f, pointerY, text, fontSize, TextAlign::Center,
               colors::kWhite);
  }
}

void drawVsi(Renderer& r, float x, float vw, float stripTop, float stripH,
             float cy, float displayH, float vsFpm) {
  const float pixelsPerFpm = (stripH * kVsiScaleHalfFraction) / kVsiMaxFpm;
  const float clamped = std::max(-kVsiMaxFpm, std::min(kVsiMaxFpm, vsFpm));
  const float pointerY = cy - clamped * pixelsPerFpm;

  r.save();
  r.clip(x, stripTop, vw, stripH);
  // Same translucent backing as the airspeed/altitude tapes: dark at the top and
  // bottom edges and clear through the middle, so the attitude sky/ground shows
  // through the VSI exactly as it does behind the altitude tape (G1000 NXi).
  drawTapeBackground(r, x, stripTop, vw, stripH, colors::kTapeEdge);
  const float midBorderY = stripTop + stripH * 0.5f;
  r.strokeLine(x, stripTop, x, midBorderY, 1.5f, colors::kTapeTopBorder);
  r.strokeLine(x, midBorderY, x, stripTop + stripH, 1.5f,
               colors::kTapeBottomBorder);

  const float minorLen = vw * 0.30f;
  const float majorLen = vw * 0.55f;
  const float labelSize = fontPx(wt::kVsi, displayH);
  const int ticks[] = {-2000, -1500, -1000, -500, 500, 1000, 1500, 2000};
  for (int f : ticks) {
    const float y = cy - static_cast<float>(f) * pixelsPerFpm;
    const bool major = (f % 1000) == 0;
    const float len = major ? majorLen : minorLen;
    r.strokeLine(x, y, x + len, y, major ? 2.0f : 1.0f, colors::kWhite);
    if (major) {
      r.fillText(x + len + displayH * 0.004f, y,
                 formatInt(std::abs(f) / 1000.0f), labelSize, TextAlign::Left,
                 colors::kWhite);
    }
  }

  r.strokeLine(x, cy, x + vw, cy, 1.5f, colors::kWhite);
  r.restore();

  // The pointer slides to the current vertical speed; its body carries the
  // numeric value when the rate exceeds 100 fpm (below that the caret shows with
  // no digits), per the G1000 NXi. Drawn after the clip is released so the body
  // can extend right of the narrow tape to fit the digits.
  const std::string ptrText =
      (std::abs(vsFpm) >= 100.0f) ? formatInt(vsFpm) : std::string();
  drawVsiPointer(r, x, vw, pointerY, displayH, ptrText);
}

}  // namespace

void drawVerticalSpeedIndicator(Renderer& r, const Layout& L,
                                const FlightData& d, float h) {
  if (!d.verticalSpeedValid) {
    drawFailureX(r, L.vsiX, L.vsiTop, L.vsiW, L.vsiH, "", h);
    return;
  }
  drawVsi(r, L.vsiX, L.vsiW, L.vsiTop, L.vsiH, L.attCy, h,
          d.verticalSpeedFpm);
}

}  // namespace avionics::pfd
