#include "render/pfd/PfdInternal.h"

#include <algorithm>
#include <cstdlib>

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

// Magenta chevron marking the Required Vertical Speed to reach a VNV target
// altitude (G1000 NXi Pilot's Guide, VSI). It rides the VSI scale at the
// required rate, pointing inward toward the tape.
void drawRequiredVsChevron(Renderer& r, float x, float vw, float stripTop,
                           float stripH, float cy, float pixelsPerFpm,
                           float reqVsFpm) {
  const float clamped = std::max(-kVsiMaxFpm, std::min(kVsiMaxFpm, reqVsFpm));
  const float y = cy - clamped * pixelsPerFpm;
  if (y < stripTop || y > stripTop + stripH) return;
  const float cw = vw * 0.42f;
  const float ch = vw * 0.34f;
  const float tipX = x + vw * 0.30f;
  const Point chevron[3] = {
      {tipX, y}, {tipX + cw, y - ch}, {tipX + cw, y + ch}};
  r.fillPolygon(chevron, 3, colors::kMagenta);
}

// Cyan Selected Vertical Speed bug riding the VSI scale at the autopilot's
// selected VS reference (G1000 NXi Pilot's Guide, VSI). A notched bug on the
// outer (right) edge of the tape, mirroring the cyan selected-altitude bug.
void drawSelectedVsBug(Renderer& r, float x, float vw, float stripTop,
                       float stripH, float cy, float pixelsPerFpm,
                       float selVsFpm) {
  const float clamped = std::max(-kVsiMaxFpm, std::min(kVsiMaxFpm, selVsFpm));
  const float y = cy - clamped * pixelsPerFpm;
  if (y < stripTop || y > stripTop + stripH) return;
  const float bw = vw * 0.34f;
  const float bh = vw * 0.42f;
  const float bx = x + vw - bw;
  const float notch = bw * 0.5f;
  const Point bug[5] = {{bx + bw, y - bh}, {bx, y - bh}, {bx + notch, y},
                        {bx, y + bh},      {bx + bw, y + bh}};
  r.fillPolygon(bug, 5, colors::kCyan);
}

void drawVsi(Renderer& r, float x, float vw, float stripTop, float stripH,
             float cy, float displayH, float vsFpm, bool reqVsValid,
             float reqVsFpm, bool selVsValid, float selVsFpm) {
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
  if (reqVsValid) {
    drawRequiredVsChevron(r, x, vw, stripTop, stripH, cy, pixelsPerFpm,
                          reqVsFpm);
  }
  if (selVsValid) {
    drawSelectedVsBug(r, x, vw, stripTop, stripH, cy, pixelsPerFpm, selVsFpm);
  }
  r.restore();

  // The pointer slides to the current vertical speed; its body carries the
  // numeric value, quantized to 50 fpm, once the rate exceeds 100 fpm (below
  // that the caret shows with no digits), per the G1000 NXi (Pilot's Guide,
  // VSI; WT quantizes the readout to 50 fpm steps). Drawn after the clip is
  // released so the body can extend right of the narrow tape to fit the digits.
  const long quantizedFpm = std::lround(vsFpm / 50.0f) * 50L;
  const std::string ptrText =
      (std::labs(quantizedFpm) >= 100L)
          ? formatInt(static_cast<float>(quantizedFpm))
          : std::string();
  drawVsiPointer(r, x, vw, pointerY, displayH, ptrText);
}

}  // namespace

void drawVerticalSpeedIndicator(Renderer& r, const Layout& L,
                                const FlightData& d, float h) {
  if (!d.verticalSpeedValid) {
    drawFailureX(r, L.vsiX, L.vsiTop, L.vsiW, L.vsiH, "", h);
    return;
  }
  drawVsi(r, L.vsiX, L.vsiW, L.vsiTop, L.vsiH, L.attCy, h, d.verticalSpeedFpm,
          d.requiredVsValid, d.requiredVsFpm, d.selectedVsValid,
          d.selectedVerticalSpeedFpm);
}

}  // namespace avionics::pfd
