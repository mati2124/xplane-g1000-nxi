#include "render/pfd/PfdInternal.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace avionics::pfd {
namespace {

// VSI pointer: a black readout box on the VSI tape with a full-height left
// caret whose tip sits on the altimeter boundary, and rounded outer (right)
// corners matching the IAS/altitude readout boxes. The body carries the
// vertical-speed value, right-aligned (G1000 NXi VS pointer).
void drawVsiPointer(Renderer& r, float x, float vw, float pointerY,
                    float displayH, const std::string& text) {
  const float fontSize = fontPx(wt::kVsi, displayH);
  const float h = fontSize * 1.5f;
  const float top = pointerY - h * 0.5f;
  const float bottom = pointerY + h * 0.5f;
  const float caretDepth = h * 0.42f;
  const float cornerR = h * 0.18f;
  const float tipX = x;
  const float bodyLeft = x + caretDepth;
  // The VS window is a fixed-size readout box: its width does not change with
  // the value and does not shrink at 0 fpm (G1000 NXi / WT constant-width
  // vsi-pointer). The value, when shown, is centered inside it.
  const float w = vw * kVsiWindowWidthFraction - caretDepth;
  const float right = bodyLeft + w;

  // Arrow silhouette (caret tip on the inner edge, body extending right), with
  // only the two outer corners rounded so the pointer shares the readout-box
  // corner style; the caret arms and tip stay sharp.
  const std::vector<Point> poly = {{tipX, pointerY},
                                   {bodyLeft, top},
                                   {right, top},
                                   {right, bottom},
                                   {bodyLeft, bottom}};
  const std::vector<float> radii = {0.0f, 0.0f, cornerR, cornerR, 0.0f};
  std::vector<Point> shape = roundPolygonCorners(poly, radii);
  r.fillPolygon(shape.data(), static_cast<int>(shape.size()),
                colors::kReadoutBox);
  shape.push_back(shape.front());
  r.strokePolyline(shape.data(), static_cast<int>(shape.size()), 2.0f,
                   colors::kWhite);

  if (!text.empty()) {
    r.fillText(bodyLeft + w * 0.5f, pointerY, text, fontSize, TextAlign::Center,
               colors::kWhite);
  }
}

// Translucent backing for the VSI tape: a roughly uniform semi-transparent dark
// overlay (a touch darker at the top/bottom edges) with BOTH outer (right)
// corners rounded -- the VSI's bottom is exposed, unlike the airspeed/altimeter
// tapes whose bottom readout box covers their outer-bottom corner. A notch is
// cut out of the overlay at the 0-fpm line so the SVT reads through it: a
// left-pointing caret (tip on the inner edge under the zero chevron) opening
// into a body that runs all the way out to the outer edge. The compact pointer
// drops into the inner part of this notch at 0 fpm, with the SVT showing through
// to the right of the pointer; off the 0-line the empty zero window shows the
// SVT through it (G1000 NXi VSI).
void drawVsiTapeBackground(Renderer& r, float x, float top, float w, float h,
                           float cy, float winHalf, float caretDepth,
                           float bodyRight, float cornerR) {
  const Color edge = colors::kTapeEdge;
  // Keep the middle translucent (not clear) so the cutout reads against the
  // backing; the opaque pointer covers it at 0 fpm.
  const Color mid{edge.r, edge.g, edge.b, edge.a * 0.72f};
  const float bottom = top + h;
  const float cornerX = x + w - cornerR;  // outer (right) edge corner column
  const float winTop = cy - winHalf;
  const float winBot = cy + winHalf;

  // Above the window: full-width gradient edge@top -> mid@cy, rounded top-right
  // corner (skip the rad x rad corner square, then fill it with a quarter disc).
  r.fillRectVerticalGradient(x, top, w - cornerR, winTop - top, top, cy, edge,
                             mid);
  r.fillRectVerticalGradient(cornerX, top + cornerR, cornerR,
                             winTop - (top + cornerR), top, cy, edge, mid);
  r.save();
  r.clip(cornerX, top, cornerR, cornerR);
  r.fillCircle(cornerX, top + cornerR, cornerR, edge);
  r.restore();

  // Below the window: full-width gradient mid@cy -> edge@bottom, rounded BR.
  r.fillRectVerticalGradient(x, winBot, w - cornerR, bottom - winBot, cy, bottom,
                             mid, edge);
  r.fillRectVerticalGradient(cornerX, winBot, cornerR,
                             (bottom - cornerR) - winBot, cy, bottom, mid, edge);
  r.save();
  r.clip(cornerX, bottom - cornerR, cornerR, cornerR);
  r.fillCircle(cornerX, bottom - cornerR, cornerR, edge);
  r.restore();

  // Window band: leave the pointer-shaped region clear. Each row keeps the dark
  // backing only on the left caret sliver [x, caretBoundary] and to the right of
  // the pointer body [x+bodyRight, outer edge]; the middle is the cutout. The
  // caret boundary runs from the inner edge at the 0-line out to caretDepth at
  // the band edges, tracing the pointer's caret.
  const int strips = std::max(8, std::min(48, static_cast<int>(winHalf)));
  const float dy = winHalf / static_cast<float>(strips);
  const float rightW = (x + w) - (x + bodyRight);  // backing right of the body
  for (int half = 0; half < 2; ++half) {
    const Color a = (half == 0) ? edge : mid;
    const Color b = (half == 0) ? mid : edge;
    const float gradTop = (half == 0) ? top : cy;
    const float gradBot = (half == 0) ? cy : bottom;
    for (int i = 0; i < strips; ++i) {
      const float sy =
          (half == 0) ? (winTop + static_cast<float>(i) * dy)
                      : (cy + static_cast<float>(i) * dy);
      const float u = std::min(1.0f, std::fabs((sy + dy * 0.5f) - cy) / winHalf);
      const float sliver = caretDepth * u;  // left caret sliver width
      if (sliver > 0.3f) {
        r.fillRectVerticalGradient(x, sy, sliver, dy, gradTop, gradBot, a, b);
      }
      if (rightW > 0.3f) {
        r.fillRectVerticalGradient(x + bodyRight, sy, rightW, dy, gradTop,
                                   gradBot, a, b);
      }
    }
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
  // Translucent backing with both outer (right) corners rounded and a
  // pointer-window-shaped cutout at the 0-fpm line. The cutout mirrors the VS
  // pointer's geometry (height = vsiFontSize * 1.5, caret depth, body width)
  // inset by a hair so the opaque pointer drops into the notch and covers it at
  // 0 fpm with no SVT leaking around it (G1000 NXi).
  const float cornerR = fontPx(kTapeCornerRadiusWt, displayH);
  const float vsiFontSize = fontPx(wt::kVsi, displayH);
  const float caretDepth = vsiFontSize * 1.5f * 0.42f;  // == pointer caretDepth
  const float inset = vsiFontSize * 0.1f;
  const float winHalf = vsiFontSize * 0.75f - inset;  // == pointer half-height
  // The cutout runs the full width out to the outer edge of the tape; the
  // fixed-size pointer drops into it at 0 fpm and the SVT shows through the
  // notch once the pointer climbs/descends away, as on the real unit.
  const float bodyRight = vw;
  drawVsiTapeBackground(r, x, stripTop, vw, stripH, cy, winHalf, caretDepth,
                        bodyRight, cornerR);
  const float midBorderY = stripTop + stripH * 0.5f;
  r.strokeLine(x, stripTop, x, midBorderY, 1.5f, colors::kTapeTopBorder);
  r.strokeLine(x, midBorderY, x, stripTop + stripH, 1.5f,
               colors::kTapeBottomBorder);

  const float minorLen = vw * 0.30f;
  const float majorLen = vw * 0.55f;
  const float labelSize = fontPx(wt::kVsi, displayH);
  const int ticks[] = {-4000, -3000, -2000, -1000, 1000, 2000, 3000, 4000};
  for (int f : ticks) {
    const float y = cy - static_cast<float>(f) * pixelsPerFpm;
    const bool major = (f % static_cast<int>(kVsiMajorFpm)) == 0;
    const float len = major ? majorLen : minorLen;
    r.strokeLine(x, y, x + len, y, major ? 2.0f : 1.0f, colors::kWhite);
    if (major) {
      r.fillText(x + len + displayH * 0.004f, y,
                 formatInt(std::abs(f) / 1000.0f), labelSize, TextAlign::Left,
                 colors::kWhite);
    }
  }

  // Zero reference: a small white chevron at the inner edge pointing toward the
  // altimeter (G1000 NXi VSI zero notch), in place of a full-width center line.
  const float zReach = vw * 0.30f;
  const float zHalf = vw * 0.22f;
  const Point zeroMark[3] = {
      {x + zReach, cy - zHalf}, {x, cy}, {x + zReach, cy + zHalf}};
  r.strokePolyline(zeroMark, 3, 2.0f, colors::kWhite);
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
    // VSI sits right of the altimeter; its scale ticks line the inner (left)
    // edge and are retained under the failure X (NXi Fig 9-2).
    drawFailureX(r, L.vsiX, L.vsiTop, L.vsiW, L.vsiH, "", h, FailTicks::LeftEdge);
    return;
  }
  drawVsi(r, L.vsiX, L.vsiW, L.vsiTop, L.vsiH, L.attCy, h, d.verticalSpeedFpm,
          d.requiredVsValid, d.requiredVsFpm, d.selectedVsValid,
          d.selectedVerticalSpeedFpm);
}

}  // namespace avionics::pfd
