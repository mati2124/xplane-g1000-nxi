#include "render/pfd/PfdInternal.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace avionics::pfd {
namespace {

struct VsiTickMark {
  float svgY;
  bool major;
};

struct VsiTapeLabel {
  float svgY;
  const char* text;
};

// Fixed scale from WT VerticalSpeedIndicator.tsx inline svg (305 px tall).
constexpr VsiTickMark kVsiTickMarks[] = {
    {20.0f, true},  {52.0f, false}, {84.0f, true},  {116.0f, false},
    {180.0f, false}, {212.0f, true}, {244.0f, false}, {276.0f, true},
};
constexpr VsiTapeLabel kVsiTapeLabels[] = {
    {27.0f, "2"}, {91.0f, "1"}, {219.0f, "1"}, {283.0f, "2"},
};

float mapVsiSvgY(float stripTop, float stripH, float svgY) {
  return stripTop + svgY * stripH / kVsiTapeSvgHeightPx;
}

// WT vsi-pointer-bug-background path in viewBox 68 x 24 (tip at x=1, y=12).
std::vector<Point> buildVsiPointerSilhouette(float tipX, float midY, float scale) {
  const float sx = (kVsiPointerWidthPx * scale) / 68.0f;
  const float sy = (kVsiPointerHeightPx * scale) / 24.0f;
  auto map = [&](float px, float py) {
    return Point{tipX + (px - 1.0f) * sx, midY + (py - 12.0f) * sy};
  };
  return {map(1.0f, 12.0f),  map(20.0f, 1.0f),  map(64.0f, 1.0f),
          map(67.0f, 4.0f),  map(67.0f, 20.0f), map(64.0f, 23.0f),
          map(20.0f, 23.0f)};
}

// G1000 NXi VS pointer: fixed 66 x 22 px black readout window (WT
// vsi-pointer-bug-background fill only — no stroke on the real unit).
void drawVsiPointer(Renderer& r, float tipX, float pointerY, float displayH,
                    const std::string& text) {
  const float scale = displayH / kWtCanvasHeightPx;
  const float fontSize = fontPx(wt::kVsi, displayH);

  std::vector<Point> poly = buildVsiPointerSilhouette(tipX, pointerY, scale);
  r.fillPolygon(poly.data(), static_cast<int>(poly.size()), colors::kReadoutBox);

  if (!text.empty()) {
    const float sx = (kVsiPointerWidthPx * scale) / 68.0f;
    const float bodyCx = tipX + 41.0f * sx;
    r.fillText(bodyCx, pointerY + kVsiReadoutInkDownPx * scale, text, fontSize,
               TextAlign::Center, colors::kWhite);
  }
}

// Magenta chevron marking the Required Vertical Speed to reach a VNV target
// altitude (G1000 NXi Pilot's Guide, VSI).
void drawRequiredVsChevron(Renderer& r, float x, float vw, float stripTop,
                           float stripH, float cy, float pixelsPerFpm,
                           float reqVsFpm) {
  const float clamped =
      std::max(-kVsiPointerClampFpm, std::min(kVsiPointerClampFpm, reqVsFpm));
  const float y = cy - clamped * pixelsPerFpm;
  if (y < stripTop || y > stripTop + stripH) {
    return;
  }
  const float cw = vw * 0.42f;
  const float ch = vw * 0.34f;
  const float tipX = x + vw * 0.30f;
  const Point chevron[3] = {
      {tipX, y}, {tipX + cw, y - ch}, {tipX + cw, y + ch}};
  r.fillPolygon(chevron, 3, colors::kMagenta);
}

// Cyan Selected Vertical Speed bug riding the VSI scale at the autopilot's
// selected VS reference (G1000 NXi Pilot's Guide, VSI).
void drawSelectedVsBug(Renderer& r, float x, float vw, float stripTop,
                       float stripH, float cy, float pixelsPerFpm,
                       float selVsFpm) {
  const float clamped =
      std::max(-kVsiPointerClampFpm, std::min(kVsiPointerClampFpm, selVsFpm));
  const float y = cy - clamped * pixelsPerFpm;
  if (y < stripTop || y > stripTop + stripH) {
    return;
  }
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
  const float scale = displayH / kWtCanvasHeightPx;
  const float pixelsPerFpm = kVsiPointerPxPerFpm * scale;
  const float clamped =
      std::max(-kVsiPointerClampFpm, std::min(kVsiPointerClampFpm, vsFpm));
  const float pointerY = cy - clamped * pixelsPerFpm;
  const float tickStroke = kTapeTickStrokePx * scale;
  const float cornerR = fontPx(kTapeCornerRadiusWt, displayH);

  r.save();
  r.clip(x, stripTop, vw, stripH);
  drawTapeBackground(r, x, stripTop, vw, stripH, colors::kTapeEdge,
                     /*tapeOnRight=*/true, cornerR);

  const float borderX = x;
  const float midY = stripTop + stripH * 0.5f;
  r.strokeLine(borderX, stripTop, borderX, midY, 1.5f, colors::kTapeTopBorder);
  r.strokeLine(borderX, midY, borderX, stripTop + stripH, 1.5f,
               colors::kTapeBottomBorder);

  const float tickInner = x + vw * kVsiTickInnerInsetFraction;
  const float tickMinorOuter = x + vw * kVsiTickMinorOuterFraction;
  const float tickMajorOuter = x + vw * kVsiTickMajorOuterFraction;
  const float labelSize = fontPx(kVsiLabelSizeWt, displayH);
  const float labelX = x + vw * kVsiLabelAnchorFraction;

  for (const VsiTickMark& mark : kVsiTickMarks) {
    const float y = mapVsiSvgY(stripTop, stripH, mark.svgY);
    const float outer = mark.major ? tickMajorOuter : tickMinorOuter;
    r.strokeLine(tickInner, y, outer, y, tickStroke, colors::kWhite);
  }
  for (const VsiTapeLabel& label : kVsiTapeLabels) {
    const float y = mapVsiSvgY(stripTop, stripH, label.svgY);
    r.fillText(labelX, y, label.text, labelSize, TextAlign::Left,
               colors::kWhite);
  }

  // Zero reference chevron at the inner edge (WT path at y=137.691 in svg).
  const float zReach = vw * (15.0f / 48.0f);
  const float zHalf = vw * (10.309f / 48.0f);
  const Point zeroMark[3] = {
      {x + zReach, cy - zHalf}, {x, cy}, {x + zReach, cy + zHalf}};
  r.strokePolyline(zeroMark, 3, tickStroke, colors::kWhite);

  if (reqVsValid) {
    drawRequiredVsChevron(r, x, vw, stripTop, stripH, cy, pixelsPerFpm,
                          reqVsFpm);
  }
  if (selVsValid) {
    drawSelectedVsBug(r, x, vw, stripTop, stripH, cy, pixelsPerFpm, selVsFpm);
  }
  r.restore();

  const long quantizedFpm = std::lround(vsFpm / 50.0f) * 50L;
  const std::string ptrText =
      std::fabs(static_cast<float>(quantizedFpm)) >= kVsiReadoutShowFpm
          ? formatInt(static_cast<float>(quantizedFpm))
          : std::string();
  drawVsiPointer(r, x, pointerY, displayH, ptrText);
}

}  // namespace

void drawVerticalSpeedIndicator(Renderer& r, const Layout& L,
                                const FlightData& d, float h) {
  if (!d.verticalSpeedValid) {
    drawFailureX(r, L.vsiX, L.vsiTop, L.vsiW, L.vsiH, "", h, FailTicks::LeftEdge,
                 kVsiTickMinorOuterFraction, kVsiTickMajorOuterFraction);
    return;
  }
  drawVsi(r, L.vsiX, L.vsiW, L.vsiTop, L.vsiH, L.attCy, h, d.verticalSpeedFpm,
          d.requiredVsValid, d.requiredVsFpm, d.selectedVsValid,
          d.selectedVerticalSpeedFpm);
}

}  // namespace avionics::pfd
