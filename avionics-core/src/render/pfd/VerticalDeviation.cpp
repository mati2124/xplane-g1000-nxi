#include "render/pfd/PfdInternal.h"

#include <algorithm>

namespace avionics::pfd {
namespace {

// WT NXi SVG uses 8x10 half-axes at 24 px; bump slightly for legibility.
constexpr float kGsGpDiamondScale = 1.3f;

// Marker beacon annunciation just left of the altimeter: outer (cyan "O"),
// middle (amber "M"), inner (white "I"), each a filled capsule with a black
// letter (G1000 NXi Pilot's Guide, Marker Beacon Annunciations).
void drawMarkerBeacon(Renderer& r, const Layout& L, const FlightData& d,
                      float displayH) {
  if (d.markerBeacon == MarkerBeacon::None) return;

  const char* letter = "O";
  Color fill = colors::kCyan;
  switch (d.markerBeacon) {
    case MarkerBeacon::Outer:  letter = "O"; fill = colors::kCyan;       break;
    case MarkerBeacon::Middle: letter = "M"; fill = colors::kBandYellow; break;
    case MarkerBeacon::Inner:  letter = "I"; fill = colors::kWhite;      break;
    case MarkerBeacon::None:   return;
  }

  r.fillRect(L.markerX, L.markerY, L.markerW, L.markerH, fill);
  const Point border[5] = {{L.markerX, L.markerY},
                           {L.markerX + L.markerW, L.markerY},
                           {L.markerX + L.markerW, L.markerY + L.markerH},
                           {L.markerX, L.markerY + L.markerH},
                           {L.markerX, L.markerY}};
  r.strokePolyline(border, 5, 1.5f, colors::kBlack);
  r.fillText(L.markerX + L.markerW * 0.5f, L.markerY + L.markerH * 0.5f, letter,
             fontPx(wt::kHeadingBox, displayH), TextAlign::Center,
             colors::kBlack);
}

// Hollow white reference ring: the WT NXi VDI reference dots are stroked,
// unfilled circles (stroke white, fill none), not filled dots.
void strokeRing(Renderer& r, float cx, float cy, float radius, float lineW,
                const Color& c) {
  constexpr int kSeg = 18;
  Point pts[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a = 2.0f * static_cast<float>(M_PI) * (i / float(kSeg));
    pts[i] = {cx + std::cos(a) * radius, cy + std::sin(a) * radius};
  }
  r.strokePolyline(pts, kSeg + 1, lineW, c);
}

}  // namespace

void drawVerticalDeviation(Renderer& r, const Layout& L, const FlightData& d,
                           float h) {
  drawMarkerBeacon(r, L, d, h);

  if (d.vdiKind == VerticalDeviationKind::None) return;

  // WT NXi VerticalDeviation geometry (authored on the 1024x768 canvas): a
  // 204 px scale region inside a 234 px box, centered on the aircraft
  // reference, with the source-label box in the 24 px directly above it. The
  // reference rings sit at +/-1 dot (40 px) and +/-2 dots (80 px) from center;
  // a full-deflection marker reaches +/-100 px (the scale edge).
  const float pxPerUnit = L.stripH * (1.0f / 330.0f);  // canvas px -> display px
  const float panelH = 204.0f * pxPerUnit;
  const float panelTop = L.attCy - panelH * 0.5f;
  const float cx = L.vdiX + L.vdiW * 0.5f;
  const float cy = L.attCy;
  const float dot1 = 40.0f * pxPerUnit;
  const float dot2 = 80.0f * pxPerUnit;
  const float scale = L.vdiW / 24.0f;  // WT marker paths are authored at 24 px

  // The deviation guidance has its own moving-tape panel (translucent vertical
  // gradient) centered on the aircraft reference, behind the reference dots and
  // pointer. The source-label box sits above it, separated by a transparent gap
  // so the two read as distinct elements (real NXi).
  drawTapeBackground(r, L.vdiX, panelTop, L.vdiW, panelH, colors::kTapeEdge);

  // Source annunciation in its own box just above the scale: magenta 'V' for a
  // VNAV vertical path, magenta 'G' for a GPS glidepath, green 'G' for an ILS
  // glideslope (WT NXi VerticalDeviation setSource).
  const bool isVnav = d.vdiKind == VerticalDeviationKind::Vnav;
  const Color srcColor = (d.vdiKind == VerticalDeviationKind::Glideslope)
                             ? colors::kActiveGreen
                             : colors::kMagenta;
  const float srcBoxH = 24.0f * pxPerUnit;
  // Center the source-label box vertically in the clear gap between the marker-
  // beacon box above and the top edge of the guidance panel below, so it sits
  // entirely between the two rather than overlapping the panel (real NXi).
  const float srcBoxY =
      (L.markerY + L.markerH + panelTop) * 0.5f - srcBoxH * 0.5f;
  // The source label sits in its own box above the scale (real NXi). It carries
  // the same translucent moving-tape styling as the altitude tape window -- a
  // vertical gradient (clear through the middle, dark at the edges), framed by
  // the tape's gray border -- so it reads as a discrete box rather than a solid
  // black panel.
  drawTapeBackground(r, L.vdiX, srcBoxY, L.vdiW, srcBoxH, colors::kTapeEdge);
  const Point srcOutline[5] = {{L.vdiX, srcBoxY},
                               {L.vdiX + L.vdiW, srcBoxY},
                               {L.vdiX + L.vdiW, srcBoxY + srcBoxH},
                               {L.vdiX, srcBoxY + srcBoxH},
                               {L.vdiX, srcBoxY}};
  r.strokePolyline(srcOutline, 5, 1.0f, colors::kTapeTopBorder);
  // The source letter is bold (heavier SemiBold face) and centered both ways in
  // the box. NVG_ALIGN_MIDDLE centers on the font's ascender/descender line, so
  // an all-caps glyph (no descender ink) rides above the true center; nudge it
  // down by ~0.10 em -- the gap between the line center and the cap-ink center
  // -- to sit centered in the box.
  const std::string srcLabel = isVnav ? "V" : "G";
  const float srcSize = fontPx(wt::kVsi, h);
  const float srcBoxMidY = srcBoxY + srcBoxH * 0.5f + srcSize * kCapInkCenterNudge;
  {
    FontScope srcFont(r, FontFace::DejaVuSemiBold);
    r.fillText(cx, srcBoxMidY, srcLabel, srcSize, TextAlign::Center, srcColor);
  }

  // Signal lost: "NO GS" replaces the glideslope diamond when an ILS localizer
  // is tuned with no glideslope; "NO GP" when the approach downgrades. A VNAV
  // path is simply removed when its deviation is invalid (G1000 NXi).
  if (!d.vdiValid) {
    if (isVnav) return;
    const char* what =
        (d.vdiKind == VerticalDeviationKind::Glideslope) ? "GS" : "GP";
    const float ts = fontPx(wt::kVsi, h) * 0.8f;
    r.fillText(cx, cy - ts * 0.55f, "NO", ts, TextAlign::Center,
               colors::kBandYellow);
    r.fillText(cx, cy + ts * 0.55f, what, ts, TextAlign::Center,
               colors::kBandYellow);
    return;
  }

  r.save();
  r.clip(L.vdiX, panelTop, L.vdiW, panelH);

  // White center line and four hollow white reference rings (+/-1, +/-2 dots).
  const float ringR = std::max(2.0f, 3.0f * pxPerUnit);
  const float ringW = std::max(1.0f, 1.0f * pxPerUnit);
  r.strokeLine(L.vdiX, cy, L.vdiX + L.vdiW, cy, 1.5f, colors::kWhite);
  strokeRing(r, cx, cy - dot1, ringR, ringW, colors::kWhite);
  strokeRing(r, cx, cy + dot1, ringR, ringW, colors::kWhite);
  strokeRing(r, cx, cy - dot2, ringR, ringW, colors::kWhite);
  strokeRing(r, cx, cy + dot2, ringR, ringW, colors::kWhite);

  // Positive deviation = above path, so the marker rides low (fly down to
  // recapture); +/-2 dots lands the marker on the +/-2-dot ring.
  const float dots = std::max(-2.5f, std::min(2.5f, d.vdiDeviationDots));
  const float y = cy + (dots * 0.5f) * dot2;
  if (isVnav) {
    // VNAV path caret: a magenta arrow pointing left toward the scale (WT
    // vertdev-vnav-deviation). Derived from the WT path but with thicker arms
    // (notch base pulled toward center, notch tip shallower) so the chevron
    // reads as boldly as the real NXi pointer.
    const auto P = [&](float nx, float ny) {
      return Point{L.vdiX + nx * scale, y + (ny - 10.0f) * scale};
    };
    const Point caret[6] = {P(20, 0),   P(1, 10),  P(20, 20),
                            P(20, 13), P(11, 10), P(20, 7)};
    r.fillPolygon(caret, 6, colors::kMagenta);
    const Point outline[7] = {P(20, 0),   P(1, 10),  P(20, 20), P(20, 13),
                              P(11, 10), P(20, 7),  P(20, 0)};
    r.strokePolyline(outline, 7, std::max(1.0f, scale), colors::kBlack);
  } else {
    // Glideslope/glidepath diamond (WT "M 12 0 l -8 10 l 8 10 l 8 -10 z"):
    // magenta for a GPS glidepath, green for an ILS glideslope.
    const Color fill = (d.vdiKind == VerticalDeviationKind::Glideslope)
                           ? colors::kActiveGreen
                           : colors::kMagenta;
    const float dw = 8.0f * scale * kGsGpDiamondScale;
    const float dh = 10.0f * scale * kGsGpDiamondScale;
    const Point diamond[4] = {
        {cx, y - dh}, {cx + dw, y}, {cx, y + dh}, {cx - dw, y}};
    r.fillPolygon(diamond, 4, fill);
    const Point outline[5] = {{cx, y - dh}, {cx + dw, y}, {cx, y + dh},
                              {cx - dw, y}, {cx, y - dh}};
    r.strokePolyline(outline, 5, std::max(1.0f, scale), colors::kBlack);
  }
  r.restore();
}

}  // namespace avionics::pfd
