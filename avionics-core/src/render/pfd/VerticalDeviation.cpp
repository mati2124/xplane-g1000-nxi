#include "render/pfd/PfdInternal.h"

#include <algorithm>

namespace avionics::pfd {
namespace {

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

}  // namespace

void drawVerticalDeviation(Renderer& r, const Layout& L, const FlightData& d,
                           float h) {
  drawMarkerBeacon(r, L, d, h);

  if (d.vdiKind == VerticalDeviationKind::None) return;

  const float cx = L.vdiX + L.vdiW * 0.5f;
  const float cy = L.attCy;
  // The scale spans ~70% of the strip half-height; full deflection (+/-2 dots)
  // reaches the top/bottom of that span.
  const float halfScaleH = L.stripH * 0.5f * 0.72f;
  const float dotR = std::max(2.0f, L.vdiW * 0.16f);

  // Source annunciation at the top of the scale: green 'G' for an ILS
  // Glideslope, magenta 'G' for a GPS Glidepath, magenta 'V' for VNAV (G1000
  // NXi Pilot's Guide, Vertical Deviation; WT NXi VerticalDeviation source).
  const bool isVnav = d.vdiKind == VerticalDeviationKind::Vnav;
  const Color markerColor = (d.vdiKind == VerticalDeviationKind::Glideslope)
                                ? colors::kActiveGreen
                                : colors::kMagenta;
  const float srcSize = fontPx(wt::kVsi, h);
  r.fillText(cx, cy - halfScaleH - srcSize * 0.9f, isVnav ? "V" : "G", srcSize,
             TextAlign::Center, markerColor);

  // Reference dots at +/-1 and +/-2 dots and a center tick.
  for (int k = -2; k <= 2; ++k) {
    if (k == 0) continue;
    const float y = cy + static_cast<float>(k) * (halfScaleH * 0.5f);
    r.fillCircle(cx, y, dotR, colors::kWhite);
  }
  r.strokeLine(cx - L.vdiW * 0.5f, cy, cx + L.vdiW * 0.5f, cy, 2.0f,
               colors::kWhite);

  // Signal lost: "NO GS" replaces the glideslope diamond when an ILS localizer
  // is tuned with no glideslope; "NO GP" replaces the glidepath diamond when
  // the approach downgrades (G1000 NXi Pilot's Guide).
  if (!d.vdiValid) {
    if (isVnav) return;  // the VDI is simply removed when VNAV dev is invalid
    const char* what =
        (d.vdiKind == VerticalDeviationKind::Glideslope) ? "GS" : "GP";
    const float ts = fontPx(wt::kVsi, h) * 0.8f;
    r.fillText(cx, cy - ts * 0.55f, "NO", ts, TextAlign::Center,
               colors::kBandYellow);
    r.fillText(cx, cy + ts * 0.55f, what, ts, TextAlign::Center,
               colors::kBandYellow);
    return;
  }

  // Positive deviation = above path, so the marker rides low (fly down to
  // recapture).
  const float dots = std::max(-2.0f, std::min(2.0f, d.vdiDeviationDots));
  const float y = cy + (dots * 0.5f) * halfScaleH;
  if (isVnav) {
    // VNAV deviation marker: an open magenta chevron pointing left (WT NXi
    // vertdev-vnav-deviation caret).
    const float ch = L.vdiW * 0.55f;
    const float cw = L.vdiW * 0.95f;
    const float lineW = std::max(2.0f, L.vdiW * 0.22f);
    const Point chevron[3] = {{cx + cw * 0.5f, y - ch},
                              {cx - cw * 0.5f, y},
                              {cx + cw * 0.5f, y + ch}};
    r.strokePolyline(chevron, 3, lineW, colors::kMagenta);
  } else {
    const float dh = L.vdiW * 0.55f;
    const float dw = L.vdiW * 0.45f;
    const Point diamond[4] = {
        {cx, y - dh}, {cx + dw, y}, {cx, y + dh}, {cx - dw, y}};
    r.fillPolygon(diamond, 4, markerColor);
  }
}

}  // namespace avionics::pfd
