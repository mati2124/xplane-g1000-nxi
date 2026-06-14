#include <algorithm>
#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {

// Nearest Airports window (Nearest softkey): a distance-sorted, scrollable
// list of nearby airports. Each entry is two plain-text rows with no field
// boxes, exactly like the real unit / Working Title NXi (Pilot's Guide
// Fig. 5-28; the boxes in that figure are the manual's callout annotations).
// Row 1: ident, airport symbol, bearing, distance, best approach.
// Row 2: COM service label, tunable frequency (cyan), longest runway.
void drawNearestWindow(Renderer& r, float w, float h, const Layout& L,
                       const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // WT popout: 310x220 px on the 1024x768 canvas; each entry is 58 px tall
  // with two 29 px rows.
  const float panelW = w * (310.0f / kWtCanvasWidth);
  const float panelH = h * (220.0f / kWtCanvasHeightPx);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Nearest),
                      "Nearest Airports", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const auto& list = ui.nearestAirports();
  const float size = fontPx(wt::kInfoLabel, h);
  const float smallSize = size * 0.875f;
  if (list.empty()) {
    const char* msg = "None within 200";
    const float msgW = r.measureTextWidth(msg, size);
    const float unitW = r.measureTextWidth("NM", smallSize);
    const float cx = f.x + f.w * 0.5f;
    const float cy = f.top + f.h * 0.55f;
    r.fillText(cx - (msgW + unitW) * 0.5f, cy, msg, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(cx - (msgW + unitW) * 0.5f + msgW, cy, "NM", smallSize,
               TextAlign::Left, withAlpha(colors::kWhite, a));
    return;
  }

  constexpr int kVisibleEntries = 3;
  const int cursor = ui.nearestCursor();
  const bool blinkOn = ui.blinkOn();
  const int first = std::max(0, cursor - kVisibleEntries + 1);
  const float entryH = fontPx(58.0f, h);
  const float rowH = entryH * 0.5f;
  const float padX = fontPx(5.0f, h);
  const float scrollW = fontPx(10.0f, h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX - scrollW - fontPx(4.0f, h);
  const float listW = listRight - listLeft;

  // Row 1 grid (WT: 1fr 29px 1fr 1fr 1fr): ident | icon | bearing | distance |
  // approach. The 29 px icon column is fixed; the four 1fr columns split the
  // rest evenly. Bearing/distance/approach are centered in their columns.
  const float iconW = fontPx(29.0f, h);
  const float colW = (listW - iconW) / 4.0f;
  const float identX = listLeft;
  const float iconCx = listLeft + colW + iconW * 0.5f;
  const float bearingCx = listLeft + colW + iconW + colW * 0.5f;
  const float distanceCx = listLeft + colW + iconW + colW * 1.5f;
  const float approachCx = listLeft + colW + iconW + colW * 2.5f;

  // Row 2 (WT flex): freqtype (30%, left) | frequency (30%, right) | RWY left.
  const float freqTypeX = listLeft;
  const float freqRight = listLeft + listW * 0.60f;
  const float rwyX = listLeft + listW * 0.62f;

  char buf[32];
  for (int row = 0; row < kVisibleEntries; ++row) {
    const int i = first + row;
    if (i >= static_cast<int>(list.size())) break;
    const NearestAirport& apt = list[i];
    const float top = f.contentTop + entryH * static_cast<float>(row);
    const float row1Cy = top + rowH * 0.5f;
    const float row2Cy = top + rowH + rowH * 0.5f;

    if (row > 0) {
      r.strokeLine(listLeft, top, listRight, top, 1.0f,
                   withAlpha(colors::kPanelSeparator, a));
    }

    // Row 1: ident (FMS cursor highlight), symbol, bearing, distance, approach.
    putField(r, identX, row1Cy, apt.id, size, colors::kCyan, i == cursor, a,
             blinkOn, 0.15f);
    MapFeature sym;
    sym.type = MapFeatureType::Airport;
    sym.airportTowered = apt.airportTowered;
    sym.airportServiced = apt.airportServiced;
    sym.airportKind = apt.airportKind;
    drawMapFeatureSymbol(r, sym, iconCx, row1Cy, fontPx(15.0f, h) * 0.5f);

    std::snprintf(buf, sizeof(buf), "%s\u00b0",
                  formatHeading(apt.bearingDeg).c_str());
    r.fillText(bearingCx, row1Cy, buf, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));
    std::snprintf(buf, sizeof(buf), "%.1fNM", apt.distanceNm);
    r.fillText(distanceCx, row1Cy, buf, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));
    const std::string& approach =
        apt.approachType.empty() ? std::string("VFR") : apt.approachType;
    r.fillText(approachCx, row1Cy, approach, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));

    // Row 2: comm-service label, tunable frequency (cyan), longest runway.
    if (!apt.comLabel.empty()) {
      r.fillText(freqTypeX, row2Cy, apt.comLabel, smallSize, TextAlign::Left,
                 withAlpha(colors::kWhite, a));
    }
    if (apt.frequencyMhz > 0.0f) {
      r.fillText(freqRight, row2Cy, formatFreq(apt.frequencyMhz, 3), size,
                 TextAlign::Right, withAlpha(colors::kCyan, a));
    }
    const std::string rwy =
        apt.longestRunwayFt > 0
            ? formatInt(static_cast<float>(apt.longestRunwayFt)) + "FT"
            : "_____";
    r.fillText(rwyX, row2Cy, "RWY", smallSize, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(rwyX + r.measureTextWidth("RWY ", smallSize), row2Cy, rwy, size,
               TextAlign::Left, withAlpha(colors::kWhite, a));
  }

  // Scroll thumb (WT ScrollBar): only when more airports exist than fit.
  if (static_cast<int>(list.size()) > kVisibleEntries) {
    const float trackTop = f.contentTop;
    const float trackH = entryH * static_cast<float>(kVisibleEntries);
    const float trackX = f.x + f.w - padX - scrollW;
    const float thumbH =
        std::max(fontPx(18.0f, h),
                 trackH * static_cast<float>(kVisibleEntries) /
                     static_cast<float>(list.size()));
    const float maxScroll =
        static_cast<float>(list.size() - kVisibleEntries);
    const float thumbTop =
        trackTop +
        (trackH - thumbH) * static_cast<float>(first) / maxScroll;
    r.fillRect(trackX + scrollW * 0.5f - 1.0f, trackTop, 2.0f, trackH,
               withAlpha(colors::kPanelSeparator, a));
    r.fillRect(trackX, thumbTop, scrollW, thumbH,
               withAlpha(colors::kMenuBorderGray, a));
  }
}

}  // namespace avionics::pfd
