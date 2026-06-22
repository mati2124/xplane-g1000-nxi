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
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Nearest),
                      "Nearest Airports", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const auto& list = ui.nearestAirports();
  // WT .nearest-airport-popout-container: 20 px body.
  const float size = fontPx(wt::kInfoValue, h);
  const float smallSize = fontPx(14.0f, h);
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
  const int first = std::max(0, cursor - kVisibleEntries + 1);
  const float entryH = fontPx(58.0f, h);
  const float rowH = entryH * 0.5f;
  const float padX = fontPx(5.0f, h);
  const float scrollW = wtScrollBarLane(h);
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

  // Row 2 (WT flex): service label (left) | frequency (cyan) | longest runway.
  // The runway block is right-aligned to the list edge and the frequency is
  // right-aligned just to its left, so both adapt to the runway's digit count
  // and never collide with each other or overflow the popup.
  const float freqTypeX = listLeft;

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
    putField(r, identX, row1Cy, apt.id, size, colors::kPopoutCyan, i == cursor, a,
             0.15f);
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
    // Distance: whole nautical miles with a smaller "NM" suffix, centered in
    // the column (Pilot's Guide Fig. 4-4 shows whole-number distances).
    std::snprintf(buf, sizeof(buf), "%.0f", apt.distanceNm);
    const float distNumW = r.measureTextWidth(buf, size);
    const float unitGap = size * 0.12f;
    const float distNmW = r.measureTextWidth("NM", smallSize);
    const float distX = distanceCx - (distNumW + unitGap + distNmW) * 0.5f;
    r.fillText(distX, row1Cy, buf, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(distX + distNumW + unitGap, row1Cy, "NM", smallSize,
               TextAlign::Left, withAlpha(colors::kWhite, a));
    const std::string& approach =
        apt.approachType.empty() ? std::string("VFR") : apt.approachType;
    r.fillText(approachCx, row1Cy, approach, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));

    // Row 2: comm-service label (left), longest runway (right-aligned to the
    // list edge), and the tunable frequency right-aligned just left of it. The
    // runway keeps the body size with smaller "RWY"/"FT" affixes (Fig. 4-4).
    if (!apt.comLabel.empty()) {
      r.fillText(freqTypeX, row2Cy, apt.comLabel, smallSize, TextAlign::Left,
                 withAlpha(colors::kWhite, a));
    }
    const bool hasRwy = apt.longestRunwayFt > 0;
    const std::string rwyNum =
        hasRwy ? formatInt(static_cast<float>(apt.longestRunwayFt))
               : std::string("_____");
    const float rwyLabelW = r.measureTextWidth("RWY ", smallSize);
    const float rwyNumW = r.measureTextWidth(rwyNum, size);
    const float ftGap = hasRwy ? size * 0.06f : 0.0f;
    const float ftW = hasRwy ? r.measureTextWidth("FT", smallSize) : 0.0f;
    const float rwyX = listRight - rwyLabelW - rwyNumW - ftGap - ftW;
    r.fillText(rwyX, row2Cy, "RWY", smallSize, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(rwyX + rwyLabelW, row2Cy, rwyNum, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    if (hasRwy) {
      r.fillText(rwyX + rwyLabelW + rwyNumW + ftGap, row2Cy, "FT", smallSize,
                 TextAlign::Left, withAlpha(colors::kWhite, a));
    }
    // Frequency left-aligned just after the COM service label (Fig. 4-4 places
    // the tunable frequency immediately to the right of the service label).
    if (apt.frequencyMhz > 0.0f) {
      const float serviceW =
          apt.comLabel.empty() ? 0.0f
                               : r.measureTextWidth(apt.comLabel, smallSize);
      r.fillText(freqTypeX + serviceW + fontPx(9.0f, h), row2Cy,
                 formatFreq(apt.frequencyMhz, 3), size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
    }
  }

  // Scroll thumb (WT ScrollBar): only when more airports exist than fit.
  if (static_cast<int>(list.size()) > kVisibleEntries) {
    const float trackTop = f.contentTop;
    const float trackH = entryH * static_cast<float>(kVisibleEntries);
    const float trackX = f.x + f.w - padX - scrollW;
    drawWtScrollBar(r, h, trackX, trackTop, trackH,
                    static_cast<int>(list.size()), kVisibleEntries, first, a);
  }
}

}  // namespace avionics::pfd
