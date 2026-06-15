#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {
namespace {

// Filled box with the NXi bottom-panel gradient (lighter at top, black at the
// bottom).
void drawInfoBox(Renderer& r, float x, float top, float w, float h) {
  r.fillRectVerticalGradient(x, top, w, h, top, top + h, colors::kInfoBoxTop,
                             colors::kBlack);
}

// Draws the full-width bottom band, optionally with a circular cut-out for the
// HSI Map compass rose. In HSI Map mode the rose is enlarged and lowered so its
// lower disc dips down into (and through) the band; the real NXi notches the
// band around it so the moving map shows through the cut-out rather than the
// band painting over it. The rose center sits above the band's top edge, so for
// every column across the rose the band is suppressed from the top edge down to
// the circle's lower boundary (the part the rose covers), leaving only the
// band below the disc -- the moving map was already drawn underneath.
void drawInfoBand(Renderer& r, float w, float top, float panelH, bool cutout,
                  float roseCx, float roseCy, float roseR) {
  if (!cutout) {
    drawInfoBox(r, 0.0f, top, w, panelH);
    return;
  }
  const float bottom = top + panelH;
  // Horizontal half-width where the disc meets the band's top edge (its widest
  // reach into the band); the band is solid outside [xL, xR].
  const float dTop = top - roseCy;  // > 0: band top is below the rose center
  const float spanTop =
      roseR > dTop ? std::sqrt(roseR * roseR - dTop * dTop) : 0.0f;
  const float xL = roseCx - spanTop;
  const float xR = roseCx + spanTop;
  if (xL > 0.0f) drawInfoBox(r, 0.0f, top, xL, panelH);
  if (xR < w) drawInfoBox(r, xR, top, w - xR, panelH);
  // Thin columns across the disc, each filled only below the circle's lower
  // edge so the cut-out follows the rose's curve.
  const float step = std::max(1.0f, panelH * 0.04f);
  for (float x = xL; x < xR; x += step) {
    const float cw = std::min(step, xR - x);
    const float dx = x + cw * 0.5f - roseCx;
    const float inside = roseR * roseR - dx * dx;
    const float dy = inside > 0.0f ? std::sqrt(inside) : 0.0f;
    const float circBottom = roseCy + dy;  // disc's lower edge in this column
    if (circBottom >= bottom) continue;     // disc covers the whole band height
    const float bandTop = std::max(circBottom, top);
    r.fillRectVerticalGradient(x, bandTop, cw, bottom - bandTop, top, bottom,
                               colors::kInfoBoxTop, colors::kBlack);
  }
}

// Small bearing-pointer icon drawn at the left of a Bearing Information Window:
// a single line for BRG1, a double line for BRG2 (G1000 NXi Pilot's Guide).
void drawBearingIcon(Renderer& r, float x, float cy, float len, bool dbl,
                     const Color& c) {
  if (!dbl) {
    r.strokeLine(x, cy, x + len, cy, 2.0f, c);
  } else {
    const float off = len * 0.16f;
    r.strokeLine(x, cy - off, x + len, cy - off, 2.0f, c);
    r.strokeLine(x, cy + off, x + len, cy + off, 2.0f, c);
  }
}

// One Bearing Information Window: pointer icon, source, station/waypoint
// identifier, and GPS-derived slant-range distance. When alignRight is set,
// `x` is the window's RIGHT edge (used for BRG2 so it never grows into the
// transponder box).
void drawBearingInfo(Renderer& r, float x, float cy, bool dbl,
                     const std::string& source, const std::string& ident,
                     float distNm, float size, bool alignRight = false) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.1fNM", distNm);

  if (alignRight) {
    float width = size * 1.2f + r.measureTextWidth(source, size) + size * 0.35f;
    if (!ident.empty()) {
      width += r.measureTextWidth(ident, size) + size * 0.5f;
    }
    width += r.measureTextWidth(buf, size);
    x -= width;
  }

  drawBearingIcon(r, x, cy, size * 0.9f, dbl, colors::kCyan);
  float bx = x + size * 1.2f;
  bx = putText(r, bx, cy, source, size, colors::kCyan, 0.35f);
  if (!ident.empty()) bx = putText(r, bx, cy, ident, size, colors::kCyan, 0.5f);
  putText(r, bx, cy, buf, size, colors::kMagenta, 0.0f);
}

}  // namespace

void drawBottomInfoPanel(Renderer& r, float w, float h, const Layout& L,
                         const FlightData& d, const SoftkeyController& ui,
                         bool powerUp) {
  // NXi bottom info panel (y=679, height 55, full width 1024): a single
  // continuous dark band spans the entire width (it covers the bottom of the
  // HSI compass rose, which pokes up above its top edge -- matching the real
  // unit). The OAT readout sits at the left, the transponder + TMR/UTC readouts
  // at the right, and the bearing-pointer / DME info windows in the middle.
  const float top = L.infoPanelTop;
  const float panelH = L.infoPanelH;
  const float labelSize = fontPx(wt::kInfoLabel, h);
  const float valueSize = fontPx(wt::kInfoValue, h);

  const float oatW = 101.0f * L.sx;
  const float xpdrX = 715.0f * L.sx;
  const float xpdrW = 185.0f * L.sx;
  const float timeX = 900.0f * L.sx;
  const float timeW = w - timeX;

  // In HSI Map mode the enlarged rose dips into the band, so the band is
  // notched around it (the standard rose sits above the band, no cut-out).
  drawInfoBand(r, w, top, panelH, ui.hsiMapVisible(), L.hsiMapCx, L.hsiMapCy,
               L.hsiMapRadius);

  const bool linkValid = d.dataLinkValid;
  const Color valueColor = linkValid ? colors::kWhite : colors::kBandYellow;

  // A thin red X over a right-aligned data field, used in place of the value
  // during power-up when its source has not initialized (NXi Fig 9-2). `cy` is
  // the field's vertical center and `bh` the X height, so a single-row field
  // (OAT) and the two stacked time rows can each X just their own row.
  const auto drawFieldX = [&](float rightX, float fieldW, float cy, float bh) {
    const float x0 = rightX - fieldW;
    const float thick = std::max(1.5f, h * 0.0038f);
    r.strokeLine(x0, cy - bh * 0.5f, rightX, cy + bh * 0.5f, thick,
                 colors::kFailedX);
    r.strokeLine(x0, cy + bh * 0.5f, rightX, cy - bh * 0.5f, thick,
                 colors::kFailedX);
  };

  // OAT readout, on the band's bottom row (shares its baseline with the XPDR
  // and UTC/LCL readouts on the real NXi). Label and value flow left-to-right
  // with a fixed gap between them (real NXi: "OAT  -8°C"); the value is no
  // longer right-aligned into a narrow box, so a wide reading can never grow
  // back over the label.
  const float oatCy = top + panelH * 0.76f;
  const float oatValueX = putText(r, oatW * 0.06f, oatCy, "OAT", labelSize,
                                  colors::kLabelText, 0.55f);
  if (powerUp) {
    const float xW = r.measureTextWidth("-99\u00b0C", valueSize);
    drawFieldX(oatValueX + xW, xW, oatCy, panelH * 0.42f);
  } else {
    r.fillText(oatValueX, oatCy,
               (linkValid ? formatOat(d.oatCelsius) : std::string(kOatDashes)) +
                   "\u00b0C",
               valueSize, TextAlign::Left, valueColor);
  }

  // Bearing-pointer info windows flanking the rose (BRG1 left, BRG2 right),
  // each with a pointer icon, source, identifier, and distance.
  const float brgSize = fontPx(wt::kInfoLabel, h);
  const float brgRowCy = top + panelH * 0.55f;
  const bool brg1On = ui.displayToggle(DisplayToggle::Bearing1);
  const bool brg2On = ui.displayToggle(DisplayToggle::Bearing2);
  if (d.bearing1Valid && brg1On) {
    drawBearingInfo(r, 200.0f * L.sx, brgRowCy, false, d.bearing1Source,
                    d.bearing1Ident, d.bearing1DistanceNm, brgSize);
  }
  if (d.bearing2Valid && brg2On) {
    // BRG2 is anchored to the left edge of the transponder box so the distance
    // readout can never run into it.
    const float brg2Right = xpdrX - 10.0f * L.sx;
    drawBearingInfo(r, brg2Right, brgRowCy, true, d.bearing2Source,
                    d.bearing2Ident, d.bearing2DistanceNm, brgSize,
                    /*alignRight=*/true);
  }

  // DME Information Window (PFD Opt > DME), shown above the BRG1 window: tuned
  // source/mode, frequency, and slant-range.
  if (ui.displayToggle(DisplayToggle::Dme) && d.dmeValid) {
    const float dmeCy = top - brgSize * 1.5f;
    float dx = 200.0f * L.sx;
    dx = putText(r, dx, dmeCy, "DME", brgSize, colors::kLabelText, 0.4f);
    dx = putText(r, dx, dmeCy, d.dmeMode, brgSize, colors::kCyan, 0.4f);
    dx = putText(r, dx, dmeCy, formatFreq(d.dmeFreqMhz, 2), brgSize,
                 colors::kCyan, 0.5f);
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1fNM", d.dmeDistanceNm);
    putText(r, dx, dmeCy, buf, brgSize, colors::kWhite, 0.0f);
  }

  // Transponder box: code, mode and reply ("R"), green in the air. With the
  // link down the squawk/mode are unknown, so the box shows amber dashes. An
  // in-progress code entry (XPDR > Code) shows in place of the active code,
  // and the Ident function replaces the mode with a green IDNT for 18 seconds.
  // XPDR sits on the bottom row of the band, sharing its baseline with the
  // UTC/LCL clock to its right (real NXi: TMR on the upper row, XPDR + clock on
  // the lower row).
  const float xpdrCy = top + panelH * 0.76f;
  const float xpdrLabelX = xpdrX + xpdrW * 0.04f;
  // A failed transponder (its failure dataref tripped, or during power-up
  // before it initializes) annunciates "XPDR FAIL". On the real NXi (Fig 9-2)
  // the whole annunciation -- label included -- is amber, with a red X struck
  // over the field like the other failed bottom-panel readouts (OAT/UTC).
  if (powerUp || !d.transponderValid) {
    // "XPDR FAIL" -- both words the same (bold) value face, amber, horizontally
    // centered in the field and sharing the bottom-row baseline with the
    // UTC/LCL clock, with the red X spanning the whole field out to the TMR/UTC
    // box edge (NXi Fig 9-2).
    FontScope xpdrFont(r, FontFace::DejaVuSemiBold);
    const float gap = valueSize * 0.4f;
    const float wXpdr = r.measureTextWidth("XPDR", valueSize);
    const float wFail = r.measureTextWidth("FAIL", valueSize);
    const float xpdrRight = xpdrX + xpdrW;
    const float startX =
        (xpdrLabelX + xpdrRight) * 0.5f - (wXpdr + gap + wFail) * 0.5f;
    float fx = putText(r, startX, xpdrCy, "XPDR", valueSize, colors::kBandYellow,
                       gap / valueSize);
    putText(r, fx, xpdrCy, "FAIL", valueSize, colors::kBandYellow, 0.0f);
    drawFieldX(xpdrRight, xpdrRight - xpdrLabelX, xpdrCy, panelH * 0.42f);
  } else {
    float xx =
        putText(r, xpdrLabelX, xpdrCy, "XPDR", labelSize, colors::kLabelText,
                0.40f);
    const std::string& pending = ui.xpdrPendingCode();
    if (!pending.empty()) {
      // Digits typed so far, with the remaining places dashed.
      std::string entry = pending;
      entry.append(4 - pending.size(), '-');
      xx = putText(r, xx, xpdrCy, entry, valueSize, colors::kWhite, 0.45f);
    } else if (linkValid) {
      char codeBuf[8];
      std::snprintf(codeBuf, sizeof(codeBuf), "%04d", d.transponderCode);
      xx =
          putText(r, xx, xpdrCy, codeBuf, valueSize, colors::kActiveGreen, 0.45f);
    } else {
      xx = putText(r, xx, xpdrCy, kXpdrDashes, valueSize, colors::kBandYellow,
                   0.45f);
    }
    if (ui.identActive()) {
      putText(r, xx, xpdrCy, "IDNT", valueSize, colors::kActiveGreen, 0.0f);
    } else if (linkValid) {
      xx = putText(r, xx, xpdrCy, d.transponderMode, valueSize,
                   colors::kActiveGreen, 0.45f);
      if (d.transponderReply) {
        putText(r, xx, xpdrCy, "R", valueSize, colors::kActiveGreen, 0.0f);
      }
    }
  }

  // Time/timer box (NXi bottom-right corner): two stacked rows -- the generic
  // TMR timer on top and the UTC/LCL clock on the bottom (Pilot's Guide Fig.
  // 2-1). Both fields are X'd during power-up per Maintenance Manual Fig. 9-2.
  const float timeLabelX = timeX + timeW * 0.06f;
  const float timeValueX = w - timeW * 0.05f;
  const float tmrRowCy = top + panelH * 0.33f;
  const float timeRowCy = top + panelH * 0.76f;
  const float rowXHeight = panelH * 0.30f;
  // Both rows must fit one above the other in the panel height, so this box's
  // values use the compact label size rather than the single-row value size
  // (matches the real NXi corner box, where label + clock leave a small gap).
  const float timeValueSize = labelSize;

  // The TMR and UTC rows share one value column: their failure Xes use a
  // common left edge (just past the widest label) and the common right edge
  // (timeValueX) so the two rows line up exactly on both sides (NXi Fig 9-2).
  const char* clockLabel = d.clockIsUtc ? "UTC" : "LCL";
  const float timeFieldLeft =
      timeLabelX +
      std::max(r.measureTextWidth("TMR", labelSize),
               r.measureTextWidth(clockLabel, labelSize)) +
      labelSize * 0.3f;

  // TMR row. The timer is local to the display, so it reads even with the data
  // link down; it is blanked with an X only during power-up.
  r.fillText(timeLabelX, tmrRowCy, "TMR", labelSize, TextAlign::Left,
             colors::kLabelText);
  if (powerUp) {
    drawFieldX(timeValueX, timeValueX - timeFieldLeft, tmrRowCy, rowXHeight);
  } else {
    r.fillText(timeValueX, tmrRowCy, formatTimer(ui.timerSeconds()),
               timeValueSize, TextAlign::Right, colors::kWhite);
  }

  // UTC/LCL clock row.
  r.fillText(timeLabelX, timeRowCy, clockLabel, labelSize, TextAlign::Left,
             colors::kLabelText);
  if (powerUp) {
    drawFieldX(timeValueX, timeValueX - timeFieldLeft, timeRowCy, rowXHeight);
  } else {
    r.fillText(timeValueX, timeRowCy,
               linkValid ? formatHms(d.utcHour, d.utcMinute, d.utcSecond)
                         : std::string(kTimeDashes),
               timeValueSize, TextAlign::Right, valueColor);
  }
}

}  // namespace avionics::pfd
