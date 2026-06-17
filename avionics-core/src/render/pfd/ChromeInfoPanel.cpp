#include <cstdio>
#include <cmath>
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

// OAT / XPDR / time sub-panels each carry their own gradient on the real unit
// (WT bip-temp-box, xpdr-container, bip-time).
void drawInfoSubPanel(Renderer& r, float x, float top, float w, float h) {
  drawInfoBox(r, x, top, w, h);
}

// Transponder code/mode color per WT Transponder.tsx: OFF grey, STBY white,
// ON/ALT/GND green.
Color xpdrValueColor(const std::string& mode) {
  if (mode == "OFF") return colors::kDisabledGray;
  if (mode == "STBY") return colors::kWhite;
  return colors::kActiveGreen;
}

// One horizontal span of the bottom-panel vertical gradient.
void fillInfoGradientSpan(Renderer& r, float x, float y, float w, float h,
                          float bandTop, float bandBottom) {
  if (w <= 0.0f || h <= 0.0f) return;
  r.fillRectVerticalGradient(x, y, w, h, bandTop, bandBottom, colors::kInfoBoxTop,
                             colors::kBlack);
}

// HSI Map off (trainer PFD HSI Map Off.bmp): full-width band with a circular
// cutout aligned to the standard rose so the compass ring shows through the
// 55px panel.
void drawBandRoseCircleCutout(Renderer& r, float w, float top, float panelH,
                              float roseCx, float roseCy, float roseRadius) {
  const float bottom = top + panelH;
  const float step = std::max(0.5f, panelH / 55.0f);

  for (float y = top; y < bottom; y += step) {
    const float rh = std::min(step, bottom - y);
    const float dy = y - roseCy;
    if (std::abs(dy) >= roseRadius) {
      fillInfoGradientSpan(r, 0.0f, y, w, rh, top, bottom);
      continue;
    }
    const float dx =
        std::sqrt(std::max(0.0f, roseRadius * roseRadius - dy * dy));
    const float uncutL = roseCx - dx;
    const float uncutR = roseCx + dx;
    if (uncutL > 0.0f) {
      fillInfoGradientSpan(r, 0.0f, y, uncutL, rh, top, bottom);
    }
    if (uncutR < w) {
      fillInfoGradientSpan(r, uncutR, y, w - uncutR, rh, top, bottom);
    }
  }
}

// Trainer PFD Inset Map.bmp: square inset-map opening (viewport width) plus the
// standard HSI rose circular cutout.
void drawBandInsetRectAndRoseCutouts(Renderer& r, float w, float top, float panelH,
                                     float insetX, float insetY, float insetW,
                                     float insetH, float roseCx, float roseCy,
                                     float roseR) {
  const float bottom = top + panelH;
  const float insetBottom = insetY + insetH;
  const float step = std::max(0.5f, panelH / 55.0f);

  for (float y = top; y < bottom; y += step) {
    const float rh = std::min(step, bottom - y);

    float holes[4];
    int holeCount = 0;

    // Square inset-map opening where this band row overlaps the map viewport.
    if (y + rh > insetY && y < insetBottom) {
      holes[holeCount++] = insetX;
      holes[holeCount++] = insetX + insetW;
    }

    const float roseDy = y - roseCy;
    if (std::abs(roseDy) < roseR) {
      const float roseDx =
          std::sqrt(std::max(0.0f, roseR * roseR - roseDy * roseDy));
      holes[holeCount++] = roseCx - roseDx;
      holes[holeCount++] = roseCx + roseDx;
    }

    if (holeCount == 0) {
      fillInfoGradientSpan(r, 0.0f, y, w, rh, top, bottom);
      continue;
    }

    for (int i = 0; i < holeCount - 1; ++i) {
      for (int j = i + 1; j < holeCount; ++j) {
        if (holes[j] < holes[i]) {
          const float tmp = holes[i];
          holes[i] = holes[j];
          holes[j] = tmp;
        }
      }
    }

    float uncutL = 0.0f;
    for (int i = 0; i < holeCount; i += 2) {
      const float holeL = std::max(0.0f, holes[i]);
      const float holeR = std::min(w, holes[i + 1]);
      if (holeL > uncutL) {
        fillInfoGradientSpan(r, uncutL, y, holeL - uncutL, rh, top, bottom);
      }
      uncutL = std::max(uncutL, holeR);
    }
    if (uncutL < w) {
      fillInfoGradientSpan(r, uncutL, y, w - uncutL, rh, top, bottom);
    }
  }
}

// Trainer PFD Default.bmp (1024x768): per-row opening bounds in the middle
// band (y=679-727), sampled from the trainer with topo on; solid fill from
// y=728 downward.
namespace trainerHsiMapCutout {
struct Row {
  float y;
  float openLeft;
  float openRight;
};

constexpr float kSolidBelowPx = 728.0f;

constexpr Row kRows[] = {
    {679.0f, 293.0f, 615.0f}, {680.0f, 293.0f, 617.0f},
    {681.0f, 293.0f, 621.0f}, {682.0f, 290.0f, 628.0f},
    {683.0f, 355.0f, 564.0f}, {684.0f, 356.0f, 563.0f},
    {685.0f, 357.0f, 562.0f}, {686.0f, 358.0f, 561.0f},
    {687.0f, 359.0f, 560.0f}, {688.0f, 360.0f, 559.0f},
    {689.0f, 361.0f, 558.0f}, {690.0f, 367.0f, 550.0f},
    {691.0f, 367.0f, 550.0f}, {692.0f, 368.0f, 549.0f},
    {693.0f, 369.0f, 548.0f}, {694.0f, 366.0f, 553.0f},
    {695.0f, 368.0f, 551.0f}, {696.0f, 373.0f, 550.0f},
    {697.0f, 370.0f, 549.0f}, {698.0f, 371.0f, 548.0f},
    {699.0f, 373.0f, 546.0f}, {700.0f, 374.0f, 545.0f},
    {701.0f, 375.0f, 544.0f}, {702.0f, 377.0f, 542.0f},
    {703.0f, 378.0f, 541.0f}, {704.0f, 380.0f, 539.0f},
    {705.0f, 381.0f, 538.0f}, {706.0f, 383.0f, 536.0f},
    {707.0f, 385.0f, 534.0f}, {708.0f, 386.0f, 533.0f},
    {709.0f, 388.0f, 531.0f}, {710.0f, 390.0f, 529.0f},
    {711.0f, 392.0f, 527.0f}, {712.0f, 394.0f, 525.0f},
    {713.0f, 396.0f, 523.0f}, {714.0f, 398.0f, 521.0f},
    {715.0f, 400.0f, 519.0f}, {716.0f, 402.0f, 517.0f},
    {717.0f, 404.0f, 515.0f}, {718.0f, 407.0f, 512.0f},
    {719.0f, 410.0f, 509.0f}, {720.0f, 412.0f, 507.0f},
    {721.0f, 415.0f, 504.0f}, {722.0f, 419.0f, 500.0f},
    {723.0f, 422.0f, 497.0f}, {724.0f, 426.0f, 493.0f},
    {725.0f, 431.0f, 488.0f}, {726.0f, 436.0f, 483.0f},
    {727.0f, 443.0f, 476.0f},
};

constexpr int kRowCount = static_cast<int>(sizeof(kRows) / sizeof(kRows[0]));

bool openingAtTrainerY(float yTrainerPx, float& openLeft, float& openRight) {
  if (yTrainerPx >= kSolidBelowPx) return false;
  if (yTrainerPx <= kRows[0].y) {
    openLeft = kRows[0].openLeft;
    openRight = kRows[0].openRight;
    return true;
  }
  for (int i = 1; i < kRowCount; ++i) {
    if (yTrainerPx <= kRows[i].y) {
      const float span = kRows[i].y - kRows[i - 1].y;
      const float t = span > 0.0f ? (yTrainerPx - kRows[i - 1].y) / span : 0.0f;
      openLeft =
          kRows[i - 1].openLeft + t * (kRows[i].openLeft - kRows[i - 1].openLeft);
      openRight =
          kRows[i - 1].openRight + t * (kRows[i].openRight - kRows[i - 1].openRight);
      return true;
    }
  }
  openLeft = kRows[kRowCount - 1].openLeft;
  openRight = kRows[kRowCount - 1].openRight;
  return true;
}
}  // namespace trainerHsiMapCutout

void drawMiddleBandHsiMapCutout(Renderer& r, float left, float right, float top,
                                float panelH, float scaleX, float scaleY) {
  const float bottom = top + panelH;
  const float step = std::max(0.5f, panelH / 55.0f);

  for (float y = top; y < bottom; y += step) {
    const float rh = std::min(step, bottom - y);
    const float yTrainer = y / scaleY;
    float openLeftTrainer = 0.0f;
    float openRightTrainer = 0.0f;
    if (!trainerHsiMapCutout::openingAtTrainerY(yTrainer, openLeftTrainer,
                                                openRightTrainer)) {
      fillInfoGradientSpan(r, left, y, right - left, rh, top, bottom);
      continue;
    }

    const float openL =
        std::max(openLeftTrainer * scaleX, left);
    const float openR =
        std::min(openRightTrainer * scaleX, right);
    if (openL > left) {
      fillInfoGradientSpan(r, left, y, openL - left, rh, top, bottom);
    }
    if (openR < right) {
      fillInfoGradientSpan(r, openR, y, right - openR, rh, top, bottom);
    }
  }
}

void drawInfoBand(Renderer& r, float w, float top, float panelH, float oatW,
                  float xpdrX, const Layout& L, bool hsiMapMode,
                  bool insetMapMode) {
  if (hsiMapMode) {
    // Trainer PFD Default / Main: OAT | HSI-map cutout | XPDR/time.
    drawInfoBox(r, 0.0f, top, oatW, panelH);
    drawMiddleBandHsiMapCutout(r, oatW, xpdrX, top, panelH, L.sx, L.sy);
    drawInfoBox(r, xpdrX, top, w - xpdrX, panelH);
  } else if (insetMapMode) {
    // Trainer PFD Inset Map.bmp: square inset-map + HSI rose cutouts.
    drawBandInsetRectAndRoseCutouts(r, w, top, panelH, L.insetMapX, L.insetMapY,
                                    L.insetMapW, L.insetMapH, L.hsiCx, L.hsiCy,
                                    L.hsiRadius);
  } else {
    // Trainer PFD HSI Map Off: circular cutout for the standard HSI rose.
    drawBandRoseCircleCutout(r, w, top, panelH, L.hsiCx, L.hsiCy, L.hsiRadius);
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

  // Trainer: HSI Map off → rose circle cutout; HSI Map on → bip-middle arc;
  // Inset Map layout → inset-map + rose dual cutouts.
  const bool insetMapMode = ui.insetMapVisible() && !ui.hsiMapVisible();
  drawInfoBand(r, w, top, panelH, oatW, xpdrX, L, ui.hsiMapVisible(),
               insetMapMode);

  // WT BottomInfoPanel: OAT, XPDR, and TMR/UTC each sit on their own gradient
  // fill; OAT also has a groove separator on its right edge. When the inset map
  // is showing, only paint the OAT box below the square map viewport so the
  // band cutout can expose the map above it (trainer PFD Inset Map.bmp).
  const float grooveW = std::max(2.0f, 4.0f * L.sx);
  const float bandBottom = top + panelH;
  if (insetMapMode) {
    const float insetBottom = L.insetMapY + L.insetMapH;
    if (insetBottom < bandBottom) {
      const float oatStripH = bandBottom - insetBottom;
      drawInfoSubPanel(r, 0.0f, insetBottom, oatW, oatStripH);
      r.fillRect(oatW - grooveW, insetBottom, grooveW, oatStripH,
                 Color{0.004f, 0.004f, 0.004f, 0.4f});
    }
  } else {
    drawInfoSubPanel(r, 0.0f, top, oatW, panelH);
    r.fillRect(oatW - grooveW, top, grooveW, panelH,
               Color{0.004f, 0.004f, 0.004f, 0.4f});
  }
  drawInfoSubPanel(r, xpdrX, top, xpdrW, panelH);
  drawInfoSubPanel(r, timeX, top, timeW, panelH);

  const float bottomRowCy = top + kInfoBottomRowCenterPx * L.sy;
  const float topRowCy = top + kInfoTopRowCenterPx * L.sy;
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
  // back over the label. When the inset map is showing, OAT sits lower in the
  // narrow strip below the square map viewport (trainer PFD Inset Map.bmp).
  const float oatCy =
      insetMapMode
          ? L.insetMapY + L.insetMapH + kInsetMapOatCyBelowViewportPx * L.sy
          : bottomRowCy;
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
  const float xpdrCy = bottomRowCy;
  const float xpdrLabelColW = 60.0f * L.sx;
  const float xpdrCodeColW = 53.0f * L.sx;
  const float xpdrLabelX = xpdrX + 4.0f * L.sx;
  const float xpdrCodeX = xpdrX + xpdrLabelColW;
  const float xpdrModeX = xpdrX + xpdrLabelColW + xpdrCodeColW;
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
    r.save();
    r.clip(xpdrX, top, xpdrW, panelH);
    putText(r, xpdrLabelX, xpdrCy, "XPDR", labelSize, colors::kLabelText, 0.0f);
    const std::string& pending = ui.xpdrPendingCode();
    if (!pending.empty()) {
      std::string entry = pending;
      entry.append(4 - pending.size(), '-');
      putText(r, xpdrCodeX, xpdrCy, entry, valueSize, colors::kWhite, 0.0f);
    } else if (linkValid) {
      const Color xpdrColor = xpdrValueColor(d.transponderMode);
      char codeBuf[8];
      std::snprintf(codeBuf, sizeof(codeBuf), "%04d", d.transponderCode);
      putText(r, xpdrCodeX, xpdrCy, codeBuf, valueSize, xpdrColor, 0.0f);
      if (ui.identActive()) {
        putText(r, xpdrModeX, xpdrCy, "IDNT", valueSize, colors::kActiveGreen,
                0.0f);
      } else {
        putText(r, xpdrModeX, xpdrCy, d.transponderMode, valueSize, xpdrColor,
                0.0f);
        if (d.transponderReply) {
          const float modeW = r.measureTextWidth(d.transponderMode, valueSize);
          putText(r, xpdrModeX + modeW, xpdrCy, "R", valueSize, xpdrColor, 0.0f);
        }
      }
    } else {
      putText(r, xpdrCodeX, xpdrCy, kXpdrDashes, valueSize, colors::kBandYellow,
               0.0f);
    }
    r.restore();
  }

  // Time/timer box (WT bip-time at x=900 w=124: 30px label + 85px value columns).
  const float timeLabelColW = 30.0f * L.sx;
  const float timeLabelX = timeX + 2.0f * L.sx;
  const float valueColLeft = timeX + timeLabelColW;
  const float timeValueRight = timeX + 124.0f * L.sx - 2.0f * L.sx;
  const float tmrRowCy = topRowCy;
  const float timeRowCy = bottomRowCy;
  const float rowXHeight = panelH * 0.30f;
  const float timeValueSize = valueSize;
  const char* clockLabel = d.clockIsUtc ? "UTC" : "LCL";

  const float timeLabelGap = labelSize * 0.45f;
  const auto drawTimeRow = [&](float cy, const char* label, const std::string& value,
                               const Color& c) {
    const float valueX =
        putText(r, timeLabelX, cy, label, labelSize, colors::kLabelText,
                timeLabelGap / labelSize);
    r.save();
    r.clip(valueColLeft, top, timeValueRight - valueColLeft, panelH);
    const float valueW = r.measureTextWidth(value, timeValueSize);
    if (timeValueRight - valueW >= valueX) {
      r.fillText(timeValueRight, cy, value, timeValueSize, TextAlign::Right, c);
    } else {
      r.fillText(valueX, cy, value, timeValueSize, TextAlign::Left, c);
    }
    r.restore();
  };

  // TMR row. The timer is local to the display, so it reads even with the data
  // link down; it is blanked with an X only during power-up.
  if (powerUp) {
    putText(r, timeLabelX, tmrRowCy, "TMR", labelSize, colors::kLabelText, 0.0f);
    drawFieldX(timeValueRight, timeValueRight - valueColLeft, tmrRowCy,
               rowXHeight);
  } else {
    drawTimeRow(tmrRowCy, "TMR", formatTimer(ui.timerSeconds()),
                colors::kWhite);
  }

  // UTC/LCL clock row.
  if (powerUp) {
    putText(r, timeLabelX, timeRowCy, clockLabel, labelSize, colors::kLabelText,
            0.0f);
    drawFieldX(timeValueRight, timeValueRight - valueColLeft, timeRowCy,
               rowXHeight);
  } else {
    drawTimeRow(timeRowCy, clockLabel,
                linkValid ? formatHms(d.utcHour, d.utcMinute, d.utcSecond)
                          : std::string(kTimeDashes),
                valueColor);
  }
}

}  // namespace avionics::pfd
