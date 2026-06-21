#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

void drawSelectedAltitude(Renderer& r, float tapeX, float tapeW, float tapeTop,
                          float stripTop, float stripH, float cy,
                          float displayH, float altitudeFt, float selectedFt,
                          const SelectedAltStyle& alert) {
  // The Selected Altitude box fills the gap between the top of the altimeter
  // instrument and the top of the scrolling tape, mirroring the BARO box at the
  // bottom (G1000 NXi).
  const float boxH = stripTop - tapeTop;
  const float boxW = tapeW;
  const float boxX = tapeX;
  const float boxY = tapeTop;

  // X-Plane reports an unselected autopilot altitude as 0 ft. There's no real
  // "select 0 ft" use case, so treat it as unset: the box shows dashes and no
  // altitude bug is drawn on the tape (matching the G1000 NXi / X-Plane PFD).
  const bool selected = std::fabs(selectedFt) > kAltSelectedEpsilonFt;

  // Tape bug is drawn before the Selected Altitude box so that when the
  // selection is at the top of the visible range the carrot rides behind the
  // box and only its left tip peeks out (NXi trainer reference).
  if (selected) {
    const float ppu = stripH / kAltitudeViewableFeet;
    float bugY = cy - (selectedFt - altitudeFt) * ppu;
    bugY = std::max(stripTop, std::min(stripTop + stripH, bugY));
    const float bw = tapeW * 0.16f;
    const float bh = displayH * 0.020f;
    const float bx = tapeX;
    const Point bug[5] = {{bx, bugY - bh},
                          {bx + bw, bugY - bh},
                          {bx + bw * 0.5f, bugY},
                          {bx + bw, bugY + bh},
                          {bx, bugY + bh}};
    r.fillPolygon(bug, 5, colors::kCyan);
  }

  // Altitude Alerting (Pilot's Guide Fig. 2-32): during a phase transition the
  // readout flashes -- black-on-cyan within 1000 ft, blinking cyan within 200
  // ft, blinking amber on a post-capture deviation.
  const Color plate = alert.cyanBackground ? colors::kCyan : colors::kReadoutBox;
  Color text = alert.cyanBackground ? colors::kBlack
               : alert.amberText    ? colors::kBandYellow
                                    : colors::kCyan;
  // Only the top-right (outer) corner is rounded, matching the airspeed tape's
  // rounded outer-top corner and the WT NXi .preselect-box border-top-right-
  // radius; the inner and bottom corners stay square so the box sits flush
  // against the attitude window and the tape below it.
  const float cornerR = fontPx(kTapeCornerRadiusWt, displayH);
  const std::vector<Point> boxPoly = {{boxX, boxY},
                                      {boxX + boxW, boxY},
                                      {boxX + boxW, boxY + boxH},
                                      {boxX, boxY + boxH}};
  const std::vector<float> boxRadii = {0.0f, cornerR, 0.0f, 0.0f};
  std::vector<Point> boxShape = roundPolygonCorners(boxPoly, boxRadii);
  r.fillPolygon(boxShape.data(), static_cast<int>(boxShape.size()), plate);
  // The box border is a thin grey (rgb 100,100,100) on the real unit, not cyan
  // (WT NXi .preselect-box). Only the digits and the bug glyph are cyan.
  boxShape.push_back(boxShape.front());
  r.strokePolyline(boxShape.data(), static_cast<int>(boxShape.size()), 1.5f,
                   colors::kTapeTopBorder);

  // Small altitude-bug glyph at the left of the box (WT NXi .preselect-box
  // alerter bug), notch facing the readout. It rides on the cyan plate during an
  // alert, so it flips to black there to stay visible.
  const Color bugColor = alert.cyanBackground ? colors::kBlack : colors::kCyan;
  const float bugH = boxH * 0.5f;
  const float bugW = bugH * 0.5f;
  const float bugX = boxX + boxW * 0.06f;
  const float bugTop = boxY + (boxH - bugH) * 0.5f;
  const auto bp = [&](float nx, float ny) {
    return Point{bugX + nx * bugW, bugTop + ny * bugH};
  };
  const Point boxBug[8] = {bp(0.0f, 0.0f),    bp(1.0f, 0.0f),
                           bp(1.0f, 0.25f),   bp(0.5f, 0.4375f),
                           bp(0.5f, 0.5625f), bp(1.0f, 0.75f),
                           bp(1.0f, 1.0f),    bp(0.0f, 1.0f)};
  r.fillPolygon(boxBug, 8, bugColor);

  if (!alert.hideText) {
    // The value is right-aligned with the last two (tens) digits drawn smaller
    // than the leading hundreds (WT NXi: 20 px vs 24 px), inset from the right
    // edge to leave a small margin.
    const float rightX = boxX + boxW * 0.92f;
    const float midY = boxY + boxH * 0.5f;
    if (selected) {
      drawAltitudeNumber(r, rightX, midY, formatInt(selectedFt),
                         fontPx(wt::kSelectedAlt, displayH), kAltTrailingDigits,
                         kAltSelectedTensScale, TextAlign::Right, text);
    } else {
      r.fillText(rightX, midY, std::string(kSelectedAltDashes),
                 fontPx(wt::kSelectedAlt, displayH), TextAlign::Right, text);
    }
  }
}

void drawBaroSetting(Renderer& r, float tapeX, float tapeW, float instrumentBottom,
                     float scale, float displayH, float baroInHg, bool hpa,
                     bool flashOff) {
  // The BARO box matches the airspeed column's TAS box: a compact readout flush
  // with the bottom of the instrument column, with only the outer bottom corner
  // rounded. The Baro Transition Alert flashes the setting (handled by the
  // caller's blink phase via flashOff).
  const float boxH = kTapeBottomBoxHeightWt * scale;
  const float boxY = instrumentBottom - boxH;
  // Only the bottom-right (outer) corner is rounded, mirroring the airspeed
  // column's TAS box (which rounds its bottom-left) so the altimeter column
  // carries the same smooth outer-bottom edge. The inner and top corners stay
  // square so the box sits flush against the attitude window and the tape above.
  const float cornerR = fontPx(kTapeCornerRadiusWt, displayH);
  const std::vector<Point> boxPoly = {{tapeX, boxY},
                                      {tapeX + tapeW, boxY},
                                      {tapeX + tapeW, boxY + boxH},
                                      {tapeX, boxY + boxH}};
  const std::vector<float> boxRadii = {0.0f, 0.0f, cornerR, 0.0f};
  std::vector<Point> boxShape = roundPolygonCorners(boxPoly, boxRadii);
  r.fillPolygon(boxShape.data(), static_cast<int>(boxShape.size()),
                colors::kReadoutBox);
  if (flashOff) return;  // blink-off half of the Baro Transition Alert cycle

  const float numSize = fontPx(wt::kBaro, displayH);
  const float midY = boxY + boxH * 0.5f + numSize * kCapInkCenterNudge;
  if (isBaroStandard(baroInHg)) {
    r.fillText(tapeX + tapeW * 0.5f, midY, "STD BARO", numSize,
               TextAlign::Center, colors::kCyan);
    return;
  }

  char buf[16];
  const char* unit;
  if (hpa) {
    std::snprintf(buf, sizeof(buf), "%d",
                  static_cast<int>(std::lround(baroInHg * 33.8639f)));
    unit = "HPA";
  } else {
    std::snprintf(buf, sizeof(buf), "%.2f", baroInHg);
    unit = "IN";
  }
  const float unitSize = numSize * 0.72f;
  const float gap = numSize * 0.06f;
  const float numW = r.measureTextWidth(buf, numSize);
  const float unitW = r.measureTextWidth(unit, unitSize);
  const float startX = tapeX + tapeW * 0.5f - (numW + gap + unitW) * 0.5f;
  r.fillText(startX, midY, std::string(buf), numSize, TextAlign::Left,
             colors::kCyan);
  r.fillText(startX + numW + gap, midY, unit, unitSize, TextAlign::Left,
             colors::kCyan);
}

// Metric altitude overlay (PFD Opt > ALT Units > Meters): the Selected Altitude
// (meters) box sits just below the Selected Altitude box at the top, and the
// Indicated Altitude (meters) box sits just below the altitude pointer.
void drawMetricAltitude(Renderer& r, float tapeX, float tapeW,
                        float selBoxBottom, float cy, float readoutH,
                        float displayH, float altitudeFt, float selectedFt,
                        bool selected) {
  const float size = fontPx(wt::kSelectedAlt, displayH) * 0.8f;
  const float boxH = size * 1.4f;
  auto meters = [](float ft) { return ft * 0.3048f; };

  // Selected altitude (meters) directly below the selected-altitude box.
  const float selY = selBoxBottom;
  r.fillRect(tapeX, selY, tapeW, boxH, colors::kReadoutBox);
  r.fillText(tapeX + tapeW * 0.5f, selY + boxH * 0.5f,
             (selected ? formatInt(meters(selectedFt)) : std::string("---")) +
                 "M",
             size, TextAlign::Center, colors::kCyan);

  // Indicated altitude (meters) just below the indicated-altitude pointer box.
  const float indY = cy + readoutH * 0.5f + boxH * 0.2f;
  r.fillRect(tapeX, indY, tapeW, boxH, colors::kReadoutBox);
  const Point border[5] = {{tapeX, indY},
                           {tapeX + tapeW, indY},
                           {tapeX + tapeW, indY + boxH},
                           {tapeX, indY + boxH},
                           {tapeX, indY}};
  r.strokePolyline(border, 5, 1.5f, colors::kWhite);
  r.fillText(tapeX + tapeW * 0.5f, indY + boxH * 0.5f,
             formatInt(meters(altitudeFt)) + "M", size, TextAlign::Center,
             colors::kWhite);
}

// Barometric minimums (MDA/DH) alerting, set in the Timer/References window
// (Pilot's Guide, Minimum Descent Altitude/Decision Height Alerting). The BARO
// MIN box sits at the bottom left of the altimeter and a bug rides the tape at
// the minimums altitude. Both stage cyan -> white (within 100 ft) -> amber (at
// or below minimums).
void drawMinimums(Renderer& r, const Layout& L, const FlightData& d,
                  const SoftkeyController& ui, float displayH) {
  if (ui.minimumsMode() == MinimumsMode::Off) return;
  // In TEMP COMP the alert references the temperature-corrected minimum.
  const float minsFt = ui.effectiveMinimumsFt();
  const char* minsLabel =
      ui.minimumsMode() == MinimumsMode::Temp ? "TEMP" : "BARO";
  const float aboveFt = d.altitudeFt - minsFt;

  const Color stage = (aboveFt <= 0.0f)     ? colors::kBandYellow
                      : (aboveFt <= 100.0f) ? colors::kWhite
                                            : colors::kCyan;

  // The box appears once the aircraft descends to within 2500 ft of the
  // MDA/DH setting.
  if (aboveFt <= 2500.0f) {
    const float stripBottom = L.stripTop + L.stripH;
    const float boxH = (L.altTop + L.altH) - stripBottom;
    const float boxW = L.altW * 0.95f;
    const float boxX = L.altX - boxW - 8.0f * L.sx;
    const float boxY = stripBottom;
    r.fillRect(boxX, boxY, boxW, boxH, colors::kReadoutBox);
    const Point outline[5] = {{boxX, boxY},
                              {boxX + boxW, boxY},
                              {boxX + boxW, boxY + boxH},
                              {boxX, boxY + boxH},
                              {boxX, boxY}};
    r.strokePolyline(outline, 5, 1.5f, colors::kPanelBorder);

    const float labelSize = fontPx(wt::kInfoLabel, displayH) * 0.8f;
    const float cx = boxX + boxW * 0.30f;
    r.fillText(cx, boxY + boxH * 0.30f, minsLabel, labelSize, TextAlign::Center,
               stage);
    r.fillText(cx, boxY + boxH * 0.72f, "MIN", labelSize, TextAlign::Center,
               stage);
    r.fillText(boxX + boxW * 0.95f, boxY + boxH * 0.5f, formatInt(minsFt),
               fontPx(wt::kBaro, displayH), TextAlign::Right, stage);
  }

  // Minimums bug on the tape (left edge, opening toward the scale) once the
  // setting is within the viewable range.
  const float ppu = L.stripH / kAltitudeViewableFeet;
  const float bugY = L.attCy - (minsFt - d.altitudeFt) * ppu;
  if (bugY < L.stripTop || bugY > L.stripTop + L.stripH) return;
  const float armLen = L.altW * 0.14f;
  const float halfH = displayH * 0.011f;
  const float lineW = std::max(2.0f, displayH * 0.004f);
  r.save();
  r.clip(L.altX, L.stripTop, L.altW, L.stripH);
  r.strokeLine(L.altX + lineW * 0.5f, bugY - halfH, L.altX + lineW * 0.5f,
               bugY + halfH, lineW * 1.6f, stage);
  r.strokeLine(L.altX, bugY - halfH, L.altX + armLen, bugY - halfH, lineW,
               stage);
  r.strokeLine(L.altX, bugY + halfH, L.altX + armLen, bugY + halfH, lineW,
               stage);
  r.restore();
}

void drawAltitudeReadout(Renderer& r, float x, float y, float w, float h,
                         float altitudeFt, float textSize, float scale) {
  const float drumSize = textSize * 0.78f;
  const float drumW = kAltReadoutDrumWidthPx * scale;
  const float caretHalfH = kAltReadoutCaretHalfHeightPx * scale;
  const TapeReadoutShape shape = buildTapeReadoutShape(
      x, y, w, h, drumW, NotchSide::Left, caretHalfH,
      kAltReadoutCaretDepthPx * scale, kAltReadoutLeadHeightFraction,
      kReadoutCornerRadiusPx * scale, kReadoutDrumCornerRadiusPx * scale);

  const float drumPad = drumSize * kReadoutDrumPadFraction;
  const float drumAnchorX = shape.drumRight - drumPad;
  const float drumClipTop = y + 2.0f;
  const float drumClipH = h - 4.0f;
  // One row spacing must fit within half the drum clip so rolling neighbors peek
  // without escaping the window (h * 0.92 was too large for the 46 px box).
  const float rowSpacing =
      std::min(drumSize * 1.08f, drumClipH * 0.45f);

  const long snapped = std::lround(altitudeFt / 20.0f) * 20L;
  const long absSnap = std::labs(snapped);
  const long leading = absSnap / 100;
  const long tensCenter = absSnap % 100;
  const float residual =
      (altitudeFt - static_cast<float>(snapped)) / 20.0f;

  char centerTens[4];
  std::snprintf(centerTens, sizeof(centerTens), "%02ld",
                ((tensCenter % 100) + 100) % 100);
  const float activeTensW = r.measureTextWidth(centerTens, drumSize);
  const float leadAnchorX = tapeReadoutLeadAnchorX(
      shape.drumX, drumAnchorX, activeTensW, kReadoutColumnGapPx * scale);

  std::vector<Point> poly =
      roundPolygonCorners(shape.silhouette, shape.radii);
  r.fillPolygon(poly.data(), static_cast<int>(poly.size()),
                colors::kReadoutBox);

  const std::string headStr =
      snapped < 0 ? std::string("-") + formatInt(static_cast<float>(leading))
                  : formatInt(static_cast<float>(leading));
  const bool showHead = snapped < 0 || leading > 0;
  const int leadColCount =
      showHead ? std::max(1, static_cast<int>(headStr.size())) : 0;
  const float leadColW = kAltReadoutDigitColumnWidthPx * scale;
  const float leadColPitch = kAltReadoutDigitPitchPx * scale;
  const float leadDrumGap = kAltReadoutLeadDrumGapPx * scale;
  const float leadH = shape.leadBot - shape.leadTop;
  std::vector<ReadoutColumnRect> leadCols;
  if (leadColCount > 0) {
    layoutReadoutDigitColumnsBeforeDrum(
        shape.drumX, leadDrumGap, shape.leadTop, leadH, leadColCount,
        leadColPitch, leadColW, leadCols);
    for (const ReadoutColumnRect& col : leadCols) {
      drawReadoutDigitDialShading(r, col.x, shape.leadTop, col.w, leadH,
                                  colors::kReadoutBox);
    }
  }
  // Full-height tens drum column (NXi alt-tens-scroller).
  drawReadoutDigitDialShading(r, shape.drumX, y, shape.drumW, h,
                              colors::kReadoutBox);

  poly.push_back(poly.front());
  r.strokePolyline(poly.data(), static_cast<int>(poly.size()),
                   kReadoutOutlineWidthPx * scale, colors::kWhite);

  const TapeReadoutDigitMidY midY = tapeReadoutDigitMidY(
      r, shape.leadTop, shape.leadBot, drumClipTop, drumClipH, leadAnchorX,
      headStr, textSize, drumAnchorX, centerTens, drumSize, false);

  if (showHead) {
    const float leadPad = scale;
    for (int i = 0; i < leadColCount; ++i) {
      const ReadoutColumnRect& col = leadCols[static_cast<size_t>(i)];
      const std::string digit(1, headStr[static_cast<size_t>(i)]);
      const bool lastLead = i == leadColCount - 1;
      // Right-align toward the next column; keep the drum-adjacent digit
      // centered so it does not touch the rolling tens (NXi: 1 px before 70 px).
      const float anchorX =
          lastLead ? col.x + col.w * 0.5f : col.x + col.w - leadPad;
      const TextAlign align = lastLead ? TextAlign::Center : TextAlign::Right;
      r.fillText(anchorX,
                 tapeReadoutLeadMidY(r, shape.leadTop, shape.leadBot, anchorX,
                                     digit, textSize, kAltReadoutLeadInkNudge,
                                     0.0f, align),
                 digit, textSize, align, colors::kWhite);
    }
  }

  r.save();
  r.clip(shape.drumX, drumClipTop, shape.drumW, drumClipH);
  for (int j = -2; j <= 2; ++j) {
    const long m = tensCenter + static_cast<long>(j) * 20L;
    const long disp = ((m % 100) + 100) % 100;
    const float ty = midY.drumMidY - static_cast<float>(j) * rowSpacing +
                     residual * rowSpacing;
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02ld", disp);
    r.fillText(drumAnchorX, ty, std::string(buf), drumSize, TextAlign::Right,
               colors::kWhite);
  }
  r.restore();
  // Mask is drawn on top of the scrolling digits (NXi alt-tens-overlay).
  const float maskPad = scale;
  drawReadoutDrumScrollerMask(r, shape.drumX - maskPad, y, shape.drumW + 2.0f * maskPad,
                              h, colors::kReadoutBox);
}

}  // namespace

void drawAltimeter(Renderer& r, const Layout& L, const FlightData& d,
                   const SoftkeyController& ui, float h) {
  // Air data computer failure: the altitude tape, readout, baro, and selected-
  // altitude column are replaced by a red X.
  if (!d.altitudeValid) {
    // Altimeter tape sits right of the attitude; its ticks line the inner
    // (left) edge and are retained under the failure X (NXi Fig 9-2).
    drawFailureX(r, L.altX, L.altTop, L.altW, L.altH, "", h, FailTicks::LeftEdge,
                 kAltTapeMinorTickFraction, kAltTapeMajorTickFraction);
    return;
  }

  // The altimeter (tape numbers, readout, selected altitude, baro) renders in
  // the same semibold face as the airspeed instrument so the two tape columns
  // share one heavier weight, matching the real G1000 NXi.
  const FontScope altimeterFont(r, FontFace::DejaVuSemiBold);

  // The selected-altitude box sits flush on top of the tape (covering its top
  // corner), so unlike the airspeed tape the altimeter keeps a square top.
  drawVerticalTape(r, L.altX, L.altW, L.stripTop, L.stripH, L.attCy, h,
                   d.altitudeFt, kAltitudeViewableFeet, kAltitudeMajorFeet,
                   kAltitudeMinorFeet, kAltitudeMinFeet, true,
                   kAltTapeMinorTickFraction, kAltTapeMajorTickFraction,
                   /*topOuterCornerRadius=*/0.0f, /*tickInset=*/0.0f,
                   kAltTrailingDigits, L.tapeBgH);
  drawTrendVector(r, L.altX, L.stripTop, L.stripH, L.attCy, h,
                  L.stripH / kAltitudeViewableFeet, d.altitudeTrendFt);

  const float readoutH = kAltReadoutHeightPx * L.sy;
  const float readoutSize = fontPx(wt::kReadoutAlt, h);
  const float readoutY = kAltReadoutTopPx * L.sy;
  const float readoutX = L.altX + kAltReadoutBodyLeftPx * L.sx;
  const float readoutW = L.altW - kAltReadoutBodyLeftPx * L.sx -
                         kAltReadoutBodyRightInsetPx * L.sx;
  drawMinimums(r, L, d, ui, h);
  drawAltitudeReadout(r, readoutX, readoutY, readoutW, readoutH, d.altitudeFt,
                      readoutSize, L.s);
  drawSelectedAltitude(r, L.altX, L.altW, L.altTop, L.stripTop, L.stripH,
                       L.attCy, h, d.altitudeFt, d.selectedAltitudeFt,
                       ui.selectedAltStyle());

  const bool selected = std::fabs(d.selectedAltitudeFt) > kAltSelectedEpsilonFt;
  if (ui.displayToggle(DisplayToggle::AltMeters)) {
    drawMetricAltitude(r, L.altX, L.altW, L.stripTop, readoutY + readoutH * 0.5f,
                       readoutH, h, d.altitudeFt, d.selectedAltitudeFt, selected);
  }

  const bool hpa = ui.displayToggle(DisplayToggle::BaroHpa);
  const bool flashOff = d.baroTransitionAlert && !ui.blinkOn();
  drawBaroSetting(r, L.altX, L.altW, L.altTop + L.altH, L.s, h,
                  d.baroSettingInHg, hpa, flashOff);
}

}  // namespace avionics::pfd
