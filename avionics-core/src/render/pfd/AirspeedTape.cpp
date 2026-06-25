#include "render/pfd/PfdInternal.h"

#include <algorithm>
#include <vector>

namespace avionics::pfd {
namespace {

void drawAirspeedColorBands(Renderer& r, float tapeX, float tapeW,
                            float stripTop, float stripH, float cy,
                            float value) {
  const float ppu = stripH / kAirspeedViewableKnots;
  const float innerX = tapeX + tapeW;
  const float bandW = tapeW * kAirspeedBandWidthFraction;
  const float bandX = innerX - bandW;
  const float whiteW = tapeW * 0.06f;
  const float whiteX = bandX - whiteW;

  auto yOf = [&](float kt) { return cy - (kt - value) * ppu; };
  auto fillBand = [&](float x, float bw, float lo, float hi, const Color& c) {
    const float yHi = yOf(hi);
    const float yLo = yOf(lo);
    if (yLo > yHi) r.fillRect(x, yHi, bw, yLo - yHi, c);
  };

  r.save();
  r.clip(tapeX, stripTop, tapeW, stripH);
  fillBand(bandX, bandW, kVs1Kt, kVnoKt, colors::kBandGreen);
  fillBand(bandX, bandW, kVnoKt, kVneKt, colors::kBandYellow);
  fillBand(whiteX, whiteW, kVsoKt, kVfeKt, colors::kWhite);

  const float vsoY = yOf(kVsoKt);
  r.fillRect(bandX, vsoY, bandW, stripH, colors::kBandRed);

  // High-speed warning range above VNE: red/white "barber pole" hatching, per
  // the G1000 NXi. Drawn as alternating red diagonal parallelograms over white.
  const float vneY = yOf(kVneKt);
  const float poleTop = stripTop;
  if (vneY > poleTop) {
    r.save();
    r.clip(bandX, poleTop, bandW, vneY - poleTop);
    r.fillRect(bandX, poleTop, bandW, vneY - poleTop, colors::kWhite);
    const float period = bandW * 1.1f;
    for (float yy = poleTop - bandW; yy < vneY + period; yy += period) {
      const Point stripe[4] = {{bandX, yy},
                               {bandX + bandW, yy - bandW},
                               {bandX + bandW, yy - bandW + period * 0.5f},
                               {bandX, yy + period * 0.5f}};
      r.fillPolygon(stripe, 4, colors::kBandRed);
    }
    r.restore();
  }
  r.strokeLine(bandX, vneY, innerX, vneY, 3.0f, colors::kBandRed);
  r.restore();
}

void drawVspeedBugs(Renderer& r, float tapeX, float tapeW, float stripTop,
                    float stripH, float cy, float displayH, float airspeed,
                    const SoftkeyController& ui) {
  // V-speed reference bugs (GLIDE/VR/VX/VY) sit just to the RIGHT of the tape
  // border (G1000 Pilot's Guide airspeed figures): a black tab with a cyan
  // letter protrudes into the gap toward the attitude window, with the cyan
  // accent on the tape-facing edge.
  const float ppu = stripH / kAirspeedViewableKnots;
  const float outerX = tapeX + tapeW;
  const float labelSize = fontPx(wt::kTapeLabel, displayH) * 0.85f;
  const float bugW = tapeW * kVspeedBugWidthFraction;
  const float bugH = labelSize * 1.25f;

  r.save();
  r.clip(tapeX, stripTop, tapeW + bugW, stripH);
  for (int i = 0; i < kVSpeedRefCount; ++i) {
    if (!ui.vspeedEnabled(static_cast<VspeedRef>(i))) continue;
    const VSpeedRef& v = kVSpeedRefs[i];
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(i));
    const float y = cy - (vKt - airspeed) * ppu;
    if (y < stripTop || y > stripTop + stripH) continue;
    const float bx = outerX;
    const float by = y - bugH * 0.5f;
    r.fillRect(bx, by, bugW, bugH, colors::kReadoutBox);
    const Point edge[2] = {{outerX, by}, {outerX, by + bugH}};
    r.strokePolyline(edge, 2, 2.5f, colors::kCyan);
    r.fillText(bx + bugW * 0.5f, y, v.bugLabel, labelSize, TextAlign::Center,
               colors::kCyan);
  }
  r.restore();
}

// Selected Airspeed box (above the tape) and cyan tape bug while FLC is active
// (G1000 NXi Pilot's Guide, Airspeed Indicator / Flight Level Change Mode).
void drawSelectedAirspeed(Renderer& r, float tapeX, float tapeW, float tapeTop,
                          float stripTop, float stripH, float cy,
                          float displayH, float airspeedKts, float selectedKts) {
  const float boxH = stripTop - tapeTop;
  const float boxW = tapeW;
  const float boxX = tapeX;
  const float boxY = tapeTop;
  const bool selected = selectedKts > kAsiSelectedEpsilonKt;

  // Only the top-left (outer) corner is rounded, mirroring the selected-
  // altitude box on the opposite tape column.
  const float cornerR = fontPx(kTapeCornerRadiusWt, displayH);
  const std::vector<Point> boxPoly = {{boxX, boxY},
                                      {boxX + boxW, boxY},
                                      {boxX + boxW, boxY + boxH},
                                      {boxX, boxY + boxH}};
  const std::vector<float> boxRadii = {cornerR, 0.0f, 0.0f, 0.0f};
  std::vector<Point> boxShape = roundPolygonCorners(boxPoly, boxRadii);
  r.fillPolygon(boxShape.data(), static_cast<int>(boxShape.size()),
                colors::kReadoutBox);
  boxShape.push_back(boxShape.front());
  r.strokePolyline(boxShape.data(), static_cast<int>(boxShape.size()), 1.5f,
                   colors::kTapeTopBorder);

  // Small airspeed-bug glyph at the right of the box (inner edge, toward the
  // attitude window), notch facing the readout.
  const float glyphH = boxH * 0.5f;
  const float glyphW = glyphH * 0.5f;
  const float glyphX = boxX + boxW * 0.94f - glyphW;
  const float glyphTop = boxY + (boxH - glyphH) * 0.5f;
  const auto gp = [&](float nx, float ny) {
    return Point{glyphX + nx * glyphW, glyphTop + ny * glyphH};
  };
  const Point boxBug[8] = {gp(0.0f, 0.0f),    gp(0.0f, 0.25f),
                           gp(0.5f, 0.4375f), gp(0.5f, 0.5625f),
                           gp(0.0f, 0.75f),   gp(0.0f, 1.0f),
                           gp(1.0f, 1.0f),    gp(1.0f, 0.0f)};
  r.fillPolygon(boxBug, 8, colors::kCyan);

  const float leftX = boxX + boxW * 0.08f;
  const float midY = boxY + boxH * 0.5f;
  const float textSize = fontPx(wt::kSelectedAlt, displayH);
  if (selected) {
    r.fillText(leftX, midY, formatInt(selectedKts), textSize, TextAlign::Left,
               colors::kCyan);
  } else {
    r.fillText(leftX, midY, std::string(kSelectedAltDashes), textSize,
               TextAlign::Left, colors::kCyan);
  }

  if (!selected) {
    return;
  }

  // Cyan bug on the inner (right) edge of the scrolling tape, pointing at the
  // selected speed on the scale (mirror of the selected-altitude tape bug).
  const float ppu = stripH / kAirspeedViewableKnots;
  float bugY = cy - (selectedKts - airspeedKts) * ppu;
  bugY = std::max(stripTop, std::min(stripTop + stripH, bugY));
  const float bw = tapeW * 0.16f;
  const float bh = displayH * 0.020f;
  const float innerX = tapeX + tapeW;
  const float bx = innerX - bw;

  r.save();
  r.clip(tapeX, stripTop, tapeW, stripH);
  const Point bug[5] = {{bx, bugY - bh},
                        {innerX, bugY - bh},
                        {innerX - bw * 0.5f, bugY},
                        {innerX, bugY + bh},
                        {bx, bugY + bh}};
  r.fillPolygon(bug, 5, colors::kCyan);
  r.restore();
}

// Airspeed pointer box with a right-pointing notch and a rolling ones digit
// (the last digit scrolls like a drum), matching the real G1000 NXi.
void drawAirspeedReadout(Renderer& r, float x, float y, float w, float h,
                         float value, float textSize, const Color& boxColor,
                         const Color& textColor, float scale) {
  if (value < 0.0f) value = 0.0f;
  const long snapped = std::lround(value);
  const long onesCenter = ((snapped % 10) + 10) % 10;
  const float residual = value - static_cast<float>(snapped);

  const float drumW = kAsiReadoutDrumWidthPx * scale;
  const TapeReadoutShape shape = buildTapeReadoutShape(
      x, y, w, h, drumW, NotchSide::Right,
      kAsiReadoutCaretHalfHeightPx * scale, kAsiReadoutCaretDepthPx * scale,
      kAsiReadoutLeadHeightFraction, kReadoutCornerRadiusPx * scale,
      kReadoutDrumCornerRadiusPx * scale);

  const float drumPad = textSize * kReadoutDrumPadFraction;
  const float drumAnchorX = shape.drumRight - drumPad;
  const float drumClipTop = y + 2.0f;
  const float drumClipH = h - 4.0f;
  const float rowSpacing =
      std::min(textSize * 1.08f, drumClipH * 0.45f);

  char centerOnes[2];
  std::snprintf(centerOnes, sizeof(centerOnes), "%ld", onesCenter);
  const float activeOnesW = r.measureTextWidth(centerOnes, textSize);
  const float leadAnchorX = tapeReadoutLeadAnchorX(
      shape.drumX, drumAnchorX, activeOnesW, kReadoutColumnGapPx * scale);

  std::vector<Point> poly =
      roundPolygonCorners(shape.silhouette, shape.radii);
  r.fillPolygon(poly.data(), static_cast<int>(poly.size()), boxColor);

  const float digitLeft = x + kReadoutDigitAreaInsetPx * scale;
  const float digitRight = shape.drumRight;
  const float colGap = kReadoutDigitColumnGapPx * scale;
  const float leadH = shape.leadBot - shape.leadTop;
  std::vector<ReadoutColumnRect> digitCols;
  layoutReadoutDigitColumns(digitLeft, digitRight, shape.leadTop, leadH,
                            kAsiReadoutDigitColumnCount, colGap, digitCols);

  for (int i = 0; i < 2 && i < static_cast<int>(digitCols.size()); ++i) {
    const ReadoutColumnRect& col = digitCols[static_cast<size_t>(i)];
    drawReadoutDigitDialShading(r, col.x, shape.leadTop, col.w, leadH, boxColor);
  }
  // Ones digit column spans the full box height (NXi airspeed-ias-box-ones).
  if (digitCols.size() >= 3) {
    const ReadoutColumnRect& col = digitCols[static_cast<size_t>(2)];
    drawReadoutDigitDialShading(r, col.x, y, col.w, h, boxColor);
  }

  poly.push_back(poly.front());
  r.strokePolyline(poly.data(), static_cast<int>(poly.size()),
                   kReadoutOutlineWidthPx * scale, colors::kWhite);

  // Below the tape minimum (20 kt) the NXi readout shows three dashes instead
  // of digits (Garmin DigitScroller NaN / off-scale behavior). All three sit
  // on the lead-digit row — not the ones-drum band.
  if (value < kAirspeedMinKnots) {
    const float dashY = tapeReadoutLeadMidY(
        r, shape.leadTop, shape.leadBot,
        digitCols[1].x + digitCols[1].w * 0.5f, std::string("-"), textSize,
        kAsiReadoutLeadInkNudge, -scale, TextAlign::Center);
    for (int i = 0; i < kAsiReadoutOffScaleDashCount &&
                    i < static_cast<int>(digitCols.size());
         ++i) {
      const ReadoutColumnRect& col = digitCols[static_cast<size_t>(i)];
      const float cx = col.x + col.w * 0.5f;
      r.fillText(cx, dashY, std::string("-"), textSize, TextAlign::Center,
                 textColor);
    }
    const float maskPad = scale;
    drawReadoutDrumScrollerMask(r, shape.drumX - maskPad, y,
                                shape.drumW + 2.0f * maskPad, h, boxColor);
    return;
  }

  const long leadValue = snapped / 10;
  const bool showHead = leadValue > 0;
  const long hundreds = snapped / 100;
  const long tens = (snapped / 10) % 10;
  const TapeReadoutDigitMidY midY = tapeReadoutDigitMidY(
      r, shape.leadTop, shape.leadBot, drumClipTop, drumClipH, leadAnchorX,
      std::string(), textSize, drumAnchorX, centerOnes, textSize, false);

  auto drawLeadDigit = [&](int colIdx, char ch) {
    if (ch == '\0' || colIdx >= static_cast<int>(digitCols.size())) {
      return;
    }
    const ReadoutColumnRect& col = digitCols[static_cast<size_t>(colIdx)];
    const std::string s(1, ch);
    const float cx = col.x + col.w * 0.5f;
    r.fillText(cx,
               tapeReadoutLeadMidY(r, shape.leadTop, shape.leadBot, cx, s,
                                   textSize, kAsiReadoutLeadInkNudge, -scale,
                                   TextAlign::Center),
               s, textSize, TextAlign::Center, textColor);
  };

  if (showHead) {
    if (hundreds > 0) {
      drawLeadDigit(0, static_cast<char>('0' + hundreds));
    }
    if (snapped >= 10 || hundreds > 0) {
      drawLeadDigit(1, static_cast<char>('0' + tens));
    } else if (snapped > 0) {
      drawLeadDigit(1, static_cast<char>('0' + onesCenter));
    }
  }

  r.save();
  r.clip(shape.drumX, drumClipTop, shape.drumW, drumClipH);
  for (int j = -1; j <= 1; ++j) {
    const long disp = (((onesCenter + j) % 10) + 10) % 10;
    const float ty = midY.drumMidY - static_cast<float>(j) * rowSpacing +
                     residual * rowSpacing;
    char buf[2];
    std::snprintf(buf, sizeof(buf), "%ld", disp);
    r.fillText(drumAnchorX, ty, std::string(buf), textSize, TextAlign::Right,
               textColor);
  }
  r.restore();
  const float maskPad = scale;
  drawReadoutDrumScrollerMask(r, shape.drumX - maskPad, y,
                              shape.drumW + 2.0f * maskPad, h, boxColor);
}

// Below 20 kt the airspeed scale bottoms out, so the enabled V-speed reference
// bugs and their values are listed at the bottom of the tape, ordered highest
// to lowest (G1000 NXi Pilot's Guide, Airspeed Indicator).
void drawVspeedList(Renderer& r, float tapeX, float tapeW, float stripTop,
                    float stripH, float displayH, const SoftkeyController& ui) {
  // Only the ENABLED reference bugs are listed (Pilot's Guide, Airspeed
  // Indicator), ordered highest to lowest.
  std::vector<int> order;
  for (int i = 0; i < kVSpeedRefCount; ++i) {
    if (ui.vspeedEnabled(static_cast<VspeedRef>(i))) order.push_back(i);
  }
  std::sort(order.begin(), order.end(), [&ui](int a, int b) {
    return ui.vspeedValueKt(static_cast<VspeedRef>(a)) >
           ui.vspeedValueKt(static_cast<VspeedRef>(b));
  });

  const float labelSize = fontPx(wt::kTapeLabel, displayH) * 0.9f;
  const float rowH = labelSize * 1.5f;
  const float bugW = tapeW * kVspeedBugWidthFraction;
  const float outerX = tapeX + tapeW;
  const int rows = static_cast<int>(order.size());
  float y = stripTop + stripH - rowH * (rows + 0.5f);
  for (int k = 0; k < rows; ++k) {
    const VSpeedRef& v = kVSpeedRefs[order[k]];
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(order[k]));
    const float rowCy = y + rowH * 0.5f;
    r.fillText(outerX + bugW * 0.15f, rowCy, v.bugLabel, labelSize,
               TextAlign::Left, colors::kCyan);
    r.fillText(outerX + bugW * 1.05f, rowCy, formatInt(vKt), labelSize,
               TextAlign::Left, colors::kWhite);
    y += rowH;
  }
}

}  // namespace

void drawAirspeedTape(Renderer& r, const Layout& L, const FlightData& d,
                      const SoftkeyController& ui, float h) {
  // Air data computer failure: the airspeed tape and TAS (both ADC-sourced)
  // are replaced by a red X.
  if (!d.airspeedValid) {
    // Airspeed tape sits left of the attitude; its ticks line the inner (right)
    // edge and are retained under the failure X (NXi Fig 9-2).
    drawFailureX(r, L.asiX, L.asiTop, L.asiW, L.asiH, "", h, FailTicks::RightEdge,
                 kAsiTapeMinorTickFraction, kAsiTapeMajorTickFraction);
    return;
  }

  // The airspeed instrument (tape numbers, IAS readout, GS/TAS) renders in a
  // semibold face to match the heavier weight of the real G1000 NXi.
  const FontScope airspeedFont(r, FontFace::DejaVuSemiBold);

  // The NXi airspeed tape has a 10 px rounded top-left corner when FLC is off;
  // when FLC is active the selected-airspeed box above the tape carries it.
  drawVerticalTape(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy, h,
                   d.airspeedKts, kAirspeedViewableKnots, kAirspeedMajorKnots,
                   kAirspeedMinorKnots, kAirspeedMinKnots, false,
                   kAsiTapeMinorTickFraction, kAsiTapeMajorTickFraction,
                   kTapeCornerRadiusWt * L.s,
                   L.asiW * kAirspeedBandWidthFraction, 0, L.tapeBgH);
  drawAirspeedColorBands(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy,
                         d.airspeedKts);
  drawVspeedBugs(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy, h,
                 d.airspeedKts, ui);
  if (d.airspeedKts < kAirspeedMinKnots) {
    drawVspeedList(r, L.asiX, L.asiW, L.stripTop, L.stripH, h, ui);
  }
  drawTrendVector(r, L.asiX + L.asiW, L.stripTop, L.stripH, L.attCy, h,
                  L.stripH / kAirspeedViewableKnots, d.airspeedTrendKts);

  const float readoutH = kAsiReadoutHeightPx * L.sy;
  const float readoutSize = fontPx(wt::kReadoutAlt, h);
  const float readoutW =
      L.asiW -
      (kAsiReadoutLeftInsetPx + kAsiReadoutBodyRightInsetPx) * L.sx;
  const float readoutX = L.asiX + kAsiReadoutLeftInsetPx * L.sx;
  const float readoutY = kAsiReadoutTopPx * L.sy;

  // The pointer is black until VNE, then red. If the trend vector crosses VNE
  // (but current speed has not), the digits turn amber as an early warning.
  const bool overVne = d.airspeedKts >= kVneKt;
  const bool trendOverVne = (d.airspeedKts + d.airspeedTrendKts) >= kVneKt;
  const Color boxColor = overVne ? colors::kBandRed : colors::kReadoutBox;
  const Color textColor =
      (!overVne && trendOverVne) ? colors::kBandYellow : colors::kWhite;
  drawAirspeedReadout(r, readoutX, readoutY, readoutW, readoutH,
                      d.airspeedKts, readoutSize, boxColor, textColor, L.s);
  if (d.selectedAirspeedValid) {
    drawSelectedAirspeed(r, L.asiX, L.asiW, L.asiTop, L.stripTop, L.stripH,
                         L.attCy, h, d.airspeedKts, d.selectedAirspeedKts);
  }

  // When Mach is high enough, the bottom tape-width box shows Mach instead of
  // TAS (G1000 NXi TBM / high-speed installs). Speed of sound from OAT.
  const float soundKts = 38.967854f * std::sqrt(d.oatCelsius + 273.15f);
  const float mach = soundKts > 1.0f ? d.tasKts / soundKts : 0.0f;
  const bool showMach = mach >= kMachDisplayThreshold;

  // Ground speed and true airspeed sit in two black boxes flush with the bottom
  // of the airspeed instrument. The TAS box is exactly the tape width and its
  // bottom-left corner is rounded to mirror the tape's rounded top-left, giving
  // the column a continuous rounded-left edge (NXi). The GS box sits to its left
  // (square, sized to content) and extends left of the tape.
  const float labelSize = fontPx(wt::kInfoLabel, h);
  const float valueSize = fontPx(wt::kInfoValue, h);
  const float boxH = kTapeBottomBoxHeightWt * L.s;
  // Bottom edge flush with the airspeed instrument column (WT bottom: 0).
  const float boxY = L.asiTop + L.asiH - boxH;
  const float gap = labelSize * 0.25f;
  const float padX = labelSize * 0.5f;
  const float boxGap = labelSize * 0.45f;

  auto runWidth = [&](const char* label, const std::string& value,
                      const char* unitSuffix) {
    float w = r.measureTextWidth(label, labelSize) + gap +
              r.measureTextWidth(value, valueSize) + gap * 0.5f;
    if (unitSuffix != nullptr && unitSuffix[0] != '\0') {
      w += r.measureTextWidth(unitSuffix, labelSize);
    }
    return w;
  };
  // Corner radii order matches the rectangle vertices below: top-left,
  // top-right, bottom-right, bottom-left.
  auto drawSpeedBox = [&](float boxX, float boxW,
                          const std::vector<float>& radii, float scale,
                          const char* label, const std::string& value,
                          const char* unitSuffix) {
    const std::vector<Point> rect = {{boxX, boxY},
                                     {boxX + boxW, boxY},
                                     {boxX + boxW, boxY + boxH},
                                     {boxX, boxY + boxH}};
    const std::vector<Point> shape = roundPolygonCorners(rect, radii);
    r.fillPolygon(shape.data(), static_cast<int>(shape.size()),
                  colors::kReadoutBox);
    const float boxCy = boxY + boxH * 0.5f;
    // Both boxes share one scale (driven by the tape-width TAS box) so GS and
    // TAS always read at the same font size. The scale applies to both font
    // sizes and the inter-element gaps.
    const float lSize = labelSize * scale;
    const float vSize = valueSize * scale;
    const float g = gap * scale;
    float tx = boxX + boxW * 0.5f -
               runWidth(label, value, unitSuffix) * scale * 0.5f;
    tx = putText(r, tx, boxCy, label, lSize, colors::kLabelText, g / lSize);
    tx = putText(r, tx, boxCy, value, vSize, colors::kWhite,
                 unitSuffix != nullptr && unitSuffix[0] != '\0'
                     ? (g * 0.5f) / vSize
                     : 0.0f);
    if (unitSuffix != nullptr && unitSuffix[0] != '\0') {
      putText(r, tx, boxCy, unitSuffix, lSize, colors::kLabelText);
    }
  };

  std::string tasBoxValue;
  const char* tasBoxLabel = "TAS";
  const char* tasBoxUnit = "KT";
  if (showMach) {
    tasBoxLabel = "M";
    tasBoxUnit = nullptr;
    char mbuf[8];
    // NXi Mach format: "M .477" (leading zero omitted after the decimal).
    std::snprintf(mbuf, sizeof(mbuf), ".%03d",
                  static_cast<int>(std::lround(mach * 1000.0f)));
    tasBoxValue = mbuf;
  } else {
    tasBoxValue = formatInt(d.tasKts);
  }

  const std::string gsValue = formatInt(d.groundSpeedKts);
  const float tasX = L.asiX;
  const float tasW = L.asiW;  // exactly the tape width
  // The TAS/Mach box is locked to the tape width, so its content may need to
  // shrink to fit. Size the scale against the widest possible layout so the
  // GS/TAS font size stays constant with speed.
  const float tasRunMax =
      std::max(runWidth("TAS", "999", "KT"), runWidth("M", ".999", nullptr));
  const float tasAvail = tasW - padX * 2.0f;
  const float speedScale =
      (tasRunMax > tasAvail && tasRunMax > 0.0f) ? tasAvail / tasRunMax : 1.0f;
  const float gsW = runWidth("GS", gsValue, "KT") * speedScale + padX * 2.0f;
  const float gsX = tasX - boxGap - gsW;
  const float cornerR = kTapeCornerRadiusWt * L.s;
  // GS is a free-floating box, so round all four corners like the rest of the
  // readouts; the TAS box only rounds its bottom-left to mirror the tape.
  drawSpeedBox(gsX, gsW, {cornerR, cornerR, cornerR, cornerR}, speedScale, "GS",
               gsValue, "KT");
  drawSpeedBox(tasX, tasW, {0.0f, 0.0f, 0.0f, cornerR}, speedScale,
               tasBoxLabel, tasBoxValue, tasBoxUnit);
}

}  // namespace avionics::pfd
