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
  // V-speed reference bugs (GLIDE/VR/VX/VY) sit along the RIGHT (inner) edge of
  // the airspeed scale as black bugs with cyan letters, per the G1000 NXi.
  // Each bug is shown only while enabled in the Timer/References window.
  const float ppu = stripH / kAirspeedViewableKnots;
  const float innerX = tapeX + tapeW;
  const float labelSize = fontPx(wt::kTapeLabel, displayH) * 0.85f;
  const float bugW = tapeW * 0.20f;
  const float bugH = labelSize * 1.25f;

  r.save();
  r.clip(tapeX, stripTop, tapeW, stripH);
  for (int i = 0; i < kVSpeedRefCount; ++i) {
    if (!ui.vspeedEnabled(static_cast<VspeedRef>(i))) continue;
    const VSpeedRef& v = kVSpeedRefs[i];
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(i));
    const float y = cy - (vKt - airspeed) * ppu;
    if (y < stripTop || y > stripTop + stripH) continue;
    const float bx = innerX - bugW;
    const float by = y - bugH * 0.5f;
    r.fillRect(bx, by, bugW, bugH, colors::kReadoutBox);
    const Point edge[2] = {{innerX, by}, {innerX, by + bugH}};
    r.strokePolyline(edge, 2, 2.5f, colors::kCyan);
    r.fillText(bx + bugW * 0.5f, y, v.bugLabel, labelSize, TextAlign::Center,
               colors::kCyan);
  }
  r.restore();
}

// Airspeed pointer box with a right-pointing notch and a rolling ones digit
// (the last digit scrolls like a drum), matching the real G1000 NXi.
void drawAirspeedReadout(Renderer& r, float x, float y, float w, float h,
                         float value, float textSize, const Color& boxColor,
                         const Color& textColor) {
  const float midY = y + h * 0.5f;
  const float notchHalfH = h * 0.20f;
  const float notchDepth = h * 0.15f;
  const float rightX = x + w;

  if (value < 0.0f) value = 0.0f;
  const long snapped = std::lround(value);
  const long onesCenter = ((snapped % 10) + 10) % 10;
  const float residual = value - static_cast<float>(snapped);

  // Layout (NXi/Garmin), mirroring the altimeter: the leading digits sit in a
  // snug box; the ones digit rides a rolling drum in a full-height window that
  // stands TALLER than the leading box (protruding above and below) and carries
  // the pointer caret on its right edge.
  const float leadH = h * 0.66f;
  const float leadTop = midY - leadH * 0.5f;
  const float leadBot = midY + leadH * 0.5f;

  // Three equal-width digit cells span the box (NXi: hundreds/tens in the snug
  // leading box, ones on the full-height drum), so the digits fill the box from
  // the tape's left edge rather than clustering against the drum.
  const float cellW = w / 3.0f;
  const float drumW = cellW;
  const float drumRight = rightX;
  const float drumX = drumRight - drumW;      // x + 2*cellW
  const float drumCx = drumX + drumW * 0.5f;  // ones-digit cell center
  const float hundredsCx = x + cellW * 0.5f;
  const float tensCx = x + cellW * 1.5f;
  const float rowSpacing = h * 0.92f;

  // Stepped silhouette (clockwise from the leading box's top-left): a snug
  // leading-digit box on the left and a taller full-height drum on the right
  // that carries the right-pointing caret. The four outer corners and the drum
  // corners are rounded; the step junctions and caret stay sharp (NXi).
  const float cornerR = h * 0.06f;
  const float drumCornerR = h * 0.04f;
  const std::vector<Point> silhouette = {
      {x, leadTop},                    // leading box top-left
      {drumX, leadTop},                // step
      {drumX, y},                      // drum top-left
      {drumRight, y},                  // drum top-right
      {drumRight, midY - notchHalfH},  // caret top
      {rightX + notchDepth, midY},     // caret tip
      {drumRight, midY + notchHalfH},  // caret bottom
      {drumRight, y + h},              // drum bottom-right
      {drumX, y + h},                  // drum bottom-left
      {drumX, leadBot},                // step
      {x, leadBot},                    // leading box bottom-left
  };
  const std::vector<float> radii = {cornerR, 0.0f, drumCornerR, drumCornerR,
                                    0.0f,    0.0f, 0.0f,        drumCornerR,
                                    drumCornerR, 0.0f, cornerR};
  std::vector<Point> shape = roundPolygonCorners(silhouette, radii);
  r.fillPolygon(shape.data(), static_cast<int>(shape.size()), boxColor);
  shape.push_back(shape.front());
  r.strokePolyline(shape.data(), static_cast<int>(shape.size()), 2.0f,
                   colors::kWhite);

  // fillText's NVG_ALIGN_MIDDLE centers on the font's ascender/descender
  // midpoint; digits have no descender ink, so they ride high. Nudge the
  // baseline down so the glyph ink sits centered in the window and lines up
  // with the caret.
  const float textMidY = midY + textSize * kCapInkCenterNudge;

  // Leading digits, one per cell and centered in it, so the number spreads
  // evenly across the box up to the drum. Leading zeros are suppressed.
  const long hundreds = (snapped / 100) % 10;
  const long tens = (snapped / 10) % 10;
  if (snapped >= 100) {
    r.fillText(hundredsCx, textMidY, formatInt(static_cast<float>(hundreds)),
               textSize, TextAlign::Center, textColor);
  }
  if (snapped >= 10) {
    r.fillText(tensCx, textMidY, formatInt(static_cast<float>(tens)), textSize,
               TextAlign::Center, textColor);
  }

  r.save();
  r.clip(drumX, y + 2.0f, drumW, h - 4.0f);
  for (int j = -1; j <= 1; ++j) {
    const long disp = (((onesCenter + j) % 10) + 10) % 10;
    const float ty =
        textMidY - static_cast<float>(j) * rowSpacing + residual * rowSpacing;
    char buf[2];
    std::snprintf(buf, sizeof(buf), "%ld", disp);
    r.fillText(drumCx, ty, std::string(buf), textSize, TextAlign::Center,
               textColor);
  }
  r.restore();
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
  const int rows = static_cast<int>(order.size());
  float y = stripTop + stripH - rowH * (rows + 0.5f);
  for (int k = 0; k < rows; ++k) {
    const VSpeedRef& v = kVSpeedRefs[order[k]];
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(order[k]));
    const float rowCy = y + rowH * 0.5f;
    r.fillText(tapeX + tapeW * 0.18f, rowCy, v.bugLabel, labelSize,
               TextAlign::Left, colors::kCyan);
    r.fillText(tapeX + tapeW * 0.92f, rowCy, formatInt(vKt), labelSize,
               TextAlign::Right, colors::kWhite);
    y += rowH;
  }
}

}  // namespace

void drawAirspeedTape(Renderer& r, const Layout& L, const FlightData& d,
                      const SoftkeyController& ui, float h) {
  // Air data computer failure: the airspeed tape and TAS (both ADC-sourced)
  // are replaced by a red X.
  if (!d.airspeedValid) {
    drawFailureX(r, L.asiX, L.asiTop, L.asiW, L.asiH, "", h);
    return;
  }

  // The airspeed instrument (tape numbers, IAS readout, GS/TAS) renders in a
  // semibold face to match the heavier weight of the real G1000 NXi.
  const FontScope airspeedFont(r, FontFace::DejaVuSemiBold);

  // The NXi airspeed tape has a 10 px rounded top-left corner (no top reference-
  // speed box is shown here, so the tape itself carries the round).
  drawVerticalTape(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy, h,
                   d.airspeedKts, kAirspeedViewableKnots, kAirspeedMajorKnots,
                   kAirspeedMinorKnots, kAirspeedMinKnots, false,
                   kTapeCornerRadiusWt * L.s,
                   L.asiW * kAirspeedBandWidthFraction);
  drawAirspeedColorBands(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy,
                         d.airspeedKts);
  drawVspeedBugs(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy, h,
                 d.airspeedKts, ui);
  if (d.airspeedKts < kAirspeedMinKnots) {
    drawVspeedList(r, L.asiX, L.asiW, L.stripTop, L.stripH, h, ui);
  }
  drawTrendVector(r, L.asiX + L.asiW, L.stripTop, L.stripH, L.attCy, h,
                  L.stripH / kAirspeedViewableKnots, d.airspeedTrendKts);

  const float readoutH = kAsiReadoutHeightWt * L.s;
  // Use the same digit size as the altimeter readout so the two windows match.
  const float readoutSize = fontPx(wt::kReadoutAlt, h);
  // Size the window to its three digits (snug) and right-align it so the caret
  // still sits at the tape's right edge -- the window is inset from the tape's
  // left edge rather than spanning the full tape width (which spread the digits).
  const float readoutW = r.measureTextWidth("0", readoutSize) * 3.75f;
  const float readoutX = L.asiX + L.asiW - readoutW;

  // The pointer is black until VNE, then red. If the trend vector crosses VNE
  // (but current speed has not), the digits turn amber as an early warning.
  const bool overVne = d.airspeedKts >= kVneKt;
  const bool trendOverVne = (d.airspeedKts + d.airspeedTrendKts) >= kVneKt;
  const Color boxColor = overVne ? colors::kBandRed : colors::kReadoutBox;
  const Color textColor =
      (!overVne && trendOverVne) ? colors::kBandYellow : colors::kWhite;
  drawAirspeedReadout(r, readoutX, L.attCy - readoutH * 0.5f, readoutW, readoutH,
                      d.airspeedKts, readoutSize, boxColor, textColor);

  // Mach readout: shown just below the IAS pointer box when the Mach number
  // reaches 0.40, matching the G1000 NXi (suppressed in the normal piston
  // envelope). Speed of sound is from OAT (a = 38.97*sqrt(T_K) kt).
  const float soundKts = 38.967854f * std::sqrt(d.oatCelsius + 273.15f);
  const float mach = soundKts > 1.0f ? d.tasKts / soundKts : 0.0f;
  if (mach >= 0.40f) {
    char mbuf[16];
    std::snprintf(mbuf, sizeof(mbuf), "M %.3f", mach);
    const float machSize = fontPx(wt::kInfoValue, h);
    r.fillText(L.asiX + L.asiW * 0.5f, L.attCy + readoutH * 0.5f + machSize,
               std::string(mbuf), machSize, TextAlign::Center, colors::kWhite);
  }

  // Ground speed and true airspeed sit in two black boxes flush with the bottom
  // of the airspeed instrument. The TAS box is exactly the tape width and its
  // bottom-left corner is rounded to mirror the tape's rounded top-left, giving
  // the column a continuous rounded-left edge (NXi). The GS box sits to its left
  // (square, sized to content) and extends left of the tape.
  const float labelSize = fontPx(wt::kInfoLabel, h);
  const float valueSize = fontPx(wt::kInfoValue, h);
  const float boxH = kTapeBottomBoxHeightWt * L.s;
  // Top edge of the boxes touches (overlaps a couple px) the bottom of the
  // tape's scroll strip, rather than floating at the instrument bottom.
  const float boxY =
      L.stripTop + L.stripH - kTapeBottomBoxTapeOverlapWt * L.s;
  const float gap = labelSize * 0.25f;
  const float padX = labelSize * 0.5f;
  const float boxGap = labelSize * 0.45f;

  auto runWidth = [&](const char* label, const std::string& value) {
    return r.measureTextWidth(label, labelSize) + gap +
           r.measureTextWidth(value, valueSize) + gap * 0.5f +
           r.measureTextWidth("KT", labelSize);
  };
  // Corner radii order matches the rectangle vertices below: top-left,
  // top-right, bottom-right, bottom-left.
  auto drawSpeedBox = [&](float boxX, float boxW,
                          const std::vector<float>& radii, float scale,
                          const char* label, const std::string& value) {
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
    float tx = boxX + boxW * 0.5f - runWidth(label, value) * scale * 0.5f;
    tx = putText(r, tx, boxCy, label, lSize, colors::kLabelText, g / lSize);
    tx = putText(r, tx, boxCy, value, vSize, colors::kWhite,
                 (g * 0.5f) / vSize);
    putText(r, tx, boxCy, "KT", lSize, colors::kLabelText);
  };

  const std::string tasValue = formatInt(d.tasKts);
  const std::string gsValue = formatInt(d.groundSpeedKts);
  const float tasX = L.asiX;
  const float tasW = L.asiW;  // exactly the tape width
  // The TAS box is locked to the tape width, so its content may need to shrink
  // to fit. Size the scale against the widest possible value (three digits)
  // rather than the live value, so the GS/TAS font size stays constant with
  // speed. The same scale is used for the GS box so both read at one size.
  float maxDigitW = 0.0f;
  for (char c = '0'; c <= '9'; ++c) {
    maxDigitW = std::max(
        maxDigitW, r.measureTextWidth(std::string(1, c), valueSize));
  }
  const float tasRunMax = r.measureTextWidth("TAS", labelSize) + gap +
                          maxDigitW * 3.0f + gap * 0.5f +
                          r.measureTextWidth("KT", labelSize);
  const float tasAvail = tasW - padX * 2.0f;
  const float speedScale =
      (tasRunMax > tasAvail && tasRunMax > 0.0f) ? tasAvail / tasRunMax : 1.0f;
  const float gsW = runWidth("GS", gsValue) * speedScale + padX * 2.0f;
  const float gsX = tasX - boxGap - gsW;
  const float cornerR = kTapeCornerRadiusWt * L.s;
  // GS is a free-floating box, so round all four corners like the rest of the
  // readouts; the TAS box only rounds its bottom-left to mirror the tape.
  drawSpeedBox(gsX, gsW, {cornerR, cornerR, cornerR, cornerR}, speedScale, "GS",
               gsValue);
  drawSpeedBox(tasX, tasW, {0.0f, 0.0f, 0.0f, cornerR}, speedScale, "TAS",
               tasValue);
}

}  // namespace avionics::pfd
