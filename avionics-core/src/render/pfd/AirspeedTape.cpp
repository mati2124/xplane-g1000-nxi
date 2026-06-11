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
  const float bandW = tapeW * 0.12f;
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

  const float drumW = w * 0.28f;
  const float drumRight = rightX;
  const float drumX = drumRight - drumW;
  const float drumCx = drumX + drumW * 0.5f;
  const float rowSpacing = h * 0.92f;

  // Black fills: snug leading-digit box, the taller full-height drum, and the
  // right caret.
  r.fillRect(x, leadTop, w, leadH, boxColor);
  r.fillRect(drumX, y, drumW, h, boxColor);
  const Point caret[3] = {{rightX, midY - notchHalfH},
                          {rightX + notchDepth, midY},
                          {rightX, midY + notchHalfH}};
  r.fillPolygon(caret, 3, boxColor);

  // White outline of the stepped silhouette: leading box on the left, taller
  // drum on the right carrying the caret.
  const Point outline[12] = {{x, leadTop},
                             {drumX, leadTop},
                             {drumX, y},
                             {drumRight, y},
                             {drumRight, midY - notchHalfH},
                             {drumRight + notchDepth, midY},
                             {drumRight, midY + notchHalfH},
                             {drumRight, y + h},
                             {drumX, y + h},
                             {drumX, leadBot},
                             {x, leadBot},
                             {x, leadTop}};
  r.strokePolyline(outline, 12, 2.0f, colors::kWhite);

  const long leadVal = snapped / 10;  // everything left of the ones digit
  if (leadVal > 0) {
    r.fillText(drumX - w * 0.02f, midY, formatInt(static_cast<float>(leadVal)),
               textSize, TextAlign::Right, textColor);
  }

  r.save();
  r.clip(drumX, y + 2.0f, drumW, h - 4.0f);
  for (int j = -1; j <= 1; ++j) {
    const long disp = (((onesCenter + j) % 10) + 10) % 10;
    const float ty =
        midY - static_cast<float>(j) * rowSpacing + residual * rowSpacing;
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

  drawVerticalTape(r, L.asiX, L.asiW, L.stripTop, L.stripH, L.attCy, h,
                   d.airspeedKts, kAirspeedViewableKnots, kAirspeedMajorKnots,
                   kAirspeedMinorKnots, kAirspeedMinKnots, false);
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
  const float readoutSize = fontPx(wt::kReadout, h);

  // The pointer is black until VNE, then red. If the trend vector crosses VNE
  // (but current speed has not), the digits turn amber as an early warning.
  const bool overVne = d.airspeedKts >= kVneKt;
  const bool trendOverVne = (d.airspeedKts + d.airspeedTrendKts) >= kVneKt;
  const Color boxColor = overVne ? colors::kBandRed : colors::kReadoutBox;
  const Color textColor =
      (!overVne && trendOverVne) ? colors::kBandYellow : colors::kWhite;
  drawAirspeedReadout(r, L.asiX, L.attCy - readoutH * 0.5f, L.asiW, readoutH,
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

  // TAS and GS (groundspeed) are shown in stacked black boxes at the bottom of
  // the airspeed instrument (NXi airspeed-bottom-container).
  const float labelSize = fontPx(wt::kInfoLabel, h);
  const float valueSize = fontPx(wt::kInfoValue, h);
  const float boxH = 28.0f * L.s;
  auto drawSpeedBox = [&](float boxY, const char* label,
                          const std::string& value) {
    r.fillRect(L.asiX, boxY, L.asiW, boxH, colors::kReadoutBox);
    const float boxCy = boxY + boxH * 0.5f;
    const float gap = labelSize * 0.25f;
    const float runW = r.measureTextWidth(label, labelSize) + gap +
                       r.measureTextWidth(value, valueSize) + gap * 0.5f +
                       r.measureTextWidth("KT", labelSize);
    float tx = L.asiX + L.asiW * 0.5f - runW * 0.5f;
    tx = putText(r, tx, boxCy, label, labelSize, colors::kLabelText,
                 gap / labelSize);
    tx = putText(r, tx, boxCy, value, valueSize, colors::kWhite,
                 (gap * 0.5f) / valueSize);
    putText(r, tx, boxCy, "KT", labelSize, colors::kLabelText);
  };
  const float tasBoxY = L.asiTop + L.asiH - boxH;
  drawSpeedBox(tasBoxY - boxH - 2.0f * L.s, "GS", formatInt(d.groundSpeedKts));
  drawSpeedBox(tasBoxY, "TAS", formatInt(d.tasKts));
}

}  // namespace avionics::pfd
