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

  // Altitude Alerting (Pilot's Guide Fig. 2-32): during a phase transition the
  // readout flashes -- black-on-cyan within 1000 ft, blinking cyan within 200
  // ft, blinking amber on a post-capture deviation.
  const Color plate = alert.cyanBackground ? colors::kCyan : colors::kReadoutBox;
  Color text = alert.cyanBackground ? colors::kBlack
               : alert.amberText    ? colors::kBandYellow
                                    : colors::kCyan;
  r.fillRect(boxX, boxY, boxW, boxH, plate);
  const Point outline[5] = {{boxX, boxY},
                            {boxX + boxW, boxY},
                            {boxX + boxW, boxY + boxH},
                            {boxX, boxY + boxH},
                            {boxX, boxY}};
  r.strokePolyline(outline, 5, 2.0f, colors::kCyan);
  if (!alert.hideText) {
    r.fillText(boxX + boxW * 0.5f, boxY + boxH * 0.5f,
               selected ? formatInt(selectedFt)
                        : std::string(kSelectedAltDashes),
               fontPx(wt::kSelectedAlt, displayH), TextAlign::Center, text);
  }

  if (!selected) {
    return;
  }

  // The bug rides the tape, so it shares the tape's feet-per-pixel scale and
  // pins at the ends of the scroll strip when the selection is off-scale.
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

void drawBaroSetting(Renderer& r, float tapeX, float tapeW, float tapeTop,
                     float tapeH, float stripBottom, float displayH,
                     float baroInHg, bool hpa, bool flashOff) {
  // The BARO box fills the gap between the bottom of the scrolling tape and the
  // bottom of the altimeter instrument, so it sits flush against the tape (G1000
  // NXi), mirroring the selected-altitude box at the top. The Baro Transition
  // Alert flashes the setting (handled by the caller's blink phase via flashOff).
  const float boxY = stripBottom;
  const float boxH = (tapeTop + tapeH) - stripBottom;
  r.fillRect(tapeX, boxY, tapeW, boxH, colors::kReadoutBox);
  if (flashOff) return;  // blink-off half of the Baro Transition Alert cycle

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
  const float numSize = fontPx(wt::kBaro, displayH);
  const float unitSize = numSize * 0.72f;
  const float gap = numSize * 0.06f;
  const float numW = r.measureTextWidth(buf, numSize);
  const float unitW = r.measureTextWidth(unit, unitSize);
  const float startX = tapeX + tapeW * 0.5f - (numW + gap + unitW) * 0.5f;
  const float midY = boxY + boxH * 0.5f;
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
                         float altitudeFt, float textSize) {
  const float midY = y + h * 0.5f;

  // Layout (NXi/Garmin): the thousands/hundreds digits are static and full-size
  // in a snug leading box; the last two digits (tens) ride a rolling drum in a
  // full-height window that stands TALLER than the leading box, protruding above
  // and below it.
  const float leadH = h * 0.66f;
  const float leadTop = midY - leadH * 0.5f;
  const float leadBot = midY + leadH * 0.5f;
  const float notchHalfH = leadH * 0.30f;
  const float notchDepth = h * 0.14f;

  const float drumW = w * 0.34f;
  const float drumRight = x + w;
  const float drumX = drumRight - drumW;
  const float drumCx = drumX + drumW * 0.5f;
  const float drumSize = textSize * 0.78f;
  const float rowSpacing = h * 0.92f;

  // Black fills: snug leading-digit box, the taller full-height drum, and the
  // left caret.
  r.fillRect(x, leadTop, w, leadH, colors::kReadoutBox);
  r.fillRect(drumX, y, drumW, h, colors::kReadoutBox);
  const Point caret[3] = {
      {x, midY - notchHalfH}, {x - notchDepth, midY}, {x, midY + notchHalfH}};
  r.fillPolygon(caret, 3, colors::kReadoutBox);

  // White outline of the stepped silhouette: caret on the left, taller drum on
  // the right.
  const Point outline[12] = {{x, leadTop},
                             {drumX, leadTop},
                             {drumX, y},
                             {drumRight, y},
                             {drumRight, y + h},
                             {drumX, y + h},
                             {drumX, leadBot},
                             {x, leadBot},
                             {x, midY + notchHalfH},
                             {x - notchDepth, midY},
                             {x, midY - notchHalfH},
                             {x, leadTop}};
  r.strokePolyline(outline, 12, 2.0f, colors::kWhite);

  const long snapped = std::lround(altitudeFt / 20.0f) * 20L;
  const long absSnap = std::labs(snapped);
  const long leading = absSnap / 100;
  const long tensCenter = absSnap % 100;
  const float residual =
      (altitudeFt - static_cast<float>(snapped)) / 20.0f;

  // Build the leading-digits string with an explicit sign so altitudes between
  // -1 and -99 ft (leading == 0) still read negative.
  std::string lead = formatInt(static_cast<float>(leading));
  if (snapped < 0) lead.insert(lead.begin(), '-');
  r.fillText(drumX - w * 0.02f, midY, lead, textSize, TextAlign::Right,
             colors::kWhite);

  r.save();
  r.clip(drumX, y + 2.0f, drumW, h - 4.0f);
  for (int j = -2; j <= 2; ++j) {
    const long m = tensCenter + static_cast<long>(j) * 20L;
    const long disp = ((m % 100) + 100) % 100;
    const float ty = midY - static_cast<float>(j) * rowSpacing +
                     residual * rowSpacing;
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02ld", disp);
    r.fillText(drumCx, ty, std::string(buf), drumSize, TextAlign::Center,
               colors::kWhite);
  }
  r.restore();
}

}  // namespace

void drawAltimeter(Renderer& r, const Layout& L, const FlightData& d,
                   const SoftkeyController& ui, float h) {
  // Air data computer failure: the altitude tape, readout, baro, and selected-
  // altitude column are replaced by a red X.
  if (!d.altitudeValid) {
    drawFailureX(r, L.altX, L.altTop, L.altW, L.altH, "", h);
    return;
  }

  drawVerticalTape(r, L.altX, L.altW, L.stripTop, L.stripH, L.attCy, h,
                   d.altitudeFt, kAltitudeViewableFeet, kAltitudeMajorFeet,
                   kAltitudeMinorFeet, kAltitudeMinFeet, true);
  drawTrendVector(r, L.altX, L.stripTop, L.stripH, L.attCy, h,
                  L.stripH / kAltitudeViewableFeet, d.altitudeTrendFt);

  const float readoutH = kAltReadoutHeightWt * L.s;
  const float readoutSize = fontPx(wt::kReadoutAlt, h);
  const float altOverhang = L.altW * kReadoutOverhangFraction;
  drawMinimums(r, L, d, ui, h);
  drawAltitudeReadout(r, L.altX - altOverhang, L.attCy - readoutH * 0.5f,
                      L.altW + altOverhang, readoutH, d.altitudeFt, readoutSize);
  drawSelectedAltitude(r, L.altX, L.altW, L.altTop, L.stripTop, L.stripH,
                       L.attCy, h, d.altitudeFt, d.selectedAltitudeFt,
                       ui.selectedAltStyle());

  const bool selected = std::fabs(d.selectedAltitudeFt) > kAltSelectedEpsilonFt;
  if (ui.displayToggle(DisplayToggle::AltMeters)) {
    drawMetricAltitude(r, L.altX, L.altW, L.stripTop, L.attCy, readoutH, h,
                       d.altitudeFt, d.selectedAltitudeFt, selected);
  }

  const bool hpa = ui.displayToggle(DisplayToggle::BaroHpa);
  const bool flashOff = d.baroTransitionAlert && !ui.blinkOn();
  drawBaroSetting(r, L.altX, L.altW, L.altTop, L.altH, L.stripTop + L.stripH, h,
                  d.baroSettingInHg, hpa, flashOff);
}

}  // namespace avionics::pfd
