#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

void drawSelectedAltitude(Renderer& r, float tapeX, float tapeW, float tapeTop,
                          float tapeH, float cy, float displayH,
                          float altitudeFt, float selectedFt) {
  const float ppu = tapeH / kAltitudeViewableFeet;
  const float boxH = fontPx(wt::kSelectedAlt, displayH) * 1.5f;
  const float boxW = tapeW;
  const float boxX = tapeX;
  const float boxY = tapeTop;

  // X-Plane reports an unselected autopilot altitude as 0 ft. There's no real
  // "select 0 ft" use case, so treat it as unset: the box shows dashes and no
  // altitude bug is drawn on the tape (matching the G1000 NXi / X-Plane PFD).
  const bool selected = std::fabs(selectedFt) > kAltSelectedEpsilonFt;

  const bool captured =
      selected && std::fabs(altitudeFt - selectedFt) <= kAltCapturedFt;
  r.fillRect(boxX, boxY, boxW, boxH,
             captured ? colors::kCyan : colors::kReadoutBox);
  const Point outline[5] = {{boxX, boxY},
                            {boxX + boxW, boxY},
                            {boxX + boxW, boxY + boxH},
                            {boxX, boxY + boxH},
                            {boxX, boxY}};
  r.strokePolyline(outline, 5, 2.0f, colors::kCyan);
  r.fillText(boxX + boxW * 0.5f, boxY + boxH * 0.5f,
             selected ? formatInt(selectedFt) : std::string(kSelectedAltDashes),
             fontPx(wt::kSelectedAlt, displayH), TextAlign::Center,
             captured ? colors::kBlack : colors::kCyan);

  if (!selected) {
    return;
  }

  float bugY = cy - (selectedFt - altitudeFt) * ppu;
  bugY = std::max(tapeTop, std::min(tapeTop + tapeH, bugY));
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
                     float baroInHg) {
  // The BARO box fills the gap between the bottom of the scrolling tape and the
  // bottom of the altimeter instrument, so it sits flush against the tape (G1000
  // NXi), mirroring the selected-altitude box at the top.
  const float boxY = stripBottom;
  const float boxH = (tapeTop + tapeH) - stripBottom;
  r.fillRect(tapeX, boxY, tapeW, boxH, colors::kReadoutBox);
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.2f", baroInHg);
  const float numSize = fontPx(wt::kBaro, displayH);
  const float unitSize = numSize * 0.72f;
  const float gap = numSize * 0.06f;
  const float numW = r.measureTextWidth(buf, numSize);
  const float unitW = r.measureTextWidth("IN", unitSize);
  const float startX = tapeX + tapeW * 0.5f - (numW + gap + unitW) * 0.5f;
  const float midY = boxY + boxH * 0.5f;
  r.fillText(startX, midY, std::string(buf), numSize, TextAlign::Left,
             colors::kCyan);
  r.fillText(startX + numW + gap, midY, "IN", unitSize, TextAlign::Left,
             colors::kCyan);
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

  std::string lead =
      formatInt(static_cast<float>(snapped < 0 ? -leading : leading));
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

void drawAltimeter(Renderer& r, const Layout& L, const FlightData& d, float h) {
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
  drawAltitudeReadout(r, L.altX - altOverhang, L.attCy - readoutH * 0.5f,
                      L.altW + altOverhang, readoutH, d.altitudeFt, readoutSize);
  drawSelectedAltitude(r, L.altX, L.altW, L.altTop, L.altH, L.attCy, h,
                       d.altitudeFt, d.selectedAltitudeFt);
  drawBaroSetting(r, L.altX, L.altW, L.altTop, L.altH, L.stripTop + L.stripH, h,
                  d.baroSettingInHg);
}

}  // namespace avionics::pfd
