#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

void drawPitchLadder(Renderer& r, const Layout& L, float displayH) {
  // NXi non-SVT pitch ladder mark density (Pilot's Guide, Attitude Indicator):
  // major 10 deg marks with numeric labels up to 90 deg; 5 deg minor marks up
  // to 25 deg below and 45 deg above the horizon; 2.5 deg minor marks only
  // between 20 deg below and 20 deg above the horizon. Geometry is in Working
  // Title pixel units scaled to the display.
  const float travel = kPitchPxPerDegWt * L.s;
  const float half10 = kPitch10HalfWt * L.sx;
  const float half5 = kPitch5HalfWt * L.sx;
  const float half25 = kPitch25HalfWt * L.sx;
  const float labelSize = fontPx(wt::kPitch, displayH);
  const float labelPad = 16.0f * L.sx;

  // +deg is nose-up (above the horizon); -deg is nose-down (below it).
  for (int q = -36; q <= 36; ++q) {
    if (q == 0) continue;
    const float deg = static_cast<float>(q) * 2.5f;
    const bool is10 = (q % 4) == 0;
    const bool is5 = (q % 2) == 0;

    if (!is5) {
      if (deg < -20.0f || deg > 20.0f) continue;  // 2.5 deg marks near horizon
    } else if (!is10) {
      if (deg < -25.0f || deg > 45.0f) continue;  // 5 deg marks
    }

    const float y = -deg * travel;
    const float half = is10 ? half10 : (is5 ? half5 : half25);
    r.strokeLine(-half, y, half, y, 1.5f, colors::kWhite);

    if (is10) {
      const std::string label = formatInt(std::abs(deg));
      r.fillText(-half - labelPad, y, label, labelSize, TextAlign::Center,
                 colors::kWhite);
      r.fillText(half + labelPad, y, label, labelSize, TextAlign::Center,
                 colors::kWhite);
    }
  }
}

void drawUnusualAttitudeChevrons(Renderer& r, float attVisW, float attRegionH,
                                 float travel) {
  // Red chevrons point toward the horizon during extreme pitch, starting at 50
  // deg above and 30 deg below the horizon line (G1000 NXi). They live in the
  // pitched ladder frame, so they scroll into view only at extreme attitudes.
  const float w = attVisW * 0.055f;
  const float hgt = attRegionH * 0.030f;
  const float lineW = std::max(3.0f, attRegionH * 0.013f);

  for (int deg = 50; deg <= 80; deg += 10) {
    const float y = -static_cast<float>(deg) * travel;
    const Point v[3] = {{-w, y}, {0.0f, y + hgt}, {w, y}};
    r.strokePolyline(v, 3, lineW, colors::kBandRed);
  }
  for (int deg = 30; deg <= 60; deg += 10) {
    const float y = static_cast<float>(deg) * travel;
    const Point v[3] = {{-w, y}, {0.0f, y - hgt}, {w, y}};
    r.strokePolyline(v, 3, lineW, colors::kBandRed);
  }
}

// Roll scale: ticks plus the inverted zero-reference triangle. The scale
// rotates WITH the horizon so the zero-reference triangle stays earth-referenced,
// while the roll pointer below it is fixed to the airframe (Garmin trainer).
void drawRollScale(Renderer& r, float cx, float cy, float radius, float s,
                   float rollDeg) {
  // The real G1000 roll-scale ticks sit directly on the sky (no dark band).
  struct Tick {
    float deg;
    float len;
  };
  const Tick ticks[] = {{-60, s * 0.050f}, {-45, s * 0.038f}, {-30, s * 0.050f},
                        {-20, s * 0.034f}, {-10, s * 0.034f}, {10, s * 0.034f},
                        {20, s * 0.034f},  {30, s * 0.050f},  {45, s * 0.038f},
                        {60, s * 0.050f}};

  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rollDeg);  // same transform as the horizon

  // Bank arc: one continuous line through all the bank-angle ticks (-60..+60).
  // The real NXi roll scale is a single arc with tick stubs hanging off it, not
  // free-floating ticks.
  constexpr int kArcSeg = 48;
  Point arc[kArcSeg + 1];
  for (int i = 0; i <= kArcSeg; ++i) {
    const float deg = -60.0f + 120.0f * (static_cast<float>(i) / kArcSeg);
    polarOffset(deg, radius, arc[i].x, arc[i].y);
  }
  r.strokePolyline(arc, kArcSeg + 1, 2.0f, colors::kWhite);

  for (const Tick& t : ticks) {
    r.save();
    r.rotateDegrees(t.deg);
    r.strokeLine(0.0f, -radius, 0.0f, -radius - t.len, 2.0f, colors::kWhite);
    r.restore();
  }

  // Zero-bank reference triangle (trainer: 16 px tall, 18 px wide at the base,
  // apex on the bank arc). Rotates with the scale to meet the fixed pointer.
  const float sy = radius / 193.0f;
  const float halfW = kTrainerRollZeroTriHalfWidthPx * sy;
  const float triH = kTrainerRollZeroTriHeightPx * sy;
  const Point zero[3] = {
      {-halfW, -radius - triH}, {halfW, -radius - triH}, {0.0f, -radius}};
  r.fillPolygon(zero, 3, colors::kWhite);
  r.restore();
}

// Roll pointer and slip/skid indicator, fixed at top center: the pointer stays
// aircraft-referenced and the slip/skid bar beneath it displaces laterally only
// (Garmin trainer PFD Default.bmp).
void drawRollPointer(Renderer& r, float cx, float cy, float radius, float s,
                     float slipDeg) {
  r.save();
  r.translate(cx, cy);

  const float sy = radius / 193.0f;
  const float halfW = kTrainerRollPointerHalfWidthPx * sy;
  const float apexY = -radius + kTrainerRollPointerApexInsetPx * sy;
  const float baseY = apexY + kTrainerRollPointerHeightPx * sy;
  const Point ptr[3] = {{0.0f, apexY}, {-halfW, baseY}, {halfW, baseY}};
  r.fillPolygon(ptr, 3, colors::kWhite);

  const float trapTopY = baseY + kTrainerRollSlipGapPx * sy;
  const float trapBotY = trapTopY + kTrainerRollSlipHeightPx * sy;
  const float trapTopHalf = kTrainerRollSlipTopHalfPx * sy;
  const float trapBotHalf = kTrainerRollSlipBotHalfPx * sy;
  // X-Plane's slip_deg is positive when the inclinometer ball is to the left, so
  // negate to make the G1000 skid/slip bar displace toward the ball (the side
  // the pilot must "step on" to coordinate).
  const float slipShift = -slipDeg * (s * 0.018f);
  const Point trap[4] = {{slipShift - trapTopHalf, trapTopY},
                         {slipShift + trapTopHalf, trapTopY},
                         {slipShift + trapBotHalf, trapBotY},
                         {slipShift - trapBotHalf, trapBotY}};
  r.fillPolygon(trap, 4, colors::kWhite);
  r.restore();
}

void drawAircraftSymbol(Renderer& r, float cx, float cy, float attVisW,
                        float attRegionH) {
  // G1000 NXi aircraft reference symbol, reproduced 1:1 from the WT attitude
  // SVG (414x315 window, geometry referenced to the symbol center (207,204)):
  // a two-tone yellow center delta -- two wedges meeting at the apex and sloping
  // down to wingtips at +/-120 px, dropping 30 px -- plus two two-tone
  // horizontal wing-tip bars near the window edges. The outer portion of every
  // face is bright yellow, the inner sliver shaded, for a lit-from-above look.
  const float ux = attVisW / 414.0f;
  const float uy = attRegionH / 315.0f;
  const float strokeW = std::max(1.0f, attRegionH * 0.004f);
  auto P = [&](float px, float py) -> Point {
    return {cx + (px - 207.0f) * ux, cy + (py - 204.0f) * uy};
  };

  // One center-delta wing: a wedge from the apex (207,204) to a base at y=234,
  // split into a bright outer triangle and a shaded inner triangle.
  auto wing = [&](float baseOuter, float baseSplit, float baseInner) {
    const Point bright[3] = {P(207, 204), P(baseOuter, 234), P(baseSplit, 234)};
    const Point dark[3] = {P(207, 204), P(baseSplit, 234), P(baseInner, 234)};
    r.fillPolygon(bright, 3, colors::kSymbolYellow);
    r.fillPolygon(dark, 3, colors::kSymbolYellowDark);
    const Point outline[4] = {P(207, 204), P(baseOuter, 234), P(baseInner, 234),
                              P(207, 204)};
    r.strokePolyline(outline, 4, strokeW, colors::kSymbolOutline);
  };
  wing(87.0f, 122.0f, 141.0f);    // left
  wing(327.0f, 292.0f, 273.0f);   // right (mirror)

  // One wing-tip bar: a horizontal two-tone bar (bright top half y200-204,
  // shaded bottom half y204-208) running from the window edge inward to a
  // pointed inner tip.
  auto tab = [&](float xOuter, float xInner, float xTip) {
    const Point top[4] = {P(xTip, 204), P(xInner, 200), P(xOuter, 200),
                          P(xOuter, 204)};
    r.fillPolygon(top, 4, colors::kSymbolYellow);
    const Point bot[4] = {P(xTip, 204), P(xInner, 208), P(xOuter, 208),
                          P(xOuter, 204)};
    r.fillPolygon(bot, 4, colors::kSymbolYellowDark);
    const Point outline[6] = {P(xOuter, 200), P(xInner, 200), P(xTip, 204),
                              P(xInner, 208), P(xOuter, 208), P(xOuter, 200)};
    r.strokePolyline(outline, 6, strokeW, colors::kSymbolOutline);
  };
  tab(1.0f, 44.0f, 47.0f);      // left  (edge x=1, body to 44, inner tip at 47)
  tab(411.0f, 368.0f, 365.0f);  // right (edge x=411, body to 368, inner tip 365)
}

void drawFlightDirector(Renderer& r, float cx, float cy, float attVisW,
                        float attRegionH, float travel, const FlightData& d) {
  if (!d.flightDirectorActive) return;
  float pitchErr = d.fdPitchDeg - d.pitchDeg;
  float rollErr = d.fdRollDeg - d.rollDeg;
  pitchErr = std::max(-15.0f, std::min(15.0f, pitchErr));
  rollErr = std::max(-30.0f, std::min(30.0f, rollErr));

  // Single-cue magenta command bars, reproduced 1:1 from the WT FlightDirector
  // SVG (414x315 window, referenced to (207,204)): each bar is a thin tapered
  // wedge running from the apex out along the top edge of the aircraft symbol's
  // wing to a squared-off tip just past the wingtip, so the pilot "flies" the
  // yellow delta up into the magenta. The whole cue is offset for pitch/roll
  // command error, so when coordinated it overlays the aircraft symbol exactly.
  const float ux = attVisW / 414.0f;
  const float uy = attRegionH / 315.0f;
  const float strokeW = std::max(1.0f, attRegionH * 0.004f);
  auto P = [&](float px, float py) -> Point {
    return {(px - 207.0f) * ux, (py - 204.0f) * uy};
  };

  r.save();
  r.translate(cx, cy - pitchErr * travel);
  r.rotateDegrees(rollErr);

  // Left bar (main wedge + squared tip merged into one pentagon) and its mirror.
  const Point left[5] = {P(207, 204), P(87, 234), P(73, 234), P(73, 225),
                         P(207, 203)};
  const Point right[5] = {P(207, 204), P(327, 234), P(341, 234), P(341, 225),
                          P(207, 203)};
  r.fillPolygon(left, 5, colors::kMagenta);
  r.fillPolygon(right, 5, colors::kMagenta);
  const Point leftOutline[6] = {left[0], left[1], left[2],
                                left[3], left[4], left[0]};
  const Point rightOutline[6] = {right[0], right[1], right[2],
                                 right[3], right[4], right[0]};
  r.strokePolyline(leftOutline, 6, strokeW, colors::kSymbolOutline);
  r.strokePolyline(rightOutline, 6, strokeW, colors::kSymbolOutline);
  r.restore();
}

}  // namespace

void drawAttitude(Renderer& r, const Layout& L, const FlightData& d, float w,
                  float h, bool powerUp) {
  // AHRS failure: no sky/ground/ladder. The attitude window fills maroon with a
  // red X, but the white roll scale and the fixed yellow aircraft symbol stay
  // drawn, matching the NXi (Maintenance Manual Fig 9-2, PFD Power-Up System
  // Annunciations). The "AHRS" annunciation is shown for an in-flight failure
  // but suppressed during power-up.
  if (!d.attitudeValid) {
    drawFailureX(r, L.attCx - L.attVisW * 0.5f, L.attTop, L.attVisW,
                 L.attRegionH, powerUp ? "" : "AHRS", h);
    drawRollScale(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH, 0.0f);
    drawRollPointer(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH, 0.0f);
    drawAircraftSymbol(r, L.attCx, L.attCy, L.attVisW, L.attRegionH);
    return;
  }

  const float travel = kPitchPxPerDegWt * L.s;

  // Sky/ground fill the entire PFD background (NXi #HorizonContainer); the HSI
  // compass rose is overlaid on top with a translucent dark backing, so the
  // brown ground still shows through dimmed at the bottom (blue-over-brown
  // without SVT). The sky is mostly flat #004cff with a vertical blend to
  // #4a65e6 in the lower ~55% above the horizon; the ground is flat #54350a.
  r.save();
  r.clip(0.0f, 0.0f, w, h);
  r.translate(L.attCx, L.attCy);
  r.rotateDegrees(-d.rollDeg);
  r.translate(0.0f, d.pitchDeg * travel);

  const float big = (w + h) * 2.0f;
  r.fillRectVerticalGradient(-big, -big, big * 2.0f, big, -h * 0.15f, 0.0f,
                             colors::kSkyTop, colors::kSkyHorizon);
  r.fillRect(-big, 0.0f, big * 2.0f, big, colors::kGroundHorizon);
  r.strokeLine(-big, 0.0f, big, 0.0f, 2.0f, colors::kHorizon);
  r.restore();

  // The pitch ladder and extreme-pitch chevrons are clipped to the attitude
  // window (the NXi 414x315 attitude container) so they never reach down into
  // the HSI or across the tapes.
  r.save();
  r.clip(L.attCx - L.attVisW * 0.5f, L.attTop, L.attVisW, L.attRegionH);
  r.translate(L.attCx, L.attCy);
  r.rotateDegrees(-d.rollDeg);
  r.translate(0.0f, d.pitchDeg * travel);
  drawPitchLadder(r, L, h);
  drawUnusualAttitudeChevrons(r, L.attVisW, L.attRegionH, travel);
  r.restore();

  drawRollScale(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH, d.rollDeg);
  drawRollPointer(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH,
                  d.slipSkidDeg);
  drawFlightDirector(r, L.attCx, L.attCy, L.attVisW, L.attRegionH, travel, d);
  drawAircraftSymbol(r, L.attCx, L.attCy, L.attVisW, L.attRegionH);
}

}  // namespace avionics::pfd
