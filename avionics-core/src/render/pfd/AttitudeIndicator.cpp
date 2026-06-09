#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

void drawPitchLadder(Renderer& r, const Layout& L, float displayH) {
  // Lines every 2.5 deg: full marks at 10 deg, medium at 5 deg, short at
  // 2.5 deg, with numbers every 10 deg, per the NXi non-SVT ladder. Geometry is
  // in Working Title pixel units scaled to the display.
  const float travel = kPitchPxPerDegWt * L.s;
  const float half10 = kPitch10HalfWt * L.sx;
  const float half5 = kPitch5HalfWt * L.sx;
  const float half25 = kPitch25HalfWt * L.sx;
  const float labelSize = fontPx(wt::kPitch, displayH);
  const float labelPad = 16.0f * L.sx;

  for (int q = -34; q <= 34; ++q) {
    if (q == 0) continue;
    const float deg = static_cast<float>(q) * 2.5f;
    const bool is10 = (q % 4) == 0;
    const bool is5 = (q % 2) == 0;

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

void drawRollScale(Renderer& r, float cx, float cy, float radius, float s) {
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
  for (const Tick& t : ticks) {
    r.save();
    r.rotateDegrees(t.deg);
    r.strokeLine(0.0f, -radius, 0.0f, -radius - t.len, 2.0f, colors::kWhite);
    r.restore();
  }

  const float tw = s * 0.030f;
  const float th = s * 0.045f;
  const Point zero[3] = {
      {-tw, -radius - th}, {tw, -radius - th}, {0.0f, -radius}};
  r.fillPolygon(zero, 3, colors::kWhite);
  r.restore();
}

void drawRollPointer(Renderer& r, float cx, float cy, float radius, float s,
                     float rollDeg, float slipDeg) {
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(rollDeg);

  const float tw = s * 0.030f;
  const float th = s * 0.045f;
  const float apexY = -radius + s * 0.008f;
  const Point ptr[3] = {{0.0f, apexY}, {-tw, apexY + th}, {tw, apexY + th}};
  r.fillPolygon(ptr, 3, colors::kWhite);

  const float baseY = apexY + th + s * 0.010f;
  const float trapH = s * 0.024f;
  const float topHalf = tw * 0.9f;
  const float botHalf = tw * 1.2f;
  const float slipShift = slipDeg * (s * 0.018f);
  const Point trap[4] = {{slipShift - topHalf, baseY},
                         {slipShift + topHalf, baseY},
                         {slipShift + botHalf, baseY + trapH},
                         {slipShift - botHalf, baseY + trapH}};
  r.fillPolygon(trap, 4, colors::kWhite);
  r.restore();
}

void drawAircraftSymbol(Renderer& r, float cx, float cy, float attVisW,
                        float attRegionH) {
  // The G1000 NXi aircraft reference symbol (manual Figure 2-1, 7-4) is a
  // two-tone yellow gull-wing chevron at the pivot plus two detached horizontal
  // wing-tip tabs out toward the window edges. Geometry is taken from the real
  // PFD: the chevron spans ~0.39 of the attitude window, the tabs sit at ~0.38
  // of the window half-width, and the chevron's vertical drop is shallow.
  const float chevHalf = attVisW * 0.190f;
  const float apexHalf = attVisW * 0.012f;
  const float chevDrop = attRegionH * 0.045f;
  const float wingThick = attRegionH * 0.026f;
  const float tabOffset = attVisW * 0.380f;
  const float tabHalf = attVisW * 0.037f;
  const float pad = attRegionH * 0.004f;
  const float strokeW = std::max(1.0f, attRegionH * 0.004f);

  // Two-tone yellow horizontal tab on a black outline plate: bright top half,
  // shaded bottom half (the detached wing-tip references).
  auto twoToneBox = [&](float x, float y, float bw, float bh) {
    r.fillRect(x - pad, y - pad, bw + 2.0f * pad, bh + 2.0f * pad,
               colors::kSymbolOutline);
    r.fillRect(x, y, bw, bh * 0.5f, colors::kSymbolYellow);
    r.fillRect(x, y + bh * 0.5f, bw, bh * 0.5f, colors::kSymbolYellowDark);
  };

  // One sloped chevron wing, split along its centerline into a bright upper
  // face and a shaded lower face, with a black outline around the perimeter.
  auto chevronWing = [&](float ix, float iy, float ox, float oy) {
    const float dx = ox - ix, dy = oy - iy;
    const float len = std::sqrt(dx * dx + dy * dy);
    float ux = -dy / len * wingThick * 0.5f;
    float uy = dx / len * wingThick * 0.5f;
    if (uy > 0.0f) {  // make (ux, uy) the upward-facing normal
      ux = -ux;
      uy = -uy;
    }
    const Point top[4] = {
        {ix + ux, iy + uy}, {ox + ux, oy + uy}, {ox, oy}, {ix, iy}};
    const Point bot[4] = {
        {ix, iy}, {ox, oy}, {ox - ux, oy - uy}, {ix - ux, iy - uy}};
    r.fillPolygon(top, 4, colors::kSymbolYellow);
    r.fillPolygon(bot, 4, colors::kSymbolYellowDark);
    const Point outline[5] = {top[0], top[1], bot[2], bot[3], top[0]};
    r.strokePolyline(outline, 5, strokeW, colors::kSymbolOutline);
  };

  chevronWing(cx - apexHalf, cy, cx - chevHalf, cy + chevDrop);
  chevronWing(cx + apexHalf, cy, cx + chevHalf, cy + chevDrop);

  twoToneBox(cx - tabOffset - tabHalf, cy - wingThick * 0.5f, 2.0f * tabHalf,
             wingThick);
  twoToneBox(cx + tabOffset - tabHalf, cy - wingThick * 0.5f, 2.0f * tabHalf,
             wingThick);
}

void drawFlightDirector(Renderer& r, float cx, float cy, float attVisW,
                        float attRegionH, float travel, const FlightData& d) {
  if (!d.flightDirectorActive) return;
  float pitchErr = d.fdPitchDeg - d.pitchDeg;
  float rollErr = d.fdRollDeg - d.rollDeg;
  pitchErr = std::max(-15.0f, std::min(15.0f, pitchErr));
  rollErr = std::max(-30.0f, std::min(30.0f, rollErr));

  // Single-cue magenta command bars form a wide, shallow chevron just outside
  // the aircraft symbol's chevron (manual Figure 7-4): each wing is a tapered
  // wedge that comes to a point near the center apex and squares off thick at
  // the outer tip, so the pilot "flies" the yellow chevron up into the magenta.
  const float fdHalf = attVisW * 0.245f;
  const float apexHalf = attVisW * 0.018f;
  const float fdDrop = attRegionH * 0.045f;
  const float tipThick = attRegionH * 0.034f;
  const float strokeW = std::max(1.0f, attRegionH * 0.004f);

  r.save();
  r.translate(cx, cy - pitchErr * travel);
  r.rotateDegrees(rollErr);

  auto wedge = [&](float sgn) {
    const float ax = sgn * apexHalf, ay = 0.0f;
    const float tx = sgn * fdHalf, ty = fdDrop;
    const float dx = tx - ax, dy = ty - ay;
    const float len = std::sqrt(dx * dx + dy * dy);
    const float nx = -dy / len * tipThick * 0.5f;
    const float ny = dx / len * tipThick * 0.5f;
    const Point tri[3] = {
        {ax, ay}, {tx + nx, ty + ny}, {tx - nx, ty - ny}};
    r.fillPolygon(tri, 3, colors::kMagenta);
    const Point outline[4] = {tri[0], tri[1], tri[2], tri[0]};
    r.strokePolyline(outline, 4, strokeW, colors::kSymbolOutline);
  };
  wedge(-1.0f);
  wedge(1.0f);
  r.restore();
}

}  // namespace

void drawAttitude(Renderer& r, const Layout& L, const FlightData& d, float w,
                  float h) {
  // AHRS failure: no sky/ground/ladder. The attitude window goes black with a
  // red X and an "AHRS" annunciation; the fixed aircraft symbol stays.
  if (!d.attitudeValid) {
    drawFailureX(r, L.attCx - L.attVisW * 0.5f, L.attTop, L.attVisW,
                 L.attRegionH, "AHRS", h);
    drawAircraftSymbol(r, L.attCx, L.attCy, L.attVisW, L.attRegionH);
    return;
  }

  const float travel = kPitchPxPerDegWt * L.s;

  // Sky/ground fill the entire PFD background (NXi #HorizonContainer); the HSI
  // compass rose is overlaid on top with a translucent dark backing, so the
  // brown ground still shows through dimmed at the bottom (blue-over-brown
  // without SVT). The sky is a near-flat deep blue that blends slightly toward
  // the horizon; the ground is flat dark brown.
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

  drawRollScale(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH);
  drawRollPointer(r, L.attCx, L.attCy, L.rollRadius, L.attRegionH, d.rollDeg,
                  d.slipSkidDeg);
  drawFlightDirector(r, L.attCx, L.attCy, L.attVisW, L.attRegionH, travel, d);
  drawAircraftSymbol(r, L.attCx, L.attCy, L.attVisW, L.attRegionH);
}

}  // namespace avionics::pfd
