#include "render/mfd/EisStrip.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "avionics/Color.h"
#include "avionics/Eis.h"
#include "avionics/EisLegacy.h"

// Cirrus Vision SF50 Engine Indication System, modeled on the Perspective
// Touch+ by Garmin Pilot's Guide for the Vision SF50 (190-02470-02 Rev. A),
// Section 3 / Figure 3-2 "EIS Display (Normal)". Unlike the piston Cessna
// stack, the jet packs a % Thrust arc, a row of vertical turbine bars
// (N1/N2/ITT/Oil °C/Oil PSI), a two-tank fuel block, a four-source electrical
// block, and the airframe synoptics (landing gear, pitch/roll trim, flaps, and
// cabin pressurization) into one narrow strip. Gauge limits/bands/bugs and the
// dataref bindings stay data-driven (sf50.eis); this file owns the fixed grid
// geometry that matches the real unit.
namespace avionics::mfd {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// The SF50 strip packs five turbine bars + two-column blocks across a column
// only ~150px wide on the 1024px reference, so its type is roughly half the
// size of the piston stack's to keep labels from colliding.
constexpr float kTitleWt = 9.0f;    // section titles ("Landing Gear", ...)
constexpr float kLabelWt = 8.0f;    // small gauge / unit labels
constexpr float kBarLabelWt = 7.5f; // the five turbine bar captions (N1%, ...)
constexpr float kValueWt = 11.0f;   // numeric readouts
constexpr float kBigWt = 22.0f;     // the large % thrust number

// Full-scale ranges for the synoptic mini-bars the Pilot's Guide draws but does
// not publish numeric limits for; chosen so the live carets sit where Fig. 3-2
// shows them. (The engine/fuel bars stay data-driven via sf50.eis.)
constexpr float kBattAmpsMax = 100.0f;   // battery amp bar
constexpr float kGenAmpsMax = 300.0f;    // generator amp bar
constexpr float kCabinDiffMax = 9.0f;    // cabin differential pressure (psi)
constexpr float kCabinAltMax = 15000.0f; // cabin altitude (ft)

// The % Thrust arc uses a brighter, more saturated green than the airspeed-tape
// band (#008000) to match the vivid green of the real Perspective Touch+ arc.
constexpr Color kThrustGreen{0.04f, 0.80f, 0.04f, 1.0f};

std::string fmt(const char* pattern, double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), pattern, v);
  return buf;
}

void strokeArc(Renderer& r, float cx, float cy, float radius, float a0Deg,
               float a1Deg, float widthPx, const Color& c) {
  constexpr int kSegments = 28;
  Point pts[kSegments + 1];
  for (int i = 0; i <= kSegments; ++i) {
    const float a =
        (a0Deg + (a1Deg - a0Deg) * static_cast<float>(i) / kSegments) * kPi /
        180.0f;
    pts[i] = {cx + radius * std::sin(a), cy - radius * std::cos(a)};
  }
  r.strokePolyline(pts, kSegments + 1, widthPx, c);
}

float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }

float chan(const FlightData& d, const char* c, float fallback = 0.0f) {
  return eisChannelValue(d, c, fallback);
}

// Thin horizontal separator rule between EIS sections (matches the faint gray
// dividers on the real strip).
void rule(Renderer& r, const Rect& a, float y) {
  r.strokeLine(a.x + a.w * 0.04f, y, a.x + a.w * 0.96f, y, 1.0f,
               colors::kPanelSeparator);
}

// One vertical turbine/parameter bar (N1, N2, ITT, Oil °C, Oil PSI). The track
// runs bottom(min) -> top(max); colored bands fill their sub-ranges, a red tick
// marks the steady-state limit (redline), a white triangle points at the live
// value from the right, and an optional cyan bug marks a reference (N1 takeoff).
// The numeric value sits below the track, the label below that.
void drawVertBar(Renderer& r, const Rect& cell, const EisGauge* g, float value,
                 bool valid, float displayH, const char* labelOverride) {
  const float labelSize = mfdFontPx(kBarLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);

  const float minV = g ? g->min : 0.0f;
  const float maxV = g ? g->max : 100.0f;
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;

  const float trackW = std::max(3.0f, cell.w * 0.16f);
  const float trackX = cell.x + cell.w * 0.34f;
  const float trackTop = cell.y + labelSize * 0.2f;
  const float trackH = cell.h - valueSize * 2.6f;
  auto yFor = [&](float v) {
    return trackTop + trackH * (1.0f - clamp01((v - minV) / span));
  };

  r.fillRect(trackX, trackTop, trackW, trackH, Color{0.13f, 0.13f, 0.13f, 1.0f});
  if (g != nullptr) {
    for (const EisBand& b : g->bands) {
      const float y0 = yFor(b.hi);
      const float y1 = yFor(b.lo);
      r.fillRect(trackX, y0, trackW, y1 - y0, eisBandColor(b.color));
    }
    if (g->hasRedline) {
      const float ry = yFor(g->redline);
      r.strokeLine(trackX - trackW * 0.35f, ry, trackX + trackW * 1.35f, ry,
                   2.0f, colors::kBandRed);
    }
  }
  r.strokeLine(trackX, trackTop, trackX, trackTop + trackH, 1.0f,
               colors::kPanelBorder);

  if (valid) {
    if (g != nullptr && g->hasBug) {
      const float by = yFor(g->bug);
      const Point bug[3] = {{trackX - trackW * 0.55f, by},
                            {trackX - trackW * 1.25f, by - trackW * 0.6f},
                            {trackX - trackW * 1.25f, by + trackW * 0.6f}};
      r.fillPolygon(bug, 3, colors::kCyan);
    }
    const float vy = yFor(value);
    const float ph = trackW * 1.1f;
    const Point ptr[3] = {{trackX + trackW * 1.05f, vy},
                          {trackX + trackW * 1.05f + ph, vy - ph * 0.85f},
                          {trackX + trackW * 1.05f + ph, vy + ph * 0.85f}};
    const bool over = g != nullptr && g->hasRedline && value >= g->redline;
    r.fillPolygon(ptr, 3, over ? colors::kBandRed : colors::kWhite);
  }

  const float cx = cell.x + cell.w * 0.5f;
  const float valY = trackTop + trackH + valueSize * 0.85f;
  r.fillText(cx, valY,
             valid ? fmt(g ? g->format.c_str() : "%.0f", value)
                   : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);
  const char* lbl = labelOverride ? labelOverride : (g ? g->label.c_str() : "");
  r.fillText(cx, valY + labelSize * 1.25f, lbl, labelSize, TextAlign::Center,
             colors::kLabelText);
}

// Small vertical synoptic bar (electrical amps, cabin Diff PSI / Alt Ft). A
// green track over [minV,maxV] with optional amber/red caution zones at the
// bottom and/or top, a white top-limit tick, and up to two inward white carets
// (left source points right, right source points left). Matches the secondary
// bars on the SF50 EIS (Fig. 3-2).
void drawMiniVBar(Renderer& r, float cx, float top, float h, float trackW,
                  float minV, float maxV, float vL, bool hasL, float vR,
                  bool hasR, bool valid, float botAmber, float topAmber,
                  float topRed) {
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;
  auto yFor = [&](float v) {
    return top + h * (1.0f - clamp01((v - minV) / span));
  };
  const float x = cx - trackW * 0.5f;
  r.fillRect(x, top, trackW, h, colors::kBandGreen);
  if (botAmber > 0.0f)
    r.fillRect(x, top + h * (1.0f - botAmber), trackW, h * botAmber,
               colors::kBandYellow);
  if (topAmber > 0.0f)
    r.fillRect(x, top + h * topRed, trackW, h * topAmber, colors::kBandYellow);
  if (topRed > 0.0f) r.fillRect(x, top, trackW, h * topRed, colors::kBandRed);
  r.strokeLine(x, top, x + trackW, top, 1.4f, colors::kWhite);
  r.strokeLine(x, top, x, top + h, 1.0f, colors::kPanelBorder);
  if (!valid) return;
  const float cw = trackW * 1.1f;
  if (hasL) {
    const float vy = yFor(vL);
    const Point p[3] = {{x - cw * 0.15f, vy},
                        {x - cw * 1.2f, vy - cw * 0.75f},
                        {x - cw * 1.2f, vy + cw * 0.75f}};
    r.fillPolygon(p, 3, colors::kWhite);
  }
  if (hasR) {
    const float vy = yFor(vR);
    const float xr = x + trackW;
    const Point p[3] = {{xr + cw * 0.15f, vy},
                        {xr + cw * 1.2f, vy - cw * 0.75f},
                        {xr + cw * 1.2f, vy + cw * 0.75f}};
    r.fillPolygon(p, 3, colors::kWhite);
  }
}

// % Thrust arc with the large central readout, "% Thrust" / "FADEC CH A"
// captions, and the throttle friction-lock padlock at the upper-left.
void drawThrustArc(Renderer& r, const FlightData& d, const EisLayout& layout,
                   const Rect& a, bool valid, float displayH) {
  const EisGauge* g = layout.gaugeForChannel(eis_channels::kThrustPct);
  const float maxV = g ? g->max : 100.0f;
  const float cx = a.x + a.w * 0.5f;
  const float cy = a.y + a.h * 0.62f;
  // Fill the cell while still leaving a black margin at the left for the
  // friction-lock padlock, matching the real unit's proportions (Fig. 3-2),
  // and a little padding above the arc so it never touches the cell top.
  const float radius = std::min(a.w * 0.30f, a.h * 0.50f);
  const float band = std::max(5.0f, radius * 0.18f);  // thick green band

  // Top semicircle: start at the 9 o'clock horizontal (270deg) and end at the
  // 3 o'clock horizontal (90deg), matching the real arc's flat ends.
  constexpr float kA0 = -90.0f;
  constexpr float kA1 = 90.0f;
  auto angleFor = [&](float v) {
    return kA0 + (kA1 - kA0) * clamp01(v / (maxV != 0.0f ? maxV : 1.0f));
  };

  // Black channel base so the un-banded span (above the green MCT limit) reads
  // black like the real unit, then the live bands fill their own ranges over it.
  strokeArc(r, cx, cy, radius, kA0, kA1, band, colors::kBlack);
  if (g != nullptr && !g->bands.empty()) {
    for (const EisBand& b : g->bands) {
      const Color c = b.color == EisBandColor::Green ? kThrustGreen
                                                     : eisBandColor(b.color);
      strokeArc(r, cx, cy, radius, angleFor(b.lo), angleFor(b.hi), band, c);
    }
  } else {
    strokeArc(r, cx, cy, radius, kA0, kA1, band, kThrustGreen);
  }
  // White outline on the OUTER edge spanning the whole sweep, plus flat end caps
  // ("carrots") closing each end (Fig. 3-2). No inner edge line.
  const float lwEdge = std::max(1.6f, band * 0.16f);
  strokeArc(r, cx, cy, radius + band * 0.5f, kA0, kA1, lwEdge, colors::kWhite);
  auto endCap = [&](float angDeg) {
    const float aa = angDeg * kPi / 180.0f;
    const float sa = std::sin(aa);
    const float ca = std::cos(aa);
    const float ri = radius - band * 0.5f - lwEdge * 0.5f;
    const float ro = radius + band * 0.5f + lwEdge * 0.5f;
    r.strokeLine(cx + ri * sa, cy - ri * ca, cx + ro * sa, cy - ro * ca, lwEdge,
                 colors::kWhite);
  };
  endCap(kA0);
  endCap(kA1);

  const float pct = chan(d, eis_channels::kThrustPct);
  if (valid) {
    if (g != nullptr && g->hasBug) {
      // Cyan takeoff-thrust reference bug, drawn as a bracket ("-|") sitting
      // just outside the band: a short radial arm pointing inward toward the
      // band, capped by a tangential bar at its outer end (matches Fig. 3-2).
      const float ab = angleFor(g->bug) * kPi / 180.0f;
      const float sa = std::sin(ab);
      const float ca = std::cos(ab);
      const float tx = ca;  // tangential (along the arc)
      const float ty = sa;
      const float lw = std::max(2.0f, band * 0.28f);
      const float rInner = radius + band * 0.55f;  // arm starts just off the band
      const float rOuter = radius + band * 1.5f;   // outer bar sits here
      const float half = band * 0.70f;             // half-length of the outer bar
      const float ox = cx + rOuter * sa;
      const float oy = cy - rOuter * ca;
      r.strokeLine(ox + half * tx, oy + half * ty, ox - half * tx,
                   oy - half * ty, lw, colors::kCyan);
      r.strokeLine(cx + rInner * sa, cy - rInner * ca, ox, oy, lw,
                   colors::kCyan);
    }
    // Bold white arrowhead riding the band, tip at the outer edge.
    const float ab = angleFor(pct) * kPi / 180.0f;
    const float ca = std::cos(ab);
    const float sa = std::sin(ab);
    const float tip = radius + band * 0.35f;
    const float base = radius - band * 2.4f;
    const float hw = radius * 0.14f;
    const Point needle[3] = {
        {cx + tip * sa, cy - tip * ca},
        {cx + base * sa - hw * ca, cy - base * ca - hw * sa},
        {cx + base * sa + hw * ca, cy - base * ca + hw * sa}};
    r.fillPolygon(needle, 3, colors::kWhite);
  }

  // Throttle friction-lock padlock at the upper-left of the arc cell.
  {
    const float lx = a.x + a.w * 0.045f;  // black margin left of the arc
    const float bw = radius * 0.21f;
    const float bh = radius * 0.27f;
    const float byTop = cy - radius * 0.25f;
    r.fillRoundedRect(lx - bw * 0.5f, byTop, bw, bh, bw * 0.14f, colors::kWhite);
    const float shR = bw * 0.30f;
    strokeArc(r, lx, byTop, shR, -90.0f, 90.0f, std::max(1.5f, bw * 0.16f),
              colors::kWhite);
    // Keyhole cutout: a circle with a tapering slot below, cut from the body.
    const float khY = byTop + bh * 0.42f;
    r.fillCircle(lx, khY, bw * 0.16f, colors::kBlack);
    r.fillRect(lx - bw * 0.07f, khY, bw * 0.14f, bh * 0.40f, colors::kBlack);
  }

  // Central readout.
  {
    const float numSize = mfdFontPx(kBigWt, displayH);
    const std::string num =
        valid ? fmt("%.0f", std::round(pct)) : std::string("--");
    r.fillText(cx, cy - radius * 0.05f, num, numSize, TextAlign::Center,
               colors::kWhite);
  }
  const float capSize = mfdFontPx(kLabelWt * 1.2f, displayH);
  const float fadecSize = mfdFontPx(kLabelWt * 1.5f, displayH);
  r.fillText(cx, cy + mfdFontPx(kBigWt, displayH) * 0.58f, "% Thrust", capSize,
             TextAlign::Center, colors::kLabelText);
  r.fillText(cx, cy + mfdFontPx(kBigWt, displayH) * 0.58f + capSize * 1.3f,
             "FADEC CH A", fadecSize, TextAlign::Center, colors::kLabelText,
             FontFace::DejaVuSemiBold);
}

// Two-tank fuel block: L/R selected-tank boxes, the per-tank quantity bars,
// total fuel, fuel flow and fuel temperature.
void drawFuelBlock(Renderer& r, const FlightData& d, const EisLayout& layout,
                   const Rect& a, bool valid, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  const EisGauge* gl = layout.gaugeForChannel(eis_channels::kFuelQtyLeft);
  const float maxV = gl ? gl->max : 150.0f;
  const float minV = gl ? gl->min : 0.0f;
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;

  const float qL = chan(d, eis_channels::kFuelQtyLeft);
  const float qR = chan(d, eis_channels::kFuelQtyRight);
  const bool rightSel = chan(d, eis_channels::kFuelSelected) > 0.5f;

  // Selected-tank L / R boxes across the top.
  const float boxW = a.w * 0.22f;
  const float boxH = labelSize * 1.4f;
  const float boxY = a.y;
  const float lxC = a.x + a.w * 0.40f;
  const float rxC = a.x + a.w * 0.78f;
  auto tankBox = [&](float cxBox, const char* tag, bool selected) {
    if (selected) {
      r.strokeRoundedRect(cxBox - boxW * 0.5f, boxY, boxW, boxH, boxH * 0.25f,
                          1.5f, colors::kWhite);
    }
    r.fillText(cxBox, boxY + boxH * 0.5f, tag, labelSize, TextAlign::Center,
               colors::kWhite);
  };
  tankBox(lxC, "L", !rightSel);
  tankBox(rxC, "R", rightSel);

  // Per-tank vertical bars.
  const float trackTop = boxY + boxH * 1.2f;
  const float trackH = a.h * 0.42f;
  const float trackW = std::max(4.0f, a.w * 0.13f);
  auto yFor = [&](float v) {
    return trackTop + trackH * (1.0f - clamp01((v - minV) / span));
  };
  auto fuelBar = [&](float cxBar, float qty) {
    const float x = cxBar - trackW * 0.5f;
    r.fillRect(x, trackTop, trackW, trackH, Color{0.13f, 0.13f, 0.13f, 1.0f});
    if (gl != nullptr) {
      for (const EisBand& b : gl->bands) {
        const float y0 = yFor(b.hi);
        const float y1 = yFor(b.lo);
        r.fillRect(x, y0, trackW, y1 - y0, eisBandColor(b.color));
      }
    }
    r.strokeLine(x, trackTop, x, trackTop + trackH, 1.0f, colors::kPanelBorder);
    if (valid) {
      const float vy = yFor(qty);
      const Point ptr[3] = {{x + trackW * 1.05f, vy},
                            {x + trackW * 2.0f, vy - trackW * 0.7f},
                            {x + trackW * 2.0f, vy + trackW * 0.7f}};
      r.fillPolygon(ptr, 3, colors::kWhite);
    }
  };
  fuelBar(lxC, qL);
  fuelBar(rxC, qR);

  // Numeric rows below the bars (Fig. 3-2): GPH (left column), per-tank L/R
  // quantities (under each bar), total below, then fuel temp + "Fuel GAL".
  const float gph = chan(d, eis_channels::kFuelFlow);
  const float ftemp = chan(d, eis_channels::kFuelTempC);
  const float rowY = trackTop + trackH + valueSize * 0.7f;
  r.fillText(a.x, rowY, valid ? fmt("%.0f", std::round(gph)) : std::string("--"),
             valueSize, TextAlign::Left, colors::kWhite);
  r.fillText(lxC, rowY, valid ? fmt("%.0f", std::round(qL)) : std::string("---"),
             labelSize, TextAlign::Center, colors::kWhite);
  r.fillText(rxC, rowY, valid ? fmt("%.0f", std::round(qR)) : std::string("---"),
             labelSize, TextAlign::Center, colors::kWhite);
  r.fillText(a.x, rowY + labelSize * 1.25f, "GPH", labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(a.x + a.w * 0.62f, rowY + labelSize * 1.25f,
             valid ? fmt("%.0f", std::round(qL + qR)) : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);
  r.fillText(a.x, rowY + labelSize * 2.6f,
             valid ? fmt("%.0f\xC2\xB0""C", std::round(ftemp))
                   : std::string("--\xC2\xB0""C"),
             labelSize, TextAlign::Left, colors::kWhite);
  r.fillText(a.x + a.w, rowY + labelSize * 2.6f, "Fuel GAL", labelSize,
             TextAlign::Right, colors::kLabelText);
}

// Four-source electrical block: emergency bus volts, battery 1/2 and
// generator 1/2 currents.
void drawElecBlock(Renderer& r, const FlightData& d, const Rect& a, bool valid,
                   float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);

  // Header row: "Emr Bus V" on the left, its volts on the right (Fig. 3-2).
  const float emer = chan(d, eis_channels::kEmerBusVolts);
  const float hY = a.y + labelSize * 0.9f;
  r.fillText(a.x, hY, "Emr Bus V", labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(a.x + a.w, hY, valid ? fmt("%.1f", emer) : std::string("--.-"),
             valueSize, TextAlign::Right, colors::kWhite);

  // Two dual-source amp bars (Battery 1/2, Generator 1/2) side by side.
  const float bat1 = chan(d, eis_channels::kBatt1Amps);
  const float bat2 = chan(d, eis_channels::kBatt2Amps);
  const float gen1 = chan(d, eis_channels::kGen1Amps);
  const float gen2 = chan(d, eis_channels::kGen2Amps);
  const float batCx = a.x + a.w * 0.26f;
  const float genCx = a.x + a.w * 0.74f;
  const float barTop = hY + labelSize * 0.8f;
  const float barH = a.h * 0.40f;
  const float trackW = std::max(4.0f, a.w * 0.07f);
  drawMiniVBar(r, batCx, barTop, barH, trackW, 0.0f, kBattAmpsMax, bat1, true,
               bat2, true, valid, 0.12f, 0.0f, 0.0f);
  drawMiniVBar(r, genCx, barTop, barH, trackW, 0.0f, kGenAmpsMax, gen1, true,
               gen2, true, valid, 0.12f, 0.0f, 0.0f);

  // Per-bar source values and the wrapped "1 Bat 2 / Amp" caption below. The
  // strip column is narrow, so this uses a compact font and tight spacing.
  const float smallSize = labelSize * 0.85f;
  const float dx = a.w * 0.10f;
  const float rowY = barTop + barH + smallSize * 0.95f;
  auto col = [&](float cxCol, float vL, float vR, const char* tag) {
    r.fillText(cxCol - dx, rowY,
               valid ? fmt("%.0f", std::round(vL)) : std::string("--"),
               smallSize, TextAlign::Center, colors::kWhite);
    r.fillText(cxCol + dx, rowY,
               valid ? fmt("%.0f", std::round(vR)) : std::string("--"),
               smallSize, TextAlign::Center, colors::kWhite);
    r.fillText(cxCol, rowY + smallSize * 1.2f, tag, smallSize,
               TextAlign::Center, colors::kLabelText);
    r.fillText(cxCol, rowY + smallSize * 2.3f, "Amp", smallSize,
               TextAlign::Center, colors::kLabelText);
  };
  col(batCx, bat1, bat2, "1 Bat 2");
  col(genCx, gen1, gen2, "1 Gen 2");
}

// Landing gear synoptic: nose gear on top, mains below, with the Vlo caption.
// Each position shows DN (green, down & locked), UP (white), or an amber state.
void drawGearBlock(Renderer& r, const FlightData& d, const Rect& a,
                   float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + labelSize * 0.6f, "Landing Gear",
             labelSize, TextAlign::Center, colors::kLabelText);
  auto state = [&](const char* ch) -> std::pair<std::string, Color> {
    const float v = chan(d, ch, 0.0f);
    if (v >= 0.95f) return {"DN", colors::kBandGreen};
    if (v <= 0.05f) return {"UP", colors::kWhite};
    return {"\xE2\x96\xA1", colors::kBandYellow};  // transit
  };
  const auto nose = state(eis_channels::kGearNose);
  const auto left = state(eis_channels::kGearLeft);
  const auto right = state(eis_channels::kGearRight);
  r.fillText(a.x + a.w * 0.5f, a.y + a.h * 0.38f, nose.first, labelSize,
             TextAlign::Center, nose.second);
  r.fillText(a.x + a.w * 0.27f, a.y + a.h * 0.62f, left.first, labelSize,
             TextAlign::Center, left.second);
  r.fillText(a.x + a.w * 0.73f, a.y + a.h * 0.62f, right.first, labelSize,
             TextAlign::Center, right.second);
  r.fillText(a.x + a.w * 0.5f, a.y + a.h * 0.90f, "Vlo extend: 210", labelSize,
             TextAlign::Center, colors::kLabelText);
}

// Vertical trim scale (pitch). -1..1 maps bottom(DN) .. top(UP); the green TO
// band and cyan pointer match the real pitch-trim indicator.
void drawPitchTrim(Renderer& r, const FlightData& d, const Rect& a,
                   float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + labelSize * 0.6f, "Pitch Trim", labelSize,
             TextAlign::Center, colors::kLabelText);
  const float scaleX = a.x + a.w * 0.62f;
  const float top = a.y + a.h * 0.28f;
  const float bot = a.y + a.h * 0.86f;
  r.fillText(scaleX, top - labelSize * 0.6f, "UP", labelSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(scaleX, bot + labelSize * 0.6f, "DN", labelSize, TextAlign::Center,
             colors::kLabelText);
  r.strokeLine(scaleX, top, scaleX, bot, 1.5f, colors::kPanelBorder);
  // Bracket end ticks at the UP/DN limits (the real scale reads as a "[").
  const float tickW = a.w * 0.10f;
  r.strokeLine(scaleX, top, scaleX + tickW, top, 1.5f, colors::kPanelBorder);
  r.strokeLine(scaleX, bot, scaleX + tickW, bot, 1.5f, colors::kPanelBorder);
  // Green takeoff band around the neutral region.
  const float midY = (top + bot) * 0.5f;
  r.strokeLine(scaleX - a.w * 0.05f, midY - (bot - top) * 0.12f,
               scaleX - a.w * 0.05f, midY + (bot - top) * 0.12f, 3.0f,
               colors::kBandGreen);

  const float t = std::max(-1.0f, std::min(1.0f, chan(d, eis_channels::kPitchTrim)));
  const float py = midY - t * (bot - top) * 0.5f;
  const float s = a.w * 0.07f;
  const Point ptr[3] = {{scaleX - a.w * 0.12f, py},
                        {scaleX - a.w * 0.12f - s, py - s},
                        {scaleX - a.w * 0.12f - s, py + s}};
  r.fillPolygon(ptr, 3, colors::kCyan);
  r.fillText(a.x + a.w * 0.30f, midY, fmt("%.0f\xC2\xB0", std::round(t * 10.0f)),
             mfdFontPx(kValueWt, displayH), TextAlign::Center, colors::kCyan);
}

// Flaps indicator: a swinging pointer with Up / 50% / 100% detents; the
// commanded detent is boxed green when the actual position agrees.
void drawFlaps(Renderer& r, const FlightData& d, const Rect& a, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(a.x + a.w * 0.40f, a.y + labelSize * 0.6f, "Flaps", labelSize,
             TextAlign::Center, colors::kLabelText);
  const float actual = clamp01(chan(d, eis_channels::kFlapsActual));
  const float pivotX = a.x + a.w * 0.18f;
  const float pivotY = a.y + a.h * 0.55f;
  const float len = a.w * 0.34f;
  const float ang = (-20.0f - actual * 70.0f) * kPi / 180.0f;  // up..down swing
  r.strokeLine(pivotX, pivotY, pivotX + len * std::cos(ang),
               pivotY - len * std::sin(ang), 3.0f, colors::kCyan);
  // The detent the flaps are actually at is boxed in green with black text, the
  // way the real unit highlights the active flap position (Fig. 3-2).
  const float rx = a.x + a.w * 0.66f;
  auto detent = [&](float yf, const char* text, bool active) {
    const float ty = a.y + a.h * yf;
    if (active) {
      const float tw = r.measureTextWidth(text, labelSize, FontFace::Default);
      r.fillRect(rx - labelSize * 0.2f, ty - labelSize * 0.6f,
                 tw + labelSize * 0.4f, labelSize * 1.2f, colors::kBandGreen);
    }
    r.fillText(rx, ty, text, labelSize, TextAlign::Left,
               active ? colors::kBlack : colors::kLabelText);
  };
  detent(0.30f, "Up", actual <= 0.1f);
  detent(0.58f, "50%", std::abs(actual - 0.5f) < 0.1f);
  detent(0.86f, "100%", actual >= 0.9f);
}

// Horizontal roll-trim arc with L .. R ends and a cyan pointer.
void drawRollTrim(Renderer& r, const FlightData& d, const Rect& a,
                  float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + labelSize * 0.6f, "Roll Trim", labelSize,
             TextAlign::Center, colors::kLabelText);
  const float cx = a.x + a.w * 0.5f;
  const float cy = a.y + a.h * 0.85f;
  const float radius = a.w * 0.30f;
  strokeArc(r, cx, cy, radius, -70.0f, 70.0f, 2.0f, colors::kPanelBorder);
  r.fillText(cx - radius * 1.05f, cy, "L", labelSize, TextAlign::Right,
             colors::kLabelText);
  r.fillText(cx + radius * 1.05f, cy, "R", labelSize, TextAlign::Left,
             colors::kLabelText);
  const float t = std::max(-1.0f, std::min(1.0f, chan(d, eis_channels::kRollTrim)));
  const float ang = (t * 70.0f) * kPi / 180.0f;
  const float bx = cx + radius * std::sin(ang);
  const float by = cy - radius * std::cos(ang);
  const Point ptr[3] = {{bx, by - 6.0f},
                        {bx - 5.0f, by + 3.0f},
                        {bx + 5.0f, by + 3.0f}};
  r.fillPolygon(ptr, 3, colors::kCyan);
}

// Cabin pressurization block: cabin altitude rate, cabin altitude, differential
// pressure, and the destination (landing field) elevation.
void drawCabinBlock(Renderer& r, const FlightData& d, const Rect& a, bool valid,
                    float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + labelSize * 0.6f, "Cabin Press", labelSize,
             TextAlign::Center, colors::kLabelText);

  const float rate = chan(d, eis_channels::kCabinRateFpm);
  const float alt = chan(d, eis_channels::kCabinAltFt);
  const float diff = chan(d, eis_channels::kCabinDiffPsi);
  const float dest = chan(d, eis_channels::kDestElevFt);

  // Diff PSI bar on the left edge and cabin Alt Ft bar on the right edge, each
  // with its value and caption stacked below (Fig. 3-2).
  const float barTop = a.y + a.h * 0.22f;
  const float barH = a.h * 0.42f;
  const float trackW = std::max(4.0f, a.w * 0.05f);
  const float diffCx = a.x + a.w * 0.10f;
  const float altCx = a.x + a.w * 0.90f;
  drawMiniVBar(r, diffCx, barTop, barH, trackW, 0.0f, kCabinDiffMax, 0.0f, false,
               diff, true, valid, 0.0f, 0.0f, 0.08f);
  drawMiniVBar(r, altCx, barTop, barH, trackW, 0.0f, kCabinAltMax, 0.0f, false,
               alt, true, valid, 0.0f, 0.15f, 0.07f);
  const float barValY = barTop + barH + valueSize * 0.9f;
  r.fillText(a.x, barValY, valid ? fmt("%.1f", diff) : std::string("-.-"),
             valueSize, TextAlign::Left, colors::kWhite);
  r.fillText(a.x, barValY + labelSize * 1.2f, "Diff PSI", labelSize,
             TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, barValY,
             valid ? fmt("%.0f", std::round(alt)) : std::string("----"),
             valueSize, TextAlign::Right, colors::kWhite);
  r.fillText(a.x + a.w, barValY + labelSize * 1.2f, "Alt Ft", labelSize,
             TextAlign::Right, colors::kLabelText);

  // Center column rows: Rate FPM, Dest Elev (magenta = FMS), each label above
  // its value so nothing collides in the narrow strip.
  const float cx = a.x + a.w * 0.5f;
  float y = a.y + a.h * 0.30f;
  r.fillText(cx, y, "Rate FPM", labelSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(cx, y + labelSize * 1.2f,
             valid ? fmt("%.0f", std::round(rate)) : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);
  y = a.y + a.h * 0.62f;
  r.fillText(cx, y, "Dest Elev", labelSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(cx, y + labelSize * 1.2f,
             valid ? fmt("%.0f", std::round(dest)) : std::string("-----"),
             valueSize, TextAlign::Center, colors::kMagenta);
}

}  // namespace

// The full turbofan turbine row, left to right. The reduced reversionary strip
// shows the subset whose gauge is flagged PFD in the .eis file.
constexpr int kTurbineBarCount = 5;
const char* const kTurbineBarChans[kTurbineBarCount] = {
    eis_channels::kN1Pct, eis_channels::kN2Pct, eis_channels::kIttC,
    eis_channels::kOilTempC, eis_channels::kOilPres};
const char* const kTurbineBarLabels[kTurbineBarCount] = {
    "N1%", "N2%", "ITT\xC2\xB0""C", "Oil\xC2\xB0""C", "Oil PSI"};

// Turbine bars row, drawing the `n` channels in `chans` evenly across the row.
void drawTurbineBars(Renderer& r, const FlightData& d, const EisLayout& layout,
                     const Rect& barsRow, bool valid, float displayH,
                     const char* const chans[], const char* const labels[],
                     int n) {
  if (n <= 0) return;
  for (int i = 0; i < n; ++i) {
    const Rect cell{barsRow.x + barsRow.w * (static_cast<float>(i) / n),
                    barsRow.y, barsRow.w / n, barsRow.h};
    const EisGauge* g = layout.gaugeForChannel(chans[i]);
    drawVertBar(r, cell, g, chan(d, chans[i]), valid, displayH, labels[i]);
  }
}

// Is the gauge bound to `channel` flagged PFD (a primary reversionary gauge)?
bool isPfdGauge(const EisLayout& layout, const char* channel) {
  const EisGauge* g = layout.gaugeForChannel(channel);
  return g != nullptr && g->pfd;
}

// Reversionary (display-backup) strip on the PFD: stack only the PFD-flagged
// primary engine blocks (% thrust arc, the flagged turbine bars, and fuel)
// down the taller column, dropping the airframe synoptics that do not fit.
void drawEisStripTurbofanReduced(Renderer& r, const FlightData& d,
                                 const EisLayout& layout, const Rect& area,
                                 const Rect& a, bool valid, float displayH) {
  const bool showThrust = isPfdGauge(layout, eis_channels::kThrustPct);

  const char* barChans[kTurbineBarCount];
  const char* barLabels[kTurbineBarCount];
  int barCount = 0;
  for (int i = 0; i < kTurbineBarCount; ++i) {
    if (isPfdGauge(layout, kTurbineBarChans[i])) {
      barChans[barCount] = kTurbineBarChans[i];
      barLabels[barCount] = kTurbineBarLabels[i];
      ++barCount;
    }
  }
  const bool showFuel = isPfdGauge(layout, eis_channels::kFuelQtyLeft) ||
                        isPfdGauge(layout, eis_channels::kFuelQtyRight);

  // Lay the present blocks out vertically with proportional weights and a thin
  // rule between each, filling the column from ~2% to ~98% of its height.
  struct Block {
    char kind;     // 't' thrust, 'b' bars, 'f' fuel
    float weight;  // relative vertical share
  };
  Block blocks[3];
  int n = 0;
  if (showThrust) blocks[n++] = {'t', 0.85f};
  if (barCount > 0) blocks[n++] = {'b', 1.25f};
  if (showFuel) blocks[n++] = {'f', 1.15f};
  if (n == 0) return;

  constexpr float kTop = 0.02f;
  constexpr float kBottom = 0.98f;
  constexpr float kGap = 0.04f;  // vertical gap (with rule) between blocks
  float totalWeight = 0.0f;
  for (int i = 0; i < n; ++i) totalWeight += blocks[i].weight;
  const float usable = (kBottom - kTop) - kGap * (n - 1);

  float f = kTop;
  for (int i = 0; i < n; ++i) {
    const float share = usable * (blocks[i].weight / totalWeight);
    const Rect band{a.x, a.y + a.h * f, a.w, a.h * share};
    switch (blocks[i].kind) {
      case 't':
        drawThrustArc(r, d, layout, band, valid, displayH);
        break;
      case 'b':
        drawTurbineBars(r, d, layout, band, valid, displayH, barChans,
                        barLabels, barCount);
        break;
      case 'f':
        drawFuelBlock(r, d, layout, band, valid, displayH);
        break;
    }
    f += share;
    if (i + 1 < n) {
      rule(r, area, a.y + a.h * (f + kGap * 0.5f));
      f += kGap;
    }
  }
}

void drawEisStripTurbofan(Renderer& r, const FlightData& d,
                          const EisLayout& layout, const Rect& area,
                          float displayH, bool reduced) {
  // The EIS strip is solid black on the real unit (no panel gradient).
  r.fillRect(area.x, area.y, area.w, area.h, colors::kBlack);
  r.strokeLine(area.x + area.w, area.y, area.x + area.w, area.y + area.h, 2.0f,
               colors::kPanelBorder);

  const bool valid = d.dataLinkValid;
  const float pad = area.w * 0.04f;
  const Rect a{area.x + pad, area.y, area.w - 2.0f * pad, area.h};

  // Vertical band fractions of the strip, top to bottom (Fig. 3-2 stacking).
  auto band = [&](float f0, float f1) {
    return Rect{a.x, a.y + a.h * f0, a.w, a.h * (f1 - f0)};
  };

  // PFD reversionary (display-backup) mode: the full synoptic grid does not fit
  // beside the flight instruments, so only the PFD-flagged primary engine
  // blocks are shown. The airframe synoptics (electrical, gear, trim, flaps,
  // pressurization) are not gauges and so are always omitted here.
  if (reduced) {
    drawEisStripTurbofanReduced(r, d, layout, area, a, valid, displayH);
    return;
  }

  drawThrustArc(r, d, layout, band(0.00f, 0.165f), valid, displayH);
  rule(r, area, a.y + a.h * 0.165f);

  drawTurbineBars(r, d, layout, band(0.175f, 0.325f), valid, displayH,
                  kTurbineBarChans, kTurbineBarLabels, kTurbineBarCount);
  rule(r, area, a.y + a.h * 0.335f);

  // Fuel (left) + electrical (right).
  const Rect feRow = band(0.345f, 0.515f);
  drawFuelBlock(r, d, layout, Rect{feRow.x, feRow.y, feRow.w * 0.52f, feRow.h},
                valid, displayH);
  r.strokeLine(feRow.x + feRow.w * 0.53f, feRow.y, feRow.x + feRow.w * 0.53f,
               feRow.y + feRow.h, 1.0f, colors::kPanelSeparator);
  drawElecBlock(r, d,
                Rect{feRow.x + feRow.w * 0.55f, feRow.y, feRow.w * 0.45f,
                     feRow.h},
                valid, displayH);
  rule(r, area, a.y + a.h * 0.525f);

  // Landing gear (left) + pitch trim (right).
  const Rect gtRow = band(0.535f, 0.655f);
  drawGearBlock(r, d, Rect{gtRow.x, gtRow.y, gtRow.w * 0.52f, gtRow.h},
                displayH);
  r.strokeLine(gtRow.x + gtRow.w * 0.53f, gtRow.y, gtRow.x + gtRow.w * 0.53f,
               gtRow.y + gtRow.h, 1.0f, colors::kPanelSeparator);
  drawPitchTrim(r, d,
                Rect{gtRow.x + gtRow.w * 0.55f, gtRow.y, gtRow.w * 0.45f,
                     gtRow.h},
                displayH);
  rule(r, area, a.y + a.h * 0.665f);

  // Flaps (left) + roll trim (right).
  const Rect frRow = band(0.675f, 0.775f);
  drawFlaps(r, d, Rect{frRow.x, frRow.y, frRow.w * 0.52f, frRow.h}, displayH);
  r.strokeLine(frRow.x + frRow.w * 0.53f, frRow.y, frRow.x + frRow.w * 0.53f,
               frRow.y + frRow.h, 1.0f, colors::kPanelSeparator);
  drawRollTrim(r, d,
               Rect{frRow.x + frRow.w * 0.55f, frRow.y, frRow.w * 0.45f,
                    frRow.h},
               displayH);
  rule(r, area, a.y + a.h * 0.785f);

  // Cabin pressurization.
  drawCabinBlock(r, d, band(0.795f, 0.99f), valid, displayH);
}

}  // namespace avionics::mfd
