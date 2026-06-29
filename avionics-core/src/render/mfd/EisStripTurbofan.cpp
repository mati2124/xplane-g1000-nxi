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
constexpr float kBarLabelWt = 13.0f; // the five turbine bar captions (N1%, ...)
constexpr float kBarValueWt = 19.0f; // the turbine bar numeric readouts
constexpr float kValueWt = 11.0f;   // numeric readouts (fuel/electrical blocks)
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

// Turbine-bar palette sampled pixel-for-pixel from the real Perspective Touch+
// EIS (Pilot's Guide Fig. 3-1): the normal-range band is a grass green
// rgb(105,189,69) and the redline ticks are rgb(237,28,36), both noticeably
// different from the generic airspeed-tape bands.
constexpr Color kEisGreen{0.412f, 0.741f, 0.271f, 1.0f};
constexpr Color kEisRed{0.929f, 0.110f, 0.141f, 1.0f};
// The gauge frame is a dim "[" bracket: a dark vertical scale spine with
// brighter horizontal end caps (sampled ~rgb(64,64,64) spine, ~rgb(120,120,120)
// caps).
constexpr Color kEisSpine{0.255f, 0.255f, 0.255f, 1.0f};
constexpr Color kEisCap{0.471f, 0.471f, 0.471f, 1.0f};

std::string fmt(const char* pattern, double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), pattern, v);
  return buf;
}

std::string fmtNoZeroSign(const char* pattern, double v) {
  if (std::round(v) == 0.0) return "0";
  return fmt(pattern, v);
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

// One vertical turbine/parameter bar (N1, N2, ITT, Oil °C, Oil PSI), modeled
// pixel-for-pixel on the real Perspective Touch+ EIS (Pilot's Guide Fig. 3-1):
//   - a dim "[" bracket frame (dark vertical scale spine + brighter horizontal
//     end caps) over a black background, NOT a filled track;
//   - a thin grass-green band showing the normal operating range, drawn snug
//     against the left spine;
//   - thin red limit ticks at the green-facing edge of each red zone (the oil
//     gauges carry both a low and a high tick);
//   - a white pointer pointing left, its tip at the green band's right edge;
//   - an optional cyan takeoff "⊢" reference bug to the left of the spine (N1).
// The numeric value sits below the track, the label below that.
void drawVertBar(Renderer& r, const Rect& cell, const EisGauge* g, float value,
                 bool valid, float displayH, const char* labelOverride) {
  const float labelSize = mfdFontPx(kBarLabelWt, displayH);
  const float valueSize = mfdFontPx(kBarValueWt, displayH);

  const float minV = g ? g->min : 0.0f;
  const float maxV = g ? g->max : 100.0f;
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;

  // "[" bracket geometry: the spine sits at trackX, caps span trackW to the
  // right. Leave room below the track for the value readout + caption.
  const float trackW = std::max(6.0f, cell.w * 0.42f);
  const float trackX = cell.x + cell.w * 0.26f;
  const float trackTop = cell.y + labelSize * 0.15f;
  const float trackH = cell.h - valueSize * 1.35f - labelSize * 1.25f;
  const float trackBot = trackTop + trackH;
  auto yFor = [&](float v) {
    return trackTop + trackH * (1.0f - clamp01((v - minV) / span));
  };

  // Grass-green normal-range band(s): a thin bar against the spine. Red zones
  // are rendered as ticks below, not as filled blocks.
  const float greenW = std::max(3.0f, trackW * 0.36f);
  if (g != nullptr) {
    for (const EisBand& b : g->bands) {
      if (b.color == EisBandColor::Red) continue;
      const Color c =
          b.color == EisBandColor::Green ? kEisGreen : eisBandColor(b.color);
      const float y0 = yFor(b.hi);
      const float y1 = yFor(b.lo);
      r.fillRect(trackX, y0, greenW, y1 - y0, c);
    }
  }

  // The "[" bracket: dim vertical spine, brighter top & bottom caps.
  r.strokeLine(trackX, trackTop, trackX, trackBot, 1.4f, kEisSpine);
  r.strokeLine(trackX, trackTop, trackX + trackW, trackTop, 1.4f, kEisCap);
  r.strokeLine(trackX, trackBot, trackX + trackW, trackBot, 1.4f, kEisCap);

  // Red limit ticks at the green-facing edge of each red zone: a low red zone
  // (touching the scale minimum) ticks at its top, a high red zone at its
  // bottom. Spans the full track width like the real redline.
  if (g != nullptr) {
    for (const EisBand& b : g->bands) {
      if (b.color != EisBandColor::Red) continue;
      const float limit = (b.lo <= minV + 1e-3f) ? b.hi : b.lo;
      const float ry = yFor(limit);
      r.strokeLine(trackX, ry, trackX + trackW, ry, 2.0f, kEisRed);
    }
  }

  // White value pointer: points left, tip at the green band's right edge, base
  // at the track's right cap. Turns red once at/over the redline.
  if (valid) {
    const float vy = yFor(value);
    const float tipX = trackX + greenW;
    const float baseX = trackX + trackW;
    // Slightly wider than tall, matching the real EIS carrots: on the native
    // strip the N2 carrot is ~9 px long × ~7 px tall, so half-height ≈ 0.40 of
    // its length (was 0.55 = too tall).
    const float ph = (baseX - tipX) * 0.40f;
    const Point ptr[3] = {{tipX, vy},
                          {baseX, vy - ph},
                          {baseX, vy + ph}};
    const bool over = g != nullptr && g->hasRedline && value >= g->redline;
    r.fillPolygon(ptr, 3, over ? kEisRed : colors::kWhite);

    // Cyan takeoff "⊢" reference bug: a tall vertical bar left of the spine
    // with a stem reaching in to the scale (N1 takeoff-thrust setting).
    if (g != nullptr && g->hasBug) {
      const float by = yFor(g->bug);
      const float bx = trackX - trackW * 0.46f;
      const float armH = trackH * 0.085f;
      r.strokeLine(bx, by - armH, bx, by + armH, 2.6f, colors::kCyan);
      r.strokeLine(bx, by, trackX, by, 2.6f, colors::kCyan);
    }
  }

  const float cx = trackX + trackW * 0.5f;
  const float valY = trackBot + valueSize * 0.95f;
  r.fillText(cx, valY,
             valid ? fmt(g ? g->format.c_str() : "%.0f", value)
                   : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);
  const char* lbl = labelOverride ? labelOverride : (g ? g->label.c_str() : "");
  r.fillText(cx, valY + labelSize * 1.4f, lbl, labelSize, TextAlign::Center,
             colors::kLabelText);
}

// Shared "[" bracket gauge chrome for every vertical EIS bar (turbine row, fuel
// quantity/flow, electrical amps): a dim spine + brighter top/bottom caps over
// black, grass-green (and amber) range bands snug against the spine, and thin
// red limit ticks at each red zone's green-facing edge. Returns the green
// band's right-edge x so callers can anchor pointers to it.
// When `greenRight` is set the green/amber fill and spine sit on the RIGHT of
// the track (the bars mirror so a value pointer can hug the outer green edge,
// as the real fuel "R" tank and right-hand amp sources do). Returns the green
// fill's OUTER edge x (right edge normally, left edge when mirrored) so callers
// can anchor pointers snug against the colored fill.
float drawBracketTrack(Renderer& r, float trackX, float trackTop, float trackW,
                       float trackH, float minV, float maxV,
                       const std::vector<EisBand>& bands,
                       float bandFrac = 0.36f, bool greenRight = false) {
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;
  auto yFor = [&](float v) {
    return trackTop + trackH * (1.0f - clamp01((v - minV) / span));
  };
  const float greenW = std::max(3.0f, trackW * bandFrac);
  const float greenX = greenRight ? trackX + trackW - greenW : trackX;
  const float spineX = greenRight ? trackX + trackW : trackX;
  for (const EisBand& b : bands) {
    if (b.color == EisBandColor::Red) continue;
    const Color c =
        b.color == EisBandColor::Green ? kEisGreen : colors::kBandYellow;
    r.fillRect(greenX, yFor(b.hi), greenW, yFor(b.lo) - yFor(b.hi), c);
  }
  r.strokeLine(spineX, trackTop, spineX, trackTop + trackH, 1.4f, kEisSpine);
  r.strokeLine(trackX, trackTop, trackX + trackW, trackTop, 1.4f, kEisCap);
  r.strokeLine(trackX, trackTop + trackH, trackX + trackW, trackTop + trackH,
               1.4f, kEisCap);
  for (const EisBand& b : bands) {
    if (b.color != EisBandColor::Red) continue;
    const float limit = (b.lo <= minV + 1e-3f) ? b.hi : b.lo;
    r.strokeLine(trackX, yFor(limit), trackX + trackW, yFor(limit), 2.0f,
                 kEisRed);
  }
  return greenRight ? greenX : trackX + greenW;
}

// White value pointer pointing LEFT (tip on the left at tipX, base len to the
// right) / RIGHT (tip on the right). Used to anchor a pointer to a bracket bar.
// Half-height as a fraction of the pointer's length: the real EIS carrots are
// slightly wider than tall (native ~9 px long × ~7 px tall), so the half-height
// is ~0.40*len.
constexpr float kPtrHalf = 0.40f;
void ptrLeft(Renderer& r, float tipX, float vy, float len, const Color& c) {
  const Point p[3] = {{tipX, vy},
                      {tipX + len, vy - len * kPtrHalf},
                      {tipX + len, vy + len * kPtrHalf}};
  r.fillPolygon(p, 3, c);
}
void ptrRight(Renderer& r, float tipX, float vy, float len, const Color& c) {
  const Point p[3] = {{tipX, vy},
                      {tipX - len, vy - len * kPtrHalf},
                      {tipX - len, vy + len * kPtrHalf}};
  r.fillPolygon(p, 3, c);
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
  // Caret half-height ≈ 0.40 of its length (slightly wider than tall), matching
  // the real EIS carrots (was 0.75 = too tall).
  const float ch = (1.2f - 0.15f) * cw * 0.40f;
  if (hasL) {
    const float vy = yFor(vL);
    const Point p[3] = {{x - cw * 0.15f, vy},
                        {x - cw * 1.2f, vy - ch},
                        {x - cw * 1.2f, vy + ch}};
    r.fillPolygon(p, 3, colors::kWhite);
  }
  if (hasR) {
    const float vy = yFor(vR);
    const float xr = x + trackW;
    const Point p[3] = {{xr + cw * 0.15f, vy},
                        {xr + cw * 1.2f, vy - ch},
                        {xr + cw * 1.2f, vy + ch}};
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

// Total-fuel bracket (Fig. 3-1): a horizontal connector spanning the two tank
// columns, with short verticals dropping down the LEFT and RIGHT sides of the
// centered total value so the value sits nested between them. Drawn in the dim
// cap-gray (not stark white) like the real unit's frame lines.
void drawFuelTotalBracket(Renderer& r, float lCx, float rCx, float qtyRowY,
                          float qtySize, float totalY, float totalSize,
                          const std::string& tot) {
  const float totCx = (lCx + rCx) * 0.5f;
  const float tw = r.measureTextWidth(tot, totalSize);
  // Corner brackets flanking the total (Fig. 3-1): each vertical runs from just
  // above the total down to its vertical MIDDLE, where a short foot turns inward
  // toward the digits. Dim cap-gray, like the real unit's frame lines.
  (void)qtyRowY;
  (void)qtySize;
  // Digits use NVG_ALIGN_MIDDLE and have no descender, so their visual center
  // rides ~0.2em above totalY; place the feet there so they hit the true middle.
  const float visMid = totalY - totalSize * 0.20f;
  const float vertTop = visMid - totalSize * 0.52f;  // a touch above the digits
  const float vertBot = visMid;                       // foot at mid-height
  const float footIn = totalSize * 0.24f;
  const float gap = totalSize * 0.24f;
  const float lx = totCx - tw * 0.5f - gap;
  const float rx = totCx + tw * 0.5f + gap;
  const float lw = 1.3f;
  r.strokeLine(lx, vertTop, lx, vertBot, lw, kEisCap);
  r.strokeLine(rx, vertTop, rx, vertBot, lw, kEisCap);
  r.strokeLine(lx, vertBot, lx + footIn, vertBot, lw, kEisCap);
  r.strokeLine(rx, vertBot, rx - footIn, vertBot, lw, kEisCap);
  r.fillText(totCx, totalY, tot, totalSize, TextAlign::Center, colors::kWhite);
}

// Two-tank fuel block (Pilot's Guide Fig. 3-1): a tall fuel-flow (GPH) bracket
// bar on the far left, then the L/R per-tank quantity bracket bars (grass-green
// with an amber low-caution band and a red low tick), with the GPH/qty/total
// readouts and fuel temperature below.
void drawFuelBlock(Renderer& r, const FlightData& d, const EisLayout& layout,
                   const Rect& a, bool valid, float displayH) {
  // Font weights calibrated from the real Perspective Touch+ EIS strip
  // (851×537 native): ink-run height × 1.6 ≈ mfdFontPx weight at 768 canvas.
  const float hdrSize = mfdFontPx(13.0f, displayH);   // "L" / "R" headers
  const float qtySize = mfdFontPx(14.0f, displayH);   // per-tank 105 readouts
  const float totSize = mfdFontPx(15.0f, displayH);   // bracketed total
  const float gphValSize = mfdFontPx(13.0f, displayH); // flow numeric
  const float labelSize = mfdFontPx(13.0f, displayH);  // GPH / Fuel GAL

  const EisGauge* gl = layout.gaugeForChannel(eis_channels::kFuelQtyLeft);
  const float fmaxV = gl ? gl->max : 150.0f;
  const float fminV = gl ? gl->min : 0.0f;
  const std::vector<EisBand> emptyBands;
  const std::vector<EisBand>& fBands = gl ? gl->bands : emptyBands;

  const float qL = chan(d, eis_channels::kFuelQtyLeft);
  const float qR = chan(d, eis_channels::kFuelQtyRight);
  const float gph = chan(d, eis_channels::kFuelFlow);
  const float ftemp = chan(d, eis_channels::kFuelTempC);

  // Bar centers and track widths as fractions of the (narrow) fuel column,
  // measured from the real strip (fuel col native x=6..73): GPH cx≈0.11,
  // L≈0.558, R≈0.779; L/R track≈0.134w, GPH track≈0.119w; green ≈56 % track.
  // The L/R pair is pulled inward from the old 0.47/0.84 so the R tank's
  // outside carrot clears the fuel/electrical divider (was touching it), while
  // staying far enough apart that the two quantity readouts don't collide.
  const float gphCx = a.x + a.w * 0.10f;
  const float lCx = a.x + a.w * 0.50f;
  const float rCx = a.x + a.w * 0.75f;
  const float lrTrackW = std::max(4.0f, a.w * 0.134f);
  const float gphTrackW = std::max(4.0f, a.w * 0.119f);
  constexpr float kFuelBandFrac = 0.56f;
  constexpr float kGphBandFrac = 0.50f;

  const float headerY = a.y + hdrSize * 0.85f;
  const float trackTop = a.y + hdrSize * 1.65f;
  const float gphTop = a.y + hdrSize * 0.15f;
  const float trackBot = a.y + a.h * 0.58f;
  const float trackH = trackBot - trackTop;
  const float gphH = trackBot - gphTop;
  const float ptrLen = lrTrackW * 0.85f;

  // GPH fuel-flow bar (taller, starts higher than L/R).
  constexpr float kGphMax = 100.0f;
  {
    const std::vector<EisBand> gb{{EisBandColor::Green, 0.0f, kGphMax}};
    const float gx = gphCx - gphTrackW * 0.5f;
    const float gRight = drawBracketTrack(r, gx, gphTop, gphTrackW, gphH, 0.0f,
                                          kGphMax, gb, kGphBandFrac);
    if (valid) {
      const float vy = gphTop + gphH * (1.0f - clamp01(gph / kGphMax));
      // Pointer snug against the right edge of the (thin) green fill, not the
      // wide track edge, matching the real unit.
      const float plen = gphTrackW * 0.8f;
      ptrLeft(r, gRight + gphTrackW * 0.04f, vy, plen, colors::kWhite);
    }
  }

  r.fillText(lCx, headerY, "L", hdrSize, TextAlign::Center, colors::kWhite);
  r.fillText(rCx, headerY, "R", hdrSize, TextAlign::Center, colors::kWhite);
  const float fspan = (fmaxV - fminV) != 0.0f ? (fmaxV - fminV) : 1.0f;
  auto fuelBar = [&](float cxBar, float qty, bool isLeft) {
    const float x = cxBar - lrTrackW * 0.5f;
    // Mirror the R tank so its green hugs the track's right edge; the L tank
    // green hugs the left edge. Each carrot's tip then sits snug against the
    // green's OUTER edge (left for L, right for R), matching the real unit.
    drawBracketTrack(r, x, trackTop, lrTrackW, trackH, fminV, fmaxV, fBands,
                     kFuelBandFrac, /*greenRight=*/!isLeft);
    if (valid) {
      const float vy =
          trackTop + trackH * (1.0f - clamp01((qty - fminV) / fspan));
      if (isLeft)
        ptrRight(r, x - ptrLen * 0.04f, vy, ptrLen, colors::kWhite);
      else
        ptrLeft(r, x + lrTrackW + ptrLen * 0.04f, vy, ptrLen, colors::kWhite);
    }
  };
  fuelBar(lCx, qL, true);
  fuelBar(rCx, qR, false);

  // Per-tank quantities, then the inverted-U total bracket, then "Fuel GAL".
  const float qtyRowY = trackBot + qtySize * 0.85f;
  r.fillText(lCx, qtyRowY, valid ? fmt("%.0f", std::round(qL)) : "---", qtySize,
             TextAlign::Center, colors::kWhite);
  r.fillText(rCx, qtyRowY, valid ? fmt("%.0f", std::round(qR)) : "---", qtySize,
             TextAlign::Center, colors::kWhite);

  const float totCx = (lCx + rCx) * 0.5f;
  const float totY = qtyRowY + qtySize * 1.55f;
  const std::string tot = valid ? fmt("%.0f", std::round(qL + qR)) : "---";
  drawFuelTotalBracket(r, lCx, rCx, qtyRowY, qtySize, totY, totSize, tot);

  r.fillText(totCx, totY + totSize * 0.95f, "Fuel GAL", labelSize,
             TextAlign::Center, colors::kLabelText);

  // GPH value / label / fuel temp under the flow bar.
  const float gphRowY = trackBot + gphValSize * 0.85f;
  r.fillText(gphCx, gphRowY, valid ? fmt("%.0f", std::round(gph)) : "--",
             gphValSize, TextAlign::Center, colors::kWhite);
  r.fillText(gphCx, gphRowY + gphValSize * 0.95f, "GPH", labelSize,
             TextAlign::Center, colors::kLabelText);
  r.fillText(gphCx, gphRowY + gphValSize * 0.95f + labelSize * 1.15f,
             valid ? fmt("%.0f\xC2\xB0""C", std::round(ftemp)) : "--\xC2\xB0""C",
             labelSize, TextAlign::Center, colors::kWhite);
}

// Four-source electrical block (Pilot's Guide Fig. 3-1): emergency bus volts in
// the header, then the Battery 1/2 and Generator 1/2 dual-source amp bars drawn
// in the same "[" bracket style (the battery bar carries a low amber band).
void drawElecBlock(Renderer& r, const FlightData& d, const Rect& a, bool valid,
                   float displayH) {
  // Same calibration as drawFuelBlock (real ink height × 1.6).
  const float hdrSize = mfdFontPx(16.0f, displayH);  // Emr Bus V + volts
  const float ampSize = mfdFontPx(15.0f, displayH);  // per-source amps
  const float labelSize = mfdFontPx(13.0f, displayH); // 1 Bat 2 / Amp

  const float emer = chan(d, eis_channels::kEmerBusVolts);
  const float hY = a.y + hdrSize * 0.85f;
  const std::string voltsStr = valid ? fmt("%.1f", emer) : std::string("--.-");
  const float voltsW = r.measureTextWidth(voltsStr, hdrSize);
  const float gap = a.w * 0.04f;
  float capWt = 16.0f;
  const float capW = r.measureTextWidth("Emr Bus V", mfdFontPx(capWt, displayH));
  if (capW + voltsW + gap > a.w && capW > 0.0f)
    capWt = std::max(13.0f, capWt * (a.w - voltsW - gap) / capW);
  const float capSize = mfdFontPx(capWt, displayH);
  r.fillText(a.x, hY, "Emr Bus V", capSize, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, hY, voltsStr, hdrSize, TextAlign::Right, colors::kWhite);

  const float bat1 = chan(d, eis_channels::kBatt1Amps);
  const float bat2 = chan(d, eis_channels::kBatt2Amps);
  const float gen1 = chan(d, eis_channels::kGen1Amps);
  const float gen2 = chan(d, eis_channels::kGen2Amps);
  // Centers and track width from real strip, relative to the (wider) electrical
  // column (native x=73..181): Bat cx≈0.319, Gen≈0.815, track≈0.120w.
  const float batCx = a.x + a.w * 0.319f;
  const float genCx = a.x + a.w * 0.815f;
  const float trackW = std::max(4.0f, a.w * 0.120f);
  constexpr float kAmpBandFrac = 0.31f;
  const float trackTop = hY + hdrSize * 0.65f;
  const float trackBot = a.y + a.h * 0.58f;
  const float trackH = trackBot - trackTop;
  const float ptrLen = trackW * 0.85f;

  auto ampBar = [&](float cx, float vL, float vR, float maxA, bool amber) {
    const float x = cx - trackW * 0.5f;
    std::vector<EisBand> bands;
    if (amber) {
      bands.push_back({EisBandColor::Yellow, 0.0f, maxA * 0.40f});
      bands.push_back({EisBandColor::Green, maxA * 0.40f, maxA});
    } else {
      bands.push_back({EisBandColor::Green, 0.0f, maxA});
    }
    const float gRight = drawBracketTrack(r, x, trackTop, trackW, trackH, 0.0f,
                                          maxA, bands, kAmpBandFrac);
    if (!valid) return;
    auto yf = [&](float v) {
      return trackTop + trackH * (1.0f - clamp01(v / maxA));
    };
    // Both source carrots flank the (narrow) central green: the left source
    // tip hugs the green's left edge, the right source tip its right edge,
    // rather than the wide cap edges (real unit).
    ptrRight(r, x - ptrLen * 0.04f, yf(vL), ptrLen, colors::kWhite);
    ptrLeft(r, gRight + ptrLen * 0.04f, yf(vR), ptrLen, colors::kWhite);
  };
  ampBar(batCx, bat1, bat2, kBattAmpsMax, true);
  ampBar(genCx, gen1, gen2, kGenAmpsMax, false);

  const float dx = a.w * 0.13f;
  const float rowY = trackBot + ampSize * 0.85f;
  auto col = [&](float cxCol, float vL, float vR, const char* tag) {
    r.fillText(cxCol - dx, rowY,
               valid ? fmtNoZeroSign("%.0f", vL) : std::string("--"), ampSize,
               TextAlign::Center, colors::kWhite);
    r.fillText(cxCol + dx, rowY,
               valid ? fmtNoZeroSign("%.0f", vR) : std::string("--"), ampSize,
               TextAlign::Center, colors::kWhite);
    r.fillText(cxCol, rowY + ampSize * 1.2f, tag, labelSize, TextAlign::Center,
               colors::kLabelText);
    r.fillText(cxCol, rowY + ampSize * 1.2f + labelSize * 1.05f, "Amp",
               labelSize, TextAlign::Center, colors::kLabelText);
  };
  col(batCx, bat1, bat2, "1 Bat 2");
  col(genCx, gen1, gen2, "1 Gen 2");
}

// Landing gear synoptic: nose gear on top, mains below, with the Vlo caption.
// Each position shows DN (green, down & locked), UP (white), or an amber state.
void drawGearBlock(Renderer& r, const FlightData& d, const Rect& a,
                   float displayH) {
  const float hdrSize = mfdFontPx(13.0f, displayH);
  const float discSize = mfdFontPx(12.0f, displayH);
  const float vloSize = mfdFontPx(11.0f, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + hdrSize * 0.85f, "Landing Gear", hdrSize,
             TextAlign::Center, colors::kLabelText);
  // Each gear position renders as a disc, matching the real unit: a green
  // filled disc with black "DN" when down, an outlined disc with white "UP"
  // when up, and an amber outline in transit.
  enum class GearState { Down, Up, Transit };
  auto state = [&](const char* ch) -> GearState {
    const float v = chan(d, ch, 0.0f);
    if (v >= 0.95f) return GearState::Down;
    if (v <= 0.05f) return GearState::Up;
    return GearState::Transit;
  };
  const float discR = std::min(a.w * 0.16f, a.h * 0.17f);
  auto disc = [&](float cx, float cy, GearState st) {
    if (st == GearState::Down) {
      r.fillCircle(cx, cy, discR, colors::kBandGreen);
      r.fillText(cx, cy + discSize * 0.36f, "DN", discSize, TextAlign::Center,
                 colors::kBlack);
    } else {
      const Color c =
          st == GearState::Up ? colors::kWhite : colors::kBandYellow;
      strokeCircle(r, cx, cy, discR, 1.8f, c);
      r.fillText(cx, cy + discSize * 0.36f, st == GearState::Up ? "UP" : "  ",
                 discSize, TextAlign::Center, c);
    }
  };
  // Triangle layout: nose top-center, mains lower-left / lower-right.
  disc(a.x + a.w * 0.5f, a.y + a.h * 0.42f, state(eis_channels::kGearNose));
  disc(a.x + a.w * 0.28f, a.y + a.h * 0.68f, state(eis_channels::kGearLeft));
  disc(a.x + a.w * 0.72f, a.y + a.h * 0.68f, state(eis_channels::kGearRight));
  r.fillText(a.x + a.w * 0.5f, a.y + a.h * 0.96f, "Vlo extend: 210", vloSize,
             TextAlign::Center, colors::kLabelText);
}

// Vertical trim scale (pitch). -1..1 maps bottom(DN) .. top(UP); the green TO
// band and cyan pointer match the real pitch-trim indicator.
void drawPitchTrim(Renderer& r, const FlightData& d, const Rect& a,
                   float displayH) {
  const float hdrSize = mfdFontPx(13.0f, displayH);
  const float lblSize = mfdFontPx(12.0f, displayH);
  const float valSize = mfdFontPx(15.0f, displayH);
  r.fillText(a.x + a.w * 0.5f, a.y + hdrSize * 0.85f, "Pitch Trim", hdrSize,
             TextAlign::Center, colors::kLabelText);
  const float scaleX = a.x + a.w * 0.66f;
  const float top = a.y + a.h * 0.32f;
  const float bot = a.y + a.h * 0.86f;
  r.fillText(scaleX, top - lblSize * 0.5f, "UP", lblSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(scaleX, bot + lblSize * 1.15f, "DN", lblSize, TextAlign::Center,
             colors::kLabelText);
  r.strokeLine(scaleX, top, scaleX, bot, 1.6f, colors::kPanelBorder);
  // Bracket end ticks at the UP/DN limits (the real scale reads as a "[").
  const float tickW = a.w * 0.10f;
  r.strokeLine(scaleX, top, scaleX + tickW, top, 1.6f, colors::kPanelBorder);
  r.strokeLine(scaleX, bot, scaleX + tickW, bot, 1.6f, colors::kPanelBorder);
  // Green takeoff band on the scale at the neutral region, labelled "TO" to
  // its left (matches the real pitch-trim indicator).
  const float midY = (top + bot) * 0.5f;
  const float toH = (bot - top) * 0.13f;
  r.strokeLine(scaleX - a.w * 0.05f, midY - toH, scaleX - a.w * 0.05f,
               midY + toH, 4.0f, colors::kBandGreen);
  r.fillText(a.x + a.w * 0.04f, midY + lblSize * 0.36f, "TO", lblSize,
             TextAlign::Left, colors::kLabelText);

  const float t =
      std::max(-1.0f, std::min(1.0f, chan(d, eis_channels::kPitchTrim)));
  const float py = midY - t * (bot - top) * 0.5f;
  // Cyan pointer pointing right toward the scale.
  const float s = a.w * 0.09f;
  const Point ptr[3] = {{scaleX - a.w * 0.10f, py},
                        {scaleX - a.w * 0.10f - s, py - s},
                        {scaleX - a.w * 0.10f - s, py + s}};
  r.fillPolygon(ptr, 3, colors::kCyan);
  r.fillText(a.x + a.w * 0.34f, midY + valSize * 0.36f,
             fmt("%.0f\xC2\xB0", std::round(t * 10.0f)), valSize,
             TextAlign::Center, colors::kCyan);
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
// `groupAfter` is the 0-based bar index after which a tall group divider is
// drawn (the real EIS splits the engine-speed group N1/N2 from the temp/
// pressure group ITT/Oil °C/Oil PSI with a vertical rule); -1 draws none.
void drawTurbineBars(Renderer& r, const FlightData& d, const EisLayout& layout,
                     const Rect& barsRow, bool valid, float displayH,
                     const char* const chans[], const char* const labels[],
                     int n, int groupAfter = -1) {
  if (n <= 0) return;
  const float cellW = barsRow.w / n;
  for (int i = 0; i < n; ++i) {
    const Rect cell{barsRow.x + cellW * static_cast<float>(i), barsRow.y, cellW,
                    barsRow.h};
    const EisGauge* g = layout.gaugeForChannel(chans[i]);
    drawVertBar(r, cell, g, chan(d, chans[i]), valid, displayH, labels[i]);
  }
  // The group divider after `groupAfter` is drawn by the caller (so it can span
  // the full section height, rule-to-rule, matching the real unit). We only
  // expose its x-coordinate here via turbineGroupDividerX().
  (void)groupAfter;
}

// X of the group divider for an `n`-bar turbine row split after 0-based bar
// `groupAfter` (the gap between that bar and the next); <0 if no split.
float turbineGroupDividerX(const Rect& barsRow, int n, int groupAfter) {
  if (n <= 0 || groupAfter < 0 || groupAfter >= n - 1) return -1.0f;
  return barsRow.x + (barsRow.w / n) * static_cast<float>(groupAfter + 1);
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

  // Taller turbine bars to match the real EIS's vertical share of the strip
  // (the bracketed gauges occupy ~0.20 of the column on the real unit). The
  // lower synoptic rows shift down to make room, with the bottom cabin block
  // absorbing the difference.
  const Rect turbineRow = band(0.175f, 0.345f);
  drawTurbineBars(r, d, layout, turbineRow, valid, displayH, kTurbineBarChans,
                  kTurbineBarLabels, kTurbineBarCount, /*groupAfter=*/1);
  rule(r, area, a.y + a.h * 0.355f);
  // Group divider between the engine-speed group (N1/N2) and the temp/pressure
  // group (ITT/Oil): a gray rule spanning the FULL section height on the real
  // unit (the rule above to the rule below, through the value/label area).
  {
    const float tdx = turbineGroupDividerX(turbineRow, kTurbineBarCount,
                                           /*groupAfter=*/1);
    if (tdx >= 0.0f) {
      r.strokeLine(tdx, a.y + a.h * 0.165f, tdx, a.y + a.h * 0.355f, 1.4f,
                   Color{0.41f, 0.41f, 0.41f, 1.0f});
    }
  }

  // Fuel (left) + electrical (right). The divider sits at ~0.42 of the strip on
  // the real unit (fuel column ~42 %, electrical ~58 %), not at mid-width. The
  // wider fuel column (was 0.385) gives the L/R quantity readouts room so they
  // don't collide once the bars are pulled inward off the divider.
  const Rect feRow = band(0.365f, 0.535f);
  constexpr float kFeDivider = 0.42f;
  drawFuelBlock(r, d, layout,
                Rect{feRow.x, feRow.y, feRow.w * kFeDivider, feRow.h}, valid,
                displayH);
  // Fuel/electrical divider: like the turbine group divider, it spans the full
  // section height (the rule above to the rule below) on the real unit, in the
  // same gray. The feRow band is inset from those rules, so anchor to them.
  {
    const float fdx = feRow.x + feRow.w * kFeDivider;
    r.strokeLine(fdx, a.y + a.h * 0.355f, fdx, a.y + a.h * 0.545f, 1.4f,
                 Color{0.41f, 0.41f, 0.41f, 1.0f});
  }
  drawElecBlock(r, d,
                Rect{feRow.x + feRow.w * (kFeDivider + 0.02f), feRow.y,
                     feRow.w * (1.0f - kFeDivider - 0.02f), feRow.h},
                valid, displayH);
  rule(r, area, a.y + a.h * 0.545f);

  // Landing gear (left) + pitch trim (right). This row is the tallest synoptic
  // block on the real unit (it holds the gear-disc triangle + the trim scale),
  // so it gets a section height on par with the turbine/fuel rows above.
  const Rect gtRow = band(0.555f, 0.705f);
  drawGearBlock(r, d, Rect{gtRow.x, gtRow.y, gtRow.w * 0.52f, gtRow.h},
                displayH);
  // Gear/pitch-trim divider: at ~0.537 of the strip on the real unit (traced
  // x=97 of 182), spanning the FULL section height rule-to-rule (the gtRow band
  // is inset from its rules, so anchor to them). Unlike the bright rgb(105)
  // engine/fuel group dividers, the lower synoptic dividers are a dim rgb(40)
  // gray (traced from the real gear divider), dimmer than kPanelSeparator.
  constexpr Color kSynopticDivider{0.157f, 0.157f, 0.157f, 1.0f};  // rgb(40)
  r.strokeLine(gtRow.x + gtRow.w * 0.532f, a.y + a.h * 0.545f,
               gtRow.x + gtRow.w * 0.532f, a.y + a.h * 0.715f, 1.0f,
               kSynopticDivider);
  drawPitchTrim(r, d,
                Rect{gtRow.x + gtRow.w * 0.55f, gtRow.y, gtRow.w * 0.45f,
                     gtRow.h},
                displayH);
  rule(r, area, a.y + a.h * 0.715f);

  // Flaps (left) + roll trim (right).
  const Rect frRow = band(0.725f, 0.795f);
  drawFlaps(r, d, Rect{frRow.x, frRow.y, frRow.w * 0.52f, frRow.h}, displayH);
  r.strokeLine(frRow.x + frRow.w * 0.53f, frRow.y, frRow.x + frRow.w * 0.53f,
               frRow.y + frRow.h, 1.0f, colors::kPanelSeparator);
  drawRollTrim(r, d,
               Rect{frRow.x + frRow.w * 0.55f, frRow.y, frRow.w * 0.45f,
                    frRow.h},
               displayH);
  rule(r, area, a.y + a.h * 0.805f);

  // Cabin pressurization.
  drawCabinBlock(r, d, band(0.815f, 0.99f), valid, displayH);
}

}  // namespace avionics::mfd
