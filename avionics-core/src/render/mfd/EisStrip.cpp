#include "render/mfd/EisStrip.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/EisLegacy.h"

namespace avionics::mfd {
namespace {

constexpr float kPi = 3.14159265358979323846f;

constexpr float kLabelWt = 13.0f;
constexpr float kValueWt = 16.0f;
constexpr float kRpmReadoutWt = 22.0f;

// Section title that triggers the framed electrical-group header (Fig. 3-9).
constexpr const char* kElectricalSection = "Electrical";

// RPM dial block height as a fraction of the strip's inner width. The dial is
// width-driven (radius = 0.46*width) and spans ~1.7 radii top-to-readout, so
// the block hugs the dial instead of reserving extra vertical slack.
constexpr float kRpmDialHeightFrac = 0.46f * 1.7f;

std::string fmt(const char* pattern, double v) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), pattern, v);
  return buf;
}

// G1000 NXi never prefixes zero with + or - on RPM or battery-amp readouts.
std::string fmtNoZeroSign(const char* pattern, double v, double quantum = 0.0) {
  const double display =
      quantum > 0.0 ? std::round(v / quantum) * quantum : std::round(v);
  if (display == 0.0) return "0";
  return fmt(pattern, quantum > 0.0 ? display : v);
}

void strokeArc(Renderer& r, float cx, float cy, float radius, float a0Deg,
               float a1Deg, float widthPx, const Color& c, float yScale = 1.0f) {
  constexpr int kSegments = 24;
  Point pts[kSegments + 1];
  for (int i = 0; i <= kSegments; ++i) {
    const float a =
        (a0Deg + (a1Deg - a0Deg) * static_cast<float>(i) / kSegments) *
        kPi / 180.0f;
    pts[i] = {cx + radius * std::sin(a), cy - (radius * std::cos(a)) * yScale};
  }
  r.strokePolyline(pts, kSegments + 1, widthPx, c);
}

struct BarBand {
  float lo, hi;
  Color color;
};

// Cessna Nav III tachometer (G1000 NXi Pilot's Guide Fig. 3-1): a bold white
// ring arc that opens at the bottom, with the green/red operating bands riding
// the outer rim and the only numeric graduations being "0" at the start tip and
// the redline value at the end tip. "RPM" and the large digital readout are
// stacked inside the arc's open bottom, flanked by those two end labels.
void drawRpmDial(Renderer& r, const FlightData& d, const EisGauge& gauge,
                 const Rect& area, float displayH) {
  const float cx = area.x + area.w * 0.5f;
  // The strip is narrow, so the dial is width-driven; seat it hard against the
  // top of its block so the arc starts at the very top of the strip (real unit).
  const float radius = area.w * 0.46f;
  const float cy = area.y + radius * 1.02f;

  // Narrower sweep -> larger open cutout at the bottom (real unit).
  constexpr float kStartDeg = -120.0f;
  constexpr float kSweepDeg = 240.0f;
  auto angleFor = [&](float rpm) {
    const float frac = std::max(0.0f, std::min(1.0f, rpm / gauge.max));
    return kStartDeg + kSweepDeg * frac;
  };
  auto rad = [](float deg) { return deg * kPi / 180.0f; };

  // The dial is a two-line track: a bold white band with a thinner white line
  // wrapping just inside it (real unit). The green/red operating bands replace
  // the bold band over their ranges (so no white shows through), and the inner
  // line + end-cap ticks stay white all the way around.
  const float ringW = std::max(2.5f, radius * 0.075f);
  const float outerLineW = std::max(1.5f, radius * 0.016f);
  const float outerLineR =
      radius + ringW * 0.5f + outerLineW * 0.5f + std::max(1.5f, radius * 0.03f);

  strokeArc(r, cx, cy, radius, kStartDeg, kStartDeg + kSweepDeg, ringW,
            colors::kWhite);
  // Configurable operating bands (BAND GREEN/RED lo hi in the .eis), drawn at
  // the band's own radius and width so they completely cover the white band.
  for (const EisBand& band : gauge.bands) {
    strokeArc(r, cx, cy, radius, angleFor(band.lo), angleFor(band.hi),
              ringW * 1.04f, eisBandColor(band.color));
  }
  // Second white line wrapping the full arc, just outside the bold band.
  strokeArc(r, cx, cy, outerLineR, kStartDeg, kStartDeg + kSweepDeg, outerLineW,
            colors::kWhite);

  // End-cap ticks at the scale ends (0 and the max). They reach the outer line
  // and project inward toward the center (real unit).
  const float tickInner = radius - ringW * 0.5f - std::max(3.0f, radius * 0.12f);
  const float tickOuter = outerLineR + outerLineW * 0.5f;
  for (float end : {kStartDeg, kStartDeg + kSweepDeg}) {
    const float a = rad(end);
    r.strokeLine(cx + tickInner * std::sin(a), cy - tickInner * std::cos(a),
                 cx + tickOuter * std::sin(a), cy - tickOuter * std::cos(a),
                 std::max(2.0f, radius * 0.022f), colors::kWhite);
  }

  const bool valid = d.dataLinkValid;
  const float rpm = eisChannelValue(d, gauge.channel, d.engineRpm);
  if (valid) {
    const float a = rad(angleFor(rpm));
    const float sa = std::sin(a);
    const float ca = std::cos(a);
    // Pointer along the value direction u=(sa,-ca), perpendicular p=(ca,sa).
    // A slim tapered shaft ending in a blunt arrowhead (G1000 NXi RPM pointer).
    auto pt = [&](float along, float side) -> Point {
      return {cx + along * sa + side * ca, cy - along * ca + side * sa};
    };
    const float tip = radius * 0.84f;
    const float head = radius * 0.60f;
    const float tail = radius * 0.10f;
    const float baseH = radius * 0.020f;
    const float shoulderH = radius * 0.030f;
    const float headH = radius * 0.072f;
    const Point needle[7] = {
        pt(tip, 0.0f),           pt(head, -headH),  pt(head, -shoulderH),
        pt(-tail, -baseH),       pt(-tail, baseH),  pt(head, shoulderH),
        pt(head, headH),
    };
    r.fillPolygon(needle, 7, colors::kWhite);
  }
  r.fillCircle(cx, cy, radius * 0.07f, colors::kPanelBorder);

  // End-tip labels: "0" at the start tip, the redline (or max) value at the end
  // tip. Placed just OUTSIDE the ring tips (radially out, lower-left/right) so
  // they flank the readout and don't overlay the ring (real unit).
  const float smallSize = mfdFontPx(kLabelWt * 0.82f, displayH);
  // "0" and the max label sit low and pulled in toward center, flanking the
  // readout just inside the arc tips (real unit).
  const float labelY = cy + radius * 0.68f;
  r.fillText(cx - radius * 0.80f, labelY, "0", smallSize, TextAlign::Center,
             colors::kWhite);
  const float endVal = gauge.hasRedline ? gauge.redline : gauge.max;
  r.fillText(cx + radius * 0.80f, labelY, fmt("%.0f", endVal), smallSize,
             TextAlign::Center, colors::kWhite);

  // "RPM" caption and the large readout stacked inside the arc opening, both in
  // the bold (SemiBold) face like the real unit.
  const char* dialLabel = gauge.label.empty() ? "RPM" : gauge.label.c_str();
  r.fillText(cx, cy + radius * 0.22f, dialLabel, smallSize, TextAlign::Center,
             colors::kWhite, FontFace::DejaVuSemiBold);
  const bool overspeed = valid && gauge.hasRedline && rpm >= gauge.redline;
  r.fillText(cx, cy + radius * 0.56f,
             valid ? fmtNoZeroSign("%.0f", rpm, 10.0) : std::string("____"),
             mfdFontPx(kRpmReadoutWt, displayH), TextAlign::Center,
             overspeed ? colors::kBandRed : colors::kWhite,
             FontFace::DejaVuSemiBold);
}

// White "home-plate" pointer used by the engine bars and the fuel L/R pointers
// (G1000 NXi Pilot's Guide): a flat-topped pentagon with straight shoulders that
// taper to a point at the track. drawDownPointer has its point at baseY (bottom);
// drawUpPointer has its point at baseY (top).
constexpr float kPointerShoulder = 0.45f; // fraction of height before tapering

void drawDownPointer(Renderer& r, float px, float baseY, float h, float halfW,
                     float shoulder = kPointerShoulder) {
  const float topY = baseY - h;
  const Point pointer[5] = {{px - halfW, topY},
                            {px + halfW, topY},
                            {px + halfW, topY + h * shoulder},
                            {px, baseY},
                            {px - halfW, topY + h * shoulder}};
  r.fillPolygon(pointer, 5, colors::kWhite);
}

void drawUpPointer(Renderer& r, float px, float baseY, float h, float halfW,
                   float shoulder = kPointerShoulder) {
  const float botY = baseY + h;
  const Point pointer[5] = {{px, baseY},
                            {px + halfW, botY - h * shoulder},
                            {px + halfW, botY},
                            {px - halfW, botY},
                            {px - halfW, botY - h * shoulder}};
  r.fillPolygon(pointer, 5, colors::kWhite);
}

// Cessna Nav III engine bar (G1000 NXi Pilot's Guide Fig. 3-1): a centered
// label over a thin white axis with end brackets, the colored operating bands
// riding on the axis, an optional combed scale (FFLOW/EGT) and min/max numerals
// (FFLOW), and a white down-pointer above the bands.
void drawBar(Renderer& r, const Rect& area, float y, const char* label,
             float value, float minV, float maxV, const BarBand* bands,
             int bandCount, int ticks, bool scale, bool valid,
             float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const bool isTurboprop = area.w < 120.0f;
  const float dialR = area.w * (area.w > 120.0f ? 0.525f : 0.60f);
  const float w = isTurboprop ? (dialR * 2.0f) : area.w;
  const float x0 = isTurboprop ? (area.x + area.w * 0.5f - dialR) : area.x;

  if (isTurboprop) {
    r.fillText(x0, y, label, labelSize, TextAlign::Left, colors::kLabelText);
  } else {
    r.fillText(area.x + area.w * 0.5f, y, label, labelSize, TextAlign::Center,
               colors::kLabelText);
  }

  // The real unit leaves a clear gap between the label and its bar (the
  // pointer rides up into it), so sit the track a little lower than the label.
  const float axisY = y + labelSize * 1.35f;
  const float bandH = labelSize * 0.48f;
  const bool ticked = ticks > 0;
  auto xFor = [&](float v) {
    const float frac =
        std::max(0.0f, std::min(1.0f, (v - minV) / (maxV - minV)));
    return x0 + w * frac;
  };

  // Operating bands sit as thin segments on top of the axis line.
  for (int i = 0; i < bandCount; ++i) {
    r.fillRect(xFor(bands[i].lo), axisY - bandH,
               xFor(bands[i].hi) - xFor(bands[i].lo), bandH, bands[i].color);
  }

  // Combed graduations rise above the axis (FFLOW/EGT), with taller end
  // brackets framing the scale. Non-combed bars keep short end brackets.
  const float combH = ticked ? labelSize * 0.88f : bandH * 1.15f;
  const float bracketH = ticked ? labelSize * 0.98f : bandH * 1.20f;
  r.strokeLine(x0, axisY, x0 + w, axisY, 1.2f, colors::kWhite);
  r.strokeLine(x0, axisY - bracketH, x0, axisY, 1.5f, colors::kWhite);
  r.strokeLine(x0 + w, axisY - bracketH, x0 + w, axisY, 1.5f, colors::kWhite);
  if (ticked) {
    for (int i = 1; i < ticks; ++i) {
      const float tx = x0 + w * static_cast<float>(i) / ticks;
      r.strokeLine(tx, axisY - combH, tx, axisY, 1.0f, colors::kWhite);
    }
  }
  // Min/max numerals under the track ends (FFLOW only).
  if (scale) {
    const float numSize = labelSize * 0.85f;
    const float numY = axisY + labelSize * 0.75f;
    r.fillText(x0, numY, fmt("%.0f", minV), numSize, TextAlign::Left,
               colors::kLabelText);
    r.fillText(x0 + w, numY, fmt("%.0f", maxV), numSize, TextAlign::Right,
               colors::kLabelText);
  }

  if (!valid) return;
  drawDownPointer(r, xFor(value), axisY - bandH * 0.20f, labelSize * 0.85f,
                  labelSize * 0.32f);
}

// Two-tank fuel quantity on one shared bar: L pointer above the track, R below,
// red/amber/green bands, and a 0/10/20/F gallon scale (Fig. 3-17).
void drawFuelQty(Renderer& r, const FlightData& d, const EisGauge& gauge,
                 const Rect& area, float& y, bool valid, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(area.x + area.w * 0.5f, y, gauge.label.c_str(), labelSize,
             TextAlign::Center, colors::kLabelText);

  const float axisY = y + labelSize * 2.2f;
  const float bandH = labelSize * 0.50f;
  const float x0 = area.x;
  const float w = area.w;
  auto xFor = [&](float v) {
    const float frac = std::max(
        0.0f, std::min(1.0f, (v - gauge.min) / (gauge.max - gauge.min)));
    return x0 + w * frac;
  };

  for (const EisBand& band : gauge.bands) {
    r.fillRect(xFor(band.lo), axisY - bandH * 0.5f,
               xFor(band.hi) - xFor(band.lo), bandH, eisBandColor(band.color));
  }
  r.strokeLine(x0, axisY, x0 + w, axisY, 1.2f, colors::kWhite);
  r.strokeLine(x0, axisY - bandH * 0.8f, x0, axisY + bandH * 0.8f, 1.5f,
               colors::kWhite);
  r.strokeLine(x0 + w, axisY - bandH * 0.8f, x0 + w, axisY + bandH * 0.8f, 1.5f,
               colors::kWhite);

  // Gallon scale: 0,10,20,... at every 10 gal, "F" at the full mark.
  const float numSize = labelSize * 0.85f;
  const float numY = axisY + labelSize * 1.7f;
  for (int g = 0; g <= static_cast<int>(gauge.max); g += 10) {
    const float gx = xFor(static_cast<float>(g));
    r.strokeLine(gx, axisY - bandH * 0.5f, gx, axisY + bandH * 0.5f, 1.0f,
                 colors::kWhite);
    r.fillText(gx, numY, std::to_string(g), numSize, TextAlign::Center,
               colors::kLabelText);
  }
  r.fillText(x0 + w, numY, "F", numSize, TextAlign::Right, colors::kLabelText);

  if (valid) {
    const float lv = eisChannelValue(d, gauge.channel, 0.0f);
    const float rv = eisChannelValue(d, gauge.channelRight, 0.0f);
    // Tall, boxy pointers (long straight body, short tip) so the L/R tank tag
    // knocks out vertically centred in the body (dark letter on the white
    // pointer), matching the real unit (Fig. 3-17).
    const float ph = labelSize * 1.15f;
    const float pw = labelSize * 0.46f;
    const float shoulder = 0.82f;
    const float lBaseY = axisY - bandH * 0.45f;  // L point at the track (above)
    const float rBaseY = axisY + bandH * 0.45f;  // R point at the track (below)
    drawDownPointer(r, xFor(lv), lBaseY, ph, pw, shoulder);
    drawUpPointer(r, xFor(rv), rBaseY, ph, pw, shoulder);
    // Centre the tag in the straight body (upper part for L, lower for R). The
    // backend's middle vertical align sits on the font's em centre, which leaves
    // a caps-only glyph riding high, so nudge it down by the cap correction.
    const float tagSize = labelSize * 0.60f;
    const float capAdj = tagSize * 0.07f;
    r.fillText(xFor(lv), lBaseY - ph + ph * shoulder * 0.5f + capAdj,
               gauge.leftTag, tagSize, TextAlign::Center, colors::kBlack);
    r.fillText(xFor(rv), rBaseY + ph - ph * shoulder * 0.5f + capAdj,
               gauge.rightTag, tagSize, TextAlign::Center, colors::kBlack);
  }
  y = numY + labelSize * 1.1f;
}

float drawReadout(Renderer& r, const Rect& area, float y, const char* label,
                  const std::string& value, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);

  const float dialR = area.w * (area.w > 120.0f ? 0.525f : 0.60f);
  const float leftEdge = area.x + area.w * 0.5f - dialR;
  const float rightEdge = area.x + area.w * 0.5f + dialR;

  r.fillText(leftEdge, y, label, labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(rightEdge, y, value, valueSize,
             TextAlign::Right, colors::kWhite);
  return y + valueSize * 1.35f;
}

// Electrical group row (G1000 NXi Pilot's Guide Fig. 3-9): the parameter name
// over its unit fills the center column ("Bus"/"Volts", "Battery"/"Amps"), with
// the two source tags (M/E, M/S) and their numeric values stacked in the
// flanking columns.
float drawElectricalRow(Renderer& r, const Rect& area, float y,
                        const std::string& label, const std::string& leftTag,
                        float leftVal, const std::string& rightTag,
                        float rightVal, const char* pattern, bool valid,
                        float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  std::string name = label;
  std::string unit;
  const std::size_t sp = label.find(' ');
  if (sp != std::string::npos) {
    name = label.substr(0, sp);
    unit = label.substr(sp + 1);
  }
  const float cx = area.x + area.w * 0.5f;
  const float xL = area.x + area.w * 0.16f;
  const float xR = area.x + area.w * 0.84f;

  r.fillText(xL, y, leftTag, labelSize, TextAlign::Center, colors::kLabelText);
  r.fillText(cx, y, name, labelSize, TextAlign::Center, colors::kLabelText);
  r.fillText(xR, y, rightTag, labelSize, TextAlign::Center, colors::kLabelText);

  y += valueSize * 1.05f;
  const bool signedFmt = std::strchr(pattern, '+') != nullptr;
  auto formatVal = [&](float val) -> std::string {
    if (!valid) return "__._";
    return signedFmt ? fmtNoZeroSign(pattern, val) : fmt(pattern, val);
  };
  const std::string lv = formatVal(leftVal);
  const std::string rv = formatVal(rightVal);
  r.fillText(xL, y, lv, valueSize, TextAlign::Center, colors::kWhite);
  r.fillText(cx, y, unit, labelSize, TextAlign::Center, colors::kLabelText);
  r.fillText(xR, y, rv, valueSize, TextAlign::Center, colors::kWhite);
  return y + valueSize * 1.4f;
}

float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }

// Two-tank vertical fuel quantity: L and R vertical bars side-by-side.
void drawFuelQtyVert(Renderer& r, const FlightData& d, const EisGauge& gauge,
                     const Rect& area, float& y, bool valid, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);

  // Title at the top (draw "FUEL QTY" and "LBS" in two separate lines)
  r.fillText(area.x + area.w * 0.5f, y, "FUEL QTY", labelSize, TextAlign::Center, colors::kLabelText);
  y += labelSize * 0.95f;
  r.fillText(area.x + area.w * 0.5f, y, "LBS", labelSize, TextAlign::Center, colors::kLabelText);

  const float trackTop = y + labelSize * 1.3f;
  const float trackH = labelSize * 6.5f * 1.25f; // 25% taller!
  const float trackW = std::max(4.0f, area.w * 0.12f);

  const float lxC = area.x + area.w * 0.32f - 5.0f; // moved 5px outwards!
  const float rxC = area.x + area.w * 0.68f + 5.0f; // moved 5px outwards!

  const float minV = gauge.min;
  const float maxV = gauge.max;
  const float span = (maxV - minV) != 0.0f ? (maxV - minV) : 1.0f;

  auto yFor = [&](float v) {
    return trackTop + trackH * (1.0f - clamp01((v - minV) / span));
  };

  auto drawOneBar = [&](float cxBar, float qty, const std::string& sideLabel) {
    const float x = cxBar - trackW * 0.5f;
    r.fillRect(x, trackTop, trackW, trackH, Color{0.13f, 0.13f, 0.13f, 1.0f});
    for (const EisBand& b : gauge.bands) {
      const float y0 = yFor(b.hi);
      const float y1 = yFor(b.lo);
      r.fillRect(x, y0, trackW, y1 - y0, eisBandColor(b.color));
    }
    r.strokeLine(x, trackTop, x, trackTop + trackH, 1.0f, colors::kPanelBorder);

    // Draw horizontal ticks on the columns for each 100 increment scale mark
    const float step = (maxV - minV) / 6.0f;
    for (float val = minV; val <= maxV + 0.1f; val += step) {
      const float tickY = yFor(val);
      r.strokeLine(x, tickY, x + trackW, tickY, 1.0f, colors::kWhite);
    }

    const float gap = 4.0f; // 4 pixel space between the gauge and static line

    // Draw the static white line on the outside of each column running the full height of the track
    if (cxBar < area.x + area.w * 0.5f) {
      r.strokeLine(x - gap, trackTop, x - gap, trackTop + trackH, 1.5f, colors::kWhite);
    } else {
      r.strokeLine(x + trackW + gap, trackTop, x + trackW + gap, trackTop + trackH, 1.5f, colors::kWhite);
    }

    if (valid) {
      const float vy = yFor(qty);
      const float barW = 8.0f;

      if (cxBar < area.x + area.w * 0.5f) {
        // Left bar: white bar on the outside (left side of the white line) up to vy, with a horizontal flat top
        r.fillRect(x - gap - barW, vy, barW, (trackTop + trackH) - vy, colors::kWhite);
      } else {
        // Right bar: white bar on the outside (right side of the white line) up to vy, with a horizontal flat top
        r.fillRect(x + trackW + gap, vy, barW, (trackTop + trackH) - vy, colors::kWhite);
      }
    }

    // Label inside/above the bar
    r.fillText(cxBar, trackTop - labelSize * 0.2f, sideLabel, labelSize * 0.9f,
               TextAlign::Center, colors::kLabelText);
  };

  drawOneBar(lxC, eisChannelValue(d, gauge.channel, 0.0f), "L");
  drawOneBar(rxC, eisChannelValue(d, gauge.channelRight, 0.0f), "R");

  // Draw central scale text labels: from 0 to 600 in increments of 100, exactly centered in the middle of L and R columns
  const float scaleTextSize = labelSize * 0.82f;
  const float step = (maxV - minV) / 6.0f;
  for (float val = minV; val <= maxV + 0.1f; val += step) {
    const float labelY = yFor(val);
    r.fillText(area.x + area.w * 0.5f, labelY + scaleTextSize * 0.35f,
               fmt("%.0f", val), scaleTextSize, TextAlign::Center, colors::kWhite);
  }

  // Draw digital readouts under each bar
  const float valY = trackTop + trackH + valueSize * 0.95f;
  const float lVal = eisChannelValue(d, gauge.channel, 0.0f);
  const float rVal = eisChannelValue(d, gauge.channelRight, 0.0f);
  r.fillText(lxC, valY, valid ? fmt("%.0f", lVal) : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);
  r.fillText(rxC, valY, valid ? fmt("%.0f", rVal) : std::string("---"),
             valueSize, TextAlign::Center, colors::kWhite);

  y = valY + valueSize * 0.5f + 30.0f; // shifted down 30 pixels!
}

void drawDial(Renderer& r, const FlightData& d, const EisGauge& gauge,
              const Rect& area, float& y, float displayH) {
  const float cx = area.x + area.w * 0.5f;
  const float lblSize = mfdFontPx(kLabelWt * 1.25f, displayH);
  const float valSize = mfdFontPx(kValueWt * 1.10f, displayH);

  // Set radius dynamically (larger for narrow column, default for single column)
  const float radius = area.w * (area.w > 120.0f ? 0.525f : 0.60f);

  const float ringW = std::max(1.8f, radius * 0.08f);
  const float outerLineW = std::max(1.0f, radius * 0.02f);
  const float outerLineR =
      radius + ringW * 0.5f + outerLineW * 0.5f + std::max(1.0f, radius * 0.03f);

  const bool isTurboprop = area.w < 120.0f;
  const float yScale = isTurboprop ? 1.12f : 1.0f;

  // Position cy such that the top of the arc starts exactly at the bottom of the previous gauge (y)
  const float cy = y + outerLineR * yScale;

  // Arc parameters (41% of 360 deg = 148 deg sweep starting at -90 deg - 0 point is horizontal)
  constexpr float kStartDeg = -90.0f;
  constexpr float kSweepDeg = 148.0f;

  auto angleFor = [&](float val) {
    const float frac = std::max(0.0f, std::min(1.0f, (val - gauge.min) / (gauge.max - gauge.min)));
    return kStartDeg + kSweepDeg * frac;
  };
  auto rad = [](float deg) { return deg * kPi / 180.0f; };

  // Base green arc (circular)
  strokeArc(r, cx, cy, radius, kStartDeg, kStartDeg + kSweepDeg, ringW,
            colors::kBandGreen, yScale);

  // Bands (circular)
  for (const EisBand& band : gauge.bands) {
    strokeArc(r, cx, cy, radius, angleFor(band.lo), angleFor(band.hi),
              ringW * 1.04f, eisBandColor(band.color), yScale);
  }

  // Outer line wrapping the arc (circular)
  strokeArc(r, cx, cy, outerLineR, kStartDeg, kStartDeg + kSweepDeg, outerLineW,
            colors::kWhite, yScale);

  // End-cap ticks
  const float tickInner = radius - ringW * 0.5f - std::max(2.0f, radius * 0.12f);
  const float tickOuter = outerLineR + outerLineW * 0.5f;
  for (float end : {kStartDeg, kStartDeg + kSweepDeg}) {
    const float a = rad(end);
    r.strokeLine(cx + tickInner * std::sin(a), cy - (tickInner * std::cos(a)) * yScale,
                 cx + tickOuter * std::sin(a), cy - (tickOuter * std::cos(a)) * yScale,
                 std::max(1.5f, radius * 0.022f), colors::kWhite);
  }

  const bool valid = d.dataLinkValid;
  const float val = eisChannelValue(d, gauge.channel, 0.0f);
  const float bugValue = !gauge.bugChannel.empty()
      ? eisChannelValue(d, gauge.bugChannel, gauge.bug)
      : gauge.bug;
  const float redlineValue = !gauge.redlineChannel.empty()
      ? eisChannelValue(d, gauge.redlineChannel, gauge.redline)
      : gauge.redline;

  if (gauge.hasBug && valid) {
    const float ba = rad(angleFor(bugValue));
    const float bsa = std::sin(ba);
    const float bca = std::cos(ba);
    const float br0 = radius + ringW * 0.20f;
    const float br1 = outerLineR + std::max(5.0f, radius * 0.16f);
    r.strokeLine(cx + br0 * bsa, cy - (br0 * bca) * yScale,
                 cx + br1 * bsa, cy - (br1 * bca) * yScale,
                 std::max(2.0f, radius * 0.055f), colors::kCyan);
  }
  if (gauge.hasRedline && valid) {
    const float ra = rad(angleFor(redlineValue));
    const float rsa = std::sin(ra);
    const float rca = std::cos(ra);
    const float rr0 = radius - ringW * 0.65f;
    const float rr1 = outerLineR + std::max(2.0f, radius * 0.05f);
    r.strokeLine(cx + rr0 * rsa, cy - (rr0 * rca) * yScale,
                 cx + rr1 * rsa, cy - (rr1 * rca) * yScale,
                 std::max(2.0f, radius * 0.045f), colors::kBandRed);
  }

  if (valid) {
    const float a = rad(angleFor(val));
    const float sa = std::sin(a);
    const float ca = std::cos(a);

    // Tip at inner edge of the arc, no needle tail to center
    const float tipR = radius - ringW * 0.5f;
    const float baseR = tipR - std::max(4.5f, radius * 0.16f);
    const float baseW = std::max(3.5f, radius * 0.12f);

    const Point ptr[3] = {
      {cx + tipR * sa, cy - (tipR * ca) * yScale},
      {cx + baseR * sa - baseW * ca, cy - (baseR * ca + baseW * sa) * yScale},
      {cx + baseR * sa + baseW * ca, cy - (baseR * ca - baseW * sa) * yScale}
    };
    const bool overspeed = gauge.hasRedline && val >= redlineValue;
    r.fillPolygon(ptr, 3, overspeed ? colors::kBandRed : colors::kWhite);
  }



  // Text labels centered slightly left-shifted to offset the right-aligned readout
  const size_t spacePos = gauge.label.find(' ');
  if (spacePos != std::string::npos) {
    const std::string line1 = gauge.label.substr(0, spacePos);
    const std::string line2 = gauge.label.substr(spacePos + 1);
    r.fillText(cx - radius * 0.18f, cy - radius * 0.38f, line1.c_str(), lblSize, TextAlign::Center,
               colors::kLabelText, FontFace::DejaVuSemiBold);
    r.fillText(cx - radius * 0.18f, cy - radius * 0.08f, line2.c_str(), lblSize * 0.9f, TextAlign::Center,
               colors::kLabelText, FontFace::DejaVuSemiBold);
  } else {
    r.fillText(cx - radius * 0.18f, cy - radius * 0.23f, gauge.label.c_str(), lblSize, TextAlign::Center,
               colors::kLabelText, FontFace::DejaVuSemiBold);
  }

  // Digital readout inside the middle of the gauge, dynamically colored by band
  Color readoutColor = colors::kWhite;
  if (valid) {
    if (gauge.hasRedline && val >= redlineValue) {
      readoutColor = colors::kBandRed;
    } else {
      for (const EisBand& band : gauge.bands) {
        if (val >= band.lo && val <= band.hi) {
          readoutColor = eisBandColor(band.color);
          break;
        }
      }
    }
  }

  // Digital readout text positioned at the bottom-right, under the end of the arc (moved 20px up, 5px right)
  r.fillText(cx + radius * 0.85f + 5.0f, cy + radius * 0.35f - 20.0f,
             valid ? fmt(gauge.format.c_str(), val) : std::string("____"),
             valSize, TextAlign::Right,
             readoutColor,
             FontFace::DejaVuSemiBold);

  // Update y to the exact bottom of the outer line of the arc (compressed for intermediate dials)
  if (gauge.channel == "eng.ng") {
    y = cy + outerLineR * yScale;
  } else {
    y = cy + radius * 0.35f * yScale;
  }
}

std::vector<BarBand> toBarBands(const EisGauge& gauge) {
  std::vector<BarBand> bands;
  bands.reserve(gauge.bands.size());
  for (const EisBand& band : gauge.bands) {
    bands.push_back({band.lo, band.hi, eisBandColor(band.color)});
  }
  return bands;
}

// Natural vertical space one gauge consumes (must mirror the advances in
// drawGauge). Used to pre-measure the stack so leftover height can be spread
// evenly between gauges, filling the strip like the real unit.
float gaugeHeight(const EisGauge& gauge, const Rect& inner, float barStride,
                  float labelSize, float valueSize) {
  switch (gauge.type) {
    case EisGaugeType::RpmDial:
      return inner.w * kRpmDialHeightFrac + labelSize * 0.9f;
    case EisGaugeType::Bar:
      return barStride;
    case EisGaugeType::FuelQty:
      return labelSize * 5.0f;
    case EisGaugeType::FuelQtyVert:
      return labelSize * 9.5f;
    case EisGaugeType::Dial:
      if (gauge.channel == "eng.ng") {
        return inner.w * 1.05f;
      } else {
        return inner.w * 0.65f;
      }
    case EisGaugeType::Readout:
      return valueSize * 1.35f;
    case EisGaugeType::Electrical:
      return valueSize * 2.45f;
  }
  return 0.0f;
}

// Vertical space a section's heading consumes before its first gauge (must
// mirror the section handling in drawEisStrip).
float sectionLead(const std::string& title, float labelSize) {
  if (title == kElectricalSection) return labelSize * 2.2f;
  if (!title.empty()) return labelSize * 0.6f;
  return 0.0f;
}

void drawGauge(Renderer& r, const FlightData& d, const EisGauge& gauge,
               const Rect& inner, float& y, bool valid, float displayH,
               float barStride, float labelSize) {
  switch (gauge.type) {
    case EisGaugeType::RpmDial: {
      const float dialH = inner.w * kRpmDialHeightFrac;
      drawRpmDial(r, d, gauge, Rect{inner.x, y, inner.w, dialH}, displayH);
      y += dialH + labelSize * 0.9f;
      break;
    }
    case EisGaugeType::Bar: {
      const std::vector<BarBand> bands = toBarBands(gauge);
      const float value = eisChannelValue(d, gauge.channel, 0.0f);
      drawBar(r, inner, y, gauge.label.c_str(), value, gauge.min, gauge.max,
              bands.empty() ? nullptr : bands.data(),
              static_cast<int>(bands.size()), gauge.ticks, gauge.scale, valid,
              displayH);
      y += barStride;
      break;
    }
    case EisGaugeType::FuelQty: {
      drawFuelQty(r, d, gauge, inner, y, valid, displayH);
      break;
    }
    case EisGaugeType::FuelQtyVert: {
      drawFuelQtyVert(r, d, gauge, inner, y, valid, displayH);
      break;
    }
    case EisGaugeType::Dial: {
      drawDial(r, d, gauge, inner, y, displayH);
      break;
    }
    case EisGaugeType::Readout: {
      y = drawReadout(
          r, inner, y, gauge.label.c_str(),
          valid ? fmt(gauge.format.c_str(), eisChannelValue(d, gauge.channel, 0.0f)) : std::string("____._"),
          displayH);
      break;
    }
    case EisGaugeType::Electrical: {
      const float leftVal = eisChannelValue(d, gauge.channel, 0.0f);
      const float rightVal = eisChannelValue(d, gauge.channelRight, 0.0f);
      y = drawElectricalRow(r, inner, y, gauge.label, gauge.leftTag, leftVal,
                            gauge.rightTag, rightVal, gauge.format.c_str(),
                            valid, displayH);
      break;
    }
  }
}

}  // namespace

void drawRightColumnPA46(Renderer& r, const FlightData& d, const Rect& a, float displayH, bool valid) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  
  auto chan = [](const FlightData& fd, const char* name) {
    auto it = fd.eisChannels.find(name);
    return it != fd.eisChannels.end() ? it->second : 0.0f;
  };

  auto clamp = [](float val, float low, float high) {
    return std::max(low, std::min(high, val));
  };

  const float bottomMargin = labelSize * 1.2f;
  const float rightNatural = labelSize * 0.9f + (labelSize * 1.1f + labelSize * 0.9f + 100.0f + valueSize * 0.9f * 2.0f + 65.0f) // cabin block height
    + (labelSize * 1.1f + labelSize * 1.1f + labelSize * 0.9f + 16.0f) // electrical block
    + (labelSize * 0.9f + 16.0f) // vacuum block
    + (labelSize * 0.9f + 18.0f) // rudder trim block
    + (labelSize * 0.85f + labelSize * 0.8f + 50.0f) // flaps block
    + 30.0f; // gear block
  const float rightSlack = std::max(0.0f, a.h - bottomMargin - rightNatural);
  const float rightExtra = rightSlack / 5.0f;

  float y = a.y + labelSize * 0.9f;

  // 1. Cabin Pressurization block
  r.fillText(a.x + a.w * 0.5f, y, "CABIN PRESS", labelSize, TextAlign::Center, colors::kLabelText);
  y += labelSize * 1.1f;

  const float rate = chan(d, eis_channels::kCabinRateFpm);
  const float alt = chan(d, eis_channels::kCabinAltFt);
  const float diff = chan(d, eis_channels::kCabinDiffPsi);
  const float dest = chan(d, eis_channels::kDestElevFt);

  // Side-by-side vertical tracks for Cabin Alt (L) and FPM (R)
  const float barH = 100.0f; // 100 pixels tall!
  const float barW = std::max(4.0f, a.w * 0.12f);
  const float leftCx = a.x + a.w * 0.22f;
  const float rightCx = a.x + a.w * 0.78f;

  // ALT FT label and value (Left side of column), FPM label and value (Right side of column)
  r.fillText(a.x, y, "ALT FT", labelSize * 0.8f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, "FPM", labelSize * 0.8f, TextAlign::Right, colors::kLabelText);

  const float trackTop = y + labelSize * 0.9f;
  
  auto yForAlt = [&](float val) {
    return trackTop + barH * (1.0f - clamp01(val / 25000.0f));
  };
  r.fillRect(leftCx - barW * 0.5f, trackTop, barW, barH, Color{0.13f, 0.13f, 0.13f, 1.0f});
  r.fillRect(leftCx - barW * 0.3f, trackTop, barW * 0.6f, barH, colors::kBandGreen);
  if (valid) {
    const float ay = yForAlt(alt);
    const float px = leftCx - barW * 0.5f;
    const Point ptr[3] = {
      {px, ay},
      {px - 6.0f, ay - 4.0f},
      {px - 6.0f, ay + 4.0f}
    };
    r.fillPolygon(ptr, 3, colors::kWhite);
    r.fillRect(px - 10.0f, ay - 4.0f, 4.0f, 8.0f, colors::kWhite);
  }
  r.fillText(a.x, trackTop + barH + valueSize * 0.9f,
             valid ? fmt("%.0f", alt) : std::string("----"),
             valueSize * 0.9f, TextAlign::Left, colors::kWhite);

  // Cabin Rate FPM: -2000 to +2000 FPM
  auto yForRate = [&](float val) {
    return trackTop + barH * (0.5f - 0.5f * clamp(val / 2000.0f, -1.0f, 1.0f));
  };
  r.fillRect(rightCx - barW * 0.5f, trackTop, barW, barH, Color{0.13f, 0.13f, 0.13f, 1.0f});
  r.fillRect(rightCx - barW * 0.3f, trackTop, barW * 0.6f, barH, colors::kBandGreen);
  if (valid) {
    const float ry = yForRate(rate);
    const float px = rightCx + barW * 0.5f;
    const Point ptr[3] = {
      {px, ry},
      {px + 6.0f, ry - 4.0f},
      {px + 6.0f, ry + 4.0f}
    };
    r.fillPolygon(ptr, 3, colors::kWhite);
    r.fillRect(px + 6.0f, ry - 4.0f, 4.0f, 8.0f, colors::kWhite);
  }
  r.fillText(a.x + a.w, trackTop + barH + valueSize * 0.9f,
             valid ? fmt("%.0f", rate) : std::string("----"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);

  y = trackTop + barH + valueSize * 1.5f;

  // Horizontal DIFF PSI
  r.fillText(a.x, y, "DIFF PSI", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.1f", diff) : std::string("-.-"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  y += labelSize * 0.9f;
  const float barW_h = a.w;
  r.fillRect(a.x, y, barW_h, 4.0f, Color{0.13f, 0.13f, 0.13f, 1.0f});
  
  // Background colored bands: green from 0 to 5, red from 5 to 6
  const float maxDiff = 6.0f;
  const float x5 = barW_h * (5.0f / maxDiff);
  r.fillRect(a.x, y - 1.0f, x5, 6.0f, Color{0.0f, 0.35f, 0.0f, 1.0f}); // dark green band
  r.fillRect(a.x + x5, y - 1.0f, barW_h - x5, 6.0f, Color{0.4f, 0.0f, 0.0f, 1.0f}); // dark red band

  if (valid) {
    if (diff <= 5.0f) {
      const float fillW = barW_h * clamp01(diff / maxDiff);
      r.fillRect(a.x, y, fillW, 4.0f, colors::kBandGreen);
    } else {
      r.fillRect(a.x, y, x5, 4.0f, colors::kBandGreen);
      const float fillRed = barW_h * (clamp(diff, 5.0f, maxDiff) - 5.0f) / maxDiff;
      r.fillRect(a.x + x5, y, fillRed, 4.0f, colors::kBandRed);
    }
    const float needleX = a.x + barW_h * clamp01(diff / maxDiff);
    r.strokeLine(needleX, y - 2.0f, needleX, y + 6.0f, 1.5f, colors::kWhite);
  }

  // Add markers below the tape for 0, 3, 6
  const float tickY = y + 8.0f;
  auto drawDiffTick = [&](float v) {
    const float tx = a.x + barW_h * (v / maxDiff);
    r.strokeLine(tx, y + 4.0f, tx, y + 7.0f, 1.0f, colors::kWhite);
    r.fillText(tx, tickY + labelSize * 0.75f, fmt("%.0f", v), labelSize * 0.75f, TextAlign::Center, colors::kWhite);
  };
  drawDiffTick(0.0f);
  drawDiffTick(3.0f);
  drawDiffTick(6.0f);
  y = tickY + labelSize * 0.75f + 8.0f;

  // Destination Altitude
  r.fillText(a.x, y, "DEST ALTD", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.0f", dest) : std::string("-----"),
             valueSize * 0.9f, TextAlign::Right, colors::kMagenta);
  y += labelSize * 1.1f;

  // Separator Line
  r.strokeLine(a.x, y, a.x + a.w, y, 1.0f, colors::kPanelSeparator);
  y += 10.0f + rightExtra;

  // 2. ELECTRICAL
  r.fillText(a.x + a.w * 0.5f, y, "ELECTRICAL", labelSize, TextAlign::Center, colors::kLabelText);
  y += labelSize * 1.1f;

  // AMPS GEN / ALT
  const float genAmps = chan(d, eis_channels::kGen1Amps);
  const float altAmps = chan(d, eis_channels::kGen2Amps);
  
  // Line 1: AMPS (left-aligned) and GEN (right-aligned at 45% of column) and GEN reading (green, right-aligned)
  r.fillText(a.x, y, "AMPS", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w * 0.45f, y, "GEN", labelSize * 0.85f, TextAlign::Right, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.0f", genAmps) : std::string("---"),
             valueSize * 0.9f, TextAlign::Right, colors::kBandGreen);
  
  y += labelSize * 1.1f;

  // Line 2: ALT (right-aligned with GEN at 45% of column) and ALT reading (white, right-aligned)
  r.fillText(a.x + a.w * 0.45f, y, "ALT", labelSize * 0.85f, TextAlign::Right, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.0f", altAmps) : std::string("---"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  
  y += labelSize * 1.1f;

  // Volts and horizontal battery bar
  const float volts = chan(d, eis_channels::kBusVoltsMain);
  r.fillText(a.x, y, "BATT VOLTS", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.1f", volts) : std::string("--.-"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  y += labelSize * 0.9f;

  r.fillRect(a.x, y, barW_h, 4.0f, Color{0.13f, 0.13f, 0.13f, 1.0f});
  const float greenLoX = barW_h * ((24.0f - 20.0f) / 12.0f);
  const float greenHiX = barW_h * ((30.0f - 20.0f) / 12.0f);
  r.fillRect(a.x + greenLoX, y - 1.0f, greenHiX - greenLoX, 6.0f, Color{0.0f, 0.35f, 0.0f, 1.0f});
  if (valid) {
    const float fillW = barW_h * clamp01((volts - 20.0f) / 12.0f);
    const Color voltsColor = (volts < 24.0f || volts > 30.0f) ? colors::kBandRed : colors::kBandGreen;
    r.fillRect(a.x, y, fillW, 4.0f, voltsColor);
    r.strokeLine(a.x + fillW, y - 2.0f, a.x + fillW, y + 6.0f, 1.5f, colors::kWhite);
  }
  y += 12.0f;

  // Separator Line
  r.strokeLine(a.x, y, a.x + a.w, y, 1.0f, colors::kPanelSeparator);
  y += 10.0f + rightExtra;

  // 3. VACUUM
  const float vac = chan(d, eis_channels::kVacuum);
  r.fillText(a.x, y, "VACUUM IN HG", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? fmt("%.2f", vac) : std::string("-.--"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  y += labelSize * 0.9f;

  r.fillRect(a.x, y, barW_h, 4.0f, Color{0.13f, 0.13f, 0.13f, 1.0f});
  const float vacGreenLoX = barW_h * ((4.5f - 3.0f) / 4.0f);
  const float vacGreenHiX = barW_h * ((5.5f - 3.0f) / 4.0f);
  r.fillRect(a.x + vacGreenLoX, y - 1.0f, vacGreenHiX - vacGreenLoX, 6.0f, Color{0.0f, 0.35f, 0.0f, 1.0f});
  if (valid) {
    const float fillW = barW_h * clamp01((vac - 3.0f) / 4.0f);
    const Color vacColor = (vac < 4.5f || vac > 5.5f) ? colors::kBandRed : colors::kBandGreen;
    r.fillRect(a.x, y, fillW, 4.0f, vacColor);
    r.strokeLine(a.x + fillW, y - 2.0f, a.x + fillW, y + 6.0f, 1.5f, colors::kWhite);
  }
  y += 12.0f;

  // Separator Line
  r.strokeLine(a.x, y, a.x + a.w, y, 1.0f, colors::kPanelSeparator);
  y += 10.0f + rightExtra;

  // 4. RUDDER TRIM
  const float rudTrim = chan(d, eis_channels::kRudderTrim);
  const float rudDeg = rudTrim * 10.0f;
  const std::string rudStr = std::abs(rudDeg) < 0.1f ? "0.0" : (fmt("%.1f", std::abs(rudDeg)) + (rudDeg < 0.0f ? " L" : " R"));

  r.fillText(a.x, y, "RUDDER TRIM", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  r.fillText(a.x + a.w, y, valid ? rudStr : std::string("---"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  y += labelSize * 0.9f;

  r.fillRect(a.x, y, barW_h, 3.0f, Color{0.25f, 0.25f, 0.25f, 1.0f});
  r.strokeLine(a.x + barW_h * 0.5f, y - 3.0f, a.x + barW_h * 0.5f, y + 6.0f, 1.0f, colors::kWhite);
  if (valid) {
    const float px = a.x + barW_h * (0.5f + 0.5f * clamp(rudTrim, -1.0f, 1.0f));
    const Point ptr[3] = {
      {px, y - 4.0f},
      {px - 4.0f, y + 4.0f},
      {px + 4.0f, y + 4.0f}
    };
    r.fillPolygon(ptr, 3, colors::kWhite);
  }
  y += rightExtra - 35.0f;

  // 5. FLAPS
  r.fillText(a.x, y, "FLAPS", labelSize * 0.85f, TextAlign::Left, colors::kLabelText);
  const float flapsAct = chan(d, eis_channels::kFlapsActual);
  const float flapFrac = clamp01(flapsAct);
  float flapsDeg = 0.0f;
  if (flapFrac <= 0.333f) {
    flapsDeg = (flapFrac / 0.333f) * 10.0f;
  } else if (flapFrac <= 0.666f) {
    flapsDeg = 10.0f + ((flapFrac - 0.333f) / 0.333f) * 10.0f;
  } else {
    flapsDeg = 20.0f + ((flapFrac - 0.666f) / 0.334f) * 16.0f;
  }
  r.fillText(a.x + a.w, y, valid ? fmt("%.0f\x5E", flapsDeg) : std::string("---"),
             valueSize * 0.9f, TextAlign::Right, colors::kWhite);
  y += labelSize * 0.8f;

  const float pivotX = a.x + 15.0f;
  const float pivotY = y + 15.0f;
  const float rot = flapsDeg * (50.0f / 36.0f) * kPi / 180.0f; // 50 degrees sweep total
  const float cosR = std::cos(rot);
  const float sinR = std::sin(rot);

  r.strokeLine(pivotX - 3.0f, pivotY, pivotX + 3.0f, pivotY, 1.0f, colors::kWhite);
  Point wing[5] = {
    {0.0f, -4.0f},
    {30.0f, -1.5f},
    {36.0f, 0.0f},
    {15.0f, 2.5f},
    {0.0f, 4.0f}
  };
  Point rotWing[5];
  for (int i = 0; i < 5; ++i) {
    rotWing[i] = {
      pivotX + wing[i].x * cosR - wing[i].y * sinR,
      pivotY + wing[i].x * sinR + wing[i].y * cosR
    };
  }
  r.fillPolygon(rotWing, 5, colors::kWhite);

  const float scaleR = 38.0f;
  const float flapsScaleSize = labelSize * 0.78f;
  auto drawFlapTick = [&](float deg, const char* txt) {
    const float aRad = deg * (50.0f / 36.0f) * kPi / 180.0f;
    const float cosA = std::cos(aRad);
    const float sinA = std::sin(aRad);
    const float tx0 = pivotX + scaleR * cosA;
    const float ty0 = pivotY + scaleR * sinA;
    const float tx1 = pivotX + (scaleR + 10.0f) * cosA; // Ticks are 10px long radial lines!
    const float ty1 = pivotY + (scaleR + 10.0f) * sinA;
    r.strokeLine(tx0, ty0, tx1, ty1, 1.0f, colors::kWhite);
    r.fillText(tx1 + 5.0f * cosA, ty1 + 5.0f * sinA + flapsScaleSize * 0.35f, txt, flapsScaleSize, TextAlign::Left, colors::kWhite);
  };
  drawFlapTick(0.0f, "0");
  drawFlapTick(10.0f, "10");
  drawFlapTick(20.0f, "20");
  drawFlapTick(36.0f, "36");

  y += 48.0f;

  // Separator Line
  r.strokeLine(a.x, y, a.x + a.w, y, 1.0f, colors::kPanelSeparator);
  y += 12.0f + rightExtra;

  // 6. LANDING GEAR
  const float gearN = chan(d, eis_channels::kGearNose);
  const float gearL = chan(d, eis_channels::kGearLeft);
  const float gearR = chan(d, eis_channels::kGearRight);

  const float gearCx = a.x + a.w * 0.5f;
  const float gearR_circ = 13.0f; // Over double the size!
  const float dx_spacing = 33.0f; // Spread out slightly more!
  const float dy_spacing = 26.0f;

  const float bottomLimit = a.y + a.h - bottomMargin;
  const float mainY = bottomLimit - gearR_circ - 4.0f; // Just above the bottom of the frame
  const float noseY = mainY - dy_spacing;
  
  // Draw the "LANDING GEAR" text header centered above the nose gear
  r.fillText(gearCx, noseY - gearR_circ - 14.0f, "LANDING GEAR", labelSize * 0.85f,
             TextAlign::Center, colors::kLabelText);

  auto drawGearLight = [&](float cx, float cy, float value) {
    strokeArc(r, cx, cy, gearR_circ, 0.0f, 360.0f, 2.5f, colors::kWhite);
    if (valid && value >= 1.0f) {
      r.fillCircle(cx, cy, gearR_circ - 1.2f, colors::kBandGreen);
    }
  };

  drawGearLight(gearCx, noseY, gearN);
  drawGearLight(gearCx - dx_spacing, mainY, gearL);
  drawGearLight(gearCx + dx_spacing, mainY, gearR);
}

float eisStripWidthFrac(EisStripStyle style) {
  // 150/1024 piston; 237/1024 turbofan (sized so its width-to-height ratio
  // matches the real Perspective Touch+ EIS, strip w/h ~0.349 over the body
  // height); 240/1024 turboprop (2-column layout), against the 1024 px GDU
  // canvas.
  if (style == EisStripStyle::Turbofan) return 237.0f / 1024.0f;
  if (style == EisStripStyle::Turboprop) return 240.0f / 1024.0f;
  return 150.0f / 1024.0f;
}

void drawEisStrip(Renderer& r, const FlightData& d, const EisLayout& layout,
                  const Rect& area, float displayH, bool reduced) {
  // The Cirrus Vision SF50 jet has a fundamentally different engine page (a
  // dense turbine + synoptic grid) than the piston Cessna stack below.
  if (layout.style == EisStripStyle::Turbofan) {
    drawEisStripTurbofan(r, d, layout, area, displayH, reduced);
    return;
  }

  // Solid black backing for the whole engine instrument strip (real unit).
  r.fillRect(area.x, area.y, area.w, area.h, colors::kBlack);
  r.strokeLine(area.x + area.w, area.y, area.x + area.w, area.y + area.h, 2.0f,
               colors::kPanelBorder);

  // The real EIS labels, readouts and scales are all in the bold (SemiBold)
  // face, so render the whole strip's default text bold.
  FontScope stripFont(r, FontFace::DejaVuSemiBold);

  const bool valid = d.dataLinkValid;
  const float pad = area.w * 0.10f;
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  const float barStride = labelSize * 3.1f;
  const float topPad = labelSize * 0.9f;
  const float bottomMargin = labelSize * 1.2f;

  if (layout.style == EisStripStyle::Caravan) {
    // Single narrow column matching the Cessna 208B G1000 EIS. The top three
    // indications are TRQ, ITT and Ng arcs; PROP RPM and the remaining systems
    // are compact rows below them.
    const Rect inner{area.x + pad * 0.55f, area.y, area.w - pad * 1.10f, area.h};
    const float topY = area.y + topPad * 0.45f;
    const float dialsBottom = area.y + area.h * 0.43f;
    const float dialStride = (dialsBottom - topY) / 3.0f;
    int dialIndex = 0;
    for (const EisSection& section : layout.sections) {
      for (const EisGauge& gauge : section.gauges) {
        if (gauge.type == EisGaugeType::Dial) {
          float gy = topY + dialIndex * dialStride;
          drawGauge(r, d, gauge, inner, gy, valid, displayH, barStride, labelSize);
          ++dialIndex;
        }
      }
    }

    float gy = dialsBottom + labelSize * 0.25f;
    for (const EisSection& section : layout.sections) {
      for (const EisGauge& gauge : section.gauges) {
        if (gauge.type != EisGaugeType::Dial) {
          drawGauge(r, d, gauge, inner, gy, valid, displayH,
                    labelSize * 2.45f, labelSize);
          if (gauge.channel == "eng.oil_temp_c" ||
              gauge.type == EisGaugeType::FuelQtyVert) {
            r.strokeLine(inner.x, gy, inner.x + inner.w, gy, 1.0f, colors::kPanelSeparator);
            gy += labelSize * 0.35f;
          }
        }
      }
    }
    return;
  }

  if (layout.style == EisStripStyle::Turboprop) {
    // Two-column layout for the PA-46T!
    const float colW = (area.w - pad * 3.0f) * 0.5f;
    const Rect leftCol{area.x + pad, area.y, colW, area.h};
    const Rect rightCol{area.x + pad * 2.0f + colW, area.y, colW, area.h};

    // Draw vertical divider line
    r.strokeLine(area.x + pad + colW + pad * 0.5f, area.y + topPad,
                 area.x + pad + colW + pad * 0.5f, area.y + area.h - bottomMargin,
                 1.0f, colors::kPanelSeparator);

    // Left Column: loaded engine gauges (from pa46t.eis)
    const float halfwayY = area.y + area.h * 0.5f;
    const float topY = area.y + topPad;
    const float dialStride = (halfwayY - topY) / 4.0f;

    // Draw dials in the top half
    int dialIndex = 0;
    for (const EisSection& section : layout.sections) {
      for (const EisGauge& gauge : section.gauges) {
        if (gauge.type == EisGaugeType::Dial || gauge.type == EisGaugeType::RpmDial) {
          float tempY = topY + dialIndex * dialStride;
          drawGauge(r, d, gauge, leftCol, tempY, valid, displayH, barStride, labelSize);
          dialIndex++;
        }
      }
    }

    // Draw remaining non-dial gauges starting exactly at the halfway mark, stacked tightly downwards
    float leftY = halfwayY;
    for (const EisSection& section : layout.sections) {
      for (const EisGauge& gauge : section.gauges) {
        if (gauge.type != EisGaugeType::Dial && gauge.type != EisGaugeType::RpmDial) {
          drawGauge(r, d, gauge, leftCol, leftY, valid, displayH, barStride, labelSize);
          if (gauge.channel == "eng.oil_temp_c") {
            // Draw a white horizontal divider line following the temp
            leftY += labelSize * 0.4f;
            r.strokeLine(leftCol.x, leftY, leftCol.x + leftCol.w, leftY, 1.0f, colors::kWhite);
            leftY += labelSize * 0.8f;
          }
        }
      }
    }

    // Right Column: custom systems/synoptics
    drawRightColumnPA46(r, d, rightCol, displayH, valid);
    return;
  }

  const Rect inner{area.x + pad, area.y, area.w - 2.0f * pad, area.h};

  // Pre-measure the stack so the leftover height can be shared evenly between
  // gauges. The real EIS strip fills the full column rather than bunching the
  // gauges at the top, so we spread our (font-sized) gauges to span the strip.
  float natural = topPad;
  int gaugeCount = 0;
  for (const EisSection& section : layout.sections) {
    natural += sectionLead(section.title, labelSize);
    for (const EisGauge& gauge : section.gauges) {
      natural += gaugeHeight(gauge, inner, barStride, labelSize, valueSize);
      ++gaugeCount;
    }
  }
  const float slack = std::max(0.0f, area.h - bottomMargin - natural);
  const float extra = gaugeCount > 1 ? slack / (gaugeCount - 1) : 0.0f;

  float y = area.y + topPad;
  int drawn = 0;
  for (const EisSection& section : layout.sections) {
    if (section.title == kElectricalSection) {
      // Framed "Electrical" header: the title centered on a horizontal rule
      // that breaks around it (Fig. 3-9).
      y += labelSize * 0.6f;
      const float headerSize = mfdFontPx(kValueWt, displayH);
      const float cx = inner.x + inner.w * 0.5f;
      const float tw = r.measureTextWidth(section.title, headerSize);
      const float gap = labelSize * 0.35f;
      r.strokeLine(inner.x, y, cx - tw * 0.5f - gap, y, 1.0f,
                   colors::kPanelSeparator);
      r.strokeLine(cx + tw * 0.5f + gap, y, inner.x + inner.w, y, 1.0f,
                   colors::kPanelSeparator);
      r.fillText(cx, y, section.title.c_str(), headerSize, TextAlign::Center,
                 colors::kWhite);
      y += labelSize * 1.6f;
    } else if (!section.title.empty()) {
      r.fillText(inner.x + inner.w * 0.5f, y, section.title.c_str(), labelSize,
                 TextAlign::Center, colors::kLabelText);
      y += labelSize * 0.6f;
    }
    for (const EisGauge& gauge : section.gauges) {
      drawGauge(r, d, gauge, inner, y, valid, displayH, barStride, labelSize);
      if (++drawn < gaugeCount) y += extra;
    }
  }
}

}  // namespace avionics::mfd
