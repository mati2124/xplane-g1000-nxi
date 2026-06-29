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
               float a1Deg, float widthPx, const Color& c) {
  constexpr int kSegments = 24;
  Point pts[kSegments + 1];
  for (int i = 0; i <= kSegments; ++i) {
    const float a =
        (a0Deg + (a1Deg - a0Deg) * static_cast<float>(i) / kSegments) *
        kPi / 180.0f;
    pts[i] = {cx + radius * std::sin(a), cy - radius * std::cos(a)};
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
  r.fillText(area.x + area.w * 0.5f, y, label, labelSize, TextAlign::Center,
             colors::kLabelText);

  // The real unit leaves a clear gap between the label and its bar (the
  // pointer rides up into it), so sit the track a little lower than the label.
  const float axisY = y + labelSize * 1.35f;
  const float bandH = labelSize * 0.48f;
  const float x0 = area.x;
  const float w = area.w;
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
  r.fillText(area.x, y, label, mfdFontPx(kLabelWt, displayH), TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w, y, value, mfdFontPx(kValueWt, displayH),
             TextAlign::Right, colors::kWhite);
  return y + mfdFontPx(kValueWt, displayH) * 1.35f;
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
    case EisGaugeType::Readout:
      return valueSize * 1.35f + labelSize * 0.5f;
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
    case EisGaugeType::Readout: {
      const float value = eisChannelValue(d, gauge.channel, 0.0f);
      y = drawReadout(
          r, inner, y, gauge.label.c_str(),
          valid ? fmt(gauge.format.c_str(), value) : std::string("____._"),
          displayH);
      y += labelSize * 0.5f;
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

float eisStripWidthFrac(EisStripStyle style) {
  // 150/1024 piston; 237/1024 turbofan. The turbofan strip is sized so its
  // width-to-height ratio matches the real Perspective Touch+ EIS (strip w/h
  // ~0.349 over the body height), which a narrower strip made look cramped.
  return style == EisStripStyle::Turbofan ? 237.0f / 1024.0f : 150.0f / 1024.0f;
}

void drawEisStrip(Renderer& r, const FlightData& d, const EisLayout& layout,
                  const Rect& area, float displayH, bool reduced) {
  // The Cirrus Vision SF50 jet has a fundamentally different engine page (a
  // dense turbine + synoptic grid) than the piston Cessna stack below.
  if (layout.style == EisStripStyle::Turbofan) {
    drawEisStripTurbofan(r, d, layout, area, displayH, reduced);
    return;
  }
  // The piston (Cessna Nav III) stack already fits the PFD reversionary column,
  // so `reduced` has no effect on it.

  // Solid black backing for the whole engine instrument strip (real unit).
  r.fillRect(area.x, area.y, area.w, area.h, colors::kBlack);
  r.strokeLine(area.x + area.w, area.y, area.x + area.w, area.y + area.h, 2.0f,
               colors::kPanelBorder);

  // The real EIS labels, readouts and scales are all in the bold (SemiBold)
  // face, so render the whole strip's default text bold.
  FontScope stripFont(r, FontFace::DejaVuSemiBold);

  const bool valid = d.dataLinkValid;
  const float pad = area.w * 0.10f;
  const Rect inner{area.x + pad, area.y, area.w - 2.0f * pad, area.h};
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  const float barStride = labelSize * 3.1f;

  // No strip heading on the real unit: the RPM dial sits near the top with a
  // small breathing gap above the arc.
  const float topPad = labelSize * 0.9f;

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
  const float bottomMargin = labelSize * 1.2f;
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
