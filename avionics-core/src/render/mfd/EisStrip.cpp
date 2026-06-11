#include "render/mfd/EisStrip.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

std::string fmt(const char* pattern, double v) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), pattern, v);
  return buf;
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

void drawRpmDial(Renderer& r, const FlightData& d, const EisGauge& gauge,
                 const Rect& area, float displayH) {
  const float cx = area.x + area.w * 0.5f;
  const float cy = area.y + area.h * 0.52f;
  const float radius = std::min(area.w * 0.40f, area.h * 0.42f);

  constexpr float kStartDeg = -135.0f;
  constexpr float kSweepDeg = 270.0f;
  auto angleFor = [&](float rpm) {
    const float frac =
        std::max(0.0f, std::min(1.0f, rpm / gauge.max));
    return kStartDeg + kSweepDeg * frac;
  };

  strokeArc(r, cx, cy, radius, kStartDeg, kStartDeg + kSweepDeg, 1.5f,
            colors::kPanelBorder);
  for (const EisBand& band : gauge.bands) {
    strokeArc(r, cx, cy, radius - 1.0f, angleFor(band.lo), angleFor(band.hi),
              4.0f, eisBandColor(band.color));
  }

  const float labelSize = mfdFontPx(kLabelWt, displayH);
  for (int rpm = 0; rpm <= static_cast<int>(gauge.max); rpm += 500) {
    const float a = angleFor(static_cast<float>(rpm)) * kPi / 180.0f;
    const bool major = rpm % 1000 == 0;
    const float inner = radius * (major ? 0.84f : 0.90f);
    r.strokeLine(cx + inner * std::sin(a), cy - inner * std::cos(a),
                 cx + radius * std::sin(a), cy - radius * std::cos(a),
                 major ? 2.0f : 1.0f, colors::kWhite);
    if (major && rpm > 0) {
      const float lr = radius * 0.68f;
      r.fillText(cx + lr * std::sin(a), cy - lr * std::cos(a),
                 fmt("%.0f", rpm / 100.0), labelSize, TextAlign::Center,
                 colors::kWhite);
    }
  }

  const bool valid = d.dataLinkValid;
  const float rpm =
      eisChannelValue(d, gauge.channel, d.engineRpm);
  if (valid) {
    const float a = angleFor(rpm) * kPi / 180.0f;
    const float ca = std::cos(a);
    const float sa = std::sin(a);
    const float tip = radius * 0.92f;
    const float half = radius * 0.055f;
    const Point needle[3] = {
        {cx + tip * sa, cy - tip * ca},
        {cx - half * ca - radius * 0.10f * sa, cy - half * sa + radius * 0.10f * ca},
        {cx + half * ca - radius * 0.10f * sa, cy + half * sa + radius * 0.10f * ca},
    };
    r.fillPolygon(needle, 3, colors::kWhite);
  }
  r.fillCircle(cx, cy, radius * 0.07f, colors::kPanelBorder);

  const char* dialLabel =
      gauge.label.empty() ? "RPM" : gauge.label.c_str();
  r.fillText(cx, cy - radius * 0.34f, dialLabel, labelSize, TextAlign::Center,
             colors::kLabelText);
  const bool overspeed =
      valid && gauge.hasRedline && rpm >= gauge.redline;
  // Place the digital readout in the open area below the arc's bottom
  // graduation labels (which sit near cy + 0.48*radius) so the large value
  // does not overlap the max ("30") label, like a real tachometer's readout.
  r.fillText(cx, cy + radius * 1.1f,
             valid ? fmt("%.0f", std::round(rpm / 10.0) * 10.0)
                   : std::string("____"),
             mfdFontPx(kRpmReadoutWt, displayH), TextAlign::Center,
             overspeed ? colors::kBandRed : colors::kWhite);
}

void drawBar(Renderer& r, const Rect& area, float y, const char* label,
             float value, float minV, float maxV, const BarBand* bands,
             int bandCount, bool valid, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  r.fillText(area.x, y, label, labelSize, TextAlign::Left, colors::kLabelText);

  const float trackY = y + labelSize * 1.15f;
  const float trackH = labelSize * 0.38f;
  const float x0 = area.x;
  const float w = area.w;
  auto xFor = [&](float v) {
    const float frac =
        std::max(0.0f, std::min(1.0f, (v - minV) / (maxV - minV)));
    return x0 + w * frac;
  };

  r.fillRect(x0, trackY, w, trackH, Color{0.16f, 0.16f, 0.16f, 1.0f});
  for (int i = 0; i < bandCount; ++i) {
    r.fillRect(xFor(bands[i].lo), trackY, xFor(bands[i].hi) - xFor(bands[i].lo),
               trackH, bands[i].color);
  }
  r.strokeLine(x0, trackY + trackH, x0 + w, trackY + trackH, 1.0f,
               colors::kPanelSeparator);

  if (!valid) return;
  const float px = xFor(value);
  const float ph = trackH * 1.9f;
  const Point pointer[3] = {{px, trackY + trackH * 0.4f},
                            {px - ph * 0.45f, trackY - ph * 0.75f},
                            {px + ph * 0.45f, trackY - ph * 0.75f}};
  r.fillPolygon(pointer, 3, colors::kWhite);
}

float drawReadout(Renderer& r, const Rect& area, float y, const char* label,
                  const std::string& value, float displayH) {
  r.fillText(area.x, y, label, mfdFontPx(kLabelWt, displayH), TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w, y, value, mfdFontPx(kValueWt, displayH),
             TextAlign::Right, colors::kWhite);
  return y + mfdFontPx(kValueWt, displayH) * 1.35f;
}

float drawElectricalRow(Renderer& r, const Rect& area, float y,
                        const char* label, const char* leftTag, float leftVal,
                        const char* rightTag, float rightVal,
                        const char* pattern, bool valid, float displayH) {
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float valueSize = mfdFontPx(kValueWt, displayH);
  r.fillText(area.x + area.w * 0.5f, y, label, labelSize, TextAlign::Center,
             colors::kLabelText);
  y += labelSize * 1.3f;
  const std::string lv = valid ? fmt(pattern, leftVal) : std::string("___");
  const std::string rv = valid ? fmt(pattern, rightVal) : std::string("___");
  r.fillText(area.x, y, leftTag, labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w * 0.46f, y, lv, valueSize, TextAlign::Right,
             colors::kWhite);
  r.fillText(area.x + area.w * 0.54f, y, rightTag, labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w, y, rv, valueSize, TextAlign::Right,
             colors::kWhite);
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

void drawGauge(Renderer& r, const FlightData& d, const EisGauge& gauge,
               const Rect& inner, float& y, bool valid, float displayH,
               float barStride, float labelSize) {
  switch (gauge.type) {
    case EisGaugeType::RpmDial: {
      const float dialH = inner.h * 0.26f;
      drawRpmDial(r, d, gauge, Rect{inner.x, y, inner.w, dialH}, displayH);
      y += dialH + labelSize * 0.9f;
      break;
    }
    case EisGaugeType::Bar: {
      const std::vector<BarBand> bands = toBarBands(gauge);
      const float value = eisChannelValue(d, gauge.channel, 0.0f);
      drawBar(r, inner, y, gauge.label.c_str(), value, gauge.min, gauge.max,
              bands.empty() ? nullptr : bands.data(),
              static_cast<int>(bands.size()), valid, displayH);
      y += barStride;
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
      y = drawElectricalRow(r, inner, y, gauge.label.c_str(),
                            gauge.leftTag.c_str(), leftVal,
                            gauge.rightTag.c_str(), rightVal,
                            gauge.format.c_str(), valid, displayH);
      break;
    }
  }
}

}  // namespace

void drawEisStrip(Renderer& r, const FlightData& d, const EisLayout& layout,
                  const Rect& area, float displayH) {
  r.fillRectVerticalGradient(area.x, area.y, area.w, area.h, area.y,
                             area.y + area.h, colors::kPanelBackgroundBottom,
                             colors::kPanelBackground);
  r.strokeLine(area.x + area.w, area.y, area.x + area.w, area.y + area.h, 2.0f,
               colors::kPanelBorder);

  const bool valid = d.dataLinkValid;
  const float pad = area.w * 0.10f;
  const Rect inner{area.x + pad, area.y, area.w - 2.0f * pad, area.h};
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  const float barStride = labelSize * 3.1f;

  float y = area.y + labelSize * 1.4f;
  r.fillText(area.x + area.w * 0.5f, y, layout.stripTitle.c_str(),
             mfdFontPx(kValueWt, displayH), TextAlign::Center, colors::kWhite);
  y += labelSize * 0.8f;

  for (const EisSection& section : layout.sections) {
    if (section.title == "ELECTRICAL") {
      r.strokeLine(inner.x, y, inner.x + inner.w, y, 1.0f,
                   colors::kPanelSeparator);
      y += labelSize * 1.2f;
      r.fillText(inner.x + inner.w * 0.5f, y, section.title.c_str(),
                 mfdFontPx(kValueWt, displayH), TextAlign::Center,
                 colors::kWhite);
      y += labelSize * 1.5f;
    } else if (!section.title.empty()) {
      r.fillText(inner.x + inner.w * 0.5f, y, section.title.c_str(), labelSize,
                 TextAlign::Center, colors::kLabelText);
      y += labelSize * 0.6f;
    }
    for (const EisGauge& gauge : section.gauges) {
      drawGauge(r, d, gauge, inner, y, valid, displayH, barStride, labelSize);
    }
    if (section.title == "FUEL QTY GAL") {
      y += barStride * 0.1f;
    }
  }
}

}  // namespace avionics::mfd
