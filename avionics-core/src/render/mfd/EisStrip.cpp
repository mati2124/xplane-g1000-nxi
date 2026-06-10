#include "render/mfd/EisStrip.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "avionics/Color.h"

namespace avionics::mfd {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Cessna 172S gauge scales and color bands (172S POH / G1000 Pilot's Guide for
// Cessna Nav III, Section 3.1). RPM green-arc top is the sea-level value; the
// real unit expands it with altitude (Figure 3-7), which needs pressure
// altitude trends we don't model here.
constexpr float kRpmMax = 3000.0f;
constexpr float kRpmGreenLo = 2100.0f;
constexpr float kRpmGreenHi = 2500.0f;
constexpr float kRpmRedline = 2700.0f;

constexpr float kFflowMax = 15.0f;
constexpr float kFflowGreenHi = 12.0f;

constexpr float kOilPresMax = 115.0f;
constexpr float kOilPresRedLo = 20.0f;
constexpr float kOilPresGreenLo = 50.0f;
constexpr float kOilPresGreenHi = 90.0f;
constexpr float kOilPresRedHi = 110.0f;

constexpr float kOilTempMin = 100.0f;
constexpr float kOilTempMax = 250.0f;
constexpr float kOilTempRedLine = 245.0f;

constexpr float kEgtMin = 1250.0f;
constexpr float kEgtMax = 1650.0f;

constexpr float kVacMin = 3.0f;
constexpr float kVacMax = 7.0f;
constexpr float kVacGreenLo = 4.5f;
constexpr float kVacGreenHi = 5.5f;

constexpr float kFuelMaxGal = 24.0f;  // usable per side, 172R/172S
constexpr float kFuelRedHi = 1.5f;

// Font weights on the shared 768 canvas.
constexpr float kLabelWt = 13.0f;
constexpr float kValueWt = 16.0f;
constexpr float kRpmReadoutWt = 22.0f;

std::string fmt(const char* pattern, double v) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), pattern, v);
  return buf;
}

// Tessellated arc stroke between two angles (deg, 0 = up, clockwise positive)
// around (cx, cy).
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

// The 172 tachometer: a 270-degree dial sweeping clockwise from the lower
// left, green normal-operating arc, red overspeed arc, white needle, and a
// digital readout in the lower half of the dial.
void drawRpmDial(Renderer& r, const FlightData& d, const Rect& area,
                 float displayH) {
  const float cx = area.x + area.w * 0.5f;
  const float cy = area.y + area.h * 0.52f;
  const float radius = std::min(area.w * 0.40f, area.h * 0.42f);

  // 0 RPM points down-left (-135 deg), max RPM down-right (+135 deg).
  constexpr float kStartDeg = -135.0f;
  constexpr float kSweepDeg = 270.0f;
  auto angleFor = [&](float rpm) {
    const float frac = std::max(0.0f, std::min(1.0f, rpm / kRpmMax));
    return kStartDeg + kSweepDeg * frac;
  };

  strokeArc(r, cx, cy, radius, kStartDeg, kStartDeg + kSweepDeg, 1.5f,
            colors::kPanelBorder);
  strokeArc(r, cx, cy, radius - 1.0f, angleFor(kRpmGreenLo),
            angleFor(kRpmGreenHi), 4.0f, colors::kBandGreen);
  strokeArc(r, cx, cy, radius - 1.0f, angleFor(kRpmRedline), angleFor(kRpmMax),
            4.0f, colors::kBandRed);

  // Major ticks + hundreds labels every 1000 RPM, minor ticks every 500.
  const float labelSize = mfdFontPx(kLabelWt, displayH);
  for (int rpm = 0; rpm <= static_cast<int>(kRpmMax); rpm += 500) {
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

  // Needle: white pointer from the hub, drawn only with live data.
  const bool valid = d.dataLinkValid;
  if (valid) {
    const float a = angleFor(d.engineRpm) * kPi / 180.0f;
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

  r.fillText(cx, cy - radius * 0.34f, "RPM", labelSize, TextAlign::Center,
             colors::kLabelText);
  const bool overspeed = valid && d.engineRpm >= kRpmRedline;
  r.fillText(cx, cy + radius * 0.52f,
             valid ? fmt("%.0f", std::round(d.engineRpm / 10.0) * 10.0)
                   : std::string("____"),
             mfdFontPx(kRpmReadoutWt, displayH), TextAlign::Center,
             overspeed ? colors::kBandRed : colors::kWhite);
}

// One horizontal bar indicator: "LABEL" line above a thin track with color
// bands and a white pointer triangle riding on top of the track.
struct BarBand {
  float lo, hi;
  Color color;
};

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

// "LABEL  value" readout row (e.g. ENG HRS), value right-aligned.
float drawReadout(Renderer& r, const Rect& area, float y, const char* label,
                  const std::string& value, float displayH) {
  r.fillText(area.x, y, label, mfdFontPx(kLabelWt, displayH), TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w, y, value, mfdFontPx(kValueWt, displayH),
             TextAlign::Right, colors::kWhite);
  return y + mfdFontPx(kValueWt, displayH) * 1.35f;
}

// Two-column electrical row: "M <value>  <S/E> <value>" under a group label.
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

}  // namespace

void drawEisStrip(Renderer& r, const FlightData& d, const Rect& area,
                  float displayH) {
  // Strip face: same dark panel gradient as the top bar, with a separator
  // along the right edge against the page body.
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

  // Title, then the tachometer dial.
  float y = area.y + labelSize * 1.4f;
  r.fillText(area.x + area.w * 0.5f, y, "ENGINE", mfdFontPx(kValueWt, displayH),
             TextAlign::Center, colors::kWhite);
  y += labelSize * 0.8f;

  const float dialH = area.h * 0.26f;
  drawRpmDial(r, d, Rect{inner.x, y, inner.w, dialH}, displayH);
  y += dialH + labelSize * 0.9f;

  // Horizontal bar indicators, top to bottom in the real Engine Display order.
  const BarBand fflowBands[] = {{0.0f, kFflowGreenHi, colors::kBandGreen}};
  drawBar(r, inner, y, "FFLOW GPH", d.fuelFlowGph, 0.0f, kFflowMax, fflowBands,
          1, valid, displayH);
  y += barStride;

  const BarBand oilPresBands[] = {
      {0.0f, kOilPresRedLo, colors::kBandRed},
      {kOilPresGreenLo, kOilPresGreenHi, colors::kBandGreen},
      {kOilPresRedHi, kOilPresMax, colors::kBandRed}};
  drawBar(r, inner, y, "OIL PRES", d.oilPressurePsi, 0.0f, kOilPresMax,
          oilPresBands, 3, valid, displayH);
  y += barStride;

  const BarBand oilTempBands[] = {
      {kOilTempMin, kOilTempRedLine, colors::kBandGreen},
      {kOilTempRedLine, kOilTempMax, colors::kBandRed}};
  drawBar(r, inner, y, "OIL TEMP", d.oilTempDegF, kOilTempMin, kOilTempMax,
          oilTempBands, 2, valid, displayH);
  y += barStride;

  drawBar(r, inner, y, "EGT \xC2\xB0""F", d.egtDegF, kEgtMin, kEgtMax, nullptr,
          0, valid, displayH);
  y += barStride;

  const BarBand vacBands[] = {{kVacGreenLo, kVacGreenHi, colors::kBandGreen}};
  drawBar(r, inner, y, "VAC", d.vacuumInHg, kVacMin, kVacMax, vacBands, 1,
          valid, displayH);
  y += barStride;

  // Fuel quantity: one bar per tank (left and right), shared group label.
  r.fillText(inner.x + inner.w * 0.5f, y, "FUEL QTY GAL", labelSize,
             TextAlign::Center, colors::kLabelText);
  y += labelSize * 0.6f;
  const BarBand fuelBands[] = {{0.0f, kFuelRedHi, colors::kBandRed},
                               {kFuelRedHi, kFuelMaxGal, colors::kBandGreen}};
  drawBar(r, inner, y, "L", d.fuelQtyLeftGal, 0.0f, kFuelMaxGal, fuelBands, 2,
          valid, displayH);
  y += barStride;
  drawBar(r, inner, y, "R", d.fuelQtyRightGal, 0.0f, kFuelMaxGal, fuelBands, 2,
          valid, displayH);
  y += barStride * 1.1f;

  // Engine hours (tach), then the electrical group.
  y = drawReadout(r, inner, y, "ENG HRS",
                  valid ? fmt("%.1f", d.engineHours) : std::string("____._"),
                  displayH);
  y += labelSize * 0.5f;

  r.strokeLine(inner.x, y, inner.x + inner.w, y, 1.0f,
               colors::kPanelSeparator);
  y += labelSize * 1.2f;
  r.fillText(inner.x + inner.w * 0.5f, y, "ELECTRICAL",
             mfdFontPx(kValueWt, displayH), TextAlign::Center, colors::kWhite);
  y += labelSize * 1.5f;
  y = drawElectricalRow(r, inner, y, "BUS VOLTS", "M", d.busVoltsMain, "E",
                        d.busVoltsEssential, "%.1f", valid, displayH);
  y = drawElectricalRow(r, inner, y, "BATT AMPS", "M", d.battAmpsMain, "S",
                        d.battAmpsStandby, "%+.0f", valid, displayH);
}

}  // namespace avionics::mfd
