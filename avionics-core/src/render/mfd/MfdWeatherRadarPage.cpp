#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/WeatherRadar.h"
#include "render/mfd/MfdPages.h"
#include "render/mfd/MfdStyle.h"

// MAP - Weather Radar page (G1000 NXi Pilot's Guide, Hazard Avoidance -
// Airborne Color Weather Radar). Modeled off the real unit: the horizontal
// scan is a forward +/-45 deg wedge with the ownship at the bottom apex, dashed
// range arcs labeled in NM, color-graded precipitation returns, an animated
// scan line, an optional cyan bearing line, the mode annunciation (upper-left),
// the antenna stabilization / altitude-compensated-tilt status (upper-right),
// the Scale legend (lower-left), and the Tilt/Bearing/Sector Scan/Gain readout
// box (lower-right). The vertical scan shows a range-vs-altitude slice.
namespace avionics::mfd {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

// ---- color ramps -----------------------------------------------------------

// Weather mode precipitation intensity (GWX 70 table: green .01-.1, yellow
// .1-.5, red >=.5 in/hr; magenta is reserved for the heaviest cores). Returns
// are opaque on the black scope.
Color weatherColor(float t) {
  struct Stop {
    float t;
    Color c;
  };
  static constexpr Stop kStops[] = {
      {0.15f, {0.00f, 0.80f, 0.00f, 1.0f}},  // green
      {0.45f, {0.95f, 0.95f, 0.00f, 1.0f}},  // yellow
      {0.72f, {1.00f, 0.00f, 0.00f, 1.0f}},  // red
      {1.00f, {1.00f, 0.00f, 1.00f, 1.0f}},  // magenta (heaviest)
  };
  constexpr int n = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
  if (t <= kStops[0].t) return kStops[0].c;
  for (int i = 1; i < n; ++i) {
    if (t <= kStops[i].t) {
      const Color& a = kStops[i - 1].c;
      const Color& b = kStops[i].c;
      const float u = (t - kStops[i - 1].t) / (kStops[i].t - kStops[i - 1].t);
      return {a.r + (b.r - a.r) * u, a.g + (b.g - a.g) * u,
              a.b + (b.b - a.b) * u, 1.0f};
    }
  }
  return kStops[n - 1].c;
}

// Ground-map intensity (GWX: cyan / yellow / magenta for terrain returns).
Color groundColor(float t) {
  struct Stop {
    float t;
    Color c;
  };
  static constexpr Stop kStops[] = {
      {0.15f, {0.00f, 0.75f, 0.85f, 1.0f}},  // cyan
      {0.55f, {0.95f, 0.95f, 0.00f, 1.0f}},  // yellow
      {1.00f, {1.00f, 0.00f, 1.00f, 1.0f}},  // magenta
  };
  constexpr int n = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
  if (t <= kStops[0].t) return kStops[0].c;
  for (int i = 1; i < n; ++i) {
    if (t <= kStops[i].t) {
      const Color& a = kStops[i - 1].c;
      const Color& b = kStops[i].c;
      const float u = (t - kStops[i - 1].t) / (kStops[i].t - kStops[i - 1].t);
      return {a.r + (b.r - a.r) * u, a.g + (b.g - a.g) * u,
              a.b + (b.b - a.b) * u, 1.0f};
    }
  }
  return kStops[n - 1].c;
}

void writePixel(unsigned char* px, const Color& c, float alpha) {
  px[0] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.r)) * 255.0f));
  px[1] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.g)) * 255.0f));
  px[2] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.b)) * 255.0f));
  px[3] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, alpha)) * 255.0f));
}

// Sample the radar source's return-strength grid at a forward distance (NM) and
// cross-track offset (NM), returning 0..1. The grid has the aircraft at the
// bottom edge with forward toward the top, the left/right edges at +/-
// halfWidthNm. Out-of-range samples read 0.
float sampleStrength(const WeatherRadarSource& src, float forwardNm,
                     float crossNm) {
  const float range = std::max(1.0f, src.rangeNm());
  const float half = std::max(1.0f, src.halfWidthNm());
  const float ff = forwardNm / range;
  const float cf = 0.5f + crossNm / (2.0f * half);
  if (ff < 0.0f || ff > 1.0f || cf < 0.0f || cf > 1.0f) return 0.0f;
  const int w = src.width();
  const int h = src.height();
  if (w <= 0 || h <= 0) return 0.0f;
  // Row 0 is the far edge (top), row h-1 is the aircraft (bottom).
  const int col = std::min(w - 1, static_cast<int>(cf * (w - 1)));
  const int row = std::min(h - 1, static_cast<int>((1.0f - ff) * (h - 1)));
  return static_cast<float>(src.returnStrength()[row * w + col]) / 255.0f;
}

// ---- per-renderer return-image cache ---------------------------------------

struct RadarImage {
  Renderer* renderer = nullptr;
  int imageId = -1;
  int w = 0;
  int h = 0;
  // Inputs that change the pixels; a mismatch triggers a rebuild.
  int revision = -1;
  float rangeNm = -1.0f;
  int mode = -1;
  int scan = -1;
  int sector = -1;
  float bearing = 0.0f;
  bool calibrated = true;
  float gain = 0.0f;
  std::vector<unsigned char> rgba;
};

RadarImage& imageFor(Renderer& r) {
  static std::vector<RadarImage> images;
  for (RadarImage& img : images) {
    if (img.renderer == &r) return img;
  }
  images.push_back(RadarImage{});
  images.back().renderer = &r;
  return images.back();
}

// Build the horizontal-scan returns into rgba (apex at bottom-center, forward
// up, +/-45 deg). Pixels outside the wedge / sector are transparent.
void buildHorizontal(std::vector<unsigned char>& rgba, int w, int h,
                     const WeatherRadarSource& src, float fullRangeNm,
                     RadarMode mode, float sectorHalfDeg, float sectorCenterDeg,
                     bool calibrated, float gain) {
  rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
  const float apexX = (w - 1) * 0.5f;
  const float apexY = static_cast<float>(h - 1);
  const float gainScale = calibrated ? 1.0f : (1.0f + gain);
  for (int row = 0; row < h; ++row) {
    for (int col = 0; col < w; ++col) {
      const float dx = (col - apexX) / apexX;        // -1..1 across the wedge
      const float dy = (apexY - row) / (h - 1);       // 0..1 forward
      const float range = std::sqrt(dx * dx + dy * dy);
      if (range > 1.0f) continue;
      const float bearingDeg = std::atan2(dx, dy) / kDegToRad;
      if (std::fabs(bearingDeg) > 45.0f) continue;
      if (std::fabs(bearingDeg - sectorCenterDeg) > sectorHalfDeg) continue;
      const float forwardNm = dy * fullRangeNm;
      const float crossNm = dx * fullRangeNm;
      float s = sampleStrength(src, forwardNm, crossNm) * gainScale;
      s = std::min(1.0f, s);
      if (s < 0.15f) continue;
      const Color c =
          mode == RadarMode::Ground ? groundColor(s) : weatherColor(s);
      writePixel(&rgba[(static_cast<std::size_t>(row) * w + col) * 4], c, 1.0f);
    }
  }
}

// Build the vertical-scan slice: range along x, altitude along y, the wedge
// opening to the right. Returns sampled along the bearing radial are spread
// across a vertical band (the suite has no true vertical reflectivity, so the
// slice approximates the storm's vertical extent from the horizontal field).
void buildVertical(std::vector<unsigned char>& rgba, int w, int h,
                   const WeatherRadarSource& src, float fullRangeNm,
                   RadarMode mode, float bearingDeg, bool calibrated,
                   float gain) {
  rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
  const float gainScale = calibrated ? 1.0f : (1.0f + gain);
  const float halfH = (h - 1) * 0.5f;
  const float br = bearingDeg * kDegToRad;
  for (int col = 0; col < w; ++col) {
    const float rangeFrac = static_cast<float>(col) / (w - 1);
    const float distNm = rangeFrac * fullRangeNm;
    const float forwardNm = distNm * std::cos(br);
    const float crossNm = distNm * std::sin(br);
    float s = sampleStrength(src, forwardNm, crossNm) * gainScale;
    s = std::min(1.0f, s);
    if (s < 0.15f) continue;
    const Color c =
        mode == RadarMode::Ground ? groundColor(s) : weatherColor(s);
    // Vertical band: inside the +/- wedge (|alt| <= rangeFrac), tapering away
    // from the beam centerline.
    for (int row = 0; row < h; ++row) {
      const float altFrac = (row - halfH) / halfH;  // -1..1
      if (std::fabs(altFrac) > rangeFrac) continue;
      const float g = std::exp(-(altFrac * 3.0f) * (altFrac * 3.0f));
      if (g < 0.05f) continue;
      writePixel(&rgba[(static_cast<std::size_t>(row) * w + col) * 4], c, g);
    }
  }
}

// ---- vector helpers ---------------------------------------------------------

// Dashed arc centered on (cx,cy), bearings measured from straight up with +to
// the right. Drawn as alternating short segments.
void dashedArc(Renderer& r, float cx, float cy, float radius, float a0Deg,
               float a1Deg, float widthPx, const Color& c) {
  constexpr int kSegs = 48;
  bool on = true;
  for (int i = 0; i < kSegs; ++i) {
    const float f0 = static_cast<float>(i) / kSegs;
    const float f1 = static_cast<float>(i + 1) / kSegs;
    const float b0 = (a0Deg + (a1Deg - a0Deg) * f0) * kDegToRad;
    const float b1 = (a0Deg + (a1Deg - a0Deg) * f1) * kDegToRad;
    if (on) {
      r.strokeLine(cx + radius * std::sin(b0), cy - radius * std::cos(b0),
                   cx + radius * std::sin(b1), cy - radius * std::cos(b1),
                   widthPx, c);
    }
    on = !on;
  }
}

// Text inside a thin gray-bordered box, like the radar mode / status / range
// annunciations on the real page.
void annunBox(Renderer& r, float cx, float cy, const std::string& text,
              float sizePx, const Color& textColor) {
  const float tw = r.measureTextWidth(text, sizePx);
  const float padX = sizePx * 0.35f;
  const float padY = sizePx * 0.28f;
  const float bw = tw + 2.0f * padX;
  const float bh = sizePx + 2.0f * padY;
  const float bx = cx - bw * 0.5f;
  const float by = cy - bh * 0.5f;
  r.fillRect(bx, by, bw, bh, mfdAlpha(colors::kBlack, 0.55f));
  const Point frame[5] = {{bx, by},
                          {bx + bw, by},
                          {bx + bw, by + bh},
                          {bx, by + bh},
                          {bx, by}};
  r.strokePolyline(frame, 5, 1.0f, colors::kGroupBoxBorder);
  r.fillText(cx, cy, text, sizePx, TextAlign::Center, textColor);
}

const char* sectorLabel(RadarSector s) {
  switch (s) {
    case RadarSector::Sixty:
      return "60\xC2\xB0";
    case RadarSector::Forty:
      return "40\xC2\xB0";
    case RadarSector::Twenty:
      return "20\xC2\xB0";
    case RadarSector::Full:
      break;
  }
  return "Full";
}

float sectorHalfDeg(RadarSector s) {
  switch (s) {
    case RadarSector::Sixty:
      return 30.0f;
    case RadarSector::Forty:
      return 20.0f;
    case RadarSector::Twenty:
      return 10.0f;
    case RadarSector::Full:
      break;
  }
  return 45.0f;
}

// The Scale legend (lower-left): a vertical color bar with Heavy at the top and
// Light at the bottom, titled "Scale".
void drawScaleLegend(Renderer& r, float x, float y, float w, float h,
                     RadarMode mode, float displayH) {
  const float titleSize = mfdFontPx(13.0f, displayH);
  r.fillText(x + w * 0.5f, y, "Scale", titleSize, TextAlign::Center,
             colors::kTitleGray);
  const float barX = x + w * 0.30f;
  const float barW = w * 0.40f;
  const float barTop = y + titleSize * 1.1f;
  const float barH = h - titleSize * 1.1f;
  // Four equal bands, heaviest at the top (magenta/red/yellow/green for
  // weather; the ground ramp uses cyan/yellow/magenta).
  const int n = 4;
  Color ramp[4];
  if (mode == RadarMode::Ground) {
    ramp[0] = groundColor(1.0f);
    ramp[1] = groundColor(0.7f);
    ramp[2] = groundColor(0.4f);
    ramp[3] = groundColor(0.2f);
  } else {
    ramp[0] = weatherColor(1.0f);   // magenta
    ramp[1] = weatherColor(0.78f);  // red
    ramp[2] = weatherColor(0.5f);   // yellow
    ramp[3] = weatherColor(0.2f);   // green
  }
  const float bandH = barH / n;
  for (int i = 0; i < n; ++i) {
    r.fillRect(barX, barTop + i * bandH, barW, bandH + 0.5f, ramp[i]);
  }
  const Point frame[5] = {{barX, barTop},
                          {barX + barW, barTop},
                          {barX + barW, barTop + barH},
                          {barX, barTop + barH},
                          {barX, barTop}};
  r.strokePolyline(frame, 5, 1.0f, colors::kGroupBoxBorder);
  const float lblSize = mfdFontPx(12.0f, displayH);
  r.fillText(barX + barW + mfdFontPx(3.0f, displayH), barTop + lblSize * 0.4f,
             "Heavy", lblSize, TextAlign::Left, colors::kTitleGray);
  r.fillText(barX + barW + mfdFontPx(3.0f, displayH),
             barTop + barH - lblSize * 0.4f, "Light", lblSize, TextAlign::Left,
             colors::kTitleGray);
}

// The Tilt / Bearing / Sector Scan / Gain readout box (lower-right).
void drawSettingsBox(Renderer& r, float x, float y, float w, float h,
                     const MfdController& ui, float displayH) {
  r.fillRect(x, y, w, h, mfdAlpha(colors::kBlack, 0.6f));
  const Point frame[5] = {
      {x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}};
  r.strokePolyline(frame, 5, 1.0f, colors::kGroupBoxBorder);

  const float rowH = h / 4.0f;
  const float labelSize = mfdFontPx(14.0f, displayH);
  const float valueSize = mfdFontPx(15.0f, displayH);
  const float lx = x + mfdFontPx(6.0f, displayH);
  const float rx = x + w - mfdFontPx(6.0f, displayH);
  char buf[24];

  auto rowCy = [&](int i) { return y + rowH * (i + 0.5f); };

  // Tilt: UP/DN with hundredths of a degree (Figure 6-70 shows "DN 0.25").
  const float tilt = ui.radarTiltDeg();
  std::snprintf(buf, sizeof(buf), "%s %.2f\xC2\xB0", tilt < 0.0f ? "DN" : "UP",
                std::fabs(tilt));
  r.fillText(lx, rowCy(0), "Tilt", labelSize, TextAlign::Left,
             colors::kTitleGray);
  r.fillText(rx, rowCy(0), buf, valueSize, TextAlign::Right, colors::kCyan);

  // Bearing: shown when the bearing line is up, else dashes.
  r.fillText(lx, rowCy(1), "Bearing", labelSize, TextAlign::Left,
             colors::kTitleGray);
  std::string brg = "\xE2\x80\x93\xE2\x80\x93";
  if (ui.radarBearingLineOn()) {
    std::snprintf(buf, sizeof(buf), "%+.0f\xC2\xB0", ui.radarBearingDeg());
    brg = buf;
  }
  r.fillText(rx, rowCy(1), brg, valueSize, TextAlign::Right, colors::kCyan);

  // Sector Scan width.
  r.fillText(lx, rowCy(2), "Sector Scan", labelSize, TextAlign::Left,
             colors::kTitleGray);
  r.fillText(rx, rowCy(2), sectorLabel(ui.radarSector()), valueSize,
             TextAlign::Right, colors::kWhite);

  // Gain: Calibrated or a manual indication.
  r.fillText(lx, rowCy(3), "Gain", labelSize, TextAlign::Left,
             colors::kTitleGray);
  const bool cal = ui.radarGainCalibrated();
  r.fillText(rx, rowCy(3), cal ? "Calibrated" : "Manual", valueSize,
             TextAlign::Right, cal ? colors::kWhite : colors::kBandYellow);
}

}  // namespace

void drawWeatherRadarPage(Renderer& r, const FlightData& d, const MapData& map,
                          const MfdController& ui, float x, float y, float w,
                          float h, float displayH) {
  (void)d;
  r.fillRect(x, y, w, h, colors::kBlack);

  const RadarMode mode = ui.radarMode();
  const RadarScan scan = ui.radarScan();
  const float fullRange = std::max(1.0f, ui.rangeNm());
  const WeatherRadarSource* src = map.weather;
  const bool haveReturns = src != nullptr && src->active() &&
                           mode != RadarMode::Standby;

  // ---- annunciations (drawn over every state) ----
  const float annSize = mfdFontPx(15.0f, displayH);
  const char* modeText = mode == RadarMode::Weather  ? "Weather"
                         : mode == RadarMode::Ground ? "Ground"
                                                     : "Standby";
  annunBox(r, x + mfdFontPx(48.0f, displayH), y + mfdFontPx(16.0f, displayH),
           modeText, annSize, colors::kWhite);

  // Feature status, upper-right (STAB On/Off; Altitude Comp Tilt On/Off).
  const float statSize = mfdFontPx(13.0f, displayH);
  const float statRight = x + w - mfdFontPx(8.0f, displayH);
  r.fillText(statRight, y + mfdFontPx(12.0f, displayH),
             ui.radarStab() ? "STAB On" : "STAB Off", statSize,
             TextAlign::Right, colors::kWhite);
  r.fillText(statRight, y + mfdFontPx(28.0f, displayH),
             ui.radarAct() ? "Altitude Comp Tilt On" : "Altitude Comp Tilt Off",
             statSize, TextAlign::Right, colors::kWhite);

  if (scan == RadarScan::Horizontal) {
    // ---- horizontal scan geometry ----
    const float cx = x + w * 0.5f;
    const float apexY = y + h - h * 0.05f;
    const float radius =
        std::min((apexY - y) - h * 0.06f, w * 0.66f);

    // Returns image, masked to the wedge / sector.
    if (haveReturns) {
      RadarImage& img = imageFor(r);
      const int iw = 512;
      const int ih = 256;
      const float secHalf = sectorHalfDeg(ui.radarSector());
      const bool stale =
          img.imageId < 0 || img.w != iw || img.h != ih ||
          img.revision != static_cast<int>(src->revision()) ||
          img.rangeNm != fullRange || img.mode != static_cast<int>(mode) ||
          img.scan != static_cast<int>(scan) ||
          img.sector != static_cast<int>(ui.radarSector()) ||
          img.bearing != ui.radarBearingDeg() ||
          img.calibrated != ui.radarGainCalibrated() ||
          img.gain != ui.radarGainManual();
      if (stale) {
        buildHorizontal(img.rgba, iw, ih, *src, fullRange, mode, secHalf,
                        ui.radarSector() == RadarSector::Full
                            ? 0.0f
                            : ui.radarBearingDeg(),
                        ui.radarGainCalibrated(), ui.radarGainManual());
        if (img.imageId >= 0 && (img.w != iw || img.h != ih)) {
          r.deleteImage(img.imageId);
          img.imageId = -1;
        }
        if (img.imageId < 0) {
          img.imageId = r.createImageRGBA(iw, ih, img.rgba.data());
        } else {
          r.updateImageRGBA(img.imageId, img.rgba.data());
        }
        img.w = iw;
        img.h = ih;
        img.revision = static_cast<int>(src->revision());
        img.rangeNm = fullRange;
        img.mode = static_cast<int>(mode);
        img.scan = static_cast<int>(scan);
        img.sector = static_cast<int>(ui.radarSector());
        img.bearing = ui.radarBearingDeg();
        img.calibrated = ui.radarGainCalibrated();
        img.gain = ui.radarGainManual();
      }
      if (img.imageId >= 0) {
        r.drawImage(img.imageId, cx - radius, apexY - radius, 2.0f * radius,
                    radius, 1.0f);
      }
    }

    // Range arcs at 1/4, 1/2, 3/4, full scale, dashed, labeled in NM.
    for (int i = 1; i <= 4; ++i) {
      const float rr = radius * static_cast<float>(i) / 4.0f;
      dashedArc(r, cx, apexY, rr, -45.0f, 45.0f, 1.0f, colors::kGroupBoxBorder);
      const float lblB = 38.0f * kDegToRad;
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%dnm",
                    static_cast<int>(std::lround(fullRange * i / 4.0f)));
      annunBox(r, cx + rr * std::sin(lblB), apexY - rr * std::cos(lblB), buf,
               mfdFontPx(12.0f, displayH), colors::kWhite);
    }

    // Wedge edges (+/-45 deg radials).
    for (float edge : {-45.0f, 45.0f}) {
      const float b = edge * kDegToRad;
      r.strokeLine(cx, apexY, cx + radius * std::sin(b),
                   apexY - radius * std::cos(b), 1.0f, colors::kGroupBoxBorder);
    }

    // Animated scan line (sawtooth sweep across the active sector).
    if (haveReturns) {
      const float secHalf = sectorHalfDeg(ui.radarSector());
      const float center =
          ui.radarSector() == RadarSector::Full ? 0.0f : ui.radarBearingDeg();
      const float sweepDeg = center - secHalf + ui.radarSweepPhase() * 2.0f *
                                                     secHalf;
      const float b = sweepDeg * kDegToRad;
      r.strokeLine(cx, apexY, cx + radius * std::sin(b),
                   apexY - radius * std::cos(b), 1.5f,
                   mfdAlpha(colors::kWhite, 0.85f));
    }

    // Bearing line (cyan) when displayed.
    if (ui.radarBearingLineOn()) {
      const float b = ui.radarBearingDeg() * kDegToRad;
      r.strokeLine(cx, apexY, cx + radius * std::sin(b),
                   apexY - radius * std::cos(b), 1.5f, colors::kCyan);
    }

    // Center banner for Standby (Table 6-12).
    if (mode == RadarMode::Standby) {
      r.fillText(cx, y + h * 0.45f, "STANDBY", mfdFontPx(28.0f, displayH),
                 TextAlign::Center, colors::kWhite);
    }
  } else {
    // ---- vertical scan geometry (range vs altitude slice) ----
    const float apexX = x + w * 0.10f;
    const float cy = y + h * 0.48f;
    const float length = w * 0.80f;
    const float vHalfDeg = 30.0f;  // 60 deg total vertical scan
    const float vHalf = length * std::tan(vHalfDeg * kDegToRad);

    if (haveReturns) {
      RadarImage& img = imageFor(r);
      const int iw = 256;
      const int ih = 256;
      const bool stale =
          img.imageId < 0 || img.w != iw || img.h != ih ||
          img.revision != static_cast<int>(src->revision()) ||
          img.rangeNm != fullRange || img.mode != static_cast<int>(mode) ||
          img.scan != static_cast<int>(scan) ||
          img.bearing != ui.radarBearingDeg() ||
          img.calibrated != ui.radarGainCalibrated() ||
          img.gain != ui.radarGainManual();
      if (stale) {
        buildVertical(img.rgba, iw, ih, *src, fullRange, mode,
                      ui.radarBearingDeg(), ui.radarGainCalibrated(),
                      ui.radarGainManual());
        if (img.imageId >= 0 && (img.w != iw || img.h != ih)) {
          r.deleteImage(img.imageId);
          img.imageId = -1;
        }
        if (img.imageId < 0) {
          img.imageId = r.createImageRGBA(iw, ih, img.rgba.data());
        } else {
          r.updateImageRGBA(img.imageId, img.rgba.data());
        }
        img.w = iw;
        img.h = ih;
        img.revision = static_cast<int>(src->revision());
        img.rangeNm = fullRange;
        img.mode = static_cast<int>(mode);
        img.scan = static_cast<int>(scan);
        img.bearing = ui.radarBearingDeg();
        img.calibrated = ui.radarGainCalibrated();
        img.gain = ui.radarGainManual();
      }
      if (img.imageId >= 0) {
        r.drawImage(img.imageId, apexX, cy - vHalf, length, 2.0f * vHalf, 1.0f);
      }
    }

    // Beam centerline and the +/- vertical wedge edges.
    r.strokeLine(apexX, cy, apexX + length, cy, 1.0f, colors::kPanelSeparator);
    r.strokeLine(apexX, cy, apexX + length, cy - vHalf, 1.0f,
                 colors::kGroupBoxBorder);
    r.strokeLine(apexX, cy, apexX + length, cy + vHalf, 1.0f,
                 colors::kGroupBoxBorder);

    // Range arcs (vertical dashed), labeled in NM.
    for (int i = 1; i <= 4; ++i) {
      const float rx = apexX + length * static_cast<float>(i) / 4.0f;
      const float vh = vHalf * static_cast<float>(i) / 4.0f;
      constexpr int kSegs = 16;
      bool on = true;
      for (int s = 0; s < kSegs; ++s) {
        const float t0 = -1.0f + 2.0f * s / kSegs;
        const float t1 = -1.0f + 2.0f * (s + 1) / kSegs;
        if (on) {
          r.strokeLine(rx, cy + vh * t0, rx, cy + vh * t1, 1.0f,
                       colors::kGroupBoxBorder);
        }
        on = !on;
      }
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%dnm",
                    static_cast<int>(std::lround(fullRange * i / 4.0f)));
      annunBox(r, rx, cy + vh + mfdFontPx(10.0f, displayH), buf,
               mfdFontPx(12.0f, displayH), colors::kWhite);
    }

    if (mode == RadarMode::Standby) {
      r.fillText(x + w * 0.5f, y + h * 0.45f, "STANDBY",
                 mfdFontPx(28.0f, displayH), TextAlign::Center, colors::kWhite);
    }
  }

  // ---- Scale legend + settings box (both scans) ----
  if (mode != RadarMode::Standby) {
    drawScaleLegend(r, x + mfdFontPx(10.0f, displayH), y + h * 0.18f,
                    w * 0.14f, h * 0.30f, mode, displayH);
  }
  drawSettingsBox(r, x + w - w * 0.26f - mfdFontPx(8.0f, displayH),
                  y + h - h * 0.30f - mfdFontPx(6.0f, displayH), w * 0.26f,
                  h * 0.30f, ui, displayH);
}

}  // namespace avionics::mfd
