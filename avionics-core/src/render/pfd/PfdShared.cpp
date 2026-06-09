#include "render/pfd/PfdInternal.h"

#include <algorithm>

namespace avionics::pfd {

const VSpeedRef kVSpeedRefs[] = {
    {"R", 55.0f},
    {"X", 62.0f},
    {"Y", 74.0f},
    {"G", 65.0f},
};
const int kVSpeedRefCount = 4;

Layout computeLayout(float w, float h) {
  Layout L;
  L.sx = w / kWtCanvasWidth;
  L.sy = h / kWtCanvasHeightPx;
  L.s = L.sy;
  const auto X = [&](float px) { return px * L.sx; };
  const auto Y = [&](float px) { return px * L.sy; };

  // Top NavComBox 56 px; bottom info panel 55 px at y=679; softkey bar 35 px.
  L.topBarH = Y(56.0f);
  L.infoPanelTop = Y(679.0f);
  L.infoPanelH = Y(55.0f);
  L.bottomBarH = Y(kWtCanvasHeightPx - 733.0f);

  // Airspeed indicator x=154 w=87 y=82 h=390; tape (scroll) area y=113 h=330.
  L.asiX = X(154.0f);
  L.asiW = X(87.0f);
  L.asiTop = Y(82.0f);
  L.asiH = Y(390.0f);
  // Altimeter x=700 w=108 y=82 h=392.
  L.altX = X(700.0f);
  L.altW = X(108.0f);
  L.altTop = Y(82.0f);
  L.altH = Y(392.0f);
  // VSI x=808 w=48 y=132 h=296.
  L.vsiX = X(808.0f);
  L.vsiW = X(48.0f);
  L.vsiTop = Y(132.0f);
  L.vsiH = Y(296.0f);

  // Both tapes share a scroll strip from y=113 (top box) to y=443, centered on
  // the aircraft reference / horizon pivot at y=278.
  L.stripTop = Y(113.0f);
  L.stripH = Y(330.0f);

  // Attitude window: container x=253 w=414 y=73 h=315, pivot (460, 277).
  L.attTop = Y(73.0f);
  L.attBottom = Y(388.0f);
  L.attRegionH = L.attBottom - L.attTop;
  L.attCx = X(460.0f);
  L.attCy = L.stripTop + L.stripH * 0.5f;
  L.attVisW = X(414.0f);
  L.rollRadius = 193.0f * L.s;

  // HSI rose center (461, 571), tick-ring radius 153.
  L.hsiCx = X(461.0f);
  L.hsiCy = Y(571.0f);
  L.hsiRadius = 153.0f * L.s;

  // CAS annunciation window. Per the G1000 Pilot's Guide (Fig. A-1) it sits to
  // the right of the VSI tape, with a fixed top at ~44% of display height (just
  // below the horizon / mid-altimeter) and grows downward toward the info panel
  // -- sized to hold ~14 messages. It is NOT below the altimeter.
  L.casAnnunTop = Y(338.0f);
  L.casAnnunLeft = L.vsiX + L.vsiW + X(28.0f);  // gap to the right of the VSI
  L.casAnnunW = X(1018.0f) - L.casAnnunLeft;
  return L;
}

std::string formatInt(float value) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(value)));
  return std::string(buf);
}

std::string formatHeading(float headingDeg) {
  int hdg = static_cast<int>(std::lround(std::fmod(headingDeg + 360.0f, 360.0f)));
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%03d", hdg);
  return std::string(buf);
}

const char* roseLabel(int deg) {
  switch (deg) {
    case 0:   return "N";
    case 30:  return "3";
    case 60:  return "6";
    case 90:  return "E";
    case 120: return "12";
    case 150: return "15";
    case 180: return "S";
    case 210: return "21";
    case 240: return "24";
    case 270: return "W";
    case 300: return "30";
    case 330: return "33";
    default:  return "";
  }
}

std::string formatFreq(float mhz, int decimals) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, mhz);
  return std::string(buf);
}

std::string formatHms(int hour, int minute, int second) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hour % 24, minute % 60,
                second % 60);
  return std::string(buf);
}

std::string formatTimer(int totalSeconds) {
  const int h = totalSeconds / 3600;
  const int m = (totalSeconds % 3600) / 60;
  const int s = totalSeconds % 60;
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%d:%02d:%02d", h, m, s);
  return std::string(buf);
}

std::string formatOat(float celsius) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.1f", celsius);
  return std::string(buf);
}

void polarOffset(float angleDeg, float radius, float& x, float& y) {
  const float rad = angleDeg * static_cast<float>(M_PI) / 180.0f;
  x = std::sin(rad) * radius;
  y = -std::cos(rad) * radius;
}

void drawReadoutBox(Renderer& r, float x, float y, float w, float h,
                    const std::string& text, float textSize, NotchSide notch,
                    const Color& boxColor, const Color& textColor) {
  const float midY = y + h * 0.5f;
  const float notchHalfH = h * 0.22f;
  const float notchDepth = h * 0.14f;

  r.fillRect(x, y, w, h, boxColor);

  if (notch == NotchSide::Right) {
    const Point tri[3] = {{x + w, midY - notchHalfH},
                          {x + w + notchDepth, midY},
                          {x + w, midY + notchHalfH}};
    r.fillPolygon(tri, 3, boxColor);
    const Point outline[8] = {
        {x, y},                     {x + w, y},
        {x + w, midY - notchHalfH}, {x + w + notchDepth, midY},
        {x + w, midY + notchHalfH}, {x + w, y + h},
        {x, y + h},                 {x, y}};
    r.strokePolyline(outline, 8, 2.0f, colors::kWhite);
  } else if (notch == NotchSide::Left) {
    const Point tri[3] = {{x, midY - notchHalfH},
                          {x - notchDepth, midY},
                          {x, midY + notchHalfH}};
    r.fillPolygon(tri, 3, boxColor);
    const Point outline[8] = {
        {x, y},                 {x + w, y},
        {x + w, y + h},         {x, y + h},
        {x, midY + notchHalfH}, {x - notchDepth, midY},
        {x, midY - notchHalfH}, {x, y}};
    r.strokePolyline(outline, 8, 2.0f, colors::kWhite);
  } else {
    const Point outline[5] = {
        {x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}};
    r.strokePolyline(outline, 5, 2.0f, colors::kWhite);
  }

  r.fillText(x + w * 0.5f, midY, text, textSize, TextAlign::Center, textColor);
}

void drawTapeBackground(Renderer& r, float x, float y, float w, float h,
                        const Color& edge) {
  // Working Title NXi tape background: a vertical gradient that is translucent
  // black at the top and bottom edges and fully clear through the middle, so
  // the scale dims out toward the center where the readout box sits.
  const Color clear{edge.r, edge.g, edge.b, 0.0f};
  const float half = h * 0.5f;
  r.fillRectVerticalGradient(x, y, w, half, y, y + half, edge, clear);
  r.fillRectVerticalGradient(x, y + half, w, half, y + half, y + h, clear, edge);
}

void drawVerticalTape(Renderer& r, float tapeX, float tapeW, float stripTop,
                      float stripH, float cy, float displayH, float value,
                      float viewableUnits, float majorInterval,
                      float minorInterval, float minValue, bool tapeOnRight) {
  const float pixelsPerUnit = stripH / viewableUnits;
  const float minorLen = tapeW * kTapeMinorTickFraction;
  const float majorLen = tapeW * kTapeMajorTickFraction;
  const float labelSize = fontPx(wt::kTapeLabel, displayH);
  const float labelPad = displayH * 0.006f;
  const long majorEveryN = std::lround(majorInterval / minorInterval);

  r.save();
  r.clip(tapeX, stripTop, tapeW, stripH);
  drawTapeBackground(r, tapeX, stripTop, tapeW, stripH, colors::kTapeEdge);

  // Inner-edge border: a vertical gradient from #646464 (top) to #2c2c2c
  // (bottom), per the NXi tape window border-image.
  const float borderX = tapeOnRight ? tapeX : tapeX + tapeW;
  const float midY = stripTop + stripH * 0.5f;
  r.strokeLine(borderX, stripTop, borderX, midY, 1.5f, colors::kTapeTopBorder);
  r.strokeLine(borderX, midY, borderX, stripTop + stripH, 1.5f,
               colors::kTapeBottomBorder);

  const float halfRange = (stripH * 0.5f) / pixelsPerUnit + majorInterval;
  const long startTick =
      static_cast<long>(std::floor((value - halfRange) / minorInterval));
  const long endTick =
      static_cast<long>(std::ceil((value + halfRange) / minorInterval));

  const float innerX = tapeOnRight ? tapeX : tapeX + tapeW;

  for (long i = startTick; i <= endTick; ++i) {
    const float s = static_cast<float>(i) * minorInterval;
    if (s < minValue) continue;

    const float y = cy - (s - value) * pixelsPerUnit;
    const bool major = (i % majorEveryN) == 0;
    const float len = major ? majorLen : minorLen;
    const float lineWidth = major ? 2.5f : 1.5f;

    if (tapeOnRight) {
      r.strokeLine(innerX, y, innerX + len, y, lineWidth, colors::kWhite);
      if (major) {
        r.fillText(innerX + majorLen + labelPad, y, formatInt(s), labelSize,
                   TextAlign::Left, colors::kWhite);
      }
    } else {
      r.strokeLine(innerX - len, y, innerX, y, lineWidth, colors::kWhite);
      if (major) {
        r.fillText(innerX - majorLen - labelPad, y, formatInt(s), labelSize,
                   TextAlign::Right, colors::kWhite);
      }
    }
  }
  r.restore();
}

void drawTrendVector(Renderer& r, float edgeX, float stripTop, float stripH,
                     float cy, float displayH, float ppu, float trend) {
  if (std::fabs(trend * ppu) < displayH * 0.006f) return;
  float endY = cy - trend * ppu;
  endY = std::max(stripTop, std::min(stripTop + stripH, endY));
  const float lineW = std::max(2.0f, displayH * 0.0045f);
  const float ah = displayH * 0.010f;

  r.save();
  r.clip(edgeX - ah * 1.5f, stripTop, ah * 3.0f, stripH);
  r.strokeLine(edgeX, cy, edgeX, endY, lineW, colors::kMagenta);
  const float dir = (endY < cy) ? -1.0f : 1.0f;
  const Point tip[3] = {{edgeX, endY},
                        {edgeX - ah, endY + dir * ah},
                        {edgeX + ah, endY + dir * ah}};
  r.fillPolygon(tip, 3, colors::kMagenta);
  r.restore();
}

float putText(Renderer& r, float x, float y, const std::string& s, float size,
              const Color& c, float trailingGapFrac) {
  r.fillText(x, y, s, size, TextAlign::Left, c);
  return x + r.measureTextWidth(s, size) + size * trailingGapFrac;
}

void drawFailureX(Renderer& r, float x, float y, float w, float h,
                  const std::string& label, float displayH) {
  r.save();
  r.clip(x, y, w, h);
  // Failed instruments go black with a red X, matching real EFIS reversionary
  // behavior (better to show no data than misleading data).
  r.fillRect(x, y, w, h, colors::kBlack);
  const float thick = std::max(3.0f, displayH * 0.012f);
  r.strokeLine(x, y, x + w, y + h, thick, colors::kBandRed);
  r.strokeLine(x, y + h, x + w, y, thick, colors::kBandRed);

  if (!label.empty()) {
    const float size = fontPx(wt::kHeadingBox, displayH);
    const float tw = r.measureTextWidth(label, size);
    const float pad = size * 0.3f;
    const float cx = x + w * 0.5f;
    const float cy = y + h * 0.5f;
    // Small black plate so the amber label stays legible over the X.
    r.fillRect(cx - tw * 0.5f - pad, cy - size * 0.7f, tw + 2.0f * pad,
               size * 1.4f, colors::kBlack);
    r.fillText(cx, cy, label, size, TextAlign::Center, colors::kBandYellow);
  }
  r.restore();
}

void drawArcBand(Renderer& r, float cx, float cy, float innerR, float outerR,
                 float startDeg, float endDeg, const Color& c) {
  constexpr int kSeg = 40;
  Point pts[(kSeg + 1) * 2];
  int n = 0;
  for (int i = 0; i <= kSeg; ++i) {
    const float t = startDeg + (endDeg - startDeg) * (i / float(kSeg));
    float x, y;
    polarOffset(t, outerR, x, y);
    pts[n++] = {cx + x, cy + y};
  }
  for (int i = kSeg; i >= 0; --i) {
    const float t = startDeg + (endDeg - startDeg) * (i / float(kSeg));
    float x, y;
    polarOffset(t, innerR, x, y);
    pts[n++] = {cx + x, cy + y};
  }
  r.fillPolygon(pts, n, c);
}

}  // namespace avionics::pfd
