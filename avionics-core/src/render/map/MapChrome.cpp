#include "render/map/MapViewInternal.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

#include "avionics/MapRange.h"

namespace avionics::mapview {

void drawNorthArrow(Renderer& r, float cx, float cy, float size,
                    float rotation) {
  // A white compass arrow with "N" near its head, below the orientation label
  // (Fig 5-26).
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotation);
  const float h = size;          // half height
  const float w = size * 0.45f;  // half width
  // Two mirrored halves so the arrow reads like the printed compass glyph.
  const Point left[3] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}};
  const Point right[3] = {{0.0f, -h}, {w, h}, {0.0f, h * 0.45f}};
  r.fillPolygon(left, 3, colors::kWhite);
  r.fillPolygon(right, 3, Color{0.62f, 0.62f, 0.62f, 1.0f});
  const Point outline[5] = {{0.0f, -h}, {-w, h}, {0.0f, h * 0.45f}, {w, h},
                            {0.0f, -h}};
  r.strokePolyline(outline, 5, 1.0f, Color{0.2f, 0.2f, 0.2f, 1.0f});
  r.fillText(0.0f, 0.0f, "N", size * 0.8f, TextAlign::Center,
             Color{0.1f, 0.1f, 0.1f, 1.0f});
  r.restore();
}

void drawMapPointer(Renderer& r, float hotX, float hotY, float displayH,
                    bool flashInverted) {
  // Working Title MapPointerLayer default SVG (viewBox 0 0 100 100).
  static constexpr float kHotSpotX = 4.54f;
  static constexpr float kHotSpotY = 4.54f;
  static constexpr float kPolyX[] = {78.93f, 49.48f, 41.18f, 4.54f,
                                     84.57f, 66.01f, 95.46f, 78.93f};
  static constexpr float kPolyY[] = {95.46f, 66.01f, 84.57f, 4.54f,
                                     41.18f, 49.48f, 78.93f, 95.46f};
  constexpr int kPointCount = 8;

  const float size = fontPx(kMapPointerSizeWt, displayH);
  const float scale = size / 100.0f;
  Point pts[kPointCount];
  for (int i = 0; i < kPointCount; ++i) {
    pts[i].x = hotX + (kPolyX[i] - kHotSpotX) * scale;
    pts[i].y = hotY + (kPolyY[i] - kHotSpotY) * scale;
  }

  const Color fill = flashInverted ? colors::kBlack : colors::kWhite;
  const Color stroke = flashInverted ? colors::kWhite : colors::kBlack;
  const float strokeW = std::max(1.0f, 4.0f * scale);

  r.fillPolygon(pts, kPointCount, fill);
  Point outline[kPointCount + 1];
  for (int i = 0; i < kPointCount; ++i) {
    outline[i] = pts[i];
  }
  outline[kPointCount] = pts[0];
  r.strokePolyline(outline, kPointCount + 1, strokeW, stroke);
}

void drawRangeLabel(Renderer& r, float x, float y, float rangeNm,
                    float labelSize, bool centerOnPoint) {
  // Cyan value with the smaller unit suffix on a chrome plate, centered on the
  // ring's upper-left (Fig 5-2 "15NM").
  char buf[16];
  formatMapRange(buf, sizeof(buf), rangeNm);
  // Split the value digits from the unit suffix so the unit renders smaller.
  std::size_t unitStart = 0;
  while (buf[unitStart] != '\0' &&
         ((buf[unitStart] >= '0' && buf[unitStart] <= '9') ||
          buf[unitStart] == '.')) {
    ++unitStart;
  }
  const std::string value(buf, unitStart);
  const std::string unit(buf + unitStart);
  const float unitSize = labelSize * 0.72f;
  const float padX = labelSize * 0.45f;
  const float textW = r.measureTextWidth(value, labelSize) +
                      r.measureTextWidth(unit, unitSize);
  const float w = textW + 2.0f * padX;
  const float h = labelSize * 1.5f;
  const float bx = centerOnPoint ? x - w * 0.5f : x - w;
  const float by = centerOnPoint ? y - h * 0.5f : y - h;
  drawChromeBox(r, bx, by, w, h);
  r.fillText(bx + padX, by + h * 0.52f, value, labelSize, TextAlign::Left,
             colors::kCyan);
  r.fillText(bx + padX + r.measureTextWidth(value, labelSize), by + h * 0.52f,
             unit, unitSize, TextAlign::Left, colors::kCyan);
}

void drawRelTerrainLegend(Renderer& r, const MapViewConfig& config,
                          float labelSize) {
  // Shown while TER REL is enabled (NXi Pilot's Guide, Hazard Avoidance). Two
  // color bands: red within 100 ft below ownship, yellow within 1000 ft.
  const float pad = labelSize * 0.45f;
  const float rowH = labelSize * 1.25f;
  const float chipW = labelSize * 1.5f;
  const float w = labelSize * 7.2f;
  const float h = rowH * 3.0f + pad;
  const float x = config.x + config.w * 0.02f;
  const float y = config.y + config.h - h - config.h * 0.02f;

  r.fillRect(x, y, w, h, Color{0.0f, 0.0f, 0.0f, 0.78f});
  r.strokeLine(x, y, x + w, y, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y + h, x + w, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x, y, x, y + h, 1.0f, colors::kPanelBorder);
  r.strokeLine(x + w, y, x + w, y + h, 1.0f, colors::kPanelBorder);

  r.fillText(x + w * 0.5f, y + pad + rowH * 0.35f, "TERRAIN",
             labelSize * 0.85f, TextAlign::Center, colors::kWhite);

  struct Row {
    Color chip;
    const char* label;
  };
  const Row rows[2] = {{colors::kBandRed, "-100 FT"},
                       {colors::kBandYellow, "-1000 FT"}};
  for (int i = 0; i < 2; ++i) {
    const float rowY = y + pad + rowH * (0.9f + static_cast<float>(i));
    r.fillRect(x + pad, rowY, chipW, rowH * 0.6f, rows[i].chip);
    r.fillText(x + pad + chipW + pad, rowY + rowH * 0.3f, rows[i].label,
               labelSize * 0.8f, TextAlign::Left, colors::kLabelText);
  }
}

void drawRangeRing(Renderer& r, float cx, float cy, float radiusPx,
                   const Color& c) {
  constexpr int kSeg = 48;
  Point ring[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a =
        static_cast<float>(i) / static_cast<float>(kSeg) * 2.0f * 3.14159265f;
    ring[i] = {cx + radiusPx * std::cos(a), cy + radiusPx * std::sin(a)};
  }
  r.strokePolyline(ring, kSeg + 1, 1.5f, c);
}

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Range-compass geometry, matching WT MapRangeCompassLayer.
constexpr float kArcHalfWidthDeg = 60.0f;
constexpr float kTickMajorIntervalDeg = 30.0f;
constexpr float kTickMinorIntervalDeg = 10.0f;

float diffAngleDeg(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

Point polarFromUp(float cx, float cy, float radius, float degFromUp) {
  const float rad = degFromUp * kPi / 180.0f;
  return {cx + radius * std::sin(rad), cy - radius * std::cos(rad)};
}

void strokeArcFromUp(Renderer& r, float cx, float cy, float radius,
                     float a0FromUp, float a1FromUp, float width,
                     const Color& c) {
  constexpr int kSeg = 32;
  Point pts[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a =
        a0FromUp + (a1FromUp - a0FromUp) * static_cast<float>(i) / kSeg;
    pts[i] = polarFromUp(cx, cy, radius, a);
  }
  r.strokePolyline(pts, kSeg + 1, width, c);
}

}  // namespace

void drawRangeCompass(Renderer& r, const MapViewConfig& config,
                      const FlightData& flight, float cx, float cy,
                      float radiusPx, float rotationDeg, float labelSize,
                      const Color& c) {
  const float majorLen = std::max(6.0f, labelSize * 0.55f);
  const float minorLen = majorLen * 0.5f;
  const float endLen = majorLen;
  const float strokeW = 1.5f;
  const float labelRadial = labelSize * 1.15f;
  const float labelFont = labelSize * 0.72f;

  strokeArcFromUp(r, cx, cy, radiusPx, -kArcHalfWidthDeg, kArcHalfWidthDeg,
                  strokeW, c);

  const Point leftInner = polarFromUp(cx, cy, radiusPx, -kArcHalfWidthDeg);
  const Point leftOuter =
      polarFromUp(cx, cy, radiusPx + endLen, -kArcHalfWidthDeg);
  const Point rightInner = polarFromUp(cx, cy, radiusPx, kArcHalfWidthDeg);
  const Point rightOuter =
      polarFromUp(cx, cy, radiusPx + endLen, kArcHalfWidthDeg);
  r.strokeLine(leftOuter.x, leftOuter.y, leftInner.x, leftInner.y, strokeW, c);
  r.strokeLine(rightInner.x, rightInner.y, rightOuter.x, rightOuter.y, strokeW,
               c);

  // Ownship track/heading reference at the top of the arc.
  const Point refInner = polarFromUp(cx, cy, radiusPx - minorLen, 0.0f);
  const Point refOuter = polarFromUp(cx, cy, radiusPx + endLen * 0.35f, 0.0f);
  r.strokeLine(refInner.x, refInner.y, refOuter.x, refOuter.y, strokeW, c);

  for (int bearing = 0; bearing < 360;
       bearing += static_cast<int>(kTickMinorIntervalDeg)) {
    const float screenDeg =
        diffAngleDeg(static_cast<float>(bearing), rotationDeg);
    if (std::fabs(screenDeg) > kArcHalfWidthDeg + 0.5f) continue;

    const bool isMajor =
        (bearing % static_cast<int>(kTickMajorIntervalDeg)) == 0;
    const float tickLen = isMajor ? majorLen : minorLen;
    const Point tInner =
        polarFromUp(cx, cy, radiusPx - tickLen, screenDeg);
    const Point tOuter = polarFromUp(cx, cy, radiusPx, screenDeg);
    r.strokeLine(tInner.x, tInner.y, tOuter.x, tOuter.y, strokeW, c);

    if (isMajor) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%03d", bearing);
      const Point lp =
          polarFromUp(cx, cy, radiusPx - labelRadial, screenDeg);
      r.fillText(lp.x, lp.y, buf, labelFont, TextAlign::Center, c);
    }
  }

  if (config.orientation == MapOrientation::HeadingUp && flight.headingValid) {
    const float bugDeg =
        diffAngleDeg(flight.selectedHeadingDeg, rotationDeg);
    if (std::fabs(bugDeg) <= kArcHalfWidthDeg) {
      const Point onArc = polarFromUp(cx, cy, radiusPx, bugDeg);
      r.strokeLine(cx, cy, onArc.x, onArc.y, strokeW, colors::kCyan);
      const float bugHalf = labelSize * 0.22f;
      const Point bug[3] = {{onArc.x, onArc.y - bugHalf},
                            {onArc.x - bugHalf * 0.65f, onArc.y + bugHalf * 0.4f},
                            {onArc.x + bugHalf * 0.65f, onArc.y + bugHalf * 0.4f}};
      r.fillPolygon(bug, 3, colors::kCyan);
    }
  }
}

namespace {

// VOR compass rose geometry (G1000 NXi map). The rose is a fixed geographic
// size around the station; radial ticks step every 10 degrees with longer
// ticks every 30, and abbreviated cardinal labels (value / 10) sit just inside
// the ring at the four quadrant radials when zoomed in enough to read them.
constexpr float kVorRoseRadiusNm = 2.5f;
constexpr float kVorRoseMinorTickDeg = 10.0f;
constexpr float kVorRoseMajorTickDeg = 30.0f;
constexpr float kVorRoseLabelDeg = 90.0f;
// Declutter the rose at the same range the VOR symbol itself drops off the map.
constexpr float kVorRoseMaxRangeNm = 100.0f;
// Below this on-screen radius the rose is too small to read, so only the VOR
// hexagon is drawn (matches the real unit dropping the rose at wider ranges).
constexpr float kVorRoseMinRadiusPx = 10.0f;
// The cardinal labels are only drawn once the rose is large enough to fit them.
constexpr float kVorRoseLabelMinRadiusPx = 52.0f;
// The radial labels are a fixed size (they do not scale with zoom) and sit a
// little smaller than the map fix/ident labels.
constexpr float kVorRoseLabelScale = 0.85f;

// One rose centered on a station's screen position. `magvarDeg` rotates the
// rose so the ticks read magnetic radials (true bearing = radial + variation),
// and `rotationDeg` accounts for the map orientation. `labelSize` is the map
// label font size, used so the radial numbers stay a fixed size when zooming.
void drawVorCompassRose(Renderer& r, float cx, float cy, float radiusPx,
                        float magvarDeg, float rotationDeg, float labelSize,
                        const Color& c) {
  const float strokeW = 1.5f;
  const float majorLen = std::max(3.0f, radiusPx * 0.13f);
  const float minorLen = majorLen * 0.5f;
  const bool drawLabels = radiusPx >= kVorRoseLabelMinRadiusPx;
  const float labelFont = labelSize * kVorRoseLabelScale;
  const float labelRadial = majorLen + labelFont * 1.05f;

  drawRangeRing(r, cx, cy, radiusPx, c);

  for (int bearing = 0; bearing < 360;
       bearing += static_cast<int>(kVorRoseMinorTickDeg)) {
    const float screenDeg = static_cast<float>(bearing) + magvarDeg - rotationDeg;
    const bool major = (bearing % static_cast<int>(kVorRoseMajorTickDeg)) == 0;
    const float tickLen = major ? majorLen : minorLen;
    const Point outer = polarFromUp(cx, cy, radiusPx, screenDeg);
    const Point inner = polarFromUp(cx, cy, radiusPx - tickLen, screenDeg);
    r.strokeLine(inner.x, inner.y, outer.x, outer.y, strokeW, c);

    if (drawLabels && bearing % static_cast<int>(kVorRoseLabelDeg) == 0) {
      char buf[4];
      std::snprintf(buf, sizeof(buf), "%d", bearing / 10);
      const Point lp = polarFromUp(cx, cy, radiusPx - labelRadial, screenDeg);
      r.fillText(lp.x, lp.y, buf, labelFont, TextAlign::Center, c);
    }
  }
}

}  // namespace

void drawVorRoses(Renderer& r, const MapData& map, const Proj& proj,
                  const MapViewConfig& config, float rangeNm,
                  float labelSize) {
  if (rangeNm > kVorRoseMaxRangeNm || !config.style.showNavaids) return;

  const float radiusPx = kVorRoseRadiusNm * proj.pixelsPerNm;
  if (radiusPx < kVorRoseMinRadiusPx) return;

  for (const MapFeature& f : map.features) {
    if (f.type != MapFeatureType::Vor) continue;

    float x = 0.0f;
    float y = 0.0f;
    proj.toPx(f.lat, f.lon, x, y);
    if (!proj.onScreen(x, y, radiusPx + kSymbologyClipMarginPx)) continue;

    // proj.rotation already encodes the map orientation (0 for north-up).
    const float magvar = f.hasMagvar ? f.magvarDeg : 0.0f;
    drawVorCompassRose(r, x, y, radiusPx, magvar, proj.rotation, labelSize,
                       colors::kCyan);
  }
}

}  // namespace avionics::mapview
