#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace avionics::mapview {
namespace {

constexpr float kObstacleRedBelowFt = 100.0f;
constexpr float kObstacleYellowBelowFt = 1000.0f;
constexpr float kPi = 3.14159265358979323846f;

Color obstacleColor(float mslFt, float ownAltFt, bool altValid) {
  if (!altValid) return colors::kWhite;
  const float rel = mslFt - ownAltFt;
  if (rel >= -kObstacleRedBelowFt) return colors::kBandRed;
  if (rel >= -kObstacleYellowBelowFt) return colors::kBandYellow;
  return colors::kWhite;
}

float towerStroke(float size) {
  return std::max(1.55f, size * 0.072f);
}

float sparkStroke(float size) {
  return std::max(1.65f, size * 0.082f);
}

// Six-ray lighted spark (PC Trainer): 60° spacing, longer upper ray, tip dots.
void drawLightedSpark(Renderer& r, float x, float y, float arm, float size,
                      const Color& c) {
  const float stroke = sparkStroke(size);
  const float tipR = stroke * 0.54f;
  const float hubR = stroke * 0.38f;
  for (int i = 0; i < 6; ++i) {
    const float deg = -90.0f + static_cast<float>(i) * 60.0f;
    const float rad = deg * (kPi / 180.0f);
    const float scale = (i == 0) ? 1.20f : (i == 3) ? 0.82f : 1.0f;
    const float ray = arm * scale;
    const float ex = x + ray * std::cos(rad);
    const float ey = y + ray * std::sin(rad);
    r.strokeLine(x, y, ex, ey, stroke, c);
    r.fillCircle(ex, ey, tipR, c);
  }
  r.fillCircle(x, y, hubR, c);
}

struct TowerShape {
  float triHeight;
  float halfWidth;
};

TowerShape towerShape(float s, float aglFt, bool isPole) {
  if (isPole) {
    return {s * 0.20f, s * 0.21f};
  }
  if (aglFt < 150.0f) {
    return {s * 0.24f, s * 0.26f};
  }
  if (aglFt < 500.0f) {
    return {s * 0.30f, s * 0.30f};
  }
  return {s * 0.36f, s * 0.34f};
}

float towerDrawRadius(float s, bool lighted, float triHeight, bool grouped) {
  const float body = triHeight + s * 0.04f;
  const float spark = lighted ? s * 0.34f : 0.0f;
  const float width = grouped ? s * 0.74f : s * 0.32f;
  return std::max(body + spark, width);
}

// Tower / pole: open upward V, apex dot, six-ray spark when lighted.
void drawTowerObstacle(Renderer& r, float x, float y, float s, const Color& c,
                       bool lighted, float aglFt, bool isPole) {
  if (lighted && aglFt < 80.0f && !isPole) {
    drawLightedSpark(r, x, y - s * 0.04f, s * 0.30f, s, c);
    return;
  }

  const TowerShape shape = towerShape(s, aglFt, isPole);
  const float stroke = towerStroke(s);
  const float baseY = y + s * 0.02f;
  const float apexY = baseY - shape.triHeight;

  r.strokeLine(x - shape.halfWidth, baseY, x, apexY, stroke, c);
  r.strokeLine(x + shape.halfWidth, baseY, x, apexY, stroke, c);
  r.fillCircle(x, apexY, stroke * 0.72f, c);

  if (lighted) {
    drawLightedSpark(r, x, apexY - s * 0.07f, s * 0.26f, s, c);
  }
}

void drawWindTurbineObstacle(Renderer& r, float x, float y, float s,
                             const Color& c, bool lighted) {
  const float stroke = towerStroke(s);
  const float baseY = y + s * 0.02f;
  const float hubY = baseY - s * 0.34f;
  const float blade = s * 0.24f;

  r.strokeLine(x, baseY, x, hubY + s * 0.05f, stroke, c);
  for (int i = 0; i < 3; ++i) {
    const float deg = -90.0f + static_cast<float>(i) * 120.0f;
    const float rad = deg * (kPi / 180.0f);
    r.strokeLine(x, hubY, x + blade * std::cos(rad), hubY + blade * std::sin(rad),
                 stroke, c);
  }

  if (lighted) {
    drawLightedSpark(r, x, hubY - s * 0.11f, s * 0.26f, s, c);
  }
}

void drawSingleObstacle(Renderer& r, float x, float y, float s, const Color& c,
                        const MapObstacle& ob) {
  if (ob.windTurbine) {
    drawWindTurbineObstacle(r, x, y, s, c, ob.lighted);
    return;
  }
  drawTowerObstacle(r, x, y, s, c, ob.lighted, ob.aglFt, ob.isPole);
}

}  // namespace

void drawObstacleSelectedTag(Renderer& r, float x, float y, float symSize,
                             float mslFt, float aglFt, float labelSize,
                             const Color& c) {
  const float lineSize = labelSize * 0.72f;
  char line1[24];
  char line2[24];
  std::snprintf(line1, sizeof(line1), "%dFT MSL",
                static_cast<int>(std::lround(mslFt)));
  std::snprintf(line2, sizeof(line2), "%dFT AGL",
                static_cast<int>(std::lround(aglFt)));

  const float w1 = r.measureTextWidth(line1, lineSize);
  const float w2 = r.measureTextWidth(line2, lineSize);
  const float boxW = std::max(w1, w2) + lineSize * 0.70f;
  const float rowH = lineSize * 1.18f;
  const float boxH = rowH * 2.0f + lineSize * 0.20f;
  const float boxX = x - boxW * 0.5f;
  const float boxY = y + symSize * 0.12f;

  r.fillRect(boxX, boxY, boxW, boxH, Color{0.0f, 0.0f, 0.0f, 0.88f});
  r.strokeLine(boxX, boxY, boxX + boxW, boxY, 1.0f, colors::kWhite);
  r.strokeLine(boxX, boxY + boxH, boxX + boxW, boxY + boxH, 1.0f,
               colors::kWhite);
  r.strokeLine(boxX, boxY, boxX, boxY + boxH, 1.0f, colors::kWhite);
  r.strokeLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.0f,
               colors::kWhite);

  r.fillText(x, boxY + rowH * 0.55f, line1, lineSize, TextAlign::Center, c);
  r.fillText(x, boxY + rowH * 1.55f, line2, lineSize, TextAlign::Center, c);
}

void drawObstacles(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, float maxRangeNm, float ownAltFt,
                   bool altValid, float symSize) {
  if (rangeNm > maxRangeNm) return;
  for (const MapObstacle& ob : map.obstacles) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(ob.lat, ob.lon, x, y);
    const float s = symSize;
    const TowerShape shape = towerShape(s, ob.aglFt, ob.isPole);
    const bool grouped = ob.quantity > 1 && !ob.windTurbine;
    if (!proj.onScreen(x, y, towerDrawRadius(s, ob.lighted, shape.triHeight, grouped))) {
      continue;
    }

    const Color c = obstacleColor(ob.mslFt, ownAltFt, altValid);
    if (grouped) {
      const float gs = s * 0.84f;
      const float gap = s * 0.16f;
      MapObstacle single = ob;
      single.quantity = 1;
      drawSingleObstacle(r, x - gap, y, gs, c, single);
      drawSingleObstacle(r, x + gap, y, gs, c, single);
    } else {
      drawSingleObstacle(r, x, y, s, c, ob);
    }
  }
}

}  // namespace avionics::mapview
