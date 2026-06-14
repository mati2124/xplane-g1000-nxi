#include "render/map/MapViewInternal.h"

#include <cmath>
#include <cstdio>

namespace avionics::mapview {

void drawWindVector(Renderer& r, const FlightData& flight,
                    const MapViewConfig& config, float rotation,
                    float labelSize) {
  // A chrome plate in the upper right holding a white arrow that points where
  // the wind blows toward (rotated with the map) and the speed with a small KT
  // suffix (Fig 5-18).
  if (!flight.windValid || flight.windSpeedKts < 1.0f) return;
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%d",
                static_cast<int>(std::lround(flight.windSpeedKts)));
  const float speedSize = labelSize * 1.25f;
  const float unitSize = speedSize * 0.72f;
  const float arrowLen = labelSize * 1.5f;
  const float padX = labelSize * 0.5f;
  const float w = arrowLen + r.measureTextWidth(buf, speedSize) +
                  r.measureTextWidth("KT", unitSize) + 2.6f * padX;
  const float h = labelSize * 2.1f;
  const float bx = config.x + config.w - w - labelSize * 0.6f;
  const float by = config.y + labelSize * 2.4f;
  drawChromeBox(r, bx, by, w, h);

  // Arrow in the left cell, pointing where the wind blows toward.
  const float cx = bx + padX + arrowLen * 0.5f;
  const float cy = by + h * 0.5f;
  const float len = arrowLen;
  const float angleRad =
      (flight.windDirectionDeg + 180.0f - rotation) * 3.14159265f / 180.0f;
  const float dx = std::sin(angleRad);
  const float dy = -std::cos(angleRad);
  const float ax = cx - dx * len * 0.5f;
  const float ay = cy - dy * len * 0.5f;
  const float ex = cx + dx * len * 0.5f;
  const float ey = cy + dy * len * 0.5f;
  r.strokeLine(ax, ay, ex, ey, 2.5f, colors::kWhite);
  const float hx = -dy, hy = dx;
  const Point head[3] = {
      {ex, ey},
      {ex - dx * len * 0.42f + hx * len * 0.26f,
       ey - dy * len * 0.42f + hy * len * 0.26f},
      {ex - dx * len * 0.42f - hx * len * 0.26f,
       ey - dy * len * 0.42f - hy * len * 0.26f}};
  r.fillPolygon(head, 3, colors::kWhite);

  const float tx = bx + padX * 1.6f + arrowLen;
  r.fillText(tx, cy, buf, speedSize, TextAlign::Left, colors::kWhite);
  r.fillText(tx + r.measureTextWidth(buf, speedSize), cy, "KT", unitSize,
             TextAlign::Left, colors::kWhite);
}

}  // namespace avionics::mapview
