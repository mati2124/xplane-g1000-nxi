#include <algorithm>
#include <cmath>

#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {

void drawTurnRateIndicator(Renderer& r, float cx, float cy, float radius,
                           float turnRateDegPerSec) {
  // The turn-rate scale hugs the top of the compass ring. The G1000 maps a
  // standard-rate turn (3 deg/sec, an 18 deg heading change in 6 s) to the long
  // outer tick and half-standard (9 deg) to the short inner tick, on each side
  // of the lubber line.
  const float stdTick = radius * 0.10f;
  const float halfTick = radius * 0.055f;

  r.save();
  r.translate(cx, cy);
  auto tick = [&](float deg, float len) {
    r.save();
    r.rotateDegrees(deg);
    r.strokeLine(0.0f, -radius, 0.0f, -radius - len, 2.0f, colors::kWhite);
    r.restore();
  };
  tick(-18.0f, stdTick);
  tick(-9.0f, halfTick);
  tick(9.0f, halfTick);
  tick(18.0f, stdTick);

  // Magenta turn-rate trend vector: an arc on the ring from the lubber line to
  // the heading predicted in six seconds at the present turn rate, capped just
  // past standard rate. Beyond 4 deg/sec an arrowhead is shown and the
  // prediction is no longer valid.
  const float predicted = turnRateDegPerSec * 6.0f;
  const float capped = std::max(-24.0f, std::min(24.0f, predicted));
  if (std::fabs(capped) > 0.5f) {
    constexpr int kSeg = 24;
    Point arc[kSeg + 1];
    for (int i = 0; i <= kSeg; ++i) {
      float x, y;
      polarOffset(capped * (i / static_cast<float>(kSeg)), radius, x, y);
      arc[i] = {x, y};
    }
    const float lineW = std::max(3.0f, radius * 0.035f);
    r.strokePolyline(arc, kSeg + 1, lineW, colors::kMagenta);

    if (std::fabs(turnRateDegPerSec) > 4.0f) {
      const float dir = capped > 0.0f ? 1.0f : -1.0f;
      float tx, ty, bx, by;
      polarOffset(capped, radius, tx, ty);
      polarOffset(capped - dir * 6.0f, radius, bx, by);
      const float ux = tx - bx, uy = ty - by;
      const float ul = std::sqrt(ux * ux + uy * uy);
      const float nx = -uy / ul, ny = ux / ul;
      const float ah = radius * 0.05f;
      const Point head[3] = {{tx, ty},
                             {bx + nx * ah, by + ny * ah},
                             {bx - nx * ah, by - ny * ah}};
      r.fillPolygon(head, 3, colors::kMagenta);
    }
  }
  r.restore();
}

}  // namespace avionics::pfd
