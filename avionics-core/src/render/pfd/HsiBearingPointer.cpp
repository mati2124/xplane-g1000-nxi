#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {

void drawBearingPointer(Renderer& r, float radius, float bearingDeg, bool dbl) {
  // NXi bearing pointers are thin (2 px) cyan needles: an arrowhead just inside
  // the ring with a short upper shaft, and a separate tail near the bottom of
  // the ring, leaving the center clear. The double-bar needle (BRG2) doubles
  // the shaft/tail and uses an open (chevron) arrowhead.
  r.save();
  r.rotateDegrees(bearingDeg);
  const Color c = colors::kCyan;
  const float headTip = radius * 0.84f;
  const float headLen = radius * 0.14f;
  const float headHalf = radius * 0.085f;
  const float shoulder = headTip - headLen;
  const float upperInner = radius * 0.50f;
  const float tailInner = radius * 0.50f;
  const float tailOuter = radius * 0.78f;
  const float w = 2.0f * (radius / 153.0f);

  if (!dbl) {
    r.strokeLine(0.0f, -shoulder, 0.0f, -upperInner, w, c);
    r.strokeLine(0.0f, tailInner, 0.0f, tailOuter, w, c);
    const Point head[3] = {
        {0.0f, -headTip}, {-headHalf, -shoulder}, {headHalf, -shoulder}};
    r.fillPolygon(head, 3, c);
  } else {
    const float off = radius * 0.035f;
    for (int k = 0; k < 2; ++k) {
      const float x = (k == 0 ? -off : off);
      r.strokeLine(x, -shoulder, x, -upperInner, w, c);
      r.strokeLine(x, tailInner, x, tailOuter, w, c);
    }
    // Open chevron arrowhead.
    r.strokeLine(0.0f, -headTip, -headHalf, -shoulder, w, c);
    r.strokeLine(0.0f, -headTip, headHalf, -shoulder, w, c);
  }
  r.restore();
}

}  // namespace avionics::pfd
