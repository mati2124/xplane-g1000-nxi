#include <algorithm>

#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {

void drawCourseNeedle(Renderer& r, float radius, float courseDeg, float devDots,
                      bool toFlag, bool valid, bool doubleLine, const Color& c) {
  // NXi CDI needle: a thin arrow whose head sits just inside the compass ring,
  // a short fixed shaft and tail, four deviation dots at 32 px (~0.21 r)
  // spacing, and a moving deviation bar offset by the cross-track in dots. The
  // course pointer is a single-line arrow for GPS/VOR1/LOC1 and a double-line
  // arrow for VOR2/LOC2 (Pilot's Guide, HSI).
  r.save();
  r.rotateDegrees(courseDeg);

  const float tip = radius * 0.88f;
  const float headLen = radius * 0.12f;
  const float headHalf = radius * 0.055f;
  const float dotSpacing = radius * 0.209f;
  const float barHalf = radius * 0.36f;
  const float tailInner = radius * 0.50f;
  const float tailOuter = radius * 0.80f;
  const float shaftW = std::max(2.0f, radius * 0.014f);
  const float dotR = radius * 0.018f;

  const Point head[3] = {
      {0.0f, -tip}, {-headHalf, -tip + headLen}, {headHalf, -tip + headLen}};
  r.fillPolygon(head, 3, c);
  if (doubleLine) {
    // Two parallel rails for the shaft and tail give the VOR2/LOC2 pointer its
    // double-line look while sharing the single arrowhead and deviation bar.
    const float off = radius * 0.028f;
    for (int k = 0; k < 2; ++k) {
      const float x = (k == 0 ? -off : off);
      r.strokeLine(x, -tip + headLen, x, -barHalf, shaftW, c);
      r.strokeLine(x, tailInner, x, tailOuter, shaftW, c);
    }
  } else {
    r.strokeLine(0.0f, -tip + headLen, 0.0f, -barHalf, shaftW, c);
    r.strokeLine(0.0f, tailInner, 0.0f, tailOuter, shaftW, c);
  }

  for (int i = 1; i <= 2; ++i) {
    const float dx = static_cast<float>(i) * dotSpacing;
    r.fillCircle(-dx, 0.0f, dotR, colors::kWhite);
    r.fillCircle(dx, 0.0f, dotR, colors::kWhite);
  }

  if (valid) {
    const float off = std::max(-2.0f, std::min(2.0f, devDots)) * dotSpacing;
    r.strokeLine(off, -barHalf, off, barHalf, shaftW, c);

    const float ty = radius * 0.46f;
    const float th = radius * 0.075f;
    if (toFlag) {
      const Point to[3] = {{0.0f, -ty - th}, {-th, -ty}, {th, -ty}};
      r.fillPolygon(to, 3, c);
    } else {
      const Point fr[3] = {{0.0f, ty + th}, {-th, ty}, {th, ty}};
      r.fillPolygon(fr, 3, c);
    }
  }
  r.restore();
}

}  // namespace avionics::pfd
