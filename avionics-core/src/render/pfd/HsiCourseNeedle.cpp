#include <algorithm>
#include <cmath>

#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {
namespace {

// Hollow white reference ring. The NXi CDI deviation dots are stroked, unfilled
// circles (open in the center), matching the VDI reference dots -- not solid.
void strokeRing(Renderer& r, float cx, float cy, float radius, float lineW,
                const Color& c) {
  constexpr int kSeg = 18;
  Point pts[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a = 6.2831853f * (static_cast<float>(i) / static_cast<float>(kSeg));
    pts[i] = {cx + std::cos(a) * radius, cy + std::sin(a) * radius};
  }
  r.strokePolyline(pts, kSeg + 1, lineW, c);
}

}  // namespace

// Grows a triangle outward from its centroid by `amt` px so a black copy drawn
// beneath the colored fill leaves a uniform rim of that width on every edge.
void expandTriangle(const Point in[3], Point out[3], float amt) {
  const float cx = (in[0].x + in[1].x + in[2].x) / 3.0f;
  const float cy = (in[0].y + in[1].y + in[2].y) / 3.0f;
  for (int i = 0; i < 3; ++i) {
    const float dx = in[i].x - cx;
    const float dy = in[i].y - cy;
    const float len = std::sqrt(dx * dx + dy * dy);
    out[i] = (len > 1e-3f)
                 ? Point{in[i].x + dx / len * amt, in[i].y + dy / len * amt}
                 : in[i];
  }
}

void drawCourseNeedle(Renderer& r, float radius, float courseDeg, float devDots,
                      bool toFlag, bool valid, bool doubleLine, const Color& c) {
  // NXi CDI needle: an arrow whose head sits just inside the compass ring, a
  // short fixed shaft and tail, four deviation dots at 32 px (~0.21 r) spacing,
  // and a moving deviation bar offset by the cross-track in dots. The course
  // pointer is a single-line arrow for GPS/VOR1/LOC1 and a double-line arrow
  // for VOR2/LOC2 (Pilot's Guide, HSI). The whole magenta/green pointer is
  // rimmed with a thin black outline so it reads against the moving map and
  // rose, matching the trainer course pointer.
  r.save();
  r.rotateDegrees(courseDeg);

  const float tip = radius * 0.88f;
  const float headLen = radius * 0.13f;
  const float headHalf = radius * 0.062f;
  const float dotSpacing = radius * 0.209f;
  const float barHalf = radius * 0.36f;
  // The fixed course pointer (shaft above, tail below) is separated from the
  // central deviation bar by a symmetric gap on both sides, as on the real NXi.
  // Both the shaft and tail start at bodyInner -- previously only the tail did,
  // while the shaft ran straight into the bar, making the top look solid and
  // the bottom look gapped.
  const float bodyInner = radius * 0.50f;
  const float tailOuter = radius * 0.80f;
  const float shaftW = std::max(2.5f, radius * 0.021f);
  const float dotR = radius * 0.020f;
  const float dotW = std::max(1.0f, radius * 0.009f);
  const float borderW = std::max(1.0f, radius * 0.0075f);

  const float devOff =
      valid ? std::max(-2.0f, std::min(2.0f, devDots)) * dotSpacing : 0.0f;
  const float doubleOff = radius * 0.028f;
  const float ty = radius * 0.46f;
  const float th = radius * 0.075f;

  const Point head[3] = {
      {0.0f, -tip}, {-headHalf, -tip + headLen}, {headHalf, -tip + headLen}};

  // Draws the pointer (arrowhead, shaft/tail rails, deviation bar and TO/FROM
  // flag) in a single color and line width -- called first for the black rim,
  // then again for the colored fill on top.
  const auto drawPointer = [&](const Color& col, float lineW, bool withHead,
                               bool withFlag) {
    if (withHead) r.fillPolygon(head, 3, col);
    if (doubleLine) {
      for (int k = 0; k < 2; ++k) {
        const float x = (k == 0 ? -doubleOff : doubleOff);
        r.strokeLine(x, -tip + headLen, x, -bodyInner, lineW, col);
        r.strokeLine(x, bodyInner, x, tailOuter, lineW, col);
      }
    } else {
      r.strokeLine(0.0f, -tip + headLen, 0.0f, -bodyInner, lineW, col);
      r.strokeLine(0.0f, bodyInner, 0.0f, tailOuter, lineW, col);
    }
    if (valid) {
      r.strokeLine(devOff, -barHalf, devOff, barHalf, lineW, col);
      if (withFlag) {
        if (toFlag) {
          const Point to[3] = {{0.0f, -ty - th}, {-th, -ty}, {th, -ty}};
          r.fillPolygon(to, 3, col);
        } else {
          const Point fr[3] = {{0.0f, ty + th}, {-th, ty}, {th, ty}};
          r.fillPolygon(fr, 3, col);
        }
      }
    }
  };

  // Black underlay: stroked elements widen by 2*borderW (a rim either side) and
  // the filled arrowhead / TO-FROM flag use centroid-expanded copies.
  {
    Point headB[3];
    expandTriangle(head, headB, borderW);
    r.fillPolygon(headB, 3, colors::kBlack);
    if (doubleLine) {
      for (int k = 0; k < 2; ++k) {
        const float x = (k == 0 ? -doubleOff : doubleOff);
        r.strokeLine(x, -tip + headLen, x, -bodyInner, shaftW + 2.0f * borderW,
                     colors::kBlack);
        r.strokeLine(x, bodyInner, x, tailOuter, shaftW + 2.0f * borderW,
                     colors::kBlack);
      }
    } else {
      r.strokeLine(0.0f, -tip + headLen, 0.0f, -bodyInner,
                   shaftW + 2.0f * borderW, colors::kBlack);
      r.strokeLine(0.0f, bodyInner, 0.0f, tailOuter, shaftW + 2.0f * borderW,
                   colors::kBlack);
    }
    if (valid) {
      r.strokeLine(devOff, -barHalf, devOff, barHalf, shaftW + 2.0f * borderW,
                   colors::kBlack);
      if (toFlag) {
        const Point to[3] = {{0.0f, -ty - th}, {-th, -ty}, {th, -ty}};
        Point toB[3];
        expandTriangle(to, toB, borderW);
        r.fillPolygon(toB, 3, colors::kBlack);
      } else {
        const Point fr[3] = {{0.0f, ty + th}, {-th, ty}, {th, ty}};
        Point frB[3];
        expandTriangle(fr, frB, borderW);
        r.fillPolygon(frB, 3, colors::kBlack);
      }
    }
  }

  // Reference deviation dots (white, hollow) sit beneath the magenta bar.
  for (int i = 1; i <= 2; ++i) {
    const float dx = static_cast<float>(i) * dotSpacing;
    strokeRing(r, -dx, 0.0f, dotR, dotW, colors::kWhite);
    strokeRing(r, dx, 0.0f, dotR, dotW, colors::kWhite);
  }

  drawPointer(c, shaftW, true, true);
  r.restore();
}

}  // namespace avionics::pfd
