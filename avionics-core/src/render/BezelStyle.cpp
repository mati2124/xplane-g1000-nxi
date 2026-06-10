#include "render/BezelStyle.h"

#include <algorithm>
#include <cmath>

namespace avionics::bezel {
namespace {

// Key cap colors: charcoal plastic, near-black outline, faint top sheen.
constexpr Color kCapBase{0.165f, 0.17f, 0.185f, 1.0f};
constexpr Color kCapPressed{0.32f, 0.33f, 0.36f, 1.0f};
constexpr Color kCapOutline{0.035f, 0.035f, 0.045f, 1.0f};
constexpr Color kCapSheen{1.0f, 1.0f, 1.0f, 0.08f};
constexpr Color kCapBottomShadow{0.0f, 0.0f, 0.0f, 0.45f};

// Corner radius as a fraction of the key's short side, and how many segments
// approximate each quarter-circle corner.
constexpr float kCornerRadiusFrac = 0.28f;
constexpr int kCornerSegments = 4;

Color lerp(const Color& a, const Color& b, float t) {
  return Color{a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
               a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t};
}

// Builds a closed rounded-rect outline into `pts` (must hold at least
// 4 * (kCornerSegments + 1) + 1 points). Returns the point count.
int roundedRect(Point* pts, float x, float y, float w, float h, float radius) {
  const float r = std::min(radius, std::min(w, h) * 0.5f);
  // Corner centers in draw order (TL, TR, BR, BL) with each corner's start
  // angle; angles in radians, screen coords (+y down).
  struct Corner {
    float cx, cy, startDeg;
  };
  const Corner corners[4] = {
      {x + r, y + r, 180.0f},
      {x + w - r, y + r, 270.0f},
      {x + w - r, y + h - r, 0.0f},
      {x + r, y + h - r, 90.0f},
  };
  int n = 0;
  for (const Corner& c : corners) {
    for (int s = 0; s <= kCornerSegments; ++s) {
      const float deg =
          c.startDeg + 90.0f * static_cast<float>(s) / kCornerSegments;
      const float rad = deg * 3.14159265f / 180.0f;
      pts[n++] = {c.cx + r * std::cos(rad), c.cy + r * std::sin(rad)};
    }
  }
  pts[n] = pts[0];  // close the loop for strokePolyline
  return n;
}

}  // namespace

void drawKeyFace(Renderer& r, float x, float y, float w, float h, float press) {
  const float level = std::clamp(press, 0.0f, 1.0f);
  const float radius = std::min(w, h) * kCornerRadiusFrac;

  Point pts[4 * (kCornerSegments + 1) + 1];
  const int n = roundedRect(pts, x, y, w, h, radius);

  r.fillPolygon(pts, n, lerp(kCapBase, kCapPressed, level));

  // Soft sheen across the top half (inset past the corners so it stays inside
  // the rounded cap), giving the molded-plastic convex look.
  r.fillRectVerticalGradient(x + radius, y + h * 0.08f, w - 2.0f * radius,
                             h * 0.42f, y + h * 0.08f, y + h * 0.5f, kCapSheen,
                             Color{1.0f, 1.0f, 1.0f, 0.0f});
  // Shadow along the bottom inner edge.
  r.strokeLine(x + radius, y + h - 1.5f, x + w - radius, y + h - 1.5f, 1.5f,
               kCapBottomShadow);

  r.strokePolyline(pts, n + 1, 1.5f, kCapOutline);
}

}  // namespace avionics::bezel
