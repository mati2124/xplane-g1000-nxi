#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>

namespace avionics::mapview {

void strokeDashedPolyline(Renderer& r, const Point* pts, int count,
                          float widthPx, const Color& c,
                          const ClipBounds* clip) {
  constexpr float kDashPx = 6.0f;
  constexpr float kGapPx = 5.0f;
  // Accumulate every dash as a disjoint segment and stroke them all in one
  // batched submission (a single draw call) instead of one per dash.
  static thread_local std::vector<Point> segs;
  segs.clear();
  float phase = 0.0f;
  for (int i = 0; i + 1 < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    constexpr float kPeriod = kDashPx + kGapPx;
    // Restrict the dash walk to the visible span of this edge; off-screen
    // portions would emit dashes that are never seen.
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;
      if (pos < lo) pos += std::floor((lo - pos) / kPeriod) * kPeriod;
      for (; pos < len && pos <= hi; pos += kPeriod) {
        // Clamp the dash to the visible span [lo, hi], not the full segment
        // [0, len]: a dash that starts before the visible region must be cut
        // at lo, otherwise it would extend into off-screen pixels.
        const float dashStart = std::max(pos, lo);
        const float dashEnd = std::min(pos + kDashPx, hi);
        if (dashEnd > dashStart) {
          segs.push_back({a.x + ux * dashStart, a.y + uy * dashStart});
          segs.push_back({a.x + ux * dashEnd, a.y + uy * dashEnd});
        }
      }
    }
    phase = std::fmod(phase + len, kPeriod);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), widthPx, c);
  }
}

void drawRailroad(Renderer& r, const Point* pts, int count, const Color& c,
                  const ClipBounds* clip) {
  if (count < 2) return;
  r.strokePolyline(pts, count, 1.0f, c);
  constexpr float kTickStepPx = 8.0f;
  constexpr float kTickHalfPx = 2.5f;
  // Batch all crossties into one stroke submission rather than one per tie.
  static thread_local std::vector<Point> segs;
  segs.clear();
  float phase = 0.0f;  // carry spacing across segments so ties stay even
  for (int i = 0; i + 1 < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    const float ux = dx / len;
    const float uy = dy / len;
    const float nx = -uy;  // perpendicular for the crosstie
    const float ny = ux;
    float lo = 0.0f, hi = len;
    bool visible = true;
    if (clip != nullptr) {
      visible = segmentVisibleSpan(a.x, a.y, ux, uy, len, clip->minX,
                                   clip->minY, clip->maxX, clip->maxY,
                                   kSymbologyClipMarginPx, lo, hi);
    }
    if (visible) {
      float pos = -phase;
      if (pos < lo) pos += std::floor((lo - pos) / kTickStepPx) * kTickStepPx;
      for (; pos < len && pos <= hi; pos += kTickStepPx) {
        if (pos >= 0.0f) {
          const float px = a.x + ux * pos;
          const float py = a.y + uy * pos;
          segs.push_back({px - nx * kTickHalfPx, py - ny * kTickHalfPx});
          segs.push_back({px + nx * kTickHalfPx, py + ny * kTickHalfPx});
        }
      }
    }
    phase = std::fmod(phase + len, kTickStepPx);
  }
  if (!segs.empty()) {
    r.strokeSegments(segs.data(), static_cast<int>(segs.size() / 2), 1.0f, c);
  }
}

void drawChromeBox(Renderer& r, float x, float y, float w, float h) {
  constexpr int kCornerSegs = 4;
  constexpr int kPts = 4 * (kCornerSegs + 1);
  const float radius = std::min(h * 0.32f, w * 0.5f);
  struct Corner {
    float cx, cy, a0;
  };
  const Corner corners[4] = {
      {x + radius, y + radius, 3.14159265f},
      {x + w - radius, y + radius, 4.71238898f},
      {x + w - radius, y + h - radius, 0.0f},
      {x + radius, y + h - radius, 1.57079633f},
  };
  Point pts[kPts + 1];
  int n = 0;
  for (const Corner& c : corners) {
    for (int s = 0; s <= kCornerSegs; ++s) {
      const float a =
          c.a0 + 1.57079633f * static_cast<float>(s) / kCornerSegs;
      pts[n++] = {c.cx + radius * std::cos(a), c.cy + radius * std::sin(a)};
    }
  }
  pts[n] = pts[0];
  r.fillPolygon(pts, kPts, Color{0.0f, 0.0f, 0.0f, 0.78f});
  r.strokePolyline(pts, n + 1, 1.0f, Color{0.75f, 0.75f, 0.75f, 1.0f});
}

float drawChromeLabel(Renderer& r, float x, float y, const char* text,
                      float size, const Color& textColor) {
  const float padX = size * 0.45f;
  const float w = r.measureTextWidth(text, size) + 2.0f * padX;
  const float h = size * 1.5f;
  drawChromeBox(r, x, y, w, h);
  r.fillText(x + padX, y + h * 0.52f, text, size, TextAlign::Left, textColor);
  return h;
}

}  // namespace avionics::mapview
