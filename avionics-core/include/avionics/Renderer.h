#pragma once

#include <string>

#include "avionics/Color.h"

namespace avionics {

enum class TextAlign { Left, Center, Right };

// A 2D point in renderer (display-pixel) coordinates, origin top-left, +y down.
struct Point {
  float x = 0.0f;
  float y = 0.0f;
};

// Abstraction over "where pixels go".
//
// Each shell supplies a concrete backend bound to its GPU context:
//   - X-Plane plugin: NanoVG over the sim's active GL/Vulkan/Metal context.
//   - Standalone:     NanoVG over a GLFW/SDL window we own.
//
// The gauge code issues backend-independent vector commands so the same
// drawing logic produces identical output on both targets.
class Renderer {
 public:
  virtual ~Renderer() = default;

  virtual void beginFrame(int widthPx, int heightPx, float pixelRatio) = 0;
  virtual void endFrame() = 0;

  // Transform stack (origin top-left, +y down).
  virtual void save() = 0;
  virtual void restore() = 0;
  virtual void translate(float x, float y) = 0;
  virtual void rotateDegrees(float degrees) = 0;
  virtual void clip(float x, float y, float w, float h) = 0;

  // Primitives.
  virtual void fillRect(float x, float y, float w, float h, const Color& c) = 0;

  // Fills the rect with a vertical (top-to-bottom) linear gradient. The
  // gradient maps topColor at gradientTopY to bottomColor at gradientBottomY
  // (same coordinate space as the rect); pixels outside that span clamp to the
  // nearest end color. Decoupling the gradient axis from the rect lets callers
  // oversize the rect (e.g. the attitude background) while keeping the color
  // transition anchored to a feature such as the horizon.
  virtual void fillRectVerticalGradient(float x, float y, float w, float h,
                                        float gradientTopY, float gradientBottomY,
                                        const Color& topColor,
                                        const Color& bottomColor) = 0;
  virtual void strokeLine(float x1, float y1, float x2, float y2,
                          float widthPx, const Color& c) = 0;
  virtual void fillCircle(float cx, float cy, float radius, const Color& c) = 0;

  // Filled, implicitly-closed polygon through `count` points (count >= 3).
  // Used for pointers, chevrons, slip/skid markers, and tape-box notches.
  virtual void fillPolygon(const Point* points, int count, const Color& c) = 0;

  // Open polyline through `count` points (count >= 2); for outlines and arcs.
  virtual void strokePolyline(const Point* points, int count, float widthPx,
                              const Color& c) = 0;

  // Strokes many disjoint line segments in a single batched submission.
  // `segPts` holds 2*segmentCount points (each consecutive pair is one
  // segment). This exists because dashed/combed/railroad symbology emits
  // hundreds of tiny segments per frame; issuing them as one path instead of
  // one draw call each is a large win on the in-sim GL2 backend, where every
  // draw call carries heavy driver overhead. The default implementation simply
  // loops strokeLine so backends that don't override it still render correctly.
  virtual void strokeSegments(const Point* segPts, int segmentCount,
                              float widthPx, const Color& c) {
    for (int i = 0; i < segmentCount; ++i) {
      const Point& a = segPts[2 * i];
      const Point& b = segPts[2 * i + 1];
      strokeLine(a.x, a.y, b.x, b.y, widthPx, c);
    }
  }

  // Raster images (RGBA8, row-major, no padding). Used for content that is
  // expensive to draw as vectors every frame (the terrain raster): a caller
  // builds a pixel buffer, uploads it once, then draws it cheaply per frame
  // under the current transform. Returns an opaque handle (< 0 on failure).
  virtual int createImageRGBA(int widthPx, int heightPx,
                              const unsigned char* rgba) = 0;
  // Re-uploads pixels into an existing image; dimensions must match creation.
  virtual void updateImageRGBA(int imageId, const unsigned char* rgba) = 0;
  virtual void deleteImage(int imageId) = 0;
  // Draws the image stretched over the rect (current transform applies, so
  // callers rotate via save/translate/rotate). `alpha` multiplies the image.
  virtual void drawImage(int imageId, float x, float y, float w, float h,
                         float alpha) = 0;

  virtual void fillText(float x, float y, const std::string& text, float sizePx,
                        TextAlign align, const Color& c) = 0;

  // Horizontal advance width (px) that fillText would use for `text` at
  // `sizePx`, in the current transform's units. Lets the gauge code lay text
  // out by measured width so adjacent fields never overlap. Backends without a
  // loaded font may return an approximation.
  virtual float measureTextWidth(const std::string& text, float sizePx) = 0;
};

}  // namespace avionics
