#pragma once

#include <string>

#include "avionics/Color.h"

namespace avionics {

enum class TextAlign { Left, Center, Right };

// Axis-aligned bounds of text as it would be drawn by fillText at (x, y).
struct TextRect {
  float left = 0.0f;
  float top = 0.0f;
  float right = 0.0f;
  float bottom = 0.0f;
  float width() const { return right - left; }
  float height() const { return bottom - top; }
};

// Selectable text face. Default is the primary UI font (Roboto, matching the
// Working Title G1000 NXi). DejaVuSemiBold is the bundled secondary face used
// where a closer match to the real unit's display typeface is wanted (e.g. the
// PFD Setup Menu); backends without that face loaded fall back to the default.
enum class FontFace { Default, DejaVuSemiBold };

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

  // Multiplies the alpha of every subsequent draw by `alpha` (0..1). It is part
  // of the saved transform state, so wrap it in save()/restore() to scope it to
  // a subtree. Used to fade whole pop-up windows/menus in and out uniformly
  // without threading an alpha through every draw call. Backends that don't
  // override this leave drawing fully opaque.
  virtual void globalAlpha(float /*alpha*/) {}

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

  // Rounded rectangle fill/outline (corner radius in the same units as the
  // rect). Used for pop-up window chrome that has rounded corners on the real
  // unit. Backends that don't override these fall back to square equivalents so
  // existing output is unchanged.
  virtual void fillRoundedRect(float x, float y, float w, float h, float radius,
                               const Color& c) {
    fillRect(x, y, w, h, c);
  }
  virtual void strokeRoundedRect(float x, float y, float w, float h,
                                 float radius, float widthPx, const Color& c) {
    const Point p[5] = {{x, y},     {x + w, y}, {x + w, y + h},
                        {x, y + h}, {x, y}};
    strokePolyline(p, 5, widthPx, c);
  }

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

  // Temporarily remaps text drawn with FontFace::Default to `face` until the
  // matching pop. Lets a whole menu/dialog subtree render in an alternate face
  // without threading a FontFace through every draw call. Calls nest (LIFO);
  // an explicit non-Default `face` argument to fillText still wins. Backends
  // that don't override these ignore the override (text stays Default).
  virtual void pushDefaultFontFace(FontFace /*face*/) {}
  virtual void popDefaultFontFace() {}

  virtual void fillText(float x, float y, const std::string& text, float sizePx,
                        TextAlign align, const Color& c,
                        FontFace face = FontFace::Default) = 0;

  // Horizontal advance width (px) that fillText would use for `text` at
  // `sizePx`, in the current transform's units. Lets the gauge code lay text
  // out by measured width so adjacent fields never overlap. Backends without a
  // loaded font may return an approximation. `face` must match the face the
  // text will be drawn with so the measured width and the draw agree.
  virtual float measureTextWidth(const std::string& text, float sizePx,
                                 FontFace face = FontFace::Default) = 0;

  // Pixel bounds of `text` drawn at (x, y) with the given alignment and face.
  // Backends without font metrics may approximate from measureTextWidth.
  virtual TextRect measureTextRect(float x, float y, const std::string& text,
                                   float sizePx, TextAlign align,
                                   FontFace face = FontFace::Default) {
    const float w = measureTextWidth(text, sizePx, face);
    float left = x;
    if (align == TextAlign::Center) {
      left = x - w * 0.5f;
    } else if (align == TextAlign::Right) {
      left = x - w;
    }
    const float halfH = sizePx * 0.40f;
    return {left, y - halfH, left + w, y + halfH};
  }
};

// RAII guard that pushes a default-font-face override for its lifetime, so all
// FontFace::Default text drawn within a scope (including early returns) uses the
// given face. Used to render a whole menu/dialog in DejaVu SemiBold.
class FontScope {
 public:
  FontScope(Renderer& r, FontFace face) : r_(r) { r_.pushDefaultFontFace(face); }
  ~FontScope() { r_.popDefaultFontFace(); }
  FontScope(const FontScope&) = delete;
  FontScope& operator=(const FontScope&) = delete;

 private:
  Renderer& r_;
};

}  // namespace avionics
