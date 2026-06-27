#pragma once

#include <string>
#include <vector>

#include "avionics/Renderer.h"

// NanoVG-backed Renderer shared by both shells.
//
// This is the one place that turns the gauge code's backend-independent vector
// commands into real GPU draw calls. It does NOT create or own a GPU context:
// the hosting shell (GLFW window for standalone, the sim's context for the
// plugin) must make a context current *before* constructing this, because the
// NanoVG GL backend is created in the constructor.
struct NVGcontext;

namespace avionics {

class NanoVgRenderer : public Renderer {
 public:
  // Which NanoVG OpenGL backend to bind. GL3 suits a modern 3.2+ core context
  // (the standalone shell's GLFW window); GL2 suits an OpenGL 2.1 context (the
  // X-Plane plugin's avionics-bridge context).
  enum class Backend { GL3, GL2 };

  explicit NanoVgRenderer(Backend backend = Backend::GL3);
  ~NanoVgRenderer() override;

  NanoVgRenderer(const NanoVgRenderer&) = delete;
  NanoVgRenderer& operator=(const NanoVgRenderer&) = delete;

  // False if the NanoVG context failed to create (e.g. no current GL context).
  bool valid() const { return vg_ != nullptr; }

  // Per-frame draw-call accounting for performance diagnostics. Each NanoVG
  // path submission (fill/stroke), text run, and image draw is counted; the
  // counters reset on beginFrame, so a reader sees the call count for the most
  // recent frame. The in-sim GL2 backend is draw-call bound, so this is the
  // number the shell logs to reason about render cost.
  struct DrawStats {
    int fills = 0;
    int strokes = 0;
    int texts = 0;
    int images = 0;
    int verts = 0;        // path vertices submitted (polygons/polylines/segments)
    int nonFinite = 0;    // primitives dropped for NaN/Inf coords (bad live data)
    int total() const { return fills + strokes + texts + images; }
  };
  const DrawStats& drawStats() const { return stats_; }

  // When set, the next beginFrame() rotates the draw surface 180° after
  // nvgBeginFrame resets the stack (used on Windows landscape-flipped monitors).
  void setDisplayFlip180(bool flip) { displayFlip180_ = flip; }

  void beginFrame(int widthPx, int heightPx, float pixelRatio) override;
  void endFrame() override;

  void save() override;
  void restore() override;
  void translate(float x, float y) override;
  void rotateDegrees(float degrees) override;
  void clip(float x, float y, float w, float h) override;
  void globalAlpha(float alpha) override;

  void fillRect(float x, float y, float w, float h, const Color& c) override;
  void fillRectVerticalGradient(float x, float y, float w, float h,
                                float gradientTopY, float gradientBottomY,
                                const Color& topColor,
                                const Color& bottomColor) override;
  void strokeLine(float x1, float y1, float x2, float y2, float widthPx,
                  const Color& c) override;
  void fillCircle(float cx, float cy, float radius, const Color& c) override;
  void fillRoundedRect(float x, float y, float w, float h, float radius,
                       const Color& c) override;
  void fillRoundedRectVerticalGradient(float x, float y, float w, float h,
                                       float radius, float gradientTopY,
                                       float gradientBottomY,
                                       const Color& topColor,
                                       const Color& bottomColor) override;
  void strokeRoundedRect(float x, float y, float w, float h, float radius,
                         float widthPx, const Color& c) override;
  void fillTopRoundedRectVerticalGradient(float x, float y, float w, float h,
                                          float radius, const Color& topColor,
                                          const Color& bottomColor) override;
  void strokeTopRoundedRect(float x, float y, float w, float h, float radius,
                            float widthPx, const Color& c) override;
  void fillRoundedRectVaryingVerticalGradient(
      float x, float y, float w, float h, float radTL, float radTR, float radBR,
      float radBL, const Color& topColor, const Color& bottomColor) override;
  void strokeRoundedRectVarying(float x, float y, float w, float h, float radTL,
                                float radTR, float radBR, float radBL,
                                float widthPx, const Color& c) override;
  void fillPolygon(const Point* points, int count, const Color& c) override;
  void strokePolyline(const Point* points, int count, float widthPx,
                      const Color& c) override;
  void strokeSegments(const Point* segPts, int segmentCount, float widthPx,
                      const Color& c) override;
  int createImageRGBA(int widthPx, int heightPx,
                      const unsigned char* rgba) override;
  void updateImageRGBA(int imageId, const unsigned char* rgba) override;
  void deleteImage(int imageId) override;
  void drawImage(int imageId, float x, float y, float w, float h,
                 float alpha) override;
  void pushDefaultFontFace(FontFace face) override;
  void popDefaultFontFace() override;
  void fillText(float x, float y, const std::string& text, float sizePx,
                TextAlign align, const Color& c,
                FontFace face = FontFace::Default) override;
  float measureTextWidth(const std::string& text, float sizePx,
                         FontFace face = FontFace::Default) override;
  TextRect measureTextRect(float x, float y, const std::string& text,
                           float sizePx, TextAlign align,
                           FontFace face = FontFace::Default) override;

 private:
  // Resolve a requested face to a loaded NanoVG font id, falling back to the
  // primary font when the secondary face is unavailable.
  int fontIdFor(FontFace face) const;

  Backend backend_;
  NVGcontext* vg_ = nullptr;
  int fontId_ = -1;         // primary UI font (Roboto)
  int dejavuFontId_ = -1;   // secondary display face (DejaVu Sans SemiBold)
  int boldFontId_ = -1;     // bold weight (Roboto Bold), e.g. softkey labels
  std::vector<FontFace> defaultFaceStack_;  // active Default-face overrides
  DrawStats stats_;
  bool displayFlip180_ = false;
};

}  // namespace avionics
