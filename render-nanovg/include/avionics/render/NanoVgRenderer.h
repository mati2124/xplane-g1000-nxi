#pragma once

#include <string>

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
  NanoVgRenderer();
  ~NanoVgRenderer() override;

  NanoVgRenderer(const NanoVgRenderer&) = delete;
  NanoVgRenderer& operator=(const NanoVgRenderer&) = delete;

  // False if the NanoVG context failed to create (e.g. no current GL context).
  bool valid() const { return vg_ != nullptr; }

  void beginFrame(int widthPx, int heightPx, float pixelRatio) override;
  void endFrame() override;

  void save() override;
  void restore() override;
  void translate(float x, float y) override;
  void rotateDegrees(float degrees) override;
  void clip(float x, float y, float w, float h) override;

  void fillRect(float x, float y, float w, float h, const Color& c) override;
  void fillRectVerticalGradient(float x, float y, float w, float h,
                                float gradientTopY, float gradientBottomY,
                                const Color& topColor,
                                const Color& bottomColor) override;
  void strokeLine(float x1, float y1, float x2, float y2, float widthPx,
                  const Color& c) override;
  void fillCircle(float cx, float cy, float radius, const Color& c) override;
  void fillPolygon(const Point* points, int count, const Color& c) override;
  void strokePolyline(const Point* points, int count, float widthPx,
                      const Color& c) override;
  void fillText(float x, float y, const std::string& text, float sizePx,
                TextAlign align, const Color& c) override;
  float measureTextWidth(const std::string& text, float sizePx) override;

 private:
  NVGcontext* vg_ = nullptr;
  int fontId_ = -1;
};

}  // namespace avionics
