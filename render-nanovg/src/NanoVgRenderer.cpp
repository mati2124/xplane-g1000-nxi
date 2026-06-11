#include "avionics/render/NanoVgRenderer.h"

#include <array>

#include "nanovg.h"

// Defined in the GL backend translation units (nanovg_gl_impl.c for GL3,
// nanovg_gl2_impl.c for GL2). Declared here with C linkage so this C++ file
// never has to pull in platform GL headers.
extern "C" NVGcontext* nvgCreateGL3(int flags);
extern "C" void nvgDeleteGL3(NVGcontext* ctx);
extern "C" NVGcontext* nvgCreateGL2(int flags);
extern "C" void nvgDeleteGL2(NVGcontext* ctx);

namespace avionics {
namespace {

// nvgCreateGL3 flag bits (mirror of the NVGcreateFlags enum in nanovg_gl.h),
// duplicated locally so this file avoids including the GL backend header.
constexpr int kNvgAntialias = 1 << 0;
constexpr int kNvgStencilStrokes = 1 << 1;

// Candidate fonts, tried in order. First one that loads wins; if none load,
// text is simply skipped (gauges still draw). Roboto *Regular* is preferred to
// match the Working Title G1000 NXi, whose default font family is Roboto
// Regular (not bold); the bundled Bold and the system fonts are fallbacks.
#ifndef AVIONICS_FONT_DIR
#define AVIONICS_FONT_DIR "."
#endif
constexpr std::array<const char*, 6> kFontCandidates = {
    AVIONICS_FONT_DIR "/Roboto-Regular.ttf",
    AVIONICS_FONT_DIR "/Roboto-Bold.ttf",
    "/System/Library/Fonts/Supplemental/Arial.ttf",
    "/System/Library/Fonts/SFNSMono.ttf",
    "/System/Library/Fonts/Menlo.ttc",
    "/System/Library/Fonts/Helvetica.ttc",
};

constexpr const char* kFontName = "sans";

NVGcolor toNvg(const Color& c) { return nvgRGBAf(c.r, c.g, c.b, c.a); }

}  // namespace

NanoVgRenderer::NanoVgRenderer(Backend backend) : backend_(backend) {
  // Stencil strokes add a stencil pass per stroke to suppress overlap fringing
  // (~3x the per-stroke cost). The standalone GL3 path keeps it for best
  // quality, but the in-sim GL2 path runs inside X-Plane's frame budget against
  // a stroke-heavy moving map, so it trades that pass for speed; the only
  // visible effect is faint double-blending where translucent strokes overlap.
  if (backend_ == Backend::GL2) {
    vg_ = nvgCreateGL2(kNvgAntialias);
  } else {
    vg_ = nvgCreateGL3(kNvgAntialias | kNvgStencilStrokes);
  }
  if (!vg_) return;

  for (const char* path : kFontCandidates) {
    fontId_ = nvgCreateFont(vg_, kFontName, path);
    if (fontId_ >= 0) break;
  }
}

NanoVgRenderer::~NanoVgRenderer() {
  if (!vg_) return;
  if (backend_ == Backend::GL2) {
    nvgDeleteGL2(vg_);
  } else {
    nvgDeleteGL3(vg_);
  }
}

void NanoVgRenderer::beginFrame(int widthPx, int heightPx, float pixelRatio) {
  if (!vg_) return;
  stats_ = DrawStats{};
  nvgBeginFrame(vg_, static_cast<float>(widthPx),
                static_cast<float>(heightPx), pixelRatio);
}

void NanoVgRenderer::endFrame() {
  if (!vg_) return;
  nvgEndFrame(vg_);
}

void NanoVgRenderer::save() {
  if (vg_) nvgSave(vg_);
}

void NanoVgRenderer::restore() {
  if (vg_) nvgRestore(vg_);
}

void NanoVgRenderer::translate(float x, float y) {
  if (vg_) nvgTranslate(vg_, x, y);
}

void NanoVgRenderer::rotateDegrees(float degrees) {
  if (vg_) nvgRotate(vg_, nvgDegToRad(degrees));
}

void NanoVgRenderer::clip(float x, float y, float w, float h) {
  if (vg_) nvgScissor(vg_, x, y, w, h);
}

void NanoVgRenderer::fillRect(float x, float y, float w, float h,
                              const Color& c) {
  if (!vg_) return;
  ++stats_.fills;
  nvgBeginPath(vg_);
  nvgRect(vg_, x, y, w, h);
  nvgFillColor(vg_, toNvg(c));
  nvgFill(vg_);
}

void NanoVgRenderer::fillRectVerticalGradient(float x, float y, float w, float h,
                                              float gradientTopY,
                                              float gradientBottomY,
                                              const Color& topColor,
                                              const Color& bottomColor) {
  if (!vg_) return;
  // sx == ex makes the gradient purely vertical; NanoVG clamps to the end
  // colors outside [gradientTopY, gradientBottomY].
  ++stats_.fills;
  NVGpaint paint = nvgLinearGradient(vg_, x, gradientTopY, x, gradientBottomY,
                                     toNvg(topColor), toNvg(bottomColor));
  nvgBeginPath(vg_);
  nvgRect(vg_, x, y, w, h);
  nvgFillPaint(vg_, paint);
  nvgFill(vg_);
}

void NanoVgRenderer::strokeLine(float x1, float y1, float x2, float y2,
                                float widthPx, const Color& c) {
  if (!vg_) return;
  ++stats_.strokes;
  nvgBeginPath(vg_);
  nvgMoveTo(vg_, x1, y1);
  nvgLineTo(vg_, x2, y2);
  nvgStrokeWidth(vg_, widthPx);
  nvgStrokeColor(vg_, toNvg(c));
  nvgStroke(vg_);
}

void NanoVgRenderer::fillCircle(float cx, float cy, float radius,
                                const Color& c) {
  if (!vg_) return;
  ++stats_.fills;
  nvgBeginPath(vg_);
  nvgCircle(vg_, cx, cy, radius);
  nvgFillColor(vg_, toNvg(c));
  nvgFill(vg_);
}

void NanoVgRenderer::fillPolygon(const Point* points, int count,
                                 const Color& c) {
  if (!vg_ || count < 3) return;
  ++stats_.fills;
  stats_.verts += count;
  nvgBeginPath(vg_);
  nvgMoveTo(vg_, points[0].x, points[0].y);
  for (int i = 1; i < count; ++i) nvgLineTo(vg_, points[i].x, points[i].y);
  nvgClosePath(vg_);
  nvgFillColor(vg_, toNvg(c));
  nvgFill(vg_);
}

void NanoVgRenderer::strokePolyline(const Point* points, int count,
                                    float widthPx, const Color& c) {
  if (!vg_ || count < 2) return;
  ++stats_.strokes;
  stats_.verts += count;
  nvgBeginPath(vg_);
  nvgMoveTo(vg_, points[0].x, points[0].y);
  for (int i = 1; i < count; ++i) nvgLineTo(vg_, points[i].x, points[i].y);
  nvgStrokeWidth(vg_, widthPx);
  nvgStrokeColor(vg_, toNvg(c));
  nvgStroke(vg_);
}

void NanoVgRenderer::strokeSegments(const Point* segPts, int segmentCount,
                                    float widthPx, const Color& c) {
  if (!vg_ || segmentCount < 1) return;
  // All segments share one path and one stroke: NanoVG supports many disjoint
  // subpaths in a single submission, so this is one tessellation + draw call
  // instead of `segmentCount` of them.
  ++stats_.strokes;
  stats_.verts += 2 * segmentCount;
  nvgBeginPath(vg_);
  for (int i = 0; i < segmentCount; ++i) {
    const Point& a = segPts[2 * i];
    const Point& b = segPts[2 * i + 1];
    nvgMoveTo(vg_, a.x, a.y);
    nvgLineTo(vg_, b.x, b.y);
  }
  nvgStrokeWidth(vg_, widthPx);
  nvgStrokeColor(vg_, toNvg(c));
  nvgStroke(vg_);
}

int NanoVgRenderer::createImageRGBA(int widthPx, int heightPx,
                                    const unsigned char* rgba) {
  if (!vg_) return -1;
  return nvgCreateImageRGBA(vg_, widthPx, heightPx, 0, rgba);
}

void NanoVgRenderer::updateImageRGBA(int imageId, const unsigned char* rgba) {
  if (!vg_ || imageId < 0) return;
  nvgUpdateImage(vg_, imageId, rgba);
}

void NanoVgRenderer::deleteImage(int imageId) {
  if (!vg_ || imageId < 0) return;
  nvgDeleteImage(vg_, imageId);
}

void NanoVgRenderer::drawImage(int imageId, float x, float y, float w, float h,
                               float alpha) {
  if (!vg_ || imageId < 0 || w <= 0.0f || h <= 0.0f) return;
  ++stats_.images;
  const NVGpaint paint = nvgImagePattern(vg_, x, y, w, h, 0.0f, imageId, alpha);
  nvgBeginPath(vg_);
  nvgRect(vg_, x, y, w, h);
  nvgFillPaint(vg_, paint);
  nvgFill(vg_);
}

void NanoVgRenderer::fillText(float x, float y, const std::string& text,
                              float sizePx, TextAlign align, const Color& c) {
  if (!vg_ || fontId_ < 0) return;
  ++stats_.texts;

  int hAlign = NVG_ALIGN_LEFT;
  switch (align) {
    case TextAlign::Left:
      hAlign = NVG_ALIGN_LEFT;
      break;
    case TextAlign::Center:
      hAlign = NVG_ALIGN_CENTER;
      break;
    case TextAlign::Right:
      hAlign = NVG_ALIGN_RIGHT;
      break;
  }

  nvgFontSize(vg_, sizePx);
  nvgFontFaceId(vg_, fontId_);
  nvgTextAlign(vg_, hAlign | NVG_ALIGN_MIDDLE);
  nvgFillColor(vg_, toNvg(c));
  nvgText(vg_, x, y, text.c_str(), nullptr);
}

float NanoVgRenderer::measureTextWidth(const std::string& text, float sizePx) {
  if (!vg_ || fontId_ < 0) {
    // No font: fall back to a rough monospace-ish estimate so layout still
    // advances sensibly.
    return static_cast<float>(text.size()) * sizePx * 0.55f;
  }
  nvgFontSize(vg_, sizePx);
  nvgFontFaceId(vg_, fontId_);
  nvgTextAlign(vg_, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
  float bounds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  return nvgTextBounds(vg_, 0.0f, 0.0f, text.c_str(), nullptr, bounds);
}

}  // namespace avionics
