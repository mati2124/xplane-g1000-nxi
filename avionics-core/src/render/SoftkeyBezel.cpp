#include "avionics/render/SoftkeyBezel.h"

#include <algorithm>

#include "avionics/Color.h"
#include "avionics/SoftkeyController.h"
#include "render/BezelStyle.h"

namespace avionics {
namespace {

// Key cap size within its cell: real GDU softkeys are wide rounded caps with
// clear gaps between neighbors and bezel face showing above and below.
constexpr float kKeyWidthFrac = 0.74f;   // of the cell width
constexpr float kKeyHeightFrac = 0.58f;  // of the strip height

struct Cell {
  float x, y, w, h;
};

Cell keyRect(int index, float x, float y, float w, float h) {
  const float cellW = w / static_cast<float>(kSoftkeyCount);
  const float keyW = cellW * kKeyWidthFrac;
  const float keyH = h * kKeyHeightFrac;
  return {x + (static_cast<float>(index) + 0.5f) * cellW - keyW * 0.5f,
          y + (h - keyH) * 0.5f, keyW, keyH};
}

}  // namespace

void SoftkeyBezelPanel::render(Renderer& r, float x, float y, float w, float h,
                               const float* pressLevels) {
  // Bezel face below the screen, continuing the right key column's finish.
  r.fillRectVerticalGradient(x, y, w, h, y, y + h, bezel::kFaceTop,
                             bezel::kFaceBottom);
  r.strokeLine(x, y, x + w, y, 2.0f, colors::kPanelBorder);

  for (int i = 0; i < kSoftkeyCount; ++i) {
    const Cell c = keyRect(i, x, y, w, h);
    const float press =
        pressLevels != nullptr ? std::max(0.0f, pressLevels[i]) : 0.0f;
    bezel::drawKeyFace(r, c.x, c.y, c.w, c.h, press);
  }
}

int SoftkeyBezelPanel::hitTest(float xPx, float yPx, float x, float y, float w,
                               float h) {
  for (int i = 0; i < kSoftkeyCount; ++i) {
    const Cell c = keyRect(i, x, y, w, h);
    if (xPx >= c.x && xPx <= c.x + c.w && yPx >= c.y && yPx <= c.y + c.h) {
      return i;
    }
  }
  return -1;
}

}  // namespace avionics
