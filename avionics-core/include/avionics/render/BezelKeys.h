#pragma once

#include "avionics/Renderer.h"

namespace avionics {

// Hardware keys that live on the right bezel of a G1000 GDU, emulated on-screen
// so the unit is usable without a physical bezel. Only the range rocker is
// wired to a function today; the rest mirror the real key column and give
// press feedback. Order is top-to-bottom, which the hit-test relies on.
enum class BezelKey {
  DirectTo,
  Menu,
  Proc,
  Clr,
  Ent,
  RangeUp,
  RangeDown,
  Count,
};
inline constexpr int kBezelKeyCount = static_cast<int>(BezelKey::Count);

// Draws and hit-tests the bezel key column inside an explicit rectangle (the
// window's bezel strip beside the screen). The keys fill the rect top-to-bottom.
// Render and hit-test share one layout so they always agree.
class BezelKeyPanel {
 public:
  // Draws the bezel face and keys filling [x, x+w] x [y, y+h]. pressLevels
  // points at kBezelKeyCount floats (0..1) for the press-flash; displayH scales
  // the label fonts to the physical display height.
  static void render(Renderer& r, float x, float y, float w, float h,
                     float displayH, const float* pressLevels);

  // Returns the key under the pointer, or BezelKey::Count if outside any key.
  static BezelKey hitTest(float xPx, float yPx, float x, float y, float w,
                          float h);
};

}  // namespace avionics
