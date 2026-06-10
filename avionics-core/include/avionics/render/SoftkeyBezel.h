#pragma once

#include "avionics/Renderer.h"

namespace avionics {

// The row of twelve physical softkey selection keys on the bottom edge of a
// GDU 104X bezel, directly below the on-screen softkey label bar (G1000
// Pilot's Guide for the Diamond DA40, "Softkey Selection Keys"). The keys are
// unlabeled hardware buttons; the function of each is shown by the label
// drawn on screen directly above it. Emulated by the standalone shell as a
// strip along the bottom of the window, aligned with the 12 label cells.
class SoftkeyBezelPanel {
 public:
  // Draws the bezel face and the 12 keys filling [x, x+w] x [y, y+h].
  // pressLevels points at kSoftkeyCount floats (0..1) for press feedback.
  static void render(Renderer& r, float x, float y, float w, float h,
                     const float* pressLevels);

  // Returns the softkey index (0..11) under the pointer, or -1 if outside any
  // key. Layout matches render() exactly.
  static int hitTest(float xPx, float yPx, float x, float y, float w, float h);
};

}  // namespace avionics
