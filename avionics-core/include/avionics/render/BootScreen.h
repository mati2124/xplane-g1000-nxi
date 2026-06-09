#pragma once

#include <string>

#include "avionics/Renderer.h"

namespace avionics {

// Power-on initialization splash shown for the first few seconds after the
// display comes up, mirroring the self-test screen a real EFIS shows at
// power-up. Stateless like the other page renderers; the engine owns the timer
// and passes the current progress.
class BootScreen {
 public:
  // progress01 in [0, 1] drives the initialization bar. sourceLabel names the
  // data feed being brought online (e.g. "X-PLANE", "MOCK DATA").
  static void render(Renderer& r, const std::string& sourceLabel,
                     float progress01, int widthPx, int heightPx);
};

}  // namespace avionics
