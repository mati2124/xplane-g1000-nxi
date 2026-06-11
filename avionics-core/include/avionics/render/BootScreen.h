#pragma once

#include <string>

#include "avionics/NavDatabase.h"
#include "avionics/Renderer.h"

namespace avionics {

// Power-on initialization shown for the first few seconds after the display
// comes up, mirroring the real G1000 NXi sequence: a centered Garmin logo
// splash, then the MFD Power-up Page (database currency table). Stateless like
// the other page renderers; the engine owns the timer/phase and the renderer
// just draws what it is told.
class BootScreen {
 public:
  enum class Phase {
    Logo,     // centered Garmin logo on black
    PowerUp,  // MFD Power-up Page: database currency table + status prompt
  };

  // sourceLabel names the data feed being brought online (e.g. "X-PLANE",
  // "MOCK DATA"). navDatabase fills the AVIATION row of the database table; an
  // expired database is shown in amber so the pilot is alerted before
  // acknowledging, as on the real unit. awaitingAck applies on the PowerUp
  // phase: true once initialization is complete and the unit is waiting for the
  // pilot to press ENT to acknowledge the database information; false shows
  // "INITIALIZING SYSTEM" (still coming up, or a feed that advances on its own).
  // powerUpAlpha in [0, 1] ramps the Power-up Page opacity during the 2s
  // ease-in-out cross-fade from the logo splash (StartupLogo.css); ignored on
  // the Logo phase.
  static void render(Renderer& r, Phase phase, const std::string& sourceLabel,
                     const NavDatabaseInfo& navDatabase, bool awaitingAck,
                     float powerUpAlpha, int widthPx, int heightPx);
};

}  // namespace avionics
