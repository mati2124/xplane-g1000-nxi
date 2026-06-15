#pragma once

#include <string>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/NavDatabase.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"

namespace avionics {

// Power-on initialization shown for the first few seconds after the display
// comes up, mirroring the real G1000 NXi sequence: a centered Garmin logo
// splash on both GDUs, then the MFD Power-up Page (database currency review)
// or the PFD initialization view (red-X instruments + AHRS align message).
class BootScreen {
 public:
  enum class Phase {
    Logo,     // centered Garmin logo on black (both displays)
    PowerUp,  // post-logo screen: MFD database page or PFD init
  };

  enum class Target {
    Pfd,
    Mfd,
  };

  // Logo splash (phase Logo) or the post-logo screen for the given GDU.
  // navDatabase fills the Navigation row on the MFD page. awaitingAck on the MFD
  // shows the ENT / right-softkey continue prompt once the boot timer has
  // elapsed (live sources); the mock feed advances on its own. phaseAlpha in
  // [0, 1] is the ease-in-out opacity for the current phase: it fades the Garmin
  // logo up from black on the Logo phase and cross-fades the post-logo screen in
  // on the PowerUp phase.
  static void render(Renderer& r, Target target, Phase phase,
                     const FlightData& flightData, const MapData& map,
                     const SoftkeyController& pfdUi, const MfdController& mfdUi,
                     const NavDatabaseInfo& navDatabase, bool awaitingAck,
                     float phaseAlpha, int widthPx, int heightPx);
};

}  // namespace avionics
