#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// G1000 NXi turn-anticipation countdown in the PFD Navigation Status Box
// (Pilot's Guide Section 5.1 / Appendix D). Replaces the active-leg field with
// "Next DTK … in N seconds" or "Turn right/left to … in N seconds", flashing
// during the final 10 seconds before the computed turn point.
struct TurnAnticipation {
  bool active = false;
  bool flashing = false;
  std::string message;
};

// Computes the turn-anticipation annunciation for the active GPS flight-plan
// leg. `obsMode` suspends automatic sequencing (OBS/SUSP softkey).
TurnAnticipation computeTurnAnticipation(const MapData& map,
                                         const FlightData& data, bool obsMode);

// Writes `navStatusAnnunciation` / `navStatusAnnunciationFlash` on `data`.
// When flashing, the message is blanked on off blink phases.
void applyTurnAnticipation(FlightData& data, const MapData& map, bool obsMode,
                           bool blinkOn);

}  // namespace avionics
