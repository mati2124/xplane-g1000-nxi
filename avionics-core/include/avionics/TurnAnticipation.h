#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// Wider fly-by lead for on-plan Direct-To entries that approach from off the
// published inbound (e.g. east of AZOMY). Matches FmsNavigator sequencing and
// applyFlyByTurnCourse Direct-To steering.
constexpr double kDirectToFlyByMaxTurnDegCap = 135.0;
// Margin past the computed lead point where outbound steering begins; the
// turn-anticipation countdown must reach "now" at this same point.
constexpr double kTurnSteeringMarginNm = 0.4;

// G1000 NXi turn-anticipation countdown in the PFD Navigation Status Box
// (Pilot's Guide Section 5.1 / Appendix D). Replaces the active-leg field with
// "Next DTK … in N seconds" or "Turn right/left to … in N seconds", shown
// steadily during the final 10 seconds before the computed turn point.
struct TurnAnticipation {
  bool active = false;
  // Always false: the NXi shows the advisory steadily, not blinking. Retained
  // so callers can still drive `navStatusAnnunciationFlash`.
  bool flashing = false;
  std::string message;
};

// Computes the turn-anticipation annunciation for the active GPS flight-plan
// leg. `obsMode` suspends automatic sequencing (OBS/SUSP softkey).
TurnAnticipation computeTurnAnticipation(const MapData& map,
                                         const FlightData& data, bool obsMode,
                                         CdiSource cdiSource);

// Writes `navStatusAnnunciation` / `navStatusAnnunciationFlash` on `data`.
// The PFD blanks the field on off blink phases when `navStatusAnnunciationFlash`
// is set.
void applyTurnAnticipation(FlightData& data, const MapData& map, bool obsMode,
                           CdiSource cdiSource);

// Shortest signed course change from inbound to outbound (degrees, + = right).
double shortestTurnDeltaDeg(double inboundDeg, double outboundDeg);

// Fly-by lead distance for a course change at the given ground speed and bank
// (G1000 NXi Pilot's Guide Appendix D, 15° bank). `maxTurnDegCap` limits the
// course-change angle used in the formula (default 90° for published legs).
double turnLeadDistanceNm(double gsKts, double turnDeltaDeg,
                          double maxTurnDegCap = 90.0);

}  // namespace avionics
