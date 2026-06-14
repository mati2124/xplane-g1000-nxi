#include <algorithm>

#include "avionics/MfdController.h"

// MAP - Weather Radar page (airborne GWX radar, Pilot's Guide, Radar Controls):
// the small FMS knob trims the bearing line while it is displayed, otherwise the
// antenna tilt. The large knob is left to step the page group out of the page.
namespace avionics {

bool MfdController::radarBezelKey(BezelKey key) {
  const bool onBearing =
      radarBearingLineOn_ && radarScan_ != RadarScan::Vertical;
  switch (key) {
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const float dir = key == BezelKey::FmsInnerCw ? 1.0f : -1.0f;
      if (onBearing) {
        radarBearingDeg_ = std::max(
            -kRadarBearingLimitDeg,
            std::min(kRadarBearingLimitDeg,
                     radarBearingDeg_ + dir * kRadarBearingStepDeg));
      } else {
        radarTiltDeg_ = std::max(
            -kRadarTiltLimitDeg,
            std::min(kRadarTiltLimitDeg,
                     radarTiltDeg_ + dir * kRadarTiltStepDeg));
      }
      return true;
    }
    default:
      return false;
  }
}

}  // namespace avionics
