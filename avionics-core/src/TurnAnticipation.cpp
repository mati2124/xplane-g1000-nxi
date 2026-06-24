#include "avionics/TurnAnticipation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "avionics/FplRouteEdit.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFeetPerNm = 6076.12;
constexpr double kG = 32.174;  // ft/s^2
// Normal 15° bank angle used for leg smoothing (G1000 NXi Pilot's Guide,
// Appendix D "When does turn anticipation begin?").
constexpr double kTurnBankDeg = 15.0;
// Countdown and flash window before the computed turn point.
constexpr double kCountdownLeadSec = 10.0;
// Use the turn-advisory wording when the outbound course change exceeds this.
constexpr double kTurnAdvisoryDeg = 15.0;

double shortestTurnDeltaDeg(double inboundDeg, double outboundDeg) {
  return std::fmod(outboundDeg - inboundDeg + 540.0, 360.0) - 180.0;
}

std::string formatTrackDeg(float deg) {
  int hdg = static_cast<int>(std::lround(deg)) % 360;
  if (hdg < 0) hdg += 360;
  if (hdg == 0) hdg = 360;
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%03d", hdg);
  return std::string(buf);
}

// Fly-by lead distance for a course change at the given ground speed and bank.
double turnLeadDistanceNm(double gsKts, double turnDeltaDeg) {
  if (gsKts < 1.0 || std::fabs(turnDeltaDeg) < 0.5) return 0.0;
  const double bankRad = kTurnBankDeg * kPi / 180.0;
  const double deltaRad =
      std::fabs(turnDeltaDeg) * kPi / 180.0 * 0.5;  // half the course change
  const double vFps = gsKts * kFeetPerNm / 3600.0;
  const double radiusFt = (vFps * vFps) / (kG * std::tan(bankRad));
  const double leadFt = radiusFt * std::tan(deltaRad);
  return leadFt / kFeetPerNm;
}

}  // namespace

TurnAnticipation computeTurnAnticipation(const MapData& map,
                                         const FlightData& data, bool obsMode) {
  TurnAnticipation out;
  if (obsMode) return out;
  if (data.cdiSource != CdiSource::Gps) return out;
  if (!map.positionValid || !data.dataLinkValid) return out;
  if (data.fmaToWpt.empty()) return out;
  // Leg navigation only: Direct-To (no FROM on the FMA) has no outbound turn.
  if (navDirectToActive(data)) return out;

  const std::vector<MapLeg>& plan = map.flightPlan;
  if (plan.size() < 2) return out;

  int activeIdx = data.fmaActiveLegIndex;
  if (activeIdx < 0 || activeIdx >= static_cast<int>(plan.size())) {
    activeIdx = legIndexInPlan(plan, data.fmaToWpt);
  }
  if (activeIdx < 0 || activeIdx + 1 >= static_cast<int>(plan.size())) {
    return out;
  }

  const MapLeg& active = plan[static_cast<std::size_t>(activeIdx)];
  const MapLeg& next = plan[static_cast<std::size_t>(activeIdx + 1)];

  const double inboundDeg =
      activeIdx > 0
          ? navBearingDeg(plan[static_cast<std::size_t>(activeIdx - 1)].lat,
                          plan[static_cast<std::size_t>(activeIdx - 1)].lon,
                          active.lat, active.lon)
          : navBearingDeg(map.ownshipLat, map.ownshipLon, active.lat,
                          active.lon);
  const double outboundDeg =
      navBearingDeg(active.lat, active.lon, next.lat, next.lon);
  const double turnDeltaDeg = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  if (std::fabs(turnDeltaDeg) < 1.0) return out;

  const double gsKts = std::max(40.0, static_cast<double>(data.groundSpeedKts));
  const double leadNm = turnLeadDistanceNm(gsKts, turnDeltaDeg);
  const double distToWptNm = std::max(0.0, static_cast<double>(data.fmaLegDistanceNm));
  const double distToTurnNm = distToWptNm - leadNm;
  const double timeToTurnSec = distToTurnNm / (gsKts / 3600.0);

  // Anticipation begins when within the lead distance plus the 10-second
  // countdown window.
  const double announceNm = leadNm + gsKts / 3600.0 * kCountdownLeadSec;
  if (distToWptNm > announceNm) return out;

  const std::string track = formatTrackDeg(static_cast<float>(outboundDeg));
  const bool useTurnAdvisory =
      std::fabs(turnDeltaDeg) >= kTurnAdvisoryDeg || timeToTurnSec <= 0.0;
  const bool countdown =
      timeToTurnSec > 0.0 && timeToTurnSec <= kCountdownLeadSec + 0.5;
  const int secondsLeft =
      countdown ? std::max(1, static_cast<int>(std::ceil(timeToTurnSec))) : 0;

  out.active = true;
  out.flashing = countdown || timeToTurnSec <= 0.0;

  char buf[96];
  if (useTurnAdvisory) {
    const char* direction = turnDeltaDeg >= 0.0 ? "right" : "left";
    if (timeToTurnSec <= 0.0) {
      std::snprintf(buf, sizeof(buf), "Turn %s to %s\u00b0 now", direction,
                    track.c_str());
    } else if (countdown) {
      std::snprintf(buf, sizeof(buf), "Turn %s to %s\u00b0 in %d seconds",
                    direction, track.c_str(), secondsLeft);
    } else {
      std::snprintf(buf, sizeof(buf), "Next DTK %s\u00b0", track.c_str());
    }
  } else if (timeToTurnSec <= 0.0) {
    std::snprintf(buf, sizeof(buf), "Next DTK %s\u00b0 now", track.c_str());
  } else if (countdown) {
    std::snprintf(buf, sizeof(buf), "Next DTK %s\u00b0 in %d seconds",
                  track.c_str(), secondsLeft);
  } else {
    std::snprintf(buf, sizeof(buf), "Next DTK %s\u00b0", track.c_str());
  }
  out.message = buf;
  return out;
}

void applyTurnAnticipation(FlightData& data, const MapData& map, bool obsMode,
                           bool blinkOn) {
  data.navStatusAnnunciation.clear();
  data.navStatusAnnunciationFlash = false;

  const TurnAnticipation ta = computeTurnAnticipation(map, data, obsMode);
  if (!ta.active) return;

  data.navStatusAnnunciationFlash = ta.flashing;
  if (ta.flashing && !blinkOn) return;
  data.navStatusAnnunciation = ta.message;
}

}  // namespace avionics
