#include "avionics/TurnAnticipation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

namespace avionics {

namespace {

// Degree sign as explicit UTF-8 bytes. A narrow "\u00b0" literal is emitted in
// the system codepage by MSVC (a lone 0xB0 byte), which the UTF-8 text renderer
// treats as invalid and drops along with everything after it -- that silently
// truncated the " in N seconds" countdown suffix from the turn advisory.
constexpr char kDegUtf8[] = "\xC2\xB0";

constexpr double kPi = 3.14159265358979323846;
constexpr double kFeetPerNm = 6076.12;
constexpr double kG = 32.174;  // ft/s^2
// Countdown and flash window before the computed turn point.
constexpr double kCountdownLeadSec = 10.0;
// Use the turn-advisory wording when the outbound course change exceeds this.
constexpr double kTurnAdvisoryDeg = 15.0;

std::string formatTrackDeg(float deg) {
  int hdg = static_cast<int>(std::lround(deg)) % 360;
  if (hdg < 0) hdg += 360;
  if (hdg == 0) hdg = 360;
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%03d", hdg);
  return std::string(buf);
}

}  // namespace

double shortestTurnDeltaDeg(double inboundDeg, double outboundDeg) {
  return std::fmod(outboundDeg - inboundDeg + 540.0, 360.0) - 180.0;
}

double turnLeadDistanceNm(double gsKts, double turnDeltaDeg,
                          double maxTurnDegCap, double bankDeg) {
  if (gsKts < 1.0 || std::fabs(turnDeltaDeg) < 0.5) return 0.0;
  // Large course reversals cap the angle used in the formula so the annunciation
  // does not read "now" for miles; Direct-To off-route entries may use a higher
  // cap (e.g. 135°) so the fly-by still begins before the fix.
  const double cappedDeltaDeg =
      std::min(std::fabs(turnDeltaDeg), maxTurnDegCap);
  const double bankRad =
      std::max(5.0, bankDeg) * kPi / 180.0;
  const double deltaRad = cappedDeltaDeg * kPi / 180.0 * 0.5;
  const double vFps = gsKts * kFeetPerNm / 3600.0;
  const double radiusFt = (vFps * vFps) / (kG * std::tan(bankRad));
  const double leadFt = radiusFt * std::tan(deltaRad);
  return leadFt / kFeetPerNm;
}

TurnAnticipation computeTurnAnticipation(const MapData& map,
                                         const FlightData& data, bool obsMode,
                                         CdiSource cdiSource) {
  TurnAnticipation out;
  if (obsMode || data.gpsSequencingSuspended) return out;
  if (cdiSource != CdiSource::Gps) return out;
  if (!map.positionValid || !data.dataLinkValid) return out;
  if (data.fmaToWpt.empty()) return out;
  const double bankDeg = static_cast<double>(data.turnLeadBankDeg);
  const std::vector<MapLeg>& plan = map.flightPlan;
  if (plan.size() < 2) return out;

  int activeIdx = data.fmaActiveLegIndex;
  if (activeIdx < 0 || activeIdx >= static_cast<int>(plan.size())) {
    activeIdx = legIndexInPlan(plan, data.fmaToWpt);
  }

  // Direct-To to a fix on the loaded plan: still announce the outbound turn to
  // the next leg (e.g. Direct-To PINTS with AZOMY following on the approach).
  bool directToOnPlan = false;
  if (map.directToActive) {
    if (map.directTo.id.empty()) return out;
    const int dtoIdx = legIndexInPlan(plan, map.directTo);
    if (dtoIdx < 0 || dtoIdx + 1 >= static_cast<int>(plan.size())) {
      return out;
    }
    activeIdx = dtoIdx;
    directToOnPlan = true;
  }

  if (activeIdx < 0 || activeIdx + 1 >= static_cast<int>(plan.size())) {
    return out;
  }

  const MapLeg& active = plan[static_cast<std::size_t>(activeIdx)];
  const MapLeg& next = plan[static_cast<std::size_t>(activeIdx + 1)];

  // A hold fix (HILPT course reversal or holding pattern) is flown OVER, not
  // flown BY: the aircraft crosses the fix and reverses course through the hold
  // entry, it does not lead the turn onto the next leg. Anticipating a turn here
  // makes the autopilot swing toward the post-hold inbound course before the fix
  // instead of entering the hold (e.g. KCMI RNAV 04 CMI->BOSTN HILPT swung the
  // aircraft right toward 041 for a few seconds before the hold took over).
  if (active.hold.active) return out;

  // After a fly-by sequences early onto the outbound leg, the active TO waypoint
  // is already the outbound fix while the aircraft is still rounding the prior
  // fix. Suppress the *next* leg's turn advisory until the prior fly-by is
  // complete so the countdown does not start miles early (regression: AZOMY→
  // UZAWO turn still in progress but GRAMS countdown already running).
  if (activeIdx >= 2) {
    const MapLeg& prevFix = plan[static_cast<std::size_t>(activeIdx - 1)];
    const MapLeg& priorFix = plan[static_cast<std::size_t>(activeIdx - 2)];
    const double inboundPrevDeg =
        navBearingDeg(priorFix.lat, priorFix.lon, prevFix.lat, prevFix.lon);
    const double outboundPrevDeg =
        navBearingDeg(prevFix.lat, prevFix.lon, active.lat, active.lon);
    const double prevTurnDeltaDeg =
        shortestTurnDeltaDeg(inboundPrevDeg, outboundPrevDeg);
    if (std::fabs(prevTurnDeltaDeg) >= 1.0) {
      const double gsKts =
          std::max(40.0, static_cast<double>(data.groundSpeedKts));
      const double prevLeadNm =
          turnLeadDistanceNm(gsKts, prevTurnDeltaDeg, 90.0, bankDeg);
      const double distPrevFixNm =
          navDistanceNm(map.ownshipLat, map.ownshipLon, prevFix.lat, prevFix.lon);
      if (distPrevFixNm <= prevLeadNm + kTurnSteeringMarginNm) {
        return out;
      }
    }
  }

  const double inboundDeg =
      directToOnPlan
          ? (map.directToOriginValid
                 ? navBearingDeg(map.directToOriginLat, map.directToOriginLon,
                                 active.lat, active.lon)
                 : navBearingDeg(map.ownshipLat, map.ownshipLon, active.lat,
                                 active.lon))
          : activeIdx > 0
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
  const double maxTurnDegCap =
      directToOnPlan ? kDirectToFlyByMaxTurnDegCap : 90.0;
  const double leadNm =
      turnLeadDistanceNm(gsKts, turnDeltaDeg, maxTurnDegCap, bankDeg);
  const double steeringMarginNm =
      directToOnPlan ? kTurnSteeringMarginNm : 0.0;
  const double distToWptNm =
      std::max(0.0, static_cast<double>(data.fmaLegDistanceNm));
  const double distToTurnNm = distToWptNm - leadNm - steeringMarginNm;
  const double timeToTurnSec = distToTurnNm / (gsKts / 3600.0);

  // Anticipation begins when within the lead distance plus the 10-second
  // countdown window (and the Direct-To steering margin when applicable).
  const double announceNm =
      leadNm + steeringMarginNm + gsKts / 3600.0 * kCountdownLeadSec;
  if (distToWptNm > announceNm) return out;

  const std::string track = formatTrackDeg(static_cast<float>(outboundDeg));
  const bool useTurnAdvisory =
      std::fabs(turnDeltaDeg) >= kTurnAdvisoryDeg || timeToTurnSec <= 0.0;
  const bool countdown =
      timeToTurnSec > 0.0 && timeToTurnSec <= kCountdownLeadSec + 0.5;
  const int secondsLeft =
      countdown ? std::max(1, static_cast<int>(std::ceil(timeToTurnSec))) : 0;

  out.active = true;
  // The NXi Navigation Status Box shows the turn advisory steadily (counting
  // the seconds down); it does not blink.
  out.flashing = false;

  char buf[96];
  if (useTurnAdvisory) {
    const char* direction = turnDeltaDeg >= 0.0 ? "right" : "left";
    if (timeToTurnSec <= 0.0) {
      std::snprintf(buf, sizeof(buf), "Turn %s to %s%s now", direction,
                    track.c_str(), kDegUtf8);
    } else if (countdown) {
      std::snprintf(buf, sizeof(buf), "Turn %s to %s%s in %d seconds",
                    direction, track.c_str(), kDegUtf8, secondsLeft);
    } else {
      std::snprintf(buf, sizeof(buf), "Next DTK %s%s", track.c_str(), kDegUtf8);
    }
  } else if (timeToTurnSec <= 0.0) {
    std::snprintf(buf, sizeof(buf), "Next DTK %s%s now", track.c_str(),
                  kDegUtf8);
  } else if (countdown) {
    std::snprintf(buf, sizeof(buf), "Next DTK %s%s in %d seconds",
                  track.c_str(), kDegUtf8, secondsLeft);
  } else {
    std::snprintf(buf, sizeof(buf), "Next DTK %s%s", track.c_str(), kDegUtf8);
  }
  out.message = buf;
  return out;
}

void applyTurnAnticipation(FlightData& data, const MapData& map, bool obsMode,
                           CdiSource cdiSource) {
  data.navStatusAnnunciation.clear();
  data.navStatusAnnunciationFlash = false;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, obsMode, cdiSource);
  if (!ta.active) return;

  data.navStatusAnnunciationFlash = ta.flashing;
  data.navStatusAnnunciation = ta.message;
}

}  // namespace avionics
