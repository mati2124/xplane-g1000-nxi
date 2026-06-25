#include "avionics/HoldNavigation.h"

#include <algorithm>
#include <cmath>

#include "avionics/NavMath.h"
#include "avionics/TurnAnticipation.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWaypointCaptureNm = 0.4;
constexpr float kEntrySectorDeg = 70.0f;
constexpr float kTeardropOutboundMin = 1.0f;  // NM outbound before teardrop turn

double crossTrackNm(double lat, double lon, double fromLat, double fromLon,
                    double toLat, double toLon) {
  const double legBrg = navBearingDeg(fromLat, fromLon, toLat, toLon);
  const double brgToFrom = navBearingDeg(lat, lon, fromLat, fromLon);
  const double distToFrom = navDistanceNm(lat, lon, fromLat, fromLon);
  const double angleRad = (brgToFrom - legBrg) * kPi / 180.0;
  return distToFrom * std::sin(angleRad);
}

HoldGuidance guidanceOnArc(double lat, double lon, double centerLat,
                           double centerLon, float turnRadiusNm, bool rightTurn,
                           const std::string& wptId) {
  HoldGuidance out;
  out.active = true;
  out.fromWpt = wptId;
  out.toWpt = wptId;
  const double distToCenter = navDistanceNm(lat, lon, centerLat, centerLon);
  const float bearingToCenter =
      static_cast<float>(navBearingDeg(lat, lon, centerLat, centerLon));
  out.desiredTrackDeg =
      normalizeHeadingDeg(bearingToCenter + (rightTurn ? 90.0f : -90.0f));
  out.crossTrackNm = static_cast<float>(distToCenter - turnRadiusNm);
  out.distanceToWaypointNm = static_cast<float>(turnRadiusNm * kPi);
  out.bearingToWaypointDeg = bearingToCenter;
  return out;
}

float arcSweepDeg(double centerLat, double centerLon, double fromLat,
                  double fromLon, double toLat, double toLon, bool rightTurn) {
  const float start =
      static_cast<float>(navBearingDeg(centerLat, centerLon, fromLat, fromLon));
  const float end =
      static_cast<float>(navBearingDeg(centerLat, centerLon, toLat, toLon));
  float sweep = shortestTurnDeltaDeg(start, end);
  if (!rightTurn && sweep > 0.0f) sweep -= 360.0f;
  if (rightTurn && sweep < 0.0f) sweep += 360.0f;
  return sweep;
}

}  // namespace

HoldEntryType classifyHoldEntry(float approachTrackDeg, float inboundCourseDeg) {
  const float outbound = normalizeHeadingDeg(inboundCourseDeg + 180.0f);
  const float toInbound =
      std::fabs(shortestTurnDeltaDeg(approachTrackDeg, inboundCourseDeg));
  const float toOutbound =
      std::fabs(shortestTurnDeltaDeg(approachTrackDeg, outbound));
  if (toInbound <= kEntrySectorDeg) return HoldEntryType::Direct;
  if (toOutbound <= kEntrySectorDeg) return HoldEntryType::Parallel;
  return HoldEntryType::Teardrop;
}

HoldPatternPhase initialHoldPhase(HoldEntryType entry) {
  switch (entry) {
    case HoldEntryType::Direct:
      // Cross the fix and turn outbound (ICAO direct entry).
      return HoldPatternPhase::Outbound;
    case HoldEntryType::Parallel:
      return HoldPatternPhase::Outbound;
    case HoldEntryType::Teardrop:
      return HoldPatternPhase::EntryTeardrop;
  }
  return HoldPatternPhase::Outbound;
}

double holdAlongTrackNm(double lat, double lon, double fixLat, double fixLon,
                        double courseDeg) {
  const double distNm = navDistanceNm(fixLat, fixLon, lat, lon);
  if (distNm < 0.001) return 0.0;
  const double bearingFromFixDeg = navBearingDeg(fixLat, fixLon, lat, lon);
  const double angleRad =
      shortestTurnDeltaDeg(courseDeg, bearingFromFixDeg) * kPi / 180.0;
  return distNm * std::cos(angleRad);
}

HoldGuidance computeHoldGuidance(double lat, double lon, const MapLeg& leg,
                                 HoldPatternPhase phase, float groundSpeedKts) {
  HoldGuidance out;
  const MapHoldPattern& hold = leg.hold;
  if (!hold.active || hold.turn == HoldTurnDirection::None) return out;

  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, groundSpeedKts);
  if (!geom.valid || geom.legLengthNm <= 0.0f) return out;

  const float inbound = geom.inboundCourseDeg;
  const float outbound = geom.outboundCourseDeg;
  const float legLen = geom.legLengthNm;
  const bool rightTurn = geom.rightTurn;
  const double fixLat = geom.fixLat;
  const double fixLon = geom.fixLon;

  out.active = true;
  out.fromWpt = leg.id;
  out.toWpt = leg.id;

  if (phase == HoldPatternPhase::EntryTeardrop) {
    out.desiredTrackDeg = outbound;
    double endLat = 0.0;
    double endLon = 0.0;
    navOffsetPoint(fixLat, fixLon, outbound, kTeardropOutboundMin, endLat,
                   endLon);
    out.crossTrackNm = static_cast<float>(
        crossTrackNm(lat, lon, fixLat, fixLon, endLat, endLon));
    const double along = holdAlongTrackNm(lat, lon, fixLat, fixLon, outbound);
    out.distanceToWaypointNm =
        static_cast<float>(std::max(0.0, kTeardropOutboundMin - along));
    out.bearingToWaypointDeg =
        static_cast<float>(navBearingDeg(lat, lon, endLat, endLon));
    return out;
  }

  if (phase == HoldPatternPhase::Outbound) {
    out.desiredTrackDeg = outbound;
    // The outbound leg is the parallel leg displaced 2R to the holding side
    // (inboundPar -> outboundPar), not a straight line out of the fix. Tracking
    // the displaced leg keeps lateral guidance aligned with the drawn racetrack
    // so successive laps stay on the oval instead of cutting back across the
    // inbound axis (which produced a figure-8).
    const double startLat = geom.inboundParLat;
    const double startLon = geom.inboundParLon;
    out.crossTrackNm = static_cast<float>(crossTrackNm(
        lat, lon, startLat, startLon, geom.outboundParLat, geom.outboundParLon));
    const double along =
        holdAlongTrackNm(lat, lon, startLat, startLon, outbound);
    out.distanceToWaypointNm =
        static_cast<float>(std::max(0.0, legLen - along));
    out.bearingToWaypointDeg = static_cast<float>(
        navBearingDeg(lat, lon, geom.outboundParLat, geom.outboundParLon));
    return out;
  }

  if (phase == HoldPatternPhase::OutboundTurn) {
    return guidanceOnArc(lat, lon, geom.turn1CenterLat, geom.turn1CenterLon,
                         geom.turnRadiusNm, rightTurn, leg.id);
  }

  if (phase == HoldPatternPhase::InboundParallel) {
    out.desiredTrackDeg = inbound;
    double farLat = 0.0;
    double farLon = 0.0;
    navOffsetPoint(fixLat, fixLon, inbound, 10.0, farLat, farLon);
    out.crossTrackNm = static_cast<float>(
        crossTrackNm(lat, lon, fixLat, fixLon, farLat, farLon));
    const double alongInbound =
        holdAlongTrackNm(lat, lon, fixLat, fixLon, inbound);
    out.distanceToWaypointNm = static_cast<float>(std::max(0.0, alongInbound));
    out.bearingToWaypointDeg =
        static_cast<float>(navBearingDeg(lat, lon, fixLat, fixLon));
    return out;
  }

  if (phase == HoldPatternPhase::InboundTurn) {
    return guidanceOnArc(lat, lon, geom.turn2CenterLat, geom.turn2CenterLon,
                         geom.turnRadiusNm, rightTurn, leg.id);
  }

  return out;
}

HoldPatternPhase advanceHoldPatternPhase(double lat, double lon,
                                         const MapLeg& leg,
                                         HoldPatternPhase phase,
                                         float groundSpeedKts) {
  const MapHoldPattern& hold = leg.hold;
  if (!hold.active) return phase;

  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, groundSpeedKts);
  if (!geom.valid) return phase;

  const float outbound = geom.outboundCourseDeg;
  const float legLen = geom.legLengthNm;
  const double gsKts = std::max(40.0, static_cast<double>(groundSpeedKts));

  if (phase == HoldPatternPhase::EntryTeardrop) {
    const double along =
        holdAlongTrackNm(lat, lon, leg.lat, leg.lon, outbound);
    const double leadNm = turnLeadDistanceNm(gsKts, 180.0);
    if (along + leadNm >= kTeardropOutboundMin) {
      return HoldPatternPhase::OutboundTurn;
    }
    return phase;
  }

  if (phase == HoldPatternPhase::Outbound) {
    const double along =
        holdAlongTrackNm(lat, lon, leg.lat, leg.lon, outbound);
    const double leadNm = turnLeadDistanceNm(gsKts, 180.0);
    if (along + leadNm >= legLen) {
      return HoldPatternPhase::OutboundTurn;
    }
    return phase;
  }

  if (phase == HoldPatternPhase::OutboundTurn) {
    // Top turn: from the displaced outbound leg end (outboundPar) around to the
    // inbound axis (outboundEnd).
    const float sweep = arcSweepDeg(
        geom.turn1CenterLat, geom.turn1CenterLon, geom.outboundParLat,
        geom.outboundParLon, geom.outboundEndLat, geom.outboundEndLon,
        geom.rightTurn);
    const float pos =
        static_cast<float>(navBearingDeg(geom.turn1CenterLat, geom.turn1CenterLon,
                                         lat, lon));
    const float start =
        static_cast<float>(navBearingDeg(geom.turn1CenterLat, geom.turn1CenterLon,
                                         geom.outboundParLat, geom.outboundParLon));
    const float traveled = geom.rightTurn
                               ? (pos - start >= 0 ? pos - start : pos - start + 360.0f)
                               : (start - pos >= 0 ? start - pos : start - pos + 360.0f);
    if (traveled >= std::fabs(sweep) - 15.0f) {
      return HoldPatternPhase::InboundParallel;
    }
    return phase;
  }

  if (phase == HoldPatternPhase::InboundParallel) {
    const double distToFix = navDistanceNm(lat, lon, leg.lat, leg.lon);
    if (distToFix <= kWaypointCaptureNm) {
      return HoldPatternPhase::InboundTurn;
    }
    return phase;
  }

  if (phase == HoldPatternPhase::InboundTurn) {
    // Bottom turn: from the fix around to the start of the displaced outbound
    // leg (inboundPar).
    const float sweep = arcSweepDeg(
        geom.turn2CenterLat, geom.turn2CenterLon, geom.fixLat, geom.fixLon,
        geom.inboundParLat, geom.inboundParLon, geom.rightTurn);
    const float pos =
        static_cast<float>(navBearingDeg(geom.turn2CenterLat, geom.turn2CenterLon,
                                         lat, lon));
    const float start =
        static_cast<float>(navBearingDeg(geom.turn2CenterLat, geom.turn2CenterLon,
                                         geom.fixLat, geom.fixLon));
    const float traveled = geom.rightTurn
                               ? (pos - start >= 0 ? pos - start : pos - start + 360.0f)
                               : (start - pos >= 0 ? start - pos : start - pos + 360.0f);
    if (traveled >= std::fabs(sweep) - 15.0f) {
      return HoldPatternPhase::Outbound;
    }
    return phase;
  }

  return phase;
}

}  // namespace avionics
