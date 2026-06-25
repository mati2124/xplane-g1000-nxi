#include "avionics/HoldGeometry.h"

#include <algorithm>
#include <cmath>

#include "avionics/NavMath.h"

namespace avionics {

namespace {

constexpr double kPi = 3.14159265358979323846;

void appendArc(std::vector<std::pair<double, double>>& pts, double centerLat,
               double centerLon, double radiusNm, float fromDeg, float sweepDeg,
               int segments) {
  const int steps = std::max(2, segments);
  const double step = sweepDeg / static_cast<double>(steps);
  for (int i = 1; i <= steps; ++i) {
    const float bearing = normalizeHeadingDeg(fromDeg + step * static_cast<float>(i));
    double lat = 0.0;
    double lon = 0.0;
    navOffsetPoint(centerLat, centerLon, bearing, radiusNm, lat, lon);
    pts.emplace_back(lat, lon);
  }
}

}  // namespace

float publishedHoldTurnRadiusNm(float legLengthNm) {
  const float legLen = std::max(1.0f, legLengthNm);
  return std::clamp(legLen * 0.25f, 0.55f, 1.1f);
}

float effectiveHoldLegLengthNm(const MapHoldPattern& hold, float groundSpeedKts) {
  if (hold.legLengthNm > 0.01f) return hold.legLengthNm;
  if (hold.legTimeMin > 0.01f) {
    const float gs = std::max(40.0f, groundSpeedKts);
    return hold.legTimeMin * gs / 60.0f;
  }
  return 4.0f;
}

HoldRacetrackGeom buildHoldRacetrack(const MapLeg& leg, float groundSpeedKts) {
  HoldRacetrackGeom geom;
  const MapHoldPattern& hold = leg.hold;
  if (!hold.active || hold.turn == HoldTurnDirection::None) {
    return geom;
  }

  geom.valid = true;
  geom.fixLat = leg.lat;
  geom.fixLon = leg.lon;
  geom.inboundCourseDeg = hold.inboundCourseDeg;
  geom.outboundCourseDeg = normalizeHeadingDeg(hold.inboundCourseDeg + 180.0f);
  geom.legLengthNm = effectiveHoldLegLengthNm(hold, groundSpeedKts);
  geom.rightTurn = hold.turn == HoldTurnDirection::Right;
  geom.turnRadiusNm = publishedHoldTurnRadiusNm(geom.legLengthNm);

  // Holding side is the protected side of the inbound course: right of the
  // inbound course for a right-hand hold, left for a left-hand hold. (SERFS:
  // inbound 174deg, right turns -> 264deg = west, so the pattern lies NW.)
  const float holdingSide = normalizeHeadingDeg(
      geom.inboundCourseDeg + (geom.rightTurn ? 90.0f : -90.0f));
  const double turnRadiusNm = geom.turnRadiusNm;

  navOffsetPoint(geom.fixLat, geom.fixLon, geom.outboundCourseDeg,
                 geom.legLengthNm, geom.outboundEndLat, geom.outboundEndLon);
  navOffsetPoint(geom.outboundEndLat, geom.outboundEndLon, holdingSide,
                 2.0 * turnRadiusNm, geom.outboundParLat, geom.outboundParLon);
  navOffsetPoint(geom.fixLat, geom.fixLon, holdingSide, 2.0 * turnRadiusNm,
                 geom.inboundParLat, geom.inboundParLon);
  navOffsetPoint(geom.outboundEndLat, geom.outboundEndLon, holdingSide,
                 turnRadiusNm, geom.turn1CenterLat, geom.turn1CenterLon);
  navOffsetPoint(geom.fixLat, geom.fixLon, holdingSide, turnRadiusNm,
                 geom.turn2CenterLat, geom.turn2CenterLon);
  return geom;
}

std::vector<std::pair<double, double>> tessellateHoldRacetrack(
    const HoldRacetrackGeom& geom, int arcSegments) {
  std::vector<std::pair<double, double>> pts;
  if (!geom.valid) return pts;

  // Sweep direction pairs with the (inbound-relative) holding side so each
  // 180deg turn bulges outward away from the pattern centerline.
  const float sweep = geom.rightTurn ? -180.0f : 180.0f;
  pts.reserve(6 + 2 * arcSegments);
  pts.emplace_back(geom.fixLat, geom.fixLon);
  pts.emplace_back(geom.outboundEndLat, geom.outboundEndLon);

  appendArc(pts, geom.turn1CenterLat, geom.turn1CenterLon, geom.turnRadiusNm,
            static_cast<float>(navBearingDeg(geom.turn1CenterLat,
                                             geom.turn1CenterLon,
                                             geom.outboundEndLat,
                                             geom.outboundEndLon)),
            sweep, arcSegments);

  pts.emplace_back(geom.outboundParLat, geom.outboundParLon);
  pts.emplace_back(geom.inboundParLat, geom.inboundParLon);

  appendArc(pts, geom.turn2CenterLat, geom.turn2CenterLon, geom.turnRadiusNm,
            static_cast<float>(navBearingDeg(geom.turn2CenterLat,
                                             geom.turn2CenterLon,
                                             geom.inboundParLat,
                                             geom.inboundParLon)),
            sweep, arcSegments);
  return pts;
}

}  // namespace avionics
