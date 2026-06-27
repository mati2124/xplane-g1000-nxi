#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "avionics/HoldGeometry.h"
#include "avionics/HoldNavigation.h"
#include "avionics/NavMath.h"

namespace avionics::test {
namespace {

MapLeg makeNorthHoldFix() {
  MapLeg leg;
  leg.id = "HOLD1";
  leg.lat = 26.000000;
  leg.lon = -81.200000;
  leg.procedureRole = "mahp";
  leg.hold.active = true;
  leg.hold.inboundCourseDeg = 0.0f;
  leg.hold.legLengthNm = 1.0f;
  leg.hold.turn = HoldTurnDirection::Right;
  return leg;
}

}  // namespace

TEST(HoldNavigationTest, OutboundTracksPublishedCourse) {
  const MapLeg leg = makeNorthHoldFix();
  const HoldGuidance g = computeHoldGuidance(25.995, leg.lon, leg,
                                             HoldPatternPhase::Outbound, 90.0f);
  ASSERT_TRUE(g.active);
  EXPECT_NEAR(g.desiredTrackDeg, 180.0f, 0.5f);
  EXPECT_GT(g.distanceToWaypointNm, 0.4f);
}

TEST(HoldNavigationTest, InboundTracksPublishedCourse) {
  const MapLeg leg = makeNorthHoldFix();
  const HoldGuidance g = computeHoldGuidance(25.998, leg.lon, leg,
                                             HoldPatternPhase::InboundParallel,
                                             90.0f);
  ASSERT_TRUE(g.active);
  EXPECT_NEAR(g.desiredTrackDeg, 0.0f, 0.5f);
}

// The outbound leg is the parallel leg displaced 2R to the holding side, not a
// straight line out of the fix. On the displaced leg cross-track must be ~0,
// while a point on the inbound axis (through the fix) is ~2R off. This guards
// the figure-8 regression where outbound was flown on the inbound axis.
TEST(HoldNavigationTest, OutboundTracksDisplacedLeg) {
  const MapLeg leg = makeNorthHoldFix();  // inbound 000, right -> holding side E
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, 90.0f);
  ASSERT_TRUE(geom.valid);

  // Midpoint of the displaced outbound leg (inboundPar -> outboundPar).
  const double midLat = 0.5 * (geom.inboundParLat + geom.outboundParLat);
  const double midLon = 0.5 * (geom.inboundParLon + geom.outboundParLon);
  const HoldGuidance onLeg = computeHoldGuidance(
      midLat, midLon, leg, HoldPatternPhase::Outbound, 90.0f);
  ASSERT_TRUE(onLeg.active);
  EXPECT_NEAR(onLeg.crossTrackNm, 0.0f, 0.05f);

  // A point on the inbound axis below the fix is roughly 2R off the leg.
  double axisLat = 0.0;
  double axisLon = 0.0;
  navOffsetPoint(leg.lat, leg.lon, 180.0, 0.5, axisLat, axisLon);
  const HoldGuidance offLeg = computeHoldGuidance(
      axisLat, axisLon, leg, HoldPatternPhase::Outbound, 90.0f);
  EXPECT_GT(std::fabs(offLeg.crossTrackNm), geom.turnRadiusNm);
}

TEST(HoldNavigationTest, AdvancesOutboundToOutboundTurnAtLegLength) {
  const MapLeg leg = makeNorthHoldFix();
  double southLat = 0.0;
  double southLon = 0.0;
  navOffsetPoint(leg.lat, leg.lon, 180.0, 0.95, southLat, southLon);
  const HoldPatternPhase next =
      advanceHoldPatternPhase(southLat, southLon, leg,
                              HoldPatternPhase::Outbound, 90.0f);
  EXPECT_EQ(next, HoldPatternPhase::OutboundTurn);
}

TEST(HoldNavigationTest, AdvancesInboundToOutboundAtFix) {
  const MapLeg leg = makeNorthHoldFix();
  const HoldPatternPhase next =
      advanceHoldPatternPhase(leg.lat, leg.lon, leg,
                              HoldPatternPhase::InboundParallel, 90.0f);
  EXPECT_EQ(next, HoldPatternPhase::InboundTurn);
}

TEST(HoldNavigationTest, CourseReversalCompletesInsteadOfLooping) {
  MapLeg leg = makeNorthHoldFix();
  leg.hold.courseReversal = true;
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, 90.0f);
  ASSERT_TRUE(geom.valid);
  const HoldPatternPhase loop = advanceHoldPatternPhase(
      geom.inboundParLat, geom.inboundParLon, leg,
      HoldPatternPhase::InboundTurn, 90.0f, false);
  EXPECT_EQ(loop, HoldPatternPhase::Outbound);
  const HoldPatternPhase done = advanceHoldPatternPhase(
      geom.inboundParLat, geom.inboundParLon, leg,
      HoldPatternPhase::InboundTurn, 90.0f, true);
  EXPECT_EQ(done, HoldPatternPhase::CircuitComplete);
}

TEST(HoldNavigationTest, ClassifyDirectEntryOnInbound) {
  EXPECT_EQ(classifyHoldEntry(0.0f, 0.0f), HoldEntryType::Direct);
  EXPECT_EQ(initialHoldPhase(HoldEntryType::Direct),
            HoldPatternPhase::Outbound);
}

TEST(HoldNavigationTest, ClassifyParallelEntryOnOutbound) {
  EXPECT_EQ(classifyHoldEntry(180.0f, 0.0f), HoldEntryType::Parallel);
  EXPECT_EQ(initialHoldPhase(HoldEntryType::Parallel),
            HoldPatternPhase::Outbound);
}

TEST(HoldNavigationTest, TimedHoldUsesGroundSpeed) {
  MapLeg leg = makeNorthHoldFix();
  leg.hold.legLengthNm = 0.0f;
  leg.hold.legTimeMin = 1.0f;
  EXPECT_NEAR(effectiveHoldLegLengthNm(leg.hold, 120.0f), 2.0f, 0.1f);
}

TEST(HoldGeometryTest, TessellatedRacetrackHasArcPoints) {
  const MapLeg leg = makeNorthHoldFix();
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, 90.0f);
  ASSERT_TRUE(geom.valid);
  const auto pts = tessellateHoldRacetrack(geom, 10);
  EXPECT_GE(pts.size(), 14u);
}

// A right-hand hold with a north inbound course (000deg) lies on the EAST side
// (inbound + 90). The two 180deg turns must bulge OUTWARD along the leg axis:
// the outbound-end turn north of the outbound end, the fix-end turn south of the
// fix. A turn that caves inward produces the pinched "fish-hook" misdraw.
TEST(HoldGeometryTest, TurnsBulgeOutwardNotInward) {
  const MapLeg leg = makeNorthHoldFix();  // inbound 000, right, 1 NM legs
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, 90.0f);
  ASSERT_TRUE(geom.valid);

  // Holding side is east of the fix for a right-hand, north-inbound hold.
  EXPECT_GT(geom.inboundParLon, geom.fixLon);
  EXPECT_GT(geom.outboundParLon, geom.fixLon);

  const auto pts = tessellateHoldRacetrack(geom, 16);
  ASSERT_GE(pts.size(), 14u);

  double maxLat = -90.0;
  double minLat = 90.0;
  for (const auto& p : pts) {
    maxLat = std::max(maxLat, p.first);
    minLat = std::min(minLat, p.first);
  }
  // Outbound end is north of the fix; the top turn must reach beyond it.
  EXPECT_GT(maxLat, geom.outboundEndLat - 1e-6);
  // The fix-end turn must dip south of the fix (outward), not cave north.
  EXPECT_LT(minLat, geom.fixLat - 1e-6);
}

}  // namespace avionics::test
