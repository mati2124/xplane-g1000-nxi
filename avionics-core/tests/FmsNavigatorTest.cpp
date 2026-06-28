#include <gtest/gtest.h>

#include "avionics/AircraftProfile.h"
#include "avionics/FmsNavigator.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/HoldGeometry.h"
#include "avionics/HoldNavigation.h"
#include "avionics/NavMath.h"
#include "avionics/NavigationComputer.h"
#include "avionics/TurnAnticipation.h"

#include "ApproachTestFixtures.h"

namespace avionics::test {
namespace {

std::vector<MapLeg> makeAirportPlusApproachPlan() {
  MapLeg apt;
  apt.id = "KPGD";
  apt.lat = 26.920000;
  apt.lon = -81.990000;

  std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  plan.insert(plan.begin(), apt);
  return plan;
}

MapLeg makeLeg(const std::string& id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

MapLeg makeOffsetLeg(const std::string& id, double centerLat, double centerLon,
                     double bearingDeg, double distanceNm) {
  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(centerLat, centerLon, bearingDeg, distanceNm, lat, lon);
  return makeLeg(id, lat, lon);
}

std::vector<MapLeg> makeRfArcPlan() {
  constexpr double cLat = 40.0;
  constexpr double cLon = -88.0;
  constexpr float radiusNm = 5.0f;
  MapLeg from = makeOffsetLeg("ARCIN", cLat, cLon, 0.0, radiusNm);
  MapLeg to = makeOffsetLeg("ARCEND", cLat, cLon, 90.0, radiusNm);
  to.pathTerminator = "RF";
  to.procedureArc.active = true;
  to.procedureArc.centerLat = cLat;
  to.procedureArc.centerLon = cLon;
  to.procedureArc.radiusNm = radiusNm;
  to.procedureArc.turn = HoldTurnDirection::Right;
  to.procedureArc.centerIdent = "ARCTR";
  MapLeg next = makeOffsetLeg("NEXT", cLat, cLon, 135.0, radiusNm);
  return {from, to, next};
}

std::vector<MapLeg> makeKcmiDmeArcPlan() {
  constexpr double cmiLat = 40.034530556;
  constexpr double cmiLon = -88.276075000;
  MapLeg fexil = makeLeg("FEXIL", 39.921805556, -88.061063889);
  MapLeg fasob = makeLeg("FASOB", 40.207791667, -88.145527778);
  fasob.pathTerminator = "AF";
  fasob.procedureArc.active = true;
  fasob.procedureArc.centerLat = cmiLat;
  fasob.procedureArc.centerLon = cmiLon;
  fasob.procedureArc.radiusNm = 12.0f;
  fasob.procedureArc.turn = HoldTurnDirection::Left;
  fasob.procedureArc.centerIdent = "CMI";
  MapLeg stadi = makeLeg("STADI", 40.121183333, -88.210869444);
  stadi.pathTerminator = "CF";
  stadi.legCourseDeg = 207.0f;
  return {fexil, fasob, stadi};
}

TEST(FmsNavigatorTest, EmptyPlanIsInactive) {
  FmsNavigator nav;
  const NavigationSolution sol = nav.update(26.0, -81.2, 90.0f);
  EXPECT_FALSE(sol.active);
  EXPECT_EQ(nav.activeLegIndex(), -1);
}

TEST(FmsNavigatorTest, LoadsPlanWithFirstLegActive) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  EXPECT_EQ(nav.activeLegIndex(), 0);
  EXPECT_EQ(nav.flightPlan().size(), 3u);
}

TEST(FmsNavigatorTest, SequencesRnavApproachLegs) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  // En route to IAF.
  NavigationSolution sol =
      nav.update(plan[0].lat - 0.02, plan[0].lon, 90.0f);
  EXPECT_TRUE(sol.active);
  EXPECT_EQ(sol.toWpt, "IAF01");
  EXPECT_EQ(nav.activeLegIndex(), 0);

  // Capture IAF -> advance to FAF.
  sol = nav.update(plan[0].lat, plan[0].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "FAF01");

  // Capture FAF -> advance to MAPt.
  sol = nav.update(plan[1].lat, plan[1].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "RW09");

  // Hold on final fix after capture.
  sol = nav.update(plan[2].lat, plan[2].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "RW09");
}

TEST(FmsNavigatorTest, RfArcGuidanceUsesTangentAndSequencesAtArcEnd) {
  const std::vector<MapLeg> plan = makeRfArcPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  double midLat = 0.0;
  double midLon = 0.0;
  navOffsetPoint(plan[1].procedureArc.centerLat, plan[1].procedureArc.centerLon,
                 45.0, 5.0, midLat, midLon);
  NavigationSolution sol = nav.update(midLat, midLon, 90.0f);
  EXPECT_EQ(sol.toWpt, "ARCEND");
  EXPECT_NEAR(sol.desiredTrackDeg, 135.0f, 2.0f);
  EXPECT_NEAR(sol.crossTrackNm, 0.0f, 0.1f);

  double offLat = 0.0;
  double offLon = 0.0;
  navOffsetPoint(plan[1].procedureArc.centerLat, plan[1].procedureArc.centerLon,
                 45.0, 6.2, offLat, offLon);
  sol = nav.update(offLat, offLon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_GT(sol.crossTrackNm, 1.0f);

  sol = nav.update(plan[1].lat, plan[1].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "NEXT");
}

TEST(FmsNavigatorTest, AfDmeArcGuidanceUsesTangentAndSequencesAtArcEnd) {
  const std::vector<MapLeg> plan = makeKcmiDmeArcPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  double midLat = 0.0;
  double midLon = 0.0;
  navOffsetPoint(plan[1].procedureArc.centerLat, plan[1].procedureArc.centerLon,
                 77.0, 12.0, midLat, midLon);
  NavigationSolution sol = nav.update(midLat, midLon, 90.0f);
  EXPECT_EQ(sol.toWpt, "FASOB");
  EXPECT_NEAR(sol.desiredTrackDeg, 347.0f, 3.0f);
  EXPECT_NEAR(sol.crossTrackNm, 0.0f, 0.1f);

  double insideLat = 0.0;
  double insideLon = 0.0;
  navOffsetPoint(plan[1].procedureArc.centerLat, plan[1].procedureArc.centerLon,
                 77.0, 10.5, insideLat, insideLon);
  sol = nav.update(insideLat, insideLon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_LT(sol.crossTrackNm, -1.0f);

  sol = nav.update(plan[1].lat, plan[1].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "STADI");
}

TEST(FmsNavigatorTest, ProcedureTurnLegSequencesFromOffCourseAtFix) {
  std::vector<MapLeg> plan;
  plan.push_back(makeLeg("CMI", 40.034530556, -88.276075000));
  MapLeg hitvu = makeLeg("HITVU", 40.070000000, -88.400000000);
  hitvu.pathTerminator = "PI";
  hitvu.legCourseDeg = 321.8f;
  plan.push_back(hitvu);
  plan.push_back(makeLeg("CUDLA", 40.050000000, -88.320000000));

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  NavigationSolution sol = nav.update(hitvu.lat + 0.02, hitvu.lon + 0.02, 90.0f);
  EXPECT_EQ(sol.toWpt, "HITVU");
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_GT(std::fabs(sol.crossTrackNm), 0.2f);

  sol = nav.update(hitvu.lat, hitvu.lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "CUDLA");
}

TEST(FmsNavigatorTest, PublishedCourseToFixUsesCourseAndSequencesAtFix) {
  std::vector<MapLeg> plan = makeKcmiDmeArcPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(2);

  double offLat = 0.0;
  double offLon = 0.0;
  navOffsetPoint(plan[2].lat, plan[2].lon, 207.0 + 180.0, 3.0, offLat, offLon);
  offLon += 0.02;
  NavigationSolution sol = nav.update(offLat, offLon, 90.0f);
  EXPECT_EQ(sol.toWpt, "STADI");
  EXPECT_NEAR(sol.desiredTrackDeg, 207.0f, 0.1f);
  EXPECT_GT(std::fabs(sol.crossTrackNm), 0.2f);

  sol = nav.update(plan[2].lat, plan[2].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "STADI");
}

TEST(FmsNavigatorTest, VectorToAltitudeLegSequencesAtAltitude) {
  MapLeg vector = makeLeg("VECT", 40.0, -88.0);
  vector.pathTerminator = "VI";
  vector.legCourseDeg = 340.0f;
  vector.altitudeConstraintFt = 2800;
  vector.altitudeConstraint = AltConstraintType::AtOrAbove;
  MapLeg lodge = makeLeg("LODGE", 40.143266667, -88.526277778);

  FmsNavigator nav;
  nav.setFlightPlan({vector, lodge});

  NavigationSolution sol = nav.update(40.01, -88.01, 90.0f, 2400.0f);
  EXPECT_EQ(nav.activeLegIndex(), 0);
  EXPECT_EQ(sol.toWpt, "LODGE");
  EXPECT_NEAR(sol.desiredTrackDeg, 340.0f, 0.1f);
  EXPECT_GT(std::fabs(sol.crossTrackNm), 0.01f);

  sol = nav.update(40.02, -88.02, 90.0f, 2850.0f);
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "LODGE");
}

TEST(FmsNavigatorTest, CombinedArrivalAndApproachSequencesAcrossJoinFix) {
  std::vector<MapLeg> plan;
  plan.push_back(makeLeg("TIGRA", 39.9, 19.4));
  plan.push_back(makeLeg("BEDEX", 39.397222222, 19.733333333));
  plan.push_back(makeLeg("FD34", 39.459761111, 19.952750000));
  plan.push_back(makeLeg("RW34", 39.601944444, 19.912222222));

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  NavigationSolution sol =
      nav.update(plan[1].lat + 0.01, plan[1].lon - 0.02, 90.0f);
  EXPECT_EQ(sol.toWpt, "BEDEX");
  EXPECT_GT(std::fabs(sol.crossTrackNm), 0.2f);

  sol = nav.update(plan[1].lat, plan[1].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "FD34");
}

TEST(FmsNavigatorTest, SuspendsAtMaptWithMissedLegsLoaded) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  nav.update(plan[0].lat - 0.02, plan[0].lon, 90.0f);
  nav.update(plan[0].lat, plan[0].lon, 90.0f);
  ASSERT_EQ(nav.activeLegIndex(), 1);

  nav.update(plan[1].lat, plan[1].lon, 90.0f);
  ASSERT_EQ(nav.activeLegIndex(), 2);

  // Capture MAPt -> suspend instead of sequencing to IBITE.
  NavigationSolution sol = nav.update(plan[2].lat, plan[2].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "RW09");
  EXPECT_TRUE(nav.missedApproachSuspended());
  EXPECT_TRUE(sol.sequencingSuspended);
  EXPECT_FALSE(nav.missedApproachActive());
}

TEST(FmsNavigatorTest, ActivateMissedApproachSequencesFirstMissedFix) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(2);

  nav.update(plan[2].lat, plan[2].lon, 90.0f);
  ASSERT_TRUE(nav.missedApproachSuspended());

  ASSERT_TRUE(nav.activateMissedApproach());
  EXPECT_FALSE(nav.missedApproachSuspended());
  EXPECT_TRUE(nav.missedApproachActive());
  EXPECT_EQ(nav.activeLegIndex(), 3);

  // Ownship south of the MAPt, tracking north toward IBITE.
  const NavigationSolution sol =
      nav.update(plan[2].lat - 0.02, plan[2].lon, 90.0f);
  EXPECT_EQ(sol.toWpt, "IBITE");
  EXPECT_EQ(sol.fromWpt, "RW09");
  EXPECT_FALSE(sol.sequencingSuspended);
}

TEST(FmsNavigatorTest, ActivateMissedApproachFromFinalApproachLeg) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  ASSERT_TRUE(nav.activateMissedApproach());
  EXPECT_EQ(nav.activeLegIndex(), 3);
  EXPECT_TRUE(nav.missedApproachActive());
  EXPECT_FALSE(nav.missedApproachSuspended());
}

TEST(FmsNavigatorTest, SequencesThroughMissedLegsAfterActivation) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(2);
  ASSERT_TRUE(nav.activateMissedApproach());

  NavigationSolution sol = nav.update(plan[3].lat, plan[3].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 4);
  EXPECT_EQ(sol.toWpt, "SERFS");
}

TEST(FmsNavigatorTest, EntersHoldAtMahpAndFliesOutbound) {
  MapLeg prior;
  prior.id = "IBITE";
  prior.lat = 26.105000;
  prior.lon = -81.200000;
  MapLeg mahp = makeRnavApproachWithMissedPlan().back();

  FmsNavigator nav;
  nav.setFlightPlan({prior, mahp});
  nav.setActiveLegIndex(1);

  // Approach from the outbound side (south of the fix, tracking north) → parallel
  // entry starts on the outbound leg (354° for this hold).
  double southLat = 0.0;
  double southLon = 0.0;
  navOffsetPoint(mahp.lat, mahp.lon, mahp.hold.inboundCourseDeg, 0.35, southLat,
                 southLon);
  NavigationSolution sol = nav.update(southLat, southLon, 90.0f);
  EXPECT_TRUE(nav.inHold());
  EXPECT_TRUE(sol.sequencingSuspended);
  EXPECT_EQ(sol.toWpt, "SERFS");
  EXPECT_NEAR(sol.desiredTrackDeg, 354.0f, 2.0f);
}

TEST(FmsNavigatorTest, FplLegDisplayRoleShowsHoldForPublishedHold) {
  MapLeg hilpt;
  hilpt.hold.active = true;
  hilpt.hold.inboundCourseDeg = 41.0f;
  EXPECT_EQ(fplLegDisplayRole(hilpt), "hold");

  MapLeg mahp = makeRnavApproachWithMissedPlan().back();
  EXPECT_EQ(fplLegDisplayRole(mahp), "mahp");
}

TEST(FmsNavigatorTest, DirectToHoldEntersPublishedHoldAtFix) {
  MapLeg cmi;
  cmi.id = "CMI";
  cmi.lat = 40.0;
  cmi.lon = -88.27;
  MapLeg bostn;
  bostn.id = "BOSTN";
  bostn.lat = 40.10;
  bostn.lon = -88.20;
  bostn.procedureRole = "hold";
  bostn.hold.active = true;
  bostn.hold.inboundCourseDeg = 41.0f;
  bostn.hold.turn = HoldTurnDirection::Right;
  bostn.hold.legLengthNm = 4.0f;
  MapLeg faf;
  faf.id = "FAF01";
  faf.lat = 40.05;
  faf.lon = -88.15;
  const std::vector<MapLeg> plan = {cmi, bostn, faf};

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.activateDirectTo(bostn, cmi.lat, cmi.lon, true, true);

  NavigationSolution enroute = nav.update(cmi.lat, cmi.lon, 116.0f);
  EXPECT_FALSE(nav.inHold());
  EXPECT_TRUE(nav.directToActive());

  NavigationSolution atFix = nav.update(bostn.lat, bostn.lon, 116.0f);
  EXPECT_FALSE(nav.directToActive());
  EXPECT_TRUE(nav.inHold());
  EXPECT_TRUE(atFix.inHold);
  EXPECT_EQ(atFix.toWpt, "BOSTN");
}

TEST(FmsNavigatorTest, ResumeFromAutoSuspendExitsHold) {
  MapLeg prior;
  prior.id = "IBITE";
  prior.lat = 26.105000;
  prior.lon = -81.200000;
  MapLeg mahp = makeRnavApproachWithMissedPlan().back();

  FmsNavigator nav;
  nav.setFlightPlan({prior, mahp});
  nav.setActiveLegIndex(1);
  nav.update(mahp.lat, mahp.lon, 90.0f);
  ASSERT_TRUE(nav.inHold());

  EXPECT_TRUE(nav.resumeFromAutoSuspend());
  EXPECT_FALSE(nav.inHold());
}

TEST(FmsNavigatorTest, HoldCyclesInboundAfterOutbound) {
  MapLeg mahp = makeRnavApproachWithMissedPlan().back();
  FmsNavigator nav;
  nav.setFlightPlan({mahp});
  nav.setActiveLegIndex(0);
  NavigationSolution sol = nav.update(mahp.lat, mahp.lon, 90.0f);
  ASSERT_TRUE(nav.inHold());
  const float outbound = normalizeHeadingDeg(mahp.hold.inboundCourseDeg + 180.0f);
  EXPECT_NEAR(sol.desiredTrackDeg, outbound, 2.0f);

  // Fly the outbound leg; the navigator should stay in the hold and continue
  // issuing guidance (outbound, turn, or inbound depending on along-track).
  double northLat = 0.0;
  double northLon = 0.0;
  navOffsetPoint(mahp.lat, mahp.lon, outbound, 3.85, northLat, northLon);
  sol = nav.update(northLat, northLon, 90.0f);
  EXPECT_TRUE(nav.inHold());
  EXPECT_TRUE(sol.active);
}

MapLeg makeCourseReversalIaf() {
  MapLeg iaf;
  iaf.id = "BOSTN";
  iaf.lat = 40.207791667;
  iaf.lon = -88.145527778;
  iaf.procedureRole = "iaf";
  iaf.hold.active = true;
  iaf.hold.courseReversal = true;
  iaf.hold.inboundCourseDeg = 44.0f;
  iaf.hold.legLengthNm = 4.0f;
  iaf.hold.turn = HoldTurnDirection::Right;
  return iaf;
}

void flyHoldRacetrackCircuit(FmsNavigator& nav, const MapLeg& leg, float gs) {
  const HoldRacetrackGeom geom = buildHoldRacetrack(leg, gs);
  ASSERT_TRUE(geom.valid);
  const auto step = [&](double lat, double lon, int frames = 4) {
    for (int i = 0; i < frames; ++i) {
      nav.update(lat, lon, gs);
    }
  };
  step(leg.lat, leg.lon);
  step(geom.outboundParLat, geom.outboundParLon, 6);
  step(geom.outboundEndLat, geom.outboundEndLon, 6);
  step(geom.fixLat, geom.fixLon, 6);
  step(geom.inboundParLat, geom.inboundParLon, 10);
}

TEST(FmsNavigatorTest, CourseReversalSequencesAfterOneCircuit) {
  MapLeg prior = makeLeg("CMI", 40.034530556, -88.276075000);
  MapLeg bostn = makeCourseReversalIaf();
  MapLeg faf = makeLeg("AFTOR", 40.121183333, -88.210869444);
  faf.procedureRole = "faf";

  FmsNavigator nav;
  nav.setFlightPlan({prior, bostn, faf});
  nav.setActiveLegIndex(1);

  flyHoldRacetrackCircuit(nav, bostn, 116.0f);
  EXPECT_FALSE(nav.inHold());
  EXPECT_EQ(nav.activeLegIndex(), 2);
  const NavigationSolution sol =
      nav.update(faf.lat, faf.lon, 116.0f);
  EXPECT_EQ(sol.toWpt, "AFTOR");
}

TEST(FmsNavigatorTest, MissedApproachHoldStillLoops) {
  MapLeg mahp = makeRnavApproachWithMissedPlan().back();
  ASSERT_FALSE(mahp.hold.courseReversal);

  FmsNavigator nav;
  nav.setFlightPlan({mahp});
  nav.setActiveLegIndex(0);
  flyHoldRacetrackCircuit(nav, mahp, 90.0f);
  EXPECT_TRUE(nav.inHold());
}

TEST(FmsNavigatorTest, RightHandHoldLiesOnInboundRightSide) {
  // SERFS: inbound 174deg, right-hand hold -> pattern on the west side (NW),
  // matching the published chart. Build the racetrack and confirm every turn
  // vertex is west of (lower longitude than) the fix.
  const MapLeg mahp = makeRnavApproachWithMissedPlan().back();
  const HoldRacetrackGeom geom = buildHoldRacetrack(mahp, 90.0f);
  ASSERT_TRUE(geom.valid);
  EXPECT_LT(geom.inboundParLon, geom.fixLon);
  EXPECT_LT(geom.outboundParLon, geom.fixLon);
  EXPECT_LT(geom.turn1CenterLon, geom.fixLon);
  EXPECT_LT(geom.turn2CenterLon, geom.fixLon);
}

TEST(FmsNavigatorTest, LegSolutionUsesTrackCrossTrack) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  // Between IAF and FAF, offset east of the meridian.
  const NavigationSolution sol =
      nav.update(plan[0].lat + 0.025, plan[0].lon + 0.01, 90.0f);
  EXPECT_TRUE(sol.active);
  EXPECT_EQ(sol.fromWpt, "IAF01");
  EXPECT_EQ(sol.toWpt, "FAF01");
  EXPECT_NEAR(sol.desiredTrackDeg, 0.0f, 1.0f);
  EXPECT_GT(std::fabs(sol.crossTrackNm), 0.01f);
}

TEST(FmsNavigatorTest, DirectToAirportSequencesIntoApproach) {
  const std::vector<MapLeg> plan = makeAirportPlusApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  nav.activateDirectTo(plan[0], 26.90, -81.99, true);
  NavigationSolution sol = nav.update(26.85, -81.99, 90.0f);
  EXPECT_TRUE(sol.directTo);
  EXPECT_EQ(sol.toWpt, "KPGD");
  EXPECT_TRUE(nav.directToActive());

  // Arrive at the airport; resume the loaded approach at IAF01.
  sol = nav.update(plan[0].lat, plan[0].lon, 90.0f);
  EXPECT_FALSE(nav.directToActive());
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "IAF01");
  EXPECT_FALSE(sol.directTo);
  EXPECT_EQ(sol.fromWpt, "KPGD");
}

TEST(FmsNavigatorTest, StandaloneDirectToDoesNotAdvancePlan) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  MapLeg fix;
  fix.id = "RANDOM";
  fix.lat = 26.030000;
  fix.lon = -81.150000;
  nav.activateDirectTo(fix, 26.0, -81.15, true);

  NavigationSolution sol = nav.update(fix.lat - 0.02, fix.lon, 90.0f);
  EXPECT_TRUE(nav.directToActive());
  EXPECT_EQ(sol.toWpt, "RANDOM");
  EXPECT_TRUE(sol.directTo);
  EXPECT_EQ(nav.activeLegIndex(), 0);
}

TEST(FmsNavigatorTest, ObsModeDoesNotSequence) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setObsMode(true);

  NavigationSolution sol = nav.update(plan[0].lat, plan[0].lon, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 0);
  EXPECT_EQ(sol.toWpt, "IAF01");
}

TEST(FmsNavigatorTest, SetActiveLegIndexSelectsLeg) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(2);

  const NavigationSolution sol = nav.update(26.08, -81.2, 90.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "RW09");
  EXPECT_EQ(sol.fromWpt, "FAF01");
  EXPECT_FALSE(nav.directToActive());
}

TEST(FmsNavigatorTest, ClearDirectToResumesPlanLeg) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  MapLeg fix;
  fix.id = "TEMP";
  fix.lat = 26.02;
  fix.lon = -81.2;
  nav.activateDirectTo(fix, 26.0, -81.2, true);
  nav.clearDirectTo();

  const NavigationSolution sol = nav.update(26.01, -81.2, 90.0f);
  EXPECT_FALSE(sol.directTo);
  EXPECT_EQ(sol.toWpt, "IAF01");
}

TEST(FmsNavigatorTest, StandaloneDirectToWithEmptyPlan) {
  FmsNavigator nav;
  MapLeg fix;
  fix.id = "RANDOM";
  fix.lat = 26.030000;
  fix.lon = -81.150000;
  nav.activateDirectTo(fix, 26.0, -81.15, true);

  const NavigationSolution sol = nav.update(26.01, -81.15, 90.0f);
  EXPECT_TRUE(sol.active);
  EXPECT_EQ(sol.toWpt, "RANDOM");
  EXPECT_TRUE(sol.directTo);
}

TEST(FmsNavigatorTest, DirectToApproachFixUpdatesActiveLeg) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(0);

  // Direct-To the FAF while still en route to the IAF.
  MapLeg fafTarget = plan[1];
  fafTarget.id = "DIFFERENT";  // nav-database id may differ from plan id
  nav.activateDirectTo(fafTarget, 26.0, -81.2, true);

  const NavigationSolution sol = nav.update(26.0, -81.2, 90.0f);
  EXPECT_TRUE(sol.directTo);
  EXPECT_EQ(sol.toWpt, "DIFFERENT");
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.activeLegIndex, 1);
}

TEST(FmsNavigatorTest, DirectToReactivationUpdatesOrigin) {
  MapLeg fix;
  fix.id = "FAF01";
  fix.lat = 26.050000;
  fix.lon = -81.200000;

  FmsNavigator nav;
  nav.activateDirectTo(fix, 26.000000, -81.200000, true);
  NavigationSolution sol =
      nav.update(26.010000, -81.190000, 90.0f);
  const float courseFromSouth = sol.desiredTrackDeg;

  nav.activateDirectTo(fix, 26.500000, -81.000000, true);
  sol = nav.update(26.510000, -81.000000, 90.0f);
  const float courseFromNorthwest = sol.desiredTrackDeg;

  EXPECT_NEAR(courseFromSouth, 0.0f, 5.0f);
  EXPECT_GT(courseFromNorthwest, 90.0f);
  EXPECT_NE(courseFromSouth, courseFromNorthwest);
}

TEST(FmsNavigatorTest, DirectToApproachFixCaptureResumesLegNavigation) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);

  MapLeg faf = plan[1];
  nav.activateDirectTo(faf, plan[0].lat, plan[0].lon, true);
  NavigationSolution sol = nav.update(plan[0].lat + 0.02, plan[0].lon, 90.0f);
  EXPECT_TRUE(sol.directTo);
  EXPECT_EQ(sol.toWpt, "FAF01");

  sol = nav.update(faf.lat, faf.lon, 90.0f);
  EXPECT_FALSE(nav.directToActive());
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "RW09");
  EXPECT_EQ(sol.fromWpt, "FAF01");
  EXPECT_FALSE(sol.directTo);

  FlightData data;
  applyNavigationSolution(data, sol, 0.3f);
  EXPECT_EQ(data.fmaFromWpt, "FAF01");
  EXPECT_EQ(data.fmaToWpt, "RW09");
  EXPECT_EQ(data.fmaActiveLegIndex, 2);
}

TEST(FmsNavigatorTest, ApplyNavigationSolutionUpdatesFlightData) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  const NavigationSolution sol =
      nav.update(plan[0].lat + 0.025, plan[0].lon, 90.0f);
  FlightData data;
  applyNavigationSolution(data, sol, 0.3f);

  EXPECT_EQ(data.fmaFromWpt, "IAF01");
  EXPECT_EQ(data.fmaToWpt, "FAF01");
  EXPECT_NEAR(data.courseDeg, sol.desiredTrackDeg, 0.01f);
  EXPECT_TRUE(data.navSignalValid);
}

TEST(FmsNavigatorTest, KfmyApproachActivateLegPointsTowardCitag) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;

  constexpr double kKfmyLat = 26.586111;
  constexpr double kKfmyLon = -81.775278;

  const std::vector<MapLeg> plan = {citag, butly};
  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(0);

  const NavigationSolution sol = nav.update(kKfmyLat, kKfmyLon, 90.0f);
  const double expectedBrg =
      navBearingDeg(kKfmyLat, kKfmyLon, citag.lat, citag.lon);

  EXPECT_TRUE(sol.active);
  EXPECT_EQ(sol.toWpt, "CITAG");
  EXPECT_FALSE(sol.directTo);
  EXPECT_NEAR(sol.desiredTrackDeg, static_cast<float>(expectedBrg), 2.0f);
  EXPECT_GT(sol.desiredTrackDeg, 90.0f);
  EXPECT_LT(sol.desiredTrackDeg, 270.0f);
}

TEST(FmsNavigatorTest, DirectToTargetLegIndexPrefersToIdentOverAirportIndex) {
  MapLeg kfmy;
  kfmy.id = "KFMY";
  MapLeg citag;
  citag.id = "CITAG";
  citag.procedureRole = "iaf";
  MapLeg butly;
  butly.id = "BUTLY";
  butly.procedureRole = "iaf";
  const std::vector<MapLeg> plan = {kfmy, citag, butly};

  FlightData d;
  d.fmaFromWpt.clear();
  d.fmaToWpt = "CITAG";
  d.fmaActiveLegIndex = 0;  // stale airport index

  EXPECT_EQ(fplDirectToTargetLegIndex(plan, d, d.fmaToWpt, true), 1);
  EXPECT_EQ(fplResolvedActiveLegIndex(plan, d, d.fmaToWpt), 1);
}

TEST(FmsNavigatorTest, ShowActiveNavRowMatchesToIdentRegardlessOfLegIndex) {
  MapLeg kfmy;
  kfmy.id = "KFMY";
  MapLeg citag;
  citag.id = "CITAG";
  citag.procedureRole = "iaf";
  MapLeg butly;
  butly.id = "BUTLY";
  butly.procedureRole = "iaf";
  const std::vector<MapLeg> plan = {kfmy, citag, butly};

  EXPECT_TRUE(fplShowActiveNavRow(false, 1, 0, citag, "CITAG"));
  EXPECT_FALSE(fplShowActiveNavRow(false, 2, 0, butly, "CITAG"));
}

TEST(FmsNavigatorTest, ActiveNavRowBlinkFollowsCursorNotPinnedNavTarget) {
  EXPECT_FALSE(fplActiveNavRowBlink(1, 1, 2, true, 1, 0));
  EXPECT_TRUE(fplActiveNavRowBlink(1, 1, 1, true, 1, 0));
  EXPECT_TRUE(fplActiveNavRowBlink(1, 1, 1, false, 0, 0));
}

TEST(FmsNavigatorTest, TurnLeadDistanceCapsSharpTurns) {
  const double moderate = turnLeadDistanceNm(116.0, 45.0);
  const double sharp = turnLeadDistanceNm(116.0, 179.0);
  const double capped = turnLeadDistanceNm(116.0, 90.0);
  EXPECT_GT(moderate, 0.0);
  EXPECT_NEAR(sharp, capped, 0.01);
  EXPECT_LT(sharp, 5.0);
}

TEST(FmsNavigatorTest, TurnLeadBankDegReducesLeadAtHigherBank) {
  const double piston = turnLeadDistanceNm(250.0, 45.0, 90.0, 15.0);
  const double jet = turnLeadDistanceNm(250.0, 45.0, 90.0, 25.0);
  EXPECT_GT(piston, jet);
  EXPECT_NEAR(resolveTurnLeadBankDeg("SF50", ""), 25.0, 0.01);
  EXPECT_NEAR(resolveTurnLeadBankDeg("C172", ""), 15.0, 0.01);
}

TEST(FmsNavigatorTest, TurnAnticipationSharpTurnShowsCountdownBeforeFlip) {
  MapLeg serfs;
  serfs.id = "SERFS";
  serfs.lat = 26.800700000;
  serfs.lon = -81.819600000;
  MapLeg pints;
  pints.id = "PINTS";
  pints.lat = 26.802891667;
  pints.lon = -82.135858333;
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  const std::vector<MapLeg> plan = {serfs, pints, azomy};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = pints.lat + 0.01;
  map.ownshipLon = pints.lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 122.0f;
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "SERFS";
  data.fmaToWpt = "PINTS";
  data.fmaLegDistanceNm = 1.0f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find("in "), std::string::npos);
  EXPECT_NE(ta.message.find("seconds"), std::string::npos);
  EXPECT_FALSE(ta.flashing);
}

TEST(FmsNavigatorTest, FlyByTurnSequencesPintsToAzomy) {
  MapLeg prior;
  prior.id = "FIX01";
  prior.lat = 26.802891667;
  prior.lon = -82.22;
  MapLeg pints;
  pints.id = "PINTS";
  pints.lat = 26.802891667;
  pints.lon = -82.135858333;
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  const std::vector<MapLeg> plan = {prior, pints, azomy};

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  NavigationSolution sol =
      nav.update(prior.lat, prior.lon + 0.02, 116.0f);
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "PINTS");

  const double gsKts = 116.0;
  const double inboundDeg =
      navBearingDeg(prior.lat, prior.lon, pints.lat, pints.lon);
  const double outboundDeg =
      navBearingDeg(pints.lat, pints.lon, azomy.lat, azomy.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta);
  ASSERT_GT(leadNm, 0.4);

  double flyByLat = 0.0;
  double flyByLon = 0.0;
  navOffsetPoint(pints.lat, pints.lon, inboundDeg + 180.0, leadNm * 0.5,
                 flyByLat, flyByLon);

  sol = nav.update(flyByLat, flyByLon, static_cast<float>(gsKts));
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "AZOMY");
  EXPECT_EQ(sol.fromWpt, "PINTS");
  EXPECT_FALSE(sol.directTo);
}

// A ~91° turn (just over the fly-over threshold) must still sequence: the turn
// anticipation / override steering rounds the corner using the 90°-capped lead,
// so the aircraft cuts inside the fix and never reaches a small capture radius.
// Sequencing the leg at the same lead distance the steering uses keeps it from
// getting stuck on the inbound leg (regression: UZAWO->GRAMS on the KFMY arrival
// stayed on UZAWO while the aircraft flew on past toward GRAMS).
TEST(FmsNavigatorTest, NinetyDegreeTurnSequencesAtLeadNotStuck) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.60;
  azomy.lon = -82.10;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.45;
  uzawo.lon = -81.95;
  // GRAMS roughly 90° left of the AZOMY->UZAWO inbound course.
  MapLeg grams;
  grams.id = "GRAMS";
  grams.lat = 26.58;
  grams.lon = -81.80;
  const std::vector<MapLeg> plan = {azomy, uzawo, grams};

  const double gsKts = 119.0;
  const double inboundDeg = navBearingDeg(azomy.lat, azomy.lon, uzawo.lat,
                                          uzawo.lon);
  const double outboundDeg = navBearingDeg(uzawo.lat, uzawo.lon, grams.lat,
                                           grams.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  ASSERT_GE(std::fabs(turnDelta), 90.0);  // the fly-over-threshold case
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta);
  ASSERT_GT(leadNm, 0.4);  // lead is wider than the 0.4 nm capture radius

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  // Still well outside the lead: stays on UZAWO.
  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(uzawo.lat, uzawo.lon, inboundDeg + 180.0, leadNm + 0.3, lat,
                 lon);
  NavigationSolution sol = nav.update(lat, lon, static_cast<float>(gsKts));
  ASSERT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "UZAWO");

  // Reaching the lead point sequences to GRAMS (the turn begins here).
  navOffsetPoint(uzawo.lat, uzawo.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);
  sol = nav.update(lat, lon, static_cast<float>(gsKts));
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "GRAMS");
}

// A moderate fly-by whose lead point sits inside the fixed capture radius must
// not sequence at the radius: doing so would flip the leg before the turn
// anticipation countdown ("Turn ... in N seconds") could run, so the pilot only
// ever sees a brief flash. The leg must hold until the lead point so the
// countdown reaches "now" (G1000 NXi Pilot's Guide Appendix D).
TEST(FmsNavigatorTest, ModerateFlyByHoldsLegSoCountdownIsVisible) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {citag, butly, uzawo};

  const double gsKts = 116.0;
  const double inboundDeg =
      navBearingDeg(citag.lat, citag.lon, butly.lat, butly.lon);
  const double leadNm = turnLeadDistanceNm(
      gsKts,
      shortestTurnDeltaDeg(
          inboundDeg, navBearingDeg(butly.lat, butly.lon, uzawo.lat, uzawo.lon)));
  ASSERT_GT(leadNm, 0.0);
  ASSERT_LT(leadNm, 0.35);  // lead point is inside the fixed capture radius

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);

  // 0.35 nm before BUTLY: inside the old 0.4 nm capture but short of the lead
  // point. The leg must stay on BUTLY and the advisory must still be counting.
  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(butly.lat, butly.lon, inboundDeg + 180.0, 0.35, lat, lon);
  NavigationSolution sol = nav.update(lat, lon, static_cast<float>(gsKts));
  ASSERT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "BUTLY");

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = lat;
  map.ownshipLon = lon;
  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "CITAG";
  data.fmaToWpt = "BUTLY";
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, butly.lat, butly.lon));

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find("in "), std::string::npos);
  EXPECT_NE(ta.message.find("seconds"), std::string::npos);
  EXPECT_EQ(ta.message.find("now"), std::string::npos);

  // Reaching the lead point sequences the leg (the countdown hits "now").
  navOffsetPoint(butly.lat, butly.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);
  sol = nav.update(lat, lon, static_cast<float>(gsKts));
  EXPECT_EQ(nav.activeLegIndex(), 2);
  EXPECT_EQ(sol.toWpt, "UZAWO");
}

TEST(FmsNavigatorTest, FlyByTurnAtButlySteersRightWithoutLeftCdi) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {citag, butly, uzawo};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = citag.lat + 0.85 * (butly.lat - citag.lat);
  map.ownshipLon = citag.lon + 0.85 * (butly.lon - citag.lon);

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "CITAG";
  data.fmaToWpt = "BUTLY";
  data.fmaLegDistanceNm = 0.05f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find(" now"), std::string::npos);
  EXPECT_NE(ta.message.find("318"), std::string::npos);

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(1);
  NavigationSolution sol =
      nav.update(map.ownshipLat, map.ownshipLon, data.groundSpeedKts);
  sol = applyFlyByTurnCourse(sol, map, data, false, CdiSource::Gps, 0.5f);

  EXPECT_NEAR(sol.desiredTrackDeg, 318.0f, 1.0f);
  EXPECT_NEAR(sol.crossTrackNm, 0.0f, 0.01f);
}

// Regression: on a sharp fly-by the navigator sequences onto the outbound leg
// at the lead point, so the corner-rounding "… now" branch no longer fires and
// the raw outbound cross-track jumps to ~lead·sin(turnΔ) at once. On a steep GPS
// sensitivity (approach) that pegs the injected CDI full scale and X-Plane drops
// NAV to ROLL. applyFlyByTurnCourse must clamp the cross-track while the aircraft
// is still completing the turn so the needle never saturates.
TEST(FmsNavigatorTest, SharpFlyByClampsOutboundCdiSoNavHolds) {
  MapLeg priorFix;
  priorFix.id = "PRIORA";
  MapLeg turnFix;
  turnFix.id = "TURNB";
  turnFix.lat = 26.5;
  turnFix.lon = -82.0;
  MapLeg outFix;
  outFix.id = "OUTC";

  // Inbound PRIORA->TURNB tracks 090; outbound TURNB->OUTC tracks 210 -> a 120°
  // right turn (the kind of sharp fly-by that drops NAV).
  navOffsetPoint(turnFix.lat, turnFix.lon, 270.0, 3.0, priorFix.lat,
                 priorFix.lon);
  navOffsetPoint(turnFix.lat, turnFix.lon, 210.0, 3.0, outFix.lat, outFix.lon);
  const std::vector<MapLeg> plan = {priorFix, turnFix, outFix};

  const double gsKts = 120.0;
  const double inboundDeg =
      navBearingDeg(priorFix.lat, priorFix.lon, turnFix.lat, turnFix.lon);
  const double outboundDeg =
      navBearingDeg(turnFix.lat, turnFix.lon, outFix.lat, outFix.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  ASSERT_GE(std::fabs(turnDelta), 90.0);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta);
  ASSERT_GT(leadNm, 0.4);

  // Just sequenced: the aircraft is one lead distance before the fix on the
  // inbound course, already on the outbound leg.
  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(turnFix.lat, turnFix.lon, inboundDeg + 180.0, leadNm, lat, lon);

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(2);  // outbound leg TURNB->OUTC
  NavigationSolution sol = nav.update(lat, lon, static_cast<float>(gsKts));
  ASSERT_EQ(nav.activeLegIndex(), 2);
  ASSERT_EQ(sol.toWpt, "OUTC");

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = lat;
  map.ownshipLon = lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaActiveLegIndex = 2;
  data.fmaFromWpt = "TURNB";
  data.fmaToWpt = "OUTC";
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, outFix.lat, outFix.lon));

  // Approach sensitivity: 0.15 nm/dot -> the raw offset would peg full scale.
  constexpr float kApproachNmPerDot = 0.15f;
  const float rawXtkNm = sol.crossTrackNm;
  ASSERT_GT(std::fabs(rawXtkNm) / kApproachNmPerDot, 2.5f);

  sol = applyFlyByTurnCourse(sol, map, data, false, CdiSource::Gps,
                             kApproachNmPerDot);

  // Course still tracks the outbound leg; only the cross-track is bounded.
  EXPECT_NEAR(sol.desiredTrackDeg, static_cast<float>(outboundDeg), 1.0f);
  EXPECT_LT(std::fabs(sol.crossTrackNm), std::fabs(rawXtkNm));

  FlightData applied;
  applyNavigationSolution(applied, sol, kApproachNmPerDot);
  EXPECT_LT(std::fabs(applied.cdiDeviationDots), 2.5f);
}

TEST(FmsNavigatorTest, TurnAnticipationShowsCountdownBeforeTurn) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {citag, butly, uzawo};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "CITAG";
  data.fmaToWpt = "BUTLY";
  data.fmaLegDistanceNm = 0.35f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find("in "), std::string::npos);
  EXPECT_NE(ta.message.find("seconds"), std::string::npos);
  EXPECT_NE(ta.message.find("318"), std::string::npos);
  EXPECT_FALSE(ta.flashing);
}

// HILPT / holding-pattern fixes are flown over (the hold entry reverses course),
// so turn anticipation must not lead the turn onto the post-hold leg. Otherwise
// the autopilot swings toward the post-hold inbound course before the fix
// instead of entering the hold (KCMI RNAV 04 CMI->BOSTN turned toward 041 for a
// few seconds before the hold took over).
TEST(FmsNavigatorTest, TurnAnticipationSuppressedAtHoldFix) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  butly.hold.active = true;
  butly.hold.turn = HoldTurnDirection::Right;
  butly.hold.inboundCourseDeg = 41.0f;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {citag, butly, uzawo};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "CITAG";
  data.fmaToWpt = "BUTLY";
  data.fmaLegDistanceNm = 0.35f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  EXPECT_FALSE(ta.active);
}

TEST(FmsNavigatorTest, TurnAnticipationWorksOnFirstFlightPlanLeg) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {citag, butly, uzawo};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.directToActive = false;
  map.ownshipLat = citag.lat - 0.02;
  map.ownshipLon = citag.lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaActiveLegIndex = 0;
  data.fmaFromWpt.clear();
  data.fmaToWpt = "CITAG";
  data.fmaLegDistanceNm = 0.05f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find(" now"), std::string::npos);
}

TEST(FmsNavigatorTest, TurnAnticipationSkippedForDirectToOffPlan) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;
  MapLeg butly;
  butly.id = "BUTLY";
  butly.lat = 26.351555556;
  butly.lon = -81.960486111;
  const std::vector<MapLeg> plan = {citag, butly};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.directToActive = true;
  map.directTo = butly;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaFromWpt.clear();
  data.fmaToWpt = "BUTLY";
  data.fmaLegDistanceNm = 1.0f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  EXPECT_FALSE(ta.active);
}

TEST(FmsNavigatorTest, TurnAnticipationCountdownDuringDirectToOnPlan) {
  MapLeg prior;
  prior.id = "FIX01";
  prior.lat = 26.802891667;
  prior.lon = -82.22;
  MapLeg pints;
  pints.id = "PINTS";
  pints.lat = 26.802891667;
  pints.lon = -82.135858333;
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  const std::vector<MapLeg> plan = {prior, pints, azomy};

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.directToActive = true;
  map.directTo = pints;
  map.directToOriginValid = true;
  map.directToOriginLat = prior.lat;
  map.directToOriginLon = prior.lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 116.0f;
  data.fmaFromWpt.clear();
  data.fmaToWpt = "PINTS";
  data.fmaActiveLegIndex = 1;
  data.fmaLegDistanceNm = 0.35f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_TRUE(ta.message.find("in ") != std::string::npos ||
              ta.message.find(" now") != std::string::npos);
}

TEST(FmsNavigatorTest, FlyByTurnSteersOutboundDuringDirectToAtLead) {
  MapLeg prior;
  prior.id = "FIX01";
  prior.lat = 26.802891667;
  prior.lon = -82.22;
  MapLeg pints;
  pints.id = "PINTS";
  pints.lat = 26.802891667;
  pints.lon = -82.135858333;
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  const std::vector<MapLeg> plan = {prior, pints, azomy};

  const double gsKts = 116.0;
  const double inboundDeg =
      navBearingDeg(prior.lat, prior.lon, pints.lat, pints.lon);
  const double outboundDeg =
      navBearingDeg(pints.lat, pints.lon, azomy.lat, azomy.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta, 135.0);

  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(pints.lat, pints.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.directToActive = true;
  map.directTo = pints;
  map.directToOriginValid = true;
  map.directToOriginLat = prior.lat;
  map.directToOriginLon = prior.lon;
  map.ownshipLat = lat;
  map.ownshipLon = lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaFromWpt.clear();
  data.fmaToWpt = "PINTS";
  data.fmaActiveLegIndex = 1;
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, pints.lat, pints.lon));

  NavigationSolution sol;
  sol.active = true;
  sol.directTo = true;
  sol.activeLegIndex = 1;
  sol.toWpt = "PINTS";
  sol.desiredTrackDeg = static_cast<float>(inboundDeg);
  sol.crossTrackNm = 0.0f;

  sol = applyFlyByTurnCourse(sol, map, data, false, CdiSource::Gps, 0.5f);

  EXPECT_TRUE(sol.directTo);
  EXPECT_NEAR(sol.desiredTrackDeg, static_cast<float>(outboundDeg), 1.0f);
  EXPECT_NEAR(sol.crossTrackNm, 0.0f, 0.01f);
  EXPECT_NE(sol.desiredTrackDeg, static_cast<float>(inboundDeg));
}

TEST(FmsNavigatorTest, DirectToWithOriginAtTargetUsesOwnshipBearing) {
  MapLeg citag;
  citag.id = "CITAG";
  citag.lat = 26.237036111;
  citag.lon = -81.779541667;

  constexpr double kKfmyLat = 26.586111;
  constexpr double kKfmyLon = -81.775278;

  FmsNavigator nav;
  nav.activateDirectTo(citag, citag.lat, citag.lon, true);
  const NavigationSolution sol = nav.update(kKfmyLat, kKfmyLon, 90.0f);
  const double expectedBrg =
      navBearingDeg(kKfmyLat, kKfmyLon, citag.lat, citag.lon);

  EXPECT_TRUE(sol.directTo);
  EXPECT_NEAR(sol.desiredTrackDeg, static_cast<float>(expectedBrg), 2.0f);
  EXPECT_GT(sol.desiredTrackDeg, 90.0f);
  EXPECT_LT(sol.desiredTrackDeg, 270.0f);
}

TEST(FmsNavigatorTest, DirectToAfterHoldResumesApproachAtNextFix) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  azomy.procedureRole = "iaf";
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  MapLeg mapt;
  mapt.id = "RW05";
  mapt.lat = 26.350000;
  mapt.lon = -81.950000;
  mapt.procedureRole = "mapt";
  MapLeg missed1;
  missed1.id = "IBITE";
  missed1.lat = 26.105000;
  missed1.lon = -81.200000;
  MapLeg serfs;
  serfs.id = "SERFS";
  serfs.lat = 26.800700000;
  serfs.lon = -81.819600000;
  serfs.procedureRole = "mahp";
  serfs.hold.active = true;
  serfs.hold.inboundCourseDeg = 174.0f;
  serfs.hold.legLengthNm = 4.0f;
  serfs.hold.turn = HoldTurnDirection::Right;
  const std::vector<MapLeg> plan = {azomy, uzawo, mapt, missed1, serfs};

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.setActiveLegIndex(4);
  nav.update(serfs.lat, serfs.lon, 90.0f);
  ASSERT_TRUE(nav.inHold());
  ASSERT_TRUE(nav.missedApproachActive());

  nav.activateDirectTo(azomy, serfs.lat, serfs.lon, true);
  EXPECT_FALSE(nav.inHold());
  EXPECT_FALSE(nav.missedApproachActive());

  NavigationSolution sol = nav.update(azomy.lat, azomy.lon, 116.0f);
  EXPECT_FALSE(nav.directToActive());
  EXPECT_FALSE(nav.inHold());
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "UZAWO");
  EXPECT_FALSE(sol.inHold);
}

TEST(FmsNavigatorTest, TurnAnticipationSuppressedWhileCompletingPriorFlyBy) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.60;
  azomy.lon = -82.10;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.45;
  uzawo.lon = -81.95;
  MapLeg grams;
  grams.id = "GRAMS";
  grams.lat = 26.58;
  grams.lon = -81.80;
  const std::vector<MapLeg> plan = {azomy, uzawo, grams};

  const double gsKts = 119.0;
  const double inboundDeg = navBearingDeg(azomy.lat, azomy.lon, uzawo.lat,
                                          uzawo.lon);
  const double outboundDeg = navBearingDeg(uzawo.lat, uzawo.lon, grams.lat,
                                           grams.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta);
  ASSERT_GT(leadNm, 0.4);

  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(uzawo.lat, uzawo.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = lat;
  map.ownshipLon = lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaActiveLegIndex = 2;
  data.fmaFromWpt = "UZAWO";
  data.fmaToWpt = "GRAMS";
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, grams.lat, grams.lon));

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  EXPECT_FALSE(ta.active);
}

TEST(FmsNavigatorTest, DirectToAzomyCountdownAlignsWithSteeringStart) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {azomy, uzawo};

  constexpr double kEastLat = 26.519147222;
  constexpr double kEastLon = -82.050000;
  const double gsKts = 116.0;
  const double inboundDeg =
      navBearingDeg(kEastLat, kEastLon, azomy.lat, azomy.lon);
  const double outboundDeg =
      navBearingDeg(azomy.lat, azomy.lon, uzawo.lat, uzawo.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm =
      turnLeadDistanceNm(gsKts, turnDelta, kDirectToFlyByMaxTurnDegCap);
  const double steerNm = leadNm + kTurnSteeringMarginNm;

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.directToActive = true;
  map.directTo = azomy;
  map.directToOriginValid = true;
  map.directToOriginLat = kEastLat;
  map.directToOriginLon = kEastLon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaFromWpt.clear();
  data.fmaToWpt = "AZOMY";
  data.fmaActiveLegIndex = 0;

  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(azomy.lat, azomy.lon, inboundDeg + 180.0, steerNm + 0.15, lat,
                 lon);
  map.ownshipLat = lat;
  map.ownshipLon = lon;
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, azomy.lat, azomy.lon));

  TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find("in "), std::string::npos);
  EXPECT_EQ(ta.message.find("now"), std::string::npos);

  navOffsetPoint(azomy.lat, azomy.lon, inboundDeg + 180.0, steerNm, lat, lon);
  map.ownshipLat = lat;
  map.ownshipLon = lon;
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, azomy.lat, azomy.lon));

  ta = computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find(" now"), std::string::npos);
}

TEST(FmsNavigatorTest, DirectToAzomyFromEastSequencesAtFlyByLead) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {azomy, uzawo};

  constexpr double kEastLat = 26.519147222;
  constexpr double kEastLon = -82.050000;
  const double gsKts = 116.0;

  const double inboundDeg =
      navBearingDeg(kEastLat, kEastLon, azomy.lat, azomy.lon);
  const double outboundDeg =
      navBearingDeg(azomy.lat, azomy.lon, uzawo.lat, uzawo.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta, 135.0);
  ASSERT_GT(leadNm, 0.4);

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.activateDirectTo(azomy, kEastLat, kEastLon, true);

  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(azomy.lat, azomy.lon, inboundDeg + 180.0, leadNm + 0.3, lat,
                 lon);
  NavigationSolution sol = nav.update(lat, lon, static_cast<float>(gsKts));
  EXPECT_TRUE(nav.directToActive());
  EXPECT_EQ(sol.toWpt, "AZOMY");

  navOffsetPoint(azomy.lat, azomy.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);
  const double distNm = navDistanceNm(lat, lon, azomy.lat, azomy.lon);
  ASSERT_GT(distNm, 0.4);
  sol = nav.update(lat, lon, static_cast<float>(gsKts));
  EXPECT_FALSE(nav.directToActive());
  EXPECT_EQ(nav.activeLegIndex(), 1);
  EXPECT_EQ(sol.toWpt, "UZAWO");
  EXPECT_FALSE(sol.directTo);
}

TEST(FmsNavigatorTest, DirectToAzomyFromEastClampsCdiOnFirstOutboundLeg) {
  MapLeg azomy;
  azomy.id = "AZOMY";
  azomy.lat = 26.519147222;
  azomy.lon = -82.129402778;
  MapLeg uzawo;
  uzawo.id = "UZAWO";
  uzawo.lat = 26.436994444;
  uzawo.lon = -82.046516667;
  const std::vector<MapLeg> plan = {azomy, uzawo};

  constexpr double kEastLat = 26.519147222;
  constexpr double kEastLon = -82.050000;
  const double gsKts = 116.0;
  const double inboundDeg =
      navBearingDeg(kEastLat, kEastLon, azomy.lat, azomy.lon);
  const double outboundDeg =
      navBearingDeg(azomy.lat, azomy.lon, uzawo.lat, uzawo.lon);
  const double turnDelta = shortestTurnDeltaDeg(inboundDeg, outboundDeg);
  const double leadNm = turnLeadDistanceNm(gsKts, turnDelta, 135.0);

  FmsNavigator nav;
  nav.setFlightPlan(plan);
  nav.activateDirectTo(azomy, kEastLat, kEastLon, true);

  double lat = 0.0;
  double lon = 0.0;
  navOffsetPoint(azomy.lat, azomy.lon, inboundDeg + 180.0, leadNm * 0.5, lat,
                 lon);
  NavigationSolution sol = nav.update(lat, lon, static_cast<float>(gsKts));
  ASSERT_EQ(nav.activeLegIndex(), 1);
  ASSERT_EQ(sol.toWpt, "UZAWO");

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = lat;
  map.ownshipLon = lon;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = static_cast<float>(gsKts);
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "AZOMY";
  data.fmaToWpt = "UZAWO";
  data.fmaLegDistanceNm =
      static_cast<float>(navDistanceNm(lat, lon, uzawo.lat, uzawo.lon));

  constexpr float kApproachNmPerDot = 0.15f;
  const float rawXtkNm = sol.crossTrackNm;
  ASSERT_GT(std::fabs(rawXtkNm) / kApproachNmPerDot, 2.5f);

  sol = applyFlyByTurnCourse(sol, map, data, false, CdiSource::Gps,
                             kApproachNmPerDot);
  EXPECT_LT(std::fabs(sol.crossTrackNm), std::fabs(rawXtkNm));

  FlightData applied;
  applyNavigationSolution(applied, sol, kApproachNmPerDot);
  EXPECT_LT(std::fabs(applied.cdiDeviationDots), 2.5f);
}

TEST(FmsNavigatorTest, TurnAdvisoryCountdownUsesUtf8DegreeAndKeepsSuffix) {
  std::vector<MapLeg> plan;
  MapLeg a; a.id = "AZOMY"; a.lat = 26.60; a.lon = -82.10; plan.push_back(a);
  MapLeg b; b.id = "UZAWO"; b.lat = 26.45; b.lon = -81.95; plan.push_back(b);
  MapLeg c; c.id = "GRAMS"; c.lat = 26.58; c.lon = -81.80; plan.push_back(c);

  MapData map;
  map.positionValid = true;
  map.flightPlan = plan;
  map.ownshipLat = 26.50;
  map.ownshipLon = -82.00;
  map.directToActive = false;

  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.groundSpeedKts = 119.0f;
  data.fmaToWpt = "UZAWO";
  data.fmaActiveLegIndex = 1;
  data.fmaLegDistanceNm = 1.00f;

  const TurnAnticipation ta =
      computeTurnAnticipation(map, data, false, CdiSource::Gps);
  ASSERT_TRUE(ta.active);
  EXPECT_NE(ta.message.find("\xC2\xB0"), std::string::npos);
  EXPECT_EQ(ta.message.find('\xB0', 0), ta.message.find("\xC2\xB0") + 1);
  EXPECT_NE(ta.message.find(" in "), std::string::npos);
  EXPECT_NE(ta.message.find("seconds"), std::string::npos);
}

}  // namespace
}  // namespace avionics::test
