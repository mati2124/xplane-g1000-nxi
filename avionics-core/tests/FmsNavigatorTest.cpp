#include <gtest/gtest.h>

#include "avionics/FmsNavigator.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

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

}  // namespace
}  // namespace avionics::test
