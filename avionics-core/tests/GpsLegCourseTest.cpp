#include <gtest/gtest.h>

#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

#include "ApproachTestFixtures.h"

namespace avionics::test {
namespace {

TEST(GpsLegCourseTest, FlightPlanIdentsEqualIgnoresCase) {
  EXPECT_TRUE(flightPlanIdentsEqual("FAF01", "faf01"));
  EXPECT_FALSE(flightPlanIdentsEqual("FAF01", "FAF02"));
}

TEST(GpsLegCourseTest, LegIndexInPlanIgnoresCase) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  EXPECT_EQ(legIndexInPlan(plan, "faf01"), 1);
  EXPECT_EQ(legIndexInPlan(plan, "MAPT"), -1);
  EXPECT_EQ(legIndexInPlan(plan, "RW09"), 2);
}

TEST(GpsLegCourseTest, ResolveActiveLegFromFromToPair) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  EXPECT_EQ(resolveActiveLegToIndex(plan, "IAF01", "FAF01"), 1);
  EXPECT_EQ(resolveActiveLegToIndex(plan, "", "RW09"), 2);
}

TEST(GpsLegCourseTest, AdvanceLegWhenWaypointCaptured) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapData map = makeMapAt(plan[1].lat, plan[1].lon, plan);
  EXPECT_EQ(advanceLegIfWaypointCaptured(plan, map, 1), 2);
}

TEST(GpsLegCourseTest, AdvanceLegWhenCloserToNextWaypoint) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& mapt = plan[2];
  const MapData map = makeMapAt(mapt.lat - 0.008, mapt.lon, plan);
  FlightData data = makeGpsFlightData("FAF01", "FAF01", 2.0f, 1500.0f);
  EXPECT_EQ(resolveNavLegToIndex(plan, data, map), 2);
}

TEST(GpsLegCourseTest, DmeFallbackFindsApproachFixWhenSimIdLags) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& faf = plan[1];
  // Approaching FAF; sim still shows the airport ident.
  const MapData map = makeMapAt(faf.lat - 0.01, faf.lon, plan);

  FlightData data = makeGpsFlightData("", "KPGD", 99.0f, 2200.0f);
  const double distToFaf =
      navDistanceNm(map.ownshipLat, map.ownshipLon, faf.lat, faf.lon);
  data.fmaLegDistanceNm = static_cast<float>(distToFaf);

  const int idx = resolveNavLegToIndex(plan, data, map);
  EXPECT_EQ(idx, 1);
}

TEST(GpsLegCourseTest, ResolveNavLegAdvancesPastCapturedWaypoint) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapData map = makeMapAt(plan[2].lat, plan[2].lon, plan);

  FlightData data = makeGpsFlightData("FAF01", "FAF01", 0.1f, 800.0f);
  const int idx = resolveNavLegToIndex(plan, data, map);
  EXPECT_EQ(idx, 2);
}

}  // namespace
}  // namespace avionics::test
