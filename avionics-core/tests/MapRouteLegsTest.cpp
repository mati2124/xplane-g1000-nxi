#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id, const char* role = "") {
  MapLeg leg;
  leg.id = id;
  leg.procedureRole = role;
  return leg;
}

TEST(MapRouteLegsTest, OmitsDestinationAirportBeforeApproach) {
  const std::vector<MapLeg> plan = {
      makeLeg("KFMY"), makeLeg("MCFIE"), makeLeg("TEBOW"), makeLeg("KJAX"),
      makeLeg("FAROT", "iaf"), makeLeg("LETRE"), makeLeg("GRRDN", "faf"),
      makeLeg("RW08", "mapt"), makeLeg("YEJWO", "mahp")};

  EXPECT_EQ(destinationAirportLegBeforeApproach(plan), 3);

  const std::vector<MapLeg> route = mapRouteDisplayLegs(plan);
  ASSERT_EQ(route.size(), plan.size() - 1);
  EXPECT_EQ(route[2].id, "TEBOW");
  EXPECT_EQ(route[3].id, "FAROT");
  EXPECT_EQ(route.back().id, "YEJWO");

  for (std::size_t i = 0; i < route.size(); ++i) {
    if (route[i].id == "KJAX") {
      FAIL() << "destination airport must not appear on the map route";
    }
  }
}

TEST(MapRouteLegsTest, KeepsDestinationWhenNoApproachLoaded) {
  const std::vector<MapLeg> plan = {makeLeg("KFMY"), makeLeg("TEBOW"),
                                    makeLeg("KJAX")};
  EXPECT_EQ(destinationAirportLegBeforeApproach(plan), -1);
  const std::vector<MapLeg> unchanged = mapRouteDisplayLegs(plan);
  ASSERT_EQ(unchanged.size(), plan.size());
  EXPECT_EQ(unchanged.back().id, "KJAX");
}

TEST(MapRouteLegsTest, DisplayIndexMappingSkipsDestinationAirport) {
  const std::vector<MapLeg> plan = {
      makeLeg("KFMY"), makeLeg("TEBOW"), makeLeg("KJAX"), makeLeg("FAROT", "iaf")};

  EXPECT_EQ(mapRouteDisplayLegIndex(plan, 2), -1);
  EXPECT_EQ(mapRouteDisplayLegIndex(plan, 3), 2);
  EXPECT_EQ(mapRoutePlanLegIndex(plan, 2), 3);
}

TEST(MapRouteLegsTest, StarPlusApproachSkipsDestinationAirport) {
  const std::vector<MapLeg> plan = {
      makeLeg("KFMY"), makeLeg("DURTE"), makeLeg("INPIN", "trans"),
      makeLeg("DEEDS", "trans"), makeLeg("SHFTY", "trans"), makeLeg("KJAX"),
      makeLeg("HOMER", "iaf"), makeLeg("GIPNO"), makeLeg("RW08", "mapt")};

  const std::vector<MapLeg> route = mapRouteDisplayLegs(plan);
  ASSERT_GE(route.size(), 2u);
  EXPECT_EQ(route[4].id, "SHFTY");
  EXPECT_EQ(route[5].id, "HOMER");
}

}  // namespace
}  // namespace avionics
