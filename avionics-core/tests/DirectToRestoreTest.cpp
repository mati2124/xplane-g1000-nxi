#include "avionics/NavigationComputer.h"

#include <gtest/gtest.h>

#include <vector>

#include "avionics/FmsNavigator.h"
#include "avionics/MapData.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

std::vector<MapLeg> kfmyToKjaxRoute() {
  return {
      makeLeg("KFMY", 26.586617, -81.863247),
      makeLeg("CSHEL", 27.023422, -81.744672),
      makeLeg("LAL", 27.986197, -82.013906),
      makeLeg("JINOS", 28.479444, -82.147778),
      makeLeg("TEBOW", 30.096092, -82.075750),
      makeLeg("KJAX", 30.494044, -81.687847),
  };
}

TEST(DirectToRestoreTest, SyncKeepsInPlanDirectToAfterFlightPlanReload) {
  FmsNavigator nav;
  MapData map;
  map.flightPlan = kfmyToKjaxRoute();
  map.directToActive = true;
  map.directTo = makeLeg("LAL", 27.986197, -82.013906);
  map.directToOriginValid = true;
  map.directToOriginLat = 27.2709;
  map.directToOriginLon = -81.8153;

  syncNavigatorFlightPlan(nav, map);
  syncNavigatorDirectTo(nav, map);

  EXPECT_TRUE(nav.directToActive());
  EXPECT_EQ(nav.directToTarget().id, "LAL");
  EXPECT_EQ(nav.activeLegIndex(), 2);
}

TEST(DirectToRestoreTest, StartupWithoutDirectToDefaultsToFirstLeg) {
  FmsNavigator nav;
  MapData map;
  map.flightPlan = kfmyToKjaxRoute();
  // Direct-To was cleared before the navigator saw the map (e.g. reconnect called
  // setRouteOverride({}) on the prior build).
  map.directToActive = false;

  syncNavigatorFlightPlan(nav, map);
  syncNavigatorDirectTo(nav, map);

  EXPECT_FALSE(nav.directToActive());
  EXPECT_EQ(nav.activeLegIndex(), 0);

  NavigationSolution sol = nav.update(27.0, -81.8, 120.0f, 5000.0f);
  ASSERT_TRUE(sol.active);
  EXPECT_EQ(sol.toWpt, "KFMY");
  EXPECT_TRUE(sol.fromWpt.empty());
}

TEST(DirectToRestoreTest, SyncPreservesDirectToWithBridgeFlightPlan) {
  FmsNavigator nav;
  MapData map;
  map.flightPlan = kfmyToKjaxRoute();
  map.directToActive = true;
  map.directTo = makeLeg("LAL", 27.986197, -82.013906);
  map.directToOriginValid = true;
  map.directToOriginLat = 27.2709;
  map.directToOriginLon = -81.8153;

  syncNavigatorFlightPlan(nav, map);
  syncNavigatorDirectTo(nav, map);

  // Re-sync the same bridge plan (as updateMap would each frame).
  syncNavigatorFlightPlan(nav, map);
  syncNavigatorDirectTo(nav, map);

  EXPECT_TRUE(nav.directToActive());
  EXPECT_EQ(nav.directToTarget().id, "LAL");
  EXPECT_EQ(nav.activeLegIndex(), 2);
}

}  // namespace
}  // namespace avionics
