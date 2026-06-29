#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "avionics/MapData.h"
#include "avionics/FlightData.h"
#include "avionics/VnavGuidance.h"

namespace avionics::test {
namespace {

// Fixes along the -81.2 meridian, 0.5 deg (~30 NM) apart, flown south->north.
MapLeg makeLeg(const std::string& id, double lat, int altFt,
               AltConstraintType type) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = -81.2;
  leg.altitudeConstraintFt = altFt;
  leg.altitudeConstraint = type;
  return leg;
}

MapData makeMap(const std::vector<MapLeg>& plan) {
  MapData map;
  map.positionValid = true;
  // Ownship sits on the active (first) leg, so along-track distance to leg i is
  // just the accumulated leg-to-leg spacing.
  map.ownshipLat = plan.front().lat;
  map.ownshipLon = plan.front().lon;
  map.flightPlan = plan;
  return map;
}

FlightData makeData(const std::string& activeId, float altitudeFt) {
  FlightData data;
  data.dataLinkValid = true;
  data.cdiSource = CdiSource::Gps;
  data.fmaToWpt = activeId;
  data.fmaActiveLegIndex = 0;
  data.altitudeFt = altitudeFt;
  data.groundSpeedKts = 300.0f;
  return data;
}

// FLOOR (>=6000) then CEILING (<=4000) then AT (3000). From high cruise the next
// gate is the earliest top-of-descent among the binding gates: the ceiling.
TEST(VnavGuidanceTest, BindingCeilingControlsTargetOverDistantAt) {
  std::vector<MapLeg> plan = {
      makeLeg("ACT", 26.0, 0, AltConstraintType::None),
      makeLeg("FLOOR", 26.5, 6000, AltConstraintType::AtOrAbove),
      makeLeg("CEIL", 27.0, 4000, AltConstraintType::AtOrBelow),
      makeLeg("ATFIX", 27.5, 3000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("ACT", 10000.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "CEIL");
  EXPECT_EQ(vnv.targetAltFt, 4000);
}

// An "at or above" floor close to the deep AT gate would be busted by the
// descent, so the aircraft levels at the floor first (floor becomes target).
TEST(VnavGuidanceTest, FloorBecomesTargetWhenDescentWouldBustIt) {
  std::vector<MapLeg> plan = {
      makeLeg("ACT", 26.0, 0, AltConstraintType::None),
      makeLeg("FLOOR", 27.45, 5000, AltConstraintType::AtOrAbove),
      makeLeg("ATFIX", 27.5, 3000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("ACT", 8000.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "FLOOR");
  EXPECT_EQ(vnv.targetAltFt, 5000);
}

// With only floors ahead, VNAV descends to the nearest floor.
TEST(VnavGuidanceTest, OnlyFloorsDescendToNearestFloor) {
  std::vector<MapLeg> plan = {
      makeLeg("ACT", 26.0, 0, AltConstraintType::None),
      makeLeg("FLOOR", 26.5, 5000, AltConstraintType::AtOrAbove),
      makeLeg("FLOOR2", 27.0, 4000, AltConstraintType::AtOrAbove),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("ACT", 9000.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "FLOOR");
  EXPECT_EQ(vnv.targetAltFt, 5000);
}

// The map TOD marker position lies on the route, one geometric descent length
// short of the constrained fix (3 deg default path). Fixes run due north along
// the -81.2 meridian, so the TOD latitude is the only thing that moves.
TEST(VnavGuidanceTest, TopOfDescentMarkerSitsOnRoute) {
  std::vector<MapLeg> plan = {
      makeLeg("ACT", 26.0, 0, AltConstraintType::None),
      makeLeg("ATFIX", 27.5, 3000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("ACT", 10000.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  ASSERT_TRUE(vnv.todValid);
  EXPECT_GT(vnv.distanceToTodNm, 0.0f);

  // ATFIX is 90 NM (1.5 deg) north; a 3 deg path loses 7000 ft over ~22 NM, so
  // the TOD sits ~68 NM (~1.13 deg) north of the start at the same longitude.
  EXPECT_NEAR(vnv.todLat, 27.13, 0.05);
  EXPECT_NEAR(vnv.todLon, -81.2, 0.01);
}

// The BUNGE case: aircraft descended early and is just above an "at or above"
// floor (16000) with the next binding gate far beyond (MOEMO at 10000). Heading
// straight for MOEMO would bust the floor, so the floor becomes the target.
TEST(VnavGuidanceTest, FloorProtectedWhenAircraftBelowIdealPath) {
  std::vector<MapLeg> plan = {
      makeLeg("WRTRS", 26.0, 0, AltConstraintType::None),
      makeLeg("BUNGE", 26.323, 16000, AltConstraintType::AtOrAbove),
      makeLeg("MAZZY", 26.581, 0, AltConstraintType::None),
      makeLeg("MOEMO", 26.698, 10000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("WRTRS", 16046.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "BUNGE");
  EXPECT_EQ(vnv.targetAltFt, 16000);
}

// Regression: KFMY->KJAX, cruising well above a near crossing restriction
// ("MCFIE at 12000") with a much lower deep gate beyond ("FAROT at 3000").
// Picking the earliest top-of-descent would target FAROT and draw a path that
// slices through MCFIE far below 12000 -- the aircraft descends early and never
// honors the 12000 crossing. The nearer restriction must control instead.
TEST(VnavGuidanceTest, NearCrossingRestrictionHonoredOverDeepGate) {
  std::vector<MapLeg> plan = {
      makeLeg("FABES", 26.0, 0, AltConstraintType::None),
      makeLeg("MCFIE", 26.058, 12000, AltConstraintType::At),
      makeLeg("TEBOW", 26.171, 0, AltConstraintType::None),
      makeLeg("FAROT", 26.337, 3000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("FABES", 21600.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "MCFIE");
  EXPECT_EQ(vnv.targetAltFt, 12000);
}

// As above, but the near restriction is an "at or above" floor rather than a
// hard "at": the straight descent to the deep gate would still bust it, so it
// remains the controlling target.
TEST(VnavGuidanceTest, NearFloorHonoredOverDeepGateFromCruise) {
  std::vector<MapLeg> plan = {
      makeLeg("FABES", 26.0, 0, AltConstraintType::None),
      makeLeg("MCFIE", 26.058, 12000, AltConstraintType::AtOrAbove),
      makeLeg("TEBOW", 26.171, 0, AltConstraintType::None),
      makeLeg("FAROT", 26.337, 3000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("FABES", 21600.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "MCFIE");
  EXPECT_EQ(vnv.targetAltFt, 12000);
}

// Same plan flown correctly: high and on the geometric path, the 3 deg descent
// to MOEMO clears the BUNGE floor, so the deeper gate stays the target.
TEST(VnavGuidanceTest, FloorNotTargetedWhenOnIdealPath) {
  std::vector<MapLeg> plan = {
      makeLeg("WRTRS", 26.0, 0, AltConstraintType::None),
      makeLeg("BUNGE", 26.323, 16000, AltConstraintType::AtOrAbove),
      makeLeg("MAZZY", 26.581, 0, AltConstraintType::None),
      makeLeg("MOEMO", 26.698, 10000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("WRTRS", 23340.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  ASSERT_TRUE(vnv.active);
  EXPECT_EQ(vnv.targetWpt, "MOEMO");
  EXPECT_EQ(vnv.targetAltFt, 10000);
}

// Constraints at or above the aircraft are not descent targets.
TEST(VnavGuidanceTest, NoDescentWhenConstraintsAtOrAboveAircraft) {
  std::vector<MapLeg> plan = {
      makeLeg("ACT", 26.0, 0, AltConstraintType::None),
      makeLeg("CEIL", 26.5, 4000, AltConstraintType::AtOrBelow),
      makeLeg("ATFIX", 27.0, 5000, AltConstraintType::At),
  };
  const MapData map = makeMap(plan);
  const FlightData data = makeData("ACT", 3000.0f);

  const VnvProfile vnv = computeVnvProfile(map, data);
  EXPECT_FALSE(vnv.active);
}

}  // namespace
}  // namespace avionics::test
