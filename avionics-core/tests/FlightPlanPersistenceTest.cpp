#include "avionics/FlightPlanPersistence.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id, double lat, double lon, const char* role = "") {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  leg.procedureRole = role;
  return leg;
}

// KFMY RNAV 05 (PINTS transition): IAF/FAF feeders, RW05 missed-approach point,
// then the missed approach (IBITE climb, SERFS holding fix). Mirrors the legs the
// standalone app persists for this approach.
std::vector<MapLeg> makeKfmyR05Approach() {
  return {
      makeLeg("PINTS", 26.802892, -82.135858),
      makeLeg("AZOMY", 26.522381, -82.132675, "iaf"),
      makeLeg("UZAWO", 26.436994, -82.046517, "iaf"),
      makeLeg("GRAMS", 26.512742, -81.953697, "faf"),
      makeLeg("HADMO", 26.558422, -81.897617),
      makeLeg("RW05", 26.580858, -81.870050, "mapt"),
      makeLeg("IBITE", 26.625472, -81.815186),
      makeLeg("SERFS", 26.800653, -81.819575, "mahp"),
  };
}

TEST(FlightPlanPersistenceTest, CollapseRemovesDuplicatedApproachCopy) {
  std::vector<MapLeg> legs = makeKfmyR05Approach();
  const std::vector<MapLeg> single = legs;
  // Simulate the bug: the whole approach (incl. missed approach) appended twice.
  legs.insert(legs.end(), single.begin(), single.end());
  ASSERT_EQ(legs.size(), 16u);

  EXPECT_TRUE(collapseDuplicateApproachTail(legs));
  ASSERT_EQ(legs.size(), single.size());
  for (std::size_t i = 0; i < single.size(); ++i) {
    EXPECT_EQ(legs[i].id, single[i].id);
    EXPECT_EQ(legs[i].procedureRole, single[i].procedureRole);
  }
}

TEST(FlightPlanPersistenceTest, CollapseLeavesSingleApproachUntouched) {
  std::vector<MapLeg> legs = makeKfmyR05Approach();
  EXPECT_FALSE(collapseDuplicateApproachTail(legs));
  EXPECT_EQ(legs.size(), makeKfmyR05Approach().size());
}

TEST(FlightPlanPersistenceTest, CollapseHealsTripledApproach) {
  std::vector<MapLeg> legs = makeKfmyR05Approach();
  const std::vector<MapLeg> single = legs;
  legs.insert(legs.end(), single.begin(), single.end());
  legs.insert(legs.end(), single.begin(), single.end());
  ASSERT_EQ(legs.size(), 24u);

  EXPECT_TRUE(collapseDuplicateApproachTail(legs));
  EXPECT_EQ(legs.size(), single.size());
}

TEST(FlightPlanPersistenceTest, EnrichReinfersMissedApproachAfterCollapse) {
  PersistedFlightPlan plan;
  plan.active = true;
  plan.legs = makeKfmyR05Approach();
  const std::vector<MapLeg> single = plan.legs;
  plan.legs.insert(plan.legs.end(), single.begin(), single.end());
  // The corrupted on-disk grouping spans both duplicated copies.
  plan.approachLegStart = 0;
  plan.approachLegCount = 16;

  enrichPersistedFlightPlanFromLegs(plan);

  // Duplicate copy removed and the approach block re-inferred over the single
  // remaining copy (PINTS feeder through the SERFS missed-approach hold).
  ASSERT_EQ(plan.legs.size(), single.size());
  EXPECT_EQ(plan.approachLegStart, 0);
  EXPECT_EQ(plan.approachLegStart + plan.approachLegCount,
            static_cast<int>(plan.legs.size()));

  // The single missed-approach point (RW05) survives so the missed approach can
  // still be distinguished after restore.
  int maptCount = 0;
  for (const MapLeg& leg : plan.legs) {
    if (leg.procedureRole == "mapt") ++maptCount;
  }
  EXPECT_EQ(maptCount, 1);
}

TEST(FlightPlanPersistenceTest, RemoveLoadedApproachLegsErasesBlock) {
  std::vector<MapLeg> legs = {
      makeLeg("ORIG", 26.0, -82.0),
      makeLeg("ENRT", 26.2, -81.9),
      makeLeg("RW05", 26.5, -81.8, "mapt"),
      makeLeg("SERFS", 26.8, -81.8, "mahp"),
  };
  removeLoadedApproachLegs(legs, 2, 2);
  ASSERT_EQ(legs.size(), 2u);
  EXPECT_EQ(legs[0].id, "ORIG");
  EXPECT_EQ(legs[1].id, "ENRT");
}

TEST(FlightPlanPersistenceTest, RemoveLoadedApproachLegsIgnoresOutOfRange) {
  std::vector<MapLeg> legs = {makeLeg("ORIG", 26.0, -82.0)};
  removeLoadedApproachLegs(legs, 0, 0);
  EXPECT_EQ(legs.size(), 1u);
  removeLoadedApproachLegs(legs, 1, 5);
  EXPECT_EQ(legs.size(), 1u);
}

TEST(FlightPlanPersistenceTest, PersistedLegRoundTripsDesignatedAltitude) {
  MapLeg leg = makeLeg("VASES", 26.5, -81.9);
  leg.altitudeConstraintFt = 4500;
  leg.altitudeConstraint = AltConstraintType::At;
  leg.altitudeDesignated = true;

  const std::string encoded = formatPersistedFlightPlanLeg(leg);
  MapLeg decoded;
  ASSERT_TRUE(parsePersistedFlightPlanLeg(encoded, decoded));
  EXPECT_EQ(decoded.id, "VASES");
  EXPECT_DOUBLE_EQ(decoded.lat, 26.5);
  EXPECT_DOUBLE_EQ(decoded.lon, -81.9);
  EXPECT_EQ(decoded.altitudeConstraintFt, 4500);
  EXPECT_EQ(decoded.altitudeConstraint, AltConstraintType::At);
  EXPECT_TRUE(decoded.altitudeDesignated);
}

TEST(FlightPlanPersistenceTest, PersistedLegParsesLegacyFormatWithoutAltitude) {
  MapLeg decoded;
  ASSERT_TRUE(parsePersistedFlightPlanLeg("VASES|26.500000|-81.900000|iaf",
                                          decoded));
  EXPECT_EQ(decoded.id, "VASES");
  EXPECT_EQ(decoded.procedureRole, "iaf");
  EXPECT_EQ(decoded.altitudeConstraintFt, 0);
  EXPECT_FALSE(decoded.altitudeDesignated);
}

TEST(FlightPlanPersistenceTest, InferApproachBlockIncludesUntaggedIafBeforeFaf) {
  // KFMY->KJAX with RNAV R08-Y via WADOR: sim export often tags only GRRDN
  // as faf, leaving WADOR/AMXUQ untagged. The approach block must start at
  // WADOR (after destination KJAX), not at AMXUQ.
  std::vector<MapLeg> legs = {
      makeLeg("KFMY", 26.58, -81.87),
      makeLeg("LAL", 27.98, -82.01),
      makeLeg("JINOS", 28.50, -82.10),
      makeLeg("TEBOW", 29.80, -82.00),
      makeLeg("KJAX", 30.49, -81.69),
      makeLeg("WADOR", 30.55, -81.75),
      makeLeg("AMXUQ", 30.52, -81.72),
      makeLeg("GRRDN", 30.50, -81.70, "faf"),
  };
  const InferredProcedureBlock block = inferProcedureBlockInPlan(legs);
  ASSERT_TRUE(block.valid());
  EXPECT_EQ(block.start, 5);
  EXPECT_EQ(legs[static_cast<std::size_t>(block.start)].id, "WADOR");
  EXPECT_EQ(block.count, 3);

  const int anchored = approachBlockStartFromTransition(legs, "WADOR", 6);
  EXPECT_EQ(anchored, 5);
}

TEST(FlightPlanPersistenceTest, ResolveApproachBlockAnchorsLoadedTransition) {
  std::vector<MapLeg> legs = {
      makeLeg("KFMY", 26.58, -81.87),
      makeLeg("TEBOW", 29.80, -82.00),
      makeLeg("KJAX", 30.49, -81.69),
      makeLeg("WADOR", 30.55, -81.75),
      makeLeg("AMXUQ", 30.52, -81.72),
      makeLeg("GRRDN", 30.50, -81.70, "faf"),
  };
  // Stale grouping that omits the IAF but still fits the leg array.
  const InferredProcedureBlock resolved =
      resolveApproachBlockInPlan(legs, 4, 2, "WADOR");
  ASSERT_TRUE(resolved.valid());
  EXPECT_EQ(resolved.start, 3);
  EXPECT_EQ(legs[static_cast<std::size_t>(resolved.start)].id, "WADOR");
  EXPECT_EQ(resolved.count, 3);
}

TEST(FlightPlanPersistenceTest, PersistedDirectToRoundTripFromMap) {
  MapData map;
  map.directToActive = true;
  map.directTo = makeLeg("PINTS", 26.802892, -82.135858);
  map.directToOriginValid = true;
  map.directToOriginLat = 26.58;
  map.directToOriginLon = -81.87;

  const PersistedDirectTo saved = persistedDirectToFromMap(map);
  EXPECT_TRUE(saved.active);
  EXPECT_EQ(saved.target.id, "PINTS");
  EXPECT_DOUBLE_EQ(saved.originLat, 26.58);
  EXPECT_DOUBLE_EQ(saved.originLon, -81.87);
  EXPECT_TRUE(saved.originValid);
  EXPECT_EQ(saved.activeLegIndex, -1);

  map.directToActive = false;
  map.directTo = {};
  EXPECT_FALSE(persistedDirectToFromMap(map).active);
}

TEST(FlightPlanPersistenceTest, PersistedNavigationSnapshotCapturesActiveLeg) {
  MapData map;
  map.flightPlan = {
      makeLeg("KFMY", 26.586617, -81.863247),
      makeLeg("TEBOW", 30.096092, -82.075750),
  };
  FlightData data;
  data.fmaActiveLegIndex = 1;
  data.fmaFromWpt = "KFMY";
  data.fmaToWpt = "TEBOW";

  const PersistedDirectTo saved = persistedNavigationSnapshot(map, data);
  EXPECT_FALSE(saved.active);
  EXPECT_EQ(saved.activeLegIndex, 1);
}

}  // namespace
}  // namespace avionics
