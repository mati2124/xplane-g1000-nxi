#include <gtest/gtest.h>

#include "avionics/GlidepathGuidance.h"
#include "avionics/MissedApproachGuidance.h"

#include "ApproachTestFixtures.h"

namespace avionics::test {
namespace {

TEST(MissedApproachGuidanceTest, SuppressGlidepathAtMaptSusp) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& mapt = plan[2];

  MapData map = makeMapAt(mapt.lat, mapt.lon, plan);
  FlightData data = makeGpsFlightData("FAF01", "RW09", 0.1f, 400.0f);
  data.fmaActiveLegIndex = 2;
  data.gpsSequencingSuspended = true;

  EXPECT_TRUE(suppressGlidepath(map, data));
  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_FALSE(gp.valid);
}

TEST(MissedApproachGuidanceTest, SuppressGlidepathWhenMissedActive) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  const MapLeg& ibite = plan[3];

  MapData map = makeMapAt(ibite.lat - 0.01, ibite.lon, plan);
  FlightData data = makeGpsFlightData("RW09", "IBITE", 0.5f, 500.0f);
  data.fmaActiveLegIndex = 3;
  data.missedApproachActive = true;

  EXPECT_TRUE(suppressGlidepath(map, data));
  EXPECT_FALSE(computeGlidepath(map, data).valid);
}

TEST(MissedApproachGuidanceTest, MissedClimbTargetsNextAtOrAboveConstraint) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  const MapLeg& ibite = plan[3];

  MapData map = makeMapAt(ibite.lat - 0.01, ibite.lon, plan);
  FlightData data = makeGpsFlightData("RW09", "IBITE", 0.5f, 500.0f);
  data.fmaActiveLegIndex = 3;
  data.missedApproachActive = true;

  const MissedClimbProfile profile = computeMissedClimbProfile(map, data);
  EXPECT_TRUE(profile.active);
  EXPECT_EQ(profile.targetWpt, "IBITE");
  EXPECT_EQ(profile.targetAltFt, 2600);
  EXPECT_GT(profile.vsRequiredFpm, 500.0f);

  applyMissedClimbProfile(data, profile);
  EXPECT_TRUE(data.requiredVsValid);
  EXPECT_GT(data.requiredVsFpm, 500.0f);
}

TEST(MissedApproachGuidanceTest, NoMissedClimbBeforeActivation) {
  const std::vector<MapLeg> plan = makeRnavApproachWithMissedPlan();
  const MapLeg& mapt = plan[2];

  MapData map = makeMapAt(mapt.lat, mapt.lon, plan);
  FlightData data = makeGpsFlightData("FAF01", "RW09", 0.1f, 400.0f);
  data.fmaActiveLegIndex = 2;
  data.gpsSequencingSuspended = true;

  const MissedClimbProfile profile = computeMissedClimbProfile(map, data);
  EXPECT_FALSE(profile.active);
}

}  // namespace
}  // namespace avionics::test
