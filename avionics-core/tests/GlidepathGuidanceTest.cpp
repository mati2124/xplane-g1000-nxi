#include <cmath>

#include <gtest/gtest.h>

#include "avionics/GlidepathGuidance.h"
#include "avionics/NavMath.h"

#include "ApproachTestFixtures.h"

namespace avionics::test {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFeetPerNm = 6076.12;

TEST(GlidepathGuidanceTest, ValidGlidepathOnFinalSegment) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& faf = plan[1];
  const MapLeg& mapt = plan[2];

  const MapData map = makeMapAt(faf.lat, faf.lon, plan);
  FlightData data = makeGpsFlightData("IAF01", "FAF01", 0.1f, 2000.0f);

  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_TRUE(gp.valid);
  EXPECT_NEAR(gp.glidePathAngleDeg, 3.0f, 0.01f);

  const double distNm =
      navDistanceNm(faf.lat, faf.lon, mapt.lat, mapt.lon);
  const double expectedPathFt =
      50.0 + std::tan(3.0 * kPi / 180.0) * distNm * kFeetPerNm;
  EXPECT_NEAR(gp.pathAltitudeFt, static_cast<float>(expectedPathFt), 80.0f);
}

TEST(GlidepathGuidanceTest, UsesMaptElevationNotFafCrossingHeight) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& mapt = plan[2];

  const MapData map = makeMapAt(mapt.lat - 0.01, mapt.lon, plan);
  FlightData data = makeGpsFlightData("FAF01", "RW09", 0.5f, 700.0f);

  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_TRUE(gp.valid);
  // Near MAPt the path should be hundreds of feet, not FAF-crossing height (~2000 ft).
  EXPECT_LT(gp.pathAltitudeFt, 800.0f);
  EXPECT_GT(gp.pathAltitudeFt, 100.0f);
}

TEST(GlidepathGuidanceTest, ActiveLegIndexDrivesGlidepathDistance) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapLeg& faf = plan[1];

  const MapData map = makeMapAt(faf.lat, faf.lon, plan);
  FlightData data = makeGpsFlightData("IAF01", "IAF01", 5.0f, 2000.0f);
  // FmsNavigator publishes the geometrically active leg even when FMA idents lag.
  data.fmaActiveLegIndex = 1;

  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_TRUE(gp.valid);

  const double distToThr =
      navDistanceNm(faf.lat, faf.lon, plan[2].lat, plan[2].lon);
  const double expectedPathFt =
      50.0 + std::tan(3.0 * M_PI / 180.0) * distToThr * kFeetPerNm;
  EXPECT_NEAR(gp.pathAltitudeFt, static_cast<float>(expectedPathFt), 120.0f);
}

TEST(GlidepathGuidanceTest, InvalidWithoutPublishedThresholdElevation) {
  std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  plan[2].altitudeConstraintFt = 0;
  plan[2].altitudeConstraint = AltConstraintType::None;

  const MapData map = makeMapAt(plan[1].lat, plan[1].lon, plan);
  FlightData data = makeGpsFlightData("IAF01", "FAF01", 0.1f, 2000.0f);

  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_FALSE(gp.valid);
}

TEST(GlidepathGuidanceTest, NotValidForNavSource) {
  const std::vector<MapLeg> plan = makeRnavFinalApproachPlan();
  const MapData map = makeMapAt(plan[1].lat, plan[1].lon, plan);
  FlightData data = makeGpsFlightData("IAF01", "FAF01", 0.1f, 2000.0f);
  data.cdiSource = CdiSource::Nav1;

  const GlidepathSolution gp = computeGlidepath(map, data);
  EXPECT_FALSE(gp.valid);
}

}  // namespace
}  // namespace avionics::test
