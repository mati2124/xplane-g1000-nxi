#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "avionics/CifpParser.h"
#include "avionics/GlidepathGuidance.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"

#include "ApproachTestFixtures.h"
#include "CifpFixLookup.h"

#ifndef AVIONICS_TEST_FIXTURE_DIR
#define AVIONICS_TEST_FIXTURE_DIR "fixtures"
#endif

namespace avionics::test {
namespace {

std::string fixturePath(const char* name) {
  return std::string(AVIONICS_TEST_FIXTURE_DIR) + "/" + name;
}

CifpAirportProcedures loadKpgdFixture() {
  std::ifstream in(fixturePath("KPGD_R04_BULOW.dat"));
  EXPECT_TRUE(in.good());
  return parseCifp(in, "KPGD");
}

int legIndexById(const std::vector<MapLeg>& legs, const std::string& id) {
  for (std::size_t i = 0; i < legs.size(); ++i) {
    if (legs[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

TEST(CifpParserTest, ParsesKpgdFixtureRunways) {
  const CifpAirportProcedures data = loadKpgdFixture();
  EXPECT_EQ(data.icao, "KPGD");
  EXPECT_FALSE(data.legs.empty());
  EXPECT_TRUE(data.runways.count("RW04") > 0);
  const auto rw = data.runways.at("RW04");
  EXPECT_GT(rw.first, 26.5);
  EXPECT_LT(rw.second, -81.5);
}

TEST(CifpParserTest, ListsBulowTransitionForR04) {
  const CifpAirportProcedures data = loadKpgdFixture();
  const std::vector<ApproachTransitionOption> transitions =
      listApproachTransitions(data, "R04");
  bool foundBulow = false;
  for (const ApproachTransitionOption& opt : transitions) {
    if (opt.id == "BULOW") foundBulow = true;
  }
  EXPECT_TRUE(foundBulow);
}

TEST(CifpParserTest, ExpandsR04BulowFinalSegment) {
  const CifpAirportProcedures data = loadKpgdFixture();
  CifpFixTable fixes = kpgdR04FixTable();
  const std::vector<MapLeg> legs =
      expandCifpProcedure(data, ProcedureType::Approach, "R04", "BULOW",
                          cifpFixLookup, &fixes);
  EXPECT_GE(legs.size(), 4);

  const int bulowIdx = legIndexById(legs, "BULOW");
  const int cistsIdx = legIndexById(legs, "CISTS");
  const int yencuIdx = legIndexById(legs, "YENCU");
  const int rwIdx = legIndexById(legs, "RW04");
  EXPECT_GE(bulowIdx, 0);
  EXPECT_GT(cistsIdx, bulowIdx);
  EXPECT_GT(yencuIdx, cistsIdx);
  EXPECT_GT(rwIdx, yencuIdx);

  EXPECT_EQ(legs[static_cast<std::size_t>(cistsIdx)].procedureRole, "faf");
  EXPECT_EQ(legs[static_cast<std::size_t>(rwIdx)].procedureRole, "mapt");
  EXPECT_EQ(legs[static_cast<std::size_t>(rwIdx)].altitudeConstraintFt, 73);
  EXPECT_GE(legs[static_cast<std::size_t>(yencuIdx)].glidePathAngleDeg, 2.5f);
}

TEST(CifpParserTest, KpgdR04GlidepathFromExpandedProcedure) {
  const CifpAirportProcedures data = loadKpgdFixture();
  CifpFixTable fixes = kpgdR04FixTable();
  const std::vector<MapLeg> legs =
      expandCifpProcedure(data, ProcedureType::Approach, "R04", "BULOW",
                          cifpFixLookup, &fixes);
  ASSERT_GE(legs.size(), 4);

  const MapLeg& yencu = legs[static_cast<std::size_t>(legIndexById(legs, "YENCU"))];
  const MapLeg& rw04 = legs.back();

  MapData map;
  map.positionValid = true;
  // ~4 nm from the threshold (inside the 15 nm glidepath annunciation window).
  map.ownshipLat = rw04.lat + 0.065;
  map.ownshipLon = rw04.lon;
  map.flightPlan = legs;

  FlightData dataFd = makeGpsFlightData("YENCU", "RW04", 3.5f, 700.0f);

  const GlidepathSolution gp = computeGlidepath(map, dataFd);
  EXPECT_TRUE(gp.valid);
  EXPECT_NEAR(gp.glidePathAngleDeg, 3.0f, 0.2f);

  const double distNm =
      navDistanceNm(map.ownshipLat, map.ownshipLon, rw04.lat, rw04.lon);
  const double expectedPathFt =
      73.0 + std::tan(3.0 * 3.14159265358979323846 / 180.0) * distNm * 6076.12;
  EXPECT_NEAR(gp.pathAltitudeFt, static_cast<float>(expectedPathFt), 150.0f);
  EXPECT_LT(gp.pathAltitudeFt, 2500.0f);
}

TEST(CifpParserTest, ListsCitagTransitionForKfmyR05) {
  std::ifstream in(fixturePath("KFMY_R05_CITAG.dat"));
  ASSERT_TRUE(in.good());
  const CifpAirportProcedures data = parseCifp(in, "KFMY");
  const std::vector<ApproachTransitionOption> transitions =
      listApproachTransitions(data, "R05");
  bool foundCitag = false;
  for (const ApproachTransitionOption& opt : transitions) {
    if (opt.id == "CITAG") foundCitag = true;
  }
  EXPECT_TRUE(foundCitag);
}

TEST(CifpParserTest, ExpandsKfmyR05CitagFeederSegment) {
  std::ifstream in(fixturePath("KFMY_R05_CITAG.dat"));
  ASSERT_TRUE(in.good());
  const CifpAirportProcedures data = parseCifp(in, "KFMY");
  CifpFixTable fixes = kfmyR05FixTable();
  const std::vector<MapLeg> legs =
      expandCifpProcedure(data, ProcedureType::Approach, "R05", "CITAG",
                          cifpFixLookup, &fixes);
  ASSERT_GE(legs.size(), 5u);

  const int citagIdx = legIndexById(legs, "CITAG");
  const int butlyIdx = legIndexById(legs, "BUTLY");
  const int uzawoIdx = legIndexById(legs, "UZAWO");
  const int gramsIdx = legIndexById(legs, "GRAMS");
  const int rwIdx = legIndexById(legs, "RW05");
  EXPECT_GE(citagIdx, 0);
  EXPECT_GT(butlyIdx, citagIdx);
  EXPECT_GT(uzawoIdx, butlyIdx);
  EXPECT_GT(gramsIdx, uzawoIdx);
  EXPECT_GT(rwIdx, gramsIdx);
  EXPECT_EQ(legs[static_cast<std::size_t>(gramsIdx)].procedureRole, "faf");
  EXPECT_EQ(legs[static_cast<std::size_t>(rwIdx)].procedureRole, "mapt");
}

TEST(CifpParserTest, KpgdR04LegSequenceNearFaf) {
  const CifpAirportProcedures data = loadKpgdFixture();
  CifpFixTable fixes = kpgdR04FixTable();
  const std::vector<MapLeg> legs =
      expandCifpProcedure(data, ProcedureType::Approach, "R04", "BULOW",
                          cifpFixLookup, &fixes);
  ASSERT_GE(legs.size(), 4);

  const MapLeg& cists = legs[static_cast<std::size_t>(legIndexById(legs, "CISTS"))];
  MapData map;
  map.positionValid = true;
  map.ownshipLat = cists.lat - 0.01;
  map.ownshipLon = cists.lon;
  map.flightPlan = legs;

  FlightData dataFd = makeGpsFlightData("BULOW", "BULOW", 8.0f, 2100.0f);
  const int idx = resolveNavLegToIndex(legs, dataFd, map);
  EXPECT_EQ(idx, legIndexById(legs, "CISTS"));
}

}  // namespace
}  // namespace avionics::test
