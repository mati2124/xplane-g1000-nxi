#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/ProcedureSupport.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics::test {
namespace {

MapLeg MakeLeg(const std::string& id, double lat = 0.0, double lon = 0.0) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

// Fake nav that expands a single STAR (SHFTY6 into KFMY, INPIN enroute
// transition), used to exercise the interactive PROC arrival load path
// end-to-end.
class FakeArrivalSource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    if (icao != "KFMY" || type != ProcedureType::Arrival) return {};
    MapProcedure proc;
    proc.type = ProcedureType::Arrival;
    proc.name = "SHFTY6";
    proc.transition = "INPIN";
    return {proc};
  }
  std::vector<MapLeg> expandProcedure(
      const std::string& icao, ProcedureType type, const std::string& name,
      const std::string& transition) const override {
    if (icao != "KFMY" || type != ProcedureType::Arrival || name != "SHFTY6") {
      return {};
    }
    if (transition == "INPIN") {
      return {MakeLeg("INPIN"), MakeLeg("DEEDS"), MakeLeg("SHFTY")};
    }
    return {};
  }
};

ProcedureMenuHost MakeHost(ProcedureMenuState& state, NavFeatureSource* nav,
                           std::vector<MapLeg>& legs, int& approachStart,
                           int& approachCount, int& cursorRow,
                           MapProcedure& loadedApproach,
                           PersistedLoadedApproach& persistedApproach,
                           std::string& approachHeader, int& departureStart,
                           int& departureCount, MapProcedure& loadedDeparture,
                           PersistedLoadedApproach& persistedDeparture,
                           std::string& departureHeader, int& arrivalStart,
                           int& arrivalCount, MapProcedure& loadedArrival,
                           PersistedLoadedApproach& persistedArrival,
                           std::string& arrivalHeader) {
  return ProcedureMenuHost{
      state,
      nav,
      nullptr,
      legs,
      approachStart,
      approachCount,
      cursorRow,
      loadedApproach,
      persistedApproach,
      &approachHeader,
      &departureStart,
      &departureCount,
      &loadedDeparture,
      &persistedDeparture,
      &departureHeader,
      &arrivalStart,
      &arrivalCount,
      &loadedArrival,
      &persistedArrival,
      &arrivalHeader,
      []() -> std::string { return "KFMY"; },
      []() { return std::vector<std::string>{}; },
      []() {},
      [](int) {},
      []() {},
      []() {},
      []() { return false; },
      [](bool) {},
      []() { return 0.0f; },
      [](float) {},
  };
}

// KJAX origin, KFMY destination, then loading the SHFTY6 STAR (INPIN
// transition) must splice the STAR fixes ahead of the destination airport and
// tag them as the arrival block - not leak them into the enroute section, and
// not let KJAX become the destination.
TEST(ArrivalLoadTest, ProcLoadPlacesArrivalBlockBeforeDestination) {
  FakeArrivalSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Arrival;
  state.selectedAirportIcao = "KFMY";
  state.selectedName = "SHFTY6";
  state.selectedTransition = "INPIN";

  std::vector<MapLeg> legs = {MakeLeg("KJAX"), MakeLeg("KFMY")};
  int approachStart = 0, approachCount = 0, cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0, departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;
  int arrivalStart = 0, arrivalCount = 0;
  MapProcedure loadedArrival;
  PersistedLoadedApproach persistedArrival;
  std::string arrivalHeader;

  ProcedureMenuHost host = MakeHost(
      state, &nav, legs, approachStart, approachCount, cursorRow, loadedApproach,
      persistedApproach, approachHeader, departureStart, departureCount,
      loadedDeparture, persistedDeparture, departureHeader, arrivalStart,
      arrivalCount, loadedArrival, persistedArrival, arrivalHeader);

  procedureMenuLoadSelected(host, "SHFTY6", "INPIN");

  // STAR fixes spliced ahead of the destination airport: [KJAX, INPIN, DEEDS,
  // SHFTY, KFMY].
  ASSERT_EQ(legs.size(), 5u);
  EXPECT_EQ(legs[0].id, "KJAX");
  EXPECT_EQ(legs[1].id, "INPIN");
  EXPECT_EQ(legs[2].id, "DEEDS");
  EXPECT_EQ(legs[3].id, "SHFTY");
  EXPECT_EQ(legs[4].id, "KFMY");

  // KFMY stays the destination (last leg), KJAX is still the origin.
  EXPECT_EQ(legs.back().id, "KFMY");

  // Arrival block metadata is set so the renderer groups the STAR.
  EXPECT_EQ(arrivalStart, 1);
  EXPECT_EQ(arrivalCount, 3);
  EXPECT_EQ(arrivalHeader, "SHFTY6.INPIN");
  EXPECT_EQ(loadedArrival.name, "SHFTY6");
  EXPECT_EQ(loadedArrival.transition, "INPIN");
  EXPECT_EQ(loadedArrival.type, ProcedureType::Arrival);
  EXPECT_EQ(persistedArrival.airportIcao, "KFMY");
}

TEST(ArrivalLoadTest, ArrivalBlockExcludedFromEnrouteIndices) {
  FakeArrivalSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Arrival;
  state.selectedAirportIcao = "KFMY";
  state.selectedName = "SHFTY6";
  state.selectedTransition = "INPIN";

  std::vector<MapLeg> legs = {MakeLeg("KJAX"), MakeLeg("KFMY")};
  int approachStart = 0, approachCount = 0, cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0, departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;
  int arrivalStart = 0, arrivalCount = 0;
  MapProcedure loadedArrival;
  PersistedLoadedApproach persistedArrival;
  std::string arrivalHeader;

  ProcedureMenuHost host = MakeHost(
      state, &nav, legs, approachStart, approachCount, cursorRow, loadedApproach,
      persistedApproach, approachHeader, departureStart, departureCount,
      loadedDeparture, persistedDeparture, departureHeader, arrivalStart,
      arrivalCount, loadedArrival, persistedArrival, arrivalHeader);

  procedureMenuLoadSelected(host, "SHFTY6", "INPIN");

  // None of the STAR fixes leak into the enroute section.
  const auto enroute = pfd::fplProcedureEnrouteLegIndices(
      legs, departureStart, departureCount, arrivalStart, arrivalCount,
      /*approachStart=*/0, /*approachCount=*/0, /*destinationFilled=*/true);
  EXPECT_TRUE(enroute.empty());

  // Display rows render the STAR under an Arrival header, not Enroute.
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, departureStart, departureCount, departureHeader, arrivalStart,
      arrivalCount, arrivalHeader, /*approachStart=*/0, /*approachCount=*/0,
      /*blankOriginSection=*/false, /*destinationFilled=*/true);

  bool sawArrivalHeader = false;
  bool shftyUnderArrival = false;
  for (const pfd::FplDisplayRow& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::ArrivalHeader) sawArrivalHeader = true;
    if (dr.kind == pfd::FplDisplayRowKind::ArrivalLeg && dr.legIndex >= 0 &&
        legs[static_cast<std::size_t>(dr.legIndex)].id == "SHFTY") {
      shftyUnderArrival = true;
    }
    if (dr.kind == pfd::FplDisplayRowKind::EnrouteLeg && dr.legIndex >= 0) {
      const std::string& id = legs[static_cast<std::size_t>(dr.legIndex)].id;
      EXPECT_NE(id, "INPIN") << "INPIN must not render under Enroute";
      EXPECT_NE(id, "DEEDS") << "DEEDS must not render under Enroute";
      EXPECT_NE(id, "SHFTY") << "SHFTY must not render under Enroute";
    }
  }
  EXPECT_TRUE(sawArrivalHeader);
  EXPECT_TRUE(shftyUnderArrival);
}

// A long STAR whose fixes carry approach-style procedureRole tags (e.g. a
// "mahp" leg) must NOT be mistaken for a loaded approach. Reproduces the
// reported KJAX->KFMY SHFTY6 plan where the arrival block (legs 8-18) ended at
// PONTY and the renderer invented a second "KJAX" approach section from MAZZY.
std::vector<MapLeg> MakeStarPlanWithRoleTaggedLeg() {
  std::vector<MapLeg> plan = {
      MakeLeg("KJAX"),  MakeLeg("RW08"),  MakeLeg("540FT"), MakeLeg("MANSEQ"),
      MakeLeg("DURTE"), MakeLeg("JETIN"), MakeLeg("FELTZ"), MakeLeg("JAYJA"),
      MakeLeg("INPIN"), MakeLeg("VALCH"), MakeLeg("SHFTY"), MakeLeg("WRTRS"),
      MakeLeg("BUNGE"), MakeLeg("MAZZY"), MakeLeg("MOEMO"), MakeLeg("LBELL"),
      MakeLeg("IRNIE"), MakeLeg("WYCOF"), MakeLeg("PONTY"), MakeLeg("KFMY")};
  plan[13].procedureRole = "mahp";  // STAR fix that happens to carry a role tag
  return plan;
}

TEST(ArrivalLoadTest, LongStarRoleTaggedLegDoesNotInferApproach) {
  const std::vector<MapLeg> plan = MakeStarPlanWithRoleTaggedLeg();
  const int arrStart = 8;
  const int arrCount = 11;  // INPIN..PONTY are all the STAR
  const int arrivalEnd = arrStart + arrCount;  // 19 (KFMY)

  // Without the arrival-aware guard the role-tagged MAZZY leg (index 13) would
  // anchor a bogus approach block; with it, nothing after the STAR is an
  // approach (only KFMY remains, an airport).
  const InferredProcedureBlock naive = inferProcedureBlockInPlan(plan);
  EXPECT_EQ(naive.start, 13);

  const InferredProcedureBlock guarded =
      inferProcedureBlockInPlan(plan, arrivalEnd);
  EXPECT_FALSE(guarded.valid());

  // Even a stale stored approach block that sits inside the STAR is rejected.
  const InferredProcedureBlock resolved = resolveApproachBlockInPlan(
      plan, /*storedStart=*/13, /*storedCount=*/7, /*transition=*/std::string(),
      arrivalEnd);
  EXPECT_FALSE(resolved.valid());
}

TEST(ArrivalLoadTest, ApproachAfterStarStillInfers) {
  // STAR INPIN..BUNGE (8-12), then KFMY (13), then a real approach (14-17).
  std::vector<MapLeg> plan = {
      MakeLeg("KJAX"),  MakeLeg("RW08"),  MakeLeg("540FT"), MakeLeg("MANSEQ"),
      MakeLeg("DURTE"), MakeLeg("JETIN"), MakeLeg("FELTZ"), MakeLeg("JAYJA"),
      MakeLeg("INPIN"), MakeLeg("VALCH"), MakeLeg("SHFTY"), MakeLeg("WRTRS"),
      MakeLeg("BUNGE"), MakeLeg("KFMY"),  MakeLeg("CETki"), MakeLeg("FOXXy"),
      MakeLeg("RW06"),  MakeLeg("MISSD")};
  plan[15].procedureRole = "faf";
  plan[17].procedureRole = "mahp";
  const int arrivalEnd = 8 + 5;  // STAR occupies 8-12, ends at 13 (KFMY)

  const InferredProcedureBlock block =
      inferProcedureBlockInPlan(plan, arrivalEnd);
  ASSERT_TRUE(block.valid());
  // Approach starts at the first role-tagged leg after the destination airport.
  EXPECT_GE(block.start, 14);
  EXPECT_EQ(block.start + block.count, static_cast<int>(plan.size()));
}

}  // namespace
}  // namespace avionics::test
