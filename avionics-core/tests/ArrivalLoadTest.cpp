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

// Fake nav that expands both the SHFTY6 STAR and an RNAV approach into KFMY, so
// the interactive PROC load path can be driven for an approach after a STAR.
class FakeStarApproachSource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    if (icao != "KFMY") return {};
    if (type == ProcedureType::Arrival) {
      MapProcedure p;
      p.type = ProcedureType::Arrival;
      p.name = "SHFTY6";
      p.transition = "INPIN";
      return {p};
    }
    if (type == ProcedureType::Approach) {
      MapProcedure p;
      p.type = ProcedureType::Approach;
      p.name = "RNAV06";
      p.transition = "HOMER";
      return {p};
    }
    return {};
  }
  std::vector<MapLeg> expandProcedure(
      const std::string& icao, ProcedureType type, const std::string& name,
      const std::string& transition) const override {
    if (icao != "KFMY") return {};
    if (type == ProcedureType::Arrival && name == "SHFTY6" &&
        transition == "INPIN") {
      return {MakeLeg("INPIN"), MakeLeg("DEEDS"), MakeLeg("SHFTY")};
    }
    if (type == ProcedureType::Approach && name == "RNAV06") {
      std::vector<MapLeg> legs = {MakeLeg("HOMER"), MakeLeg("GIPNO"),
                                  MakeLeg("RW06"), MakeLeg("MISSD")};
      legs[0].procedureRole = "iaf";
      legs[2].procedureRole = "mapt";
      legs[3].procedureRole = "mahp";
      return legs;
    }
    return {};
  }
};

// The reported bug: a STAR is already in the plan but its arrival block is NOT
// separately tracked (a sim / external-FMS import), and an earlier role-tag
// inference left the stored approach block spanning the STAR fixes. Loading an
// approach must NOT erase the STAR.
TEST(ArrivalLoadTest, LoadingApproachKeepsUntrackedStar) {
  FakeStarApproachSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Approach;
  state.selectedAirportIcao = "KFMY";
  state.selectedName = "RNAV06";
  state.selectedTransition = "HOMER";

  std::vector<MapLeg> legs = {MakeLeg("KJAX"), MakeLeg("DURTE"),
                              MakeLeg("INPIN"), MakeLeg("DEEDS"),
                              MakeLeg("SHFTY"), MakeLeg("KFMY")};
  legs[2].procedureRole = "trans";
  legs[3].procedureRole = "trans";
  legs[4].procedureRole = "trans";
  // Stale inferred approach block spans the STAR fixes (INPIN..KFMY).
  int approachStart = 2, approachCount = 4, cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0, departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;
  int arrivalStart = 0, arrivalCount = 0;  // arrival not tracked
  MapProcedure loadedArrival;
  PersistedLoadedApproach persistedArrival;
  std::string arrivalHeader;

  ProcedureMenuHost host = MakeHost(
      state, &nav, legs, approachStart, approachCount, cursorRow, loadedApproach,
      persistedApproach, approachHeader, departureStart, departureCount,
      loadedDeparture, persistedDeparture, departureHeader, arrivalStart,
      arrivalCount, loadedArrival, persistedArrival, arrivalHeader);

  procedureMenuLoadSelected(host, "RNAV06", "HOMER");

  const auto contains = [&](const std::string& id) {
    for (const MapLeg& l : legs)
      if (l.id == id) return true;
    return false;
  };
  EXPECT_TRUE(contains("INPIN")) << "STAR fix INPIN was erased";
  EXPECT_TRUE(contains("DEEDS")) << "STAR fix DEEDS was erased";
  EXPECT_TRUE(contains("SHFTY")) << "STAR fix SHFTY was erased";
  // The approach is appended at the tail.
  EXPECT_TRUE(contains("HOMER"));
  EXPECT_EQ(legs.back().id, "MISSD");
}

// A genuine prior approach (loaded after the destination airport) must still be
// replaced when a new approach is loaded, without disturbing the STAR.
TEST(ArrivalLoadTest, LoadingApproachReplacesPriorApproachKeepsStar) {
  FakeStarApproachSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Approach;
  state.selectedAirportIcao = "KFMY";
  state.selectedName = "RNAV06";
  state.selectedTransition = "HOMER";

  std::vector<MapLeg> legs = {
      MakeLeg("KJAX"),  MakeLeg("DURTE"), MakeLeg("INPIN"), MakeLeg("DEEDS"),
      MakeLeg("SHFTY"), MakeLeg("KFMY"),  MakeLeg("OLDIA"), MakeLeg("OLDFA"),
      MakeLeg("RW13"),  MakeLeg("MISAP")};
  legs[2].procedureRole = "trans";
  legs[3].procedureRole = "trans";
  legs[4].procedureRole = "trans";
  legs[6].procedureRole = "iaf";
  legs[8].procedureRole = "mapt";
  legs[9].procedureRole = "mahp";
  int approachStart = 6, approachCount = 4, cursorRow = 0;
  MapProcedure loadedApproach;
  loadedApproach.type = ProcedureType::Approach;
  loadedApproach.name = "RNAV13";  // a real approach was previously loaded
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0, departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;
  int arrivalStart = 2, arrivalCount = 3;
  MapProcedure loadedArrival;
  PersistedLoadedApproach persistedArrival;
  std::string arrivalHeader;

  ProcedureMenuHost host = MakeHost(
      state, &nav, legs, approachStart, approachCount, cursorRow, loadedApproach,
      persistedApproach, approachHeader, departureStart, departureCount,
      loadedDeparture, persistedDeparture, departureHeader, arrivalStart,
      arrivalCount, loadedArrival, persistedArrival, arrivalHeader);

  procedureMenuLoadSelected(host, "RNAV06", "HOMER");

  const auto contains = [&](const std::string& id) {
    for (const MapLeg& l : legs)
      if (l.id == id) return true;
    return false;
  };
  // STAR survives, prior approach removed, new approach appended.
  EXPECT_TRUE(contains("INPIN"));
  EXPECT_TRUE(contains("SHFTY"));
  EXPECT_FALSE(contains("OLDIA")) << "prior approach was not replaced";
  EXPECT_FALSE(contains("OLDFA")) << "prior approach was not replaced";
  EXPECT_TRUE(contains("HOMER"));
  EXPECT_EQ(legs.back().id, "MISSD");
}

}  // namespace
}  // namespace avionics::test
