#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "avionics/CifpParser.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FmsNavigator.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/ProcedureSupport.h"
#include "avionics/SimBriefOfpSupport.h"
#include "render/pfd/PfdFlightPlanSections.h"

#include "CifpFixLookup.h"

namespace avionics::test {
namespace {

MapLeg MakeLeg(const std::string& id, double lat = 0.0, double lon = 0.0) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

CifpAirportProcedures makeCshel6SidData() {
  CifpAirportProcedures data;
  data.icao = "KFMY";
  data.runways["RW05"] = {26.5803025, -81.8672716};
  data.runways["RW23"] = {26.6000000, -81.8500000};

  auto addLeg = [&](int seq, const std::string& routeType,
                    const std::string& transition,
                    const std::string& pathTerm, const std::string& fixIdent,
                    int alt1Ft = 0, float courseDeg = 0.0f) {
    CifpLeg leg;
    leg.kind = ProcedureType::Departure;
    leg.sequence = seq;
    leg.routeType = routeType;
    leg.procedureName = "CSHEL6";
    leg.transition = transition;
    leg.pathTerminator = pathTerm;
    leg.fixIdent = fixIdent;
    leg.altitude1Ft = alt1Ft;
    leg.magneticCourseDeg = courseDeg;
    data.legs.push_back(leg);
  };

  addLeg(10, "2", "RW05", "CA", "", 420, 51.0f);
  addLeg(20, "2", "RW05", "VM", "", 0, 51.0f);
  addLeg(30, "2", "RW05", "IF", "CSHEL");
  addLeg(40, "5", "LAL", "IF", "CSHEL");
  addLeg(50, "5", "LAL", "TF", "JUNLO");
  return data;
}

std::unordered_map<std::string, std::pair<double, double>> cshel6FixTable() {
  std::unordered_map<std::string, std::pair<double, double>> fixes;
  fixes["RW05"] = {26.5803025, -81.8672716};
  fixes["RW23"] = {26.6000000, -81.8500000};
  fixes["CSHEL"] = {26.6500000, -81.9000000};
  fixes["JUNLO"] = {26.7000000, -81.9500000};
  return fixes;
}

int LegIndexById(const std::vector<MapLeg>& legs, const std::string& id) {
  for (std::size_t i = 0; i < legs.size(); ++i) {
    if (legs[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

TEST(DepartureLoadTest, ExpandSidRunwayTransitionIncludesLeadingRows) {
  const CifpAirportProcedures data = makeCshel6SidData();
  auto fixes = cshel6FixTable();
  const std::vector<MapLeg> legs =
      expandCifpProcedure(data, ProcedureType::Departure, "CSHEL6", "RW05",
                          mapFixLookup, &fixes);
  ASSERT_GE(legs.size(), 4u);
  EXPECT_EQ(legs[0].id, "RW05");
  EXPECT_EQ(legs[1].id, "420FT");
  EXPECT_EQ(legs[1].pathTerminator, "CA");
  EXPECT_EQ(legs[2].id, "MANSEQ");
  EXPECT_EQ(legs[2].pathTerminator, "VM");
  EXPECT_GE(LegIndexById(legs, "CSHEL"), 3);
}

TEST(DepartureLoadTest, DepartureBlockExcludedFromEnrouteIndices) {
  std::vector<MapLeg> plan = {MakeLeg("KFMY"), MakeLeg("RW05"), MakeLeg("420FT"),
                              MakeLeg("MANSEQ"), MakeLeg("CSHEL"),
                              MakeLeg("JUNLO"), MakeLeg("JINOS"), MakeLeg("KJAX")};
  plan[1].pathTerminator = "";
  plan[2].pathTerminator = "CA";
  plan[2].altitudeConstraintFt = 420;
  plan[3].pathTerminator = "VM";
  plan[3].id = "MANSEQ";

  const auto enroute = pfd::fplProcedureEnrouteLegIndices(
      plan, /*depStart=*/1, /*depCount=*/5, /*arrStart=*/0, /*arrCount=*/0,
      /*approachStart=*/0, /*approachCount=*/0, /*destinationFilled=*/true);
  ASSERT_EQ(enroute.size(), 1u);
  EXPECT_EQ(plan[static_cast<std::size_t>(enroute[0])].id, "JINOS");
}

TEST(DepartureLoadTest, ProcedureDisplayRowsShowDepartureHeader) {
  std::vector<MapLeg> plan = {MakeLeg("KFMY"), MakeLeg("RW05"), MakeLeg("420FT"),
                              MakeLeg("MANSEQ"), MakeLeg("CSHEL"),
                              MakeLeg("JUNLO"), MakeLeg("JINOS"), MakeLeg("KJAX")};

  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, 1, 5, "RW05.CSHEL6.LAL", 0, 0, "", 0, 0,
      /*blankOriginSection=*/false, /*destinationFilled=*/true,
      /*airwaysCollapsed=*/false);

  EXPECT_EQ(rows[0].kind, pfd::FplDisplayRowKind::DepartureHeader);
  bool sawEnrouteLabel = false;
  bool cshelUnderDeparture = false;
  for (const auto& row : rows) {
    if (row.kind == pfd::FplDisplayRowKind::EnrouteLabel) {
      sawEnrouteLabel = true;
    }
    if (row.kind == pfd::FplDisplayRowKind::DepartureLeg &&
        plan[static_cast<std::size_t>(row.legIndex)].id == "CSHEL") {
      cshelUnderDeparture = true;
    }
  }
  EXPECT_TRUE(cshelUnderDeparture);
  EXPECT_TRUE(sawEnrouteLabel);
}

TEST(DepartureLoadTest, FmsSequencesClimbLegAtAltitude) {
  MapLeg rw = MakeLeg("RW05", 26.58, -81.87);
  MapLeg climb = MakeLeg("420FT", 26.581, -81.869);
  climb.pathTerminator = "CA";
  climb.legCourseDeg = 51.0f;
  climb.altitudeConstraintFt = 420;
  climb.altitudeConstraint = AltConstraintType::AtOrAbove;
  MapLeg manseq = MakeLeg("MANSEQ", 26.582, -81.868);
  manseq.pathTerminator = "VM";
  manseq.legCourseDeg = 51.0f;
  MapLeg cshel = MakeLeg("CSHEL", 26.65, -81.90);

  FmsNavigator nav;
  nav.setFlightPlan({rw, climb, manseq, cshel});
  nav.setActiveLegIndex(1);

  nav.update(26.581, -81.869, 120.0f, 500.0f);
  EXPECT_EQ(nav.activeLegIndex(), 2);

  NavigationSolution sol = nav.update(26.582, -81.868, 120.0f, 500.0f);
  EXPECT_TRUE(sol.active);
  EXPECT_NEAR(sol.desiredTrackDeg, 51.0f, 0.5f);
}

TEST(DepartureLoadTest, TerminalHeaderLabelIncludesRunwayAndTransition) {
  EXPECT_EQ(formatTerminalProcedureFplHeaderLabel("05", "CSHEL6", "LAL"),
            "RW05.CSHEL6.LAL");
}

// Direct-To from a synthetic departure row (RWxx threshold, climb-to-altitude,
// or vector/heading leg) must resolve to the next real fix in the plan.
TEST(DepartureLoadTest, DirectToSkipsNonFixDepartureLegsToNextFix) {
  MapLeg rw = MakeLeg("RW08");
  MapLeg climb = MakeLeg("540FT");
  climb.pathTerminator = "CA";
  MapLeg manseq = MakeLeg("MANSEQ");
  manseq.pathTerminator = "VM";
  const std::vector<MapLeg> plan = {rw, climb, manseq, MakeLeg("FELTZ"),
                                    MakeLeg("JAYJA")};

  EXPECT_TRUE(isNonFixDepartureLeg(rw));
  EXPECT_TRUE(isNonFixDepartureLeg(climb));
  EXPECT_TRUE(isNonFixDepartureLeg(manseq));
  EXPECT_FALSE(isNonFixDepartureLeg(plan[3]));

  // Cursor on the runway, climb, or vector leg all resolve forward to FELTZ.
  EXPECT_EQ(nextNavigableFixLegIndex(plan, 0), 3);
  EXPECT_EQ(nextNavigableFixLegIndex(plan, 1), 3);
  EXPECT_EQ(nextNavigableFixLegIndex(plan, 2), 3);
  // A real fix resolves to itself.
  EXPECT_EQ(nextNavigableFixLegIndex(plan, 3), 3);
  EXPECT_EQ(nextNavigableFixLegIndex(plan, 4), 4);
}

// Fake nav that expands a single SID (JETIN2 off KJAX RW08), used to exercise
// the interactive PROC departure load path end-to-end.
class FakeDepartureSource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    if (icao != "KJAX" || type != ProcedureType::Departure) return {};
    MapProcedure proc;
    proc.type = ProcedureType::Departure;
    proc.name = "JETIN2";
    proc.transition = "JAYJA";
    proc.runway = "08";
    return {proc};
  }
  std::vector<MapLeg> expandProcedure(
      const std::string& icao, ProcedureType type, const std::string& name,
      const std::string& transition) const override {
    if (icao != "KJAX" || type != ProcedureType::Departure ||
        name != "JETIN2") {
      return {};
    }
    if (transition == "RW08") {
      MapLeg climb = MakeLeg("540FT");
      climb.pathTerminator = "CA";
      climb.altitudeConstraintFt = 540;
      MapLeg manseq = MakeLeg("MANSEQ");
      manseq.pathTerminator = "VM";
      return {MakeLeg("RW08"), climb, manseq, MakeLeg("JETIN"),
              MakeLeg("DURTE"), MakeLeg("JAYJA")};
    }
    if (transition == "JAYJA") {
      return {MakeLeg("JETIN"), MakeLeg("DURTE"), MakeLeg("JAYJA")};
    }
    return {};
  }
};

TEST(DepartureLoadTest, ProcLoadPlacesDepartureBlockAfterOrigin) {
  FakeDepartureSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Departure;
  state.selectedAirportIcao = "KJAX";
  state.selectedName = "JETIN2";
  state.selectedTransition = "JAYJA";
  state.selectedRunway = "RW08";

  std::vector<MapLeg> legs = {MakeLeg("KJAX"), MakeLeg("KFMY")};
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0;
  int departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;

  ProcedureMenuHost host{
      state,
      &nav,
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
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      []() -> std::string { return "KJAX"; },
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

  procedureMenuLoadSelected(host, "JETIN2", "JAYJA");

  // RW08/540FT/MANSEQ/JETIN/DURTE/JAYJA were spliced right after the origin
  // (KJAX), not at the tail of the plan.
  EXPECT_EQ(departureStart, 1);
  EXPECT_EQ(departureCount, 6);
  ASSERT_GE(static_cast<int>(legs.size()), departureStart + departureCount);
  EXPECT_EQ(legs[1].id, "RW08");
  EXPECT_EQ(legs[2].id, "540FT");
  EXPECT_EQ(legs[3].id, "MANSEQ");
  EXPECT_EQ(departureHeader, "RW08.JETIN2.JAYJA");

  // The departure legs must fall outside the enroute index set.
  const auto enroute = pfd::fplProcedureEnrouteLegIndices(
      legs, departureStart, departureCount, /*arrStart=*/0, /*arrCount=*/0,
      /*approachStart=*/0, /*approachCount=*/0, /*destinationFilled=*/true);
  for (int idx : enroute) {
    EXPECT_FALSE(idx >= departureStart && idx < departureStart + departureCount);
  }
}

TEST(DepartureLoadTest, ProcLoadAbsorbsDuplicateEnrouteFixes) {
  FakeDepartureSource nav;
  ProcedureMenuState state;
  state.category = ProcedureType::Departure;
  state.selectedAirportIcao = "KJAX";
  state.selectedName = "JETIN2";
  state.selectedTransition = "JAYJA";
  state.selectedRunway = "RW08";

  // Route imported with the SID's transition fixes already present (e.g. from
  // the sim's FMS), then KFMY destination and an enroute fix after the SID.
  std::vector<MapLeg> legs = {MakeLeg("KJAX"), MakeLeg("JETIN"),
                              MakeLeg("DURTE"), MakeLeg("JAYJA"),
                              MakeLeg("SAALR"), MakeLeg("KFMY")};
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedApproach;
  std::string approachHeader;
  int departureStart = 0;
  int departureCount = 0;
  MapProcedure loadedDeparture;
  PersistedLoadedApproach persistedDeparture;
  std::string departureHeader;

  ProcedureMenuHost host{
      state,
      &nav,
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
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      []() -> std::string { return "KJAX"; },
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

  procedureMenuLoadSelected(host, "JETIN2", "JAYJA");

  // The pre-existing JETIN/DURTE/JAYJA must be absorbed by the SID block - they
  // should appear exactly once each, inside the departure block, with the
  // post-SID enroute fix (SAALR) and destination (KFMY) preserved after.
  auto countId = [&](const std::string& id) {
    int n = 0;
    for (const MapLeg& l : legs) {
      if (l.id == id) ++n;
    }
    return n;
  };
  EXPECT_EQ(countId("JETIN"), 1);
  EXPECT_EQ(countId("DURTE"), 1);
  EXPECT_EQ(countId("JAYJA"), 1);
  EXPECT_EQ(departureStart, 1);
  EXPECT_EQ(departureCount, 6);

  // None of the SID fixes leak into the enroute section; only SAALR remains
  // enroute (KFMY is the destination).
  const auto enroute = pfd::fplProcedureEnrouteLegIndices(
      legs, departureStart, departureCount, /*arrStart=*/0, /*arrCount=*/0,
      /*approachStart=*/0, /*approachCount=*/0, /*destinationFilled=*/true);
  ASSERT_EQ(enroute.size(), 1u);
  EXPECT_EQ(legs[static_cast<std::size_t>(enroute[0])].id, "SAALR");

  // Display rows: JETIN/DURTE appear under the Departure header, not Enroute.
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, departureStart, departureCount, departureHeader, /*arrStart=*/0,
      /*arrCount=*/0, /*arrivalHeader=*/std::string(), /*approachStart=*/0,
      /*approachCount=*/0, /*blankOriginSection=*/true,
      /*destinationFilled=*/true);
  for (const pfd::FplDisplayRow& dr : rows) {
    if (dr.kind != pfd::FplDisplayRowKind::EnrouteLeg || dr.legIndex < 0) {
      continue;
    }
    const std::string& id = legs[static_cast<std::size_t>(dr.legIndex)].id;
    EXPECT_NE(id, "JETIN") << "JETIN must not render under Enroute";
    EXPECT_NE(id, "DURTE") << "DURTE must not render under Enroute";
  }
}

}  // namespace
}  // namespace avionics::test
