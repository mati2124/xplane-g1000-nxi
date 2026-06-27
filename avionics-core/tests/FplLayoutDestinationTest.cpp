#include "avionics/FplRouteEdit.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id) {
  MapLeg leg;
  leg.id = id;
  return leg;
}

TEST(FplLayoutDestinationTest, LayoutDestinationFilledRequiresAirport) {
  const std::vector<MapLeg> fixEnding = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"), makeLeg("JINOS"),
      makeLeg("TEBOW")};
  EXPECT_TRUE(fplDestinationFilledFromLegCount(
      static_cast<int>(fixEnding.size())));
  EXPECT_FALSE(fplLayoutDestinationFilled(fixEnding, true, 0));

  const std::vector<MapLeg> airportEnding = {makeLeg("KFMY"), makeLeg("KJAX")};
  EXPECT_TRUE(fplLayoutDestinationFilled(airportEnding, true, 0));
}

TEST(FplLayoutDestinationTest, SectionLayoutKeepsFixInEnroute) {
  const pfd::FplSectionLayout layout = pfd::fplSectionLayout(5, false);
  EXPECT_EQ(layout.destLegIndex, -1);
  EXPECT_EQ(layout.enrouteCount, 4);
}

TEST(FplLayoutDestinationTest, ProcedureDisplaySelectableRowFindsDestinationAirport) {
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"), makeLeg("JINOS"),
      makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  const bool destFilled = fplLayoutDestinationFilled(legs, true, 0);
  ASSERT_TRUE(destFilled);
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 1, 1, "KFMY-RW05.CSHEL8", 0, 0, std::string(), 0, 0, true,
      destFilled);
  bool sawKjaxDestination = false;
  for (const pfd::FplDisplayRow& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::Destination && dr.legIndex == 5) {
      sawKjaxDestination = true;
    }
  }
  EXPECT_TRUE(sawKjaxDestination);
  const int kjaxRow = pfd::fplProcedureSelectableRowForLegIndex(
      5, legs, 1, 1, "KFMY-RW05.CSHEL8", 0, 0, std::string(), 0, 0, true,
      destFilled);
  EXPECT_GE(kjaxRow, 0);
}

TEST(FplLayoutDestinationTest, ProcedureRowsKeepDestinationAirportOutOfEnroute) {
  // KFMY (CSHEL8 SID) -> ... -> KJAX, then RNAV 08 (WADOR iaf). With a
  // departure loaded the procedure-display path is used; the destination
  // airport KJAX must stay in the approach header, never under Enroute.
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"), makeLeg("JINOS"),
      makeLeg("TEBOW"), makeLeg("KJAX"), makeLeg("WADOR"), makeLeg("AMXUQ")};
  legs[6].procedureRole = "iaf";
  legs[7].procedureRole = "faf";
  const int depStart = 1;
  const int depCount = 1;
  const int approachStart = 6;
  const int approachCount = 2;
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, depStart, depCount, "RW05.CSHEL8", -1, 0, std::string(),
      approachStart, approachCount, /*blankOriginSection=*/true,
      /*destinationFilled=*/true);

  for (const pfd::FplDisplayRow& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::EnrouteLeg && dr.legIndex >= 0) {
      EXPECT_FALSE(isAirportIdent(legs[static_cast<std::size_t>(dr.legIndex)].id))
          << "airport leaked into Enroute: "
          << legs[static_cast<std::size_t>(dr.legIndex)].id;
      EXPECT_NE(dr.legIndex, 5) << "KJAX must not be an Enroute leg";
    }
  }
}

TEST(FplLayoutDestinationTest, ProcedureRowsFixEndingSidPlan) {
  const std::vector<MapLeg> legs = {makeLeg("KFMY"), makeLeg("CSHEL"),
                                    makeLeg("LAL"), makeLeg("JINOS"),
                                    makeLeg("TEBOW")};
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 1, 1, "RW05.CSHEL8", -1, 0, std::string(), -1, 0, true, false);

  bool sawTebowEnroute = false;
  bool sawEnrouteBlank = false;
  bool sawDestPlaceholder = false;
  bool sawDestBlank = false;
  bool sawFilledDestination = false;
  for (const pfd::FplDisplayRow& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::EnrouteLeg && dr.legIndex == 4) {
      sawTebowEnroute = true;
    }
    if (dr.kind == pfd::FplDisplayRowKind::EnrouteBlank) sawEnrouteBlank = true;
    if (dr.kind == pfd::FplDisplayRowKind::Destination && dr.legIndex < 0) {
      sawDestPlaceholder = true;
    }
    if (dr.kind == pfd::FplDisplayRowKind::DestinationBlank) {
      sawDestBlank = true;
    }
    if (dr.kind == pfd::FplDisplayRowKind::Destination && dr.legIndex >= 0) {
      sawFilledDestination = true;
    }
  }
  EXPECT_TRUE(sawTebowEnroute);
  EXPECT_TRUE(sawEnrouteBlank);
  EXPECT_TRUE(sawDestPlaceholder);
  EXPECT_TRUE(sawDestBlank);
  EXPECT_FALSE(sawFilledDestination);
}

TEST(FplLayoutDestinationTest, ProcedureInsertOnEnrouteBlankAppendsFix) {
  const std::vector<MapLeg> legs = {makeLeg("KFMY"), makeLeg("CSHEL"),
                                    makeLeg("LAL"), makeLeg("JINOS"),
                                    makeLeg("TEBOW")};
  int enrouteBlankSelectable = -1;
  int sel = 0;
  for (const pfd::FplDisplayRow& dr : pfd::fplProcedureDisplayRowList(
           legs, 1, 1, "RW05.CSHEL8", -1, 0, std::string(), -1, 0, true,
           false)) {
    if (!pfd::fplProcedureDisplayRowSelectable(dr.kind)) continue;
    if (dr.kind == pfd::FplDisplayRowKind::EnrouteBlank) {
      enrouteBlankSelectable = sel;
      break;
    }
    ++sel;
  }
  ASSERT_GE(enrouteBlankSelectable, 0);
  const int insertRow = pfd::fplProcedureInsertIndexForSelectable(
      enrouteBlankSelectable, legs, 1, 1, "RW05.CSHEL8", -1, 0, std::string(),
      -1, 0, static_cast<int>(legs.size()), true, false);
  EXPECT_EQ(insertRow, static_cast<int>(legs.size()));
}

TEST(FplLayoutDestinationTest, RemovingLastDepartureLegClearsDeparture) {
  std::vector<MapLeg> legs = {makeLeg("KFMY"), makeLeg("CSHEL"),
                              makeLeg("LAL"), makeLeg("TEBOW")};
  bool destinationFilled = true;
  int approachLegStart = 0;
  int approachLegCount = 0;
  int cursorRow = 0;
  int depStart = 1;
  int depCount = 1;
  std::string depHeader = "RW05.CSHEL8";
  MapProcedure loadedDeparture;
  loadedDeparture.name = "CSHEL8";
  int arrStart = 0;
  int arrCount = 0;
  std::string arrHeader;
  MapProcedure loadedArrival;

  FplRouteEdit edit{legs, destinationFilled, approachLegStart,
                    approachLegCount, cursorRow};
  fplRouteEditWireTerminalProcedures(edit, depStart, depCount, depHeader,
                                     arrStart, arrCount, arrHeader,
                                     &loadedDeparture, &loadedArrival);

  // Remove the single departure leg (CSHEL at index 1).
  EXPECT_TRUE(fplRemoveLegAtIndex(edit, 1));
  EXPECT_EQ(depCount, 0);
  EXPECT_EQ(depStart, 0);
  EXPECT_TRUE(depHeader.empty());
  EXPECT_TRUE(loadedDeparture.name.empty());
}

}  // namespace
}  // namespace avionics
