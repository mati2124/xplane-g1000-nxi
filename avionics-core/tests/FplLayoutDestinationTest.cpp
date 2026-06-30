#include "avionics/FplRouteEdit.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/SoftkeyController.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id) {
  MapLeg leg;
  leg.id = id;
  return leg;
}

// A loaded nav database that resolves nothing -- models a session whose nav data
// is ready but does not contain a small destination airport (e.g. K1H2).
class EmptyReadyNavSource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapFeature> lookupIdent(const std::string&,
                                      std::size_t) const override {
    return {};
  }
  std::string firstIdentWithPrefix(const std::string&) const override {
    return {};
  }
};

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

TEST(FplLayoutDestinationTest, AirportCodeWithDigitClassification) {
  // The strict four-letter check stays strict (used for procedure-block
  // boundaries); the looser format / digit helpers recognise airport codes that
  // contain digits like K1H2 without being fooled by enroute fixes.
  EXPECT_FALSE(isAirportIdent("K1H2"));
  EXPECT_TRUE(isAirportCodeFormat("K1H2"));
  EXPECT_TRUE(isAirportCodeWithDigit("K1H2"));
  EXPECT_TRUE(isAirportCodeWithDigit("KX01"));
  EXPECT_FALSE(isAirportCodeWithDigit("KJAX"));   // no digit
  EXPECT_FALSE(isAirportCodeWithDigit("TEBOW"));  // five-letter enroute fix
  EXPECT_FALSE(isAirportCodeWithDigit("BNA"));    // three-letter VOR

  // Small US fields with no ICAO code keep their three-character FAA local
  // identifier (1H2, 06C); these contain a digit and must read as airports, but
  // a plain three-letter VOR/NDB ident must not.
  EXPECT_TRUE(isAirportCodeFormat("1H2"));
  EXPECT_TRUE(isAirportCodeWithDigit("1H2"));
  EXPECT_TRUE(isAirportCodeWithDigit("06C"));
  EXPECT_FALSE(isAirportIdent("1H2"));          // not four letters
  EXPECT_FALSE(isAirportCodeFormat("BNA"));     // three-letter VOR, no digit
  EXPECT_FALSE(isAirportCodeWithDigit("LAL"));  // three-letter VOR, no digit
}

TEST(FplLayoutDestinationTest, LayoutDestinationFilledAcceptsFaaLocalIdAirport) {
  // The reported bug: entering the small field 1H2 (a three-character FAA local
  // identifier, no ICAO code) as the destination must place it under
  // Destination, not Enroute.
  const std::vector<MapLeg> legs = {makeLeg("KBNA"), makeLeg("1H2")};
  EXPECT_TRUE(fplLayoutDestinationFilled(legs, true, 0));
}

TEST(FplLayoutDestinationTest, ControllerDestinationFaaLocalIdNoNavDb) {
  // 1H2 as the route endpoint, nav data loaded but not containing the field,
  // must still place it under Destination.
  EmptyReadyNavSource nav;
  SoftkeyController ui;
  ui.setNavFeatureSource(&nav);
  ui.replaceFlightPlanFromExternal({makeLeg("KBNA"), makeLeg("1H2")});
  EXPECT_TRUE(ui.flightPlanDestinationFilledForLayout());
}

TEST(FplLayoutDestinationTest, LayoutDestinationFilledAcceptsDigitAirport) {
  // KBNA -> ... -> K1H2: the layout flag must treat K1H2 as the destination, not
  // an enroute fix, even though the four-letter strict check rejects it.
  const std::vector<MapLeg> legs = {makeLeg("KBNA"), makeLeg("GHM"),
                                    makeLeg("K1H2")};
  EXPECT_TRUE(fplLayoutDestinationFilled(legs, true, 0));
}

TEST(FplLayoutDestinationTest, ControllerDestinationDigitAirportNoNavDb) {
  // Reproduces the reported bug: a SimBrief plan ending at the small field K1H2,
  // with nav data loaded but not containing K1H2, must still place K1H2 under
  // Destination (flightPlanDestinationFilledForLayout() == true).
  EmptyReadyNavSource nav;
  SoftkeyController ui;
  ui.setNavFeatureSource(&nav);
  ui.replaceFlightPlanFromExternal(
      {makeLeg("KBNA"), makeLeg("GHM"), makeLeg("K1H2")});
  EXPECT_TRUE(ui.flightPlanDestinationFilledForLayout());

  // A fix-ending route (no airport) must still NOT be destination-filled.
  ui.replaceFlightPlanFromExternal(
      {makeLeg("KBNA"), makeLeg("GHM"), makeLeg("TEBOW")});
  EXPECT_FALSE(ui.flightPlanDestinationFilledForLayout());
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

TEST(FplLayoutDestinationTest, EnrouteBlankInsertKeepsDestinationAirportLast) {
  // KJAX (origin) -> KFMY (destination). Adding an enroute fix on the Enroute
  // "add a fix" row must insert before the destination airport, not after it,
  // so KFMY stays the destination instead of being pushed into Enroute.
  const std::vector<pfd::FplSectionRow> rows =
      pfd::buildFplSectionRows(2, /*destinationFilled=*/true);
  int enrouteBlankSelectable = -1;
  int sel = 0;
  for (const pfd::FplSectionRow& sr : rows) {
    if (!pfd::fplSectionRowIsSelectable(sr, 2, true)) continue;
    if (sr.kind == pfd::FplSectionRow::Kind::EnrouteBlank) {
      enrouteBlankSelectable = sel;
      break;
    }
    ++sel;
  }
  ASSERT_GE(enrouteBlankSelectable, 0);
  const int insertRow = pfd::fplSectionInsertIndexForSelectable(
      enrouteBlankSelectable, rows, 2, true);
  EXPECT_EQ(insertRow, 1) << "enroute fix should insert before the destination";
}

TEST(FplLayoutDestinationTest, CommitEnrouteFixKeepsDestinationAirport) {
  std::vector<MapLeg> legs = {makeLeg("KJAX"), makeLeg("KFMY")};
  bool destinationFilled = true;
  int approachLegStart = 0;
  int approachLegCount = 0;
  int cursorRow = 0;
  FplRouteEdit edit{legs, destinationFilled, approachLegStart, approachLegCount,
                    cursorRow};

  // Cursor on the Enroute "add a fix" blank slot (selectable row 1: Origin=0,
  // EnrouteBlank=1, Destination=2).
  const int enrouteBlankRow = 1;
  MapFeature match;
  match.id = "JAYJA";
  ASSERT_TRUE(fplCommitWaypointIdent(edit, /*navSource=*/nullptr, match, "JAYJA",
                                     enrouteBlankRow, /*approachAirport=*/{},
                                     FplCursorLayout::SectionRows));

  ASSERT_EQ(legs.size(), 3u);
  EXPECT_EQ(legs[0].id, "KJAX");
  EXPECT_EQ(legs[1].id, "JAYJA");
  EXPECT_EQ(legs[2].id, "KFMY") << "destination airport must stay last";
  EXPECT_TRUE(fplLayoutDestinationFilled(legs, destinationFilled, 0));
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
