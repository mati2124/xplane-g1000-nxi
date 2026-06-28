#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg MakeLeg(const std::string& id, double lat = 0.0, double lon = 0.0) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

int CountKind(const std::vector<pfd::FplDisplayRow>& rows,
              pfd::FplDisplayRowKind kind) {
  int n = 0;
  for (const auto& row : rows) {
    if (row.kind == kind) ++n;
  }
  return n;
}

// ---- Display grouping (buildFplProcedureDisplayRows) ----

TEST(AirwayLoadTest, ExpandedListsEveryAirwayFix) {
  std::vector<MapLeg> legs = {MakeLeg("KFMY"), MakeLeg("JINOS")};
  legs.push_back(MakeLeg("BRUTS"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("JAMIZ"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("KJAX"));

  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 0, 0, "", 0, 0, "", 0, 0, /*blankOriginSection=*/false,
      /*destinationFilled=*/true, /*airwaysCollapsed=*/false);

  EXPECT_EQ(CountKind(rows, pfd::FplDisplayRowKind::AirwayHeader), 1);
  // Both airway fixes (BRUTS, JAMIZ) are listed plus JINOS and the endpoints.
  int brutsRows = 0;
  int jamizRows = 0;
  for (const auto& row : rows) {
    if (row.kind != pfd::FplDisplayRowKind::EnrouteLeg) continue;
    const std::string& id = legs[static_cast<std::size_t>(row.legIndex)].id;
    if (id == "BRUTS") ++brutsRows;
    if (id == "JAMIZ") ++jamizRows;
  }
  EXPECT_EQ(brutsRows, 1);
  EXPECT_EQ(jamizRows, 1);
}

TEST(AirwayLoadTest, CollapsedShowsOnlyExitFix) {
  std::vector<MapLeg> legs = {MakeLeg("KFMY"), MakeLeg("JINOS")};
  legs.push_back(MakeLeg("BRUTS"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("JAMIZ"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("KJAX"));

  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 0, 0, "", 0, 0, "", 0, 0, /*blankOriginSection=*/false,
      /*destinationFilled=*/true, /*airwaysCollapsed=*/true);

  EXPECT_EQ(CountKind(rows, pfd::FplDisplayRowKind::AirwayHeader), 1);
  // The intermediate fix (BRUTS) is hidden; only the exit fix (JAMIZ) shows.
  int brutsRows = 0;
  int jamizRows = 0;
  for (const auto& row : rows) {
    if (row.kind != pfd::FplDisplayRowKind::EnrouteLeg) continue;
    const std::string& id = legs[static_cast<std::size_t>(row.legIndex)].id;
    if (id == "BRUTS") ++brutsRows;
    if (id == "JAMIZ") ++jamizRows;
  }
  EXPECT_EQ(brutsRows, 0);
  EXPECT_EQ(jamizRows, 1);
}

TEST(AirwayLoadTest, AirwayHeaderPointsAtExitFix) {
  std::vector<MapLeg> legs = {MakeLeg("KFMY"), MakeLeg("JINOS")};
  legs.push_back(MakeLeg("BRUTS"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("JAMIZ"));
  legs.back().viaAirway = "Q118";
  legs.push_back(MakeLeg("KJAX"));

  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 0, 0, "", 0, 0, "", 0, 0, false, true, false);
  int headerLegIndex = -1;
  for (const auto& row : rows) {
    if (row.kind == pfd::FplDisplayRowKind::AirwayHeader) {
      headerLegIndex = row.legIndex;
    }
  }
  ASSERT_GE(headerLegIndex, 0);
  EXPECT_EQ(legs[static_cast<std::size_t>(headerLegIndex)].id, "JAMIZ");
  EXPECT_EQ(legs[static_cast<std::size_t>(headerLegIndex)].viaAirway, "Q118");
}

// ---- Controller Load Airway flow ----

class FakeAirwaySource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<std::string> airwaysThrough(
      const std::string& ident) const override {
    if (ident == "JINOS") return {"Q118"};
    return {};
  }
  std::vector<MapLeg> airwayFixes(const std::string& airwayName,
                                  const std::string& fromIdent) const override {
    if (airwayName == "Q118" && fromIdent == "JINOS") {
      return {MakeLeg("JINOS", 27.0, -82.0), MakeLeg("BRUTS", 28.0, -82.0),
              MakeLeg("JAMIZ", 29.0, -82.0)};
    }
    return {};
  }
  std::vector<MapLeg> expandAirway(const std::string& airwayName,
                                   const std::string& fromIdent,
                                   const std::string& toIdent) const override {
    if (airwayName != "Q118" || fromIdent != "JINOS") return {};
    if (toIdent == "BRUTS") return {MakeLeg("BRUTS", 28.0, -82.0)};
    if (toIdent == "JAMIZ") {
      return {MakeLeg("BRUTS", 28.0, -82.0), MakeLeg("JAMIZ", 29.0, -82.0)};
    }
    return {};
  }
};

TEST(AirwayLoadTest, OpenWindowSeedsAirwayAndExit) {
  FakeAirwaySource src;
  MfdController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  EXPECT_EQ(ui.loadAirwayEntryIdent(), "JINOS");
  EXPECT_EQ(ui.loadAirwayName(), "Q118");
  // The default exit is the first fix after the entry.
  EXPECT_EQ(ui.loadAirwayExitIdent(), "BRUTS");
  EXPECT_TRUE(ui.loadAirwayCanLoad());
}

TEST(AirwayLoadTest, NoAirwayThroughFixDoesNotOpen) {
  FakeAirwaySource src;
  MfdController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal({MakeLeg("KFMY"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("KFMY");
  EXPECT_FALSE(ui.loadAirwayWindowOpen());
}

TEST(AirwayLoadTest, LoadInsertsTaggedSegmentAfterEntry) {
  FakeAirwaySource src;
  MfdController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  // Move the field cursor to Exit and scroll to JAMIZ, then Load.
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Airway -> Exit
  EXPECT_EQ(ui.loadAirwayField(), MfdController::LoadAirwayField::Exit);
  ui.pressBezelKey(BezelKey::FmsInnerCw);  // BRUTS -> JAMIZ
  EXPECT_EQ(ui.loadAirwayExitIdent(), "JAMIZ");
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Exit -> Load
  EXPECT_EQ(ui.loadAirwayField(), MfdController::LoadAirwayField::Load);
  ui.pressBezelKey(BezelKey::Ent);         // Load?

  EXPECT_FALSE(ui.loadAirwayWindowOpen());
  const std::vector<MapLeg>& legs = ui.fplLegs();
  ASSERT_EQ(legs.size(), 5u);  // KFMY JINOS BRUTS JAMIZ KJAX
  EXPECT_EQ(legs[1].id, "JINOS");
  EXPECT_EQ(legs[2].id, "BRUTS");
  EXPECT_EQ(legs[2].viaAirway, "Q118");
  EXPECT_EQ(legs[3].id, "JAMIZ");
  EXPECT_EQ(legs[3].viaAirway, "Q118");
  EXPECT_TRUE(legs[1].viaAirway.empty());  // entry fix is not tagged
  EXPECT_TRUE(ui.fplHasAirwayLegs());
}

TEST(AirwayLoadTest, ClrCancelsWithoutEditing) {
  FakeAirwaySource src;
  MfdController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  ui.pressBezelKey(BezelKey::Clr);
  EXPECT_FALSE(ui.loadAirwayWindowOpen());
  EXPECT_EQ(ui.fplLegs().size(), 3u);
  EXPECT_FALSE(ui.fplHasAirwayLegs());
}

TEST(AirwayLoadTest, CollapseToggleReflectsState) {
  FakeAirwaySource src;
  MfdController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});
  ui.openLoadAirwayWindow("JINOS");
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Exit
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Load
  ui.pressBezelKey(BezelKey::Ent);         // load BRUTS

  ASSERT_TRUE(ui.fplHasAirwayLegs());
  EXPECT_FALSE(ui.fplAirwaysCollapsed());
}

// ---- PFD (SoftkeyController) Load Airway flow ----

TEST(AirwayLoadTest, PfdOpenWindowSeedsAirwayAndExit) {
  FakeAirwaySource src;
  SoftkeyController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  EXPECT_EQ(ui.loadAirwayEntryIdent(), "JINOS");
  EXPECT_EQ(ui.loadAirwayName(), "Q118");
  EXPECT_EQ(ui.loadAirwayExitIdent(), "BRUTS");
  EXPECT_TRUE(ui.loadAirwayCanLoad());
}

TEST(AirwayLoadTest, PfdNoAirwayThroughFixDoesNotOpen) {
  FakeAirwaySource src;
  SoftkeyController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal({MakeLeg("KFMY"), MakeLeg("KJAX")});

  ui.openLoadAirwayWindow("KFMY");
  EXPECT_FALSE(ui.loadAirwayWindowOpen());
}

TEST(AirwayLoadTest, PfdLoadInsertsTaggedSegmentAfterEntry) {
  FakeAirwaySource src;
  SoftkeyController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  // The window is a sub-mode of the FlightPlan popout, so open it first.
  ui.pressBezelKey(BezelKey::Fpl);
  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Airway -> Exit
  EXPECT_EQ(ui.loadAirwayField(), SoftkeyController::LoadAirwayField::Exit);
  ui.pressBezelKey(BezelKey::FmsInnerCw);  // BRUTS -> JAMIZ
  EXPECT_EQ(ui.loadAirwayExitIdent(), "JAMIZ");
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // Exit -> Load
  EXPECT_EQ(ui.loadAirwayField(), SoftkeyController::LoadAirwayField::Load);
  ui.pressBezelKey(BezelKey::Ent);         // Load?

  EXPECT_FALSE(ui.loadAirwayWindowOpen());
  const std::vector<MapLeg>& legs = ui.flightPlanLegs();
  ASSERT_EQ(legs.size(), 5u);  // KFMY JINOS BRUTS JAMIZ KJAX
  EXPECT_EQ(legs[2].id, "BRUTS");
  EXPECT_EQ(legs[2].viaAirway, "Q118");
  EXPECT_EQ(legs[3].id, "JAMIZ");
  EXPECT_EQ(legs[3].viaAirway, "Q118");
  EXPECT_TRUE(legs[1].viaAirway.empty());  // entry fix is not tagged
  EXPECT_TRUE(ui.flightPlanHasAirwayLegs());
}

TEST(AirwayLoadTest, PfdClrCancelsWithoutEditing) {
  FakeAirwaySource src;
  SoftkeyController ui;
  ui.setNavFeatureSource(&src);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KFMY"), MakeLeg("JINOS"), MakeLeg("KJAX")});

  ui.pressBezelKey(BezelKey::Fpl);
  ui.openLoadAirwayWindow("JINOS");
  ASSERT_TRUE(ui.loadAirwayWindowOpen());
  ui.pressBezelKey(BezelKey::Clr);
  EXPECT_FALSE(ui.loadAirwayWindowOpen());
  EXPECT_EQ(ui.flightPlanLegs().size(), 3u);
  EXPECT_FALSE(ui.flightPlanHasAirwayLegs());
}

}  // namespace
}  // namespace avionics
