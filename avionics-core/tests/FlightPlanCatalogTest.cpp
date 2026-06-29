#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanCatalog.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/SimBriefOfpSupport.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

MapLeg MakeLeg(const std::string& id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

std::vector<MapLeg> SampleRoute() {
  return {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
}

SimBriefOfpImport SampleSimBriefImport() {
  SimBriefOfpImport imp;
  imp.legs = SampleRoute();
  imp.originIcao = "KFMY";
  imp.destinationIcao = "KCMI";
  return imp;
}

// ---- FlightPlanCatalog model --------------------------------------------

TEST(FlightPlanCatalogModelTest, AddFromLegsStoresOriginDestAndCount) {
  FlightPlanCatalog cat;
  const int idx = cat.addPlanFromLegs(SampleRoute());

  ASSERT_EQ(idx, 0);
  ASSERT_EQ(cat.size(), 1);
  const PersistedFlightPlan& entry = cat.plan(0);
  EXPECT_EQ(FlightPlanCatalog::originIdent(entry), "KFMY");
  EXPECT_EQ(FlightPlanCatalog::destIdent(entry), "KCMI");
  EXPECT_EQ(FlightPlanCatalog::legCount(entry), 3);
}

TEST(FlightPlanCatalogModelTest, InvertSwapsOriginAndDestination) {
  PersistedFlightPlan entry = FlightPlanCatalog::makeEntryFromLegs(SampleRoute());
  const PersistedFlightPlan inv = FlightPlanCatalog::inverted(entry);

  EXPECT_EQ(FlightPlanCatalog::originIdent(inv), "KCMI");
  EXPECT_EQ(FlightPlanCatalog::destIdent(inv), "KFMY");
  EXPECT_EQ(FlightPlanCatalog::legCount(inv), 3);
}

TEST(FlightPlanCatalogModelTest, RemovePlanShrinksTheCatalog) {
  FlightPlanCatalog cat;
  cat.addPlanFromLegs(SampleRoute());
  cat.addPlanFromLegs(SampleRoute());
  ASSERT_EQ(cat.size(), 2);

  EXPECT_TRUE(cat.removePlan(0));
  EXPECT_EQ(cat.size(), 1);
  EXPECT_FALSE(cat.removePlan(5));  // out of range
}

TEST(FlightPlanCatalogModelTest, SimBriefReimportUpdatesExistingSlot) {
  FlightPlanCatalog cat;
  const SimBriefOfpImport imp = SampleSimBriefImport();

  EXPECT_EQ(cat.addPlanFromSimBriefImport(imp), 0);
  ASSERT_EQ(cat.size(), 1);

  SimBriefOfpImport again = imp;
  again.sidIdent = "DCT1";
  again.sidTrans = "TRANS";
  EXPECT_EQ(cat.addPlanFromSimBriefImport(again), 0);
  EXPECT_EQ(cat.size(), 1);
  EXPECT_EQ(cat.plan(0).departureMeta.name, "DCT1");
}

TEST(FlightPlanCatalogModelTest, SimBriefImportWithDifferentRouteAddsSlot) {
  FlightPlanCatalog cat;
  EXPECT_EQ(cat.addPlanFromSimBriefImport(SampleSimBriefImport()), 0);

  SimBriefOfpImport other = SampleSimBriefImport();
  other.legs.back().id = "KORD";
  other.destinationIcao = "KORD";
  EXPECT_EQ(cat.addPlanFromSimBriefImport(other), 1);
  EXPECT_EQ(cat.size(), 2);
}

TEST(FlightPlanCatalogModelTest, DedupeByRouteCollapsesDuplicates) {
  FlightPlanCatalog cat;
  cat.addPlanFromLegs(SampleRoute());
  cat.addPlanFromLegs(SampleRoute());
  cat.addPlanFromLegs(SampleRoute());
  ASSERT_EQ(cat.size(), 3);

  EXPECT_EQ(cat.dedupeByRoute(), 2);
  EXPECT_EQ(cat.size(), 1);
}

TEST(FlightPlanCatalogTest, ImportingAnEmptyRouteStoresNothing) {
  // The SimBrief / OFP import path refuses to create an empty stored plan (the
  // "New" softkey is what makes a blank slot, not an import).
  MfdController ui;
  EXPECT_EQ(ui.storeFlightPlanInCatalog({}), -1);
  EXPECT_TRUE(ui.flightPlanCatalog().empty());
}

// ---- MfdController catalog integration -----------------------------------

TEST(FlightPlanCatalogTest, ImportStoresWithoutActivating) {
  MfdController ui;
  const int idx = ui.storeFlightPlanInCatalog(SampleRoute());

  EXPECT_EQ(idx, 0);
  EXPECT_EQ(ui.flightPlanCatalog().size(), 1);

  // The import must NOT touch the active route / map (no published edit).
  EXPECT_TRUE(ui.fplLegs().empty());
  std::vector<MapLeg> published;
  EXPECT_FALSE(ui.consumeFlightPlanEdit(published));

  // The shell is told the catalog changed so it can persist it.
  EXPECT_TRUE(ui.consumeCatalogDirty());
  EXPECT_FALSE(ui.consumeCatalogDirty());  // latch clears after read
}

TEST(FlightPlanCatalogTest, ActivateLoadsStoredPlanIntoActiveRoute) {
  MfdController ui;
  ui.storeFlightPlanInCatalog(SampleRoute());

  ASSERT_TRUE(ui.catalogActivateSelected());

  // Now the active plan carries the route, and the shell gets a published edit
  // to push to the sim / map.
  EXPECT_EQ(ui.fplLegs().size(), 3u);
  std::vector<MapLeg> published;
  ASSERT_TRUE(ui.consumeFlightPlanEdit(published));
  EXPECT_EQ(published.size(), 3u);
  EXPECT_EQ(published.front().id, "KFMY");
  EXPECT_EQ(published.back().id, "KCMI");
}

TEST(FlightPlanCatalogTest, InvertActivateReversesTheRoute) {
  MfdController ui;
  ui.storeFlightPlanInCatalog(SampleRoute());

  ASSERT_TRUE(ui.catalogInvertActivateSelected());

  std::vector<MapLeg> published;
  ASSERT_TRUE(ui.consumeFlightPlanEdit(published));
  ASSERT_EQ(published.size(), 3u);
  EXPECT_EQ(published.front().id, "KCMI");
  EXPECT_EQ(published.back().id, "KFMY");
}

TEST(FlightPlanCatalogTest, CopyAddsADuplicateSlot) {
  MfdController ui;
  ui.storeFlightPlanInCatalog(SampleRoute());

  const int copyIdx = ui.catalogCopySelected();
  EXPECT_EQ(copyIdx, 1);
  EXPECT_EQ(ui.flightPlanCatalog().size(), 2);
}

TEST(FlightPlanCatalogTest, SimBriefReimportDoesNotGrowCatalog) {
  MfdController ui;
  const SimBriefOfpImport imp = SampleSimBriefImport();

  EXPECT_EQ(ui.storeFlightPlanFromSimBriefImport(imp), 0);
  EXPECT_TRUE(ui.consumeCatalogDirty());
  EXPECT_EQ(ui.storeFlightPlanFromSimBriefImport(imp), 0);
  EXPECT_EQ(ui.flightPlanCatalog().size(), 1);
  EXPECT_TRUE(ui.consumeCatalogDirty());
}

TEST(FlightPlanCatalogTest, RestoreDedupesLegacyDuplicateImports) {
  std::vector<PersistedFlightPlan> snapshot;
  snapshot.push_back(FlightPlanCatalog::makeEntryFromLegs(SampleRoute()));
  snapshot.push_back(FlightPlanCatalog::makeEntryFromLegs(SampleRoute()));
  snapshot.push_back(FlightPlanCatalog::makeEntryFromLegs(SampleRoute()));
  ASSERT_EQ(snapshot.size(), 3u);

  MfdController dst;
  dst.restoreFlightPlanCatalog(snapshot);
  EXPECT_EQ(dst.flightPlanCatalog().size(), 1);
  EXPECT_TRUE(dst.consumeCatalogDirty());
}

TEST(FlightPlanCatalogTest, DeleteRemovesTheSelectedSlot) {
  MfdController ui;
  ui.storeFlightPlanInCatalog(SampleRoute());
  ui.storeFlightPlanInCatalog(SampleRoute());
  ASSERT_EQ(ui.flightPlanCatalog().size(), 2);

  EXPECT_TRUE(ui.catalogDeleteSelected());
  EXPECT_EQ(ui.flightPlanCatalog().size(), 1);

  ui.catalogDeleteAll();
  EXPECT_TRUE(ui.flightPlanCatalog().empty());
}

TEST(FlightPlanCatalogTest, RestoreRoundTripsStoredPlans) {
  MfdController src;
  src.storeFlightPlanInCatalog(SampleRoute());
  const std::vector<PersistedFlightPlan> snapshot =
      src.flightPlanCatalogSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);

  MfdController dst;
  dst.restoreFlightPlanCatalog(snapshot);
  ASSERT_EQ(dst.flightPlanCatalog().size(), 1);
  EXPECT_EQ(FlightPlanCatalog::originIdent(dst.flightPlanCatalog().plan(0)),
            "KFMY");
}

// ---- Catalog page is the FPL group's 2nd page ----------------------------

TEST(FlightPlanCatalogTest, IsSecondPageOfFplGroup) {
  MfdController ui;
  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.pageGroup(), MfdPageGroup::FlightPlan);
  EXPECT_EQ(ui.page(), MfdPage::ActiveFlightPlan);
  EXPECT_GE(ui.pageCount(MfdPageGroup::FlightPlan), 2);

  // The small FMS knob steps within the group to the Flight Plan Catalog.
  ui.pressBezelKey(BezelKey::FmsInnerCw);
  EXPECT_EQ(ui.page(), MfdPage::FlightPlanCatalog);
}

}  // namespace
}  // namespace avionics
