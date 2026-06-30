#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/MfdController.h"
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

// Drives the MFD onto the Active Flight Plan page with a loaded route, opens the
// Page Menu, and selects "Delete Flight Plan". Returns with the confirmation
// window armed (OK highlighted).
void OpenDeleteFlightPlanConfirm(MfdController& ui) {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
  ui.syncFlightPlan(map, /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);  // Active Flight Plan page
  ASSERT_EQ(ui.pageGroup(), MfdPageGroup::FlightPlan);
  if (ui.page() != MfdPage::ActiveFlightPlan) {
    ui.pressBezelKey(BezelKey::FmsInnerCcw);
  }
  ASSERT_EQ(ui.page(), MfdPage::ActiveFlightPlan);
  ASSERT_FALSE(ui.fplLegs().empty());

  ui.pressBezelKey(BezelKey::Menu);  // open the Page Menu
  ASSERT_TRUE(ui.pageMenuOpen());

  // Step to the "Delete Flight Plan" row (the menu skips disabled rows).
  for (int i = 0; i < ui.pageMenuItemCount(); ++i) {
    if (ui.pageMenuItemText(ui.pageMenuSelected()) == "Delete Flight Plan") {
      break;
    }
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ASSERT_EQ(ui.pageMenuItemText(ui.pageMenuSelected()), "Delete Flight Plan");

  ui.pressBezelKey(BezelKey::Ent);  // open the confirmation
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::DeleteFlightPlan);
}

TEST(DeleteFlightPlanTest, ConfirmingOkClearsTheRoute) {
  MfdController ui;
  OpenDeleteFlightPlanConfirm(ui);

  ui.pressBezelKey(BezelKey::Ent);  // OK is highlighted by default

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_TRUE(ui.fplLegs().empty());

  std::vector<MapLeg> published;
  EXPECT_TRUE(ui.consumeFlightPlanEdit(published));
  EXPECT_TRUE(published.empty());
}

TEST(DeleteFlightPlanTest, ClearedRouteSurvivesStaleSimSync) {
  MfdController ui;
  OpenDeleteFlightPlanConfirm(ui);
  ui.pressBezelKey(BezelKey::Ent);

  std::vector<MapLeg> published;
  ASSERT_TRUE(ui.consumeFlightPlanEdit(published));

  // The shell has not yet pushed the empty route back, so the next sim snapshot
  // still carries the old plan. The deleted plan must stay empty (the local edit
  // owns the route until an external change replaces it).
  MapData stale;
  stale.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
  ui.syncFlightPlan(stale, /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  EXPECT_TRUE(ui.fplLegs().empty());
}

TEST(DeleteFlightPlanTest, ClearedRouteSurvivesStaleSimSyncAfterOffPlanDirectTo) {
  MfdController ui;
  const std::vector<MapLeg> plan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
  MapLeg offPlanVor = MakeLeg("RSW", 26.536, -81.755);

  MapData map;
  map.flightPlan = plan;
  map.directToActive = true;
  map.directTo = offPlanVor;
  ui.syncFlightPlan(map, /*activeWaypoint=*/{}, /*navDirectTo=*/true);
  ASSERT_TRUE(ui.fplLegs().empty());

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::Menu);
  for (int i = 0; i < ui.pageMenuItemCount(); ++i) {
    if (ui.pageMenuItemText(ui.pageMenuSelected()) == "Delete Flight Plan") {
      break;
    }
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ui.pressBezelKey(BezelKey::Ent);
  ui.pressBezelKey(BezelKey::Ent);

  std::vector<MapLeg> published;
  ASSERT_TRUE(ui.consumeFlightPlanEdit(published));
  EXPECT_TRUE(published.empty());
  EXPECT_TRUE(ui.fplLocalDraft());

  map.directToActive = false;
  map.directTo = {};
  map.flightPlan = plan;
  ui.syncFlightPlan(map, /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  EXPECT_TRUE(ui.fplLegs().empty());
}

TEST(DeleteFlightPlanTest, CancelKeepsTheRoute) {
  MfdController ui;
  OpenDeleteFlightPlanConfirm(ui);

  ui.pressBezelKey(BezelKey::FmsInnerCw);  // highlight CANCEL
  ui.pressBezelKey(BezelKey::Ent);

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_EQ(ui.fplLegs().size(), 3u);

  std::vector<MapLeg> published;
  EXPECT_FALSE(ui.consumeFlightPlanEdit(published));
}

}  // namespace
}  // namespace avionics
