#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg MakeLeg(const std::string& id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

MapData ThreeLegPlan() {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
  return map;
}

bool PlanContains(const std::vector<MapLeg>& legs, const std::string& id) {
  for (const MapLeg& leg : legs) {
    if (leg.id == id) return true;
  }
  return false;
}

// MFD: turning the FMS cursor on, scrolling onto a leg and pressing CLR must arm
// the "Remove <wpt>?" confirmation for the highlighted fix; ENT then deletes it.
TEST(RemoveWaypointTest, MfdClrOnLegArmsRemoveConfirmAndEntDeletes) {
  MfdController ui;
  ui.syncFlightPlan(ThreeLegPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.pageGroup(), MfdPageGroup::FlightPlan);
  ASSERT_EQ(ui.fplLegs().size(), 3u);

  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on

  // Scroll until CLR lands on a real leg row (the Ident column, not the VNAV
  // ALT column) and arms the Remove confirmation.
  std::string removedIdent;
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint) {
      removedIdent = ui.fplRemoveIdent();
      break;
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }

  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);
  ASSERT_FALSE(removedIdent.empty());
  ASSERT_TRUE(PlanContains(ui.fplLegs(), removedIdent));
  EXPECT_TRUE(ui.fplConfirmOk());  // OK highlighted by default

  ui.pressBezelKey(BezelKey::Ent);  // confirm OK -> remove the fix

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_EQ(ui.fplLegs().size(), 2u);
  EXPECT_FALSE(PlanContains(ui.fplLegs(), removedIdent));

  std::vector<MapLeg> published;
  EXPECT_TRUE(ui.consumeFlightPlanEdit(published));
}

// MFD: pressing CLR on the confirmation cancels without changing the route.
TEST(RemoveWaypointTest, MfdRemoveConfirmCancelKeepsRoute) {
  MfdController ui;
  ui.syncFlightPlan(ThreeLegPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint) break;
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);

  ui.pressBezelKey(BezelKey::Clr);  // CLR cancels the modal

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_EQ(ui.fplLegs().size(), 3u);
}

// PFD Active Flight Plan window: same CLR-on-leg behavior as the MFD page.
TEST(RemoveWaypointTest, PfdClrOnLegArmsRemoveConfirmAndEntDeletes) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(ThreeLegPlan(), /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.flightPlanLegs().size(), 3u);

  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on

  std::string removedIdent;
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.flightPlanConfirm() ==
        SoftkeyController::FplConfirm::RemoveWaypoint) {
      removedIdent = ui.flightPlanRemoveIdent();
      break;
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }

  ASSERT_EQ(ui.flightPlanConfirm(),
            SoftkeyController::FplConfirm::RemoveWaypoint);
  ASSERT_FALSE(removedIdent.empty());
  ASSERT_TRUE(PlanContains(ui.flightPlanLegs(), removedIdent));
  EXPECT_TRUE(ui.flightPlanConfirmOk());

  ui.pressBezelKey(BezelKey::Ent);  // confirm OK -> remove the fix

  EXPECT_EQ(ui.flightPlanConfirm(), SoftkeyController::FplConfirm::None);
  EXPECT_EQ(ui.flightPlanLegs().size(), 2u);
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), removedIdent));
}

// MFD: scrolling the list with the cursor OFF still highlights a fix; CLR there
// must remove it (arm the confirm), not fall through to page-step / DFLT MAP.
TEST(RemoveWaypointTest, MfdClrOnScrolledFixWithCursorOffRemoves) {
  MfdController ui;
  ui.syncFlightPlan(ThreeLegPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.fplLegs().size(), 3u);

  // No FmsPush: cursor stays off. Scroll the list to highlight a fix.
  std::string removedIdent;
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint) {
      removedIdent = ui.fplRemoveIdent();
      break;
    }
  }

  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);
  ASSERT_FALSE(removedIdent.empty());

  ui.pressBezelKey(BezelKey::Ent);
  EXPECT_EQ(ui.fplLegs().size(), 2u);
  EXPECT_FALSE(PlanContains(ui.fplLegs(), removedIdent));
}

// PFD: same as above, and CLR must NOT close the Active Flight Plan window when
// a scrolled fix is highlighted (regression for "CLR just closes the page").
TEST(RemoveWaypointTest, PfdClrOnScrolledFixWithCursorOffRemovesAndKeepsWindow) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(ThreeLegPlan(), /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.activeWindow(), PfdWindow::FlightPlan);

  std::string removedIdent;
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.flightPlanConfirm() ==
        SoftkeyController::FplConfirm::RemoveWaypoint) {
      removedIdent = ui.flightPlanRemoveIdent();
      break;
    }
    // The window must stay open while we hunt for a removable fix.
    ASSERT_EQ(ui.activeWindow(), PfdWindow::FlightPlan);
  }

  ASSERT_EQ(ui.flightPlanConfirm(),
            SoftkeyController::FplConfirm::RemoveWaypoint);
  EXPECT_EQ(ui.activeWindow(), PfdWindow::FlightPlan);

  ui.pressBezelKey(BezelKey::Ent);
  EXPECT_EQ(ui.flightPlanLegs().size(), 2u);
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), removedIdent));
}

// MFD: KFMY->KJAX route with cursor on destination must confirm KJAX, not an
// enroute fix one slot earlier (regression when duplicate idents are hidden).
TEST(RemoveWaypointTest, MfdClrOnKjaxConfirmsKjax) {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("CSHEL", 26.650, -81.700),
      MakeLeg("LAL", 27.900, -82.000),
      MakeLeg("JINOS", 28.500, -81.800),
      MakeLeg("TEBOW", 29.000, -81.600),
      MakeLeg("KJAX", 30.494, -81.688),
  };
  MfdController ui;
  ui.syncFlightPlan(map, /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  const auto rows = pfd::fplFilteredSectionRows(
      static_cast<int>(map.flightPlan.size()), true, false, map.flightPlan);
  const int kjaxRow = pfd::fplSectionSelectableRowForLegIndex(
      5, rows, static_cast<int>(map.flightPlan.size()), true);
  ASSERT_GE(kjaxRow, 0);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);
  // Walk the list until CLR arms Remove KJAX (cursor may start on the active leg).
  std::string removedIdent;
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint) {
      removedIdent = ui.fplRemoveIdent();
      if (removedIdent == "KJAX") break;
      ui.pressBezelKey(BezelKey::Clr);  // cancel — wrong row
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }

  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);
  EXPECT_EQ(removedIdent, "KJAX");
}

// MFD: loaded SID (procedure display rows) — cursor on destination KJAX must
// confirm KJAX, not the enroute fix TEBOW one slot earlier.
TEST(RemoveWaypointTest, MfdClrOnKjaxWithDepartureProcedureConfirmsKjax) {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("CSHEL", 26.650, -81.700),
      MakeLeg("LAL", 27.900, -82.000),
      MakeLeg("JINOS", 28.500, -81.800),
      MakeLeg("TEBOW", 29.000, -81.600),
      MakeLeg("KJAX", 30.494, -81.688),
  };
  MfdController ui;
  ui.syncFlightPlan(map, /*activeWaypoint=*/"JINOS", /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;
  dep.legCount = 1;
  dep.headerLabel = "KFMY-RW05.CSHEL8";
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  const int kjaxRow = pfd::fplProcedureSelectableRowForLegIndex(
      5, map.flightPlan, dep.legStart, dep.legCount, dep.headerLabel, 0, 0,
      std::string(), 0, 0, true, true);
  ASSERT_GE(kjaxRow, 0);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.adoptFlightPlanCursorFromPeer(kjaxRow, /*followsActive=*/false);
  ui.pressBezelKey(BezelKey::Clr);

  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);
  EXPECT_EQ(ui.fplRemoveIdent(), "KJAX");
}

// While the "Remove <wpt>?" confirmation is open it owns CLR, so the
// press-and-hold CLR (DFLT MAP) the shell would otherwise fire after ~1s must
// be suppressed (regression for "popup shows for a second, then the page
// closes"). clrDefaultMap() is what the hold would call; it tears the popup
// down, which the engine's holdBezelKey now refuses while suppressed.
TEST(RemoveWaypointTest, MfdRemoveConfirmSuppressesClrDefaultMapHold) {
  MfdController ui;
  ui.syncFlightPlan(ThreeLegPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);
  EXPECT_FALSE(ui.clrDefaultMapHoldSuppressed());  // nothing modal yet

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);
  for (int i = 0; i < 12; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint) break;
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveWaypoint);

  EXPECT_TRUE(ui.clrDefaultMapHoldSuppressed());

  // If the hold were allowed to fire it would reset the page and drop the popup.
  ui.clrDefaultMap();
  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_EQ(ui.pageGroup(), MfdPageGroup::Map);
}

// PFD: CLR right after opening the window (cursor following the active leg, no
// explicit selection) still closes the window, as before.
TEST(RemoveWaypointTest, PfdClrOnFreshWindowStillCloses) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(ThreeLegPlan(), /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.activeWindow(), PfdWindow::FlightPlan);

  ui.pressBezelKey(BezelKey::Clr);

  EXPECT_EQ(ui.activeWindow(), PfdWindow::None);
  EXPECT_EQ(ui.flightPlanConfirm(), SoftkeyController::FplConfirm::None);
  EXPECT_EQ(ui.flightPlanLegs().size(), 3u);
}

}  // namespace
}  // namespace avionics
