#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// CLR on a SID/STAR/approach *header* row, and the page-menu "Remove Departure
// / Arrival / Approach" options, open the procedure removal confirmation and
// (on OK) erase the whole procedure. CLR on an individual procedure *leg*
// removes just that fix, leaving the rest of the procedure in place (same as a
// plain enroute fix).
namespace avionics {
namespace {

MapLeg MakeLeg(const std::string& id, double lat = 0.0, double lon = 0.0) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

bool PlanContains(const std::vector<MapLeg>& legs, const std::string& id) {
  for (const MapLeg& leg : legs) {
    if (leg.id == id) return true;
  }
  return false;
}

MapData KfmyKjaxPlan() {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863), MakeLeg("CSHEL", 26.650, -81.700),
      MakeLeg("LAL", 27.900, -82.000),  MakeLeg("JINOS", 28.500, -81.800),
      MakeLeg("TEBOW", 29.000, -81.600), MakeLeg("KJAX", 30.494, -81.688),
  };
  return map;
}

// CLR with the cursor on a single SID leg arms Remove Waypoint (not Remove
// Departure); ENT then erases just that fix and leaves the rest of the SID — and
// the loaded departure — in place.
TEST(ProcedureRemoveTest, MfdClrOnDepartureLegRemovesSingleFix) {
  MfdController ui;
  ui.syncFlightPlan(KfmyKjaxPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;  // CSHEL + LAL are the loaded SID legs
  dep.legCount = 2;
  dep.headerLabel = "KFMY-RW05.CSHEL8";
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.pageGroup(), MfdPageGroup::FlightPlan);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on

  bool armed = false;
  for (int i = 0; i < 16; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint &&
        ui.fplRemoveIdent() == "CSHEL") {
      armed = true;
      break;
    }
    if (ui.fplConfirm() != MfdController::FplConfirm::None) {
      ui.pressBezelKey(BezelKey::Clr);  // cancel — wrong row, keep hunting
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }

  ASSERT_TRUE(armed);
  EXPECT_TRUE(ui.fplConfirmOk());

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove just CSHEL

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_FALSE(PlanContains(ui.fplLegs(), "CSHEL"));
  EXPECT_TRUE(PlanContains(ui.fplLegs(), "LAL"));
  EXPECT_TRUE(ui.fplHasLoadedDeparture());  // SID survives with its other leg
  EXPECT_EQ(ui.fplLegs().size(), 5u);

  std::vector<MapLeg> published;
  EXPECT_TRUE(ui.consumeFlightPlanEdit(published));
}

// The SID/STAR/approach header row is itself a selectable cursor stop: scrolling
// the cursor to the top lands on the departure header, and CLR there arms Remove
// Departure (the trainer lets you select the procedure header directly).
TEST(ProcedureRemoveTest, MfdClrOnDepartureHeaderRemovesWholeProcedure) {
  MfdController ui;
  ui.syncFlightPlan(KfmyKjaxPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;
  dep.legCount = 1;
  dep.headerLabel = "KFMY-RW05.CSHEL8";
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on
  // Scroll to the very top of the list, which is the Departure header row.
  for (int i = 0; i < 20; ++i) ui.pressBezelKey(BezelKey::FmsOuterCcw);

  ui.pressBezelKey(BezelKey::Clr);
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveDeparture);

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove the departure
  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_FALSE(ui.fplHasLoadedDeparture());
  EXPECT_FALSE(PlanContains(ui.fplLegs(), "CSHEL"));
}

// CLR on the whole-procedure confirmation (armed from the departure header)
// cancels and leaves the SID in place.
TEST(ProcedureRemoveTest, MfdRemoveDepartureCancelKeepsProcedure) {
  MfdController ui;
  ui.syncFlightPlan(KfmyKjaxPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;
  dep.legCount = 1;
  dep.headerLabel = "KFMY-RW05.CSHEL8";
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);
  // Scroll to the top of the list: the Departure header row arms Remove
  // Departure (a single leg only arms Remove Waypoint now).
  for (int i = 0; i < 20; ++i) ui.pressBezelKey(BezelKey::FmsOuterCcw);
  ui.pressBezelKey(BezelKey::Clr);
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveDeparture);

  ui.pressBezelKey(BezelKey::Clr);  // cancel the modal

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_TRUE(ui.fplHasLoadedDeparture());
  EXPECT_TRUE(PlanContains(ui.fplLegs(), "CSHEL"));
  EXPECT_EQ(ui.fplLegs().size(), 6u);
}

// The page-menu "Remove Approach" option opens the confirmation; ENT erases the
// loaded approach block.
TEST(ProcedureRemoveTest, MfdPageMenuRemoveApproach) {
  MapData map;
  map.flightPlan = {
      MakeLeg("KFMY", 26.586, -81.863), MakeLeg("CSHEL", 26.650, -81.700),
      MakeLeg("KJAX", 30.494, -81.688), MakeLeg("FAFIX", 30.300, -81.700),
      MakeLeg("RW07", 30.480, -81.690),
  };
  MfdController ui;
  ui.syncFlightPlan(map, /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  FlightPlanApproachState appr;
  appr.legStart = 3;  // FAFIX + RW07 are the approach legs
  appr.legCount = 2;
  appr.headerLabel = "RNAV 07";
  appr.loaded.name = "RNAV 07";
  ui.applyFlightPlanApproachState(appr);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_TRUE(ui.fplHasLoadedApproach());

  ui.pressBezelKey(BezelKey::Menu);
  ASSERT_TRUE(ui.pageMenuOpen());

  bool onRemoveApproach = false;
  for (int i = 0; i < 40; ++i) {
    if (ui.pageMenuItemText(ui.pageMenuSelected()) == "Remove Approach") {
      onRemoveApproach = true;
      break;
    }
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ASSERT_TRUE(onRemoveApproach);

  ui.pressBezelKey(BezelKey::Ent);  // run the option -> open the confirmation
  ASSERT_EQ(ui.fplConfirm(), MfdController::FplConfirm::RemoveApproach);

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove the approach

  EXPECT_EQ(ui.fplConfirm(), MfdController::FplConfirm::None);
  EXPECT_FALSE(ui.fplHasLoadedApproach());
  EXPECT_FALSE(PlanContains(ui.fplLegs(), "FAFIX"));
  EXPECT_FALSE(PlanContains(ui.fplLegs(), "RW07"));
  EXPECT_EQ(ui.fplLegs().size(), 3u);
}

// Remove Departure / Arrival / Approach grey out when no such procedure is
// loaded, so the page menu cannot act on a procedure that is not there.
TEST(ProcedureRemoveTest, MfdPageMenuRemoveItemsDisabledWithoutProcedures) {
  MfdController ui;
  ui.syncFlightPlan(KfmyKjaxPlan(), /*activeWaypoint=*/{}, /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::Menu);
  ASSERT_TRUE(ui.pageMenuOpen());

  for (int i = 0; i < ui.pageMenuItemCount(); ++i) {
    const std::string& text = ui.pageMenuItemText(i);
    if (text == "Remove Departure" || text == "Remove Arrival" ||
        text == "Remove Approach") {
      EXPECT_FALSE(ui.pageMenuItemEnabled(i)) << text << " must be disabled";
    }
  }
}

// ---- PFD Active Flight Plan window mirrors the MFD page behavior ----

// PFD: CLR with the cursor on a single arrival (STAR) leg arms Remove Waypoint
// (not Remove Arrival); ENT then erases just that fix and leaves the rest of the
// STAR — and the loaded arrival — in place.
TEST(ProcedureRemoveTest, PfdClrOnArrivalLegRemovesSingleFix) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(KfmyKjaxPlan(), /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState arr;
  arr.legStart = 3;  // JINOS + TEBOW are the loaded STAR legs
  arr.legCount = 2;
  arr.headerLabel = "LAL.FERN1";  // header label carries no airport prefix
  arr.loaded.name = "FERN1";
  ui.applyFlightPlanArrivalState(arr);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on

  bool armed = false;
  for (int i = 0; i < 16; ++i) {
    ui.pressBezelKey(BezelKey::Clr);
    if (ui.flightPlanConfirm() == SoftkeyController::FplConfirm::RemoveWaypoint &&
        ui.flightPlanRemoveIdent() == "JINOS") {
      armed = true;
      break;
    }
    if (ui.flightPlanConfirm() != SoftkeyController::FplConfirm::None) {
      ui.pressBezelKey(BezelKey::Clr);  // cancel — wrong row, keep hunting
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }

  ASSERT_TRUE(armed);
  EXPECT_TRUE(ui.flightPlanConfirmOk());

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove just JINOS

  EXPECT_EQ(ui.flightPlanConfirm(), SoftkeyController::FplConfirm::None);
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), "JINOS"));
  EXPECT_TRUE(PlanContains(ui.flightPlanLegs(), "TEBOW"));
  EXPECT_TRUE(ui.flightPlanHasLoadedArrival());  // STAR survives with its other leg
  EXPECT_EQ(ui.flightPlanLegs().size(), 5u);

  std::vector<MapLeg> published;
  EXPECT_TRUE(ui.consumeFlightPlanEdit(published));
}

// PFD: the page-menu "Remove Departure" option opens the confirmation; ENT
// erases the loaded SID block.
TEST(ProcedureRemoveTest, PfdPageMenuRemoveDeparture) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(KfmyKjaxPlan(), /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;  // CSHEL is the loaded SID's single tracked leg
  dep.legCount = 1;
  dep.headerLabel = "RW05.CSHEL8";  // header label carries no airport prefix
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_TRUE(ui.flightPlanHasLoadedDeparture());

  ui.pressBezelKey(BezelKey::Menu);
  ASSERT_TRUE(ui.pageMenuOpen());

  bool onRemoveDeparture = false;
  for (int i = 0; i < 40; ++i) {
    if (ui.pageMenuItemText(ui.pageMenuSelected()) == "Remove Departure") {
      onRemoveDeparture = true;
      break;
    }
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ASSERT_TRUE(onRemoveDeparture);

  ui.pressBezelKey(BezelKey::Ent);  // run the option -> open the confirmation
  ASSERT_EQ(ui.flightPlanConfirm(),
            SoftkeyController::FplConfirm::RemoveDeparture);
  EXPECT_EQ(ui.flightPlanRemoveIdent(), "KFMY-RW05.CSHEL8");

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove the departure

  EXPECT_EQ(ui.flightPlanConfirm(), SoftkeyController::FplConfirm::None);
  EXPECT_FALSE(ui.flightPlanHasLoadedDeparture());
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), "CSHEL"));
  EXPECT_EQ(ui.flightPlanLegs().size(), 5u);
}

// PFD: the departure header row is a selectable cursor stop. Scrolling to the
// top of the list lands on it and CLR arms Remove Departure with the trainer's
// "KFMY-RW05.CSHEL8" prompt subject.
TEST(ProcedureRemoveTest, PfdClrOnDepartureHeaderRemovesWholeProcedure) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(KfmyKjaxPlan(), /*navDirectTo=*/false);

  FlightPlanTerminalProcedureState dep;
  dep.legStart = 1;
  dep.legCount = 1;
  dep.headerLabel = "RW05.CSHEL8";  // header label carries no airport prefix
  dep.loaded.name = "CSHEL8";
  ui.applyFlightPlanDepartureState(dep);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on
  // Scroll to the very top of the list: the Departure header row.
  for (int i = 0; i < 20; ++i) ui.pressBezelKey(BezelKey::FmsOuterCcw);

  ui.pressBezelKey(BezelKey::Clr);
  ASSERT_EQ(ui.flightPlanConfirm(),
            SoftkeyController::FplConfirm::RemoveDeparture);
  EXPECT_EQ(ui.flightPlanRemoveIdent(), "KFMY-RW05.CSHEL8");

  ui.pressBezelKey(BezelKey::Ent);  // OK -> remove the departure
  EXPECT_EQ(ui.flightPlanConfirm(), SoftkeyController::FplConfirm::None);
  EXPECT_FALSE(ui.flightPlanHasLoadedDeparture());
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), "CSHEL"));
}

// PFD: Remove Departure / Arrival / Approach grey out when no procedure loaded.
TEST(ProcedureRemoveTest, PfdPageMenuRemoveItemsDisabledWithoutProcedures) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(KfmyKjaxPlan(), /*navDirectTo=*/false);

  ui.pressBezelKey(BezelKey::Fpl);
  ui.pressBezelKey(BezelKey::Menu);
  ASSERT_TRUE(ui.pageMenuOpen());

  for (int i = 0; i < ui.pageMenuItemCount(); ++i) {
    const std::string& text = ui.pageMenuItemText(i);
    if (text == "Remove Departure" || text == "Remove Arrival" ||
        text == "Remove Approach") {
      EXPECT_FALSE(ui.pageMenuItemEnabled(i)) << text << " must be disabled";
    }
  }
}

}  // namespace
}  // namespace avionics
