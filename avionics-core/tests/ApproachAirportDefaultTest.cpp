#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// Reproduction harness: opening PROC -> "Select Approach" must default the
// approach airport to the flight plan destination airport.
namespace avionics {
namespace {

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

MapLeg MakeLeg(const std::string& id, double lat = 0.0, double lon = 0.0) {  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

MapData TwoAirportPlan(const std::string& dest) {
  MapData map;
  map.flightPlan = {MakeLeg("KFMY", 26.586, -81.863),
                    MakeLeg("LAL", 27.900, -82.000),
                    MakeLeg(dest, 30.494, -81.688)};
  map.positionValid = true;
  map.ownshipLat = 26.586;
  map.ownshipLon = -81.863;
  return map;
}

void OpenProcMenu(MfdController& ui) {
  // Close any open PROC sub-window, then open it fresh so we land on the
  // top-level Procedures menu (as PROC does on the unit).
  if (ui.procMenuOpen()) ui.pressBezelKey(BezelKey::Proc);
  ui.pressBezelKey(BezelKey::Proc);
}

void OpenProcMenu(SoftkeyController& ui) {
  if (ui.activeWindow() == PfdWindow::Procedures) {
    ui.pressBezelKey(BezelKey::Proc);
  }
  ui.pressBezelKey(BezelKey::Proc);
}

template <typename Ui>
void SelectMenuItem(Ui& ui, const char* label) {
  OpenProcMenu(ui);
  for (int i = 0;
       i < 12 && ui.procMenuItemText(ui.procMenuSelected()) != label; ++i) {
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ui.pressBezelKey(BezelKey::Ent);
}

void MfdSelectApproach(MfdController& ui) { SelectMenuItem(ui, "Select Approach"); }
void MfdSelectDeparture(MfdController& ui) {
  SelectMenuItem(ui, "Select Departure");
}
void PfdSelectApproach(SoftkeyController& ui) {
  SelectMenuItem(ui, "Select Approach");
}
void PfdSelectDeparture(SoftkeyController& ui) {
  SelectMenuItem(ui, "Select Departure");
}

TEST(ApproachAirportDefaultTest, MfdDefaultsToDestination) {
  MfdController ui;
  ui.syncFlightPlan(TwoAirportPlan("KJAX"), {}, false);
  MfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KJAX");
}

TEST(ApproachAirportDefaultTest, MfdDefaultsToDestinationWithDigit) {
  MfdController ui;
  ui.syncFlightPlan(TwoAirportPlan("KX01"), {}, false);
  MfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KX01");
}

TEST(ApproachAirportDefaultTest, MfdApproachAfterDepartureUsesDestination) {
  MfdController ui;
  ui.syncFlightPlan(TwoAirportPlan("KJAX"), {}, false);
  MfdSelectDeparture(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KFMY");  // departure uses origin
  MfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KJAX");
}

TEST(ApproachAirportDefaultTest, PfdDefaultsToDestination) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(TwoAirportPlan("KJAX"), false);
  PfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KJAX");
}

TEST(ApproachAirportDefaultTest, PfdDefaultsToDestinationWithDigit) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(TwoAirportPlan("KX01"), false);
  PfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KX01");
}

TEST(ApproachAirportDefaultTest, PfdApproachAfterDepartureUsesDestination) {
  SoftkeyController ui;
  ui.syncFlightPlanFromMap(TwoAirportPlan("KJAX"), false);
  PfdSelectDeparture(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KFMY");  // departure uses origin
  PfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KJAX");
}

// Nav data is loaded but does not contain the small destination field (K1H2).
// PROC must still default to that destination, not the origin.
TEST(ApproachAirportDefaultTest, MfdDigitDestinationWithReadyNavDb) {
  EmptyReadyNavSource nav;
  MfdController ui;
  ui.setNavFeatureSource(&nav);
  MapData map;
  map.flightPlan = {MakeLeg("KBNA"), MakeLeg("GHM"), MakeLeg("K1H2")};
  ui.syncFlightPlan(map, {}, false);
  MfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "K1H2");
}

TEST(ApproachAirportDefaultTest, PfdDigitDestinationWithReadyNavDb) {
  EmptyReadyNavSource nav;
  SoftkeyController ui;
  ui.setNavFeatureSource(&nav);
  ui.replaceFlightPlanFromExternal(
      {MakeLeg("KBNA"), MakeLeg("GHM"), MakeLeg("K1H2")});
  PfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "K1H2");
}

TEST(ApproachAirportDefaultTest, MfdTwoLegDestinationWithReadyNavDb) {
  EmptyReadyNavSource nav;
  MfdController ui;
  ui.setNavFeatureSource(&nav);
  ui.syncFlightPlan(TwoAirportPlan("KJAX"), {}, false);
  MfdSelectApproach(ui);
  EXPECT_EQ(ui.procAirportIcao(), "KJAX");
}

}  // namespace
}  // namespace avionics