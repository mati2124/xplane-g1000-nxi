#include <gtest/gtest.h>

#include "avionics/MfdController.h"
#include "avionics/render/BezelKeys.h"

// WPT - Airport Information page softkey bar + sub-view selection (NXi trainer
// apt_054..058): Engine, Map Opt, Chart, Info, DP, STAR, APR, WX, Checklist,
// with Map Opt hidden on the procedure sub-views and the active sub-view's
// softkey lit.
namespace avionics {
namespace {

// Trainer softkey cell positions for the WPT - Airport Information bar.
constexpr int kEngine = 0;
constexpr int kMapOpt = 2;
constexpr int kChart = 3;
constexpr int kInfo = 4;
constexpr int kDp = 5;
constexpr int kStar = 6;
constexpr int kApr = 7;
constexpr int kWx = 8;
constexpr int kChecklist = 11;

// Step the page group from MAP to WPT (large FMS knob), landing on the first
// WPT page (Airport Information).
MfdController onAirportInfoPage() {
  MfdController ui;
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // MAP -> WPT
  EXPECT_EQ(ui.page(), MfdPage::AirportInformation);
  return ui;
}

TEST(WptAirportInfoPageTest, RootBarMatchesTrainer) {
  MfdController ui = onAirportInfoPage();
  EXPECT_EQ(ui.label(kEngine), "Engine");
  EXPECT_EQ(ui.label(kMapOpt), "Map Opt");
  EXPECT_EQ(ui.label(kChart), "Chart");
  EXPECT_EQ(ui.label(kInfo), "Info");
  EXPECT_EQ(ui.label(kDp), "DP");
  EXPECT_EQ(ui.label(kStar), "STAR");
  EXPECT_EQ(ui.label(kApr), "APR");
  EXPECT_EQ(ui.label(kWx), "WX");
  EXPECT_EQ(ui.label(kChecklist), "Checklist");
  // The default sub-view is the airport information panel (Info lit).
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Airport);
  EXPECT_TRUE(ui.keyActive(kInfo));
}

TEST(WptAirportInfoPageTest, ProcedureSoftkeysSelectSubViewAndHideMapOpt) {
  MfdController ui = onAirportInfoPage();

  ASSERT_TRUE(ui.pressKey(kDp));
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Departure);
  EXPECT_TRUE(ui.keyActive(kDp));
  EXPECT_TRUE(ui.label(kMapOpt).empty());  // blank on the procedure sub-views

  ASSERT_TRUE(ui.pressKey(kStar));
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Arrival);
  EXPECT_TRUE(ui.keyActive(kStar));

  ASSERT_TRUE(ui.pressKey(kApr));
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Approach);
  EXPECT_TRUE(ui.keyActive(kApr));
}

TEST(WptAirportInfoPageTest, WeatherKeepsMapOptAndInfoReturnsToAirport) {
  MfdController ui = onAirportInfoPage();

  ASSERT_TRUE(ui.pressKey(kWx));
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Weather);
  EXPECT_TRUE(ui.keyActive(kWx));
  EXPECT_EQ(ui.label(kMapOpt), "Map Opt");  // present on the Weather view

  ASSERT_TRUE(ui.pressKey(kInfo));
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Airport);
  EXPECT_TRUE(ui.keyActive(kInfo));
  EXPECT_EQ(ui.label(kMapOpt), "Map Opt");
}

TEST(WptAirportInfoPageTest, ChartSoftkeyEntersChartView) {
  MfdController ui = onAirportInfoPage();
  ASSERT_TRUE(ui.pressKey(kChart));
  EXPECT_TRUE(ui.chartViewActive());
  EXPECT_EQ(ui.page(), MfdPage::AirportInformation);
}

TEST(WptAirportInfoPageTest, LeavingGroupResetsSubView) {
  MfdController ui = onAirportInfoPage();
  ASSERT_TRUE(ui.pressKey(kApr));
  ASSERT_EQ(ui.wptInfoView(), WptInfoView::Approach);

  ui.pressBezelKey(BezelKey::FmsOuterCw);   // WPT -> AUX
  ui.pressBezelKey(BezelKey::FmsOuterCcw);  // AUX -> WPT
  EXPECT_EQ(ui.page(), MfdPage::AirportInformation);
  EXPECT_EQ(ui.wptInfoView(), WptInfoView::Airport);
}

}  // namespace
}  // namespace avionics
