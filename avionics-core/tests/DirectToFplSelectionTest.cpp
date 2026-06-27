#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/AvionicsEngine.h"
#include "avionics/Color.h"
#include "avionics/ConnectionState.h"
#include "avionics/DataSource.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

class NullRenderer : public Renderer {
 public:
  void beginFrame(int, int, float) override {}
  void endFrame() override {}
  void save() override {}
  void restore() override {}
  void translate(float, float) override {}
  void rotateDegrees(float) override {}
  void clip(float, float, float, float) override {}
  void fillRect(float, float, float, float, const Color&) override {}
  void fillRectVerticalGradient(float, float, float, float, float, float,
                                const Color&, const Color&) override {}
  void strokeLine(float, float, float, float, float, const Color&) override {}
  void fillCircle(float, float, float, const Color&) override {}
  void fillPolygon(const Point*, int, const Color&) override {}
  void strokePolyline(const Point*, int, float, const Color&) override {}
  int createImageRGBA(int, int, const unsigned char*) override { return -1; }
  void updateImageRGBA(int, const unsigned char*) override {}
  void deleteImage(int) override {}
  void drawImage(int, float, float, float, float, float) override {}
  void fillText(float, float, const std::string&, float, TextAlign, const Color&,
                FontFace) override {}
  float measureTextWidth(const std::string& text, float sizePx,
                         FontFace) override {
    return static_cast<float>(text.size()) * sizePx * 0.5f;
  }
};

class RouteDataSource : public DataSource {
 public:
  void setRoute(const std::vector<MapLeg>& plan,
                const std::string& activeWaypoint) {
    map_.flightPlan = plan;
    data_.fmaToWpt = activeWaypoint;
  }
  void update(double) override {}
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }
  ConnectionState connectionState() const override {
    return ConnectionState::Connected;
  }
  bool requiresPowerUpAcknowledge() const override { return false; }

 private:
  FlightData data_;
  MapData map_;
};

MapLeg MakeLeg(const std::string& id) {
  MapLeg leg;
  leg.id = id;
  return leg;
}

std::vector<MapLeg> KfmyKjaxRoute() {
  return {MakeLeg("KFMY"), MakeLeg("CSHEL"), MakeLeg("LAL"), MakeLeg("JINOS"),
          MakeLeg("TEBOW"), MakeLeg("KJAX")};
}

int TebowSelectableRow(const std::vector<MapLeg>& plan) {
  const auto rows = pfd::fplFilteredSectionRows(static_cast<int>(plan.size()),
                                                true, false, plan);
  return pfd::fplSectionSelectableRowForLegIndex(
      4, rows, static_cast<int>(plan.size()), true);
}

void ScrollPfdFplToTebow(AvionicsEngine& pfd, const std::vector<MapLeg>& plan) {
  const int tebowRow = TebowSelectableRow(plan);
  ASSERT_GE(tebowRow, 0);
  pfd.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(pfd.softkeyController().activeWindow(), PfdWindow::FlightPlan);
  for (int i = 0; i < 20; ++i) {
    pfd.pressBezelKey(BezelKey::FmsOuterCw);
    if (pfd.softkeyController().flightPlanCursor() == tebowRow) break;
  }
  ASSERT_EQ(pfd.softkeyController().flightPlanCursor(), tebowRow);
  ASSERT_FALSE(pfd.softkeyController().flightPlanListCursorFollowsActive());
}

// PFD FPL: scrolled selection must win over the active nav leg (JINOS).
TEST(DirectToFplSelectionTest, PfdDtoUsesScrolledSelectionNotActiveLeg) {
  const std::vector<MapLeg> plan = KfmyKjaxRoute();
  MapData map;
  map.flightPlan = plan;

  SoftkeyController ui;
  ui.syncFlightPlanFromMap(map, /*navDirectTo=*/false);
  FlightData data;
  data.fmaToWpt = "JINOS";
  ui.update(0.0, data, map);

  const int tebowRow = TebowSelectableRow(plan);
  ASSERT_GE(tebowRow, 0);

  ui.pressBezelKey(BezelKey::Fpl);
  for (int i = 0; i < 20; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    if (ui.flightPlanCursor() == tebowRow) break;
  }
  ASSERT_EQ(ui.flightPlanCursor(), tebowRow);
  ASSERT_FALSE(ui.flightPlanListCursorFollowsActive());

  ui.pressBezelKey(BezelKey::DirectTo);
  ASSERT_TRUE(ui.directToWindowOpen());
  EXPECT_EQ(ui.directToIdent(), "TEBOW");
}

// Dual GDU: scrolling TEBOW on the PFD FPL window must seed MFD Direct-To.
TEST(DirectToFplSelectionTest, MfdDtoUsesPfdFplScrollSelection) {
  NullRenderer renderer;
  RouteDataSource source;
  const std::vector<MapLeg> plan = KfmyKjaxRoute();
  source.setRoute(plan, "JINOS");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  pfd.softkeyController().adoptFlightPlanFromPeer(plan, /*destinationFilled=*/true,
                                                  {});
  mfd.mfdController().adoptFlightPlanFromPeer(plan, /*destinationFilled=*/true,
                                              {});
  pfd.update(0.0);
  mfd.update(0.0);

  ScrollPfdFplToTebow(pfd, plan);
  pfd.update(0.0);
  mfd.update(0.0);

  mfd.pressBezelKey(BezelKey::DirectTo);
  ASSERT_TRUE(mfd.mfdController().directToWindowOpen());
  EXPECT_EQ(mfd.mfdController().directToIdent(), "TEBOW");
}

}  // namespace
}  // namespace avionics
