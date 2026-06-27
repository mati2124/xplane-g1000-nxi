#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/AvionicsEngine.h"
#include "avionics/Color.h"
#include "avionics/DataSource.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// No-op renderer: the engine only needs a Renderer reference to construct; these
// tests drive update() (the flight-plan peer reconciliation) and never paint.
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

// Powered, connected source with no flight plan in the map snapshot, mirroring
// the standalone after a Delete Flight Plan pushes an empty route override.
class TestDataSource : public DataSource {
 public:
  void update(double) override {}
  const FlightData& snapshot() const override { return data_; }
  bool requiresPowerUpAcknowledge() const override { return false; }

 private:
  FlightData data_;
};

MapLeg MakeLeg(const std::string& id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

// Drives the MFD's Active Flight Plan Page Menu to "Delete Flight Plan" and
// confirms OK, leaving the page's route cleared (matching DeleteFlightPlanTest).
void DeleteFlightPlanOnMfd(MfdController& ui) {
  ui.pressBezelKey(BezelKey::Fpl);   // Active Flight Plan page
  ui.pressBezelKey(BezelKey::Menu);  // open the Page Menu
  for (int i = 0; i < ui.pageMenuItemCount(); ++i) {
    if (ui.pageMenuItemText(ui.pageMenuSelected()) == "Delete Flight Plan") {
      break;
    }
    ui.pressBezelKey(BezelKey::FmsInnerCw);
  }
  ASSERT_EQ(ui.pageMenuItemText(ui.pageMenuSelected()), "Delete Flight Plan");
  ui.pressBezelKey(BezelKey::Ent);  // open the confirmation (OK highlighted)
  ui.pressBezelKey(BezelKey::Ent);  // confirm OK -> clear the route
}

// Regression: when a route was edited and propagated across both GDUs, both the
// PFD window and the MFD page hold it as a local draft. Deleting it on the MFD
// must not be undone by the peer sync re-copying the PFD's still-loaded route.
TEST(FlightPlanPeerSyncTest, DeleteOnMfdIsNotRevivedByPeerSync) {
  NullRenderer renderer;
  TestDataSource source;

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  const std::vector<MapLeg> plan = {
      MakeLeg("KFMY", 26.586, -81.863),
      MakeLeg("BOSTN", 26.700, -81.500),
      MakeLeg("KCMI", 40.039, -88.278),
  };
  // Both GDUs hold the same route as a local draft (the state left behind once a
  // PFD edit has been mirrored to the MFD via the peer sync).
  pfd.softkeyController().adoptFlightPlanFromPeer(plan, /*destinationFilled=*/true,
                                                  {});
  mfd.mfdController().adoptFlightPlanFromPeer(plan, /*destinationFilled=*/true, {});
  ASSERT_TRUE(pfd.softkeyController().flightPlanLocalDraft());
  ASSERT_TRUE(mfd.mfdController().fplLocalDraft());

  // Settle the reconciliation so both engines record this as the agreed plan.
  pfd.update(0.0);
  mfd.update(0.0);
  ASSERT_EQ(pfd.softkeyController().flightPlanLegs().size(), 3u);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 3u);

  DeleteFlightPlanOnMfd(mfd.mfdController());
  ASSERT_TRUE(mfd.mfdController().fplLegs().empty());

  // Run several frames of both engines: the MFD-side delete must win and clear
  // the PFD too, instead of the PFD's draft re-populating the MFD.
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  EXPECT_TRUE(mfd.mfdController().fplLegs().empty());
  EXPECT_TRUE(pfd.softkeyController().flightPlanLegs().empty());
}

}  // namespace
}  // namespace avionics
