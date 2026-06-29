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

// Powered source that publishes a flight plan in its map snapshot plus a live
// active leg (FMA TO waypoint), mirroring the real standalone where the engine
// adopts the route from the map and the list cursor follows the active leg.
class PlanDataSource : public DataSource {
 public:
  PlanDataSource(std::vector<MapLeg> plan, std::string activeFromIdent,
                 std::string activeToIdent) {
    map_.flightPlan = std::move(plan);
    // A real active leg has both a FROM and a TO waypoint; leaving FROM empty
    // would put the engine into GPS Direct-To mode (no enroute leg list).
    data_.fmaFromWpt = std::move(activeFromIdent);
    data_.fmaToWpt = std::move(activeToIdent);
  }
  void update(double) override {}
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }
  bool requiresPowerUpAcknowledge() const override { return false; }

 private:
  FlightData data_;
  MapData map_;
};

// Powered source whose map plan + nearby features can be mutated, mirroring the
// standalone where a published flight-plan edit is pushed back into the map
// snapshot both GDUs read (and where the FMS waypoint-entry window resolves a
// typed ident against the nearby map features when no nav database is loaded).
class MutablePlanSource : public DataSource {
 public:
  MutablePlanSource(std::vector<MapLeg> plan, std::vector<MapFeature> features,
                    std::string activeFromIdent, std::string activeToIdent) {
    map_.flightPlan = std::move(plan);
    map_.features = std::move(features);
    map_.positionValid = true;
    data_.fmaFromWpt = std::move(activeFromIdent);
    data_.fmaToWpt = std::move(activeToIdent);
  }
  void update(double) override {}
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }
  bool requiresPowerUpAcknowledge() const override { return false; }
  void setPlan(std::vector<MapLeg> plan) { map_.flightPlan = std::move(plan); }

 private:
  FlightData data_;
  MapData map_;
};

MapFeature MakeFeature(const std::string& id, double lat, double lon) {
  MapFeature f;
  f.id = id;
  f.lat = lat;
  f.lon = lon;
  return f;
}

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

// Off-plan GPS Direct-To blanks both GDUs' leg lists (localDraft=false). Delete
// Flight Plan on the MFD must stick even if the PFD re-adopts a stale map route.
TEST(FlightPlanPeerSyncTest, DeleteAfterOffPlanDirectToIsNotRevivedByPeerSync) {
  NullRenderer renderer;

  class DirectToThenStaleMapSource : public DataSource {
   public:
    DirectToThenStaleMapSource() {
      map_.flightPlan = {
          MakeLeg("KFMY", 26.586, -81.863),
          MakeLeg("BOSTN", 26.700, -81.500),
          MakeLeg("KCMI", 40.039, -88.278),
      };
      data_.fmaToWpt = "RSW";
      map_.directToActive = true;
      map_.directTo = MakeLeg("RSW", 26.536, -81.755);
    }
    void update(double) override {}
    const FlightData& snapshot() const override { return data_; }
    const MapData& mapSnapshot() const override { return map_; }
    bool requiresPowerUpAcknowledge() const override { return false; }
    void endDirectToKeepStaleMapPlan() {
      data_.fmaToWpt.clear();
      map_.directToActive = false;
      map_.directTo = {};
    }

   private:
    FlightData data_;
    MapData map_;
  };

  DirectToThenStaleMapSource source;

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_TRUE(pfd.softkeyController().flightPlanLegs().empty());
  ASSERT_TRUE(mfd.mfdController().fplLegs().empty());

  DeleteFlightPlanOnMfd(mfd.mfdController());
  ASSERT_TRUE(mfd.mfdController().fplLegs().empty());
  ASSERT_TRUE(mfd.mfdController().fplLocalDraft());

  std::vector<MapLeg> published;
  ASSERT_TRUE(mfd.mfdController().consumeFlightPlanEdit(published));
  EXPECT_TRUE(published.empty());

  source.endDirectToKeepStaleMapPlan();

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  EXPECT_TRUE(mfd.mfdController().fplLegs().empty());
  EXPECT_TRUE(pfd.softkeyController().flightPlanLegs().empty());
  EXPECT_TRUE(pfd.softkeyController().flightPlanLocalDraft());
}

const std::vector<MapLeg> kReproPlan = {
    MakeLeg("KFMY", 26.586, -81.863),
    MakeLeg("BOSTN", 26.700, -81.500),
    MakeLeg("WINCO", 27.100, -81.200),
    MakeLeg("KCMI", 40.039, -88.278),
};

// Steps the MFD large knob clockwise, settling a frame of engine reconciliation
// between detents like the real per-frame update loop.
int DriveMfdCursorDownTwoSteps(AvionicsEngine& mfd, AvionicsEngine* pfd) {
  MfdController& ui = mfd.mfdController();
  const int startLeg = ui.fplCursorLegIndexPublic();
  for (int i = 0; i < 8 && ui.fplCursorLegIndexPublic() <= startLeg; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    if (pfd != nullptr) pfd->update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  return ui.fplCursorLegIndexPublic();
}

// The MFD FMS selection cursor must advance through the leg list as the large
// knob is turned, even though the engine reconciles the flight-plan cursor every
// frame. (Regression: the per-frame clamp wiped the VNAV ALT column back to the
// Ident column, so the large knob oscillated Ident<->ALT and never stepped row.)
TEST(FlightPlanPeerSyncTest, MfdFmsCursorAdvancesWithEnginePerFrameSync) {
  NullRenderer renderer;
  TestDataSource source;

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();

  mfd.mfdController().adoptFlightPlanFromPeer(kReproPlan, /*destinationFilled=*/true,
                                              {});
  mfd.update(0.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  mfd.mfdController().pressBezelKey(BezelKey::FmsPush);  // cursor on
  ASSERT_TRUE(mfd.mfdController().fplCursorOn());

  const int startLeg = mfd.mfdController().fplCursorLegIndexPublic();
  const int afterLeg = DriveMfdCursorDownTwoSteps(mfd, /*pfd=*/nullptr);
  EXPECT_GT(afterLeg, startLeg);
}

// Reported bug: the MFD FMS cursor misbehaves on the Active Flight Plan page when
// the PFD Active Flight Plan window is also open. With both displays running, the
// peer cursor sync (PFD passively following the active leg) must not clobber the
// row/column the pilot is actively driving on the MFD.
TEST(FlightPlanPeerSyncTest, MfdFmsCursorAdvancesWhenPfdFplWindowOpen) {
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

  pfd.softkeyController().adoptFlightPlanFromPeer(kReproPlan,
                                                  /*destinationFilled=*/true, {});
  mfd.mfdController().adoptFlightPlanFromPeer(kReproPlan, /*destinationFilled=*/true,
                                              {});
  pfd.update(0.0);
  mfd.update(0.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  // Open the PFD Active Flight Plan window and the MFD Active Flight Plan page.
  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(pfd.softkeyController().activeWindow(), PfdWindow::FlightPlan);
  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  pfd.update(1.0 / 60.0);
  mfd.update(1.0 / 60.0);

  mfd.mfdController().pressBezelKey(BezelKey::FmsPush);  // MFD cursor on
  ASSERT_TRUE(mfd.mfdController().fplCursorOn());

  const int startLeg = mfd.mfdController().fplCursorLegIndexPublic();
  const int afterLeg = DriveMfdCursorDownTwoSteps(mfd, &pfd);
  EXPECT_GT(afterLeg, startLeg);
}

// Turns the MFD large knob `detents` times, settling engine frames between each
// detent like the real per-frame update loop, and reports the furthest leg index
// the selection cursor reached.
int DriveMfdLargeKnob(AvionicsEngine& mfd, AvionicsEngine* pfd, int detents) {
  MfdController& ui = mfd.mfdController();
  int maxLeg = ui.fplCursorLegIndexPublic();
  for (int i = 0; i < detents; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    for (int f = 0; f < 3; ++f) {
      if (pfd != nullptr) pfd->update(1.0 / 60.0);
      mfd.update(1.0 / 60.0);
    }
    maxLeg = std::max(maxLeg, ui.fplCursorLegIndexPublic());
  }
  return maxLeg;
}

// Faithful repro: the route comes from the map snapshot and the list cursor
// follows the active leg (FMA TO = BOSTN), exactly like the live standalone. The
// MFD FMS cursor must still walk down the leg list when only the MFD FPL page is
// open.
TEST(FlightPlanPeerSyncTest, MfdFmsCursorWalksLegListWithActiveLeg) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  mfd.mfdController().pressBezelKey(BezelKey::FmsPush);  // cursor on
  ASSERT_TRUE(mfd.mfdController().fplCursorOn());

  EXPECT_GE(DriveMfdLargeKnob(mfd, /*pfd=*/nullptr, /*detents=*/6),
            static_cast<int>(kReproPlan.size()) - 1);
}

// Faithful repro of the reported bug: same live active leg, but with the PFD
// Active Flight Plan window also open (its cursor passively following the active
// leg). The MFD FMS cursor must still walk to the end of the leg list.
TEST(FlightPlanPeerSyncTest, MfdFmsCursorWalksLegListWithPfdFplOpenAndActiveLeg) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(pfd.softkeyController().activeWindow(), PfdWindow::FlightPlan);
  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  for (int i = 0; i < 3; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  mfd.mfdController().pressBezelKey(BezelKey::FmsPush);  // MFD cursor on
  ASSERT_TRUE(mfd.mfdController().fplCursorOn());

  EXPECT_GE(DriveMfdLargeKnob(mfd, &pfd, /*detents=*/6),
            static_cast<int>(kReproPlan.size()) - 1);
}

// The MFD Active Flight Plan page opens with the FMS cursor inactive, like the
// real unit: the active leg is shown, but no selection cursor is engaged until
// the FMS knob is pushed.
TEST(FlightPlanPeerSyncTest, MfdFplPageOpensWithCursorInactive) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  EXPECT_FALSE(mfd.mfdController().fplCursorOn());
}

// With the cursor inactive, the large FMS knob navigates page groups (it steps
// out of the FPL group) instead of scrolling the leg list. The selection cursor
// only engages once the FMS knob is pushed.
TEST(FlightPlanPeerSyncTest, MfdFplCursorOffLargeKnobNavigatesPageGroups) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  ASSERT_FALSE(mfd.mfdController().fplCursorOn());

  mfd.mfdController().pressBezelKey(BezelKey::FmsOuterCw);
  EXPECT_NE(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
}

// With the cursor inactive, the small FMS knob steps pages within the FPL group
// (Active Flight Plan <-> Flight Plan Catalog), staying in the group.
TEST(FlightPlanPeerSyncTest, MfdFplCursorOffSmallKnobStepsPages) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  ASSERT_FALSE(mfd.mfdController().fplCursorOn());
  const MfdPage startPage = mfd.mfdController().page();

  mfd.mfdController().pressBezelKey(BezelKey::FmsInnerCw);
  EXPECT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  EXPECT_NE(mfd.mfdController().page(), startPage);
}

// With both displays open, the MFD page large knob (cursor off) must navigate
// page groups rather than scroll — and must not disturb the PFD window cursor.
TEST(FlightPlanPeerSyncTest, MfdFplCursorOffLargeKnobNavigatesWithPfdFplOpen) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(pfd.softkeyController().activeWindow(), PfdWindow::FlightPlan);
  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  for (int i = 0; i < 3; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  ASSERT_FALSE(mfd.mfdController().fplCursorOn());
  const int pfdRow0 = pfd.softkeyController().flightPlanCursor();
  mfd.mfdController().pressBezelKey(BezelKey::FmsOuterCw);
  EXPECT_NE(mfd.mfdController().pageGroup(), MfdPageGroup::FlightPlan);
  EXPECT_EQ(pfd.softkeyController().flightPlanCursor(), pfdRow0);
}

int ScrollPfdCursorOff(AvionicsEngine& pfd, AvionicsEngine* mfd, int detents) {
  SoftkeyController& ui = pfd.softkeyController();
  int maxRow = ui.flightPlanCursor();
  for (int i = 0; i < detents; ++i) {
    ui.pressBezelKey(BezelKey::FmsOuterCw);
    for (int f = 0; f < 3; ++f) {
      pfd.update(1.0 / 60.0);
      if (mfd != nullptr) mfd->update(1.0 / 60.0);
    }
    maxRow = std::max(maxRow, ui.flightPlanCursor());
  }
  return maxRow;
}

// Each GDU owns its own FMS list cursor. Driving one must not move the other
// when both the PFD window and the MFD page are visible.
TEST(FlightPlanPeerSyncTest, FplCursorsIndependentWhenBothDisplaysOpen) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  mfd.mfdController().pressBezelKey(BezelKey::Fpl);
  for (int i = 0; i < 3; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  const int pfdRow0 = pfd.softkeyController().flightPlanCursor();
  const int mfdRow0 = mfd.mfdController().fplCursorRow();

  // Drive the MFD selection cursor (knob pushed on) down the leg list; the PFD
  // window's own cursor must not move.
  mfd.mfdController().pressBezelKey(BezelKey::FmsPush);
  ASSERT_TRUE(mfd.mfdController().fplCursorOn());
  DriveMfdLargeKnob(mfd, &pfd, /*detents=*/4);
  const int mfdRowAfter = mfd.mfdController().fplCursorRow();
  EXPECT_GT(mfdRowAfter, mfdRow0);
  EXPECT_EQ(pfd.softkeyController().flightPlanCursor(), pfdRow0);

  // Scroll the PFD window (its own cursor); the MFD selection must hold.
  const int pfdAfter = ScrollPfdCursorOff(pfd, &mfd, /*detents=*/3);
  EXPECT_GT(pfdAfter, pfdRow0);
  EXPECT_EQ(mfd.mfdController().fplCursorRow(), mfdRowAfter);
}

// With the cursor on, the large knob must step the MFD selection from a leg's
// ident to its VNAV ALT column (and the column must survive the engine's
// per-frame flight-plan reconciliation so the small knob can then edit it).
TEST(FlightPlanPeerSyncTest, MfdCursorCanSelectAltitudeColumn) {
  NullRenderer renderer;
  PlanDataSource source(kReproPlan, /*activeFromIdent=*/"KFMY",
                        /*activeToIdent=*/"BOSTN");

  AvionicsEngine mfd(source, renderer, "TEST");
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  mfd.skipBoot();
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), 4u);

  MfdController& ui = mfd.mfdController();
  ui.pressBezelKey(BezelKey::Fpl);
  // Frames run between mouse clicks in the real app.
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  ASSERT_TRUE(ui.fplCursorOn());
  ASSERT_EQ(ui.fplCursorCol(), MfdController::FplCursorCol::Ident);

  // One large-knob detent moves from the ident to the ALT column on the same
  // leg row.
  ui.pressBezelKey(BezelKey::FmsOuterCw);
  EXPECT_EQ(ui.fplCursorCol(), MfdController::FplCursorCol::Altitude);

  // The ALT column selection must persist across engine frames (the per-frame
  // clamp must not snap it back to the ident column).
  for (int i = 0; i < 5; ++i) mfd.update(1.0 / 60.0);
  EXPECT_EQ(ui.fplCursorCol(), MfdController::FplCursorCol::Altitude);
}

// Drives the MFD Active Flight Plan page to insert a new enroute fix exactly the
// way the pilot does: cursor on, large knob down to the Enroute "add a fix"
// slot, small knob to open the entry, GCU keys to spell the ident, ENT to
// commit. Returns true once the fix is in the MFD's working route.
bool AddEnrouteFixOnMfd(AvionicsEngine& mfd, const std::string& ident) {
  MfdController& ui = mfd.mfdController();
  ui.pressBezelKey(BezelKey::Fpl);
  if (ui.pageGroup() != MfdPageGroup::FlightPlan) return false;
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on
  if (!ui.fplCursorOn()) return false;

  // Step the large knob until the selection lands on the Enroute blank "add a
  // fix" slot (a non-leg row in the Ident column). Filled leg rows also expose
  // the VNAV ALT column on the knob, so each takes an extra detent.
  bool onBlank = false;
  for (int i = 0; i < 16 && !onBlank; ++i) {
    mfd.update(1.0 / 60.0);
    const bool blankIdent =
        ui.fplCursorLegIndexPublic() < 0 &&
        ui.fplCursorCol() == MfdController::FplCursorCol::Ident &&
        ui.fplCursorRow() > 0;
    if (blankIdent) {
      onBlank = true;
      break;
    }
    ui.pressBezelKey(BezelKey::FmsOuterCw);
  }
  if (!onBlank) return false;

  ui.pressBezelKey(BezelKey::FmsInnerCw);  // open the ident entry box
  if (!ui.fplEntryActive()) return false;
  for (char ch : ident) ui.applyGcuEntryKey(ch);
  ui.pressBezelKey(BezelKey::Ent);  // commit the fix
  if (ui.fplEntryActive()) return false;
  for (const MapLeg& leg : ui.fplLegs()) {
    if (leg.id == ident) return true;
  }
  return false;
}

bool PlanContainsIdent(const std::vector<MapLeg>& legs, const std::string& id) {
  for (const MapLeg& leg : legs) {
    if (leg.id == id) return true;
  }
  return false;
}

const std::vector<MapLeg> kEnrouteAddPlan = {
    MakeLeg("KFMY", 26.586, -81.863),
    MakeLeg("RSW", 26.536, -81.755),
    MakeLeg("KCMI", 40.039, -88.278),
};

// Reported bug: a fix added in the Enroute section on the MFD Active Flight Plan
// page does not appear on the PFD Active Flight Plan window. The new fix must
// propagate to the PFD via the per-frame peer sync (and the published map plan).
TEST(FlightPlanPeerSyncTest, EnrouteFixAddedOnMfdAppearsOnPfd) {
  NullRenderer renderer;
  MutablePlanSource source(kEnrouteAddPlan,
                           {MakeFeature("SINCA", 27.10, -81.40)},
                           /*activeFromIdent=*/"KFMY", /*activeToIdent=*/"RSW");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  // Both GDUs adopt the route from the shared map snapshot and settle the peer
  // reconciliation (both hold the same agreed plan, no local draft yet).
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_EQ(mfd.mfdController().fplLegs().size(), kEnrouteAddPlan.size());
  ASSERT_EQ(pfd.softkeyController().flightPlanLegs().size(),
            kEnrouteAddPlan.size());

  ASSERT_TRUE(AddEnrouteFixOnMfd(mfd, "SINCA"));
  ASSERT_TRUE(PlanContainsIdent(mfd.mfdController().fplLegs(), "SINCA"));

  // Standalone publishes the consumed edit back into the shared map snapshot.
  std::vector<MapLeg> published;
  if (mfd.mfdController().consumeFlightPlanEdit(published)) {
    source.setPlan(published);
  }

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  EXPECT_TRUE(PlanContainsIdent(pfd.softkeyController().flightPlanLegs(), "SINCA"))
      << "enroute fix added on the MFD should appear on the PFD";
}

// Same as above but with the PFD Active Flight Plan window open while the fix is
// added on the MFD (the state the pilot is actually looking at when they notice
// the PFD did not update).
TEST(FlightPlanPeerSyncTest, EnrouteFixAddedOnMfdAppearsOnPfdWithPfdFplOpen) {
  NullRenderer renderer;
  MutablePlanSource source(kEnrouteAddPlan,
                           {MakeFeature("SINCA", 27.10, -81.40)},
                           /*activeFromIdent=*/"KFMY", /*activeToIdent=*/"RSW");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_EQ(pfd.softkeyController().flightPlanLegs().size(),
            kEnrouteAddPlan.size());

  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(pfd.softkeyController().activeWindow(), PfdWindow::FlightPlan);
  for (int i = 0; i < 3; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  ASSERT_TRUE(AddEnrouteFixOnMfd(mfd, "SINCA"));
  std::vector<MapLeg> published;
  if (mfd.mfdController().consumeFlightPlanEdit(published)) {
    source.setPlan(published);
  }

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  EXPECT_TRUE(PlanContainsIdent(pfd.softkeyController().flightPlanLegs(), "SINCA"))
      << "enroute fix added on the MFD should appear on the open PFD FPL window";
}

// Reported bug: an enroute fix added on the MFD never appears on the PFD when
// the PFD Active Flight Plan window has a waypoint-entry box open at the moment
// of the edit. The PFD's adopt is (correctly) blocked while its entry is open,
// but the peer sync still advances its change-detection baseline to the MFD's
// new plan -- so once the PFD entry closes, the PFD's stale route is mistaken
// for the freshly edited side and the MFD's fix is never propagated.
TEST(FlightPlanPeerSyncTest, EnrouteFixAddedOnMfdReachesPfdAfterPfdEntryCloses) {
  NullRenderer renderer;
  MutablePlanSource source(kEnrouteAddPlan,
                           {MakeFeature("SINCA", 27.10, -81.40)},
                           /*activeFromIdent=*/"KFMY", /*activeToIdent=*/"RSW");

  AvionicsEngine pfd(source, renderer, "TEST");
  AvionicsEngine mfd(source, renderer, "TEST");
  pfd.setPage(DisplayPage::PrimaryFlightDisplay);
  mfd.setPage(DisplayPage::MultiFunctionDisplay);
  pfd.setSoftkeyPeer(&mfd);
  mfd.setSoftkeyPeer(&pfd);
  pfd.skipBoot();
  mfd.skipBoot();

  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }
  ASSERT_EQ(pfd.softkeyController().flightPlanLegs().size(),
            kEnrouteAddPlan.size());

  // Open the PFD Active Flight Plan window and a waypoint-entry box (the small
  // knob on the highlighted ident row opens it).
  pfd.softkeyController().pressBezelKey(BezelKey::Fpl);
  pfd.softkeyController().pressBezelKey(BezelKey::FmsInnerCw);
  ASSERT_TRUE(pfd.softkeyController().flightPlanEntryActive());

  // Add the fix on the MFD while the PFD entry is open, and publish it back to
  // the shared map snapshot like the standalone does.
  ASSERT_TRUE(AddEnrouteFixOnMfd(mfd, "SINCA"));
  std::vector<MapLeg> published;
  if (mfd.mfdController().consumeFlightPlanEdit(published)) {
    source.setPlan(published);
  }
  for (int i = 0; i < 5; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  // Close the PFD entry box. The PFD must now pick up the MFD's fix.
  pfd.softkeyController().pressBezelKey(BezelKey::Clr);
  ASSERT_FALSE(pfd.softkeyController().flightPlanEntryActive());
  for (int i = 0; i < 10; ++i) {
    pfd.update(1.0 / 60.0);
    mfd.update(1.0 / 60.0);
  }

  EXPECT_TRUE(PlanContainsIdent(pfd.softkeyController().flightPlanLegs(), "SINCA"))
      << "fix added on the MFD must reach the PFD once its entry box closes";
}

}  // namespace
}  // namespace avionics
