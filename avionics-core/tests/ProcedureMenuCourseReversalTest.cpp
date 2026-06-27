// Verifies the HILPT "Fly Course Reversal at <fix>?" prompt fires the moment a
// course-reversal transition is selected in the approach-loading window (as on
// the NXi), and that the YES/NO answer is deferred and applied when the approach
// is finally loaded -- including the preview Sequence dropping the HOLD on NO.

#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenu.h"
#include "avionics/ProcedureMenuTypes.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// Minimal nav backend: KCMI "R04" with a BOSTN transition that carries a HILPT
// course-reversal hold, plus a CMI transition (so a transition list opens).
class FakeNav : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapProcedure> proceduresForAirport(
      const std::string& icao, ProcedureType type) const override {
    if (icao != "KCMI" || type != ProcedureType::Approach) return {};
    MapProcedure proc;
    proc.type = ProcedureType::Approach;
    proc.name = "R04";
    proc.transition = "BOSTN";
    return {proc};
  }
  std::vector<ApproachTransitionOption> approachTransitionsFor(
      const std::string& icao, const std::string& approachName) const override {
    if (icao != "KCMI" || approachName != "R04") return {};
    return {{"BOSTN", "BOSTN iaf"}, {"CMI", "CMI"}};
  }
  std::vector<MapLeg> expandProcedure(const std::string& icao, ProcedureType,
                                      const std::string& name,
                                      const std::string& transition)
      const override {
    if (icao != "KCMI" || name != "R04") return {};
    std::vector<MapLeg> legs;
    MapLeg bostn;
    bostn.id = "BOSTN";
    if (transition == "BOSTN") {
      bostn.hold.active = true;
      bostn.hold.courseReversal = true;
      bostn.hold.turn = HoldTurnDirection::Right;
    }
    legs.push_back(bostn);
    MapLeg aftor;
    aftor.id = "AFTOR";
    legs.push_back(aftor);
    MapLeg rw04;
    rw04.id = "RW04";
    legs.push_back(rw04);
    return legs;
  }
};

// Backing storage + wiring for a ProcedureMenuHost, mirroring how the
// controllers build it.
struct Harness {
  FakeNav nav;
  MapData map;
  ProcedureMenuState state;
  std::vector<MapLeg> fplLegs;
  int approachLegStart = -1;
  int approachLegCount = 0;
  int cursorRow = 0;
  MapProcedure loadedApproach;
  PersistedLoadedApproach persistedRestore;
  std::string approachHeaderLabel;
  int publishCount = 0;

  ProcedureMenuHost host() {
    ProcedureMenuHost h{state,
                        &nav,
                        &map,
                        fplLegs,
                        approachLegStart,
                        approachLegCount,
                        cursorRow,
                        loadedApproach,
                        persistedRestore,
                        &approachHeaderLabel};
    h.defaultAirportIcao = [] { return std::string("KCMI"); };
    h.nearestAirportIds = [] { return std::vector<std::string>{}; };
    h.publishFlightPlanEdit = [this] { ++publishCount; };
    h.requestActivateLeg = [](int) {};
    h.requestActivateMissed = [] {};
    h.closeProceduresMenu = [] {};
    h.minimumsBaroEnabled = [] { return false; };
    h.setMinimumsBaro = [](bool) {};
    h.minimumsAltitudeFt = [] { return 0.0f; };
    h.setMinimumsAltitudeFt = [](float) {};
    return h;
  }
};

const MapLeg* findLeg(const std::vector<MapLeg>& legs, const std::string& id) {
  for (const MapLeg& leg : legs) {
    if (leg.id == id) return &leg;
  }
  return nullptr;
}

// Selecting the BOSTN transition in the loading window raises the prompt as a
// deferred decision (courseReversalLegIndex == -1), before any Load/Activate.
TEST(ProcedureMenuCourseReversal, PromptRaisedOnTransitionSelect) {
  Harness h;
  ProcedureMenuHost host = h.host();
  procedureMenuOpenApproachLoading(host, "KCMI", "R04", "BOSTN",
                                   ProcLoadingList::Transition);
  ASSERT_TRUE(h.state.subListOpen);

  procedureMenuBezelKey(host, BezelKey::Ent);  // confirm the BOSTN transition

  EXPECT_TRUE(h.state.courseReversalPromptActive);
  EXPECT_EQ(h.state.courseReversalFix, "BOSTN");
  EXPECT_EQ(h.state.courseReversalLegIndex, -1);  // not yet loaded
  EXPECT_TRUE(h.fplLegs.empty());                 // approach not committed yet
}

// Answering NO before Load drops the HOLD from the preview Sequence and, on
// Load, strips it from the committed approach without re-prompting.
TEST(ProcedureMenuCourseReversal, AnswerNoDeferredAppliedOnLoad) {
  Harness h;
  ProcedureMenuHost host = h.host();
  procedureMenuOpenApproachLoading(host, "KCMI", "R04", "BOSTN",
                                   ProcLoadingList::Transition);
  procedureMenuBezelKey(host, BezelKey::Ent);  // select BOSTN -> prompt
  ASSERT_TRUE(h.state.courseReversalPromptActive);

  procedureMenuAnswerCourseReversal(host, /*flyIt=*/false);
  EXPECT_FALSE(h.state.courseReversalPromptActive);
  EXPECT_TRUE(h.state.courseReversalDecisionMade);
  EXPECT_FALSE(h.state.courseReversalDecisionFlyIt);

  const std::vector<MapLeg> preview = procedureMenuPreviewLegs(host);
  const MapLeg* previewBostn = findLeg(preview, "BOSTN");
  ASSERT_NE(previewBostn, nullptr);
  EXPECT_FALSE(previewBostn->hold.courseReversal);  // HOLD dropped in preview

  procedureMenuBezelKey(host, BezelKey::Ent);  // Load (focus is armed on Load)

  const MapLeg* loadedBostn = findLeg(h.fplLegs, "BOSTN");
  ASSERT_NE(loadedBostn, nullptr);
  EXPECT_FALSE(loadedBostn->hold.courseReversal);  // stripped on commit
  EXPECT_FALSE(h.state.courseReversalPromptActive);  // not re-prompted
  EXPECT_FALSE(h.state.courseReversalDecisionMade);  // decision consumed
}

// Answering YES keeps the HILPT hold when the approach is loaded.
TEST(ProcedureMenuCourseReversal, AnswerYesKeepsHoldOnLoad) {
  Harness h;
  ProcedureMenuHost host = h.host();
  procedureMenuOpenApproachLoading(host, "KCMI", "R04", "BOSTN",
                                   ProcLoadingList::Transition);
  procedureMenuBezelKey(host, BezelKey::Ent);  // select BOSTN -> prompt
  ASSERT_TRUE(h.state.courseReversalPromptActive);

  procedureMenuAnswerCourseReversal(host, /*flyIt=*/true);
  EXPECT_FALSE(h.state.courseReversalPromptActive);

  procedureMenuBezelKey(host, BezelKey::Ent);  // Load

  const MapLeg* loadedBostn = findLeg(h.fplLegs, "BOSTN");
  ASSERT_NE(loadedBostn, nullptr);
  EXPECT_TRUE(loadedBostn->hold.courseReversal);  // hold retained
  EXPECT_FALSE(h.state.courseReversalPromptActive);
}

}  // namespace
}  // namespace avionics
