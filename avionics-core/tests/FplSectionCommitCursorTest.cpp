#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FplRouteEdit.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"
#include "render/pfd/PfdFlightPlanSections.h"

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

// Minimal nav backend resolving a single fix so FMS entry can spell it.
class FixSource : public NavFeatureSource {
 public:
  bool ready() const override { return true; }
  std::vector<MapFeature> nearby(double, double, float,
                                 std::size_t) const override {
    return {};
  }
  std::vector<MapFeature> lookupIdent(const std::string& ident,
                                      std::size_t) const override {
    if (ident == "GLOSS") {
      MapFeature f;
      f.id = "GLOSS";
      f.lat = 33.0;
      f.lon = -83.0;
      f.type = MapFeatureType::Fix;
      return {f};
    }
    return {};
  }
  std::string firstIdentWithPrefix(const std::string& prefix) const override {
    if (std::string("GLOSS").compare(0, prefix.size(), prefix) == 0) {
      return "GLOSS";
    }
    return {};
  }
};

// After committing a fix on a section row, the list cursor must land on the new
// leg so it is immediately selectable for removal/activation. Regression for the
// PFD FPL window leaving the cursor stranded on a blank slot after entry
// ("I can't even move my cursor to it to delete it").
TEST(FplSectionCommitCursor, CursorLandsOnCommittedLeg) {
  std::vector<MapLeg> legs;  // empty plan
  bool destinationFilled = false;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = 0;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow};

  // Find the Enroute "add a fix" blank selectable row for an empty plan.
  const auto rows = pfd::fplFilteredSectionRows(0, false, false, legs);
  int enrouteBlankSel = -1;
  int sel = 0;
  for (const pfd::FplSectionRow& sr : rows) {
    if (!pfd::fplSectionRowIsSelectable(sr, 0, false)) continue;
    if (sr.kind == pfd::FplSectionRow::Kind::EnrouteBlank) {
      enrouteBlankSel = sel;
      break;
    }
    ++sel;
  }
  ASSERT_GE(enrouteBlankSel, 0);

  MapFeature match;
  match.id = "GLOSS";
  cursorRow = enrouteBlankSel;
  ASSERT_TRUE(fplCommitWaypointIdent(edit, nullptr, match, "GLOSS",
                                     enrouteBlankSel, {},
                                     FplCursorLayout::SectionRows));
  ASSERT_EQ(legs.size(), 1u);

  // The cursor row must now resolve back to the leg that was just entered.
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 0);
}

// End-to-end PFD draft flow: open the Active Flight Plan window on an empty plan,
// spell a fix on the Enroute line, then delete it. Before the fix the cursor was
// left below the new leg and CLR never armed the Remove confirmation.
TEST(FplSectionCommitCursor, DraftEntryThenDeleteOnPfd) {
  FixSource src;
  SoftkeyController ui;
  ui.setNavFeatureSource(&src);

  ui.pressBezelKey(BezelKey::Fpl);
  ASSERT_EQ(ui.activeWindow(), PfdWindow::FlightPlan);
  ASSERT_EQ(ui.flightPlanLegs().size(), 0u);
  ui.pressBezelKey(BezelKey::FmsPush);  // cursor on

  // Move down onto the Enroute blank row, then spell GLOSS.
  ui.pressBezelKey(BezelKey::FmsOuterCw);
  ui.pressBezelKey(BezelKey::FmsOuterCw);
  ui.pressBezelKey(BezelKey::FmsInnerCw);  // opens entry
  for (char c : std::string("GLOSS")) ui.applyGcuEntryKey(c);
  ui.pressBezelKey(BezelKey::Ent);  // commit

  ASSERT_EQ(ui.flightPlanLegs().size(), 1u);
  ASSERT_EQ(ui.flightPlanLegs()[0].id, "GLOSS");

  // The cursor is now on GLOSS; a single CLR arms the Remove confirmation.
  ui.pressBezelKey(BezelKey::Clr);
  ASSERT_EQ(ui.flightPlanConfirm(),
            SoftkeyController::FplConfirm::RemoveWaypoint);
  EXPECT_EQ(ui.flightPlanRemoveIdent(), "GLOSS");

  ui.pressBezelKey(BezelKey::Ent);  // confirm OK
  EXPECT_FALSE(PlanContains(ui.flightPlanLegs(), "GLOSS"));
}

// Building a route the normal way still works: enter the origin airport, then a
// fix on the Enroute line lands under Enroute (not merged into the origin) and
// the cursor follows it.
TEST(FplSectionCommitCursor, OriginThenEnrouteFix) {
  std::vector<MapLeg> legs;
  bool destinationFilled = false;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = 0;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow};

  // Enter the origin airport on selectable row 0 (the Origin field).
  MapFeature kfmy;
  kfmy.id = "KFMY";
  ASSERT_TRUE(fplCommitWaypointIdent(edit, nullptr, kfmy, "KFMY", 0, {},
                                     FplCursorLayout::SectionRows));
  ASSERT_EQ(legs.size(), 1u);
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 0);

  // Now place the cursor on the Enroute blank and enter a fix.
  const auto rows =
      pfd::fplFilteredSectionRows(1, false, false, legs);
  int enrouteBlankSel = -1;
  int sel = 0;
  for (const pfd::FplSectionRow& sr : rows) {
    if (!pfd::fplSectionRowIsSelectable(sr, 1, false)) continue;
    if (sr.kind == pfd::FplSectionRow::Kind::EnrouteBlank) {
      enrouteBlankSel = sel;
      break;
    }
    ++sel;
  }
  ASSERT_GE(enrouteBlankSel, 0);

  MapFeature gloss;
  gloss.id = "GLOSS";
  ASSERT_TRUE(fplCommitWaypointIdent(edit, nullptr, gloss, "GLOSS",
                                     enrouteBlankSel, {},
                                     FplCursorLayout::SectionRows));
  ASSERT_EQ(legs.size(), 2u);
  EXPECT_EQ(legs[0].id, "KFMY");
  EXPECT_EQ(legs[1].id, "GLOSS") << "fix should follow the origin under Enroute";

  // The cursor follows the just-entered enroute fix (leg index 1).
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 1);
}

}  // namespace
}  // namespace avionics
