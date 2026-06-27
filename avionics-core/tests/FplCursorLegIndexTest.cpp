#include "avionics/FplRouteEdit.h"

#include <gtest/gtest.h>

#include <vector>

#include "avionics/MapData.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id) {
  MapLeg leg;
  leg.id = id;
  return leg;
}

TEST(FplCursorLegIndexTest, KfmyKjaxRouteCursorOnTebow) {
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"),
      makeLeg("JINOS"), makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  const int tebowLeg = 4;
  const auto rows = pfd::fplFilteredSectionRows(static_cast<int>(legs.size()),
                                                true, false, legs);
  const int tebowRow = pfd::fplSectionSelectableRowForLegIndex(
      tebowLeg, rows, static_cast<int>(legs.size()), true);
  ASSERT_GE(tebowRow, 0);

  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = tebowRow;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow, nullptr, nullptr};
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows),
            tebowLeg);
}

TEST(FplCursorLegIndexTest, KfmyKjaxRouteCursorOnKjax) {
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"),
      makeLeg("JINOS"), makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  const int kjaxLeg = 5;
  const auto rows = pfd::fplFilteredSectionRows(static_cast<int>(legs.size()),
                                                true, false, legs);
  const int kjaxRow = pfd::fplSectionSelectableRowForLegIndex(
      kjaxLeg, rows, static_cast<int>(legs.size()), true);
  ASSERT_GE(kjaxRow, 0);

  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = kjaxRow;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow, nullptr, nullptr};
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), kjaxLeg);
}

TEST(FplCursorLegIndexTest, HiddenDuplicateCursorOnKjaxStillResolvesKjax) {
  // When an earlier duplicate is hidden, KJAX stays on the last selectable row
  // but the old unfiltered row math would map that row to TEBOW instead.
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("JINOS"), makeLeg("LAL"),
      makeLeg("JINOS"), makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  const auto rows = pfd::fplFilteredSectionRows(static_cast<int>(legs.size()),
                                                true, false, legs);
  const int kjaxLeg = 5;
  const int kjaxRow =
      pfd::fplSectionSelectableRowForLegIndex(kjaxLeg, rows,
                                              static_cast<int>(legs.size()),
                                              true);
  ASSERT_GE(kjaxRow, 0);

  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = kjaxRow;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow, nullptr, nullptr};
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows),
            kjaxLeg);
  EXPECT_NE(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 4);
}

TEST(FplCursorLegIndexTest, HiddenDuplicateDoesNotShiftCursorLeg) {
  // Earlier duplicate is hidden from the list; the cursor row for TEBOW must
  // still resolve to TEBOW, not the visible leg one slot earlier.
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("JINOS"), makeLeg("LAL"),
      makeLeg("JINOS"), makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  const auto rows = pfd::fplFilteredSectionRows(static_cast<int>(legs.size()),
                                                true, false, legs);
  const int tebowLeg = 4;
  const int tebowRow =
      pfd::fplSectionSelectableRowForLegIndex(tebowLeg, rows,
                                              static_cast<int>(legs.size()),
                                              true);
  ASSERT_GE(tebowRow, 0);

  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 0;
  int cursorRow = tebowRow;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow, nullptr, nullptr};
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows),
            tebowLeg);
  EXPECT_NE(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 3);
}

TEST(FplCursorLegIndexTest, DepartureProcedureDisplayCursorOnKjax) {
  std::vector<MapLeg> legs = {
      makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("LAL"),
      makeLeg("JINOS"), makeLeg("TEBOW"), makeLeg("KJAX"),
  };
  int depStart = 1;
  int depCount = 1;
  std::string depHeader = "KFMY-RW05.CSHEL8";
  int arrStart = 0;
  int arrCount = 0;
  std::string arrHeader;
  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 0;
  const int kjaxLeg = 5;
  const bool blankOrigin = true;
  const bool destFilled = true;
  const int kjaxRow = pfd::fplProcedureSelectableRowForLegIndex(
      kjaxLeg, legs, depStart, depCount, depHeader, arrStart, arrCount,
      arrHeader, approachStart, approachCount, blankOrigin, destFilled);
  ASSERT_GE(kjaxRow, 0);

  int cursorRow = kjaxRow;
  FplRouteEdit edit{legs, destinationFilled, approachStart, approachCount,
                    cursorRow, nullptr, nullptr};
  fplRouteEditWireTerminalProcedures(edit, depStart, depCount, depHeader, arrStart,
                                     arrCount, arrHeader);
  EXPECT_EQ(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), kjaxLeg);
  EXPECT_NE(fplCursorLegIndex(edit, {}, FplCursorLayout::SectionRows), 4);
}

}  // namespace
}  // namespace avionics
