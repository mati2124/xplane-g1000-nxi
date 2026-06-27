#include <gtest/gtest.h>

#include "avionics/Checklist.h"
#include "avionics/MfdController.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

const char kTwoChecklists[] = R"(
GROUP NORMAL
  CHECKLIST FIRST
    Item A : OK
    Item B : OK
  CHECKLIST SECOND
    Item C : OK
)";

TEST(ChecklistPageTest, EnterOnNextChecklistPromptAdvances) {
  ChecklistData data = parseChecklistText(kTwoChecklists);
  MfdController ui;
  ui.syncChecklist(data);
  ASSERT_TRUE(ui.pressKey(11));  // Checklist softkey

  ui.pressBezelKey(BezelKey::FmsInnerCw);
  ui.pressBezelKey(BezelKey::FmsInnerCw);
  ASSERT_EQ(ui.checklistCursor(), 2);

  ui.pressBezelKey(BezelKey::Ent);
  EXPECT_EQ(ui.checklistIndex(), 1);
  EXPECT_EQ(ui.checklistCursor(), 0);
}

TEST(ChecklistPageTest, OwnsLocalFmsInputOnChecklistPage) {
  ChecklistData data = parseChecklistText(kTwoChecklists);
  MfdController ui;
  ui.syncChecklist(data);
  ASSERT_TRUE(ui.pressKey(11));
  EXPECT_TRUE(ui.ownsLocalFmsInput());
  ui.pressBezelKey(BezelKey::FmsOuterCw);  // leave Checklist group
  EXPECT_FALSE(ui.ownsLocalFmsInput());
}

}  // namespace
}  // namespace avionics
