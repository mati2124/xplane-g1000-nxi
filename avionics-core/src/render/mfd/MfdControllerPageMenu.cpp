#include "avionics/MfdController.h"

// Page Menu (MENU bezel key, Pilot's Guide Fig. 5-6): the context popout listing
// the current page's options. Only the Navigation Map defines one in this suite.
namespace avionics {
namespace {

// Declutter level as the Navigation Map Page Menu shows it: "Declutter
// (Current Detail All)" / "...3)" / "...2)" / "...1)" (Pilot's Guide Fig. 5-6).
const char* declutterLevelText(MapDetail d) {
  switch (d) {
    case MapDetail::Detail3:
      return "3";
    case MapDetail::Detail2:
      return "2";
    case MapDetail::Detail1:
      return "1";
    case MapDetail::All:
      break;
  }
  return "All";
}

}  // namespace

std::vector<MfdController::PageMenuItem> MfdController::buildPageMenu() const {
  // Only the Navigation Map page defines a Page Menu in this suite for now
  // (Pilot's Guide Fig. 5-6). Other pages return an empty list, so MENU is
  // inert there, matching the real unit's pages that have no page menu.
  if (pageGroup_ != MfdPageGroup::Map || page() != MfdPage::NavigationMap) {
    return {};
  }
  // Verbatim from the figure, in on-unit order. Map Settings opens the Map
  // Settings window (Fig. 5-7); Declutter cycles the map Detail level. Measure
  // Bearing/Distance and Show VSD need tools this suite has not yet modeled,
  // so they are listed disabled; Charts is greyed on the real unit too.
  std::string declutter = "Declutter (Current Detail ";
  declutter += declutterLevelText(detail_);
  declutter += ")";
  return {
      {"Map Settings", PageMenuAction::OpenMapSettings},
      {declutter, PageMenuAction::MapDeclutter},
      {"Measure Bearing/Distance", PageMenuAction::Disabled},
      {"Charts", PageMenuAction::Disabled},
      {"Show VSD", PageMenuAction::Disabled},
  };
}

void MfdController::openPageMenu() {
  pageMenuItems_ = buildPageMenu();
  if (pageMenuItems_.empty()) return;  // no page menu on this page
  pageMenuOpen_ = true;
  // Highlight the first enabled option (the cursor never parks on a disabled
  // row, which the real unit skips).
  pageMenuSel_ = 0;
  for (int i = 0; i < static_cast<int>(pageMenuItems_.size()); ++i) {
    if (pageMenuItems_[i].action != PageMenuAction::Disabled) {
      pageMenuSel_ = i;
      break;
    }
  }
}

const std::string& MfdController::pageMenuItemText(int i) const {
  static const std::string kEmpty;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return kEmpty;
  return pageMenuItems_[static_cast<std::size_t>(i)].text;
}

bool MfdController::pageMenuItemEnabled(int i) const {
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return false;
  return pageMenuItems_[static_cast<std::size_t>(i)].action !=
         PageMenuAction::Disabled;
}

void MfdController::pageMenuStep(int direction) {
  const int n = static_cast<int>(pageMenuItems_.size());
  if (n == 0) return;
  const int step = direction >= 0 ? 1 : -1;
  // Walk in the requested direction to the next enabled option, wrapping.
  for (int i = 0; i < n; ++i) {
    pageMenuSel_ = (pageMenuSel_ + step + n) % n;
    if (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action !=
        PageMenuAction::Disabled) {
      return;
    }
  }
}

void MfdController::pageMenuActivate() {
  if (pageMenuSel_ < 0 ||
      pageMenuSel_ >= static_cast<int>(pageMenuItems_.size())) {
    return;
  }
  switch (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action) {
    case PageMenuAction::MapDeclutter:
      detail_ = nextMapDetail(detail_);
      pageMenuOpen_ = false;  // the option runs and closes the menu
      break;
    case PageMenuAction::OpenMapSettings:
      pageMenuOpen_ = false;  // the page menu closes as the window opens
      openMapSettings();
      break;
    case PageMenuAction::Disabled:
      break;  // inert: a disabled row is never highlighted, so this is a no-op
  }
}

bool MfdController::pageMenuBezelKey(BezelKey key) {
  switch (key) {
    case BezelKey::Ent:
      pageMenuActivate();
      break;
    case BezelKey::Clr:
    case BezelKey::Menu:
    case BezelKey::FmsPush:
      pageMenuOpen_ = false;  // back out to the base page
      break;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsOuterCw:
      pageMenuStep(1);
      break;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsOuterCcw:
      pageMenuStep(-1);
      break;
    default:
      break;
  }
  return true;
}

}  // namespace avionics
