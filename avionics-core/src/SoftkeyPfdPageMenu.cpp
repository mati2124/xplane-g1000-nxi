#include "avionics/SoftkeyController.h"

#include "avionics/render/BezelKeys.h"

namespace avionics {

namespace {
constexpr const char* kEmptyItem = "";
constexpr int kPageMenuVisibleRows = 3;
}  // namespace

std::vector<SoftkeyController::PfdPageMenuItem>
SoftkeyController::buildPfdPageMenu() const {
  switch (window_) {
    case PfdWindow::References:
      return {{"All References On", PfdPageMenuAction::RefAllOn},
              {"All References Off", PfdPageMenuAction::RefAllOff},
              {"Restore Defaults", PfdPageMenuAction::RefRestoreDefaults}};
    case PfdWindow::Nearest:
    case PfdWindow::Alerts:
      return {};
    default:
      return {};
  }
}

void SoftkeyController::openPfdPageMenu() {
  pageMenuItems_ = buildPfdPageMenu();
  pageMenuOpen_ = true;
  pageMenuSel_ = 0;
  pageMenuScroll_ = 0;
  for (int i = 0; i < static_cast<int>(pageMenuItems_.size()); ++i) {
    if (pageMenuItems_[static_cast<std::size_t>(i)].action !=
        PfdPageMenuAction::Disabled) {
      pageMenuSel_ = i;
      break;
    }
  }
}

const std::string& SoftkeyController::pageMenuItemText(int i) const {
  static const std::string kNoOptions = "No Options";
  static const std::string kEmpty;
  if (pageMenuItems_.empty()) return kNoOptions;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return kEmpty;
  return pageMenuItems_[static_cast<std::size_t>(i)].text;
}

bool SoftkeyController::pageMenuItemEnabled(int i) const {
  if (pageMenuItems_.empty()) return false;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return false;
  return pageMenuItems_[static_cast<std::size_t>(i)].action !=
         PfdPageMenuAction::Disabled;
}

void SoftkeyController::pageMenuStep(int direction) {
  const int n = static_cast<int>(pageMenuItems_.size());
  if (n <= 0) return;
  const int step = direction >= 0 ? 1 : -1;
  for (int tries = 0; tries < n; ++tries) {
    pageMenuSel_ = (pageMenuSel_ + step + n) % n;
    if (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action !=
        PfdPageMenuAction::Disabled) {
      break;
    }
  }
  if (pageMenuSel_ < pageMenuScroll_) {
    pageMenuScroll_ = pageMenuSel_;
  } else if (pageMenuSel_ >= pageMenuScroll_ + kPageMenuVisibleRows) {
    pageMenuScroll_ = pageMenuSel_ - kPageMenuVisibleRows + 1;
  }
}

void SoftkeyController::pageMenuActivate() {
  if (pageMenuSel_ < 0 ||
      pageMenuSel_ >= static_cast<int>(pageMenuItems_.size())) {
    return;
  }
  switch (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action) {
    case PfdPageMenuAction::RefAllOn:
      vspeedOn_.fill(true);
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::RefAllOff:
      vspeedOn_.fill(false);
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::RefRestoreDefaults:
      for (int i = 0; i < kVspeedRefCount; ++i) {
        vspeedKt_[i] = kDefaultVspeedKt[i];
      }
      pageMenuOpen_ = false;
      break;
    case PfdPageMenuAction::Disabled:
      break;
  }
}

bool SoftkeyController::pageMenuBezelKey(BezelKey key) {
  if (!pageMenuOpen_) return false;
  switch (key) {
    case BezelKey::Ent:
      if (!pageMenuItems_.empty()) pageMenuActivate();
      return true;
    case BezelKey::Clr:
    case BezelKey::Menu:
    case BezelKey::FmsPush:
      pageMenuOpen_ = false;
      return true;
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsInnerCw:
      if (!pageMenuItems_.empty()) pageMenuStep(+1);
      return true;
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsInnerCcw:
      if (!pageMenuItems_.empty()) pageMenuStep(-1);
      return true;
    default:
      return true;
  }
}

}  // namespace avionics
