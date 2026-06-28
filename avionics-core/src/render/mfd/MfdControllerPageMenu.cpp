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
  // The Navigation Map and the Active Flight Plan page each define a Page Menu
  // in this suite (Pilot's Guide Fig. 5-6). Other pages return an empty list,
  // so MENU is inert there, matching the real unit's pages with no page menu.
  if (pageGroup_ == MfdPageGroup::FlightPlan &&
      page() == MfdPage::FlightPlanCatalog) {
    // Flight Plan Catalog page menu (Pilot's Guide, Flight Plan Storage). The
    // plan-specific rows grey out when the catalog is empty (nothing to act on),
    // matching the real unit. Delete All is menu-only.
    const bool hasPlans = !catalog_.empty();
    const PageMenuAction actState = hasPlans ? PageMenuAction::CatalogActivate
                                             : PageMenuAction::Disabled;
    const PageMenuAction invState =
        hasPlans ? PageMenuAction::CatalogInvertActivate
                 : PageMenuAction::Disabled;
    const PageMenuAction copyState =
        hasPlans ? PageMenuAction::CatalogCopy : PageMenuAction::Disabled;
    const PageMenuAction delState =
        hasPlans ? PageMenuAction::CatalogDelete : PageMenuAction::Disabled;
    const PageMenuAction delAllState =
        hasPlans ? PageMenuAction::CatalogDeleteAll : PageMenuAction::Disabled;
    return {
        {"Create New Flight Plan", PageMenuAction::CatalogCreateNew},
        {"Activate Flight Plan", actState},
        {"Invert & Activate FPL?", invState},
        {"Copy Flight Plan", copyState},
        {"Delete Flight Plan", delState},
        {"Delete All", delAllState},
    };
  }
  if (pageGroup_ == MfdPageGroup::FlightPlan) {
    // The Active Flight Plan page menu, verbatim from the trainer in on-unit
    // order and enable state. Delete Flight Plan opens the confirmation; every
    // other row's feature is not modeled, so the rows the trainer shows active
    // are DisplayOnly (selectable, inert) and the rows it greys are Disabled.

    // Load Airway is live only when the list cursor sits on a fix that lies on
    // a published airway (Pilot's Guide, Load Airway); otherwise it greys out.
    const int cursorLeg = fplCursorLegIndex();
    PageMenuAction loadAirwayState = PageMenuAction::Disabled;
    if (fplCursorOn_ && cursorLeg >= 0 &&
        cursorLeg < static_cast<int>(fplLegs_.size())) {
      const std::string& ident =
          fplLegs_[static_cast<std::size_t>(cursorLeg)].id;
      if (!airwaysThroughFix(ident).empty()) {
        loadAirwayState = PageMenuAction::FplLoadAirway;
      }
    }
    // Collapse/Expand Airways toggles the loaded-airway display; it is only
    // live (and only changes its label) when the plan carries airway legs.
    const bool hasAirways = fplHasAirwayLegs();
    const char* collapseText =
        fplAirwaysCollapsed_ ? "Expand Airways" : "Collapse Airways";
    const PageMenuAction collapseState =
        hasAirways ? PageMenuAction::FplCollapseAirways
                   : PageMenuAction::DisplayOnly;
    return {
        {"Load Airway", loadAirwayState},
        {collapseText, collapseState},
        {"Hold At Waypoint", PageMenuAction::Disabled},
        {"Hold At Present Position", PageMenuAction::DisplayOnly},
        {"Create ATK Offset Waypoint", PageMenuAction::Disabled},
        {"VNV", PageMenuAction::DisplayOnly, /*dtoSuffix=*/true},
        {"Select VNV Profile Window", PageMenuAction::DisplayOnly},
        {"Cancel VNV", PageMenuAction::DisplayOnly},
        {"Delete Flight Plan", PageMenuAction::FplDeleteFlightPlan},
        {"Store Flight Plan", PageMenuAction::DisplayOnly},
        {"Invert Flight Plan", PageMenuAction::DisplayOnly},
        {"Temperature Compensation", PageMenuAction::DisplayOnly},
        {"Create New User Waypoint", PageMenuAction::Disabled},
        {"Remove Departure", PageMenuAction::Disabled},
        {"Remove Arrival", PageMenuAction::Disabled},
        {"Remove Approach", PageMenuAction::DisplayOnly},
    };
  }
  if (chartViewActive_) {
    // Chart Setup menu (Pilot's Guide §8.3, Figs 8-24..8-27): Full Screen and
    // Color Scheme are modeled; Preferred Charts Source is Navigraph-only here,
    // so it is shown but inert.
    std::string fullScreen = "Full Screen (";
    fullScreen += chartsFullScreen_ ? "On" : "Off";
    fullScreen += ")";
    std::string colorScheme = "Color Scheme (";
    colorScheme += chartsNight_ ? "Night" : "Day";
    colorScheme += ")";
    return {
        {fullScreen, PageMenuAction::ChartsFullScreen},
        {colorScheme, PageMenuAction::ChartsColorScheme},
        {"Preferred Charts Source (Navigraph)", PageMenuAction::DisplayOnly},
    };
  }
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

bool MfdController::pageMenuItemDtoSuffix(int i) const {
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return false;
  return pageMenuItems_[static_cast<std::size_t>(i)].dtoSuffix;
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
    case PageMenuAction::FplLoadAirway: {
      pageMenuOpen_ = false;  // the page menu closes as the window opens
      const int cursorLeg = fplCursorLegIndex();
      if (cursorLeg >= 0 && cursorLeg < static_cast<int>(fplLegs_.size())) {
        openLoadAirwayWindow(fplLegs_[static_cast<std::size_t>(cursorLeg)].id);
      }
      break;
    }
    case PageMenuAction::FplCollapseAirways:
      fplAirwaysCollapsed_ = !fplAirwaysCollapsed_;
      fplClampCursorRow();  // the row count changes; keep the cursor in range
      pageMenuOpen_ = false;
      break;
    case PageMenuAction::FplDeleteFlightPlan:
      pageMenuOpen_ = false;  // the page menu closes as the confirmation opens
      fplConfirm_ = FplConfirm::DeleteFlightPlan;
      fplConfirmOk_ = true;
      break;
    case PageMenuAction::ChartsFullScreen:
      chartsFullScreen_ = !chartsFullScreen_;
      pageMenuItems_ = buildPageMenu();  // refresh the On/Off label, stay open
      break;
    case PageMenuAction::ChartsColorScheme:
      chartsNight_ = !chartsNight_;
      pageMenuItems_ = buildPageMenu();  // refresh the Day/Night label, stay open
      break;
    case PageMenuAction::CatalogCreateNew:
      pageMenuOpen_ = false;
      catalogCreateNew();
      break;
    case PageMenuAction::CatalogActivate:
      pageMenuOpen_ = false;  // the menu closes as the confirmation opens
      catalogConfirm_ = CatalogConfirm::Activate;
      catalogConfirmOk_ = true;
      break;
    case PageMenuAction::CatalogInvertActivate:
      pageMenuOpen_ = false;
      catalogConfirm_ = CatalogConfirm::InvertActivate;
      catalogConfirmOk_ = true;
      break;
    case PageMenuAction::CatalogCopy:
      pageMenuOpen_ = false;
      catalogCopySelected();
      break;
    case PageMenuAction::CatalogDelete:
      pageMenuOpen_ = false;
      catalogConfirm_ = CatalogConfirm::Delete;
      catalogConfirmOk_ = true;
      break;
    case PageMenuAction::CatalogDeleteAll:
      pageMenuOpen_ = false;
      catalogConfirm_ = CatalogConfirm::DeleteAll;
      catalogConfirmOk_ = true;
      break;
    case PageMenuAction::DisplayOnly:
    case PageMenuAction::Disabled:
      break;  // inert: the underlying feature is not modeled, so leave the menu
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
