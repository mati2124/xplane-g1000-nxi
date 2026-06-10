#include "avionics/MfdController.h"

#include <algorithm>

#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// Softkey cell assignments for the MFD bar. The four page groups form a radio
// group on the left (standing in for the large FMS knob); range control sits
// on the right two cells, mirroring the G1000 MFD softkey layout.
constexpr int kKeyMap = 0;
constexpr int kKeyWaypoint = 1;
constexpr int kKeyAux = 2;
constexpr int kKeyNearest = 3;
constexpr int kKeyOrient = 4;
constexpr int kKeyTerrain = 5;
constexpr int kKeyChecklist = 7;  // CHKLIST: selects the Checklist page group
constexpr int kKeyRangeDown = 10;
constexpr int kKeyRangeUp = 11;

// First page of each group, in MfdPage enum order. Keep in sync with the page
// counts below.
constexpr MfdPage kGroupFirstPage[] = {
    MfdPage::NavigationMap,        // Map
    MfdPage::AirportInformation,   // Waypoint
    MfdPage::TripPlanning,         // Aux
    MfdPage::NearestAirports,      // Nearest
    MfdPage::ActiveFlightPlan,     // FlightPlan
};

constexpr int kGroupPageCount[] = {
    1,  // Map: Navigation Map
    4,  // Waypoint: Airport / Intersection / NDB / VOR Information
    3,  // Aux: Trip Planning / GPS Status / System Status
    5,  // Nearest: Airports / Intersections / NDB / VOR / Airspaces
    1,  // FlightPlan: Active Flight Plan
};

}  // namespace

MfdController::MfdController() {
  labels_[kKeyMap] = "Map";
  labels_[kKeyWaypoint] = "WPT";
  labels_[kKeyAux] = "AUX";
  labels_[kKeyNearest] = "NRST";
  labels_[kKeyOrient] = "TRK";
  labels_[kKeyTerrain] = "TERR";
  labels_[kKeyChecklist] = "CHKLIST";
  labels_[kKeyRangeDown] = "RNG-";
  labels_[kKeyRangeUp] = "RNG+";
}

float MfdController::rangeNm() const { return mapRangeNmAt(rangeIndex_); }

int MfdController::pageCount(MfdPageGroup group) {
  return kGroupPageCount[static_cast<int>(group)];
}

int MfdController::pageIndex() const {
  if (pageGroup_ == MfdPageGroup::Checklist) return checklistIndex_;
  return pageIndex_[static_cast<int>(pageGroup_)];
}

MfdPage MfdController::page() const {
  return static_cast<MfdPage>(
      static_cast<int>(kGroupFirstPage[static_cast<int>(pageGroup_)]) +
      pageIndex());
}

void MfdController::stepPage(int direction) {
  if (pageGroup_ == MfdPageGroup::Checklist) {
    stepChecklist(direction);
    return;
  }
  const int count = pageCount(pageGroup_);
  int& index = pageIndex_[static_cast<int>(pageGroup_)];
  index = ((index + direction) % count + count) % count;
}

int MfdController::checklistCount() const {
  return checklist_ != nullptr ? checklist_->totalChecklists() : 0;
}

int MfdController::checklistItemCount(int checklistIndex) const {
  if (checklist_ == nullptr) return 0;
  const Checklist* c = checklist_->at(checklistIndex);
  return c != nullptr ? static_cast<int>(c->items.size()) : 0;
}

bool MfdController::checklistItemChecked(int checklistIndex,
                                        int itemIndex) const {
  if (checklistIndex < 0 ||
      checklistIndex >= static_cast<int>(checked_.size())) {
    return false;
  }
  const std::vector<std::uint8_t>& items = checked_[checklistIndex];
  if (itemIndex < 0 || itemIndex >= static_cast<int>(items.size())) return false;
  return items[itemIndex] != 0;
}

void MfdController::stepChecklist(int direction) {
  const int count = checklistCount();
  if (count <= 0) {
    checklistIndex_ = 0;
    cursorItem_ = 0;
    return;
  }
  checklistIndex_ = ((checklistIndex_ + direction) % count + count) % count;
  cursorItem_ = 0;
}

void MfdController::syncChecklist(const ChecklistData& data) {
  checklist_ = &data;
  const int total = data.totalChecklists();

  // Steady-state fast path: when the file's shape is unchanged (the common
  // case, since this runs every frame) keep the existing checked flags and skip
  // the reallocation.
  bool sameShape = static_cast<int>(checked_.size()) == total;
  for (int ci = 0; sameShape && ci < total; ++ci) {
    if (static_cast<int>(checked_[ci].size()) != checklistItemCount(ci)) {
      sameShape = false;
    }
  }
  if (!sameShape) {
    // Rebuild to match the file, preserving flags whose checklist/item still
    // exists so a live reload does not wipe progress.
    std::vector<std::vector<std::uint8_t>> resized(
        static_cast<std::size_t>(total));
    for (int ci = 0; ci < total; ++ci) {
      const int items = checklistItemCount(ci);
      std::vector<std::uint8_t> row(static_cast<std::size_t>(items), 0);
      if (ci < static_cast<int>(checked_.size())) {
        const std::vector<std::uint8_t>& old = checked_[ci];
        const std::size_t n = std::min(row.size(), old.size());
        for (std::size_t i = 0; i < n; ++i) row[i] = old[i];
      }
      resized[static_cast<std::size_t>(ci)] = std::move(row);
    }
    checked_ = std::move(resized);
  }

  // Clamp the selection in case the file shrank.
  if (checklistIndex_ >= total) checklistIndex_ = total > 0 ? total - 1 : 0;
  const int itemCount = checklistItemCount(checklistIndex_);
  if (cursorItem_ > itemCount) cursorItem_ = itemCount;
}

void MfdController::selectGroup(MfdPageGroup group) {
  // Pressing the already-selected group's key steps to its next page (the
  // softkey doubles as the small FMS knob); selecting a new group restores
  // that group's last-viewed page.
  if (pageGroup_ == group) {
    stepPage(1);
  } else {
    pageGroup_ = group;
  }
}

void MfdController::update(double dtSeconds) {
  const float pressStep = static_cast<float>(dtSeconds) / kPressFlashSeconds;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    press_[i] = std::max(0.0f, press_[i] - pressStep);
  }
  for (int i = 0; i < kBezelKeyCount; ++i) {
    bezelPress_[i] = std::max(0.0f, bezelPress_[i] - pressStep);
  }
}

bool MfdController::keyActive(int i) const {
  switch (i) {
    case kKeyMap:
      return pageGroup_ == MfdPageGroup::Map;
    case kKeyWaypoint:
      return pageGroup_ == MfdPageGroup::Waypoint;
    case kKeyAux:
      return pageGroup_ == MfdPageGroup::Aux;
    case kKeyNearest:
      return pageGroup_ == MfdPageGroup::Nearest;
    case kKeyChecklist:
      return pageGroup_ == MfdPageGroup::Checklist;
    case kKeyOrient:
      return mapOrientation_ == MapOrientation::TrackUp;
    case kKeyTerrain:
      return showTerrain_;
    default:
      return false;
  }
}

void MfdController::pressBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i < 0 || i >= kBezelKeyCount) return;
  bezelPress_[i] = 1.0f;  // trigger the press-flash animation
  switch (key) {
    case BezelKey::RangeUp:
      rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
      break;
    case BezelKey::RangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      break;
    case BezelKey::Fpl:
      // FPL toggles the Active Flight Plan page; pressing it again returns to
      // the page that was displayed before.
      if (pageGroup_ == MfdPageGroup::FlightPlan) {
        pageGroup_ = groupBeforeFpl_;
      } else {
        groupBeforeFpl_ = pageGroup_;
        pageGroup_ = MfdPageGroup::FlightPlan;
      }
      break;
    case BezelKey::FmsNext:
      // Small FMS knob: move the item cursor down the checklist, else step
      // pages within the active group.
      if (pageGroup_ == MfdPageGroup::Checklist) {
        cursorItem_ =
            std::min(checklistItemCount(checklistIndex_), cursorItem_ + 1);
      } else {
        stepPage(1);
      }
      break;
    case BezelKey::FmsPrev:
      if (pageGroup_ == MfdPageGroup::Checklist) {
        cursorItem_ = std::max(0, cursorItem_ - 1);
      } else {
        stepPage(-1);
      }
      break;
    case BezelKey::Ent:
      if (pageGroup_ == MfdPageGroup::Checklist) checklistEnter();
      break;
    case BezelKey::Clr:
      if (pageGroup_ == MfdPageGroup::Checklist) checklistClear();
      break;
    default:
      break;
  }
}

void MfdController::checklistEnter() {
  const int itemCount = checklistItemCount(checklistIndex_);
  // On the "go to next checklist?" prompt below the last item: advance.
  if (cursorItem_ >= itemCount) {
    stepChecklist(1);
    return;
  }
  if (checklistIndex_ < static_cast<int>(checked_.size()) &&
      cursorItem_ < static_cast<int>(checked_[checklistIndex_].size())) {
    checked_[checklistIndex_][cursorItem_] = 1;
  }
  cursorItem_ = std::min(itemCount, cursorItem_ + 1);  // auto-advance
}

void MfdController::checklistClear() {
  const int itemCount = checklistItemCount(checklistIndex_);
  if (cursorItem_ < itemCount &&
      checklistIndex_ < static_cast<int>(checked_.size()) &&
      cursorItem_ < static_cast<int>(checked_[checklistIndex_].size())) {
    checked_[checklistIndex_][cursorItem_] = 0;
  }
}

bool MfdController::pressKey(int key) {
  if (key < 0 || key >= kSoftkeyCount || labels_[key].empty()) return false;

  press_[key] = 1.0f;  // trigger the press-flash animation

  switch (key) {
    case kKeyMap:
      selectGroup(MfdPageGroup::Map);
      break;
    case kKeyWaypoint:
      selectGroup(MfdPageGroup::Waypoint);
      break;
    case kKeyAux:
      selectGroup(MfdPageGroup::Aux);
      break;
    case kKeyNearest:
      selectGroup(MfdPageGroup::Nearest);
      break;
    case kKeyChecklist:
      // Selecting the group enters it; pressing again steps to the next
      // checklist (selectGroup -> stepPage -> stepChecklist).
      selectGroup(MfdPageGroup::Checklist);
      break;
    case kKeyOrient:
      mapOrientation_ = (mapOrientation_ == MapOrientation::NorthUp)
                            ? MapOrientation::TrackUp
                            : MapOrientation::NorthUp;
      break;
    case kKeyTerrain:
      showTerrain_ = !showTerrain_;
      break;
    case kKeyRangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      break;
    case kKeyRangeUp:
      rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
      break;
    default:
      return false;
  }
  return true;
}

}  // namespace avionics
