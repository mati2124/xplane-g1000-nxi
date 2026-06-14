#include <algorithm>

#include "avionics/MfdController.h"

// Checklist page group: the displayed checklist, its item cursor, and the
// interactive checked state (kept here because it is UI state, not file data).
namespace avionics {

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

}  // namespace avionics
