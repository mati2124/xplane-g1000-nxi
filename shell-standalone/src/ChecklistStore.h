#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "avionics/Checklist.h"

namespace avionics {

// Loads author-supplied checklists for the MFD Checklist page group from a
// plain-text file (see parseChecklistText for the format). Checklists are not
// hardcoded: each aircraft can ship its own file, so the displayed checklists
// change with the aircraft.
//
// File selection:
//   - Non-empty selector: an absolute/relative path to a checklist file.
//   - Empty selector: falls back to the build-time sample (AVIONICS_DEFAULT_
//     CHECKLIST) so the page is populated during development. The X-Plane
//     plugin shell instead resolves a per-aircraft file next to the .acf.
//
// The initial resolve + parse runs on a background thread (like FmsPlanStore);
// refreshIfChanged() re-parses when the file's modification time advances, so an
// author can edit the file and see the MFD update without a restart.
class ChecklistStore : public ChecklistSource {
 public:
  explicit ChecklistStore(std::string selector = "");
  ~ChecklistStore() override;

  ChecklistStore(const ChecklistStore&) = delete;
  ChecklistStore& operator=(const ChecklistStore&) = delete;

  bool ready() const override { return loaded_.load(std::memory_order_acquire); }
  const ChecklistData& checklists() const override { return checklists_; }
  const std::string& sourcePath() const { return sourcePath_; }

  // Cheap mtime check; re-parses synchronously when the file changed.
  void refreshIfChanged();

 private:
  void loadOnBackgroundThread();

  std::string selector_;
  ChecklistData checklists_;
  std::string sourcePath_;
  std::int64_t lastMtimeNs_ = 0;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
