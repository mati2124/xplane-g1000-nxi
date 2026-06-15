#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "avionics/Checklist.h"

namespace avionics {

// Loads author-supplied checklists for the MFD Checklist page group from a
// plain-text file (see parseChecklistText for the format). Checklists are not
// hardcoded: the displayed set changes with the loaded aircraft.
//
// File selection (first match wins):
//   - Non-empty selector: an absolute/relative path to a checklist file.
//   - aircraftAcfRelativePath set: g1000_checklist.txt and
//     <acf_stem>_checklist.txt next to the loaded .acf (so an aircraft can ship
//     its own checklists).
//   - Otherwise: the bundled checklist for the detected AircraftProfile (keyed
//     off acf_ICAO / path), defaulting to the Cessna piston set.
//
// The initial resolve + parse runs on a background thread; refreshIfChanged()
// re-parses when the file's mtime advances, so an author can edit the file and
// see the MFD update without a restart. setAircraftIdentity() swaps profiles
// (and reloads) when the user changes aircraft.
class ChecklistStore : public ChecklistSource {
 public:
  explicit ChecklistStore(std::string selector = "");
  ~ChecklistStore() override;

  ChecklistStore(const ChecklistStore&) = delete;
  ChecklistStore& operator=(const ChecklistStore&) = delete;

  bool ready() const override { return loaded_.load(std::memory_order_acquire); }
  const ChecklistData& checklists() const override { return checklists_; }
  const std::string& sourcePath() const { return sourcePath_; }

  void setAircraftIdentity(const std::string& icaoType,
                           const std::string& acfRelativePath) override;

  void refreshIfChanged() override;

 private:
  void loadOnBackgroundThread();
  void resolveAndLoad();
  std::string resolvePath() const;

  std::string selector_;
  std::string aircraftIcao_;
  std::string aircraftAcfRelativePath_;
  ChecklistData checklists_;
  std::string sourcePath_;
  std::int64_t lastMtimeNs_ = 0;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
