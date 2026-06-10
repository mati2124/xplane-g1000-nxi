#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Loads an X-Plane v11 .fms flight plan from the install's Output/FMS plans/
// directory so the standalone shell can draw the route on the inset map.
//
// X-Plane does not expose the live FMS over UDP, so this is the practical
// offline source: parse the plan file Little Navmap / XPFlightPlanner exports,
// or the user loads from the G1000 CO ROUTE LIST.
//
// Plan selection:
//   - Non-empty selector: absolute path, or a name under Output/FMS plans/
//     (".fms" is appended when missing).
//   - Empty selector: the most recently modified .fms in that directory.
//
// The initial resolve + parse runs on a background thread (like NavDataStore).
// refreshIfChanged() re-parses when the file's modification time advances.
class FmsPlanStore {
 public:
  explicit FmsPlanStore(std::string planSelector = "");
  ~FmsPlanStore();

  FmsPlanStore(const FmsPlanStore&) = delete;
  FmsPlanStore& operator=(const FmsPlanStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  const std::string& sourcePath() const { return sourcePath_; }
  const std::vector<MapLeg>& flightPlan() const { return flightPlan_; }

  // Cheap mtime check; re-parses synchronously when the file changed.
  void refreshIfChanged();

 private:
  void loadOnBackgroundThread();

  std::string planSelector_;
  std::vector<MapLeg> flightPlan_;
  std::string sourcePath_;
  std::int64_t lastMtimeNs_ = 0;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
