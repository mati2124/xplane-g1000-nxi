#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "SimBriefClient.h"

namespace avionics {

// Runs SimBrief OFP fetches on a background thread so a slow or absent
// network never stalls the render loop, following the XPlaneWebApi /
// FmsPlanStore pattern: the render thread requests a fetch, polls fetching(),
// and consumes the published result when it lands.
class SimBriefStore {
 public:
  SimBriefStore() = default;
  ~SimBriefStore();

  SimBriefStore(const SimBriefStore&) = delete;
  SimBriefStore& operator=(const SimBriefStore&) = delete;

  // Starts a fetch for the given Pilot ID. Ignored while one is in flight.
  void requestFetch(const std::string& pilotId);

  bool fetching() const { return fetching_.load(std::memory_order_acquire); }

  // True once per completed fetch: copies the result out and clears the
  // ready latch.
  bool consumeResult(SimBriefFetchResult& out);

 private:
  std::thread thread_;
  std::atomic<bool> fetching_{false};
  std::atomic<bool> resultReady_{false};
  mutable std::mutex mutex_;
  SimBriefFetchResult result_;
};

}  // namespace avionics
