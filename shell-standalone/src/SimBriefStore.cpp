#include "SimBriefStore.h"

#include <utility>

namespace avionics {

SimBriefStore::~SimBriefStore() {
  if (thread_.joinable()) thread_.join();
}

void SimBriefStore::requestFetch(const std::string& pilotId) {
  if (fetching_.load(std::memory_order_acquire)) return;
  if (thread_.joinable()) thread_.join();  // reap the previous fetch's thread

  fetching_.store(true, std::memory_order_release);
  thread_ = std::thread([this, pilotId]() {
    SimBriefFetchResult result = FetchSimBriefOfp(pilotId);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_ = std::move(result);
    }
    resultReady_.store(true, std::memory_order_release);
    fetching_.store(false, std::memory_order_release);
  });
}

bool SimBriefStore::consumeResult(SimBriefFetchResult& out) {
  if (!resultReady_.exchange(false, std::memory_order_acq_rel)) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  out = result_;
  return true;
}

}  // namespace avionics
