#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "avionics/CommandBridgeProtocol.h"

namespace avionics {

// Receives G1000 bezel / softkey / radio events forwarded by the in-sim plugin
// and queues them for the render thread to drain each frame.
class CommandBridgeClient {
 public:
  CommandBridgeClient(std::string xplaneHost,
                      std::uint16_t listenPort = cmdbridge::kDefaultListenPort,
                      std::uint16_t registrationPort =
                          cmdbridge::kRegistrationPort);
  ~CommandBridgeClient();

  CommandBridgeClient(const CommandBridgeClient&) = delete;
  CommandBridgeClient& operator=(const CommandBridgeClient&) = delete;

  // True once at least one event has been received from the plugin.
  bool connected() const;

  // True once the plugin has acknowledged a registration datagram (the bridge
  // is linked even before the first bezel press).
  bool registered() const;

  // True when the UDP listen socket bound successfully. When false the bridge
  // thread exited immediately (usually the listen port is already in use).
  bool listening() const;

  // Move all queued events into `out` and clear the queue. Call from the main
  // thread once per frame before applying input to the engines.
  void drainEvents(std::vector<cmdbridge::Event>& out);

 private:
  void run();

  std::string xplaneHost_;
  std::uint16_t listenPort_;
  std::uint16_t registrationPort_;

  mutable std::mutex mutex_;
  std::deque<cmdbridge::Event> queue_;
  bool connected_ = false;
  bool registered_ = false;
  bool listening_ = false;
  bool loggedLink_ = false;

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace avionics
