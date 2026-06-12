#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#endif

#include "avionics/CommandBridgeProtocol.h"

namespace avionics {

// Receives registration datagrams from the standalone shell and forwards G1000
// bezel / softkey / radio events to the registered client over UDP.
//
// The plugin intercepts X-Plane commands on the sim thread; this bridge is
// thread-safe so handlers can push events without blocking the sim.
class CommandBridge {
 public:
  explicit CommandBridge(std::uint16_t registrationPort =
                             cmdbridge::kRegistrationPort);
  ~CommandBridge();

  CommandBridge(const CommandBridge&) = delete;
  CommandBridge& operator=(const CommandBridge&) = delete;

  void start();
  void stop();

  // Push one input event to the registered standalone client. Returns true when
  // a recent registration exists and the datagram was sent.
  bool sendEvent(const cmdbridge::Event& ev);

 private:
  void recvLoop();

#ifdef _WIN32
  using SocketHandle = SOCKET;
  static constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
  using SocketHandle = int;
  static constexpr SocketHandle kInvalidSocket = -1;
#endif

  std::uint16_t registrationPort_;

  mutable std::mutex mutex_;
  bool hasClient_ = false;
  sockaddr_storage clientAddr_{};
  socklen_t clientLen_ = 0;
  std::chrono::steady_clock::time_point lastRegister_{};

  bool started_ = false;
  SocketHandle listenSock_ = kInvalidSocket;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace avionics
