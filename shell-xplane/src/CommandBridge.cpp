#include "CommandBridge.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <cstdio>
#include <cstring>
#include <vector>

#include "XPLMUtilities.h"

namespace avionics {
namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

// Routes diagnostics into X-Plane's Log.txt (grep "G1000 NXi"), unlike stderr
// which the sim doesn't capture on a windowed/.app launch.
void BridgeLog(const char* msg) { XPLMDebugString(msg); }

// Standalone re-registers every few seconds; treat the client as gone after this
// gap so we don't send to a stale address forever.
constexpr auto kClientTimeout = std::chrono::seconds(15);

// recvfrom() wakes at least this often so the server thread can observe stop_
// and exit promptly on plugin disable.
constexpr int kRecvTimeoutMs = 500;

void closeSocket(SocketHandle sock) {
#ifdef _WIN32
  closesocket(sock);
#else
  ::close(sock);
#endif
}

void setRecvTimeout(SocketHandle sock, int timeoutMs) {
#ifdef _WIN32
  DWORD tv = static_cast<DWORD>(timeoutMs);
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv),
             sizeof(tv));
#else
  timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
}

void setReuseAddr(SocketHandle sock) {
  int reuse = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse),
             sizeof(reuse));
}

}  // namespace

CommandBridge::CommandBridge(std::uint16_t registrationPort)
    : registrationPort_(registrationPort) {}

CommandBridge::~CommandBridge() { stop(); }

void CommandBridge::start() {
  if (started_) return;
  stop_.store(false);

#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

  // Bind on the enable thread (XPluginEnable) so diagnostics always reach
  // Log.txt and a short-lived worker thread cannot exit before we know the
  // port is open.
  listenSock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (listenSock_ == kInvalidSocket) {
    BridgeLog("G1000 NXi command bridge: failed to create UDP socket\n");
    return;
  }

  setReuseAddr(listenSock_);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(registrationPort_);
  if (::bind(listenSock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) !=
      0) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "G1000 NXi command bridge: failed to bind UDP port %u "
                  "(errno=%d); cockpit keys will not reach the standalone\n",
                  static_cast<unsigned>(registrationPort_), errno);
    BridgeLog(buf);
    closeSocket(listenSock_);
    listenSock_ = kInvalidSocket;
    return;
  }

  char logBuf[160];
  std::snprintf(logBuf, sizeof(logBuf),
                "G1000 NXi command bridge: listening for standalone "
                "registration on UDP %u\n",
                static_cast<unsigned>(registrationPort_));
  BridgeLog(logBuf);

  setRecvTimeout(listenSock_, kRecvTimeoutMs);
  thread_ = std::thread(&CommandBridge::recvLoop, this);
  started_ = true;
}

void CommandBridge::stop() {
  if (!started_) return;
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
  if (listenSock_ != kInvalidSocket) {
    closeSocket(listenSock_);
    listenSock_ = kInvalidSocket;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hasClient_ = false;
  }
  started_ = false;
#ifdef _WIN32
  WSACleanup();
#endif
}

bool CommandBridge::sendEvent(const cmdbridge::Event& ev) {
  sockaddr_storage addr{};
  socklen_t addrLen = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasClient_) return false;
    const auto age = std::chrono::steady_clock::now() - lastRegister_;
    if (age > kClientTimeout) {
      hasClient_ = false;
      return false;
    }
    addr = clientAddr_;
    addrLen = clientLen_;
  }

  // Send from the already-bound listen socket rather than creating and tearing
  // down a fresh UDP socket on every key event (this runs on X-Plane's main
  // thread). Concurrent sendto here and recvfrom on the worker thread is safe
  // for a UDP socket. listenSock_ is valid for the lifetime of started_, and a
  // client can only be registered while the recv loop owns that socket.
  if (listenSock_ == kInvalidSocket) return false;

  const std::vector<unsigned char> datagram = cmdbridge::encodeEvent(ev);
  const int sent = static_cast<int>(
      ::sendto(listenSock_, reinterpret_cast<const char*>(datagram.data()),
               static_cast<int>(datagram.size()), 0,
               reinterpret_cast<sockaddr*>(&addr), addrLen));
  return sent >= 0;
}

void CommandBridge::recvLoop() {
  const std::vector<unsigned char> ack = cmdbridge::encodeAck();
  unsigned char buf[64];
  bool loggedFirstClient = false;

  while (!stop_.load()) {
    sockaddr_storage src{};
#ifdef _WIN32
    int srcLen = sizeof(src);
    const int n = ::recvfrom(listenSock_, reinterpret_cast<char*>(buf),
                             sizeof(buf), 0,
                             reinterpret_cast<sockaddr*>(&src), &srcLen);
#else
    socklen_t srcLen = sizeof(src);
    const ssize_t n = ::recvfrom(listenSock_, buf, sizeof(buf), 0,
                                 reinterpret_cast<sockaddr*>(&src), &srcLen);
#endif
    if (n <= 0) continue;
    if (!cmdbridge::isRegister(buf, static_cast<std::size_t>(n))) continue;

    ::sendto(listenSock_, reinterpret_cast<const char*>(ack.data()),
             static_cast<int>(ack.size()), 0,
             reinterpret_cast<sockaddr*>(&src), srcLen);

    if (!loggedFirstClient) {
      loggedFirstClient = true;
      BridgeLog(
          "G1000 NXi command bridge: standalone registered; cockpit keys "
          "now forward to it\n");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    clientAddr_ = src;
    clientLen_ = srcLen;
    hasClient_ = true;
    lastRegister_ = std::chrono::steady_clock::now();
  }
}

}  // namespace avionics
