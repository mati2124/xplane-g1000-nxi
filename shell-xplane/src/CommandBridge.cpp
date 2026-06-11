#include "CommandBridge.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using SocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using SocketHandle = int;
#endif

#include <cstring>
#include <vector>

namespace avionics {
namespace {

#ifdef _WIN32
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
constexpr SocketHandle kInvalidSocket = -1;
#endif

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

}  // namespace

CommandBridge::CommandBridge(std::uint16_t registrationPort)
    : registrationPort_(registrationPort) {}

CommandBridge::~CommandBridge() { stop(); }

void CommandBridge::start() {
  if (thread_.joinable()) return;
  stop_.store(false);
  thread_ = std::thread(&CommandBridge::serverLoop, this);
}

void CommandBridge::stop() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  hasClient_ = false;
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

  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == kInvalidSocket) return false;

  const std::vector<unsigned char> datagram = cmdbridge::encodeEvent(ev);
  const int sent = static_cast<int>(
      ::sendto(sock, reinterpret_cast<const char*>(datagram.data()),
               static_cast<int>(datagram.size()), 0,
               reinterpret_cast<sockaddr*>(&addr), addrLen));
  closeSocket(sock);
  return sent >= 0;
}

void CommandBridge::serverLoop() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock != kInvalidSocket) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(registrationPort_);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      closeSocket(sock);
      sock = kInvalidSocket;
    }
  }

  if (sock != kInvalidSocket) {
    setRecvTimeout(sock, kRecvTimeoutMs);
    unsigned char buf[64];
    while (!stop_.load()) {
      sockaddr_storage src{};
#ifdef _WIN32
      int srcLen = sizeof(src);
      const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                               reinterpret_cast<sockaddr*>(&src), &srcLen);
#else
      socklen_t srcLen = sizeof(src);
      const ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0,
                                   reinterpret_cast<sockaddr*>(&src), &srcLen);
#endif
      if (n <= 0) continue;
      if (!cmdbridge::isRegister(buf, static_cast<std::size_t>(n))) continue;

      std::lock_guard<std::mutex> lock(mutex_);
      clientAddr_ = src;
      clientLen_ = srcLen;
      hasClient_ = true;
      lastRegister_ = std::chrono::steady_clock::now();
    }
    closeSocket(sock);
  }

#ifdef _WIN32
  WSACleanup();
#endif
}

}  // namespace avionics
