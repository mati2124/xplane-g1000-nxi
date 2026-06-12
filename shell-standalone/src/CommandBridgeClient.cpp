#include "CommandBridgeClient.h"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

#ifdef _WIN32
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

namespace avionics {
namespace {

#ifdef _WIN32
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
constexpr SocketHandle kInvalidSocket = -1;
#endif

constexpr int kRegisterIntervalMs = 3000;
constexpr int kRegisterSliceMs = 100;
constexpr int kRecvTimeoutMs = 200;

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

bool fillDest(const std::string& host, std::uint16_t port, sockaddr_in& dest) {
  dest = sockaddr_in{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);
#ifdef _WIN32
  return inet_pton(AF_INET, host.c_str(), &dest.sin_addr) == 1;
#else
  dest.sin_addr.s_addr = inet_addr(host.c_str());
  return dest.sin_addr.s_addr != INADDR_NONE;
#endif
}

}  // namespace

CommandBridgeClient::CommandBridgeClient(std::string xplaneHost,
                                         std::uint16_t listenPort,
                                         std::uint16_t registrationPort)
    : xplaneHost_(std::move(xplaneHost)),
      listenPort_(listenPort),
      registrationPort_(registrationPort) {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  thread_ = std::thread(&CommandBridgeClient::run, this);
}

CommandBridgeClient::~CommandBridgeClient() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
#ifdef _WIN32
  WSACleanup();
#endif
}

bool CommandBridgeClient::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

bool CommandBridgeClient::registered() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return registered_;
}

bool CommandBridgeClient::listening() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return listening_;
}

void CommandBridgeClient::drainEvents(std::vector<cmdbridge::Event>& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  out.insert(out.end(), queue_.begin(), queue_.end());
  queue_.clear();
}

void CommandBridgeClient::run() {
  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == kInvalidSocket) {
    std::fprintf(stderr,
                 "Command bridge: failed to create UDP socket (errno=%d)\n",
                 errno);
    return;
  }

  setReuseAddr(sock);

  sockaddr_in listenAddr{};
  listenAddr.sin_family = AF_INET;
  listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  listenAddr.sin_port = htons(listenPort_);
  if (::bind(sock, reinterpret_cast<sockaddr*>(&listenAddr),
             sizeof(listenAddr)) != 0) {
    std::fprintf(stderr,
                 "Command bridge: failed to bind UDP port %u (errno=%d). "
                 "Cockpit keys cannot reach the standalone; try "
                 "--command-bridge-port.\n",
                 static_cast<unsigned>(listenPort_), errno);
    closeSocket(sock);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    listening_ = true;
  }

  setRecvTimeout(sock, kRecvTimeoutMs);

  const std::vector<unsigned char> reg = cmdbridge::encodeRegister();
  sockaddr_in pluginAddr{};
  const bool hasPluginAddr =
      fillDest(xplaneHost_, registrationPort_, pluginAddr);
  if (!hasPluginAddr) {
    std::fprintf(stderr,
                 "Command bridge: invalid X-Plane host %s (cannot register).\n",
                 xplaneHost_.c_str());
  } else {
    std::fprintf(stderr,
                 "Command bridge: listening on UDP %u, registering with %s:%u\n",
                 static_cast<unsigned>(listenPort_), xplaneHost_.c_str(),
                 static_cast<unsigned>(registrationPort_));
  }

  int sinceRegisterMs = kRegisterIntervalMs;  // register immediately
  unsigned char buf[cmdbridge::kEventBytes];

  while (!stop_.load()) {
    if (hasPluginAddr && sinceRegisterMs >= kRegisterIntervalMs) {
      ::sendto(sock, reinterpret_cast<const char*>(reg.data()),
               static_cast<int>(reg.size()), 0,
               reinterpret_cast<sockaddr*>(&pluginAddr), sizeof(pluginAddr));
      sinceRegisterMs = 0;
    }

#ifdef _WIN32
    const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                             nullptr, nullptr);
#else
    const ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
#endif
    if (n > 0) {
      if (cmdbridge::isAck(buf, static_cast<std::size_t>(n))) {
        std::lock_guard<std::mutex> lock(mutex_);
        registered_ = true;
        if (!loggedLink_) {
          loggedLink_ = true;
          std::fprintf(stderr,
                       "Command bridge: linked to X-Plane plugin "
                       "(cockpit keys will forward to the standalone)\n");
        }
      } else {
        cmdbridge::Event ev;
        if (cmdbridge::decodeEvent(buf, static_cast<std::size_t>(n), ev)) {
          std::lock_guard<std::mutex> lock(mutex_);
          queue_.push_back(ev);
          connected_ = true;
          registered_ = true;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kRegisterSliceMs));
    sinceRegisterMs += kRegisterSliceMs;
  }

  closeSocket(sock);
}

}  // namespace avionics
