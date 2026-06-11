#include "FlightPlanBridgeClient.h"

#include <chrono>
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

// Request cadence and how long to wait for the bridge's reply. The route
// changes rarely, so a 1 s poll is responsive enough while staying cheap.
constexpr int kPollIntervalMs = 1000;
constexpr int kPollSliceMs = 100;  // shutdown responsiveness within a poll wait
constexpr int kReplyTimeoutMs = 400;

// A write (program route / Direct-To) is retransmitted until acknowledged, to
// ride out UDP loss, then abandoned so a missing bridge doesn't retry forever.
constexpr int kWriteAckTimeoutMs = 300;
constexpr int kWriteMaxAttempts = 4;

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

// Fill `dest` with the bridge endpoint. Returns false on an unparseable host.
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

FlightPlanBridgeClient::FlightPlanBridgeClient(std::string host,
                                               std::uint16_t port)
    : host_(std::move(host)), port_(port) {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  thread_ = std::thread(&FlightPlanBridgeClient::run, this);
}

FlightPlanBridgeClient::~FlightPlanBridgeClient() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
#ifdef _WIN32
  WSACleanup();
#endif
}

std::vector<MapLeg> FlightPlanBridgeClient::flightPlan(bool& available) const {
  std::lock_guard<std::mutex> lock(mutex_);
  available = available_;
  return plan_;
}

void FlightPlanBridgeClient::writePlan(std::vector<MapLeg> legs) {
  std::lock_guard<std::mutex> lock(mutex_);
  planCmd_ = std::move(legs);
  hasPlanCmd_ = true;
}

void FlightPlanBridgeClient::writeDirectTo(MapLeg target) {
  std::lock_guard<std::mutex> lock(mutex_);
  dtoCmdActive_ = true;
  dtoCmdTarget_ = std::move(target);
  hasDtoCmd_ = true;
}

void FlightPlanBridgeClient::clearDirectTo() {
  std::lock_guard<std::mutex> lock(mutex_);
  dtoCmdActive_ = false;
  dtoCmdTarget_ = MapLeg{};
  hasDtoCmd_ = true;
}

bool FlightPlanBridgeClient::pendingCommand() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return hasPlanCmd_ || hasDtoCmd_;
}

void FlightPlanBridgeClient::run() {
  while (!stop_.load()) {
    flushCommands();  // push shell-side edits into the FMS first
    if (!pollOnce()) {
      std::lock_guard<std::mutex> lock(mutex_);
      available_ = false;
    }
    // Wait out the poll interval, but wake early when a new edit arrives so
    // write-back feels immediate.
    for (int waited = 0; waited < kPollIntervalMs && !stop_.load();
         waited += kPollSliceMs) {
      if (pendingCommand()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollSliceMs));
    }
  }
}

void FlightPlanBridgeClient::flushCommands() {
  // Snapshot and clear the pending writes; re-queue only on send failure so a
  // newer edit (set meanwhile) is never clobbered by a stale retry.
  bool hasPlan = false;
  std::vector<MapLeg> plan;
  bool hasDto = false;
  bool dtoActive = false;
  MapLeg dtoTarget;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasPlanCmd_) {
      hasPlan = true;
      plan = planCmd_;
      hasPlanCmd_ = false;
    }
    if (hasDtoCmd_) {
      hasDto = true;
      dtoActive = dtoCmdActive_;
      dtoTarget = dtoCmdTarget_;
      hasDtoCmd_ = false;
    }
  }

  if (hasPlan && !sendWithAck(fpbridge::encodeSetPlan(plan))) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasPlanCmd_) {  // no newer plan queued since the snapshot
      planCmd_ = std::move(plan);
      hasPlanCmd_ = true;
    }
  }
  if (hasDto &&
      !sendWithAck(fpbridge::encodeSetDirectTo(dtoActive, dtoTarget))) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasDtoCmd_) {
      dtoCmdActive_ = dtoActive;
      dtoCmdTarget_ = std::move(dtoTarget);
      hasDtoCmd_ = true;
    }
  }
}

bool FlightPlanBridgeClient::sendWithAck(
    const std::vector<unsigned char>& datagram) {
  sockaddr_in dest{};
  if (!fillDest(host_, port_, dest)) return false;

  for (int attempt = 0; attempt < kWriteMaxAttempts && !stop_.load();
       ++attempt) {
    SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == kInvalidSocket) return false;
    setRecvTimeout(sock, kWriteAckTimeoutMs);

    const int sent = static_cast<int>(
        ::sendto(sock, reinterpret_cast<const char*>(datagram.data()),
                 static_cast<int>(datagram.size()), 0,
                 reinterpret_cast<sockaddr*>(&dest), sizeof(dest)));
    if (sent < 0) {
      closeSocket(sock);
      continue;
    }

    unsigned char ack[16];
#ifdef _WIN32
    const int n = ::recvfrom(sock, reinterpret_cast<char*>(ack), sizeof(ack), 0,
                             nullptr, nullptr);
#else
    const ssize_t n = ::recvfrom(sock, ack, sizeof(ack), 0, nullptr, nullptr);
#endif
    closeSocket(sock);
    if (n > 0 && fpbridge::isAck(ack, static_cast<std::size_t>(n))) return true;
  }
  return false;
}

bool FlightPlanBridgeClient::pollOnce() {
  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == kInvalidSocket) return false;
  setRecvTimeout(sock, kReplyTimeoutMs);

  sockaddr_in dest{};
  if (!fillDest(host_, port_, dest)) {
    closeSocket(sock);
    return false;
  }

  const std::vector<unsigned char> req = fpbridge::encodeRequest();
  const int sent =
      static_cast<int>(::sendto(sock, reinterpret_cast<const char*>(req.data()),
                                static_cast<int>(req.size()), 0,
                                reinterpret_cast<sockaddr*>(&dest),
                                sizeof(dest)));
  if (sent < 0) {
    closeSocket(sock);
    return false;
  }

  std::vector<unsigned char> buf(fpbridge::kMaxDatagramBytes);
#ifdef _WIN32
  const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf.data()),
                           static_cast<int>(buf.size()), 0, nullptr, nullptr);
#else
  const ssize_t n =
      ::recvfrom(sock, buf.data(), buf.size(), 0, nullptr, nullptr);
#endif
  closeSocket(sock);
  if (n <= 0) return false;  // timeout / no bridge listening

  std::vector<MapLeg> plan;
  if (!fpbridge::decodeReply(buf.data(), static_cast<std::size_t>(n), plan)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = std::move(plan);
  available_ = true;
  return true;
}

}  // namespace avionics
