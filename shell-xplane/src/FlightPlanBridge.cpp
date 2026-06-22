#include "FlightPlanBridge.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#include <cstring>
#include <utility>

#include "FmsRouteProgrammer.h"
#include "XPLMNavigation.h"
#include "XPLMProcessing.h"
#include "avionics/FlightPlanBridgeProtocol.h"

#ifdef _WIN32
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

// How often the flight loop re-reads the FMS. The route changes rarely, so a
// 1 Hz refresh is plenty and keeps the sim-thread cost negligible.
constexpr float kFmsPollIntervalSeconds = 1.0f;

// recvfrom() wakes at least this often so the server thread can observe stop_
// and exit promptly on plugin disable.
constexpr int kRecvTimeoutMs = 500;

// FMS identifier buffer. The SDK documents lat/lon entries can be long, so use
// its recommended generous size and force null-termination.
constexpr int kFmsIdBufferSize = 256;

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

FlightPlanBridge::FlightPlanBridge(std::uint16_t port) : port_(port) {}

FlightPlanBridge::~FlightPlanBridge() { stop(); }

void FlightPlanBridge::start() {
  if (started_) return;
  started_ = true;
  stop_.store(false);

  readFmsOnSimThread();  // prime so the first request has data
  XPLMRegisterFlightLoopCallback(FlightLoopCb, kFmsPollIntervalSeconds, this);
  thread_ = std::thread([this] { serverLoop(); });
}

void FlightPlanBridge::stop() {
  if (!started_) return;
  XPLMUnregisterFlightLoopCallback(FlightLoopCb, this);
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
  started_ = false;
}

float FlightPlanBridge::FlightLoopCb(float, float, int, void* refcon) {
  auto* self = static_cast<FlightPlanBridge*>(refcon);
  self->applyPendingWritesOnSimThread();  // program first...
  self->readFmsOnSimThread();             // ...then snapshot the result
  return kFmsPollIntervalSeconds;         // reschedule at the same cadence
}

void FlightPlanBridge::applyPendingWritesOnSimThread() {
  bool hasPlan = false;
  std::vector<MapLeg> plan;
  bool hasDto = false;
  bool dtoActive = false;
  MapLeg dtoTarget;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasPlanWrite_) {
      hasPlan = true;
      plan = std::move(planWrite_);
      planWrite_.clear();
      hasPlanWrite_ = false;
    }
    if (hasDtoWrite_) {
      hasDto = true;
      dtoActive = dtoWriteActive_;
      dtoTarget = dtoWriteTarget_;
      hasDtoWrite_ = false;
    }
  }
  // Apply the route before the Direct-To so a combined edit lands coherently.
  if (hasPlan) programFmsRoute(plan);
  if (hasDto) programFmsDirectTo(dtoActive, dtoTarget);
}

void FlightPlanBridge::readFmsOnSimThread() {
  std::vector<MapLeg> plan;
  const int count = XPLMCountFMSEntries();
  if (count > 0) plan.reserve(static_cast<std::size_t>(count));

  for (int i = 0; i < count; ++i) {
    XPLMNavType type = xplm_Nav_Unknown;
    char id[kFmsIdBufferSize] = {};
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int altitude = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    XPLMGetFMSEntryInfo(i, &type, id, &ref, &altitude, &lat, &lon);
    id[sizeof(id) - 1] = '\0';
    // Skip empty / unpopulated entries (a 0/0 fix would draw a spurious leg).
    if (lat == 0.0f && lon == 0.0f) continue;
    plan.push_back({static_cast<double>(lat), static_cast<double>(lon),
                    std::string(id)});
  }

  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = std::move(plan);
}

void FlightPlanBridge::serverLoop() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock != kInvalidSocket) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port_);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      closeSocket(sock);
      sock = kInvalidSocket;
    }
  }

  if (sock != kInvalidSocket) {
    setRecvTimeout(sock, kRecvTimeoutMs);

    // Large enough for a full program-route datagram (FPLS), not just the tiny
    // read request.
    std::vector<unsigned char> buf(fpbridge::kMaxDatagramBytes);
    while (!stop_.load()) {
      sockaddr_in src{};
#ifdef _WIN32
      int srcLen = sizeof(src);
      const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf.data()),
                               static_cast<int>(buf.size()), 0,
                               reinterpret_cast<sockaddr*>(&src), &srcLen);
#else
      socklen_t srcLen = sizeof(src);
      const ssize_t n = ::recvfrom(sock, buf.data(), buf.size(), 0,
                                   reinterpret_cast<sockaddr*>(&src), &srcLen);
#endif
      if (n <= 0) continue;  // timeout / error -> re-check stop_
      const auto len = static_cast<std::size_t>(n);

      if (fpbridge::isRequest(buf.data(), len)) {
        // Read: reply with the latest route snapshot.
        std::vector<MapLeg> plan;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          plan = plan_;
        }
        const std::vector<unsigned char> reply = fpbridge::encodeReply(plan);
        ::sendto(sock, reinterpret_cast<const char*>(reply.data()),
                 static_cast<int>(reply.size()), 0,
                 reinterpret_cast<sockaddr*>(&src), srcLen);
      } else if (fpbridge::isSetPlan(buf.data(), len)) {
        // Program route: queue for the flight loop, then acknowledge. The XPLM
        // SDK must only be touched on the sim thread, so we never write here.
        std::vector<MapLeg> plan;
        if (fpbridge::decodeSetPlan(buf.data(), len, plan)) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            planWrite_ = std::move(plan);
            hasPlanWrite_ = true;
          }
          const std::vector<unsigned char> ack = fpbridge::encodeAck();
          ::sendto(sock, reinterpret_cast<const char*>(ack.data()),
                   static_cast<int>(ack.size()), 0,
                   reinterpret_cast<sockaddr*>(&src), srcLen);
        }
      } else if (fpbridge::isSetDirectTo(buf.data(), len)) {
        bool active = false;
        MapLeg target;
        if (fpbridge::decodeSetDirectTo(buf.data(), len, active, target)) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            dtoWriteActive_ = active;
            dtoWriteTarget_ = std::move(target);
            hasDtoWrite_ = true;
          }
          const std::vector<unsigned char> ack = fpbridge::encodeAck();
          ::sendto(sock, reinterpret_cast<const char*>(ack.data()),
                   static_cast<int>(ack.size()), 0,
                   reinterpret_cast<sockaddr*>(&src), srcLen);
        }
      }
    }
    closeSocket(sock);
  }

#ifdef _WIN32
  WSACleanup();
#endif
}

}  // namespace avionics
