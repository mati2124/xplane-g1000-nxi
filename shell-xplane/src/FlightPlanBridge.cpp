#include "FlightPlanBridge.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#include <cmath>
#include <cstring>
#include <string>
#include <utility>

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

// A written entry carries no altitude constraint (the shell's route is lateral
// only); 0 ft tells the FMS "no constraint".
constexpr int kNoAltitudeConstraint = 0;

// When matching a route leg's identifier to a database navaid, accept the match
// only if it is within this many degrees of the leg's coordinates, so a far-off
// like-named navaid never hijacks the entry. ~0.1 deg latitude is ~6 NM.
constexpr float kNavMatchToleranceDeg = 0.1f;

// Navaid types a route leg may resolve to (airports, fixes, and the beacon
// types the FMS can hold).
constexpr XPLMNavType kRouteNavTypes = xplm_Nav_Airport | xplm_Nav_VOR |
                                       xplm_Nav_NDB | xplm_Nav_Fix;

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

// Resolve a route leg to a real database navaid so the FMS shows the proper
// identifier (and frequency/type) rather than a bare lat/lon point. Returns
// XPLM_NAV_NOT_FOUND when the id is empty or no like-named navaid sits near the
// leg's coordinates -- the caller then writes a lat/lon entry instead. Must run
// on the sim thread.
XPLMNavRef resolveNavRef(const MapLeg& leg) {
  if (leg.id.empty()) return XPLM_NAV_NOT_FOUND;
  float lat = static_cast<float>(leg.lat);
  float lon = static_cast<float>(leg.lon);
  const XPLMNavRef ref = XPLMFindNavAid(nullptr, leg.id.c_str(), &lat, &lon,
                                        nullptr, kRouteNavTypes);
  if (ref == XPLM_NAV_NOT_FOUND) return XPLM_NAV_NOT_FOUND;

  float foundLat = 0.0f;
  float foundLon = 0.0f;
  char foundId[kFmsIdBufferSize] = {};
  XPLMGetNavAidInfo(ref, nullptr, &foundLat, &foundLon, nullptr, nullptr,
                    nullptr, foundId, nullptr, nullptr);
  foundId[sizeof(foundId) - 1] = '\0';
  if (std::strcmp(foundId, leg.id.c_str()) != 0) return XPLM_NAV_NOT_FOUND;
  if (std::fabs(foundLat - static_cast<float>(leg.lat)) > kNavMatchToleranceDeg ||
      std::fabs(foundLon - static_cast<float>(leg.lon)) > kNavMatchToleranceDeg) {
    return XPLM_NAV_NOT_FOUND;
  }
  return ref;
}

// Write one route leg into FMS slot `index`, as a database navaid when it
// resolves (preserving the identifier) or a plain lat/lon entry otherwise.
void writeEntry(int index, const MapLeg& leg) {
  const XPLMNavRef ref = resolveNavRef(leg);
  if (ref != XPLM_NAV_NOT_FOUND) {
    XPLMSetFMSEntryInfo(index, ref, kNoAltitudeConstraint);
  } else {
    XPLMSetFMSEntryLatLon(index, static_cast<float>(leg.lat),
                          static_cast<float>(leg.lon), kNoAltitudeConstraint);
  }
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
  if (hasPlan) programRoute(plan);
  if (hasDto) programDirectTo(dtoActive, dtoTarget);
}

void FlightPlanBridge::programRoute(const std::vector<MapLeg>& legs) {
  const int oldCount = XPLMCountFMSEntries();
  const int newCount = static_cast<int>(legs.size());

  // Overwrite/extend entries 0..newCount-1 (setting an index at the current
  // count appends; the FMS requires contiguous entries).
  for (int i = 0; i < newCount; ++i) {
    writeEntry(i, legs[static_cast<std::size_t>(i)]);
  }
  // Drop any trailing entries from a previously longer plan, back to front so
  // the indices stay valid as the plan shortens.
  for (int i = oldCount - 1; i >= newCount; --i) {
    XPLMClearFMSEntry(i);
  }
  // Fly the first leg (track from entry 0 to entry 1) so the new plan is
  // active rather than leaving the destination on a stale index.
  if (newCount >= 2) XPLMSetDestinationFMSEntry(1);
}

void FlightPlanBridge::programDirectTo(bool active, const MapLeg& target) {
  if (!active) {
    // Cancel Direct-To: resume flying the active leg of the plan. With no plan
    // there is nothing to resume, so this is a no-op.
    const int count = XPLMCountFMSEntries();
    if (count >= 2) XPLMSetDestinationFMSEntry(1);
    return;
  }

  // Find the target among the existing entries; otherwise append it so we have
  // an index to fly directly to.
  int targetIndex = -1;
  const int count = XPLMCountFMSEntries();
  for (int i = 0; i < count; ++i) {
    XPLMNavType type = xplm_Nav_Unknown;
    char id[kFmsIdBufferSize] = {};
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int altitude = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    XPLMGetFMSEntryInfo(i, &type, id, &ref, &altitude, &lat, &lon);
    id[sizeof(id) - 1] = '\0';
    if (!target.id.empty() && std::strcmp(id, target.id.c_str()) == 0) {
      targetIndex = i;
      break;
    }
  }
  if (targetIndex < 0) {
    targetIndex = count;  // append
    writeEntry(targetIndex, target);
  }

#if defined(XPLM410)
  // A true present-position Direct-To (track from the aircraft straight to the
  // entry, ignoring the leg before it).
  XPLMSetDirectToFMSFlightPlanEntry(xplm_Fpl_Pilot_Primary, targetIndex);
#else
  // Older SDKs: fly the leg ending at the target (closest available behavior).
  XPLMSetDestinationFMSEntry(targetIndex);
#endif
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
