#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "avionics/FlightPlanBridgeProtocol.h"
#include "avionics/MapData.h"

namespace avionics {

// Reads and programs the live X-Plane FMS route through the in-sim flight-plan
// bridge.
//
// The standalone shell links to X-Plane over UDP RREF (float telemetry) and the
// Web API (string datarefs), neither of which can carry the FMS flight plan --
// that is only reachable through the in-sim plugin SDK. The shell-xplane plugin
// exposes it over a small UDP channel (see avionics/FlightPlanBridgeProtocol.h);
// this client runs a background thread (mirroring XPlaneWebApi) that polls the
// route and pushes shell-side edits back into the FMS, so a missing or
// unreachable bridge never stalls the render loop.
//
// When the bridge is unreachable (the plugin is not installed, or X-Plane is on
// another machine without the plugin) reads report `available` false so callers
// fall back to the .fms-file route, and writes are dropped after a few retries.
class FlightPlanBridgeClient {
 public:
  explicit FlightPlanBridgeClient(std::string host,
                                  std::uint16_t port = fpbridge::kDefaultPort);
  ~FlightPlanBridgeClient();

  FlightPlanBridgeClient(const FlightPlanBridgeClient&) = delete;
  FlightPlanBridgeClient& operator=(const FlightPlanBridgeClient&) = delete;

  // Latest live FMS route. `available` is set true only when the bridge has
  // answered at least one recent request; when false the returned vector is
  // meaningless and the caller should use its .fms fallback.
  std::vector<MapLeg> flightPlan(bool& available) const;

  // Program a route into the X-Plane FMS (the latest call wins). The send is
  // best-effort with ack/retry on the background thread; passing an empty
  // vector clears the FMS plan.
  void writePlan(std::vector<MapLeg> legs);

  // Activate / cancel a present-position Direct-To in the FMS.
  void writeDirectTo(MapLeg target);
  void clearDirectTo();

 private:
  void run();              // background read/write loop
  bool pollOnce();         // one read request/response round trip
  void flushCommands();    // send any pending writes (ack/retry)
  bool sendWithAck(const std::vector<unsigned char>& datagram);
  bool pendingCommand() const;

  std::string host_;
  std::uint16_t port_;

  mutable std::mutex mutex_;
  std::vector<MapLeg> plan_;
  bool available_ = false;

  // Pending writes from the render thread, drained by the background thread.
  bool hasPlanCmd_ = false;
  std::vector<MapLeg> planCmd_;
  bool hasDtoCmd_ = false;
  bool dtoCmdActive_ = false;
  MapLeg dtoCmdTarget_;

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace avionics
