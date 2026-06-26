#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "avionics/FlightPlanBridgeProtocol.h"
#include "avionics/MapData.h"

namespace avionics {

// Reads and programs the X-Plane FMS flight plan for the networked standalone
// shell.
//
// X-Plane exposes the FMS route only through the plugin SDK (XPLMGetFMSEntryInfo
// / XPLMSetFMSEntry*) -- not over the UDP RREF telemetry stream and not over the
// Web API -- so the standalone shell can neither see nor program an in-cockpit
// flight plan on its own. This bridge closes that gap from inside the sim:
//
//   * A flight-loop callback runs on the sim thread (the only thread allowed to
//     call the XPLM navigation APIs). Each tick it applies any pending write
//     (a route or Direct-To pushed from the shell) and snapshots the live route.
//   * A background UDP server answers the shell's read requests with the latest
//     snapshot and accepts route / Direct-To writes, all serialized per
//     avionics/FlightPlanBridgeProtocol.h. Network I/O never touches the SDK;
//     writes are queued for the flight loop to apply.
//
// The server is request/response and stateless, so it works whether the shell
// runs on the same machine or another host on the LAN.
class FlightPlanBridge {
 public:
  explicit FlightPlanBridge(std::uint16_t port);
  ~FlightPlanBridge();

  FlightPlanBridge(const FlightPlanBridge&) = delete;
  FlightPlanBridge& operator=(const FlightPlanBridge&) = delete;

  // Registers the flight-loop callback and starts the UDP server thread. Call
  // from XPluginEnable (sim thread).
  void start();

  // Unregisters the callback and stops the server thread. Call from
  // XPluginDisable (sim thread). Safe to call when not started.
  void stop();

 private:
  // XPLM flight-loop trampoline; runs on the sim thread.
  static float FlightLoopCb(float, float, int, void* refcon);

  // Sim-thread work, all called from the flight loop:
  void applyPendingWritesOnSimThread();  // program queued route / Direct-To
  void readFmsOnSimThread();             // snapshot the live route for serving

  // UDP server loop (background thread): answers read requests and queues write
  // commands for the flight loop. Never touches the XPLM SDK.
  void serverLoop();

  std::uint16_t port_;
  bool started_ = false;

  // Latest FMS route snapshot. The sim thread writes it; the server thread
  // reads it. Guarded by mutex_.
  mutable std::mutex mutex_;
  std::vector<MapLeg> plan_;

  // Pending writes received from the shell, applied by the next flight loop on
  // the sim thread. Guarded by mutex_.
  bool hasPlanWrite_ = false;
  std::vector<MapLeg> planWrite_;
  bool hasDtoWrite_ = false;
  bool dtoWriteActive_ = false;
  MapLeg dtoWriteTarget_;
  bool dtoWriteOriginValid_ = false;
  double dtoWriteOriginLat_ = 0.0;
  double dtoWriteOriginLon_ = 0.0;
  bool dtoWriteProgramFms_ = true;
  bool hasDtoDisplayClearWrite_ = false;
  bool hasActiveLegWrite_ = false;
  int activeLegWriteIndex_ = -1;

  // Stored Direct-To display state (target + course origin at activation).
  // Survives standalone restarts while the plugin stays loaded; reported in
  // FPLR replies so the map magenta course can be restored.
  fpbridge::DirectToState dtoDisplay_;

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace avionics
