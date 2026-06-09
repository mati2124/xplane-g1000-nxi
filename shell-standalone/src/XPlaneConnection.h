#pragma once

#include <cstdint>
#include <string>

#include "XPlaneWebApi.h"
#include "avionics/SimulatorConnection.h"

namespace avionics {

// Live link to X-Plane over its UDP RREF protocol.
//
// Why UDP RREF instead of the 12.1+ Web API (WebSocket/JSON): RREF needs no
// third-party WebSocket/JSON dependency (keeping the project dependency-light),
// works on X-Plane 11 and 12, and makes connection-loss detection trivial -- if
// no packets arrive within the stale timeout the link is considered down and
// the display shows the red X.
//
// On construction we open a non-blocking UDP socket and send RREF subscription
// requests for the datarefs we care about. X-Plane then streams back
// (index, float value) pairs which we decode into FlightData. All socket I/O is
// drained on the render thread inside update(), so no extra thread is needed.
//
// This is a SimulatorConnection so a future Microsoft Flight Simulator backend
// (SimConnect) can drop in behind the same interface.
class XPlaneConnection : public SimulatorConnection {
 public:
  explicit XPlaneConnection(std::string host = "127.0.0.1",
                            std::uint16_t port = 49000);
  ~XPlaneConnection() override;

  XPlaneConnection(const XPlaneConnection&) = delete;
  XPlaneConnection& operator=(const XPlaneConnection&) = delete;

  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  ConnectionState connectionState() const override;
  const char* simulatorName() const override { return "X-PLANE"; }

 private:
  void sendSubscriptions(int frequencyHz);
  void drainSocket();

  // Advance the locally-ticked zulu (UTC) clock and write it into data_'s
  // hour/minute/second fields. Decouples the displayed clock from packet
  // cadence so the seconds never skip on UDP jitter/loss.
  void updateZuluClock(double dtSeconds);

  // Turn the latest raw autopilot mode statuses into the FMA strings/value
  // (lateral + vertical active/armed modes and the cyan altitude reference)
  // shown on the top bar.
  void updateFmaModes();

  // data_ is the smoothed state returned by snapshot(); target_ holds the most
  // recent values decoded from packets, which data_ is eased toward each frame.
  FlightData data_;
  FlightData target_;
  bool primed_ = false;  // false until the first data snaps data_ to target_
  float prevAirspeedKts_ = 0.0f;  // for deriving the airspeed trend vector

  // Continuous zulu (UTC) seconds-since-midnight. zuluTargetSec_ is the latest
  // value from X-Plane; zuluDisplaySec_ free-runs locally and is eased toward it
  // so the rendered clock ticks smoothly between (and through lost) packets.
  double zuluTargetSec_ = 0.0;
  double zuluDisplaySec_ = 0.0;
  bool zuluHasTarget_ = false;  // false until the first zulu packet arrives
  bool zuluPrimed_ = false;     // false until display snaps to the first value

  // Latest raw autopilot mode-status ints (0=off, 1=armed, 2=active), indexed
  // by the ApMode enum in the .cpp. Accumulated across packets and decoded into
  // the FMA fields each frame by updateFmaModes(). The count is mirrored by a
  // static_assert against the subscription table.
  static constexpr int kApModeStatusCount = 11;
  int apModeStatus_[kApModeStatusCount] = {};

  // Latest GPS lateral CDI sensitivity (NM per dot). The annunciated GPS flight
  // phase (ENR/TERM/APR/OCN) is derived from it each frame; 0 means no usable
  // GPS scale (no active flight plan), which blanks the phase annunciation.
  float gpsHdefNmPerDot_ = 0.0f;
  std::string host_;
  std::uint16_t port_;

  // String datarefs (the GPS destination identifier) can't ride the float-only
  // RREF stream, so they come from X-Plane's Web API on a background thread.
  XPlaneWebApi webApi_;

  // Native socket handle stored width-safe: -1 is "invalid" on both POSIX (int
  // fd) and Windows (SOCKET, where INVALID_SOCKET is all-ones == -1).
  std::intptr_t socketHandle_ = -1;

  double elapsedSeconds_ = 0.0;
  double lastPacketSeconds_ = -1.0;       // < 0 until the first packet arrives
  double sinceResubscribeSeconds_ = 0.0;
  bool everConnected_ = false;
};

}  // namespace avionics
