#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace avionics {

// Minimal client for X-Plane's local Web API (REST), used only to read the
// string datarefs that the float-only RREF UDP protocol cannot carry: the
// active GPS destination identifier (navigation status box), the decoded
// NAV1/2 station idents shown beside the active frequencies on the PFD, and
// the loaded aircraft's ICAO type code + .acf path (to swap EIS/checklists).
//
// Why this exists alongside the UDP link: X-Plane streams telemetry to the
// standalone shell over UDP RREF, but an RREF record is a fixed (int index,
// float value) pair, so byte-array string datarefs (e.g. gps_nav_id) are
// unreachable that way. The Web API (X-Plane 12.1.1+, default
// http://<host>:8086) returns dataref values of any type, with byte strings
// base64-encoded.
//
// The API is polled on a background thread so a slow, absent, or older
// (pre-12.1.1) web server never stalls the render loop; the latest value is
// published behind a mutex for the render thread to read. When the server is
// unreachable the published identifier is simply empty.
class XPlaneWebApi {
 public:
  static constexpr std::uint16_t kDefaultPort = 8086;

  explicit XPlaneWebApi(std::string host = "127.0.0.1",
                        std::uint16_t port = kDefaultPort);
  ~XPlaneWebApi();

  XPlaneWebApi(const XPlaneWebApi&) = delete;
  XPlaneWebApi& operator=(const XPlaneWebApi&) = delete;

  // Latest active GPS destination identifier (e.g. "KPAO"), or empty if the
  // Web API is unreachable or no destination is currently selected.
  std::string destinationId() const;

  // Decoded Morse idents of the stations being received on NAV1/2 (e.g. "SAU"),
  // or empty when out of range / Web API unavailable.
  std::string nav1Ident() const;
  std::string nav2Ident() const;

  // Loaded-aircraft identity from sim/aircraft/view/acf_ICAO and
  // acf_relative_path, or empty when the Web API is unreachable.
  std::string aircraftIcao() const;
  std::string aircraftAcfRelativePath() const;

 private:
  void run();  // background polling loop

  std::string host_;
  std::uint16_t port_;

  mutable std::mutex mutex_;
  std::string destinationId_;
  std::string nav1Ident_;
  std::string nav2Ident_;
  std::string aircraftIcao_;
  std::string aircraftAcfRelativePath_;

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace avionics
