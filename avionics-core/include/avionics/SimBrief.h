#pragma once

#include <string>

namespace avionics {

// SimBrief is configured with the account's numeric Pilot ID (up to 7 digits).
// SimBrief recommends the Pilot ID over the username for avionics-style
// limited keyboards, which fits the bezel softkey digit entry used here.
inline constexpr int kSimBriefPilotIdMaxDigits = 7;

// Lifecycle of the SimBrief OFP fetch, driven by the shell (which owns the
// network client) and rendered by the AUX - SIMBRIEF page.
enum class SimBriefStatus {
  NotConfigured,  // no Pilot ID entered yet
  Idle,           // Pilot ID set, nothing fetched this session
  Fetching,       // request in flight
  Ok,             // latest OFP fetched and loaded as the flight plan
  Error,          // fetch or parse failed (see SimBriefState::error)
};

// Snapshot of the SimBrief integration shown on the AUX - SIMBRIEF page. The
// shell publishes it into the MfdController each frame; the page draws it.
struct SimBriefState {
  SimBriefStatus status = SimBriefStatus::NotConfigured;
  // Human-readable failure summary when status == Error.
  std::string error;

  // Summary of the most recently fetched OFP (valid when status == Ok).
  std::string originIcao;
  std::string destinationIcao;
  std::string route;         // filed route string (fixes/airways)
  std::string generatedUtc;  // OFP generation time, e.g. "10JUN 19:42Z"
  int waypointCount = 0;     // legs loaded into the flight plan
};

}  // namespace avionics
