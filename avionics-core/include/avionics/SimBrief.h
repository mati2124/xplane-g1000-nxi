#pragma once

#include <string>

namespace avionics {

inline constexpr int kSimBriefPilotIdMaxDigits = 7;

// Lifecycle of the Navigraph sign-in (OAuth device-authorization flow) that
// backs the SimBrief OFP import. The shell owns the network client and the
// token store; this state is published into the MfdController for rendering on
// the AUX - SIMBRIEF page.
enum class NavigraphLoginPhase {
  LoggedOut,     // no session; the pilot must sign in
  AwaitingUser,  // device code issued; the page shows userCode + verificationUri
  LoggedIn,      // signed in; the Navigraph alias (username) is known
  Error,         // sign-in failed (see SimBriefState::loginError)
};

// Lifecycle of the SimBrief OFP fetch itself, driven by the shell and rendered
// by the AUX - SIMBRIEF page.
enum class SimBriefStatus {
  NotConfigured,  // not signed in to Navigraph yet
  Idle,           // signed in, nothing fetched this session
  Fetching,       // request in flight
  Ok,             // latest OFP fetched; shell decides active route vs catalog
  Error,          // fetch or parse failed (see SimBriefState::error)
};

// Snapshot of the SimBrief integration shown on the AUX - SIMBRIEF page. The
// shell publishes it into the MfdController each frame; the page draws it.
struct SimBriefState {
  // Whether Navigraph communication is currently permitted. Navigraph's terms
  // only allow data access from a connected simulator session: the in-sim
  // plugin is always permitted, while the standalone shell is permitted only
  // while it has a live link to X-Plane. When false the AUX - SIMBRIEF page
  // disables Login/FETCH and prompts the pilot to connect the sim.
  bool commAllowed = true;

  // ---- Navigraph account / sign-in ----
  NavigraphLoginPhase loginPhase = NavigraphLoginPhase::LoggedOut;
  std::string username;         // Navigraph Alias, shown when signed in
  std::string userCode;         // device-flow code shown during sign-in
  std::string verificationUri;  // URL the pilot visits to authorize
  // Verification URL with the code pre-filled; encoded into the sign-in QR code
  // so scanning it jumps straight to the authorize page (no manual code entry).
  std::string verificationUriComplete;
  std::string loginError;       // human-readable summary when loginPhase==Error

  // ---- OFP fetch ----
  SimBriefStatus status = SimBriefStatus::NotConfigured;
  // Human-readable failure summary when status == Error.
  std::string error;

  // Summary of the most recently fetched OFP (valid when status == Ok).
  std::string originIcao;
  std::string destinationIcao;
  std::string route;         // filed route string (fixes/airways)
  std::string generatedUtc;  // OFP generation time, e.g. "10JUN 19:42Z"
  int waypointCount = 0;     // legs parsed from the fetched OFP
};

}  // namespace avionics
