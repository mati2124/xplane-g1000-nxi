#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "NavigraphChartsClient.h"
#include "NavigraphClient.h"
#include "SimBriefClient.h"

namespace avionics {

// Where the Navigraph sign-in currently stands. Drives both the AUX - SIMBRIEF
// page rendering and what the FETCH softkey is allowed to do.
enum class NavigraphAuthPhase {
  LoggedOut,     // no session; the pilot must sign in
  AwaitingUser,  // device code issued; show userCode + verificationUri
  LoggedIn,      // tokens valid; username known
  Error,         // sign-in failed (see error)
};

// Thread-safe snapshot the render loop reads each frame.
struct NavigraphAuthSnapshot {
  NavigraphAuthPhase phase = NavigraphAuthPhase::LoggedOut;
  std::string userCode;                 // shown to the pilot during sign-in
  std::string verificationUri;          // URL the pilot visits
  std::string verificationUriComplete;  // URL with the code pre-filled
  std::string username;                 // Navigraph Alias when logged in
  std::string error;                    // populated when phase == Error
};

// Lifecycle of the chart index fetch for the currently selected airport.
enum class ChartIndexStatus { Idle, Loading, Ready, Empty, Error };

// Thread-safe chart index snapshot the render loop reads each frame.
struct ChartIndexSnapshot {
  ChartIndexStatus status = ChartIndexStatus::Idle;
  std::string icao;
  std::string error;
  std::vector<NavigraphChartMeta> charts;
};

// Runs the Navigraph device-authorization flow and SimBrief OFP fetches on a
// single background worker thread so a slow network or the multi-second sign-in
// poll never stalls the 60 fps render loop. The render thread posts user
// actions (sign in / out, fetch) and polls the published snapshot + results.
//
// Mirrors the SimBriefStore lifecycle (requestFetch / fetching / consumeResult)
// but adds the longer-lived auth state machine.
class NavigraphStore {
 public:
  NavigraphStore();
  ~NavigraphStore();

  NavigraphStore(const NavigraphStore&) = delete;
  NavigraphStore& operator=(const NavigraphStore&) = delete;

  // Credentials must be set before any request; an invalid set leaves the
  // store permanently LoggedOut with a credentials error on sign-in attempts.
  void setCredentials(const NavigraphCredentials& creds) { creds_ = creds; }
  bool hasCredentials() const { return creds_.valid(); }

  // Gate all Navigraph network traffic on a live simulator session, as required
  // by Navigraph's terms. The in-sim plugin leaves this true (it always runs
  // inside the sim); the standalone shell sets it from the X-Plane link state
  // each frame. While false, sign-in / refresh / fetch are refused, and an
  // in-progress device-authorization poll loop aborts so no polling continues
  // off-sim. Defaults to true so callers that never gate stay unrestricted.
  void setCommunicationAllowed(bool allowed);

  // Silently restore a prior session from a persisted refresh token (no user
  // interaction). On success the snapshot becomes LoggedIn and an OFP fetch is
  // kicked off automatically.
  void restoreSession(const std::string& refreshToken);

  // Begin an interactive device-authorization sign-in.
  void requestLogin();
  // Forget the session (clears the persisted refresh token via consumeToken).
  void requestLogout();
  // Fetch the latest OFP for the signed-in account. Ignored when logged out.
  void requestFetch();

  NavigraphAuthSnapshot snapshot() const;
  bool fetching() const { return fetching_.load(std::memory_order_acquire); }

  // ---- Navigraph charts (AUX - Charts page) ----
  // Select the airport whose chart index to fetch; a change kicks off a new
  // index download (empty clears the index). Deduplicated, so it is safe to
  // call every frame. Charts require a signed-in session + live sim link.
  void setChartAirport(const std::string& icao);
  // Select which chart image to download (its id, day/night variant, and the
  // absolute image URL from the index). A change kicks off a new download;
  // deduplicated, safe to call every frame.
  void setChartSelection(const std::string& chartId, bool night,
                         const std::string& imageUrl);
  ChartIndexSnapshot chartIndexSnapshot() const;
  // True once per completed chart image download: copies the bytes out.
  bool consumeChartImage(ChartImageResult& out);

  // True once per completed OFP fetch: copies the result out.
  bool consumeResult(SimBriefFetchResult& out);
  // True once whenever the persisted refresh token should change (new token
  // after sign-in/refresh, or empty after sign-out). Caller persists `out`.
  bool consumeRefreshToken(std::string& out);

 private:
  enum class CommandType { Login, Logout, Fetch, Restore, ChartIndex,
                           ChartImage };
  struct Command {
    CommandType type;
    std::string payload;   // refresh token (Restore) / icao / chart id
    std::string payload2;  // image URL (ChartImage)
    bool flag = false;     // night variant (ChartImage)
  };

  void worker();
  // Auth/OFP commands bump the auth generation (superseding a prior sign-in or
  // fetch); chart commands bump a separate generation so a chart fetch never
  // cancels an in-flight sign-in (and vice versa).
  void enqueue(Command cmd);
  void enqueueChart(Command cmd);
  bool commsAllowed() const {
    return commsAllowed_.load(std::memory_order_acquire);
  }
  // Interruptible sleep between token polls; returns false if the operation was
  // cancelled (a newer command arrived or the store is shutting down).
  bool sleepInterruptible(int seconds, unsigned generation);
  bool cancelled(unsigned generation) const;
  bool chartCancelled(unsigned generation) const;
  // Refresh the cached access token when missing/expiring (charts need it
  // explicitly, unlike the OFP fetch which only needs the username). Returns
  // false when no valid token can be obtained.
  bool ensureFreshAccessToken(unsigned generation);

  void doLogin(unsigned generation);
  void doRestore(const std::string& refreshToken, unsigned generation);
  void doFetch(unsigned generation);
  void doLogout();
  void doChartIndex(const std::string& icao, unsigned generation);
  void doChartImage(const std::string& chartId, bool night,
                    const std::string& imageUrl, unsigned generation);

  void setSnapshot(const NavigraphAuthSnapshot& snap);
  void publishRefreshToken(const std::string& token);

  NavigraphCredentials creds_;

  // Owned by the worker thread only (no locking needed).
  std::string username_;
  std::string refreshToken_;
  std::string accessToken_;  // Bearer token for the Charts API
  std::chrono::steady_clock::time_point accessTokenExpiry_{};

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Command> queue_;
  bool stop_ = false;
  std::atomic<unsigned> generation_{0};
  std::thread thread_;

  NavigraphAuthSnapshot snapshot_;

  // Whether Navigraph communication is currently permitted (see
  // setCommunicationAllowed). Read by the worker before every network call.
  std::atomic<bool> commsAllowed_{true};

  std::atomic<bool> fetching_{false};
  bool resultReady_ = false;
  SimBriefFetchResult result_;

  bool refreshTokenReady_ = false;
  std::string pendingRefreshToken_;

  // Charts: a separate generation so superseding a chart fetch never cancels an
  // in-flight sign-in / OFP fetch.
  std::atomic<unsigned> chartGeneration_{0};
  ChartIndexSnapshot chartIndex_;  // guarded by mutex_
  bool chartImageReady_ = false;   // guarded by mutex_
  ChartImageResult chartImage_;    // guarded by mutex_
  // Render-thread-owned dedup keys for the per-frame selection setters.
  std::string lastChartAirport_;
  std::string lastChartKey_;
};

}  // namespace avionics
