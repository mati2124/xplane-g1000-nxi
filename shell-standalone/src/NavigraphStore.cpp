#include "NavigraphStore.h"

#include <chrono>
#include <utility>

#include "avionics/FplRouteEdit.h"

namespace avionics {

NavigraphStore::NavigraphStore() {
  thread_ = std::thread([this]() { worker(); });
}

NavigraphStore::~NavigraphStore() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    generation_.fetch_add(1, std::memory_order_acq_rel);  // abort any poll loop
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void NavigraphStore::enqueue(Command cmd) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A new action supersedes any in-flight one (e.g. Logout during a sign-in
    // poll loop); the generation bump makes the running operation bail out.
    generation_.fetch_add(1, std::memory_order_acq_rel);
    queue_.push_back(std::move(cmd));
  }
  cv_.notify_all();
}

void NavigraphStore::enqueueChart(Command cmd) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Supersede any prior chart fetch only (its own generation), so it never
    // aborts an in-flight sign-in / OFP fetch.
    chartGeneration_.fetch_add(1, std::memory_order_acq_rel);
    queue_.push_back(std::move(cmd));
  }
  cv_.notify_all();
}

void NavigraphStore::setCommunicationAllowed(bool allowed) {
  const bool prev = commsAllowed_.exchange(allowed, std::memory_order_acq_rel);
  if (prev && !allowed) {
    // The link just dropped: wake any in-progress sign-in poll so it bails out
    // promptly instead of continuing to poll Navigraph off-sim.
    cv_.notify_all();
  }
}

void NavigraphStore::restoreSession(const std::string& refreshToken) {
  if (refreshToken.empty()) return;
  enqueue({CommandType::Restore, refreshToken});
}

void NavigraphStore::requestLogin() { enqueue({CommandType::Login, {}}); }

void NavigraphStore::requestLogout() { enqueue({CommandType::Logout, {}}); }

void NavigraphStore::requestFetch() { enqueue({CommandType::Fetch, {}}); }

void NavigraphStore::setChartAirport(const std::string& icao) {
  // Only 4-letter ICAO airport idents are valid chart keys (not fixes).
  const std::string airport = isAirportIdent(icao) ? icao : std::string();
  if (airport == lastChartAirport_) return;
  lastChartAirport_ = airport;
  lastChartKey_.clear();  // a new airport invalidates the displayed image
  if (airport.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    chartIndex_ = ChartIndexSnapshot{};
    return;
  }
  enqueueChart({CommandType::ChartIndex, airport, {}, false});
}

void NavigraphStore::setChartSelection(const std::string& chartId, bool night,
                                       const std::string& imageUrl) {
  if (chartId.empty() || imageUrl.empty()) return;
  const std::string key = chartId + (night ? 'N' : 'D');
  if (key == lastChartKey_) return;
  lastChartKey_ = key;
  enqueueChart({CommandType::ChartImage, chartId, imageUrl, night});
}

ChartIndexSnapshot NavigraphStore::chartIndexSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return chartIndex_;
}

bool NavigraphStore::consumeChartImage(ChartImageResult& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!chartImageReady_) return false;
  chartImageReady_ = false;
  out = chartImage_;
  return true;
}

NavigraphAuthSnapshot NavigraphStore::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool NavigraphStore::consumeResult(SimBriefFetchResult& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!resultReady_) return false;
  resultReady_ = false;
  out = result_;
  return true;
}

bool NavigraphStore::consumeRefreshToken(std::string& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!refreshTokenReady_) return false;
  refreshTokenReady_ = false;
  out = pendingRefreshToken_;
  return true;
}

void NavigraphStore::setSnapshot(const NavigraphAuthSnapshot& snap) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = snap;
}

void NavigraphStore::publishRefreshToken(const std::string& token) {
  std::lock_guard<std::mutex> lock(mutex_);
  pendingRefreshToken_ = token;
  refreshTokenReady_ = true;
}

bool NavigraphStore::cancelled(unsigned generation) const {
  return stop_ || generation_.load(std::memory_order_acquire) != generation;
}

bool NavigraphStore::chartCancelled(unsigned generation) const {
  return stop_ ||
         chartGeneration_.load(std::memory_order_acquire) != generation;
}

bool NavigraphStore::ensureFreshAccessToken(unsigned generation) {
  const auto now = std::chrono::steady_clock::now();
  // Keep a 60 s safety margin so a token does not expire mid-request.
  if (!accessToken_.empty() &&
      now + std::chrono::seconds(60) < accessTokenExpiry_) {
    return true;
  }
  if (refreshToken_.empty()) return false;
  const NavigraphTokens t = NavigraphRefresh(creds_, refreshToken_);
  if (chartCancelled(generation)) return false;
  if (!t.ok || t.accessToken.empty()) return false;
  accessToken_ = t.accessToken;
  accessTokenExpiry_ =
      std::chrono::steady_clock::now() + std::chrono::seconds(t.expiresInSeconds);
  if (!t.refreshToken.empty()) {
    refreshToken_ = t.refreshToken;
    publishRefreshToken(refreshToken_);
  }
  return true;
}

bool NavigraphStore::sleepInterruptible(int seconds, unsigned generation) {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, std::chrono::seconds(seconds), [&]() {
    return stop_ || generation_.load(std::memory_order_acquire) != generation ||
           !commsAllowed_.load(std::memory_order_acquire);
  });
  return !stop_ &&
         generation_.load(std::memory_order_acquire) == generation;
}

void NavigraphStore::worker() {
  for (;;) {
    Command cmd;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [&]() { return stop_ || !queue_.empty(); });
      if (stop_ && queue_.empty()) return;
      cmd = std::move(queue_.front());
      queue_.pop_front();
    }
    const unsigned gen = generation_.load(std::memory_order_acquire);
    const unsigned chartGen = chartGeneration_.load(std::memory_order_acquire);
    switch (cmd.type) {
      case CommandType::Login:
        doLogin(gen);
        break;
      case CommandType::Restore:
        doRestore(cmd.payload, gen);
        break;
      case CommandType::Fetch:
        doFetch(gen);
        break;
      case CommandType::Logout:
        doLogout();
        break;
      case CommandType::ChartIndex:
        doChartIndex(cmd.payload, chartGen);
        break;
      case CommandType::ChartImage:
        doChartImage(cmd.payload, cmd.flag, cmd.payload2, chartGen);
        break;
    }
  }
}

void NavigraphStore::doLogin(unsigned generation) {
  if (!creds_.valid()) {
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::Error;
    snap.error = "NO CREDENTIALS";
    setSnapshot(snap);
    return;
  }
  if (!commsAllowed()) {
    // Navigraph's terms require a connected sim session for data access.
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::Error;
    snap.error = "CONNECT SIM";
    setSnapshot(snap);
    return;
  }

  const DeviceAuthResult da = NavigraphBeginDeviceAuth(creds_);
  if (cancelled(generation)) return;
  if (!da.ok) {
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::Error;
    snap.error = da.error.empty() ? "SIGN IN FAILED" : da.error;
    setSnapshot(snap);
    return;
  }

  {
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::AwaitingUser;
    snap.userCode = da.userCode;
    snap.verificationUri = da.verificationUri;
    snap.verificationUriComplete = da.verificationUriComplete;
    setSnapshot(snap);
  }

  int interval = da.intervalSeconds > 0 ? da.intervalSeconds : 5;
  int elapsed = 0;
  while (elapsed < da.expiresInSeconds) {
    if (!sleepInterruptible(interval, generation)) return;  // cancelled
    if (!commsAllowed()) {
      // Sim link dropped mid sign-in: stop polling Navigraph and revert to the
      // signed-out prompt rather than leaving a stale "signing in" state.
      NavigraphAuthSnapshot snap;
      snap.phase = NavigraphAuthPhase::LoggedOut;
      setSnapshot(snap);
      return;
    }
    elapsed += interval;

    const TokenPollResult poll =
        NavigraphPollToken(creds_, da.deviceCode, da.codeVerifier);
    if (cancelled(generation)) return;

    if (poll.status == TokenPollStatus::Pending) {
      continue;
    }
    if (poll.status == TokenPollStatus::SlowDown) {
      interval += 5;
      continue;
    }
    if (poll.status == TokenPollStatus::Success) {
      username_ = poll.tokens.username;
      refreshToken_ = poll.tokens.refreshToken;
      accessToken_ = poll.tokens.accessToken;
      accessTokenExpiry_ = std::chrono::steady_clock::now() +
                           std::chrono::seconds(poll.tokens.expiresInSeconds);
      publishRefreshToken(refreshToken_);
      NavigraphAuthSnapshot snap;
      snap.phase = NavigraphAuthPhase::LoggedIn;
      snap.username = username_;
      setSnapshot(snap);
      doFetch(generation);  // auto-load the latest OFP after sign-in
      return;
    }

    // Terminal failure (Denied / Expired / Error).
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::Error;
    switch (poll.status) {
      case TokenPollStatus::Denied:
        snap.error = "ACCESS DENIED";
        break;
      case TokenPollStatus::Expired:
        snap.error = "CODE EXPIRED";
        break;
      default:
        snap.error = poll.tokens.error.empty() ? "SIGN IN FAILED"
                                               : poll.tokens.error;
        break;
    }
    setSnapshot(snap);
    return;
  }

  NavigraphAuthSnapshot snap;
  snap.phase = NavigraphAuthPhase::Error;
  snap.error = "CODE EXPIRED";
  setSnapshot(snap);
}

void NavigraphStore::doRestore(const std::string& refreshToken,
                               unsigned generation) {
  // Refreshing the token is network traffic; only do it on a live sim session.
  // The shell defers the restore until connected, so this is a safety net.
  if (!commsAllowed()) return;
  const NavigraphTokens tokens = NavigraphRefresh(creds_, refreshToken);
  if (cancelled(generation)) return;
  if (!tokens.ok) {
    // A stale/invalid refresh token just means the pilot must sign in again;
    // surface it as logged-out rather than an error banner.
    NavigraphAuthSnapshot snap;
    snap.phase = NavigraphAuthPhase::LoggedOut;
    setSnapshot(snap);
    return;
  }
  username_ = tokens.username;
  refreshToken_ = tokens.refreshToken;
  accessToken_ = tokens.accessToken;
  accessTokenExpiry_ = std::chrono::steady_clock::now() +
                       std::chrono::seconds(tokens.expiresInSeconds);
  publishRefreshToken(refreshToken_);
  NavigraphAuthSnapshot snap;
  snap.phase = NavigraphAuthPhase::LoggedIn;
  snap.username = username_;
  setSnapshot(snap);
  doFetch(generation);
}

void NavigraphStore::doFetch(unsigned generation) {
  if (username_.empty()) return;
  if (!commsAllowed()) return;  // SimBrief fetch requires a live sim session
  fetching_.store(true, std::memory_order_release);
  SimBriefFetchResult result = FetchSimBriefOfpByUsername(username_);
  fetching_.store(false, std::memory_order_release);
  if (cancelled(generation)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  result_ = std::move(result);
  resultReady_ = true;
}

void NavigraphStore::doLogout() {
  username_.clear();
  refreshToken_.clear();
  accessToken_.clear();
  accessTokenExpiry_ = {};
  lastChartAirport_.clear();
  lastChartKey_.clear();
  publishRefreshToken(std::string());  // clears the persisted token
  NavigraphAuthSnapshot snap;
  snap.phase = NavigraphAuthPhase::LoggedOut;
  setSnapshot(snap);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    chartIndex_ = ChartIndexSnapshot{};
    chartImageReady_ = false;
    chartImage_ = ChartImageResult{};
  }
}

void NavigraphStore::doChartIndex(const std::string& icao, unsigned generation) {
  if (icao.empty()) return;
  if (!commsAllowed()) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    chartIndex_.status = ChartIndexStatus::Loading;
    chartIndex_.icao = icao;
    chartIndex_.error.clear();
    chartIndex_.charts.clear();
  }
  if (!ensureFreshAccessToken(generation)) {
    if (chartCancelled(generation)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    chartIndex_.status = ChartIndexStatus::Error;
    chartIndex_.error = "NOT SIGNED IN";
    return;
  }
  const ChartIndexResult result =
      NavigraphFetchChartIndex(accessToken_, icao);
  if (chartCancelled(generation)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  chartIndex_.icao = icao;
  if (!result.ok) {
    chartIndex_.status = ChartIndexStatus::Error;
    chartIndex_.error = result.error.empty() ? "FAIL" : result.error;
    chartIndex_.charts.clear();
    return;
  }
  chartIndex_.charts = result.charts;
  chartIndex_.status =
      result.charts.empty() ? ChartIndexStatus::Empty : ChartIndexStatus::Ready;
  chartIndex_.error.clear();
}

void NavigraphStore::doChartImage(const std::string& chartId, bool night,
                                  const std::string& imageUrl,
                                  unsigned generation) {
  if (chartId.empty() || imageUrl.empty()) return;
  if (!commsAllowed()) return;
  if (!ensureFreshAccessToken(generation)) {
    if (chartCancelled(generation)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    chartImage_ = ChartImageResult{};
    chartImage_.chartId = chartId;
    chartImage_.night = night;
    chartImage_.error = "NOT SIGNED IN";
    chartImageReady_ = true;
    return;
  }
  ChartImageResult result =
      NavigraphFetchChartImage(accessToken_, chartId, night, imageUrl);
  if (chartCancelled(generation)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  chartImage_ = std::move(result);
  chartImageReady_ = true;
}

}  // namespace avionics
