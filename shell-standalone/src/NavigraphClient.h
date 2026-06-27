#pragma once

#include <string>

// Navigraph OpenID Connect client: the Device Authorization Flow with PKCE
// (RFC 8628) described at
// https://developers.navigraph.com/docs/authentication/device-authorization.
//
// The flow has three blocking HTTPS steps, all of which must run off the render
// thread (see NavigraphStore):
//   1. NavigraphBeginDeviceAuth  -> a user_code + verification URL to show the
//      pilot, plus a device_code we poll on.
//   2. NavigraphPollToken        -> repeated until the pilot authorizes in a
//      browser, yielding an access + refresh token.
//   3. NavigraphRefresh          -> swap a stored refresh token for fresh
//      tokens on the next launch (no manual sign-in needed).
//
// The access token is a JWT whose "preferred_username" claim is the user's
// Navigraph Alias, which doubles as their SimBrief username for the OFP fetch.

namespace avionics {

// Initializes libcurl's global state exactly once (thread-safe). libcurl's
// implicit init inside curl_easy_init is NOT thread-safe, and Navigraph/SimBrief
// HTTPS calls run on a background worker thread (NavigraphStore) concurrently
// with other curl users (update check, NEXRAD). Shells MUST call this on the
// main thread during startup, before any worker thread can issue a request.
void EnsureCurlGlobalInit();

// Client credentials issued by Navigraph (dev@navigraph.com). Loaded at runtime
// so the secret never lives in source control (see LoadNavigraphCredentials).
struct NavigraphCredentials {
  std::string clientId;
  std::string clientSecret;

  bool valid() const { return !clientId.empty() && !clientSecret.empty(); }
};

// Resolves credentials from, in order: the NAVIGRAPH_CLIENT_ID /
// NAVIGRAPH_CLIENT_SECRET environment variables, then a key=value
// "navigraph_credentials.txt" file in the current directory or the per-user
// config directory (the same XPlaneAvionics folder AppSettings uses). Returns
// an empty (invalid) struct when none is found.
NavigraphCredentials LoadNavigraphCredentials();

// Refresh-token persistence in the per-user config directory
// ("navigraph_token.txt"). The standalone shell persists the token through its
// own AppSettings instead; these helpers let the X-Plane plugin (which has no
// AppSettings) restore a session across sim restarts. Empty string on miss.
std::string LoadNavigraphRefreshToken();
void SaveNavigraphRefreshToken(const std::string& token);

// Result of step 1: the verification codes the pilot needs, plus the PKCE
// code_verifier that step 2 must echo back.
struct DeviceAuthResult {
  bool ok = false;
  std::string error;  // human-readable summary when !ok

  std::string deviceCode;
  std::string userCode;                 // short code shown on screen
  std::string verificationUri;          // base URL the pilot visits
  std::string verificationUriComplete;  // URL with the code pre-filled
  std::string codeVerifier;             // PKCE verifier for the token request
  int intervalSeconds = 5;              // minimum gap between token polls
  int expiresInSeconds = 600;          // lifetime of device_code / user_code
};

// Tokens and identity from a successful authorization or refresh.
struct NavigraphTokens {
  bool ok = false;
  std::string error;

  std::string accessToken;
  std::string refreshToken;
  std::string username;  // preferred_username claim (Navigraph Alias)
  int expiresInSeconds = 3600;
};

// Outcome of a single token-poll attempt during the device flow.
enum class TokenPollStatus {
  Success,    // tokens populated
  Pending,    // user has not authorized yet; poll again after the interval
  SlowDown,   // polling too fast; add 5 s to the interval and poll again
  Denied,     // user declined the authorization
  Expired,    // device_code expired; restart the flow
  Error,      // transport/parse failure (see tokens.error)
};

struct TokenPollResult {
  TokenPollStatus status = TokenPollStatus::Error;
  NavigraphTokens tokens;  // valid when status == Success
};

// Step 1: request device + user codes. Blocking.
DeviceAuthResult NavigraphBeginDeviceAuth(const NavigraphCredentials& creds);

// Step 2: exchange the device_code for tokens. Blocking; call once per poll
// interval until it returns a terminal status.
TokenPollResult NavigraphPollToken(const NavigraphCredentials& creds,
                                   const std::string& deviceCode,
                                   const std::string& codeVerifier);

// Step 3: exchange a stored refresh token for fresh tokens. Blocking. The
// refresh token is single-use, so callers must persist the new one returned.
NavigraphTokens NavigraphRefresh(const NavigraphCredentials& creds,
                                 const std::string& refreshToken);

// True when the JWT access token's `subscriptions` claim includes `charts`
// (Navigraph Ultimate). Used to distinguish a missing scope / stale sign-in from
// a subscription that does not include chart access.
bool NavigraphTokenHasChartsSubscription(const std::string& accessToken);

}  // namespace avionics
