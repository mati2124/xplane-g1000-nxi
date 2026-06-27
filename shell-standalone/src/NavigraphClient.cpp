#include "NavigraphClient.h"

#include <curl/curl.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <random>
#include <string>

#include "Sha256.h"

namespace avionics {
namespace {

using nlohmann::json;

// libcurl's implicit global init (inside the first curl_easy_init) is not
// thread-safe. These HTTPS calls run on the NavigraphStore worker thread, so
// guard global init behind call_once and make every curl entry point trigger it.
std::once_flag g_curlInitFlag;
void ensureCurlInit() {
  std::call_once(g_curlInitFlag, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

// Navigraph identity server (OpenID Connect). The device-authorization and
// token endpoints both live here.
constexpr const char* kDeviceAuthUrl =
    "https://identity.api.navigraph.com/connect/deviceauthorization";
constexpr const char* kTokenUrl =
    "https://identity.api.navigraph.com/connect/token";
// Scopes: openid yields the preferred_username claim used as the SimBrief
// username; offline_access yields the long-lived refresh token; charts grants
// the Navigraph Charts API (the AUX - Charts page). Without an Ultimate
// subscription the charts scope still authorizes, but the API only returns the
// demo airports (NZWN / YBBN).
constexpr const char* kScopes = "openid offline_access charts";
constexpr long kHttpTimeoutSeconds = 20;

constexpr const char* kAppDirName = "XPlaneAvionics";
constexpr const char* kCredentialsFileName = "navigraph_credentials.txt";
constexpr const char* kRefreshTokenFileName = "navigraph_token.txt";

std::size_t WriteToString(const char* data, std::size_t size, std::size_t nmemb,
                          void* userData) {
  static_cast<std::string*>(userData)->append(data, size * nmemb);
  return size * nmemb;
}

// POSTs an application/x-www-form-urlencoded body and returns the response body
// (out) plus HTTP status. Returns false only on a transport failure.
bool HttpPostForm(const char* url, const std::string& body, std::string& out,
                  long& httpStatus) {
  ensureCurlInit();
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;

  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                   static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, kHttpTimeoutSeconds);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "XPlaneAvionics");

  const CURLcode rc = curl_easy_perform(curl);
  httpStatus = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpStatus);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK;
}

// Percent-encodes a value for an x-www-form-urlencoded body.
std::string UrlEncode(const std::string& value) {
  ensureCurlInit();
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return value;
  char* escaped = curl_easy_escape(curl, value.c_str(),
                                   static_cast<int>(value.size()));
  std::string out = escaped != nullptr ? escaped : value;
  if (escaped != nullptr) curl_free(escaped);
  curl_easy_cleanup(curl);
  return out;
}

std::string Base64UrlEncode(const std::string& data) {
  static const char* kAlphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  std::string out;
  out.reserve((data.size() + 2) / 3 * 4);
  std::size_t i = 0;
  while (i + 3 <= data.size()) {
    const std::uint32_t n = (static_cast<unsigned char>(data[i]) << 16) |
                            (static_cast<unsigned char>(data[i + 1]) << 8) |
                            static_cast<unsigned char>(data[i + 2]);
    out += kAlphabet[(n >> 18) & 63];
    out += kAlphabet[(n >> 12) & 63];
    out += kAlphabet[(n >> 6) & 63];
    out += kAlphabet[n & 63];
    i += 3;
  }
  if (data.size() - i == 1) {
    const std::uint32_t n = static_cast<unsigned char>(data[i]) << 16;
    out += kAlphabet[(n >> 18) & 63];
    out += kAlphabet[(n >> 12) & 63];
  } else if (data.size() - i == 2) {
    const std::uint32_t n = (static_cast<unsigned char>(data[i]) << 16) |
                            (static_cast<unsigned char>(data[i + 1]) << 8);
    out += kAlphabet[(n >> 18) & 63];
    out += kAlphabet[(n >> 12) & 63];
    out += kAlphabet[(n >> 6) & 63];
  }
  return out;  // base64url: no padding
}

std::string Base64UrlDecode(const std::string& in) {
  auto value = [](char c) -> int {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return -1;
  };
  std::string out;
  int buffer = 0;
  int bits = 0;
  for (const char c : in) {
    const int v = value(c);
    if (v < 0) continue;
    buffer = (buffer << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out += static_cast<char>((buffer >> bits) & 0xFF);
    }
  }
  return out;
}

// PKCE code_verifier: 64 characters drawn from the RFC 7636 unreserved set.
std::string MakeCodeVerifier() {
  static const char* kUnreserved =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-._~";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, 64);  // 65 chars, indices 0..64
  std::string out;
  out.reserve(64);
  for (int i = 0; i < 64; ++i) out += kUnreserved[dist(gen)];
  return out;
}

// Reads the "preferred_username" claim out of a JWT access token. Empty when
// the token is malformed or lacks the claim.
std::string UsernameFromJwt(const std::string& jwt) {
  const std::size_t first = jwt.find('.');
  if (first == std::string::npos) return {};
  const std::size_t second = jwt.find('.', first + 1);
  if (second == std::string::npos) return {};
  const std::string payload =
      Base64UrlDecode(jwt.substr(first + 1, second - first - 1));
  const json j = json::parse(payload, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) return {};
  const auto it = j.find("preferred_username");
  if (it != j.end() && it->is_string()) return it->get<std::string>();
  return {};
}

json JwtPayload(const std::string& jwt) {
  const std::size_t first = jwt.find('.');
  if (first == std::string::npos) return json::object();
  const std::size_t second = jwt.find('.', first + 1);
  if (second == std::string::npos) return json::object();
  const std::string payload =
      Base64UrlDecode(jwt.substr(first + 1, second - first - 1));
  const json j = json::parse(payload, nullptr, /*allow_exceptions=*/false);
  return j.is_discarded() || !j.is_object() ? json::object() : j;
}

std::filesystem::path ConfigDir() {
#if defined(_WIN32)
  if (const char* appData = std::getenv("APPDATA")) {
    return std::filesystem::path(appData) / kAppDirName;
  }
#elif defined(__APPLE__)
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / "Library" / "Application Support" /
           kAppDirName;
  }
#else
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) {
    return std::filesystem::path(xdg) / kAppDirName;
  }
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / ".config" / kAppDirName;
  }
#endif
  return {};
}

// Parses a key=value credentials file, accepting both snake_case and the
// environment-variable spellings for each field.
bool ReadCredentialsFile(const std::filesystem::path& path,
                         NavigraphCredentials& creds) {
  std::ifstream in(path);
  if (!in.is_open()) return false;
  std::string line;
  while (std::getline(in, line)) {
    const std::string::size_type eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string key = line.substr(0, eq);
    std::string value = line.substr(eq + 1);
    // Trim trailing CR (Windows line endings) and surrounding spaces.
    while (!value.empty() && (value.back() == '\r' || value.back() == ' ')) {
      value.pop_back();
    }
    if (key == "client_id" || key == "clientId" ||
        key == "NAVIGRAPH_CLIENT_ID") {
      creds.clientId = value;
    } else if (key == "client_secret" || key == "clientSecret" ||
               key == "NAVIGRAPH_CLIENT_SECRET") {
      creds.clientSecret = value;
    }
  }
  return creds.valid();
}

// Common token-endpoint response handling: maps a JSON body into tokens.
NavigraphTokens ParseTokenResponse(const std::string& body, long httpStatus) {
  NavigraphTokens tokens;
  const json j = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    tokens.error = "HTTP " + std::to_string(httpStatus);
    return tokens;
  }
  const auto access = j.find("access_token");
  const auto refresh = j.find("refresh_token");
  if (access == j.end() || !access->is_string()) {
    const auto err = j.find("error");
    tokens.error = err != j.end() && err->is_string()
                       ? err->get<std::string>()
                       : "HTTP " + std::to_string(httpStatus);
    return tokens;
  }
  tokens.accessToken = access->get<std::string>();
  if (refresh != j.end() && refresh->is_string()) {
    tokens.refreshToken = refresh->get<std::string>();
  }
  const auto expires = j.find("expires_in");
  if (expires != j.end() && expires->is_number()) {
    tokens.expiresInSeconds = expires->get<int>();
  }
  tokens.username = UsernameFromJwt(tokens.accessToken);
  tokens.ok = true;
  return tokens;
}

}  // namespace

void EnsureCurlGlobalInit() { ensureCurlInit(); }

NavigraphCredentials LoadNavigraphCredentials() {
  NavigraphCredentials creds;
  if (const char* id = std::getenv("NAVIGRAPH_CLIENT_ID")) creds.clientId = id;
  if (const char* secret = std::getenv("NAVIGRAPH_CLIENT_SECRET")) {
    creds.clientSecret = secret;
  }
  if (creds.valid()) return creds;

  if (ReadCredentialsFile(std::filesystem::path(kCredentialsFileName), creds)) {
    return creds;
  }
  const std::filesystem::path dir = ConfigDir();
  if (!dir.empty()) {
    ReadCredentialsFile(dir / kCredentialsFileName, creds);
  }
  return creds;
}

std::string LoadNavigraphRefreshToken() {
  const std::filesystem::path dir = ConfigDir();
  if (dir.empty()) return {};
  std::ifstream in(dir / kRefreshTokenFileName);
  if (!in.is_open()) return {};
  std::string token;
  std::getline(in, token);
  while (!token.empty() && (token.back() == '\r' || token.back() == '\n')) {
    token.pop_back();
  }
  return token;
}

void SaveNavigraphRefreshToken(const std::string& token) {
  const std::filesystem::path dir = ConfigDir();
  if (dir.empty()) return;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  std::ofstream out(dir / kRefreshTokenFileName, std::ios::trunc);
  if (out.is_open()) out << token << '\n';
}

DeviceAuthResult NavigraphBeginDeviceAuth(const NavigraphCredentials& creds) {
  DeviceAuthResult result;
  if (!creds.valid()) {
    result.error = "NO CREDENTIALS";
    return result;
  }

  const std::string codeVerifier = MakeCodeVerifier();
  const std::string codeChallenge = Base64UrlEncode(sha256Raw(codeVerifier));

  const std::string post = "client_id=" + UrlEncode(creds.clientId) +
                           "&client_secret=" + UrlEncode(creds.clientSecret) +
                           "&code_challenge=" + UrlEncode(codeChallenge) +
                           "&code_challenge_method=S256";
  std::string body;
  long httpStatus = 0;
  if (!HttpPostForm(kDeviceAuthUrl, post, body, httpStatus)) {
    result.error = "NO CONNECTION";
    return result;
  }
  const json j = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object() || !j.contains("device_code")) {
    result.error = "HTTP " + std::to_string(httpStatus);
    return result;
  }

  result.deviceCode = j.value("device_code", "");
  result.userCode = j.value("user_code", "");
  result.verificationUri = j.value("verification_uri", "");
  result.verificationUriComplete = j.value("verification_uri_complete", "");
  result.intervalSeconds = j.value("interval", 5);
  result.expiresInSeconds = j.value("expires_in", 600);
  result.codeVerifier = codeVerifier;
  result.ok = !result.deviceCode.empty();
  if (!result.ok) result.error = "BAD RESPONSE";
  return result;
}

TokenPollResult NavigraphPollToken(const NavigraphCredentials& creds,
                                   const std::string& deviceCode,
                                   const std::string& codeVerifier) {
  TokenPollResult result;
  const std::string post =
      "grant_type=urn:ietf:params:oauth:grant-type:device_code"
      "&device_code=" +
      UrlEncode(deviceCode) + "&code_verifier=" + UrlEncode(codeVerifier) +
      "&client_id=" + UrlEncode(creds.clientId) +
      "&client_secret=" + UrlEncode(creds.clientSecret) +
      "&scope=" + UrlEncode(kScopes);

  std::string body;
  long httpStatus = 0;
  if (!HttpPostForm(kTokenUrl, post, body, httpStatus)) {
    result.status = TokenPollStatus::Error;
    result.tokens.error = "NO CONNECTION";
    return result;
  }

  if (httpStatus == 200) {
    result.tokens = ParseTokenResponse(body, httpStatus);
    result.status = result.tokens.ok ? TokenPollStatus::Success
                                     : TokenPollStatus::Error;
    return result;
  }

  // Non-200: the body carries an OAuth error code that drives the poll loop.
  const json j = json::parse(body, nullptr, /*allow_exceptions=*/false);
  const std::string error =
      (!j.is_discarded() && j.is_object() && j.contains("error"))
          ? j.value("error", "")
          : "";
  if (error == "authorization_pending") {
    result.status = TokenPollStatus::Pending;
  } else if (error == "slow_down") {
    result.status = TokenPollStatus::SlowDown;
  } else if (error == "access_denied") {
    result.status = TokenPollStatus::Denied;
  } else if (error == "expired_token") {
    result.status = TokenPollStatus::Expired;
  } else {
    result.status = TokenPollStatus::Error;
    result.tokens.error = error.empty() ? "HTTP " + std::to_string(httpStatus)
                                        : error;
  }
  return result;
}

NavigraphTokens NavigraphRefresh(const NavigraphCredentials& creds,
                                 const std::string& refreshToken) {
  NavigraphTokens tokens;
  if (!creds.valid() || refreshToken.empty()) {
    tokens.error = "NO CREDENTIALS";
    return tokens;
  }
  const std::string post = "grant_type=refresh_token"
                           "&client_id=" +
                           UrlEncode(creds.clientId) +
                           "&client_secret=" + UrlEncode(creds.clientSecret) +
                           "&refresh_token=" + UrlEncode(refreshToken);
  std::string body;
  long httpStatus = 0;
  if (!HttpPostForm(kTokenUrl, post, body, httpStatus)) {
    tokens.error = "NO CONNECTION";
    return tokens;
  }
  return ParseTokenResponse(body, httpStatus);
}

bool NavigraphTokenHasChartsSubscription(const std::string& accessToken) {
  const json payload = JwtPayload(accessToken);
  const auto subs = payload.find("subscriptions");
  if (subs == payload.end() || !subs->is_array()) return false;
  for (const json& entry : *subs) {
    if (entry.is_string() && entry.get<std::string>() == "charts") return true;
  }
  return false;
}

}  // namespace avionics
