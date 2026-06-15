#include "avionics/UpdateChecker.h"

#include "avionics/Version.h"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#if defined(AVIONICS_HAS_CURL)
#include <curl/curl.h>
#endif

namespace avionics {
namespace {

std::vector<int> parseVersionParts(const std::string& version) {
  std::vector<int> parts;
  std::istringstream stream(version);
  std::string segment;
  while (std::getline(stream, segment, '.')) {
    int value = 0;
    for (char ch : segment) {
      if (!std::isdigit(static_cast<unsigned char>(ch))) {
        value = 0;
        break;
      }
      value = value * 10 + (ch - '0');
    }
    parts.push_back(value);
  }
  return parts;
}

bool startsWith(const std::string& s, const std::string& prefix) {
  return s.size() >= prefix.size() &&
         s.compare(0, prefix.size(), prefix) == 0;
}

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Extract the first JSON string value for `"key":` without pulling in a JSON
// library (the Releases API response is small and stable).
std::string jsonStringField(const std::string& json, const char* key) {
  const std::string needle = std::string("\"") + key + "\"";
  std::size_t pos = json.find(needle);
  if (pos == std::string::npos) return std::string();
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) return std::string();
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) return std::string();
  const std::size_t end = json.find('"', pos + 1);
  if (end == std::string::npos) return std::string();
  return json.substr(pos + 1, end - pos - 1);
}

#if defined(AVIONICS_HAS_CURL)
std::size_t appendToString(char* data, std::size_t size, std::size_t nmemb,
                           void* userData) {
  static_cast<std::string*>(userData)->append(data, size * nmemb);
  return size * nmemb;
}
#endif

}  // namespace

bool isVersionNewer(const std::string& latest, const std::string& current) {
  const std::vector<int> lhs = parseVersionParts(latest);
  const std::vector<int> rhs = parseVersionParts(current);
  const std::size_t count = std::max(lhs.size(), rhs.size());
  for (std::size_t i = 0; i < count; ++i) {
    const int a = i < lhs.size() ? lhs[i] : 0;
    const int b = i < rhs.size() ? rhs[i] : 0;
    if (a > b) return true;
    if (a < b) return false;
  }
  return false;
}

std::string latestReleaseApiUrl() {
  return std::string("https://api.github.com/repos/") + kReleasesOwner + '/' +
         kReleasesRepo + "/releases/latest";
}

std::string findReleaseAssetUrl(const std::string& jsonBody,
                                const std::string& namePrefix,
                                const std::string& nameSuffix) {
  // Restrict scanning to the "assets" array so the release-level "name" / URL
  // fields (and the uploader object) are never mistaken for an asset.
  const std::size_t region = jsonBody.find("\"assets\"");
  if (region == std::string::npos) return std::string();

  std::size_t pos = region;
  while (true) {
    const std::size_t namePos = jsonBody.find("\"name\"", pos);
    if (namePos == std::string::npos) break;
    const std::size_t colon = jsonBody.find(':', namePos + 6);
    if (colon == std::string::npos) break;
    const std::size_t q1 = jsonBody.find('"', colon + 1);
    if (q1 == std::string::npos) break;
    const std::size_t q2 = jsonBody.find('"', q1 + 1);
    if (q2 == std::string::npos) break;
    const std::string name = jsonBody.substr(q1 + 1, q2 - q1 - 1);
    pos = q2 + 1;

    if ((namePrefix.empty() || startsWith(name, namePrefix)) &&
        (nameSuffix.empty() || endsWith(name, nameSuffix))) {
      // Within a GitHub asset object "name" precedes "browser_download_url",
      // so the next occurrence after the name belongs to this asset.
      const std::size_t urlKey =
          jsonBody.find("\"browser_download_url\"", q2);
      if (urlKey == std::string::npos) return std::string();
      const std::size_t uColon = jsonBody.find(':', urlKey + 22);
      if (uColon == std::string::npos) return std::string();
      const std::size_t u1 = jsonBody.find('"', uColon + 1);
      if (u1 == std::string::npos) return std::string();
      const std::size_t u2 = jsonBody.find('"', u1 + 1);
      if (u2 == std::string::npos) return std::string();
      return jsonBody.substr(u1 + 1, u2 - u1 - 1);
    }
  }
  return std::string();
}

UpdateInfo parseLatestReleaseJson(const std::string& jsonBody) {
  UpdateInfo info;
  if (jsonBody.empty()) return info;

  std::string tag = jsonStringField(jsonBody, "tag_name");
  if (!tag.empty() && tag[0] == 'v') tag.erase(0, 1);
  info.latestVersion = tag;
  info.releasePageUrl = jsonStringField(jsonBody, "html_url");
  info.newerAvailable =
      !info.latestVersion.empty() &&
      isVersionNewer(info.latestVersion, kVersion);

  // Direct-download assets for the in-app updater (the shell falls back to the
  // release page when these are empty). The installer asset is platform
  // specific; the checksum file is shared.
  info.checksumsUrl = findReleaseAssetUrl(jsonBody, "SHA256SUMS", "");
#if defined(_WIN32)
  info.installerUrl = findReleaseAssetUrl(jsonBody, "g1000nxi-setup-", ".exe");
  info.pluginArchiveUrl =
      findReleaseAssetUrl(jsonBody, "g1000nxi-plugin-windows-", ".zip");
#elif defined(__APPLE__)
  info.installerUrl =
      findReleaseAssetUrl(jsonBody, "g1000nxi-installer-macos-", ".dmg");
  info.pluginArchiveUrl =
      findReleaseAssetUrl(jsonBody, "g1000nxi-plugin-macos-", ".zip");
#elif defined(__linux__)
  info.installerUrl =
      findReleaseAssetUrl(jsonBody, "g1000nxi-installer-linux-", ".tar.gz");
  info.pluginArchiveUrl =
      findReleaseAssetUrl(jsonBody, "g1000nxi-plugin-linux-", ".zip");
#endif
  return info;
}

#if defined(AVIONICS_HAS_CURL)
std::string fetchLatestReleaseJson() {
  std::string body;
  CURL* curl = curl_easy_init();
  if (!curl) return body;

  const std::string url = latestReleaseApiUrl();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "G1000-NXi-UpdateChecker");
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

  const CURLcode rc = curl_easy_perform(curl);
  long httpCode = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK || httpCode != 200) return std::string();
  return body;
}

void checkForUpdatesAsync(std::function<void(UpdateInfo)> callback) {
  if (!callback) return;
  std::thread([cb = std::move(callback)] {
    const UpdateInfo info =
        parseLatestReleaseJson(fetchLatestReleaseJson());
    cb(info);
  }).detach();
}
#endif

namespace {
// Guards the process-global advisory string, which the update-check worker
// thread writes and the render thread reads each frame.
std::mutex& updateAdvisoryMutex() {
  static std::mutex m;
  return m;
}
std::string& updateAdvisoryStorage() {
  static std::string text;
  return text;
}
}  // namespace

std::string updateAdvisoryText(const std::string& latestVersion) {
  return "UPDATE AVAILABLE v" + latestVersion;
}

void setUpdateAdvisory(std::string text) {
  const std::lock_guard<std::mutex> lock(updateAdvisoryMutex());
  updateAdvisoryStorage() = std::move(text);
}

std::string updateAdvisory() {
  const std::lock_guard<std::mutex> lock(updateAdvisoryMutex());
  return updateAdvisoryStorage();
}

}  // namespace avionics
