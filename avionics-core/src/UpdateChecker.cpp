#include "avionics/UpdateChecker.h"

#include "avionics/Version.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <thread>
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

}  // namespace avionics
