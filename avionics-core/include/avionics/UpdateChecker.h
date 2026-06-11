#pragma once

#include <functional>
#include <string>

namespace avionics {

// Result of comparing the running build against the latest GitHub release.
struct UpdateInfo {
  bool newerAvailable = false;
  std::string latestVersion;  // no leading "v"
  std::string releasePageUrl; // browser URL for the release notes / downloads
};

// Parse a GitHub Releases API JSON body and compare against kVersion.
// Returns an empty latestVersion when the response cannot be parsed.
UpdateInfo parseLatestReleaseJson(const std::string& jsonBody);

// Returns true when `latest` is strictly newer than `current` (semver-ish:
// dot-separated non-negative integers compared left to right).
bool isVersionNewer(const std::string& latest, const std::string& current);

// Build the REST URL for the latest release of this project.
std::string latestReleaseApiUrl();

#if defined(AVIONICS_HAS_CURL)
// Fetch the latest-release JSON over HTTPS (blocks; intended for a worker
// thread). Returns an empty string on network/HTTP failure.
std::string fetchLatestReleaseJson();

// Fire-and-forget background check. The callback runs on the worker thread when
// the fetch completes; the UI shell should marshal to its main thread if needed.
void checkForUpdatesAsync(std::function<void(UpdateInfo)> callback);
#endif

}  // namespace avionics
