#pragma once

#include <functional>
#include <string>

namespace avionics {

// Result of comparing the running build against the latest GitHub release.
struct UpdateInfo {
  bool newerAvailable = false;
  std::string latestVersion;  // no leading "v"
  std::string releasePageUrl; // browser URL for the release notes / downloads
  // Direct download URL of the installer asset for the running platform (empty
  // when none matches), used by the shell's in-app updater instead of opening
  // the browser. Currently populated on Windows (the g1000nxi-setup-*.exe).
  std::string installerUrl;
  // Download URL of the release's SHA256SUMS asset (empty when absent), used to
  // verify the downloaded installer before running it.
  std::string checksumsUrl;
  // Direct download URL of the plugin-only archive for the running platform
  // (g1000nxi-plugin-<platform>-<version>.zip), used by the X-Plane plugin's
  // in-sim updater to refresh just the .xpl + assets and hot-reload. Empty when
  // no matching asset is present.
  std::string pluginArchiveUrl;
};

// Returns the browser_download_url of the first release asset whose file name
// matches the given prefix and suffix (either may be empty to match any).
// Exposed for testing; parseLatestReleaseJson fills UpdateInfo with it.
std::string findReleaseAssetUrl(const std::string& jsonBody,
                                const std::string& namePrefix,
                                const std::string& nameSuffix);

// Parse a GitHub Releases API JSON body and compare against kVersion.
// Returns an empty latestVersion when the response cannot be parsed.
UpdateInfo parseLatestReleaseJson(const std::string& jsonBody);

// Returns true when `latest` is strictly newer than `current` (semver-ish:
// dot-separated non-negative integers compared left to right).
bool isVersionNewer(const std::string& latest, const std::string& current);

// Build the REST URL for the latest release of this project.
std::string latestReleaseApiUrl();

// Short advisory line shown in the PFD Alerts window when a newer release is
// detected (e.g. "UPDATE AVAILABLE v1.2.3"). Kept short so it fits the Alerts
// window width, and centralized here so both shells word it identically.
std::string updateAdvisoryText(const std::string& latestVersion);

// Process-global advisory surfaced in the PFD Alerts window. The shell's
// update-check callback sets it (from a background thread) when a newer release
// is found; the softkey controller reads it each frame and injects it into the
// Alerts list (which also flashes the "Messages" softkey). Empty means none.
// Both accessors are thread-safe.
void setUpdateAdvisory(std::string text);
std::string updateAdvisory();

#if defined(AVIONICS_HAS_CURL)
// Fetch the latest-release JSON over HTTPS (blocks; intended for a worker
// thread). Returns an empty string on network/HTTP failure.
std::string fetchLatestReleaseJson();

// Fire-and-forget background check. The callback runs on the worker thread when
// the fetch completes; the UI shell should marshal to its main thread if needed.
void checkForUpdatesAsync(std::function<void(UpdateInfo)> callback);
#endif

}  // namespace avionics
