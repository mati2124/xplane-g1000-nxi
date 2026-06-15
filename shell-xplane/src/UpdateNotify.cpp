#include "UpdateNotify.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "XPLMUtilities.h"
#include "avionics/UpdateChecker.h"
#include "avionics/Version.h"

#if defined(AVIONICS_HAS_CURL)
#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <curl/curl.h>

#include "Sha256.h"
#include "XPLMPlugin.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#endif
#endif  // AVIONICS_HAS_CURL

namespace avionics {
namespace {

#if defined(AVIONICS_HAS_CURL)

namespace fs = std::filesystem;

// Where the self-updater is in its lifecycle. All transitions out of the
// background-thread states (ReadyToReload / ReadyAfterRestart / Failed) happen
// on the main thread in pumpPluginSelfUpdate(), since the XPLM API and plugin
// reload must run there.
enum class SelfUpdateState {
  Idle,
  Downloading,
  ReadyToReload,      // files swapped in place (POSIX); reload to finish
  ReadyAfterRestart,  // Windows: cannot swap a loaded module in place
  Failed,
};

std::atomic<bool> g_updateAvailable{false};
std::atomic<SelfUpdateState> g_state{SelfUpdateState::Idle};

std::mutex g_mutex;  // guards the strings below
std::string g_latestVersion;
std::string g_archiveUrl;
std::string g_checksumsUrl;
std::string g_releasePageUrl;

// Short PFD-Alerts-window advisories for the self-update phases (kept brief to
// fit the Alerts window width).
constexpr const char* kAdvisoryDownloading = "DOWNLOADING UPDATE";
constexpr const char* kAdvisoryRestart = "UPDATE READY - RESTART X-PLANE";
constexpr const char* kAdvisoryFailed = "UPDATE DOWNLOAD FAILED";

std::size_t writeToFile(char* ptr, std::size_t size, std::size_t nmemb,
                        void* userData) {
  return std::fwrite(ptr, 1, size * nmemb, static_cast<FILE*>(userData));
}

std::size_t writeToString(char* ptr, std::size_t size, std::size_t nmemb,
                          void* userData) {
  static_cast<std::string*>(userData)->append(ptr, size * nmemb);
  return size * nmemb;
}

// HTTPS GET writing the body through `writeFn`/`writeData`. True only on 200.
bool curlFetch(const std::string& url,
               std::size_t (*writeFn)(char*, std::size_t, std::size_t, void*),
               void* writeData) {
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFn);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, writeData);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "G1000-NXi-UpdateChecker");
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 600L);
  const CURLcode rc = curl_easy_perform(curl);
  long httpCode = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK && httpCode == 200;
}

std::string baseName(const std::string& url) {
  const std::size_t slash = url.find_last_of('/');
  return slash == std::string::npos ? url : url.substr(slash + 1);
}

bool equalsIgnoreCase(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(a[i])) !=
        std::tolower(static_cast<unsigned char>(b[i]))) {
      return false;
    }
  }
  return true;
}

// The hash listed for `fileName` in a `sha256sum`-format SHA256SUMS body
// ("<hex>  <name>" per line). Empty when the file is not listed.
std::string expectedHashFor(const std::string& sums,
                            const std::string& fileName) {
  std::size_t start = 0;
  while (start < sums.size()) {
    std::size_t end = sums.find('\n', start);
    if (end == std::string::npos) end = sums.size();
    std::string line = sums.substr(start, end - start);
    start = end + 1;
    const std::size_t sp = line.find(' ');
    if (sp == std::string::npos) continue;
    std::string hash = line.substr(0, sp);
    std::string name = line.substr(sp);
    while (!name.empty() && (name.front() == ' ' || name.front() == '*')) {
      name.erase(0, 1);
    }
    while (!name.empty() && (name.back() == '\r' || name.back() == '\n' ||
                             name.back() == ' ')) {
      name.pop_back();
    }
    if (name == fileName) return hash;
  }
  return std::string();
}

// Downloads `url` to `localPath`, verifies its SHA-256 against the entry for
// its file name in the release's SHA256SUMS, and deletes it on any mismatch.
bool downloadAndVerify(const std::string& url, const std::string& checksumsUrl,
                       const fs::path& localPath) {
  FILE* file = std::fopen(localPath.string().c_str(), "wb");
  if (file == nullptr) return false;
  const bool got = curlFetch(url, writeToFile, file);
  std::fclose(file);
  std::error_code ec;
  if (!got) {
    fs::remove(localPath, ec);
    return false;
  }

  std::string sums;
  if (!curlFetch(checksumsUrl, writeToString, &sums)) {
    fs::remove(localPath, ec);
    return false;
  }
  const std::string expected = expectedHashFor(sums, baseName(url));

  std::ifstream in(localPath, std::ios::binary);
  const std::string actual = in ? sha256HexOfStream(in) : std::string();
  if (expected.empty() || actual.empty() ||
      !equalsIgnoreCase(expected, actual)) {
    fs::remove(localPath, ec);
    return false;
  }
  return true;
}

#if !defined(_WIN32)
// Single-quotes a string for safe embedding in a /bin/sh command.
std::string shq(const std::string& s) {
  std::string out = "'";
  for (char c : s) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  out += "'";
  return out;
}

// Extracts the (zip) plugin archive to `dest` using the platform's stock CLI
// unzip tool (ditto on macOS, unzip on Linux). Returns true on success.
bool extractArchive(const fs::path& archive, const fs::path& dest) {
#if defined(__APPLE__)
  const std::string cmd =
      "/usr/bin/ditto -x -k --noqtn " + shq(archive.string()) + " " +
      shq(dest.string());
#else
  const std::string cmd =
      "unzip -o -q " + shq(archive.string()) + " -d " + shq(dest.string());
#endif
  return std::system(cmd.c_str()) == 0;
}
#endif  // !_WIN32

// Copies every file under `from` over `to`, unlinking each destination first so
// a currently-loaded .xpl is replaced cleanly (POSIX keeps the running mapping
// of the unlinked inode until the reload). Returns false on any I/O error.
bool overlayTree(const fs::path& from, const fs::path& to) {
  std::error_code ec;
  fs::recursive_directory_iterator it(from, ec);
  if (ec) return false;
  for (; it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (ec) return false;
    const fs::path rel = fs::relative(it->path(), from, ec);
    if (ec) return false;
    const fs::path dst = to / rel;
    if (it->is_directory()) {
      fs::create_directories(dst, ec);
    } else {
      fs::create_directories(dst.parent_path(), ec);
      fs::remove(dst, ec);
      fs::copy_file(it->path(), dst, fs::copy_options::overwrite_existing, ec);
      if (ec) return false;
    }
  }
  return true;
}

// Absolute path of this plugin's .xpl, via the SDK. Empty on failure. Must be
// called on the main thread (XPLM API).
fs::path thisPluginXplPath() {
  char path[1024] = {0};
  XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
  return path[0] != '\0' ? fs::path(path) : fs::path();
}

void logLine(const std::string& text) {
  XPLMDebugString(("G1000 NXi: " + text + "\n").c_str());
}

// Background worker: download + verify + extract the plugin archive, then (on
// POSIX) overlay it onto the install. Leaves the final reload to the main
// thread via g_state. `pluginsDir` is the install parent (.../plugins).
void runSelfUpdate(std::string archiveUrl, std::string checksumsUrl,
                   fs::path pluginsDir) {
  std::error_code ec;
  const fs::path work =
      fs::temp_directory_path(ec) / "g1000nxi-plugin-update";
  fs::remove_all(work, ec);
  fs::create_directories(work, ec);
  if (ec) {
    g_state.store(SelfUpdateState::Failed);
    return;
  }

  const fs::path archive = work / baseName(archiveUrl);
  if (!downloadAndVerify(archiveUrl, checksumsUrl, archive)) {
    g_state.store(SelfUpdateState::Failed);
    return;
  }

#if defined(_WIN32)
  // A loaded .xpl (DLL) cannot be replaced in place on Windows, so an in-sim
  // hot-reload is not possible. Point the user at the installer instead; it
  // runs after X-Plane exits and the update applies on the next launch.
  (void)pluginsDir;
  fs::remove_all(work, ec);
  g_state.store(SelfUpdateState::ReadyAfterRestart);
#else
  const fs::path staging = work / "extracted";
  fs::create_directories(staging, ec);
  if (ec || !extractArchive(archive, staging)) {
    g_state.store(SelfUpdateState::Failed);
    return;
  }
  // The archive carries a top-level "xplane-avionics/" tree (the plugin folder
  // plus its assets); overlay it onto the plugins directory.
  const fs::path src = staging / "xplane-avionics";
  if (!fs::exists(src, ec) ||
      !overlayTree(src, pluginsDir / "xplane-avionics")) {
    g_state.store(SelfUpdateState::Failed);
    return;
  }
  fs::remove_all(work, ec);
  g_state.store(SelfUpdateState::ReadyToReload);
#endif
}

#if defined(_WIN32)
void openReleasePage() {
  std::string url;
  {
    const std::lock_guard<std::mutex> lock(g_mutex);
    url = g_releasePageUrl;
  }
  if (!url.empty()) {
    ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
  }
}
#endif

#endif  // AVIONICS_HAS_CURL

}  // namespace

void startUpdateCheckOnLaunch() {
  if (std::getenv("AVIONICS_SKIP_UPDATE_CHECK") != nullptr) return;

#if defined(AVIONICS_HAS_CURL)
  checkForUpdatesAsync([](UpdateInfo info) {
    if (!info.newerAvailable) return;
    {
      const std::lock_guard<std::mutex> lock(g_mutex);
      g_latestVersion = info.latestVersion;
      g_archiveUrl = info.pluginArchiveUrl;
      g_checksumsUrl = info.checksumsUrl;
      g_releasePageUrl = info.releasePageUrl;
    }
    // Surface it in the PFD Alerts window (flashing "Messages" softkey) and the
    // log, so it is visible in the sim rather than only in Log.txt.
    setUpdateAdvisory(updateAdvisoryText(info.latestVersion));
    g_updateAvailable.store(true);

    char line[512];
    std::snprintf(line, sizeof(line),
                  "G1000 NXi: update available %s (running %s). Download: %s\n",
                  info.latestVersion.c_str(), kVersion,
                  info.releasePageUrl.c_str());
    XPLMDebugString(line);
  });
#endif
}

#if defined(AVIONICS_HAS_CURL)

bool updateAvailable() { return g_updateAvailable.load(); }

std::string availableUpdateVersion() {
  const std::lock_guard<std::mutex> lock(g_mutex);
  return g_latestVersion;
}

void beginPluginSelfUpdate() {
  if (!g_updateAvailable.load()) return;

#if defined(_WIN32)
  // A loaded .xpl (DLL) cannot be replaced in place while X-Plane has it
  // mapped, so an in-sim hot-reload is not possible on Windows. Open the
  // download page instead; the installer applies on the next X-Plane launch.
  openReleasePage();
  logLine("opening download page (Windows applies the update on next launch).");
#else
  std::string archiveUrl;
  std::string checksumsUrl;
  {
    const std::lock_guard<std::mutex> lock(g_mutex);
    archiveUrl = g_archiveUrl;
    checksumsUrl = g_checksumsUrl;
  }
  // Without a verifiable plugin archive we cannot self-update.
  if (archiveUrl.empty() || checksumsUrl.empty()) {
    logLine("self-update unavailable (no plugin archive asset); see release "
            "page.");
    return;
  }

  // Only start once: claim the Idle -> Downloading transition.
  SelfUpdateState expected = SelfUpdateState::Idle;
  if (!g_state.compare_exchange_strong(expected, SelfUpdateState::Downloading)) {
    return;  // already downloading / applied
  }

  // Resolve install paths on the main thread (XPLM API), then hand the file
  // work to a background thread.
  const fs::path xpl = thisPluginXplPath();
  if (xpl.empty()) {
    g_state.store(SelfUpdateState::Failed);
    return;
  }
  // .../plugins/xplane-avionics/<platform>/xplane-avionics.xpl -> .../plugins
  const fs::path pluginsDir = xpl.parent_path().parent_path().parent_path();

  setUpdateAdvisory(kAdvisoryDownloading);
  logLine("downloading update...");
  std::thread(&runSelfUpdate, archiveUrl, checksumsUrl, pluginsDir).detach();
#endif  // _WIN32
}

void pumpPluginSelfUpdate() {
  switch (g_state.load()) {
    case SelfUpdateState::ReadyToReload:
      g_state.store(SelfUpdateState::Idle);
      logLine("update downloaded and verified; reloading plugins.");
      XPLMReloadPlugins();
      break;
    case SelfUpdateState::ReadyAfterRestart:
      g_state.store(SelfUpdateState::Idle);
      setUpdateAdvisory(kAdvisoryRestart);
#if defined(_WIN32)
      openReleasePage();
#endif
      logLine("update downloaded; restart X-Plane (or run the installer) to "
              "finish.");
      break;
    case SelfUpdateState::Failed:
      g_state.store(SelfUpdateState::Idle);
      setUpdateAdvisory(kAdvisoryFailed);
      logLine("update download/verify failed; left current install in place.");
      break;
    default:
      break;
  }
}

#else  // !AVIONICS_HAS_CURL

bool updateAvailable() { return false; }
std::string availableUpdateVersion() { return std::string(); }
void beginPluginSelfUpdate() {}
void pumpPluginSelfUpdate() {}

#endif  // AVIONICS_HAS_CURL

}  // namespace avionics
