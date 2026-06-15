#include "UpdateNotify.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "avionics/UpdateChecker.h"
#include "avionics/Version.h"

#if defined(AVIONICS_HAS_CURL)
#include <curl/curl.h>

#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

#include "Sha256.h"

#if defined(__APPLE__)
#include "MacMenu.h"
#include <mach-o/dyld.h>
#endif

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#else
#include <climits>
#include <cstdint>
#include <unistd.h>
#include <sys/stat.h>
#endif
#endif  // AVIONICS_HAS_CURL

namespace avionics {
namespace {

#if defined(AVIONICS_HAS_CURL)

// File name (last path component) of a download URL.
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
  std::istringstream in(sums);
  std::string line;
  while (std::getline(in, line)) {
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

std::size_t writeToFile(char* ptr, std::size_t size, std::size_t nmemb,
                        void* userData) {
  return std::fwrite(ptr, 1, size * nmemb, static_cast<FILE*>(userData));
}

std::size_t writeToString(char* ptr, std::size_t size, std::size_t nmemb,
                          void* userData) {
  static_cast<std::string*>(userData)->append(ptr, size * nmemb);
  return size * nmemb;
}

// Runs an HTTPS GET writing the body through `writeFn`/`writeData`. Returns
// true only on a 200 response.
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

bool downloadToString(const std::string& url, std::string& out) {
  return curlFetch(url, writeToString, &out);
}

// Downloads `url` to `localPath`, verifies its SHA-256 against the entry for
// its file name in the release's SHA256SUMS, and deletes it on any mismatch.
// `localPath` is narrow UTF-8 on POSIX and used to open an ifstream for hashing
// (the temp paths we build are ASCII). Returns true only when verified.
bool downloadAndVerify(const UpdateInfo& info, const std::string& localPath) {
  FILE* file = std::fopen(localPath.c_str(), "wb");
  if (file == nullptr) return false;
  const bool got = curlFetch(info.installerUrl, writeToFile, file);
  std::fclose(file);
  if (!got) {
    std::remove(localPath.c_str());
    return false;
  }

  std::string sums;
  if (!downloadToString(info.checksumsUrl, sums)) {
    std::remove(localPath.c_str());
    return false;
  }
  const std::string expected = expectedHashFor(sums, baseName(info.installerUrl));

  std::ifstream in(localPath, std::ios::binary);
  const std::string actual = in ? sha256HexOfStream(in) : std::string();
  if (expected.empty() || actual.empty() ||
      !equalsIgnoreCase(expected, actual)) {
    std::remove(localPath.c_str());
    return false;
  }
  return true;
}

#if !defined(_WIN32)
// Single-quotes a string for safe embedding in a /bin/sh script (handles paths
// with spaces, e.g. "/Applications/G1000 NXi.app").
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

std::string tempDir() {
  const char* t = std::getenv("TMPDIR");
  std::string dir = (t != nullptr && *t != '\0') ? t : "/tmp";
  if (dir.back() != '/') dir += '/';
  return dir;
}

// Path of the running executable (resolves symlinks), or empty on failure.
std::string currentExePath() {
#if defined(__APPLE__)
  std::uint32_t size = 0;
  _NSGetExecutablePath(nullptr, &size);
  std::string buf(size, '\0');
  if (_NSGetExecutablePath(buf.data(), &size) != 0) return std::string();
  char real[PATH_MAX] = {};
  if (realpath(buf.c_str(), real) == nullptr) return std::string();
  return std::string(real);
#else
  char real[PATH_MAX] = {};
  const ssize_t n = readlink("/proc/self/exe", real, sizeof(real) - 1);
  if (n <= 0) return std::string();
  real[n] = '\0';
  return std::string(real);
#endif
}

// Writes `script` to a private temp file (mode 0700) and returns its path.
std::string writeTempScript(const std::string& script) {
  std::string path = tempDir() + "g1000-update-XXXXXX";
  std::vector<char> buf(path.begin(), path.end());
  buf.push_back('\0');
  const int fd = mkstemp(buf.data());
  if (fd < 0) return std::string();
  const std::string outPath(buf.data());
  const ssize_t written =
      ::write(fd, script.data(), static_cast<std::size_t>(script.size()));
  ::close(fd);
  if (written != static_cast<ssize_t>(script.size())) {
    std::remove(outPath.c_str());
    return std::string();
  }
  ::chmod(outPath.c_str(), 0700);
  return outPath;
}

// Launches the helper script detached (own session) so it outlives this process
// and can replace our files once we exit. The script waits for our PID itself.
bool spawnDetached(const std::string& scriptPath) {
  const pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    setsid();
    execl("/bin/sh", "sh", scriptPath.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  return true;
}

void posixOpenUrl(const std::string& url) {
  if (url.empty()) return;
#if defined(__APPLE__)
  const std::string cmd = "open " + shq(url) + " >/dev/null 2>&1 &";
#else
  const std::string cmd = "xdg-open " + shq(url) + " >/dev/null 2>&1 &";
#endif
  std::system(cmd.c_str());
}
#endif  // !_WIN32

#if defined(_WIN32)

std::wstring widen(const std::string& s) {
  if (s.empty()) return std::wstring();
  const int len = MultiByteToWideChar(CP_UTF8, 0, s.c_str(),
                                      static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<std::size_t>(len), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()),
                      w.data(), len);
  return w;
}

void openUrl(const std::string& url) {
  if (url.empty()) return;
  ShellExecuteW(nullptr, L"open", widen(url).c_str(), nullptr, nullptr,
                SW_SHOWNORMAL);
}

std::wstring currentAppDir() {
  std::wstring buf(MAX_PATH, L'\0');
  const DWORD len =
      GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
  if (len == 0) return std::wstring();
  buf.resize(len);
  const std::size_t slash = buf.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : buf.substr(0, slash);
}

// Downloads + verifies the installer, then launches it silently to update only
// the standalone in place and relaunch it. Terminates the process on success
// (so the running .exe unlocks for the installer); returns false on failure.
bool performWindowsUpdate(const UpdateInfo& info) {
  if (info.installerUrl.empty() || info.checksumsUrl.empty()) return false;
  const std::string fileName = baseName(info.installerUrl);
  if (fileName.empty()) return false;

  wchar_t tempBuf[MAX_PATH] = {};
  const DWORD n = GetTempPathW(MAX_PATH, tempBuf);
  if (n == 0) return false;
  // Build a narrow path for downloadAndVerify (temp dir + ASCII asset name).
  char narrowTemp[MAX_PATH * 2] = {};
  WideCharToMultiByte(CP_UTF8, 0, tempBuf, -1, narrowTemp, sizeof(narrowTemp),
                      nullptr, nullptr);
  const std::string installerPath = std::string(narrowTemp) + fileName;
  if (!downloadAndVerify(info, installerPath)) return false;

  // /COMPONENTS limits the silent reinstall to the standalone (the running
  // app), leaving the X-Plane plugin untouched. /RUNAPP=1 relaunches us.
  std::wstring params =
      L"/VERYSILENT /SUPPRESSMSGBOXES /NORESTART /RUNAPP=1 "
      L"/COMPONENTS=\"standalone\"";
  const std::wstring appDir = currentAppDir();
  if (!appDir.empty()) params += L" /DIR=\"" + appDir + L"\"";

  const HINSTANCE rc =
      ShellExecuteW(nullptr, L"open", widen(installerPath).c_str(),
                    params.c_str(), nullptr, SW_SHOWNORMAL);
  if (reinterpret_cast<INT_PTR>(rc) <= 32) {
    std::remove(installerPath.c_str());
    return false;
  }
  std::fflush(nullptr);
  std::_Exit(0);
}

#elif defined(__APPLE__)

// Replaces the installed .app bundle with the one inside the downloaded DMG,
// clears the quarantine flag (the running app is already trusted), and relaunches.
// Terminates the process on success; returns false when not applicable.
bool performMacUpdate(const UpdateInfo& info) {
  if (info.installerUrl.empty() || info.checksumsUrl.empty()) return false;
  const std::string exe = currentExePath();
  const std::string marker = "/Contents/MacOS/";
  const std::size_t pos = exe.rfind(marker);
  if (pos == std::string::npos) return false;  // not running from a .app bundle
  const std::string bundle = exe.substr(0, pos);
  if (bundle.size() < 4 || bundle.substr(bundle.size() - 4) != ".app") {
    return false;
  }

  const std::string dmgPath = tempDir() + baseName(info.installerUrl);
  if (!downloadAndVerify(info, dmgPath)) return false;

  std::ostringstream s;
  s << "#!/bin/sh\n"
    << "while kill -0 " << static_cast<long>(getpid()) << " 2>/dev/null; do "
       "sleep 0.2; done\n"
    << "MNT=$(mktemp -d /tmp/g1000mnt.XXXXXX) || exit 1\n"
    << "if hdiutil attach -nobrowse -noverify -mountpoint \"$MNT\" "
    << shq(dmgPath) << " >/dev/null 2>&1; then\n"
    << "  if [ -d \"$MNT/G1000 NXi.app\" ]; then\n"
    << "    /usr/bin/ditto \"$MNT/G1000 NXi.app\" " << shq(bundle) << " && \\\n"
    << "      /usr/bin/xattr -dr com.apple.quarantine " << shq(bundle)
    << " 2>/dev/null\n"
    << "  fi\n"
    << "  hdiutil detach \"$MNT\" >/dev/null 2>&1\n"
    << "fi\n"
    << "rm -f " << shq(dmgPath) << "\n"
    << "open " << shq(bundle) << "\n";

  const std::string script = writeTempScript(s.str());
  if (script.empty() || !spawnDetached(script)) {
    std::remove(dmgPath.c_str());
    return false;
  }
  std::fflush(nullptr);
  std::_Exit(0);
}

#else  // Linux

// Extracts the downloaded tarball and copies its standalone subtree over the
// install directory, then relaunches. Terminates the process on success;
// returns false when the install dir is not writable or the download fails.
bool performLinuxUpdate(const UpdateInfo& info) {
  if (info.installerUrl.empty() || info.checksumsUrl.empty()) return false;
  const std::string exe = currentExePath();
  if (exe.empty()) return false;
  const std::size_t slash = exe.find_last_of('/');
  if (slash == std::string::npos) return false;
  const std::string installDir = exe.substr(0, slash);
  if (access(installDir.c_str(), W_OK) != 0) return false;  // not user-writable

  const std::string tarPath = tempDir() + baseName(info.installerUrl);
  if (!downloadAndVerify(info, tarPath)) return false;

  // The release tarball extracts to a top-level "installer/" dir containing
  // standalone/, plugin/, install.sh (see installer/linux/build_installer.sh);
  // we refresh only the standalone subtree.
  std::ostringstream s;
  s << "#!/bin/sh\n"
    << "while kill -0 " << static_cast<long>(getpid()) << " 2>/dev/null; do "
       "sleep 0.2; done\n"
    << "TMP=$(mktemp -d) || exit 1\n"
    << "if tar -xzf " << shq(tarPath) << " -C \"$TMP\"; then\n"
    << "  if [ -d \"$TMP/installer/standalone\" ]; then\n"
    << "    cp -a \"$TMP/installer/standalone/.\" " << shq(installDir) << "/\n"
    << "    chmod +x " << shq(installDir + "/avionics-standalone") << "\n"
    << "  fi\n"
    << "fi\n"
    << "rm -rf \"$TMP\"\n"
    << "rm -f " << shq(tarPath) << "\n"
    << "setsid " << shq(installDir + "/avionics-standalone")
    << " >/dev/null 2>&1 &\n";

  const std::string script = writeTempScript(s.str());
  if (script.empty() || !spawnDetached(script)) {
    std::remove(tarPath.c_str());
    return false;
  }
  std::fflush(nullptr);
  std::_Exit(0);
}

// Graphical yes/no prompt via the desktop's zenity or kdialog, since the shell
// has no GUI toolkit. Returns false when neither tool is available (the stderr
// notice then stands in).
bool linuxGuiConfirm(const std::string& body) {
  if (std::system("command -v zenity >/dev/null 2>&1") == 0) {
    const std::string cmd =
        "zenity --question --title='G1000 NXi Update' --text=" + shq(body) +
        " >/dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
  }
  if (std::system("command -v kdialog >/dev/null 2>&1") == 0) {
    const std::string cmd =
        "kdialog --title 'G1000 NXi Update' --yesno " + shq(body) +
        " >/dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
  }
  return false;
}

#endif  // platform

void showNotice(const UpdateInfo& info) {
  std::fprintf(stderr, "G1000 NXi update: %s -> %s (%s)\n", kVersion,
               info.latestVersion.c_str(), info.releasePageUrl.c_str());
  const bool canSelfUpdate =
      !info.installerUrl.empty() && !info.checksumsUrl.empty();

#if defined(_WIN32)
  const std::wstring header =
      L"A newer G1000 NXi release is available.\n\nInstalled: " +
      widen(kVersion) + L"\nLatest: " + widen(info.latestVersion) + L"\n\n";
  if (canSelfUpdate) {
    const int choice = MessageBoxW(
        nullptr,
        (header + L"Download and install it now? The app will update itself "
                  L"and restart.")
            .c_str(),
        L"G1000 NXi \u2014 Update Available", MB_YESNO | MB_ICONINFORMATION);
    if (choice != IDYES) return;
    if (performWindowsUpdate(info)) return;  // never returns on success
    MessageBoxW(nullptr,
                L"The update could not be downloaded or verified.\nOpening the "
                L"download page instead.",
                L"G1000 NXi \u2014 Update", MB_OK | MB_ICONWARNING);
    openUrl(info.releasePageUrl);
    return;
  }
  const int choice = MessageBoxW(
      nullptr, (header + L"Open the download page in your browser?").c_str(),
      L"G1000 NXi \u2014 Update Available", MB_YESNO | MB_ICONINFORMATION);
  if (choice == IDYES) openUrl(info.releasePageUrl);

#elif defined(__APPLE__)
  if (canSelfUpdate) {
    const std::string releaseUrl = info.releasePageUrl;
    const UpdateInfo captured = info;
    ShowUpdateAvailableAlert(kVersion, info.latestVersion.c_str(),
                             "Update & Restart", [captured, releaseUrl] {
                               if (!performMacUpdate(captured)) {
                                 posixOpenUrl(releaseUrl);
                               }
                             });
    return;
  }
  const std::string releaseUrl = info.releasePageUrl;
  ShowUpdateAvailableAlert(kVersion, info.latestVersion.c_str(),
                           "Open Download Page",
                           [releaseUrl] { posixOpenUrl(releaseUrl); });

#else  // Linux
  std::string body = "A newer G1000 NXi release is available.\n\nInstalled: ";
  body += kVersion;
  body += "\nLatest: ";
  body += info.latestVersion;
  if (canSelfUpdate) {
    body += "\n\nDownload and install it now? The app will restart.";
    if (linuxGuiConfirm(body)) {
      if (performLinuxUpdate(info)) return;  // never returns on success
      posixOpenUrl(info.releasePageUrl);
    }
    return;
  }
  body += "\n\nOpen the download page in your browser?";
  if (linuxGuiConfirm(body)) posixOpenUrl(info.releasePageUrl);
#endif
}

#endif  // AVIONICS_HAS_CURL

}  // namespace

void startUpdateCheckOnLaunch() {
  if (std::getenv("AVIONICS_SKIP_UPDATE_CHECK") != nullptr) return;

#if defined(AVIONICS_HAS_CURL)
  checkForUpdatesAsync([](UpdateInfo info) {
    if (!info.newerAvailable) return;
    // Surface the notice in the PFD Alerts window (flashing "Messages"
    // softkey), in addition to the actionable self-update dialog.
    setUpdateAdvisory(updateAdvisoryText(info.latestVersion));
    showNotice(info);
  });
#endif
}

}  // namespace avionics
