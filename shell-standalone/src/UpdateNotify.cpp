#include "UpdateNotify.h"

#include <cstdio>
#include <cstdlib>
#include <string>

#include "avionics/UpdateChecker.h"
#include "avionics/Version.h"

#if defined(__APPLE__)
#include "MacMenu.h"
#elif defined(_WIN32)
#include <windows.h>
#endif

namespace avionics {
namespace {

void showNotice(const UpdateInfo& info) {
  char message[512];
  std::snprintf(message, sizeof(message),
                "A newer G1000 NXi release is available.\n\n"
                "Installed: %s\n"
                "Latest: %s\n\n"
                "Open the release page in your browser to download it.",
                kVersion, info.latestVersion.c_str());

  std::fprintf(stderr, "G1000 NXi update: %s -> %s (%s)\n", kVersion,
               info.latestVersion.c_str(), info.releasePageUrl.c_str());

#if defined(__APPLE__)
  ShowUpdateAvailableAlert(kVersion, info.latestVersion.c_str(),
                           info.releasePageUrl.c_str());
#elif defined(_WIN32)
  MessageBoxA(nullptr, message, "G1000 NXi — Update Available",
              MB_OK | MB_ICONINFORMATION);
#else
  (void)message;
#endif
}

}  // namespace

void startUpdateCheckOnLaunch() {
  if (std::getenv("AVIONICS_SKIP_UPDATE_CHECK") != nullptr) return;

#if defined(AVIONICS_HAS_CURL)
  checkForUpdatesAsync([](UpdateInfo info) {
    if (!info.newerAvailable) return;
    showNotice(info);
  });
#endif
}

}  // namespace avionics
