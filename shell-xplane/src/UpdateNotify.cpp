#include "UpdateNotify.h"

#include <cstdio>
#include <cstdlib>

#include "XPLMUtilities.h"
#include "avionics/UpdateChecker.h"
#include "avionics/Version.h"

namespace avionics {

void startUpdateCheckOnLaunch() {
  if (std::getenv("AVIONICS_SKIP_UPDATE_CHECK") != nullptr) return;

#if defined(AVIONICS_HAS_CURL)
  checkForUpdatesAsync([](UpdateInfo info) {
    if (!info.newerAvailable) return;
    char line[512];
    std::snprintf(line, sizeof(line),
                  "G1000 NXi: update available %s (running %s). Download: %s\n",
                  info.latestVersion.c_str(), kVersion,
                  info.releasePageUrl.c_str());
    XPLMDebugString(line);
  });
#endif
}

}  // namespace avionics
