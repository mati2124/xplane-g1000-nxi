#pragma once

namespace avionics {

// Background GitHub release check; logs to X-Plane's Log.txt when an update
// exists. No-op when AVIONICS_SKIP_UPDATE_CHECK is set.
void startUpdateCheckOnLaunch();

}  // namespace avionics
