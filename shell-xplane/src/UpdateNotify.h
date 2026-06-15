#pragma once

#include <string>

namespace avionics {

// Background GitHub release check, started at plugin load. When a newer release
// exists it logs to X-Plane's Log.txt and surfaces an advisory in the PFD
// Alerts window. No-op when AVIONICS_SKIP_UPDATE_CHECK is set.
void startUpdateCheckOnLaunch();

// True once the launch-time check has found a newer release than the running
// build (drives the "Install Update" menu item / command enable state).
bool updateAvailable();

// Latest available version string (e.g. "1.2.3"), empty until the check finds a
// newer release. Used to label the "Install Update" menu item.
std::string availableUpdateVersion();

// Begin downloading + verifying the plugin archive on a background thread, then
// (on the next main-thread pump) swap the files and hot-reload. Safe to call
// repeatedly; ignored while a download/apply is already in flight or when no
// update is available. Triggered by the menu item / bindable command.
void beginPluginSelfUpdate();

// Drive the self-updater's main-thread work: must be called from a flight-loop
// callback (the X-Plane SDK is main-thread only). Applies a staged update and
// reloads plugins once the background download has verified successfully. A
// cheap no-op until then.
void pumpPluginSelfUpdate();

}  // namespace avionics
