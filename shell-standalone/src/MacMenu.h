#pragma once

// Native macOS integration for the standalone shell: a Cocoa alert used by the
// update checker. The app's menus are now drawn in-app (AppMenu) so they are
// identical on every platform; this file only retains the macOS-native alert.
//
// Declared unconditionally but only implemented on macOS (MacMenu.mm is compiled
// only on Apple), so callers guard the calls with __APPLE__.

namespace avionics {

// Native alert when a newer GitHub release is detected. May be called from a
// background thread; marshals to the AppKit main queue internally.
void ShowUpdateAvailableAlert(const char* currentVersion,
                              const char* latestVersion, const char* releaseUrl);

}  // namespace avionics
