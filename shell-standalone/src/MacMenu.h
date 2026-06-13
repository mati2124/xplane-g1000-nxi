#pragma once

#include <functional>

// Native macOS integration for the standalone shell: a Cocoa alert used by the
// update checker. The app's menus are now drawn in-app (AppMenu) so they are
// identical on every platform; this file only retains the macOS-native alert.
//
// Declared unconditionally but only implemented on macOS (MacMenu.mm is compiled
// only on Apple), so callers guard the calls with __APPLE__.

namespace avionics {

// Native alert when a newer GitHub release is detected. May be called from a
// background thread; marshals to the AppKit main queue internally to show the
// modal. When the user picks the primary button, `onAccept` is invoked once on
// a background queue (so the caller can download/apply without blocking the UI).
//
// `primaryButtonTitle` labels the action button ("Update & Restart" for an
// in-app self-update, or "Open Download Page" for the browser fallback).
void ShowUpdateAvailableAlert(const char* currentVersion,
                              const char* latestVersion,
                              const char* primaryButtonTitle,
                              std::function<void()> onAccept);

}  // namespace avionics
