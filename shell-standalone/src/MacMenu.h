#pragma once

#include <functional>

#include "AppSettings.h"  // DebugDataSource

// Native macOS integration for the standalone shell: a Cocoa alert used by the
// update checker, plus the dev-only "Debug" menu for switching the data source
// and flipping the demo power switches.
//
// The Debug menu is only installed when the app is launched with --debug-menu
// (which the IDE launch configs/tasks pass but the installer never does), so it
// never appears in a shipped build.
//
// Declared unconditionally but only implemented on macOS (MacMenu.mm is compiled
// only on Apple), so callers guard the calls with __APPLE__.

namespace avionics {

// Fired on the main thread when the user picks a feed/state from the Debug menu.
using DebugSourceCallback = void (*)(void* context, DebugDataSource selection);

// Fired on the main thread when the user flips the Master or Avionics power
// item. The new (post-toggle) state of both switches is passed.
using DebugPowerCallback = void (*)(void* context, bool masterOn,
                                    bool avionicsOn);

// Fired on the main thread when the user flips the demo CAS messages item. The
// new (post-toggle) enabled state is passed.
using DebugCasCallback = void (*)(void* context, bool enabled);

// Initial states and callbacks for the Debug menu.
struct DebugMenuConfig {
  DebugDataSource source = DebugDataSource::XPlane;
  bool masterPowerOn = true;
  bool avionicsPowerOn = true;
  bool casMessagesOn = true;
  DebugSourceCallback onSource = nullptr;
  DebugPowerCallback onPower = nullptr;
  DebugCasCallback onCas = nullptr;
  void* context = nullptr;
};

// Adds the "Debug" menu (data-source / demo-state radio items and the
// Master/Avionics power toggles) to the macOS application menu bar.
void InstallDebugMenu(const DebugMenuConfig& config);

// Re-syncs the menu checkmarks after a change made elsewhere (e.g. the
// Ctrl+Shift+D keyboard toggle), so the menu always reflects the live state.
void SyncDebugMenuSource(DebugDataSource selection);
void SyncDebugMenuPower(bool masterOn, bool avionicsOn);
void SyncDebugMenuCas(bool enabled);

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
