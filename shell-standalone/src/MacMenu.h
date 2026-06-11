#pragma once

// Native macOS menu-bar integration for the standalone shell. The standalone
// app already gets a Cocoa menu bar from GLFW; this adds a "Data Source" menu
// for switching between the mock feed and the live X-Plane connection and a
// "View" menu for toggling the hardware bezel strips.
//
// These are declared unconditionally but only implemented on macOS (MacMenu.mm
// is compiled only on Apple), so callers guard the calls with __APPLE__.

namespace avionics {

// Which feed (and, for the mock feed, which sub-mode) the user picked from the
// Data Source menu.
enum class DataSourceSelection {
  MockFlying,  // animated mock feed flying the demo route
  MockGround,  // mock feed parked on KFMY runway 31 with the engine idling
  XPlane,      // live X-Plane connection
};

// Invoked on the main thread when the user picks a data source from the menu.
using DataSourceMenuCallback = void (*)(void* context,
                                        DataSourceSelection selection);

// Invoked on the main thread when the user flips the "Simulate Turbulence"
// item. enabled is the item's new (post-toggle) state.
using TurbulenceMenuToggleCallback = void (*)(void* context, bool enabled);

// Adds the "Data Source" menu to the application menu bar. initialSelection
// sets which feed item starts checked; initiallyTurbulent sets the "Simulate
// Turbulence" checkmark. sourceCallback fires when the user changes the feed;
// turbulenceCallback fires when the turbulence toggle flips (it only affects
// the mock feed).
void InstallDataSourceMenu(DataSourceSelection initialSelection,
                           bool initiallyTurbulent,
                           DataSourceMenuCallback sourceCallback,
                           TurbulenceMenuToggleCallback turbulenceCallback,
                           void* context);

// Updates the menu checkmarks to reflect the current selection (e.g. after the
// source was toggled with the keyboard rather than the menu).
void SetDataSourceMenuSelection(DataSourceSelection selection);

// Invoked on the main thread when the user flips one of the checkable View
// menu items. enabled is the item's new (post-toggle) state.
using ViewMenuToggleCallback = void (*)(void* context, bool enabled);

// Initial states and toggle callbacks for the "View" menu items:
//   - "Show Bezel Keys": the hardware bezel strips around the screen.
//   - "Show Window Title Bar": the OS window chrome (title bar with its
//     close / minimize / maximize controls).
//   - "Always on Top": keep the windows floating above other windows.
//   - "Remember Window Position": capture positions at exit, restore at launch.
struct ViewMenuConfig {
  bool showBezel = true;
  bool showWindowChrome = true;
  bool alwaysOnTop = false;
  bool rememberWindowPos = false;
  ViewMenuToggleCallback onToggleBezel = nullptr;
  ViewMenuToggleCallback onToggleWindowChrome = nullptr;
  ViewMenuToggleCallback onToggleAlwaysOnTop = nullptr;
  ViewMenuToggleCallback onToggleRememberWindowPos = nullptr;
  void* context = nullptr;
};

// Adds the "View" menu with its checkable items to the application menu bar.
void InstallViewMenu(const ViewMenuConfig& config);

// Updates the "Show Bezel Keys" checkmark to reflect the current state.
void SetBezelVisibilityMenuSelection(bool showBezel);

}  // namespace avionics
