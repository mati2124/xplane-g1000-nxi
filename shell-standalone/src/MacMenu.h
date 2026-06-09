#pragma once

// Native macOS menu-bar integration for the standalone shell. The standalone
// app already gets a Cocoa menu bar from GLFW; this adds a "Data Source" menu
// for switching between the mock feed and the live X-Plane connection.
//
// These are declared unconditionally but only implemented on macOS (MacMenu.mm
// is compiled only on Apple), so callers guard the calls with __APPLE__.

namespace avionics {

// Invoked on the main thread when the user picks a data source from the menu.
// useXPlane == true selects the live X-Plane connection, false selects mock.
using DataSourceMenuCallback = void (*)(void* context, bool useXPlane);

// Adds the "Data Source" menu to the application menu bar. initiallyXPlane sets
// which item starts checked. The callback fires when the user changes the
// selection.
void InstallDataSourceMenu(bool initiallyXPlane, DataSourceMenuCallback callback,
                           void* context);

// Updates the menu checkmarks to reflect the current selection (e.g. after the
// source was toggled with the keyboard rather than the menu).
void SetDataSourceMenuSelection(bool useXPlane);

}  // namespace avionics
