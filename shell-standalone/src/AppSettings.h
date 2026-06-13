#pragma once

// User-facing preferences for the standalone shell that survive across runs.
//
// Stored as a tiny key=value text file under the platform's per-user config
// directory (Application Support on macOS, %APPDATA% on Windows, XDG config /
// ~/.config elsewhere). Persistence is platform-independent even though the
// menu that drives it is currently macOS-only.

#include <string>

#include "avionics/PersistentState.h"

namespace avionics {

struct AppSettings {
  // Whether the hardware bezel strips (right-hand key column + bottom softkey
  // row) are drawn around the avionics screen.
  bool showBezel = true;
  // SimBrief account Pilot ID (digits only; empty = not configured). Entered
  // on the MFD AUX - SIMBRIEF page and used to fetch the latest OFP.
  std::string simbriefPilotId;
  // Directory holding a copied X-Plane nav-data tree (laid out like an install
  // root), used when no local X-Plane install is present so the moving map
  // works on a display-only PC. Empty = discover a local install as usual.
  // The --nav-data-dir command-line flag overrides this when given.
  std::string navDataDir;
  // Whether the OS window chrome (the title bar with its close / minimize /
  // maximize controls) is drawn on the PFD and MFD windows.
  bool showWindowChrome = true;
  // Whether the PFD and MFD windows float above other windows (GLFW_FLOATING).
  bool alwaysOnTop = false;
  // Borderless full-screen mode: each display takes over a whole monitor with
  // no title bar, the 4:3 avionics image letterboxed to keep its proportions.
  // The monitor index is into GLFW's monitor list; -1 means "let the app pick"
  // (PFD on the first monitor, MFD on the second when one exists).
  bool pfdFullscreen = false;
  bool mfdFullscreen = false;
  int pfdMonitor = -1;
  int mfdMonitor = -1;
  // Last saved window positions (screen coordinates of the window's top-left
  // corner), only meaningful when hasWindowPos is true.
  bool hasWindowPos = false;
  int pfdWindowX = 0;
  int pfdWindowY = 0;
  int mfdWindowX = 0;
  int mfdWindowY = 0;
  // Durable PFD/MFD display preferences (the softkey-selectable options that
  // survive between flights, e.g. the PFD inset map on/off). Captured from the
  // engines and restored on the next launch.
  AvionicsPersistentState avionics;
  // True once a settings file has been read back, i.e. the user has made (and
  // we have persisted) an explicit choice before. Lets callers tell a real
  // saved preference apart from the defaults above.
  bool loaded = false;
};

// Reads the settings file, returning defaults (with loaded == false) when none
// exists yet or it cannot be parsed.
AppSettings LoadAppSettings();

// Writes the settings file, creating the config directory if needed. Failures
// are silently ignored (a missing preference is not worth interrupting flight).
void SaveAppSettings(const AppSettings& settings);

}  // namespace avionics
