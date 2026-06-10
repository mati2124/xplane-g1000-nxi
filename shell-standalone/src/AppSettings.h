#pragma once

// User-facing preferences for the standalone shell that survive across runs.
//
// Stored as a tiny key=value text file under the platform's per-user config
// directory (Application Support on macOS, %APPDATA% on Windows, XDG config /
// ~/.config elsewhere). Persistence is platform-independent even though the
// menu that drives it is currently macOS-only.

namespace avionics {

struct AppSettings {
  // Data feed the user last selected explicitly (mock vs. live X-Plane).
  bool useXPlane = false;
  // Whether the hardware bezel strips (right-hand key column + bottom softkey
  // row) are drawn around the avionics screen.
  bool showBezel = true;
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
