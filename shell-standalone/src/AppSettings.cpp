#include "AppSettings.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace avionics {
namespace {

constexpr const char* kAppDirName = "XPlaneAvionics";
constexpr const char* kSettingsFileName = "settings.txt";
constexpr const char* kKeyShowBezel = "showBezel";
constexpr const char* kKeySimbriefPilotId = "simbriefPilotId";
constexpr const char* kKeyNavDataDir = "navDataDir";
constexpr const char* kKeyShowWindowChrome = "showWindowChrome";
constexpr const char* kKeyAlwaysOnTop = "alwaysOnTop";
constexpr const char* kKeyPfdFullscreen = "pfdFullscreen";
constexpr const char* kKeyMfdFullscreen = "mfdFullscreen";
constexpr const char* kKeyPfdMonitor = "pfdMonitor";
constexpr const char* kKeyMfdMonitor = "mfdMonitor";
constexpr const char* kKeyPfdWindowX = "pfdWindowX";
constexpr const char* kKeyPfdWindowY = "pfdWindowY";
constexpr const char* kKeyMfdWindowX = "mfdWindowX";
constexpr const char* kKeyMfdWindowY = "mfdWindowY";
constexpr const char* kKeyDebugDataSource = "debugDataSource";
constexpr const char* kKeyDemoMasterPower = "demoMasterPower";
constexpr const char* kKeyDemoAvionicsPower = "demoAvionicsPower";
constexpr const char* kKeyDemoCasMessages = "demoCasMessages";

// Stable text tokens for the persisted DebugDataSource selection.
constexpr const char* kSourceXPlane = "xplane";
constexpr const char* kSourceDemoGround = "demoGround";
constexpr const char* kSourceDemoFlying = "demoFlying";
constexpr const char* kSourceDemoTurbulence = "demoTurbulence";

const char* DebugDataSourceToken(DebugDataSource source) {
  switch (source) {
    case DebugDataSource::XPlane:
      return kSourceXPlane;
    case DebugDataSource::DemoGround:
      return kSourceDemoGround;
    case DebugDataSource::DemoFlying:
      return kSourceDemoFlying;
    case DebugDataSource::DemoTurbulence:
      return kSourceDemoTurbulence;
  }
  return kSourceXPlane;
}

DebugDataSource ParseDebugDataSource(const std::string& value,
                                     DebugDataSource fallback) {
  if (value == kSourceXPlane) return DebugDataSource::XPlane;
  if (value == kSourceDemoGround) return DebugDataSource::DemoGround;
  if (value == kSourceDemoFlying) return DebugDataSource::DemoFlying;
  if (value == kSourceDemoTurbulence) return DebugDataSource::DemoTurbulence;
  return fallback;
}

// Per-user config directory, following each platform's convention. Empty when
// the environment does not point anywhere sensible (settings then no-op).
std::filesystem::path ConfigDir() {
#if defined(_WIN32)
  if (const char* appData = std::getenv("APPDATA")) {
    return std::filesystem::path(appData) / kAppDirName;
  }
  return {};
#elif defined(__APPLE__)
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / "Library" / "Application Support" /
           kAppDirName;
  }
  return {};
#else
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) {
    return std::filesystem::path(xdg) / kAppDirName;
  }
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / ".config" / kAppDirName;
  }
  return {};
#endif
}

std::filesystem::path SettingsPath() {
  const std::filesystem::path dir = ConfigDir();
  return dir.empty() ? std::filesystem::path() : dir / kSettingsFileName;
}

bool ParseBool(const std::string& value, bool fallback) {
  if (value == "1" || value == "true") return true;
  if (value == "0" || value == "false") return false;
  return fallback;
}

// Parses a window coordinate (may legitimately be negative on multi-monitor
// setups). Marks hasWindowPos so a saved position is only restored when every
// coordinate key was actually present and numeric.
void ParseWindowCoord(const std::string& value, int& target, bool& valid) {
  try {
    target = std::stoi(value);
  } catch (...) {
    valid = false;
  }
}

}  // namespace

AppSettings LoadAppSettings() {
  AppSettings settings;
  const std::filesystem::path path = SettingsPath();
  if (path.empty()) return settings;

  std::ifstream in(path);
  if (!in.is_open()) return settings;

  std::string line;
  int windowCoords = 0;
  bool coordsValid = true;
  while (std::getline(in, line)) {
    const std::string::size_type eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    if (key == kKeyShowBezel) {
      settings.showBezel = ParseBool(value, settings.showBezel);
    } else if (key == kKeySimbriefPilotId) {
      settings.simbriefPilotId = value;
    } else if (key == kKeyNavDataDir) {
      settings.navDataDir = value;
    } else if (key == kKeyShowWindowChrome) {
      settings.showWindowChrome =
          ParseBool(value, settings.showWindowChrome);
    } else if (key == kKeyAlwaysOnTop) {
      settings.alwaysOnTop = ParseBool(value, settings.alwaysOnTop);
    } else if (key == kKeyPfdFullscreen) {
      settings.pfdFullscreen = ParseBool(value, settings.pfdFullscreen);
    } else if (key == kKeyMfdFullscreen) {
      settings.mfdFullscreen = ParseBool(value, settings.mfdFullscreen);
    } else if (key == kKeyPfdMonitor) {
      try {
        settings.pfdMonitor = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyMfdMonitor) {
      try {
        settings.mfdMonitor = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyPfdWindowX) {
      ParseWindowCoord(value, settings.pfdWindowX, coordsValid);
      ++windowCoords;
    } else if (key == kKeyPfdWindowY) {
      ParseWindowCoord(value, settings.pfdWindowY, coordsValid);
      ++windowCoords;
    } else if (key == kKeyMfdWindowX) {
      ParseWindowCoord(value, settings.mfdWindowX, coordsValid);
      ++windowCoords;
    } else if (key == kKeyMfdWindowY) {
      ParseWindowCoord(value, settings.mfdWindowY, coordsValid);
      ++windowCoords;
    } else if (key == kKeyDebugDataSource) {
      settings.debugDataSource =
          ParseDebugDataSource(value, settings.debugDataSource);
    } else if (key == kKeyDemoMasterPower) {
      settings.demoMasterPowerOn =
          ParseBool(value, settings.demoMasterPowerOn);
    } else if (key == kKeyDemoAvionicsPower) {
      settings.demoAvionicsPowerOn =
          ParseBool(value, settings.demoAvionicsPowerOn);
    } else if (key == kKeyDemoCasMessages) {
      settings.demoCasMessagesOn =
          ParseBool(value, settings.demoCasMessagesOn);
    } else {
      // Durable avionics display preferences are owned by the shared core, so
      // it parses its own keys; anything else is silently ignored.
      applyStateLine(key, value, settings.avionics);
    }
  }
  settings.hasWindowPos = coordsValid && windowCoords == 4;
  settings.loaded = true;
  return settings;
}

void SaveAppSettings(const AppSettings& settings) {
  const std::filesystem::path path = SettingsPath();
  if (path.empty()) return;

  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);

  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) return;
  out << kKeyShowBezel << '=' << (settings.showBezel ? '1' : '0') << '\n';
  out << kKeySimbriefPilotId << '=' << settings.simbriefPilotId << '\n';
  out << kKeyNavDataDir << '=' << settings.navDataDir << '\n';
  out << kKeyShowWindowChrome << '='
      << (settings.showWindowChrome ? '1' : '0') << '\n';
  out << kKeyAlwaysOnTop << '=' << (settings.alwaysOnTop ? '1' : '0') << '\n';
  out << kKeyPfdFullscreen << '=' << (settings.pfdFullscreen ? '1' : '0')
      << '\n';
  out << kKeyMfdFullscreen << '=' << (settings.mfdFullscreen ? '1' : '0')
      << '\n';
  out << kKeyPfdMonitor << '=' << settings.pfdMonitor << '\n';
  out << kKeyMfdMonitor << '=' << settings.mfdMonitor << '\n';
  out << kKeyDebugDataSource << '='
      << DebugDataSourceToken(settings.debugDataSource) << '\n';
  out << kKeyDemoMasterPower << '='
      << (settings.demoMasterPowerOn ? '1' : '0') << '\n';
  out << kKeyDemoAvionicsPower << '='
      << (settings.demoAvionicsPowerOn ? '1' : '0') << '\n';
  out << kKeyDemoCasMessages << '='
      << (settings.demoCasMessagesOn ? '1' : '0') << '\n';
  // Window coordinates are only written once a position has been captured, so
  // a fresh install never restores a bogus (0, 0) placement.
  if (settings.hasWindowPos) {
    out << kKeyPfdWindowX << '=' << settings.pfdWindowX << '\n';
    out << kKeyPfdWindowY << '=' << settings.pfdWindowY << '\n';
    out << kKeyMfdWindowX << '=' << settings.mfdWindowX << '\n';
    out << kKeyMfdWindowY << '=' << settings.mfdWindowY << '\n';
  }
  // Durable avionics display preferences, serialized by the shared core.
  std::string avionicsLines;
  appendStateLines(settings.avionics, avionicsLines);
  out << avionicsLines;
}

}  // namespace avionics
