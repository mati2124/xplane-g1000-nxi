#include "AppSettings.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace avionics {
namespace {

constexpr const char* kAppDirName = "XPlaneAvionics";
constexpr const char* kSettingsFileName = "settings.txt";
constexpr const char* kKeyUseXPlane = "useXPlane";
constexpr const char* kKeyShowBezel = "showBezel";

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

}  // namespace

AppSettings LoadAppSettings() {
  AppSettings settings;
  const std::filesystem::path path = SettingsPath();
  if (path.empty()) return settings;

  std::ifstream in(path);
  if (!in.is_open()) return settings;

  std::string line;
  while (std::getline(in, line)) {
    const std::string::size_type eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    if (key == kKeyUseXPlane) {
      settings.useXPlane = ParseBool(value, settings.useXPlane);
    } else if (key == kKeyShowBezel) {
      settings.showBezel = ParseBool(value, settings.showBezel);
    }
  }
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
  out << kKeyUseXPlane << '=' << (settings.useXPlane ? '1' : '0') << '\n';
  out << kKeyShowBezel << '=' << (settings.showBezel ? '1' : '0') << '\n';
}

}  // namespace avionics
