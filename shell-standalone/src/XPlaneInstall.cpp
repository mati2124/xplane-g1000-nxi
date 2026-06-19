#include "XPlaneInstall.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace avionics {
namespace xplane_install {
namespace {

char pathSep() {
#if defined(_WIN32)
  return '\\';
#else
  return '/';
#endif
}

std::string join(const std::string& dir, const std::string& leaf) {
  if (dir.empty()) return leaf;
  if (dir.back() == '/' || dir.back() == '\\') return dir + leaf;
  return dir + pathSep() + leaf;
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

bool dirExists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::is_directory(path, ec);
}

std::string installListDir() {
#if defined(_WIN32)
  const char* base = std::getenv("LOCALAPPDATA");
  return base ? std::string(base) + "\\" : std::string();
#elif defined(__APPLE__)
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/Library/Preferences/" : std::string();
#else
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + "/.x-plane/" : std::string();
#endif
}

std::string g_navDataRoot;

}  // namespace

void setNavDataRoot(const std::string& root) { g_navDataRoot = root; }

std::vector<std::string> readInstallRoots() {
  std::vector<std::string> roots;
  // An explicit --nav-data-dir wins: it is tried first, then real installs are
  // appended so a partial copied tree can still fall back to a local install.
  if (!g_navDataRoot.empty()) roots.push_back(g_navDataRoot);
  const std::string dir = installListDir();
  if (dir.empty()) return roots;
  for (const char* version : {"12", "11"}) {
    const std::string listPath =
        join(dir, std::string("x-plane_install_") + version + ".txt");
    std::ifstream in(listPath);
    if (!in.good()) continue;
    std::string line;
    while (std::getline(in, line)) {
      while (!line.empty() &&
             (line.back() == '\r' || line.back() == '\n' ||
              line.back() == ' ' || line.back() == '\t')) {
        line.pop_back();
      }
      if (!line.empty()) roots.push_back(line);
    }
  }
  return roots;
}

std::string navDataDirForRoot(const std::string& root) {
  const std::string candidates[] = {join(root, "Custom Data"),
                                    join(join(root, "Resources"),
                                         "default data")};
  for (const std::string& dir : candidates) {
    if (fileExists(join(dir, "earth_nav.dat")) &&
        fileExists(join(dir, "earth_fix.dat"))) {
      return dir;
    }
  }
  return std::string();
}

std::string earthNavDataDir() {
  for (const std::string& root : readInstallRoots()) {
    const std::string candidates[] = {
        join(join(root, "Global Scenery"),
             "X-Plane 12 Global Scenery/Earth nav data"),
        join(join(root, "Global Scenery"),
             "X-Plane 11 Global Scenery/Earth nav data"),
        join(join(root, "Global Scenery"), "Earth nav data"),
    };
    for (const std::string& dir : candidates) {
      if (dirExists(dir)) return dir;
    }
  }
  return std::string();
}

}  // namespace xplane_install
}  // namespace avionics
