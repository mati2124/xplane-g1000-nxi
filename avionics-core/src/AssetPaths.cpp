#include "avionics/AssetPaths.h"

#include <algorithm>
#include <filesystem>
#include <mutex>
#include <vector>

namespace avionics {
namespace assets {
namespace {

namespace fs = std::filesystem;

// Guards the search-dir list. addSearchDir runs at startup, but resolve() is
// also called from background loader threads (EIS/checklist/land data), so the
// list is read under the same lock.
std::mutex& mutex() {
  static std::mutex m;
  return m;
}

std::vector<std::string>& searchDirs() {
  static std::vector<std::string> dirs;
  return dirs;
}

}  // namespace

void addSearchDir(const std::string& dir) {
  if (dir.empty()) return;
  std::lock_guard<std::mutex> lock(mutex());
  std::vector<std::string>& dirs = searchDirs();
  if (std::find(dirs.begin(), dirs.end(), dir) == dirs.end()) {
    dirs.push_back(dir);
  }
}

std::string resolve(const std::string& relativePath,
                    const std::string& devFallback) {
  std::error_code ec;
  std::lock_guard<std::mutex> lock(mutex());
  for (const std::string& dir : searchDirs()) {
    const fs::path candidate = fs::path(dir) / relativePath;
    if (fs::is_regular_file(candidate, ec)) {
      return candidate.string();
    }
  }
  return devFallback;
}

}  // namespace assets
}  // namespace avionics
