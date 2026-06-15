#include "avionics/ChecklistStore.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

#include "avionics/AircraftProfile.h"
#include "avionics/AssetPaths.h"

#ifndef AVIONICS_CORE_ASSET_DIR
#define AVIONICS_CORE_ASSET_DIR ""
#endif

namespace avionics {
namespace {

namespace fs = std::filesystem;

std::int64_t fileMtimeNs(const std::string& path) {
  std::error_code ec;
  const fs::file_time_type t = fs::last_write_time(path, ec);
  if (ec) return 0;
  return static_cast<std::int64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch())
          .count());
}

bool readFile(const std::string& path, std::string& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in.good()) return false;
  std::ostringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

std::string devFallback(const std::string& assetRel) {
  const std::string base = AVIONICS_CORE_ASSET_DIR;
  if (base.empty()) return std::string();
  return base + "/" + assetRel;
}

}  // namespace

ChecklistStore::ChecklistStore(std::string selector)
    : selector_(std::move(selector)) {
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

ChecklistStore::~ChecklistStore() {
  if (thread_.joinable()) thread_.join();
}

std::string ChecklistStore::resolvePath() const {
  const AircraftProfile profile =
      resolveAircraftProfile(aircraftIcao_, aircraftAcfRelativePath_);
  const std::string bundled =
      assets::resolve(profile.checklistAsset, devFallback(profile.checklistAsset));
  // User-droppable, ICAO-keyed override (assets/checklists/<icao>.checklist):
  // lets a user add support for any aircraft by dropping a file into the
  // plugin's assets folder, no rebuild. Empty/absent is skipped below.
  const std::string typeAsset = typeKeyedChecklistAsset(aircraftIcao_);
  const std::string typeKeyed =
      typeAsset.empty() ? std::string()
                        : assets::resolve(typeAsset, devFallback(typeAsset));
  const std::vector<std::string> candidates = candidateChecklistPaths(
      selector_, aircraftAcfRelativePath_, typeKeyed, bundled);
  if (candidates.empty()) return std::string();
  return candidates.front();
}

void ChecklistStore::resolveAndLoad() {
  const std::string path = resolvePath();
  if (path.empty()) {
    if (!selector_.empty()) {
      std::fprintf(stderr, "Checklists: file not found: %s\n",
                   selector_.c_str());
    }
    loaded_.store(true, std::memory_order_release);
    return;
  }

  std::string text;
  if (readFile(path, text)) {
    checklists_ = parseChecklistText(text);
    sourcePath_ = path;
    lastMtimeNs_ = fileMtimeNs(path);
    std::fprintf(stderr, "Checklists: loaded %d checklists from %s\n",
                 checklists_.totalChecklists(), sourcePath_.c_str());
  } else {
    std::fprintf(stderr, "Checklists: failed to read %s\n", path.c_str());
  }

  loaded_.store(true, std::memory_order_release);
}

void ChecklistStore::loadOnBackgroundThread() { resolveAndLoad(); }

void ChecklistStore::setAircraftIdentity(const std::string& icaoType,
                                         const std::string& acfRelativePath) {
  if (icaoType == aircraftIcao_ && acfRelativePath == aircraftAcfRelativePath_) {
    return;
  }
  aircraftIcao_ = icaoType;
  aircraftAcfRelativePath_ = acfRelativePath;
  if (thread_.joinable()) thread_.join();
  loaded_.store(false, std::memory_order_release);
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

void ChecklistStore::refreshIfChanged() {
  if (sourcePath_.empty()) return;

  const std::int64_t mtime = fileMtimeNs(sourcePath_);
  if (mtime == 0 || mtime == lastMtimeNs_) return;

  std::string text;
  if (!readFile(sourcePath_, text)) return;
  checklists_ = parseChecklistText(text);
  lastMtimeNs_ = mtime;
  std::fprintf(stderr, "Checklists: reloaded %d checklists from %s\n",
               checklists_.totalChecklists(), sourcePath_.c_str());
}

}  // namespace avionics
