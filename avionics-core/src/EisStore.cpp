#include "avionics/EisStore.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "avionics/AircraftProfile.h"
#include "avionics/AssetPaths.h"

#ifndef AVIONICS_DEFAULT_EIS
#define AVIONICS_DEFAULT_EIS ""
#endif

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

EisStore::EisStore(std::string selector) : selector_(std::move(selector)) {
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

EisStore::~EisStore() {
  if (thread_.joinable()) thread_.join();
}

std::string EisStore::resolvePath(const std::string& acfRelativePath) const {
  // The bundled fallback follows the detected aircraft profile (keyed off
  // acf_ICAO / path) so the engine page swaps with the airframe even when the
  // aircraft does not ship its own EIS file beside the .acf.
  const AircraftProfile profile =
      resolveAircraftProfile(aircraftIcao_, acfRelativePath);
  const std::string bundled =
      assets::resolve(profile.eisAsset, devFallback(profile.eisAsset));
  // User-droppable, ICAO-keyed override (assets/eis/<icao>.eis): lets a user add
  // support for any aircraft by dropping a file into the plugin's assets folder,
  // no rebuild. Empty/absent resolves to nothing and is skipped below.
  const std::string typeAsset = typeKeyedEisAsset(aircraftIcao_);
  const std::string typeKeyed =
      typeAsset.empty() ? std::string()
                        : assets::resolve(typeAsset, devFallback(typeAsset));
  const std::vector<std::string> candidates =
      candidateEisPaths(selector_, acfRelativePath, typeKeyed, bundled);
  if (candidates.empty()) return std::string();
  return candidates.front();
}

void EisStore::resolveAndLoad(const std::string& acfRelativePath) {
  const std::string path = resolvePath(acfRelativePath);
  if (path.empty()) {
    if (!selector_.empty()) {
      std::fprintf(stderr, "EIS: file not found: %s\n", selector_.c_str());
    }
    loaded_.store(true, std::memory_order_release);
    return;
  }

  std::string text;
  if (readFile(path, text)) {
    layout_ = parseEisText(text);
    sourcePath_ = path;
    lastMtimeNs_ = fileMtimeNs(path);
    std::fprintf(stderr, "EIS: loaded %zu gauge sections from %s\n",
                 layout_.sections.size(), sourcePath_.c_str());
  } else {
    std::fprintf(stderr, "EIS: failed to read %s\n", path.c_str());
  }

  loaded_.store(true, std::memory_order_release);
}

void EisStore::loadOnBackgroundThread() {
  resolveAndLoad(aircraftAcfRelativePath_);
}

void EisStore::setAircraftAcfRelativePath(const std::string& acfRelativePath) {
  setAircraftIdentity(aircraftIcao_, acfRelativePath);
}

void EisStore::setAircraftIdentity(const std::string& icaoType,
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

void EisStore::refreshIfChanged() {
  if (sourcePath_.empty()) return;

  const std::int64_t mtime = fileMtimeNs(sourcePath_);
  if (mtime == 0 || mtime == lastMtimeNs_) return;

  std::string text;
  if (!readFile(sourcePath_, text)) return;
  layout_ = parseEisText(text);
  lastMtimeNs_ = mtime;
  std::fprintf(stderr, "EIS: reloaded layout from %s\n", sourcePath_.c_str());
}

}  // namespace avionics
