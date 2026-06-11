#include "avionics/EisStore.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifndef AVIONICS_DEFAULT_EIS
#define AVIONICS_DEFAULT_EIS ""
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

}  // namespace

EisStore::EisStore(std::string selector) : selector_(std::move(selector)) {
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

EisStore::~EisStore() {
  if (thread_.joinable()) thread_.join();
}

std::string EisStore::resolvePath(const std::string& acfRelativePath) const {
  const std::vector<std::string> candidates = candidateEisPaths(
      selector_, acfRelativePath, AVIONICS_DEFAULT_EIS);
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
  if (acfRelativePath == aircraftAcfRelativePath_) return;
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
