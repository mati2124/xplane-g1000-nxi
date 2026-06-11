#include "ChecklistStore.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

#include "avionics/AssetPaths.h"

#ifndef AVIONICS_DEFAULT_CHECKLIST
#define AVIONICS_DEFAULT_CHECKLIST ""
#endif

namespace avionics {
namespace {

namespace fs = std::filesystem;

std::string resolvePath(const std::string& selector) {
  std::error_code ec;
  if (!selector.empty()) {
    if (fs::is_regular_file(selector, ec)) return selector;
    return std::string();
  }
  // No selector: fall back to the bundled sample so the page is populated in
  // development. (The X-Plane plugin shell resolves a per-aircraft file.)
  const std::string sample =
      assets::resolve("checklists.txt", AVIONICS_DEFAULT_CHECKLIST);
  if (!sample.empty() && fs::is_regular_file(sample, ec)) return sample;
  return std::string();
}

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

ChecklistStore::ChecklistStore(std::string selector)
    : selector_(std::move(selector)) {
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

ChecklistStore::~ChecklistStore() {
  if (thread_.joinable()) thread_.join();
}

void ChecklistStore::loadOnBackgroundThread() {
  const std::string path = resolvePath(selector_);
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
