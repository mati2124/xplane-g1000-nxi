#include "FmsPlanStore.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace avionics {
namespace {

namespace fs = std::filesystem;

constexpr const char* kFmsPlansRel = "Output/FMS plans";

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

std::vector<std::string> readInstallRoots() {
  std::vector<std::string> roots;
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

std::string fmsPlansDir() {
  for (const std::string& root : readInstallRoots()) {
    const std::string dir = join(root, kFmsPlansRel);
    std::error_code ec;
    if (fs::is_directory(dir, ec)) return dir;
  }
  return std::string();
}

bool hasFmsExtension(const std::string& name) {
  if (name.size() < 4) return false;
  const std::string ext = name.substr(name.size() - 4);
  std::string lower = ext;
  for (char& c : lower) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return lower == ".fms";
}

std::string newestFmsInDir(const std::string& dir) {
  std::string best;
  fs::file_time_type bestTime{};
  bool haveBest = false;
  std::error_code ec;
  for (const fs::directory_entry& ent :
       fs::directory_iterator(dir, fs::directory_options::skip_permission_denied,
                              ec)) {
    if (ec) break;
    if (!ent.is_regular_file(ec)) continue;
    if (!hasFmsExtension(ent.path().filename().string())) continue;
    const fs::file_time_type t = ent.last_write_time(ec);
    if (ec) continue;
    if (!haveBest || t > bestTime) {
      haveBest = true;
      bestTime = t;
      best = ent.path().string();
    }
  }
  return best;
}

std::string resolvePlanPath(const std::string& selector) {
  if (!selector.empty()) {
    std::error_code ec;
    if (fs::is_regular_file(selector, ec)) return selector;

    const std::string plansDir = fmsPlansDir();
    if (plansDir.empty()) return std::string();

    const std::string leaf =
        hasFmsExtension(selector) ? selector : selector + ".fms";
    const std::string candidate = join(plansDir, leaf);
    if (fs::is_regular_file(candidate, ec)) return candidate;
    return std::string();
  }

  const std::string plansDir = fmsPlansDir();
  if (plansDir.empty()) return std::string();
  return newestFmsInDir(plansDir);
}

std::int64_t fileMtimeNs(const std::string& path) {
  std::error_code ec;
  const fs::file_time_type t = fs::last_write_time(path, ec);
  if (ec) return 0;
  return static_cast<std::int64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch())
          .count());
}

const char* skipSpaces(const char* p) {
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

std::string readToken(const char*& p) {
  p = skipSpaces(p);
  const char* start = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  return std::string(start, static_cast<std::size_t>(p - start));
}

// v11 .fms: header line, count line, then "type id via alt lat lon" rows.
// Lat/lon are always the last two fields so minor format drift still parses.
bool parseFmsFile(const std::string& path, std::vector<MapLeg>& out) {
  std::ifstream in(path);
  if (!in.good()) return false;

  std::string line;
  if (!std::getline(in, line)) return false;  // version header
  if (!std::getline(in, line)) return false;  // waypoint count

  out.clear();
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    p = skipSpaces(p);
    if (*p == '\0') continue;

    char* end = nullptr;
    const long type = std::strtol(p, &end, 10);
    if (end == p) continue;
    if (type <= 0) continue;

    p = end;
    const std::string id = readToken(p);
    if (id.empty()) continue;
    (void)readToken(p);  // via
    (void)readToken(p);  // altitude ft

    p = skipSpaces(p);
    const double lat = std::strtod(p, &end);
    if (end == p) continue;
    p = end;
    const double lon = std::strtod(p, &end);
    if (end == p) continue;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) continue;

    out.push_back({lat, lon, id});
  }
  return !out.empty();
}

bool loadPlanFromDisk(const std::string& path, std::vector<MapLeg>& legs,
                      std::int64_t& mtimeNs) {
  if (!parseFmsFile(path, legs)) return false;
  mtimeNs = fileMtimeNs(path);
  return true;
}

}  // namespace

FmsPlanStore::FmsPlanStore(std::string planSelector)
    : planSelector_(std::move(planSelector)) {
  thread_ = std::thread([this] { loadOnBackgroundThread(); });
}

FmsPlanStore::~FmsPlanStore() {
  if (thread_.joinable()) thread_.join();
}

void FmsPlanStore::loadOnBackgroundThread() {
  const std::string path = resolvePlanPath(planSelector_);
  if (path.empty()) {
    loaded_.store(true, std::memory_order_release);
    return;
  }

  std::vector<MapLeg> legs;
  std::int64_t mtime = 0;
  if (loadPlanFromDisk(path, legs, mtime)) {
    flightPlan_ = std::move(legs);
    sourcePath_ = path;
    lastMtimeNs_ = mtime;
    std::fprintf(stderr, "FMS plan: loaded %zu legs from %s\n",
                 flightPlan_.size(), sourcePath_.c_str());
  } else {
    std::fprintf(stderr, "FMS plan: failed to parse %s\n", path.c_str());
  }

  loaded_.store(true, std::memory_order_release);
}

void FmsPlanStore::refreshIfChanged() {
  if (sourcePath_.empty()) return;

  std::int64_t mtime = fileMtimeNs(sourcePath_);
  if (mtime == 0 || mtime == lastMtimeNs_) return;

  std::vector<MapLeg> legs;
  if (!loadPlanFromDisk(sourcePath_, legs, mtime)) return;

  flightPlan_ = std::move(legs);
  lastMtimeNs_ = mtime;
  std::fprintf(stderr, "FMS plan: reloaded %zu legs from %s\n",
               flightPlan_.size(), sourcePath_.c_str());
}

}  // namespace avionics
