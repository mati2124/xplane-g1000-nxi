#include "MetarFileStore.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <utility>

namespace avionics {
namespace {

// Re-scan for a newer file at most this often; X-Plane refreshes the real-
// weather METARs roughly every 15 minutes, so a slow poll is plenty.
constexpr double kPollIntervalSec = 60.0;

// True when `line` begins with a 4-character ICAO station id followed by a
// space (e.g. "KFMY 262112Z ..."), distinguishing a report line from the
// "YYYY/MM/DD HH:MM" timestamp line (whose 5th char is '/') or a blank line.
bool looksLikeReportLine(const std::string& line) {
  if (line.size() < 5 || line[4] != ' ') return false;
  for (int i = 0; i < 4; ++i) {
    const unsigned char c = static_cast<unsigned char>(line[i]);
    if (std::isalnum(c) == 0) return false;
  }
  return true;
}

std::string upper(std::string s) {
  for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

}  // namespace

MetarFileStore::~MetarFileStore() {
  if (thread_.joinable()) thread_.join();
}

void MetarFileStore::start(std::string xplaneRoot) {
  if (started_.exchange(true)) return;
  if (!xplaneRoot.empty() && xplaneRoot.back() != '/' &&
      xplaneRoot.back() != '\\') {
    xplaneRoot += '/';
  }
  weatherDir_ = xplaneRoot + "Output/real weather";
  reloadAsync();
}

void MetarFileStore::poll(double dtSeconds) {
  if (!started_.load() || weatherDir_.empty()) return;
  sincePollSeconds_ += dtSeconds;
  if (sincePollSeconds_ < kPollIntervalSec) return;
  sincePollSeconds_ = 0.0;
  reloadAsync();
}

void MetarFileStore::reloadAsync() {
  if (loading_.exchange(true)) return;  // a load is already in flight
  if (thread_.joinable()) thread_.join();
  thread_ = std::thread([this] {
    namespace fs = std::filesystem;

    // Newest metar-*.txt by filename: the zero-padded timestamp names sort
    // chronologically, so the lexicographic maximum is the latest download.
    std::error_code ec;
    std::string newest;
    for (fs::directory_iterator it(weatherDir_, ec), end; it != end;
         it.increment(ec)) {
      if (ec) break;
      if (!it->is_regular_file(ec)) continue;
      const std::string name = it->path().filename().string();
      if (name.rfind("metar-", 0) == 0 && name.size() > 4 &&
          name.compare(name.size() - 4, 4, ".txt") == 0) {
        const std::string full = it->path().string();
        if (full > newest) newest = full;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mu_);
      if (newest.empty() || newest == loadedFile_) {
        loading_.store(false);
        return;
      }
    }

    std::unordered_map<std::string, std::string> parsed;
    std::ifstream in(newest);
    std::string line;
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!looksLikeReportLine(line)) continue;
      parsed.emplace(upper(line.substr(0, 4)), line);
    }

    {
      std::lock_guard<std::mutex> lock(mu_);
      metarByIcao_ = std::move(parsed);
      loadedFile_ = newest;
    }
    loading_.store(false);
  });
}

std::optional<StationWeather> MetarFileStore::weatherForStation(
    const std::string& icao) const {
  if (icao.empty()) return std::nullopt;
  std::string raw;
  {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = metarByIcao_.find(upper(icao));
    if (it == metarByIcao_.end()) return std::nullopt;
    raw = it->second;
  }
  StationWeather wx;
  wx.icao = upper(icao);
  wx.rawMetar = raw;
  // X-Plane exposes no TAF data, so rawTaf intentionally stays empty (the WPT
  // Weather page then dashes the TAF box). The page decodes the METAR itself.
  return wx;
}

}  // namespace avionics
