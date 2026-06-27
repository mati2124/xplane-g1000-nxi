#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "avionics/StationWeather.h"

namespace avionics {

// Serves station weather for the WPT - Weather Information page from X-Plane
// 12's downloaded real-weather METAR files.
//
// X-Plane 12 caches the METARs it fetches from Laminar's server (sourced from
// NOAA) under "<X-Plane>/Output/real weather/metar-YYYY-MM-DD-HH.MM.txt". Each
// record is a timestamp line ("2026/06/26 21:00") followed by the raw report on
// its own line ("KFMY 262112Z ... A3011 RMK ..."). There is no dataref carrying
// a station's raw METAR text, so the in-sim plugin reads the newest file from
// disk and indexes it by ICAO.
//
// IMPORTANT: X-Plane provides no TAF data at all (the real-weather folder holds
// only metar-*.txt and GRIB-* wind/turbulence files, and no dataref exposes a
// TAF). StationWeather.rawTaf is therefore always left empty, and the WPT
// Weather page honestly shows the TAF box dashed when running in the sim.
//
// File I/O runs on a background thread; weatherForStation() is a cheap locked
// map lookup safe to call from the render path each frame.
class MetarFileStore : public StationWeatherSource {
 public:
  ~MetarFileStore() override;

  // Begin loading from the X-Plane install root (the XPLMGetSystemPath result;
  // a trailing path separator is optional). Idempotent: only the first call
  // takes effect.
  void start(std::string xplaneRoot);

  // Periodically re-scan for a freshly downloaded metar file. Call once per
  // frame from the sim thread with the frame delta; the actual parse runs on a
  // worker thread, so this only kicks one off when due.
  void poll(double dtSeconds);

  std::optional<StationWeather> weatherForStation(
      const std::string& icao) const override;

 private:
  // Scan weatherDir_ for the newest metar-*.txt and, if it differs from the one
  // already indexed, parse it into a fresh ICAO->report map.
  void reloadAsync();

  std::string weatherDir_;
  std::atomic<bool> started_{false};
  std::atomic<bool> loading_{false};
  double sincePollSeconds_ = 0.0;

  std::thread thread_;
  mutable std::mutex mu_;  // guards metarByIcao_ and loadedFile_
  std::unordered_map<std::string, std::string> metarByIcao_;
  std::string loadedFile_;
};

}  // namespace avionics
