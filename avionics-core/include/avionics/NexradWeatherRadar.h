#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "avionics/WeatherRadar.h"

namespace avionics {

// Live datalink-style NEXRAD for the moving-map overlay. Fetches the public
// RainViewer composite-reflectivity tiles (real ground weather radar, global
// coverage, ~10-minute frames) on a background thread, decodes them, and
// resamples a square, ownship-centered, north-up intensity grid. The shared
// WeatherRaster overlay then recolors that intensity with the G1000
// reflectivity ramp (green -> yellow -> red -> magenta) so the display matches
// a real datalink NEXRAD legend rather than RainViewer's own (blue) palette.
// Unlike the onboard radar (DatarefWeatherRadar / ProceduralWeatherRadar) this
// reflects the actual weather on the ground around the aircraft.
//
// Built into avionics-core only when libcurl + stb_image are available (the
// shells); otherwise it compiles to an always-inactive stub so the core still
// builds standalone for tests.
class NexradWeatherRadar : public WeatherRadarSource {
 public:
  NexradWeatherRadar();
  ~NexradWeatherRadar() override;

  NexradWeatherRadar(const NexradWeatherRadar&) = delete;
  NexradWeatherRadar& operator=(const NexradWeatherRadar&) = delete;

  // Set the aircraft position the overlay is centered on. Cheap to call every
  // frame; a background tile fetch is kicked off when the position first
  // becomes valid, drifts far from the last fetch, or the data goes stale.
  void setCenter(double lat, double lon);

  // Pump the source: adopts a finished background fetch and resamples the grid.
  void advance(double dtSeconds);

  bool active() const override { return active_; }
  WeatherRadarLayout layout() const override {
    return WeatherRadarLayout::Centered;
  }
  int width() const override { return kGrid; }
  int height() const override { return kGrid; }
  const unsigned char* returnStrength() const override {
    return strength_.empty() ? nullptr : strength_.data();
  }
  float rangeNm() const override { return kRadiusNm; }
  unsigned revision() const override { return revision_; }

 private:
  // Ownship-centered grid resolution and the radius (NM) it spans from the
  // aircraft to each edge (square; half-width defaults to range for a 360 view).
  static constexpr int kGrid = 512;
  static constexpr float kRadiusNm = 250.0f;
  // RainViewer's maximum tile zoom.
  static constexpr int kZoom = 7;

  // A decoded web-mercator radar mosaic covering a region, ready to sample by
  // global pixel coordinate at zoom kZoom.
  struct Mosaic {
    bool valid = false;
    int zoom = kZoom;
    // Decoded 256x256 RGBA tiles keyed by (tileX << 32 | tileY).
    std::unordered_map<std::uint64_t, std::vector<unsigned char>> tiles;
  };

  void maybeStartFetch();

  // Resample the ownship-centered intensity grid from a decoded mosaic. This is
  // the expensive step (kGrid*kGrid pixels, each a nearest-palette dBZ lookup),
  // so it runs on resampleThread_ off the caller's thread; advance() only swaps
  // the finished buffer into strength_ (keeping every strength_ write on the
  // consumer thread, so returnStrength()/revision() readers stay race-free).
  static void resampleInto(const Mosaic& mosaic, double centerLat,
                           double centerLon, std::vector<unsigned char>& out);
  void startResampleWorker();
  void stopResampleWorker();
  void resampleWorkerMain();
  void submitResample();

  // Worker: download + decode the tiles for (lat,lon); fills `out`. Aborts
  // promptly (mid-transfer) once `*abort` becomes true so teardown never blocks
  // on a slow network request.
  static void fetchMosaic(double lat, double lon, const std::atomic<bool>* abort,
                          Mosaic& out);

  // kGrid*kGrid intensity (0-255), recolored by the shared G1000 ramp.
  std::vector<unsigned char> strength_;

  bool active_ = false;
  unsigned revision_ = 0;

  // Center the overlay is built for / requested at.
  bool haveCenter_ = false;
  double centerLat_ = 0.0;
  double centerLon_ = 0.0;
  double resampledLat_ = 0.0;
  double resampledLon_ = 0.0;
  double secondsSinceFetch_ = 1e9;  // force an initial fetch once centered

  // Background fetch handoff.
  std::thread worker_;
  std::atomic<bool> fetching_{false};
  std::atomic<bool> resultReady_{false};
  std::atomic<bool> abortFetch_{false};  // set in the destructor to cancel
  std::mutex mutex_;
  Mosaic pending_;  // guarded by mutex_, published by the worker
  // The decoded mosaic the grid resamples from. shared_ptr so the resample
  // worker can keep its input alive while the sim thread swaps in a newer fetch.
  std::shared_ptr<const Mosaic> activeMosaic_;
  double fetchCenterLat_ = 0.0;  // center the in-flight / active fetch used
  double fetchCenterLon_ = 0.0;

  // Resample worker handoff. The worker fills resampleScratch_ off-thread; the
  // sim thread (advance) swaps it into strength_ and bumps revision_.
  enum class ResamplePhase { Idle, Running, Done };
  std::thread resampleThread_;
  std::mutex resampleMu_;
  std::condition_variable resampleCv_;
  ResamplePhase resamplePhase_ = ResamplePhase::Idle;
  bool resampleStop_ = false;
  std::shared_ptr<const Mosaic> resampleMosaic_;  // input for the running job
  double resampleReqLat_ = 0.0;
  double resampleReqLon_ = 0.0;
  std::vector<unsigned char> resampleScratch_;  // worker output (kGrid*kGrid)
  double resampleDoneLat_ = 0.0;
  double resampleDoneLon_ = 0.0;
};

}  // namespace avionics
