#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "avionics/Terrain.h"

namespace avionics {

// Samples real-world elevation from X-Plane Global Scenery DSF tiles (the same
// 1°x1° raster DEMs the simulator meshes). When no install is available it
// falls back to ProceduralTerrain so the map still renders offline.
class DsfTerrainStore : public TerrainSource {
 public:
  DsfTerrainStore();

  explicit DsfTerrainStore(std::string earthNavDir);

  ~DsfTerrainStore() override;

  bool ready() const { return !earthNavDir_.empty(); }
  const std::string& sourceDir() const { return earthNavDir_; }

  float elevationFt(double lat, double lon) const override;
  void elevationFtRow(double lat, double lonStart, double lonStep, int count,
                      float* out) const override;
  void ensureCoverage(double minLat, double maxLat, double minLon,
                      double maxLon, bool waitForTiles = false) const override;
  void setBulkTerrainSample(bool enabled) const override;
  void setCoarseTerrainSample(bool enabled) const override;
  void setTerrainViewCenter(double lat, double lon,
                            float detailHalfNm) const override;
  unsigned revision() const override {
    return revision_.load(std::memory_order_relaxed);
  }

 private:
  struct TileCacheEntry;

  struct PendingLoad {
    int southLat = 0;
    int lonIndex = 0;
    double lat = 0.0;
    double lon = 0.0;
  };

  static std::uint64_t tileKey(int southLat, int lonIndex);
  static float tileAbsentElevationFt() {
    return std::numeric_limits<float>::quiet_NaN();
  }

  void workerMain();
  std::string tilePath(int southLat, int lonIndex, double lat,
                       double lon) const;

  static constexpr int kQuickGridSize = 4;
  static constexpr int kCoarseGridSize = 16;
  static constexpr int kCoarseGridCells =
      kCoarseGridSize * kCoarseGridSize;

  struct CoarseSummary {
    std::uint8_t gridSize = 0;
    float elevFt[kCoarseGridCells] = {};
  };

  TileCacheEntry* findResidentTileLocked(
      int southLat, int lonIndex,
      std::chrono::steady_clock::time_point now) const;
  const CoarseSummary* findSummaryLocked(int southLat, int lonIndex) const;
  float sampleCoarseGridLocked(const CoarseSummary& summary, int southLat,
                               int lonIndex, double lat, double lon) const;
  float sampleElevationLocked(int southLat, int lonIndex, double lat,
                              double lon,
                              std::chrono::steady_clock::time_point now) const;
  void queueTileLocked(int southLat, int lonIndex, double lat, double lon,
                       std::chrono::steady_clock::time_point now) const;
  void queueTileLocked(int southLat, int lonIndex, double lat, double lon,
                       std::chrono::steady_clock::time_point now,
                       bool force) const;
  bool tileMissedLocked(int southLat, int lonIndex) const;
  bool tileInDetailZoneLocked(int southLat, int lonIndex) const;
  bool popPendingLoadLocked(PendingLoad& out);
  bool tileReadyLocked(int southLat, int lonIndex,
                       std::chrono::steady_clock::time_point now) const;

  std::size_t maxCacheTilesLocked() const {
    return bulkSample_ ? static_cast<std::size_t>(kBulkMaxCachedTiles)
                       : static_cast<std::size_t>(kMaxCachedTiles);
  }

  std::size_t maxPendingLoadsLocked() const {
    return bulkSample_ ? kBulkMaxPendingLoads : kMaxPendingLoads;
  }

  std::string earthNavDir_;
  ProceduralTerrain fallback_;

  static constexpr int kWorkerCount = 4;
  static constexpr int kMaxCachedTiles = 64;
  static constexpr int kBulkMaxCachedTiles = 64;
  static constexpr std::size_t kMaxPendingLoads = 16;
  static constexpr std::size_t kBulkMaxPendingLoads = 128;
  static constexpr int kMaxBlockingEnsureTiles = 48;

  mutable std::mutex mu_;
  mutable std::condition_variable cv_;
  mutable std::vector<TileCacheEntry*> cache_;
  mutable std::unordered_set<std::uint64_t> misses_;
  mutable std::unordered_map<std::uint64_t, CoarseSummary> summaries_;
  mutable std::vector<PendingLoad> pending_;
  mutable bool bulkSample_ = false;
  mutable bool coarseSample_ = false;
  mutable double viewCenterLat_ = 0.0;
  mutable double viewCenterLon_ = 0.0;
  mutable float detailHalfNm_ = 0.0f;
  std::atomic<unsigned> revision_{0};
  bool stop_ = false;
  std::array<std::thread, kWorkerCount> workers_;
};

}  // namespace avionics
