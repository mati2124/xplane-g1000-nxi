#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "avionics/Terrain.h"

namespace avionics {

// Samples real-world elevation from X-Plane Global Scenery DSF tiles (the same
// 1°x1° raster DEMs the simulator meshes). When no install or tile is available
// it falls back to ProceduralTerrain so the map still renders.
//
// Loading a tile is expensive (full file read, often a 7z decompress through a
// subprocess), and the terrain raster samples tens of thousands of points per
// frame while rebuilding, so all tile loading happens on a dedicated worker
// thread: a render-thread sample that misses the cache enqueues the tile and
// returns the procedural fallback, and `revision()` bumps when the real tile
// arrives so cached terrain rasters know to resample. A negative cache keeps
// absent tiles (ocean / scenery not installed) from retrying the filesystem.
class DsfTerrainStore : public TerrainSource {
 public:
  // Discovers the Global Scenery "Earth nav data" directory via the per-OS
  // install-list files (used by the standalone shell).
  DsfTerrainStore();

  // Uses a caller-provided "Earth nav data" directory. The X-Plane plugin shell
  // resolves the install root through the SDK (XPLMGetSystemPath) rather than
  // the install-list files, so it passes the directory in directly. An empty
  // string disables tile loading and falls back to ProceduralTerrain.
  explicit DsfTerrainStore(std::string earthNavDir);

  ~DsfTerrainStore() override;

  bool ready() const { return !earthNavDir_.empty(); }
  const std::string& sourceDir() const { return earthNavDir_; }

  float elevationFt(double lat, double lon) const override;
  void elevationFtRow(double lat, double lonStart, double lonStep, int count,
                      float* out) const override;
  unsigned revision() const override {
    return revision_.load(std::memory_order_relaxed);
  }

 private:
  struct TileCacheEntry;

  // A queued tile load: the integer tile key plus a point inside the tile
  // (the path's 10x10-degree folder bucket needs a real signed lat/lon).
  struct PendingLoad {
    int southLat = 0;
    int lonIndex = 0;
    double lat = 0.0;
    double lon = 0.0;
  };

  void workerMain();
  std::string tilePath(int southLat, int lonIndex, double lat,
                       double lon) const;

  // Cache/queue helpers that assume mu_ is already held, so a single lock can
  // cover a whole raster row (see elevationFtRow). findResidentTileLocked
  // returns the resident tile for a key (LRU-touched) or nullptr; on a miss
  // queueTileLocked records the negative cache / enqueues a worker load.
  TileCacheEntry* findResidentTileLocked(
      int southLat, int lonIndex,
      std::chrono::steady_clock::time_point now) const;
  void queueTileLocked(int southLat, int lonIndex, double lat, double lon,
                       std::chrono::steady_clock::time_point now) const;

  std::string earthNavDir_;
  ProceduralTerrain fallback_;

  static constexpr int kMaxCachedTiles = 32;
  static constexpr int kMaxMissEntries = 256;
  static constexpr std::size_t kMaxPendingLoads = 4;

  mutable std::mutex mu_;
  mutable std::condition_variable cv_;
  mutable std::vector<TileCacheEntry*> cache_;       // front = most recent
  mutable std::vector<std::pair<int, int>> misses_;  // {southLat, lonIndex}
  // Tiles queued for (or currently in) a worker load; guards re-enqueueing.
  mutable std::vector<PendingLoad> pending_;
  std::atomic<unsigned> revision_{0};
  bool stop_ = false;
  std::thread worker_;
};

}  // namespace avionics
