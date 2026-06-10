#pragma once

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "avionics/Terrain.h"

namespace avionics {

// Samples real-world elevation from X-Plane Global Scenery DSF tiles (the same
// 1°x1° raster DEMs the simulator meshes). When no install or tile is available
// it falls back to ProceduralTerrain so the map still renders.
//
// Loading a tile is expensive (full file read, often a 7z decompress through a
// subprocess), and the topo raster samples hundreds of points per frame, so the
// store leans on three things to keep the render loop responsive:
//   - an LRU cache big enough for the widest tile footprint a map page touches,
//   - a negative cache so absent tiles (ocean / scenery not installed) don't
//     retry the filesystem on every sample, and
//   - a load throttle so at most one tile load happens per cooldown window;
//     samples that lose the race use the procedural fallback and pick up the
//     real tile on a later frame.
class DsfTerrainStore : public TerrainSource {
 public:
  DsfTerrainStore();
  ~DsfTerrainStore() override;

  bool ready() const { return !earthNavDir_.empty(); }
  const std::string& sourceDir() const { return earthNavDir_; }

  float elevationFt(double lat, double lon) const override;

 private:
  struct TileCacheEntry;

  const TileCacheEntry* tileFor(double lat, double lon) const;
  std::string tilePath(double lat, double lon) const;

  std::string earthNavDir_;
  ProceduralTerrain fallback_;

  static constexpr int kMaxCachedTiles = 16;
  static constexpr int kMaxMissEntries = 256;

  mutable std::vector<TileCacheEntry*> cache_;     // front = most recent
  mutable std::vector<std::pair<int, int>> misses_;  // {southLat, lonIndex}
  mutable std::chrono::steady_clock::time_point lastLoad_{};
};

}  // namespace avionics
