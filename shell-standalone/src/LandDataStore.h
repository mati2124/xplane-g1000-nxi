#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Loads the bundled Natural Earth land-data asset (rivers, lakes, major
// roads, country borders, populated places) produced by
// tools/convert_natural_earth.py, for the map's land overlay.
//
// Binary layout (little-endian) -- must match the converter:
//   u32 magic "AVLD", u32 version, u32 line count, u32 city count
//   lines:  u8 class (0 river,1 lake,2 road,3 border,4 coast,5 state,
//           6 railroad), u16 point count, then (f32 lat, f32 lon) pairs
//   cities: f32 lat, f32 lon, u8 rank, u8 name length, name bytes
//
// Parsed once on a background thread; immutable after, so the nearby queries
// read without locks.
class LandDataStore {
 public:
  explicit LandDataStore(std::string path);
  ~LandDataStore();

  LandDataStore(const LandDataStore&) = delete;
  LandDataStore& operator=(const LandDataStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }

  // Lines whose bounding box overlaps the query box around (lat, lon), and
  // cities inside it, capped at maxCount. Empty until loaded().
  std::vector<MapLandLine> nearbyLines(double lat, double lon, float rangeNm,
                                       std::size_t maxCount) const;
  std::vector<MapLandCity> nearbyCities(double lat, double lon, float rangeNm,
                                        std::size_t maxCount) const;

 private:
  void load();  // background-thread entry point

  // Precomputed bounding box per line so the per-second nearby scan skips
  // most of the world without touching point data.
  struct Bounds {
    float minLat = 0.0f, maxLat = 0.0f, minLon = 0.0f, maxLon = 0.0f;
  };

  std::vector<MapLandLine> lines_;
  std::vector<Bounds> lineBounds_;
  std::vector<MapLandCity> cities_;
  std::string path_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
