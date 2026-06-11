#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Optional obstacle database for the map overlay, loaded from a
// user-supplied FAA Digital Obstacle File in CSV format (the "DDOF CSV"
// distribution, whose header carries LATDEC/LONDEC decimal coordinates and
// AGL/AMSL heights in feet). US-only coverage; when no file is given the
// layer simply stays empty.
//
// Parsed once on a background thread into a 1-degree-cell spatial index;
// immutable once loaded so nearby() reads without locks.
class ObstacleStore {
 public:
  explicit ObstacleStore(std::string path);
  ~ObstacleStore();

  ObstacleStore(const ObstacleStore&) = delete;
  ObstacleStore& operator=(const ObstacleStore&) = delete;

  bool loaded() const { return loaded_.load(std::memory_order_acquire); }
  const std::string& sourcePath() const { return path_; }

  // Obstacles within rangeNm (+margin) of (lat, lon), capped at maxCount.
  // Empty until loaded().
  std::vector<MapObstacle> nearby(double lat, double lon, float rangeNm,
                                  std::size_t maxCount) const;

 private:
  void load();  // background-thread entry point

  std::unordered_map<int, std::vector<MapObstacle>> cells_;
  std::string path_;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
