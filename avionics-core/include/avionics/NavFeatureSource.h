#pragma once

#include <cstddef>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// "Where nearby navaids/fixes come from" for the moving map. This lets the
// platform-agnostic core (e.g. MockDataSource) pull real nav data from a
// shell-provided database (which knows how to find and parse the X-Plane files)
// without the core depending on the shell.
class NavFeatureSource {
 public:
  virtual ~NavFeatureSource() = default;

  // True once the database has finished loading (loading is async on most
  // backends, so this is false for a short while after construction).
  virtual bool ready() const = 0;

  // Up to maxCount features within rangeNm of (lat, lon), nearest first.
  // Returns empty until ready().
  virtual std::vector<MapFeature> nearby(double lat, double lon, float rangeNm,
                                         std::size_t maxCount) const = 0;
};

}  // namespace avionics
