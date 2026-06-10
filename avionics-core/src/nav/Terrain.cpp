#include "avionics/Terrain.h"

#include <algorithm>
#include <cmath>

namespace avionics {

// Multi-octave sinusoidal height field. The per-degree frequencies are chosen
// so features span roughly 15-120 NM (1 degree of latitude is 60 NM), giving
// terrain that varies smoothly within a map view yet has structure across it.
// The result is offset/clamped so most of the field is low (green) with hills
// and the occasional ridge, plus a few sub-sea-level basins drawn as water.
float ProceduralTerrain::elevationFt(double lat, double lon) const {
  const double e =
      1500.0 + 1150.0 * std::sin(lat * 2.1 + 1.0) * std::cos(lon * 1.7 - 0.5) +
      820.0 * std::sin(lat * 5.3 - lon * 4.1 + 2.0) +
      520.0 * std::cos(lat * 9.7 + lon * 8.3) +
      300.0 * std::sin(lon * 13.0 + 0.7);
  return static_cast<float>(std::max(-400.0, e));
}

}  // namespace avionics
