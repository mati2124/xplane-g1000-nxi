#pragma once

#include <algorithm>

namespace avionics {

// Shared G1000 moving-map range ladder, in NM. The PFD inset map and the MFD
// MAP page both step through this list (RNG- / RNG+ on the softkeys or the
// on-screen bezel range rocker), so they share one definition rather than each
// hard-coding their own.
inline constexpr float kMapRangeLadderNm[] = {0.5f,  1.0f,  1.5f,   2.5f,
                                              5.0f,  10.0f, 15.0f,  25.0f,
                                              50.0f, 100.0f, 150.0f, 250.0f};
inline constexpr int kMapRangeLadderCount =
    static_cast<int>(sizeof(kMapRangeLadderNm) / sizeof(kMapRangeLadderNm[0]));

// Default ladder position (10 NM), the G1000 power-on map range.
inline constexpr int kMapRangeDefaultIndex = 5;

// Clamp a ladder index into range and return the corresponding NM value.
inline float mapRangeNmAt(int index) {
  index = std::max(0, std::min(kMapRangeLadderCount - 1, index));
  return kMapRangeLadderNm[index];
}

}  // namespace avionics
