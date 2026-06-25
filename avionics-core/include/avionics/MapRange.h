#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace avionics {

// Shared G1000 moving-map range ladder, in NM. The PFD inset map and the MFD
// MAP page both step through this list (RNG- / RNG+ on the softkeys or the
// on-screen bezel range rocker), so they share one definition rather than each
// hard-coding their own. The NXi exposes 28 steps from 250 ft to 1000 NM
// (Pilot's Guide §5); close-range foot steps support SafeTaxi taxi routing.
inline constexpr float kFtPerNm = 6076.115f;
inline constexpr float kMapRangeLadderNm[] = {
    250.0f / kFtPerNm,  500.0f / kFtPerNm,  750.0f / kFtPerNm,
    1000.0f / kFtPerNm, 1250.0f / kFtPerNm, 1500.0f / kFtPerNm,
    1750.0f / kFtPerNm, 2000.0f / kFtPerNm, 2250.0f / kFtPerNm,
    2500.0f / kFtPerNm, 2750.0f / kFtPerNm, 3000.0f / kFtPerNm,
    0.5f,   1.0f,   1.5f,   2.5f,   5.0f,   10.0f,  15.0f,  25.0f,
    50.0f,  100.0f, 150.0f, 250.0f, 350.0f, 500.0f, 750.0f, 1000.0f};
inline constexpr int kMapRangeLadderCount =
    static_cast<int>(sizeof(kMapRangeLadderNm) / sizeof(kMapRangeLadderNm[0]));

// Close-range foot rungs prepended to the legacy NM ladder (16 steps).
inline constexpr int kMapRangeCloseRungCount = 12;
// Bump when the ladder layout changes so persisted range indices can migrate.
inline constexpr int kMapRangeLadderVersion = 2;
// Pre-close-range ladder length (0.5 NM … 1000 NM); used to migrate saved
// indices from before kMapRangeCloseRungCount foot steps were added.
inline constexpr int kLegacyMapRangeLadderCount = 16;

// Minimum selectable map range (250 ft on the NXi).
inline constexpr float kMapRangeMinNm = kMapRangeLadderNm[0];

// Default ladder position (10 NM), the G1000 power-on map range.
inline constexpr int kMapRangeDefaultIndex = 17;

// Default max map range (NM) for traffic symbols and labels on the navigation
// map (Map Setup "Traffic Symbols" / "Traffic Labels" ranges).
inline constexpr float kTrafficMapRangeDefaultNm = 15.0f;

// Default max map range (NM) for airport symbols on the navigation map (Map
// Setup "Large Airport" / "Medium Airport" ranges). Airports declutter once the
// map opens past this step on the real NXi.
inline constexpr float kAirportMaxRangeNm = 100.0f;
inline constexpr float kMediumAirportMaxRangeNm = 50.0f;
inline constexpr float kSmallAirportMaxRangeNm = 25.0f;
// Intersections / VFR waypoints on the navigation map (Map Setup "Fixes").
inline constexpr float kFixMaxRangeNm = 25.0f;
// At 50 NM and wider the NXi keeps only the most significant airports on chart
// (longest runway / towered), not every field that passes the size-class gate.
inline constexpr float kAirportImportanceBudgetMinNm = 50.0f;
inline constexpr int kAirportImportanceBudget = 15;

// Top of the shared range ladder (1000 NM on the NXi).
inline constexpr float kMapRangeMaxNm =
    kMapRangeLadderNm[kMapRangeLadderCount - 1];

// Land/border vector queries must cover the full ladder with margin so
// coastlines and country borders reach the edges of a wide view (the US–Mexico
// border reaches ~118°W).
inline constexpr float kLandQueryRangeNm = kMapRangeMaxNm * 1.15f;

// MFD north-up map corners reach ~4× the labeled range in ground distance
// (range ring at 0.25× viewport height; see mapRangeSpanFrac in MapViewInternal.h).
// LandDataStore bbox queries must use at least this east/west reach or GSHHG
// lon-band fills stop short of the chart edge above ~100 NM.
inline constexpr float kMapViewportReachFactor = 4.0f;

// GSHHG regional lon-band land carries peninsula-scale shore geometry from the
// closest ladder step through mid range. Below ~15 NM the detail spatial index
// layers local shore rings on top; continental silhouettes stay out of the query.
inline constexpr float kRegionalLandMinRangeNm = kMapRangeLadderNm[0];
inline constexpr float kRegionalLandMaxRangeNm = 120.0f;
// Continental silhouettes are omitted from close-range queries; regional lon-bands
// resume above this range (see LandDataStore and MapLandLayer).
inline constexpr float kRegionalSilhouetteSuppressMaxNm = 15.0f;
// GSHHG regional lon-band rings span at least ~20°; use this to separate them
// from local high-point-count shore rings in fill draw order.
inline constexpr float kRegionalLonBandMinGeoSpanDeg = 40.0f;
// GSHHG L1 continent rings and mid-range chart silhouettes (MapLandLayer).
inline constexpr float kContinentalSilhouetteMinGeoSpanDeg = 50.0f;

// State/province borders and labels stay on the chart through this range; past
// it only nation outlines and the largest region names remain (NXi declutter).
inline constexpr float kStateBorderMaxRangeNm = 400.0f;

// SafeTaxi runway/taxiway pavement and identifier labels on the navigation
// map. The real NXi shows airport surfaces at the 1.5 NM range step (Pilot's
// Guide close-range ladder); wider views keep only the airport symbol.
inline constexpr float kAirportDiagramMaxRangeNm = 1.5f;
// Embedded WPT Airport Information map uses the same tight zoom.
inline constexpr float kAirportDiagramRangeNm = kAirportDiagramMaxRangeNm;

// Clamp a ladder index into range and return the corresponding NM value.
inline float mapRangeNmAt(int index) {
  index = std::max(0, std::min(kMapRangeLadderCount - 1, index));
  return kMapRangeLadderNm[index];
}

// Nearest ladder index for a continuous range value (e.g. sim/cockpit2/EFIS/map_range_nm).
inline int mapRangeIndexForNm(float rangeNm) {
  if (rangeNm <= 0.0f) return kMapRangeDefaultIndex;
  int best = kMapRangeDefaultIndex;
  float bestLogDist = 1e9f;
  for (int i = 0; i < kMapRangeLadderCount; ++i) {
    const float logDist =
        std::fabs(std::log(kMapRangeLadderNm[i]) - std::log(rangeNm));
    if (logDist < bestLogDist) {
      bestLogDist = logDist;
      best = i;
    }
  }
  return best;
}

// Time constant (seconds) for the moving-map zoom animation. The displayed
// scale eases toward the selected ladder step instead of snapping, matching
// the Working Title G1000 NXi's smooth zoom. Small enough to feel responsive
// to a range-rocker press, large enough to read as an animation.
inline constexpr float kMapZoomTimeConstantSec = 0.12f;

// Ease an animated map range (NM) toward `target` over `dtSeconds`, using
// frame-rate-independent exponential smoothing in log space. Working in log
// space makes each ladder step cover a similar-looking zoom regardless of the
// absolute range (0.5->1 NM reads like 100->250 NM). Returns the target exactly
// once within ~0.4% so the animation settles; a non-positive `current` (an
// uninitialized animator) jumps straight to the target.
inline float animateMapRange(float current, float target, double dtSeconds) {
  if (current <= 0.0f || target <= 0.0f) return target;
  const float logCur = std::log(current);
  const float logTgt = std::log(target);
  if (std::fabs(logTgt - logCur) < 0.004f) return target;
  const float a =
      1.0f - std::exp(-static_cast<float>(dtSeconds) / kMapZoomTimeConstantSec);
  return std::exp(logCur + (logTgt - logCur) * a);
}

// True once the animated range has reached the selected ladder step (same
// ~0.4% log tolerance as animateMapRange uses to snap to target).
inline bool mapRangeZoomSettled(float displayRangeNm, float rangeNm) {
  if (displayRangeNm <= 0.0f || rangeNm <= 0.0f) return true;
  return std::fabs(std::log(displayRangeNm) - std::log(rangeNm)) < 0.004f;
}

// Format a map range for the on-screen range readout, matching the G1000 NXi:
// close-range foot steps print as integers ("250FT"), whole-NM ranges as
// integers ("10NM"), and half-NM ladder steps keep one decimal ("2.5NM").
inline void formatMapRange(char* buf, std::size_t n, float rangeNm) {
  if (rangeNm < 0.5f) {
    const int ft = static_cast<int>(std::lround(rangeNm * kFtPerNm));
    std::snprintf(buf, n, "%dFT", ft);
    return;
  }
  const float tenths = std::round(rangeNm * 10.0f) / 10.0f;
  if (std::fabs(tenths - std::round(tenths)) < 0.05f) {
    std::snprintf(buf, n, "%dNM", static_cast<int>(std::lround(tenths)));
  } else {
    std::snprintf(buf, n, "%.1fNM", static_cast<double>(tenths));
  }
}

// Migrate a persisted range index from an older ladder layout.
inline int migrateMapRangeIndex(int index, int savedVersion) {
  index = std::max(0, std::min(kMapRangeLadderCount - 1, index));
  if (savedVersion >= kMapRangeLadderVersion) return index;
  // Indices 16+ did not exist on the legacy 16-step ladder.
  if (index >= kLegacyMapRangeLadderCount) return index;
  return std::min(kMapRangeLadderCount - 1,
                  index + kMapRangeCloseRungCount);
}

}  // namespace avionics
