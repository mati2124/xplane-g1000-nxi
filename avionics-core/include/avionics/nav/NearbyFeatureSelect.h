#pragma once

#include <cstddef>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Up to maxCount fixes within rangeNm of (lat, lon), nearest first. Every fix
// in range is a candidate (no spatial decimation): the renderer draws each
// symbol and declutters only the idents, matching the Garmin PC Trainer where
// dense areas show many unlabeled triangles.
std::vector<MapFeature> selectNearbyFixes(
    const std::vector<MapFeature>& fixes, double lat, double lon, float rangeNm,
    std::size_t maxCount);

// Assemble a nearby map-feature list: nearest airports and navaids first, then
// the nearest fixes fill the remaining budget. Shared by the standalone
// NavDataStore and the X-Plane plugin cache filter.
std::vector<MapFeature> assembleNearbyMapFeatures(
    const std::vector<MapFeature>& airports,
    const std::vector<MapFeature>& navaids,
    const std::vector<MapFeature>& fixes, double lat, double lon, float rangeNm,
    std::size_t maxCount);

// Same as assembleNearbyMapFeatures, but accepts one mixed feature vector
// (e.g. the X-Plane plugin nav cache).
std::vector<MapFeature> assembleNearbyMapFeaturesMixed(
    const std::vector<MapFeature>& src, double lat, double lon, float rangeNm,
    std::size_t maxCount);

}  // namespace avionics
