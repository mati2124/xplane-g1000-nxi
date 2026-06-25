#include "avionics/nav/NearbyFeatureSelect.h"

#include <gtest/gtest.h>

namespace avionics {
namespace {

MapFeature makeFix(double lat, double lon, const char* id) {
  MapFeature f;
  f.type = MapFeatureType::Fix;
  f.lat = lat;
  f.lon = lon;
  f.id = id;
  return f;
}

TEST(NearbyFeatureSelectTest, NearbyFixesReachAcrossQueryRange) {
  const double centerLat = 26.5;
  const double centerLon = -81.7;
  const std::vector<MapFeature> fixes = {
      makeFix(centerLat + 0.5, centerLon, "NEAR"),
      makeFix(centerLat + 1.5, centerLon, "FAR"),
  };

  const auto picked =
      selectNearbyFixes(fixes, centerLat, centerLon, 160.0f, 16);

  EXPECT_EQ(picked.size(), 2u);
}

TEST(NearbyFeatureSelectTest, KeepsTightlyClusteredFixes) {
  // Three fixes within ~1 NM of each other must all be kept (the old grid
  // decimation dropped all but one per cell).
  const double centerLat = 26.5;
  const double centerLon = -81.7;
  const std::vector<MapFeature> fixes = {
      makeFix(centerLat, centerLon, "AAA"),
      makeFix(centerLat + 0.005, centerLon, "BBB"),
      makeFix(centerLat, centerLon + 0.005, "CCC"),
  };

  const auto picked =
      selectNearbyFixes(fixes, centerLat, centerLon, 25.0f, 64);

  EXPECT_EQ(picked.size(), 3u);
}

TEST(NearbyFeatureSelectTest, NearestFirstWithinBudget) {
  const double centerLat = 26.5;
  const double centerLon = -81.7;
  const std::vector<MapFeature> fixes = {
      makeFix(centerLat + 0.2, centerLon, "MID"),
      makeFix(centerLat + 0.4, centerLon, "FAR"),
      makeFix(centerLat + 0.01, centerLon, "NEAR"),
  };

  const auto picked = selectNearbyFixes(fixes, centerLat, centerLon, 160.0f, 2);

  ASSERT_EQ(picked.size(), 2u);
  EXPECT_EQ(picked[0].id, "NEAR");
  EXPECT_EQ(picked[1].id, "MID");
}

}  // namespace
}  // namespace avionics
