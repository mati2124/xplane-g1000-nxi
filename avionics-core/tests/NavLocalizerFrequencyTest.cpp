#include <gtest/gtest.h>

#include "avionics/Radio.h"

namespace avionics {
namespace {

TEST(NavLocalizerFrequencyTest, DistinguishesLocFromVorByFrequency) {
  EXPECT_TRUE(isNavLocalizerMhz(108.10f));
  EXPECT_TRUE(isNavLocalizerMhz(108.15f));
  EXPECT_TRUE(isNavLocalizerMhz(111.95f));

  EXPECT_FALSE(isNavLocalizerMhz(108.00f));
  EXPECT_FALSE(isNavLocalizerMhz(108.20f));
  EXPECT_FALSE(isNavLocalizerMhz(113.00f));
  EXPECT_FALSE(isNavLocalizerMhz(107.95f));
}

TEST(NavLocalizerFrequencyTest, FmaAndCdiLabelsFollowActiveNavSource) {
  EXPECT_STREQ(fmaLateralNavModeLabel(CdiSource::Gps, 108.10f, 113.00f), "GPS");
  EXPECT_STREQ(fmaLateralNavModeLabel(CdiSource::Nav1, 108.10f, 113.00f), "LOC");
  EXPECT_STREQ(fmaLateralNavModeLabel(CdiSource::Nav2, 108.10f, 113.00f), "VOR");

  EXPECT_STREQ(cdiNavSourceLabel(CdiSource::Nav1, 108.10f, 113.00f), "LOC1");
  EXPECT_STREQ(cdiNavSourceLabel(CdiSource::Nav2, 113.00f, 108.10f), "LOC2");
  EXPECT_STREQ(cdiNavSourceLabel(CdiSource::Nav1, 113.00f, 108.10f), "VOR1");
}

}  // namespace
}  // namespace avionics
