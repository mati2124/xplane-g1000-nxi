#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

#include "avionics/AptDatParser.h"

namespace avionics {
namespace {

TEST(AptDatPavementTest, KeepsOuterAndHoleContours) {
  // One hard-surface pavement slab with a single grass island punched out.
  const std::string apt =
      "1 873 0 0 KTEST Test Airport\n"
      "110 2 0 0 0\n"
      "111 33.0 -84.0\n"
      "111 33.01 -84.0\n"
      "111 33.01 -83.99\n"
      "111 33.0 -83.99\n"
      "113\n"
      "111 33.004 -83.996\n"
      "111 33.006 -83.996\n"
      "111 33.006 -83.994\n"
      "113\n"
      "115\n";

  std::istringstream in(apt);
  const AptDatParseResult result = parseAptDat(in);

  std::size_t pavementCount = 0;
  const MapPavement* found = nullptr;
  for (const auto& [key, pavements] : result.pavementCells) {
    (void)key;
    pavementCount += pavements.size();
    if (!pavements.empty()) found = &pavements.front();
  }
  ASSERT_EQ(pavementCount, 1u);
  ASSERT_NE(found, nullptr);
  ASSERT_EQ(found->contours.size(), 2u);
  EXPECT_EQ(found->contours[0].size(), 4u);
  EXPECT_EQ(found->contours[1].size(), 3u);
}

TEST(AptDatPavementTest, KatlExcerptKeepsMultiContourPavement) {
  const std::string path =
      std::string(AVIONICS_TEST_FIXTURE_DIR) + "/katl_apt_excerpt.dat";
  std::ifstream in(path);
  if (!in.good()) GTEST_SKIP() << "fixture missing: " << path;

  const AptDatParseResult result = parseAptDat(in);

  std::size_t pavementCount = 0;
  std::size_t maxContours = 0;
  for (const auto& [key, pavements] : result.pavementCells) {
    (void)key;
    for (const MapPavement& pav : pavements) {
      ++pavementCount;
      maxContours = std::max(maxContours, pav.contours.size());
    }
  }
  EXPECT_EQ(pavementCount, 20u);
  EXPECT_GE(maxContours, 10u)
      << "KATL pavement should retain grass-island holes, not one outer loop";
}

}  // namespace
}  // namespace avionics
