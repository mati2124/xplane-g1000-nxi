#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "avionics/CifpParser.h"

namespace avionics::test {

struct CifpFixTable {
  std::unordered_map<std::string, std::pair<double, double>> fixes;
};

inline bool cifpFixLookup(const std::string& ident, double& lat, double& lon,
                          void* ctx) {
  auto* table = static_cast<CifpFixTable*>(ctx);
  const auto it = table->fixes.find(ident);
  if (it == table->fixes.end()) return false;
  lat = it->second.first;
  lon = it->second.second;
  return true;
}

inline CifpFixTable kpgdR04FixTable() {
  CifpFixTable table;
  table.fixes["BULOW"] = {26.753280556, -82.089683333};
  table.fixes["CISTS"] = {26.840213889, -82.035677778};
  table.fixes["YENCU"] = {26.882383333, -82.009438889};
  table.fixes["RW04"] = {26.543665, -81.593173};
  return table;
}

inline CifpFixTable kfmyR05FixTable() {
  CifpFixTable table;
  table.fixes["CITAG"] = {26.237036111, -81.779541667};
  table.fixes["BUTLY"] = {26.351555556, -81.960486111};
  table.fixes["UZAWO"] = {26.436994444, -82.046516667};
  table.fixes["GRAMS"] = {26.512741667, -81.953697222};
  table.fixes["HADMO"] = {26.534000000, -81.940000000};
  table.fixes["RW05"] = {26.5803025, -81.8672716};
  return table;
}

}  // namespace avionics::test
