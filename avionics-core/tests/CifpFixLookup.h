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

}  // namespace avionics::test
