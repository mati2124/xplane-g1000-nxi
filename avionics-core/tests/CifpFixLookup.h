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
  table.fixes["DOGLE"] = {26.620000000, -81.650000000};
  table.fixes["JOCKS"] = {26.700000000, -81.720000000};
  return table;
}

inline bool mapFixLookup(const std::string& ident, double& lat, double& lon,
                         void* ctx) {
  using FixMap = std::unordered_map<std::string, std::pair<double, double>>;
  auto* fixes = static_cast<FixMap*>(ctx);
  if (fixes == nullptr) return false;
  const auto it = fixes->find(ident);
  if (it == fixes->end()) return false;
  lat = it->second.first;
  lon = it->second.second;
  return true;
}

inline CifpFixTable kfmyR05FixTable() {
  CifpFixTable table;
  table.fixes["CITAG"] = {26.237036111, -81.779541667};
  table.fixes["BUTLY"] = {26.351555556, -81.960486111};
  table.fixes["PINTS"] = {26.802891667, -82.135858333};
  table.fixes["AZOMY"] = {26.519147222, -82.129402778};
  table.fixes["QUZSY"] = {26.700000000, -82.050000000};
  table.fixes["UZAWO"] = {26.436994444, -82.046516667};
  table.fixes["GRAMS"] = {26.512741667, -81.953697222};
  table.fixes["HADMO"] = {26.534000000, -81.940000000};
  table.fixes["RW05"] = {26.5803025, -81.8672716};
  table.fixes["IBITE"] = {26.583333333, -81.866666667};
  table.fixes["SERFS"] = {26.800700000, -81.819600000};
  return table;
}

}  // namespace avionics::test
