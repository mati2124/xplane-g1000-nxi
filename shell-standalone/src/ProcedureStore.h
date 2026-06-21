#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include "avionics/CifpParser.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Loads X-Plane CIFP terminal procedure files (Resources/default data/CIFP or
// Custom Data/CIFP). Each airport is parsed on first access and cached.
class NavDataStore;
class AptDatStore;

class ProcedureStore {
 public:
  ProcedureStore(const NavDataStore& navData, const AptDatStore* aptData = nullptr);

  const std::string& sourceDir() const { return sourceDir_; }

  std::vector<MapProcedure> proceduresForAirport(const std::string& icao,
                                                  ProcedureType type) const;

  std::vector<MapLeg> expandProcedure(const std::string& icao,
                                      ProcedureType type,
                                      const std::string& name,
                                      const std::string& transition,
                                      const NavFeatureSource* lookup) const;

 private:
  const CifpAirportProcedures& loadAirport(const std::string& icao) const;

  const NavDataStore& navData_;
  const AptDatStore* aptData_ = nullptr;
  std::string sourceDir_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<std::string, CifpAirportProcedures> cache_;
};

}  // namespace avionics
