#pragma once

#include <string>
#include <vector>

namespace avionics {

// Locates an X-Plane installation using the per-OS install-list files
// (x-plane_install_12.txt / _11.txt). Shared by NavDataStore, AirspaceStore,
// and DsfTerrainStore so each subsystem does not duplicate discovery logic.
namespace xplane_install {

std::vector<std::string> readInstallRoots();

// Directory containing earth_nav.dat / earth_fix.dat, preferring Custom Data.
std::string navDataDirForRoot(const std::string& root);

// Global Scenery "Earth nav data" directory with 1° DSF tiles (+LAT-LON.dsf).
// Returns empty when no suitable install is found.
std::string earthNavDataDir();

}  // namespace xplane_install

}  // namespace avionics
