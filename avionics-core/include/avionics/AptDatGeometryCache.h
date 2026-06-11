#pragma once

#include <string>

#include "avionics/AptDatParser.h"

namespace avionics {

// Serializes AptDatParseResult (runway + pavement cells and airport metadata)
// to disk so the X-Plane plugin can skip re-parsing the 380 MB apt.dat on every
// reload. The cache is keyed on apt.dat file size and modification time.
bool loadAptDatGeometryCache(const std::string& cachePath,
                             const std::string& aptDatPath,
                             AptDatParseResult& out);

bool saveAptDatGeometryCache(const std::string& cachePath,
                             const std::string& aptDatPath,
                             const AptDatParseResult& in);

}  // namespace avionics
