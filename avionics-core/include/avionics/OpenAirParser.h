#pragma once

#include <cstddef>
#include <istream>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Parses an OpenAir airspace file (the format X-Plane ships at
// Resources/default data/airspaces/airspace.txt) into drawable airspaces.
//
// Supported records: AC (class), AN (name), AH/AL (ceiling/floor), V X= (arc
// center), V D= (arc direction), DP (polygon vertex), DC (circle), DA (arc by
// radius/angles), DB (arc between two points). Arcs and circles are
// tessellated into the boundary point list so the renderer only deals with
// polylines. Unsupported/ignored records (AT, SP, SB, comments) are skipped.
//
// Airspaces whose class we don't draw (Class A, CTR, wave windows, ...) are
// returned as AirspaceClass::Other so callers can filter them out cheaply.
//
// The parser is pure (no file or thread ownership); shells open the file and
// own any background loading, mirroring the navaid/fix parsing split.
std::vector<MapAirspace> parseOpenAir(std::istream& in);

// Returns the airspaces whose bounding box overlaps a box of (rangeNm + margin)
// around (lat, lon), capped at maxCount. Cheap first-pass cull for the moving
// map; AirspaceClass::Other entries are dropped.
std::vector<MapAirspace> airspacesNear(const std::vector<MapAirspace>& src,
                                       double lat, double lon, float rangeNm,
                                       std::size_t maxCount);

}  // namespace avionics
