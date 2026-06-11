#pragma once

#include "avionics/Color.h"
#include "avionics/MapData.h"
#include "avionics/Renderer.h"

namespace avionics {

// Shared G1000 NXi map symbology for airports, navaids, and intersections.
// Used by MapView and the MFD waypoint list/header rows.
Color mapFeatureColor(const MapFeature& feature);
Color mapFeatureColor(MapFeatureType type);

void drawMapFeatureSymbol(Renderer& r, const MapFeature& feature, float x,
                          float y, float size);
void drawMapFeatureSymbol(Renderer& r, MapFeatureType type, float x, float y,
                          float size, const Color& c,
                          AirportFacilityKind airportKind = AirportFacilityKind::Land,
                          bool airportTowered = false,
                          bool airportServiced = false);

}  // namespace avionics
