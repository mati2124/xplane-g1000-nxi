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

// UI popup symbology (Direct-To, Nearest, etc.): diamond airport icons and
// atlas-style navaid glyphs matching icons-map/*.png in the WT G1000 project.
void drawUiWaypointIcon(Renderer& r, const MapFeature& feature, float x,
                        float y, float size);
void drawUiWaypointIcon(Renderer& r, MapFeatureType type, float x, float y,
                        float size, const Color& c,
                        AirportFacilityKind airportKind = AirportFacilityKind::Land,
                        bool airportTowered = false,
                        bool airportServiced = false);

// TCAS traffic target symbol by threat level (TIS/TAS symbology). `half` is the
// symbol half-extent in pixels: open white diamond (Other), solid white diamond
// (Proximity), or solid yellow circle (Advisory).
void drawTrafficSymbol(Renderer& r, float x, float y, float half,
                       TrafficThreat threat);

// Off-scale Traffic Advisory: a half yellow circle drawn at the outer range
// ring, bulging toward the intruder. `bearingRad` is the screen angle from the
// scope center to the target (0 = up/ahead, increasing clockwise).
void drawTrafficOffScaleAdvisory(Renderer& r, float x, float y, float half,
                                 float bearingRad);

}  // namespace avionics
