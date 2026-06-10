#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/render/MapView.h"

namespace avionics::mfd {

// MAP group.
void drawMapPage(Renderer& r, const FlightData& d, const MapData& map,
                 const MfdController& ui, float x, float y, float w, float h,
                 float displayH);

// WPT group. The airport page mirrors the real Airport Information layout;
// the navaid page renders Intersection/NDB/VOR Information depending on the
// feature type requested.
void drawWaypointPage(Renderer& r, const FlightData& d, const MapData& map,
                      float x, float y, float w, float h, float displayH);
void drawWaypointNavaidPage(Renderer& r, const FlightData& d,
                            const MapData& map, MapFeatureType type, float x,
                            float y, float w, float h, float displayH);

// AUX group.
void drawTripPlanningPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH);
void drawGpsStatusPage(Renderer& r, const FlightData& d, float x, float y,
                       float w, float h, float displayH);
void drawSystemStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH);

// NRST group. The feature page renders Nearest Intersections/NDB/VOR
// depending on the feature type requested.
void drawNearestAirportsPage(Renderer& r, const FlightData& d,
                             const MapData& map, float x, float y, float w,
                             float h, float displayH);
void drawNearestFeaturePage(Renderer& r, const FlightData& d,
                            const MapData& map, MapFeatureType type, float x,
                            float y, float w, float h, float displayH);
void drawNearestAirspacesPage(Renderer& r, const FlightData& d,
                              const MapData& map, float x, float y, float w,
                              float h, float displayH);

// FPL group.
void drawActiveFlightPlanPage(Renderer& r, const FlightData& d,
                              const MapData& map, float x, float y, float w,
                              float h, float displayH);

// Checklist group. Renders the currently selected checklist (one per "page")
// from the author-supplied file, with the item cursor and checked state from
// the controller.
void drawChecklistPage(Renderer& r, const ChecklistData& checklist,
                       const MfdController& ui, float x, float y, float w,
                       float h, float displayH);

}  // namespace avionics::mfd
