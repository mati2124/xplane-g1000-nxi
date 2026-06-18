#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/render/MapView.h"

namespace avionics::mfd {

// MAP group.
void drawMapPage(Renderer& r, const FlightData& d, const MapData& map,
                 MfdController& ui, float x, float y, float w, float h,
                 float displayH);
// Dedicated Traffic Map: ownship-centered range rings with traffic symbols and
// no topography, the NXi traffic display (Pilot's Guide, Hazard Avoidance).
void drawTrafficMapPage(Renderer& r, const FlightData& d, const MapData& map,
                        const MfdController& ui, float x, float y, float w,
                        float h, float displayH);
// Airborne color weather radar (GWX): the forward +/-45 deg scan wedge with
// color-graded precipitation returns, dashed range arcs, sweeping scan line,
// bearing line, mode/feature annunciations, the Scale legend, and the
// Tilt/Bearing/Sector Scan/Gain readout box (Pilot's Guide, Hazard Avoidance -
// Airborne Color Weather Radar, Figures 6-70 / 6-72 / 6-73).
void drawWeatherRadarPage(Renderer& r, const FlightData& d, const MapData& map,
                          const MfdController& ui, float x, float y, float w,
                          float h, float displayH);

// WPT group. The airport page mirrors the real Airport Information layout;
// the navaid page renders Intersection/NDB/VOR Information depending on the
// feature type requested.
void drawWaypointPage(Renderer& r, const FlightData& d, const MapData& map,
                      const MfdController& ui, float x, float y, float w,
                      float h, float displayH);
void drawWaypointNavaidPage(Renderer& r, const FlightData& d,
                            const MapData& map, const MfdController& ui,
                            MapFeatureType type, float x, float y, float w,
                            float h, float displayH);

// AUX group.
void drawTripPlanningPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH);
// Utility: the generic/flight/departure timers and trip statistics
// (Pilot's Guide, Section 5, AUX - Utility). The running totals come from the
// controller's accumulated session stats.
void drawUtilityPage(Renderer& r, const FlightData& d, const MfdController& ui,
                     float x, float y, float w, float h, float displayH);
void drawGpsStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                       float x, float y, float w, float h, float displayH);
// System Setup: date/time, display units, and alert configuration readouts
// (Pilot's Guide, Section 5, AUX - System Setup).
void drawSystemSetupPage(Renderer& r, const FlightData& d, float x, float y,
                         float w, float h, float displayH);
void drawSystemStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH);
// SimBrief OFP integration: Pilot ID + fetch status on the left, the latest
// OFP summary and filed route on the right. All state comes through the
// controller (the shell owns the network client).
void drawSimBriefPage(Renderer& r, const MfdController& ui, float x, float y,
                      float w, float h, float displayH);

// NRST group. The feature page renders Nearest Intersections/NDB/VOR
// depending on the feature type requested.
void drawNearestAirportsPage(Renderer& r, const FlightData& d,
                             const MapData& map, const MfdController& ui,
                             float x, float y, float w, float h, float displayH);
void drawNearestFeaturePage(Renderer& r, const FlightData& d,
                            const MapData& map, const MfdController& ui,
                            MapFeatureType type, float x, float y, float w,
                            float h, float displayH);
void drawNearestAirspacesPage(Renderer& r, const FlightData& d,
                              const MapData& map, const MfdController& ui,
                              float x, float y, float w, float h,
                              float displayH);
// Nearest Frequencies: ARTCC / FSS / WX frequency columns for the area
// (Pilot's Guide, Section 5, NRST - Nearest Frequencies).
void drawNearestFrequenciesPage(Renderer& r, const FlightData& d,
                                const MapData& map, float x, float y, float w,
                                float h, float displayH);

// FPL group. Renders the leg list with the controller's selection cursor and
// the editing overlays (Waypoint Information entry, Remove/Delete
// confirmations, page menu) driven by the FMS knob.
void drawActiveFlightPlanPage(Renderer& r, const FlightData& d,
                              const MapData& map, const MfdController& ui,
                              float x, float y, float w, float h,
                              float displayH);

// The GPS Direct-To window (Direct-To bezel key), drawn over the current MFD
// page: the destination identifier entry, the resolved waypoint with its map
// symbol and inset map, the bearing/distance and direct course from present
// position, the VNV constraints, and the ACTIVATE? prompt.
void drawDirectToWindow(Renderer& r, const FlightData& d, const MapData& map,
                        const MfdController& ui, float x, float y, float w,
                        float h, float displayH);

// The Map Settings window (MENU -> Map Settings on the Navigation Map page),
// drawn over the current MFD page: the Group selector and the active group's
// settings rows (Pilot's Guide Fig. 5-7).
void drawMapSettingsWindow(Renderer& r, const MfdController& ui, float x,
                           float y, float w, float h, float displayH);

// Checklist group. Renders the currently selected checklist (one per "page")
// from the author-supplied file, with the item cursor and checked state from
// the controller.
void drawChecklistPage(Renderer& r, const ChecklistData& checklist,
                       const MfdController& ui, float x, float y, float w,
                       float h, float displayH);

}  // namespace avionics::mfd
