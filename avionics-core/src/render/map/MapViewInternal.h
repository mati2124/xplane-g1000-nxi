#pragma once

#include <cmath>
#include <vector>

#include "avionics/Color.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/Renderer.h"
#include "avionics/render/MapView.h"
#include "render/map/MapProjection.h"

// Shared internals for the moving-map renderer. MapView::render (MapView.cpp) is
// the orchestrator; each visible map layer/symbol is its own translation unit
// (MapLandLayer.cpp, MapAirwayLayer.cpp, MapAirspaceLayer.cpp, ...). They all
// share the projection context and the low-level symbology/chrome primitives
// declared here, mirroring the per-instrument split under render/pfd.
namespace avionics::mapview {

constexpr float kWtCanvasHeight = 768.0f;

// Nav-feature symbol size, in 768-px-canvas units. Symbols are sized off the
// display (like text) rather than the viewport, so they stay a fixed, legible
// size on both the small PFD inset and the full-screen MFD MAP page instead of
// ballooning on the larger viewport.
constexpr float kFeatureSymbolWt = 8.0f;

// Ownship airplane symbol size, in the same 768-px-canvas units. The G1000 NXi
// ownship icon is drawn noticeably larger than the nav-feature symbols so the
// aircraft stands out from the airports/navaids it overflies.
constexpr float kOwnshipSymbolWt = 15.0f;

// Margin (px) added around the viewport when clipping per-pixel symbology, so
// teeth/dashes near the edge are not cut early.
constexpr float kSymbologyClipMarginPx = 32.0f;

inline float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kWtCanvasHeight);
}

// Shared projection context for the layer painters: geographic center,
// scale/rotation, and the viewport bounds for culling.
struct Proj {
  double centerLat = 0.0;
  double centerLon = 0.0;
  float cx = 0.0f;
  float cy = 0.0f;
  float pixelsPerNm = 0.0f;
  float rotation = 0.0f;
  float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;

  // Per-frame projection constants, computed once in init(). The hot path
  // projects thousands of vertices per render (coastlines, airspaces, airways,
  // nav features), and the rotation sin/cos and longitude scale are identical
  // for every one of them, so deriving them once here instead of per point
  // removes ~3 trig calls per vertex from each map redraw.
  double cosR = 1.0;
  double sinR = 0.0;
  double nmPerLon = map::kNmPerDegLat;

  void init() {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double rot = static_cast<double>(rotation) * kDegToRad;
    cosR = std::cos(rot);
    sinR = std::sin(rot);
    nmPerLon = map::nmPerDegLon(centerLat);
  }

  void toPx(double lat, double lon, float& x, float& y) const {
    const double northNm = (lat - centerLat) * map::kNmPerDegLat;
    const double eastNm = (lon - centerLon) * nmPerLon;
    const double mapEast = eastNm * cosR - northNm * sinR;
    const double mapNorth = eastNm * sinR + northNm * cosR;
    x = cx + static_cast<float>(mapEast * static_cast<double>(pixelsPerNm));
    y = cy - static_cast<float>(mapNorth * static_cast<double>(pixelsPerNm));
  }

  bool onScreen(float x, float y, float margin) const {
    return x >= minX - margin && x <= maxX + margin && y >= minY - margin &&
           y <= maxY + margin;
  }
};

// Viewport bounds passed to the per-pixel symbology generators so they can clip
// each edge before stepping along it. A null clip disables clipping (used by
// small, already-bounded callers like the fuel-reserve ring).
struct ClipBounds {
  float minX, minY, maxX, maxY;
};

// Per-pixel symbology (dashes, comb teeth, railroad ties) steps along a line's
// full on-screen pixel length. At close map zoom an airspace/border edge can be
// enormous in pixels and reach far off-screen, so without clipping it generates
// hundreds of thousands of invisible segments (a ~100x cost cliff measured on
// the MFD MAP page). This clips one segment to the viewport rect (expanded by
// `margin`) and returns the visible distance interval [lo, hi] along it, so the
// generators can iterate only the portion that can actually be seen. Liang-
// Barsky parametric clip with t in [0, len] (direction is the unit vector).
// Returns false when the segment is entirely outside the rect.
inline bool segmentVisibleSpan(float ax, float ay, float ux, float uy,
                               float len, float minX, float minY, float maxX,
                               float maxY, float margin, float& lo, float& hi) {
  lo = 0.0f;
  hi = len;
  const float x0 = minX - margin, x1 = maxX + margin;
  const float y0 = minY - margin, y1 = maxY + margin;
  const float p[4] = {-ux, ux, -uy, uy};
  const float q[4] = {ax - x0, x1 - ax, ay - y0, y1 - ay};
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(p[i]) < 1e-6f) {
      if (q[i] < 0.0f) return false;  // parallel to this edge and outside it
    } else {
      const float t = q[i] / p[i];
      if (p[i] < 0.0f) {
        if (t > lo) lo = t;
      } else {
        if (t < hi) hi = t;
      }
    }
  }
  return hi >= lo;
}

// --- Shared low-level primitives (MapPrimitives.cpp) ---

// Strokes an open polyline emitting fixed-length dashes (screen-space), used
// for borders and the fuel-reserve ring.
void strokeDashedPolyline(Renderer& r, const Point* pts, int count,
                          float widthPx, const Color& c,
                          const ClipBounds* clip = nullptr);

// Railroad: a thin base line with periodic perpendicular crossties, matching
// the G1000 railroad symbol.
void drawRailroad(Renderer& r, const Point* pts, int count, const Color& c,
                  const ClipBounds* clip = nullptr);

// Rounded map-chrome box (the NXi "NORTH UP" / range / wind-vector plates):
// dark fill with a 1px light-gray border. The caller places the contents.
void drawChromeBox(Renderer& r, float x, float y, float w, float h);

// Boxed single-text chrome label ("NORTH UP", Fig 5-2/5-26: cyan text on the
// rounded plate). x/y is the top-left corner; returns the box height.
float drawChromeLabel(Renderer& r, float x, float y, const char* text,
                      float size, const Color& textColor);

// --- Per-layer painters (one component per translation unit) ---

// Land data: lakes filled, rivers/roads/borders stroked, with per-class range
// declutter. Drawn right above the map background so everything overlays it.
void drawLandData(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm);

// Populated places: a dot plus name, decluttered by city rank vs. range.
void drawCities(Renderer& r, const MapData& map, const Proj& proj,
                float rangeNm, float symSize, float labelSize);

// Airways declutter above their max range. Low-altitude routes draw first;
// high-altitude Jet/Q-routes draw on top when both are shown (Fig 5-15).
void drawAirways(Renderer& r, const MapData& map, const Proj& proj,
                 AirwayDisplay display, float rangeNm, float labelSize);

// Special-use and controlled airspace boundaries (Class B/C/D, restricted/MOA),
// with per-class range and altitude declutter.
void drawAirspaces(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, const FlightData& flight);

// Taxiway/apron diagram (SafeTaxi pavement), drawn under the runway quads.
void drawTaxiways(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm);

// Runway diagrams: filled pavement quads with runway-end numbers at close range.
void drawRunways(Renderer& r, const MapData& map, const Proj& proj,
                 float rangeNm, float labelSize);

// SafeTaxi taxiway identifier labels, on top of the pavement.
void drawTaxiwayLabels(Renderer& r, const MapData& map, const Proj& proj,
                       float rangeNm, float labelSize);

// Nav-feature symbols (airports/VOR/NDB/fix) with range/size declutter and
// identifier labels.
void drawNavFeatures(Renderer& r, const MapData& map, const Proj& proj,
                     const MapViewConfig& config, float rangeNm, float symSize,
                     float labelSize);

// Active flight-plan route (white, with the active leg magenta) and waypoint
// idents.
void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize, float labelSize);

// Procedure preview polyline (PROC menu): dashed cyan course through the
// published fixes.
void drawProcedurePreview(Renderer& r, const Proj& proj,
                          const MapViewConfig& config, float symSize,
                          float labelSize);

// Direct-To course: a magenta line from ownship to the direct-to waypoint.
void drawDirectToCourse(Renderer& r, const MapData& map, const Proj& proj,
                        const MapViewConfig& config, float symSize,
                        float labelSize);

// Obstacles (FAA DOF): tower symbol colored by proximity below ownship.
void drawObstacles(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, float ownAltFt, bool altValid, float symSize);

// Traffic overlay (TIS symbology): diamonds/circles with relative-altitude tags.
void drawTraffic(Renderer& r, const MapData& map, const Proj& proj,
                 float symSize, float labelSize);

// Track vector: a line projected along the current ground track.
void drawTrackVector(Renderer& r, const FlightData& flight, float ownX,
                     float ownY, float pixelsPerNm, float rotation);

// Fuel range ring: solid ring at total endurance, dashed ring at the reserve.
void drawFuelRing(Renderer& r, const FlightData& flight, float ownX, float ownY,
                  float pixelsPerNm, float viewRadiusPx);

// Ownship symbol, rotated to the aircraft heading in screen space.
void drawOwnshipSymbol(Renderer& r, float cx, float cy, float size,
                       float rotationDeg);

// Wind vector (Fig 5-18): a chrome plate in the upper right with a white arrow.
void drawWindVector(Renderer& r, const FlightData& flight,
                    const MapViewConfig& config, float rotation,
                    float labelSize);

// North indicator below the orientation label (Fig 5-26).
void drawNorthArrow(Renderer& r, float cx, float cy, float size,
                    float rotation);

// Range readout on the outer range ring / lower-right corner (Fig 5-2).
void drawRangeLabel(Renderer& r, float x, float y, float rangeNm,
                    float labelSize, bool centerOnPoint);

// Relative-terrain color legend, shown while TER REL is enabled.
void drawRelTerrainLegend(Renderer& r, const MapViewConfig& config,
                          float labelSize);

// A single range ring circle.
void drawRangeRing(Renderer& r, float cx, float cy, float radiusPx,
                   const Color& c);

}  // namespace avionics::mapview
