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

// NXi navigation-map base colors (sampled from the PC Trainer screenshots):
// deep navy ocean behind the land overlay, black continents, slightly dimmer
// inland lakes.
inline constexpr Color kMapOceanFill{0.0f, 0.0f, 0.52f, 1.0f};
inline constexpr Color kMapLandFill{0.0f, 0.0f, 0.0f, 1.0f};
inline constexpr Color kMapLakeFill{0.0f, 0.0f, 0.38f, 1.0f};

// Past this map range the navigation chart keeps only geography (land/ocean
// fills, nation outlines, major place names). Matches the Garmin PC Trainer
// at 1000 NM (MFD Default.bmp): no airports, navaids, or route symbology.
inline constexpr float kContinentalChartRangeNm = 500.0f;
// Maximum navigation-map range; only the largest country/region names remain.
inline constexpr float kWideChartRangeNm = 1000.0f;

// Nav-feature symbol size, in 768-px-canvas units. Symbols are sized off the
// display (like text) rather than the viewport, so they stay a fixed, legible
// size on both the small PFD inset and the full-screen MFD MAP page instead of
// ballooning on the larger viewport.
constexpr float kFeatureSymbolWt = 8.0f;
// High-res GSHHG land fill replaces coarse silhouettes at and below this range.
constexpr float kDetailLandMaxRangeNm = 50.0f;

// Obstacle symbols (Figs. 6-56 / 6-8): open-V tower/pole, 8-ray spark, turbine.
// Sized to match the PC Trainer 10 NM chart (≈14–22 px at 768 px display height).
// Sized against the PC Trainer / real NXi at 10 NM (lighted tower ≈ 16–25 px tall).
constexpr float kObstacleSymbolWt = 24.0f;

// Extra lift applied to ident labels so they clear the symbol geometry (trainer
// keeps a small gap between the icon apex and the text baseline).
constexpr float kMapLabelLiftPx = 15.0f;

// Map symbology labels: semi-bold to match the Garmin PC Trainer weight.
constexpr FontFace kMapLabelFace = FontFace::DejaVuSemiBold;
// Scale applied to labelFontWt for nav/city/route idents (was 0.80–0.85).
constexpr float kMapIdentLabelScale = 1.0f;
// Extra scale for ranked geo/hydro place names on the continental chart.
constexpr float kMapGeoLabelScale = 1.10f;

// Ownship airplane symbol size, in the same 768-px-canvas units. The G1000 NXi
// ownship icon is drawn noticeably larger than the nav-feature symbols so the
// aircraft stands out from the airports/navaids it overflies.
constexpr float kOwnshipSymbolWt = 15.0f;

// Margin (px) added around the viewport when clipping per-pixel symbology, so
// teeth/dashes near the edge are not cut early.
constexpr float kSymbologyClipMarginPx = 32.0f;

// On-screen span of the selected map range, as a fraction of viewport height.
// Taken from WT NextGenNavMapBuilder range endpoints (MFD) and MapInset (PFD):
// the range ring / compass arc sits at this radius, and the map scale is keyed
// to the same distance so features match the labeled range.
inline float mapRangeSpanFrac(MapOrientation orientation, float viewportHeightPx) {
  const bool inset = viewportHeightPx < 300.0f;
  switch (orientation) {
    case MapOrientation::NorthUp:
      return inset ? 0.40f : 0.25f;  // |0.5-0.1| inset, |0.5-0.25| MFD
    case MapOrientation::HeadingUp:
    case MapOrientation::TrackUp:
      return inset ? 0.51f : 0.34f;  // |0.67-0.16| inset, |0.67-0.33| MFD
  }
  return 0.25f;
}

inline float mapRangeSpanPx(const MapViewConfig& config) {
  return mapRangeSpanFrac(config.orientation, config.h) * config.h;
}

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

  // Per-frame Mercator projection constants, computed once in init().
  double cosR = 1.0;
  double sinR = 0.0;
  double mercatorYCenter = 0.0;
  float mercatorPxPerRad = 0.0f;

  void init() {
    const double rot = static_cast<double>(rotation) * map::kDegToRad;
    cosR = std::cos(rot);
    sinR = std::sin(rot);
    mercatorYCenter = map::mercatorYRad(centerLat);
    mercatorPxPerRad =
        pixelsPerNm * static_cast<float>(map::kNmPerEarthRad);
  }

  void toPx(double lat, double lon, float& x, float& y) const {
    double eastRad = 0.0;
    double northRad = 0.0;
    map::mercatorOffsetRad(lat, lon, centerLat, centerLon, eastRad, northRad);
    map::mercatorToScreen(eastRad, northRad, cx, cy, mercatorPxPerRad, cosR,
                          sinR, x, y);
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

// True when any vertex is inside the clip rect (expanded by margin) or any edge
// crosses it — needed for long border segments whose endpoints are off-screen.
inline bool polylineIntersectsClip(const Point* pts, int count,
                                   const ClipBounds& clip, float margin) {
  for (int i = 0; i < count; ++i) {
    const Point& p = pts[i];
    if (p.x >= clip.minX - margin && p.x <= clip.maxX + margin &&
        p.y >= clip.minY - margin && p.y <= clip.maxY + margin) {
      return true;
    }
  }
  for (int i = 0; i + 1 < count; ++i) {
    const Point& a = pts[i];
    const Point& b = pts[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) continue;
    float lo = 0.0f, hi = 0.0f;
    if (segmentVisibleSpan(a.x, a.y, dx / len, dy / len, len, clip.minX,
                           clip.minY, clip.maxX, clip.maxY, margin, lo, hi)) {
      return true;
    }
  }
  return false;
}

// Strokes only the portion of each edge that lies inside the clip rect.
void strokeClippedPolyline(Renderer& r, const Point* pts, int count,
                           float widthPx, const Color& c,
                           const ClipBounds& clip, float margin);

// Sutherland–Hodgman clip of a closed polygon to an axis-aligned rectangle.
// Used before land/lake fills so Mercator blow-up at high latitude cannot
// paint a wedge across the viewport when ring vertices sit far off-screen.
void clipPolygonToRect(const Point* pts, int count, const ClipBounds& clip,
                       float margin, std::vector<Point>& out);

// Continental landmass rings can project to tens of thousands of vertices after
// geo clip + Mercator subdivision; decimate before NanoVG tessellation.
inline constexpr std::size_t kMaxLandFillVerts = 4096;

void decimateClosedPolygon(const std::vector<Point>& in, std::size_t maxVerts,
                           std::vector<Point>& out);

void simplifyColinearRing(const std::vector<Point>& in, float areaEps,
                          std::vector<Point>& out);

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
// When a topo/rel terrain raster is showing at close range, skip only the
// high-vertex landmass detail so the DEM shoreline stays visible; coarse
// silhouettes and island fills still paint black under the terrain.
void drawLandData(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm, bool skipLandMassFill = false);

// Populated places: a dot plus name, decluttered by city rank vs. range.
void drawCities(Renderer& r, const MapData& map, const Proj& proj,
                float rangeNm, float symSize, float labelSize);

// City dots only (drawn with other symbology).
void drawCityDots(Renderer& r, const MapData& map, const Proj& proj, float rangeNm,
                  float symSize);

// Geo/city name labels (white/cyan, drawn on top of map symbology).
void drawMapPlaceLabels(Renderer& r, const MapData& map, const Proj& proj,
                        float rangeNm, float labelSize);

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

// Nav-feature symbols (airports/VOR/NDB/fix) with range/size declutter.
void drawNavFeatures(Renderer& r, const MapData& map, const Proj& proj,
                     const MapViewConfig& config, float rangeNm, float symSize);

// Nav-feature identifier labels (white, centered above symbols).
void drawNavFeatureLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, float rangeNm,
                          float symSize, float labelSize);

// Active flight-plan route (white, with the active leg magenta).
void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize);

// Flight-plan waypoint idents (white/magenta, centered above symbols).
void drawFlightPlanLabels(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, const FlightData& flight,
                          float symSize, float labelSize);

// Procedure preview polyline (PROC menu): dashed cyan course through the
// published fixes.
void drawProcedurePreview(Renderer& r, const Proj& proj,
                          const MapViewConfig& config, float symSize);

// Procedure preview fix idents (cyan, centered above symbols).
void drawProcedurePreviewLabels(Renderer& r, const Proj& proj,
                                const MapViewConfig& config, float symSize,
                                float labelSize);

// Direct-To course: a magenta line from ownship to the direct-to waypoint.
void drawDirectToCourse(Renderer& r, const MapData& map, const Proj& proj,
                          const MapViewConfig& config, float symSize);

// Direct-To waypoint ident (magenta, centered above symbol).
void drawDirectToCourseLabel(Renderer& r, const MapData& map, const Proj& proj,
                             const MapViewConfig& config, float symSize,
                             float labelSize);

// Obstacles (FAA DOF): open-V tower/pole, six-ray lighted spark, wind turbine.
// wind-turbine blades, and paired group symbols (Tables 6-7/6-8).
void drawObstacles(Renderer& r, const MapData& map, const Proj& proj,
                   float rangeNm, float maxRangeNm, float ownAltFt, bool altValid,
                   float symSize);

// Map-pointer selection tag (Fig. 6-56): boxed MSL/AGL below the obstacle.
void drawObstacleSelectedTag(Renderer& r, float x, float y, float symSize,
                             float mslFt, float aglFt, float labelSize,
                             const Color& c);

// Traffic overlay (TIS symbology): diamonds/circles with relative-altitude tags.
void drawTraffic(Renderer& r, const MapData& map, const Proj& proj,
                 float symSize, float labelSize, bool showLabels);

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

// A single range ring circle (north-up navigation map, Fig 5-2).
void drawRangeRing(Renderer& r, float cx, float cy, float radiusPx,
                   const Color& c);

// Range compass: fixed 120-degree arc with rotating bearing ticks (track-up /
// heading-up navigation map). Replaces the range ring when the map is not
// north-up, matching the real NXi and the WT MapRangeCompassController.
void drawRangeCompass(Renderer& r, const MapViewConfig& config,
                      const FlightData& flight, float cx, float cy,
                      float radiusPx, float rotationDeg, float labelSize,
                      const Color& c);

}  // namespace avionics::mapview
