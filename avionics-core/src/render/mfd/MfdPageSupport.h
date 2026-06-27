#pragma once

#include <string>
#include <vector>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/Renderer.h"
#include "avionics/render/MapView.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

inline constexpr const char* kDash = "_ _ _ _";
inline constexpr const char* kDashTime = "__:__";
inline constexpr char kDeg[] = "\xC2\xB0";  // UTF-8 degree sign

// MapAirspace::ceilingFt sentinel for "unlimited" (see MapData.h).
inline constexpr float kUnlimitedCeilingFt = 50000.0f;

// WT NXi list metrics: 28px rows of 20px text in the right-panel group boxes.
inline constexpr float kWtListRow = 28.0f;

std::string formatLatLon(double value, bool isLat);
std::string formatFrequency(MapFeatureType type, float frequency);
long eteSeconds(float distNm, float gsKts);
std::string formatDuration(long seconds);
std::string formatClock(int hour, int minute);

// G1000 region string for an X-Plane region code: the FAA enroute-chart
// region names for K1-K7 ("N CEN USA"), country names for a few common ICAO
// prefixes, otherwise the raw code.
std::string regionName(const std::string& code);
// "Hard Surface" / "Turf" / ... (Pilot's Guide Airport Information vocabulary).
const char* runwaySurfaceName(RunwaySurface surface);
// "Public" / "Private" / "Heliport" usage type for the Airport box header.
const char* airportUsageType(const MapFeature& apt);
// "Terminal" / "Low Altitude" / "High Altitude" from the service-volume range.
const char* vorClassName(int rangeNm);
// Slaved variation as the G1000 shows it ("8°E"), empty when unknown.
std::string formatMagvar(const MapFeature& navaid);

// Sunrise / sunset UTC (decimal hours, 0..24) for a 1-based day of year and a
// location, using the NOAA solar-position approximation with the standard
// refraction zenith (90.833 deg). Returns false for polar day/night (no
// sunrise/sunset event that day) or an unknown date (dayOfYear < 1).
bool sunriseSunsetUtc(int dayOfYear, double latDeg, double lonDeg,
                      double& riseUtcHours, double& setUtcHours);

const MapFeature* nearestFeature(const MapData& map, MapFeatureType type);

struct NearRow {
  const MapFeature* feature = nullptr;
  float bearingDeg = 0.0f;
  float distanceNm = 0.0f;
};

std::vector<NearRow> collectNearest(const MapData& map, MapFeatureType type,
                                    int maxRows);

struct PageFrame {
  Rect map;
  Rect panel;
};

PageFrame beginPanelPage(Renderer& r, float x, float y, float w, float h,
                         bool widePanel);

struct PanelStack {
  Rect panel;
  float displayH;
  float y;

  PanelStack(const Rect& p, float dh)
      : panel(p), displayH(dh), y(p.y + mfdFontPx(10.0f, dh)) {}

  float px(float v) const { return mfdFontPx(v, displayH); }

  Rect slot(float wtHeight) {
    const Rect s{panel.x + px(5.0f), y, panel.w - 2.0f * px(5.0f),
                 px(wtHeight)};
    y += px(wtHeight) + px(10.0f);
    return s;
  }

  float remainingWt(float reserveWt = 0.0f) const {
    return (panel.y + panel.h - y) / px(1.0f) - 10.0f - reserveWt;
  }
};

// Sentinel for drawPageMap's optional explicit view center: when left at this
// value the view centers on `center` (or the ownship if `center` is null).
inline constexpr double kNoViewCenter = 1.0e9;

// PROC -> Select Approach preview auto-fit. The framed procedure (legs + its
// airport) should fill at most this fraction of the available half-extent along
// each axis, leaving the rest as margin so the airport and legs sit clear of
// the viewport edges.
inline constexpr float kProcPreviewFillFrac = 0.7f;

// Auto-fit framing for the procedure preview shown while PROC -> Select is open
// (Pilot's Guide 5.8): the center and the smallest range-ladder step that frame
// the highlighted procedure's legs together with its airport. Shared by the
// navigation Map page and the FPL inset so both zoom to display the whole
// approach. The fit honors the usable viewport size (`usableWPx`/`usableHPx`)
// so the narrower FPL inset -- and the Map page's window-occluded width -- zoom
// out enough to fit the procedure horizontally as well as vertically. `airport`
// is included in the bounds when non-null so the field is always visible.
// `valid` is false when there are too few legs to frame (size < 2).
struct ProcPreviewFit {
  double centerLat = 0.0;
  double centerLon = 0.0;
  int ladderIndex = 0;
  bool valid = false;
};
ProcPreviewFit procPreviewMapFit(const std::vector<MapLeg>& legs,
                                 const MapFeature* airport, float usableWPx,
                                 float usableHPx);

void drawPageMap(Renderer& r, const FlightData& d, const MapData& map,
                 const Rect& area, float rangeNm, const MapFeature* center,
                 float displayH, bool showFixes = false,
                 const std::vector<MapLeg>* procedurePreview = nullptr,
                 float displayRangeNm = 0.0f,
                 TerrainDisplay terrain = TerrainDisplay::Topo,
                 bool useInsetMapData = false,
                 AirwayDisplay airways = AirwayDisplay::Off,
                 bool showWeather = false,
                 double viewCenterLat = kNoViewCenter,
                 double viewCenterLon = kNoViewCenter);

// Direct-To inset map framing: the view centers on the Direct-To target and
// zooms in tight (kDirectToInsetRangeNm) so the destination and its immediate
// surroundings fill the panel, like the WPT Information insets. The ownship may
// sit off-screen for distant targets; the destination detail is what matters.
struct DirectToInsetView {
  double centerLat = 0.0;
  double centerLon = 0.0;
  float rangeNm = kDirectToInsetRangeNm;
  bool centeredOnWaypoint = true;
};
bool mapFeatureHasGeo(const MapFeature& feature);
MapFeature resolveWaypointGeo(const MapData& map, const MapFeature& wpt);

DirectToInsetView directToInsetView(const MapData& map, const MapFeature& wpt);

float directToInsetRangeNm(const MapData& map, const MapFeature& wpt);
float directToInsetViewHalfExtentNm(float rangeNm, float viewportWPx = 0.0f,
                                    float viewportHPx = 0.0f);

void drawWaypointIcon(Renderer& r, float cx, float cy, float size,
                      const MapFeature* feature, MapFeatureType type);
void drawSelectArrow(Renderer& r, float x, float cy, float size);

// Draws the FMS identifier entry cells starting at startX, baseline cy: the
// cursor cell as a pulsing highlight-select plate, the spell-ahead fill in
// cyan, and the typed characters in white. Shared by the FPL insert / Direct-To
// windows, the PROC airport field, and the Charts airport field so the ident
// entry looks and behaves identically everywhere. Returns the x just past the
// last cell (for placing the waypoint symbol).
float drawIdentEntryCells(Renderer& r, float startX, float cy,
                          const std::string& ident, int cursor, int typedCount,
                          bool selectAll, bool blinkOn, float displayH,
                          float fontSpec = kWtIdentLarge);

float drawFacilityHeader(Renderer& r, const Rect& area, const MapFeature* f,
                         MapFeatureType type, float displayH);
// Facility name and (when known) city rows in cyan below the header (the
// Airport/NDB/VOR box per Fig 5-26/5-33/5-35). Returns the y below the rows.
float drawFacilityNameCity(Renderer& r, const Rect& area, float y,
                           const MapFeature* f, float displayH);
// maxRows: 4 = designation/dimensions/surface/lighting (WPT, Fig 5-26),
// 2 = designation/dimensions only (NRST, Fig 5-30).
void drawRunwayGroup(Renderer& r, const Rect& area,
                     const std::vector<AirportRunwayInfo>& runways,
                     int selectedIndex, int maxRows, float displayH);
void drawFrequencyGroup(Renderer& r, const Rect& area, float displayH,
                        int maxRows,
                        const std::vector<MapAirportFrequency>& frequencies);
void drawApproachesGroup(Renderer& r, const Rect& area, float displayH,
                         const std::vector<MapProcedure>& procedures);
void drawNearestRows(Renderer& r, const Rect& area,
                     const std::vector<NearRow>& rows, int selected,
                     MapFeatureType type, float displayH);

// Word-wrap `text` left-aligned inside `area`, breaking on spaces, at `rowSize`
// text with `lineH` row spacing, starting at baseline `startY`. Stops at the
// bottom of `area`. Returns the baseline y below the last drawn line. Shared by
// the OFP route box and the WPT - Weather Information METAR/TAF boxes.
float drawWrappedText(Renderer& r, const Rect& area, float startY,
                      const std::string& text, float rowSize, float lineH,
                      const Color& color);

const char* airspaceClassName(AirspaceClass c);
bool insideBoundary(const std::vector<GeoPoint>& ring, double lat, double lon);
double distanceToBoundaryNm(const MapAirspace& airspace, double lat,
                            double lon);
std::string formatAirspaceAltitude(float ft, bool isCeiling);

}  // namespace avionics::mfd
