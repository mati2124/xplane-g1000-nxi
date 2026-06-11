#pragma once

#include <string>
#include <vector>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
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

void drawPageMap(Renderer& r, const FlightData& d, const MapData& map,
                 const Rect& area, float rangeNm, const MapFeature* center,
                 float displayH, bool showFixes = false,
                 const std::vector<MapLeg>* procedurePreview = nullptr);

void drawWaypointIcon(Renderer& r, float cx, float cy, float size,
                      const MapFeature* feature, MapFeatureType type);
void drawSelectArrow(Renderer& r, float x, float cy, float size);

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

const char* airspaceClassName(AirspaceClass c);
bool insideBoundary(const std::vector<GeoPoint>& ring, double lat, double lon);
double distanceToBoundaryNm(const MapAirspace& airspace, double lat,
                            double lon);
std::string formatAirspaceAltitude(float ft, bool isCeiling);

}  // namespace avionics::mfd
