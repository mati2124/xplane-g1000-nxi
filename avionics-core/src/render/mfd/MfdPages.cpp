#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/NavMath.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {
namespace {

constexpr char kDeg[] = "\xC2\xB0";  // UTF-8 degree sign
constexpr const char* kDash = "_ _ _ _";
constexpr const char* kDashTime = "__:__";

// MapAirspace::ceilingFt sentinel for "unlimited" (see MapData.h).
constexpr float kUnlimitedCeilingFt = 50000.0f;

std::string formatLatLon(double value, bool isLat) {
  const char hemi = isLat ? (value >= 0.0 ? 'N' : 'S')
                          : (value >= 0.0 ? 'E' : 'W');
  const double a = std::fabs(value);
  const int deg = static_cast<int>(a);
  const double minutes = (a - deg) * 60.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%c%0*d%s%05.2f'", hemi, isLat ? 2 : 3, deg,
                kDeg, minutes);
  return buf;
}

// Navaid tuning frequency: VORs in MHz (113.90), NDBs in kHz (362.0).
std::string formatFrequency(MapFeatureType type, float frequency) {
  if (frequency <= 0.0f) return kDash;
  char buf[16];
  std::snprintf(buf, sizeof(buf), type == MapFeatureType::Vor ? "%.2f" : "%.1f",
                static_cast<double>(frequency));
  return buf;
}

// ETE in seconds to cover distNm at gsKts, or -1 when no meaningful estimate
// exists (the G1000 dashes time fields below ~30 kt ground speed).
long eteSeconds(float distNm, float gsKts) {
  if (gsKts < 30.0f || distNm <= 0.0f) return -1;
  return std::lround(distNm / gsKts * 3600.0f);
}

// H:MM above an hour, MM:SS below, dashes when invalid -- the Trip Planning
// ETE format (G1000 Pilot's Guide for Cessna Nav III, Section 5.9).
std::string formatDuration(long seconds) {
  if (seconds < 0) return kDashTime;
  char buf[16];
  if (seconds >= 3600) {
    std::snprintf(buf, sizeof(buf), "%ld:%02ld", seconds / 3600,
                  (seconds % 3600) / 60);
  } else {
    std::snprintf(buf, sizeof(buf), "%02ld:%02ld", seconds / 60, seconds % 60);
  }
  return buf;
}

std::string formatClock(int hour, int minute) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute);
  return buf;
}

const MapFeature* nearestFeature(const MapData& map, MapFeatureType type) {
  const MapFeature* best = nullptr;
  double bestSq = 1e18;
  if (!map.positionValid) return nullptr;
  for (const MapFeature& f : map.features) {
    if (f.type != type) continue;
    const double n = (f.lat - map.ownshipLat) * 60.0;
    const double e = (f.lon - map.ownshipLon) * 60.0 *
                     std::cos(map.ownshipLat * 3.14159265358979323846 / 180.0);
    const double sq = n * n + e * e;
    if (sq < bestSq) {
      bestSq = sq;
      best = &f;
    }
  }
  return best;
}

struct NearRow {
  const MapFeature* feature = nullptr;
  float bearingDeg = 0.0f;
  float distanceNm = 0.0f;
};

std::vector<NearRow> collectNearest(const MapData& map, MapFeatureType type,
                                    int maxRows) {
  std::vector<NearRow> rows;
  if (!map.positionValid) return rows;
  for (const MapFeature& f : map.features) {
    if (f.type != type) continue;
    rows.push_back({&f,
                    static_cast<float>(navBearingDeg(map.ownshipLat,
                                                     map.ownshipLon, f.lat,
                                                     f.lon)),
                    static_cast<float>(navDistanceNm(map.ownshipLat,
                                                     map.ownshipLon, f.lat,
                                                     f.lon))});
  }
  std::sort(rows.begin(), rows.end(),
            [](const NearRow& a, const NearRow& b) {
              return a.distanceNm < b.distanceNm;
            });
  if (static_cast<int>(rows.size()) > maxRows) rows.resize(maxRows);
  return rows;
}

// Embeds a MapView inside a window content rect (no MapView chrome -- the window
// frame provides the border), optionally centered on a selected feature.
// showFixes is enabled for pages whose subject is an intersection (otherwise
// the close-range declutter would hide the very waypoint the window shows).
void drawEmbeddedMap(Renderer& r, const FlightData& d, const MapData& map,
                     const Rect& area, float rangeNm, const MapFeature* center,
                     float displayH, bool showFixes = false) {
  MapViewConfig cfg;
  cfg.x = area.x;
  cfg.y = area.y;
  cfg.w = area.w;
  cfg.h = area.h;
  cfg.orientation = MapOrientation::NorthUp;
  cfg.rangeNm = rangeNm;
  cfg.style.showChrome = false;
  cfg.style.showTerrain = true;
  cfg.style.showFixes = showFixes;
  cfg.style.labelFontWt = 13.0f;
  if (center != nullptr) {
    cfg.hasCenterOverride = true;
    cfg.centerLat = center->lat;
    cfg.centerLon = center->lon;
  }
  r.fillRect(area.x, area.y, area.w, area.h, colors::kBlack);
  MapView::render(r, map, d, cfg, displayH);

  char buf[16];
  std::snprintf(buf, sizeof(buf), "%dNM", static_cast<int>(std::lround(rangeNm)));
  r.fillText(area.x + area.w * 0.04f, area.y + area.h * 0.07f, buf,
             mfdFontPx(kWtHeader, displayH), TextAlign::Left, colors::kWhite);
}

void drawNearestList(Renderer& r, const Rect& area, const char* idHeader,
                     const std::vector<NearRow>& rows, int selected,
                     float displayH) {
  const float rowH = area.h / 6.0f;
  float y = area.y;

  // Column header.
  const float hSize = mfdFontPx(kWtHeader, displayH);
  r.fillText(area.x, y + rowH * 0.5f, idHeader, hSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(area.x + area.w * 0.66f, y + rowH * 0.5f, "BRG", hSize,
             TextAlign::Right, colors::kLabelText);
  r.fillText(area.x + area.w, y + rowH * 0.5f, "DIST", hSize, TextAlign::Right,
             colors::kLabelText);
  y += rowH;

  const float rowSize = mfdFontPx(kWtRow, displayH);
  char buf[24];
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const NearRow& row = rows[i];
    const float cy = y + rowH * 0.5f;
    if (static_cast<int>(i) == selected) {
      r.fillRect(area.x - area.w * 0.02f, y, area.w * 1.04f, rowH,
                 mfdAlpha(colors::kCyan, 0.16f));
    }
    const Color idColor =
        static_cast<int>(i) == selected ? colors::kCyan : colors::kWhite;
    r.fillText(area.x, cy, row.feature->id, rowSize, TextAlign::Left, idColor);
    std::snprintf(buf, sizeof(buf), "%03.0f%s",
                  static_cast<double>(row.bearingDeg), kDeg);
    r.fillText(area.x + area.w * 0.66f, cy, buf, rowSize, TextAlign::Right,
               colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fNM",
                  static_cast<double>(row.distanceNm));
    r.fillText(area.x + area.w, cy, buf, rowSize, TextAlign::Right,
               colors::kWhite);
    y += rowH;
  }
  if (rows.empty()) {
    r.fillText(area.x + area.w * 0.5f, y + rowH, "NONE WITHIN RANGE", rowSize,
               TextAlign::Center, colors::kLabelText);
  }
}

void drawAirportInfoFields(Renderer& r, const Rect& area,
                           const MapFeature* apt, float displayH) {
  if (apt == nullptr) {
    r.fillText(area.x + area.w * 0.5f, area.y + area.h * 0.4f, "NO AIRPORT",
               mfdFontPx(kWtRow, displayH), TextAlign::Center,
               colors::kLabelText);
    return;
  }
  // Large ident + region.
  r.fillText(area.x, area.y + mfdFontPx(kWtIdentLarge, displayH) * 0.5f, apt->id,
             mfdFontPx(kWtIdentLarge, displayH), TextAlign::Left,
             colors::kWhite);
  r.fillText(area.x + area.w, area.y + mfdFontPx(kWtIdentLarge, displayH) * 0.5f,
             apt->region.empty() ? kDash : apt->region,
             mfdFontPx(kWtFieldValue, displayH), TextAlign::Right,
             colors::kLabelText);

  const float rowH = (area.h - mfdFontPx(kWtIdentLarge, displayH)) / 5.0f;
  float y = area.y + mfdFontPx(kWtIdentLarge, displayH) * 1.1f;
  char buf[24];

  y = drawField(r, area, y, rowH, "LAT", formatLatLon(apt->lat, true), displayH,
                colors::kWhite);
  y = drawField(r, area, y, rowH, "LON", formatLatLon(apt->lon, false),
                displayH, colors::kWhite);
  std::snprintf(buf, sizeof(buf), "%dFT", static_cast<int>(apt->elevationFt));
  y = drawField(r, area, y, rowH, "ELEVATION",
                apt->elevationFt != 0.0f ? buf : kDash, displayH,
                colors::kWhite);
  y = drawField(r, area, y, rowH, "FUEL", kDash, displayH, colors::kWhite);
}

void drawRunwayFields(Renderer& r, const Rect& area, const MapFeature* apt,
                      float displayH) {
  const float rowH = area.h / 3.0f;
  float y = area.y;
  char buf[24];
  if (apt != nullptr && apt->longestRunwayFt > 0) {
    std::snprintf(buf, sizeof(buf), "%dFT", apt->longestRunwayFt);
    y = drawField(r, area, y, rowH, "LONGEST", buf, displayH, colors::kWhite);
  } else {
    y = drawField(r, area, y, rowH, "LONGEST", kDash, displayH, colors::kWhite);
  }
  y = drawField(r, area, y, rowH, "SURFACE", kDash, displayH, colors::kWhite);
  y = drawField(r, area, y, rowH, "LIGHTING", kDash, displayH, colors::kWhite);
}

void drawFrequencyFields(Renderer& r, const Rect& area, float displayH) {
  const float rowH = area.h / 4.0f;
  float y = area.y;
  y = drawField(r, area, y, rowH, "TOWER", kDash, displayH, colors::kWhite);
  y = drawField(r, area, y, rowH, "GROUND", kDash, displayH, colors::kWhite);
  y = drawField(r, area, y, rowH, "ATIS", kDash, displayH, colors::kWhite);
  y = drawField(r, area, y, rowH, "CTAF", kDash, displayH, colors::kWhite);
}

const char* airspaceClassName(AirspaceClass c) {
  switch (c) {
    case AirspaceClass::ClassB:
      return "CLASS B";
    case AirspaceClass::ClassC:
      return "CLASS C";
    case AirspaceClass::ClassD:
      return "CLASS D";
    case AirspaceClass::Restricted:
      return "RESTRICTED";
    case AirspaceClass::Prohibited:
      return "PROHIBITED";
    case AirspaceClass::Danger:
      return "DANGER";
    case AirspaceClass::Other:
      break;
  }
  return "OTHER";
}

// Even-odd ray cast in lat/lon space; fine at airspace-boundary scales.
bool insideBoundary(const std::vector<GeoPoint>& ring, double lat, double lon) {
  bool inside = false;
  const std::size_t n = ring.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const GeoPoint& a = ring[i];
    const GeoPoint& b = ring[j];
    if ((a.lat > lat) != (b.lat > lat) &&
        lon < (b.lon - a.lon) * (lat - a.lat) / (b.lat - a.lat) + a.lon) {
      inside = !inside;
    }
  }
  return inside;
}

double distanceToBoundaryNm(const MapAirspace& airspace, double lat,
                            double lon) {
  double best = 1e18;
  for (const GeoPoint& p : airspace.boundary) {
    best = std::min(best, navDistanceNm(lat, lon, p.lat, p.lon));
  }
  return best;
}

std::string formatAirspaceAltitude(float ft, bool isCeiling) {
  if (isCeiling && ft >= kUnlimitedCeilingFt) return "UNLIMITED";
  if (!isCeiling && ft <= 0.0f) return "GROUND";
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%dFT", static_cast<int>(ft));
  return buf;
}

}  // namespace

void drawMapPage(Renderer& r, const FlightData& d, const MapData& map,
                 const MfdController& ui, float x, float y, float w, float h,
                 float displayH) {
  MapViewConfig config;
  config.x = x;
  config.y = y;
  config.w = w;
  config.h = h;
  config.orientation = ui.mapOrientation();
  config.rangeNm = ui.rangeNm();
  config.style.showChrome = true;
  config.style.showTerrain = ui.showTerrain();
  config.style.labelFontWt = 16.0f;
  MapView::render(r, map, d, config, displayH);
}

void drawWaypointPage(Renderer& r, const FlightData& d, const MapData& map,
                      float x, float y, float w, float h, float displayH) {
  r.fillRect(x, y, w, h, colors::kBlack);
  const MapFeature* apt = nearestFeature(map, MapFeatureType::Airport);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.5f;
  const float rightW = leftW;
  const float rx = x + pad + leftW + colGap;

  // Left column: AIRPORT INFORMATION over RUNWAYS.
  const float infoH = (h - 2.0f * pad) * 0.6f;
  Rect infoBox{x + pad, y + pad, leftW, infoH};
  Rect infoInner = drawWindow(r, infoBox, "AIRPORT", displayH);
  drawAirportInfoFields(r, infoInner, apt, displayH);

  Rect rwyBox{x + pad, y + pad + infoH + pad, leftW,
              h - 3.0f * pad - infoH};
  Rect rwyInner = drawWindow(r, rwyBox, "RUNWAYS", displayH);
  drawRunwayFields(r, rwyInner, apt, displayH);

  // Right column: MAP over FREQUENCIES.
  const float mapH = (h - 2.0f * pad) * 0.6f;
  Rect mapBox{rx, y + pad, rightW, mapH};
  Rect mapInner = drawWindow(r, mapBox, "MAP", displayH);
  drawEmbeddedMap(r, d, map, mapInner, 5.0f, apt, displayH);

  Rect freqBox{rx, y + pad + mapH + pad, rightW, h - 3.0f * pad - mapH};
  Rect freqInner = drawWindow(r, freqBox, "FREQUENCIES", displayH);
  drawFrequencyFields(r, freqInner, displayH);
}

void drawWaypointNavaidPage(Renderer& r, const FlightData& d,
                            const MapData& map, MapFeatureType type, float x,
                            float y, float w, float h, float displayH) {
  // Intersection/NDB/VOR Information layout: identifier window and reference
  // data on the left, map centered on the waypoint on the right (G1000
  // Pilot's Guide for Cessna Nav III, Section 5.5).
  r.fillRect(x, y, w, h, colors::kBlack);
  const bool isIntersection = type == MapFeatureType::Fix;
  const char* subject = isIntersection ? "INTERSECTION"
                        : type == MapFeatureType::Ndb ? "NDB"
                                                      : "VOR";
  const MapFeature* navaid = nearestFeature(map, type);

  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.5f;
  const float rightW = leftW;
  const float rx = x + pad + leftW + colGap;

  // Left column: the waypoint window, then FREQUENCY (navaids) or the nearest
  // reference VOR (intersections).
  const float infoH = (h - 2.0f * pad) * 0.55f;
  Rect infoInner =
      drawWindow(r, Rect{x + pad, y + pad, leftW, infoH}, subject, displayH);
  if (navaid == nullptr) {
    r.fillText(infoInner.x + infoInner.w * 0.5f,
               infoInner.y + infoInner.h * 0.4f, "NO WAYPOINT",
               mfdFontPx(kWtRow, displayH), TextAlign::Center,
               colors::kLabelText);
  } else {
    const float identSize = mfdFontPx(kWtIdentLarge, displayH);
    r.fillText(infoInner.x, infoInner.y + identSize * 0.5f, navaid->id,
               identSize, TextAlign::Left, colors::kWhite);
    const float rowH = (infoInner.h - identSize) / 4.0f;
    float fy = infoInner.y + identSize * 1.1f;
    fy = drawField(r, infoInner, fy, rowH, "LAT",
                   formatLatLon(navaid->lat, true), displayH, colors::kWhite);
    fy = drawField(r, infoInner, fy, rowH, "LON",
                   formatLatLon(navaid->lon, false), displayH, colors::kWhite);
    if (map.positionValid) {
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%03.0f%s",
                    navBearingDeg(map.ownshipLat, map.ownshipLon, navaid->lat,
                                  navaid->lon),
                    kDeg);
      fy = drawField(r, infoInner, fy, rowH, "BRG", buf, displayH,
                     colors::kWhite);
      std::snprintf(buf, sizeof(buf), "%.1fNM",
                    navDistanceNm(map.ownshipLat, map.ownshipLon, navaid->lat,
                                  navaid->lon));
      fy = drawField(r, infoInner, fy, rowH, "DIS", buf, displayH,
                     colors::kWhite);
    }
  }

  Rect lowerBox{x + pad, y + pad + infoH + pad, leftW, h - 3.0f * pad - infoH};
  if (isIntersection) {
    // Reference VOR for the intersection (ident, bearing, distance).
    Rect refInner = drawWindow(r, lowerBox, "NEAREST VOR", displayH);
    const MapFeature* vor = nearestFeature(map, MapFeatureType::Vor);
    const float rowH = refInner.h / 4.0f;
    float fy = refInner.y;
    if (vor != nullptr && navaid != nullptr) {
      char buf[24];
      fy = drawField(r, refInner, fy, rowH, "VOR", vor->id, displayH,
                     colors::kWhite);
      fy = drawField(r, refInner, fy, rowH, "FREQUENCY",
                     formatFrequency(MapFeatureType::Vor, vor->frequency),
                     displayH, colors::kCyan);
      std::snprintf(buf, sizeof(buf), "%03.0f%s",
                    navBearingDeg(navaid->lat, navaid->lon, vor->lat, vor->lon),
                    kDeg);
      fy = drawField(r, refInner, fy, rowH, "BRG", buf, displayH,
                     colors::kWhite);
      std::snprintf(buf, sizeof(buf), "%.1fNM",
                    navDistanceNm(navaid->lat, navaid->lon, vor->lat, vor->lon));
      fy = drawField(r, refInner, fy, rowH, "DIS", buf, displayH,
                     colors::kWhite);
    } else {
      fy = drawField(r, refInner, fy, rowH, "VOR", kDash, displayH,
                     colors::kWhite);
    }
  } else {
    Rect freqInner = drawWindow(r, lowerBox, "FREQUENCY", displayH);
    const float rowH = freqInner.h / 4.0f;
    drawField(r, freqInner, freqInner.y, rowH, "FREQUENCY",
              navaid != nullptr ? formatFrequency(type, navaid->frequency)
                                : std::string(kDash),
              displayH, colors::kCyan);
  }

  // Right column: map centered on the waypoint.
  Rect mapInner =
      drawWindow(r, Rect{rx, y + pad, rightW, h - 2.0f * pad}, "MAP", displayH);
  drawEmbeddedMap(r, d, map, mapInner, isIntersection ? 10.0f : 25.0f, navaid,
                  displayH, isIntersection);
}

void drawTripPlanningPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH) {
  // AUX Trip Planning, automatic page mode for the remaining active leg
  // (present position -> active waypoint): input data + leg preview on top,
  // TRIP / FUEL / OTHER STATS along the bottom (G1000 Pilot's Guide for
  // Cessna Nav III, Section 5.9). Fields whose inputs this suite has no
  // source for (ESA, sunrise/sunset) are dashed, as the real unit dashes
  // stats it cannot compute.
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  char buf[32];

  const bool linkValid = d.dataLinkValid;
  const bool hasLeg = linkValid && !d.fmaToWpt.empty();
  const float gs = d.groundSpeedKts;
  const float distNm = d.fmaLegDistanceNm;
  const long ete = hasLeg ? eteSeconds(distNm, gs) : -1;
  const float fuelOnBoard = d.fuelQtyLeftGal + d.fuelQtyRightGal;
  const bool fuelFlowValid = linkValid && d.fuelFlowGph > 0.1f;

  // Top row: INPUT DATA (left) and the leg preview map (right).
  const float topH = (h - 3.0f * pad) * 0.55f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.5f;
  Rect inputInner = drawWindow(r, Rect{x + pad, y + pad, leftW, topH},
                               "INPUT DATA", displayH);
  {
    const float rowH = inputInner.h / 6.0f;
    float fy = inputInner.y;
    fy = drawField(r, inputInner, fy, rowH, "PAGE MODE", "AUTOMATIC", displayH,
                   colors::kCyan);
    fy = drawField(r, inputInner, fy, rowH, "DEP TIME",
                   linkValid ? formatClock(d.utcHour, d.utcMinute) + " UTC"
                             : std::string(kDashTime),
                   displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%dKT", static_cast<int>(std::lround(gs)));
    fy = drawField(r, inputInner, fy, rowH, "GS",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fGPH",
                  static_cast<double>(d.fuelFlowGph));
    fy = drawField(r, inputInner, fy, rowH, "FUEL FLOW",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fGAL",
                  static_cast<double>(fuelOnBoard));
    fy = drawField(r, inputInner, fy, rowH, "FUEL ONBOARD",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(d.airspeedKts)));
    fy = drawField(r, inputInner, fy, rowH, "CALIBRATED AS",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
  }

  // Selected leg annotation + preview map of the remaining active leg.
  const float rx = x + pad + leftW + colGap;
  const float rightW = w - 2.0f * pad - colGap - leftW;
  const std::string legTitle =
      hasLeg ? "P.POS \xE2\x86\x92 " + d.fmaToWpt : "NO ACTIVE LEG";
  Rect mapInner = drawWindow(r, Rect{rx, y + pad, rightW, topH},
                             legTitle.c_str(), displayH);
  drawEmbeddedMap(r, d, map, mapInner,
                  std::max(10.0f, std::min(100.0f, distNm * 1.4f)), nullptr,
                  displayH);

  // Bottom row: TRIP STATS / FUEL STATS / OTHER STATS.
  const float statsY = y + pad + topH + pad;
  const float statsH = h - statsY + y - pad;
  const float statsW = (w - 2.0f * pad - 2.0f * colGap) / 3.0f;

  Rect tripInner = drawWindow(r, Rect{x + pad, statsY, statsW, statsH},
                              "TRIP STATS", displayH);
  {
    const float rowH = tripInner.h / 7.0f;
    float fy = tripInner.y;
    const bool gpsDtk = hasLeg && d.cdiSource == CdiSource::Gps;
    if (gpsDtk) {
      std::snprintf(buf, sizeof(buf), "%03d%s",
                    static_cast<int>(std::lround(d.courseDeg)), kDeg);
    }
    fy = drawField(r, tripInner, fy, rowH, "DTK", gpsDtk ? buf : kDash,
                   displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fNM", static_cast<double>(distNm));
    fy = drawField(r, tripInner, fy, rowH, "DIS", hasLeg ? buf : kDash,
                   displayH, colors::kWhite);
    fy = drawField(r, tripInner, fy, rowH, "ETE", formatDuration(ete), displayH,
                   colors::kWhite);
    std::string eta = kDashTime;
    if (ete >= 0) {
      const long arrive =
          (d.utcHour * 3600L + d.utcMinute * 60L + d.utcSecond + ete) % 86400L;
      eta = formatClock(static_cast<int>(arrive / 3600),
                        static_cast<int>((arrive % 3600) / 60)) +
            " UTC";
    }
    fy = drawField(r, tripInner, fy, rowH, "ETA", eta, displayH,
                   colors::kWhite);
    fy = drawField(r, tripInner, fy, rowH, "ESA", kDash, displayH,
                   colors::kWhite);
    fy = drawField(r, tripInner, fy, rowH, "SUNRISE", kDashTime, displayH,
                   colors::kWhite);
    fy = drawField(r, tripInner, fy, rowH, "SUNSET", kDashTime, displayH,
                   colors::kWhite);
  }

  Rect fuelInner =
      drawWindow(r, Rect{x + pad + statsW + colGap, statsY, statsW, statsH},
                 "FUEL STATS", displayH);
  {
    const float rowH = fuelInner.h / 6.0f;
    float fy = fuelInner.y;
    const long endurance =
        fuelFlowValid
            ? std::lround(fuelOnBoard / d.fuelFlowGph * 3600.0f)
            : -1;
    const float fuelReq =
        fuelFlowValid && ete >= 0
            ? d.fuelFlowGph * static_cast<float>(ete) / 3600.0f
            : -1.0f;

    if (fuelFlowValid && gs >= 30.0f) {
      std::snprintf(buf, sizeof(buf), "%.1fNM/GAL",
                    static_cast<double>(gs / d.fuelFlowGph));
    }
    fy = drawField(r, fuelInner, fy, rowH, "EFFICIENCY",
                   fuelFlowValid && gs >= 30.0f ? buf : kDash, displayH,
                   colors::kWhite);
    fy = drawField(r, fuelInner, fy, rowH, "TOTAL ENDUR",
                   formatDuration(endurance), displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fGAL",
                  static_cast<double>(fuelOnBoard - fuelReq));
    fy = drawField(r, fuelInner, fy, rowH, "REM FUEL",
                   fuelReq >= 0.0f ? buf : kDash, displayH, colors::kWhite);
    fy = drawField(r, fuelInner, fy, rowH, "REM ENDUR",
                   endurance >= 0 && ete >= 0
                       ? formatDuration(endurance - ete)
                       : std::string(kDashTime),
                   displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%.1fGAL", static_cast<double>(fuelReq));
    fy = drawField(r, fuelInner, fy, rowH, "FUEL REQ",
                   fuelReq >= 0.0f ? buf : kDash, displayH, colors::kWhite);
    if (endurance >= 0 && gs >= 30.0f) {
      std::snprintf(buf, sizeof(buf), "%.0fNM",
                    static_cast<double>(gs * endurance / 3600.0f));
    }
    fy = drawField(r, fuelInner, fy, rowH, "TOTAL RANGE",
                   endurance >= 0 && gs >= 30.0f ? buf : kDash, displayH,
                   colors::kWhite);
  }

  Rect otherInner = drawWindow(
      r, Rect{x + pad + 2.0f * (statsW + colGap), statsY, statsW, statsH},
      "OTHER STATS", displayH);
  {
    const float rowH = otherInner.h / 6.0f;
    float fy = otherInner.y;
    // Density altitude from pressure altitude and the ISA temperature delta.
    const float pressureAltFt =
        d.altitudeFt + (29.92f - d.baroSettingInHg) * 1000.0f;
    const float isaTempC = 15.0f - 1.98f * d.altitudeFt / 1000.0f;
    const float densityAltFt =
        pressureAltFt + 118.8f * (d.oatCelsius - isaTempC);
    std::snprintf(buf, sizeof(buf), "%dFT",
                  static_cast<int>(std::lround(densityAltFt / 10.0f) * 10));
    fy = drawField(r, otherInner, fy, rowH, "DENSITY ALT",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(d.tasKts)));
    fy = drawField(r, otherInner, fy, rowH, "TRUE AIRSPEED",
                   linkValid ? buf : kDash, displayH, colors::kWhite);
  }
}

void drawGpsStatusPage(Renderer& r, const FlightData& d, float x, float y,
                       float w, float h, float displayH) {
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float colW = (w - 2.0f * pad - colGap) * 0.5f;
  const float rx = x + pad + colW + colGap;
  char buf[32];

  // Left: GPS STATUS.
  Rect gpsInner = drawWindow(r, Rect{x + pad, y + pad, colW, h - 2.0f * pad},
                             "GPS STATUS", displayH);
  {
    const float rowH = gpsInner.h / 7.0f;
    float fy = gpsInner.y;
    fy = drawField(r, gpsInner, fy, rowH, "PHASE", d.gpsFlightPhase, displayH,
                   colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%03.0f%s",
                  static_cast<double>(d.trackDeg), kDeg);
    fy = drawField(r, gpsInner, fy, rowH, "TRK", buf, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%d KT", static_cast<int>(d.groundSpeedKts));
    fy = drawField(r, gpsInner, fy, rowH, "GS", buf, displayH, colors::kWhite);
    // CDI source annunciation, colored like the HSI: GPS magenta, VOR green.
    const char* cdiSrc = "GPS";
    Color cdiColor = colors::kMagenta;
    switch (d.cdiSource) {
      case CdiSource::Gps:
        break;
      case CdiSource::Nav1:
        cdiSrc = "VOR1";
        cdiColor = colors::kActiveGreen;
        break;
      case CdiSource::Nav2:
        cdiSrc = "VOR2";
        cdiColor = colors::kActiveGreen;
        break;
    }
    fy = drawField(r, gpsInner, fy, rowH, "CDI SRC", cdiSrc, displayH,
                   cdiColor);
    fy = drawField(r, gpsInner, fy, rowH, "LATERAL", d.fmaLateralActive,
                   displayH, colors::kActiveGreen);
    fy = drawField(r, gpsInner, fy, rowH, "VERTICAL", d.fmaVerticalActive,
                   displayH, colors::kActiveGreen);
  }

  // Right: SYSTEM / AIR DATA.
  Rect sysInner = drawWindow(r, Rect{rx, y + pad, colW, h - 2.0f * pad},
                             "AIR DATA", displayH);
  {
    const float rowH = sysInner.h / 7.0f;
    float fy = sysInner.y;
    std::snprintf(buf, sizeof(buf), "%.1f%sC", static_cast<double>(d.oatCelsius),
                  kDeg);
    fy = drawField(r, sysInner, fy, rowH, "OAT", buf, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%d KT", static_cast<int>(d.tasKts));
    fy = drawField(r, sysInner, fy, rowH, "TAS", buf, displayH, colors::kWhite);
    std::snprintf(buf, sizeof(buf), "%d FT", static_cast<int>(d.altitudeFt));
    fy = drawField(r, sysInner, fy, rowH, "ALTITUDE", buf, displayH,
                   colors::kWhite);
    if (d.windValid) {
      std::snprintf(buf, sizeof(buf), "%03.0f%s / %d KT",
                    static_cast<double>(d.windDirectionDeg), kDeg,
                    static_cast<int>(d.windSpeedKts));
      fy = drawField(r, sysInner, fy, rowH, "WIND", buf, displayH,
                     colors::kWhite);
    } else {
      fy = drawField(r, sysInner, fy, rowH, "WIND", kDash, displayH,
                     colors::kWhite);
    }
    std::snprintf(buf, sizeof(buf), "%.2f IN",
                  static_cast<double>(d.baroSettingInHg));
    fy = drawField(r, sysInner, fy, rowH, "BARO", buf, displayH, colors::kCyan);
  }
}

void drawSystemStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH) {
  // AUX System Status: LRU health on the left, airframe and database info on
  // the right. LRU statuses are derived from the live sensor-validity flags
  // (a failed source annunciates a red X, like the real LRU list).
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.42f;
  const float rx = x + pad + leftW + colGap;
  const float rightW = w - 2.0f * pad - colGap - leftW;

  Rect lruInner = drawWindow(r, Rect{x + pad, y + pad, leftW, h - 2.0f * pad},
                             "LRU INFO", displayH);
  {
    struct Lru {
      const char* name;
      bool ok;
    };
    const bool link = d.dataLinkValid;
    const Lru lrus[] = {
        {"ADC1", link && d.airspeedValid && d.altitudeValid},
        {"AHRS1", link && d.attitudeValid},
        {"COM1", link},
        {"COM2", link},
        {"GEA1", link},
        {"GIA1", link},
        {"GIA2", link},
        {"GMU1", link && d.headingValid},
        {"GPS1", link && map.positionValid},
        {"GPS2", link && map.positionValid},
        {"GTX1", link},
        {"NAV1", link && d.navSignalValid},
        {"NAV2", link},
    };
    const int count = static_cast<int>(sizeof(lrus) / sizeof(lrus[0]));
    const float rowH = lruInner.h / static_cast<float>(count + 1);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    r.fillText(lruInner.x, lruInner.y + rowH * 0.5f, "LRU",
               mfdFontPx(kWtHeader, displayH), TextAlign::Left,
               colors::kLabelText);
    r.fillText(lruInner.x + lruInner.w, lruInner.y + rowH * 0.5f, "STATUS",
               mfdFontPx(kWtHeader, displayH), TextAlign::Right,
               colors::kLabelText);
    float fy = lruInner.y + rowH;
    for (const Lru& lru : lrus) {
      const float cy = fy + rowH * 0.5f;
      r.fillText(lruInner.x, cy, lru.name, rowSize, TextAlign::Left,
                 colors::kWhite);
      // Status icon: green check for OK, red X for failed.
      const float sx = lruInner.x + lruInner.w - rowSize * 0.6f;
      const float s = rowSize * 0.40f;
      if (lru.ok) {
        r.strokeLine(sx - s, cy, sx - s * 0.2f, cy + s * 0.8f, 2.0f,
                     colors::kActiveGreen);
        r.strokeLine(sx - s * 0.2f, cy + s * 0.8f, sx + s, cy - s * 0.8f, 2.0f,
                     colors::kActiveGreen);
      } else {
        r.strokeLine(sx - s, cy - s, sx + s, cy + s, 2.0f, colors::kBandRed);
        r.strokeLine(sx - s, cy + s, sx + s, cy - s, 2.0f, colors::kBandRed);
      }
      fy += rowH;
    }
  }

  // Right column: AIRFRAME over MFD1 DATABASE.
  const float airframeH = (h - 3.0f * pad) * 0.4f;
  Rect airframeInner = drawWindow(r, Rect{rx, y + pad, rightW, airframeH},
                                  "AIRFRAME", displayH);
  {
    const float rowH = airframeInner.h / 3.0f;
    float fy = airframeInner.y;
    fy = drawField(r, airframeInner, fy, rowH, "AIRFRAME", "Cessna 172S",
                   displayH, colors::kWhite);
    fy = drawField(r, airframeInner, fy, rowH, "SYS SOFTWARE VERSION",
                   "0563.00", displayH, colors::kWhite);
    fy = drawField(r, airframeInner, fy, rowH, "CRG PART NUMBER", kDash,
                   displayH, colors::kWhite);
  }

  Rect dbInner = drawWindow(
      r, Rect{rx, y + pad + airframeH + pad, rightW, h - 3.0f * pad - airframeH},
      "MFD1 DATABASE", displayH);
  {
    const float rowH = dbInner.h / 4.0f;
    float fy = dbInner.y;
    const bool navDb = !map.features.empty();
    const bool airspaceDb = !map.airspaces.empty();
    const bool terrainDb = map.terrain != nullptr;
    fy = drawField(r, dbInner, fy, rowH, "NAVIGATION DATA",
                   navDb ? "AVAILABLE" : kDash, displayH,
                   navDb ? colors::kActiveGreen : colors::kWhite);
    fy = drawField(r, dbInner, fy, rowH, "AIRSPACE DATA",
                   airspaceDb ? "AVAILABLE" : kDash, displayH,
                   airspaceDb ? colors::kActiveGreen : colors::kWhite);
    fy = drawField(r, dbInner, fy, rowH, "TERRAIN DATA",
                   terrainDb ? "AVAILABLE" : kDash, displayH,
                   terrainDb ? colors::kActiveGreen : colors::kWhite);
    fy = drawField(r, dbInner, fy, rowH, "FLIGHT PLAN",
                   map.flightPlan.empty() ? kDash : "LOADED", displayH,
                   map.flightPlan.empty() ? colors::kWhite
                                          : colors::kActiveGreen);
  }
}

void drawNearestAirportsPage(Renderer& r, const FlightData& d,
                             const MapData& map, float x, float y, float w,
                             float h, float displayH) {
  // The MFD Nearest Airports page: map on the left, stacked info boxes on the
  // right (NEAREST AIRPORTS, INFORMATION, RUNWAYS, FREQUENCIES).
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.46f;
  const float rightW = w - 2.0f * pad - colGap - leftW;
  const float rx = x + pad + leftW + colGap;

  std::vector<NearRow> airports = collectNearest(map, MapFeatureType::Airport, 5);
  const MapFeature* selected =
      airports.empty() ? nullptr : airports.front().feature;

  // Left: map centered on ownship. The real flight data drives the ownship
  // symbol orientation and airspace altitude declutter.
  Rect mapBox{x + pad, y + pad, leftW, h - 2.0f * pad};
  Rect mapInner = drawWindow(r, mapBox, "MAP", displayH);
  drawEmbeddedMap(r, d, map, mapInner, 25.0f, nullptr, displayH);

  // Right column: four stacked windows.
  const float gap = pad;
  const float bodyH = h - 2.0f * pad - 3.0f * gap;
  const float listH = bodyH * 0.40f;
  const float infoH = bodyH * 0.24f;
  const float rwyH = bodyH * 0.18f;
  const float freqH = bodyH - listH - infoH - rwyH;
  float ry = y + pad;

  Rect listInner =
      drawWindow(r, Rect{rx, ry, rightW, listH}, "NEAREST AIRPORTS", displayH);
  drawNearestList(r, listInner, "AIRPORT", airports, 0, displayH);
  ry += listH + gap;

  Rect infoInner =
      drawWindow(r, Rect{rx, ry, rightW, infoH}, "INFORMATION", displayH);
  if (selected != nullptr) {
    const float rowH = infoInner.h / 4.0f;
    float fy = infoInner.y;
    fy = drawField(r, infoInner, fy, rowH, "REGION",
                   selected->region.empty() ? kDash : selected->region,
                   displayH, colors::kWhite);
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%dFT",
                  static_cast<int>(selected->elevationFt));
    fy = drawField(r, infoInner, fy, rowH, "ELEV",
                   selected->elevationFt != 0.0f ? buf : kDash, displayH,
                   colors::kWhite);
    fy = drawField(r, infoInner, fy, rowH, "LAT",
                   formatLatLon(selected->lat, true), displayH, colors::kWhite);
    fy = drawField(r, infoInner, fy, rowH, "LON",
                   formatLatLon(selected->lon, false), displayH,
                   colors::kWhite);
  }
  ry += infoH + gap;

  Rect rwyInner =
      drawWindow(r, Rect{rx, ry, rightW, rwyH}, "RUNWAYS", displayH);
  drawRunwayFields(r, rwyInner, selected, displayH);
  ry += rwyH + gap;

  Rect freqInner =
      drawWindow(r, Rect{rx, ry, rightW, freqH}, "FREQUENCIES", displayH);
  drawFrequencyFields(r, freqInner, displayH);
}

void drawNearestFeaturePage(Renderer& r, const FlightData& d,
                            const MapData& map, MapFeatureType type, float x,
                            float y, float w, float h, float displayH) {
  // Nearest Intersections/NDB/VOR layout: map on the left; list, INFORMATION,
  // and (for navaids) FREQUENCY windows on the right (G1000 Pilot's Guide for
  // Cessna Nav III, Sections 7.10-7.12).
  r.fillRect(x, y, w, h, colors::kBlack);
  const bool isIntersection = type == MapFeatureType::Fix;
  const char* listTitle = isIntersection ? "NEAREST INTERSECTIONS"
                          : type == MapFeatureType::Ndb ? "NEAREST NDB"
                                                        : "NEAREST VOR";
  const char* idHeader = isIntersection ? "INT"
                         : type == MapFeatureType::Ndb ? "NDB"
                                                       : "VOR";

  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.46f;
  const float rightW = w - 2.0f * pad - colGap - leftW;
  const float rx = x + pad + leftW + colGap;

  std::vector<NearRow> rows = collectNearest(map, type, 5);
  const MapFeature* selected = rows.empty() ? nullptr : rows.front().feature;

  Rect mapInner = drawWindow(r, Rect{x + pad, y + pad, leftW, h - 2.0f * pad},
                             "MAP", displayH);
  drawEmbeddedMap(r, d, map, mapInner, 25.0f, nullptr, displayH,
                  isIntersection);

  const float gap = pad;
  const float bodyH = h - 2.0f * pad - 2.0f * gap;
  const float listH = bodyH * 0.5f;
  const float infoH = isIntersection ? bodyH - listH : bodyH * 0.32f;
  float ry = y + pad;

  Rect listInner =
      drawWindow(r, Rect{rx, ry, rightW, listH}, listTitle, displayH);
  drawNearestList(r, listInner, idHeader, rows, 0, displayH);
  ry += listH + gap;

  Rect infoInner =
      drawWindow(r, Rect{rx, ry, rightW, infoH}, "INFORMATION", displayH);
  if (selected != nullptr) {
    const float rowH = infoInner.h / 2.0f;
    float fy = infoInner.y;
    fy = drawField(r, infoInner, fy, rowH, "LAT",
                   formatLatLon(selected->lat, true), displayH, colors::kWhite);
    fy = drawField(r, infoInner, fy, rowH, "LON",
                   formatLatLon(selected->lon, false), displayH,
                   colors::kWhite);
  }
  ry += infoH + gap;

  if (!isIntersection) {
    Rect freqInner = drawWindow(r, Rect{rx, ry, rightW, bodyH - listH - infoH},
                                "FREQUENCY", displayH);
    const float rowH = freqInner.h / 2.0f;
    drawField(r, freqInner, freqInner.y, rowH, "FREQUENCY",
              selected != nullptr ? formatFrequency(type, selected->frequency)
                                  : std::string(kDash),
              displayH, colors::kCyan);
  }
}

void drawNearestAirspacesPage(Renderer& r, const FlightData& d,
                              const MapData& map, float x, float y, float w,
                              float h, float displayH) {
  // Nearest Airspaces: map on the left; the airspace list (name, class, and
  // proximity status) and the selected airspace's vertical limits on the
  // right (G1000 Pilot's Guide for Cessna Nav III, Section 7.15).
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.46f;
  const float rightW = w - 2.0f * pad - colGap - leftW;
  const float rx = x + pad + leftW + colGap;

  Rect mapInner = drawWindow(r, Rect{x + pad, y + pad, leftW, h - 2.0f * pad},
                             "MAP", displayH);
  drawEmbeddedMap(r, d, map, mapInner, 25.0f, nullptr, displayH);

  // Sort airspaces by proximity (inside counts as zero distance).
  struct AirspaceRow {
    const MapAirspace* airspace = nullptr;
    double distanceNm = 0.0;
    bool inside = false;
  };
  std::vector<AirspaceRow> rows;
  if (map.positionValid) {
    for (const MapAirspace& a : map.airspaces) {
      if (a.boundary.size() < 3) continue;
      AirspaceRow row;
      row.airspace = &a;
      row.inside = insideBoundary(a.boundary, map.ownshipLat, map.ownshipLon);
      row.distanceNm =
          row.inside ? 0.0
                     : distanceToBoundaryNm(a, map.ownshipLat, map.ownshipLon);
      rows.push_back(row);
    }
    std::sort(rows.begin(), rows.end(),
              [](const AirspaceRow& a, const AirspaceRow& b) {
                return a.distanceNm < b.distanceNm;
              });
    if (rows.size() > 6) rows.resize(6);
  }
  const AirspaceRow* selected = rows.empty() ? nullptr : &rows.front();

  const float gap = pad;
  const float bodyH = h - 2.0f * pad - gap;
  const float listH = bodyH * 0.62f;
  float ry = y + pad;

  Rect listInner = drawWindow(r, Rect{rx, ry, rightW, listH},
                              "NEAREST AIRSPACES", displayH);
  {
    const float rowH = listInner.h / 7.0f;
    const float hSize = mfdFontPx(kWtHeader, displayH);
    float fy = listInner.y;
    r.fillText(listInner.x, fy + rowH * 0.5f, "AIRSPACE", hSize,
               TextAlign::Left, colors::kLabelText);
    r.fillText(listInner.x + listInner.w * 0.70f, fy + rowH * 0.5f, "CLASS",
               hSize, TextAlign::Right, colors::kLabelText);
    r.fillText(listInner.x + listInner.w, fy + rowH * 0.5f, "STATUS", hSize,
               TextAlign::Right, colors::kLabelText);
    fy += rowH;

    const float rowSize = mfdFontPx(kWtRow, displayH);
    char buf[24];
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const AirspaceRow& row = rows[i];
      const float cy = fy + rowH * 0.5f;
      if (i == 0) {
        r.fillRect(listInner.x - listInner.w * 0.02f, fy, listInner.w * 1.04f,
                   rowH, mfdAlpha(colors::kCyan, 0.16f));
      }
      // Long names are clipped by the columns to the right; keep them short.
      std::string name = row.airspace->name.empty() ? std::string("UNNAMED")
                                                    : row.airspace->name;
      if (name.size() > 12) name.resize(12);
      r.fillText(listInner.x, cy, name, rowSize, TextAlign::Left,
                 i == 0 ? colors::kCyan : colors::kWhite);
      r.fillText(listInner.x + listInner.w * 0.70f, cy,
                 airspaceClassName(row.airspace->airspaceClass), rowSize,
                 TextAlign::Right, colors::kWhite);
      if (row.inside) {
        r.fillText(listInner.x + listInner.w, cy, "INSIDE", rowSize,
                   TextAlign::Right, colors::kBandYellow);
      } else {
        std::snprintf(buf, sizeof(buf), "%.1fNM", row.distanceNm);
        r.fillText(listInner.x + listInner.w, cy, buf, rowSize,
                   TextAlign::Right, colors::kWhite);
      }
      fy += rowH;
    }
    if (rows.empty()) {
      r.fillText(listInner.x + listInner.w * 0.5f, fy + rowH,
                 "NONE WITHIN RANGE", rowSize, TextAlign::Center,
                 colors::kLabelText);
    }
  }
  ry += listH + gap;

  Rect limitsInner = drawWindow(r, Rect{rx, ry, rightW, bodyH - listH},
                                "VERTICAL LIMITS", displayH);
  {
    const float rowH = limitsInner.h / 3.0f;
    float fy = limitsInner.y;
    fy = drawField(r, limitsInner, fy, rowH, "AIRSPACE",
                   selected != nullptr ? selected->airspace->name
                                       : std::string(kDash),
                   displayH, colors::kWhite);
    fy = drawField(r, limitsInner, fy, rowH, "CEILING",
                   selected != nullptr
                       ? formatAirspaceAltitude(selected->airspace->ceilingFt,
                                                true)
                       : std::string(kDash),
                   displayH, colors::kWhite);
    fy = drawField(r, limitsInner, fy, rowH, "FLOOR",
                   selected != nullptr
                       ? formatAirspaceAltitude(selected->airspace->floorFt,
                                                false)
                       : std::string(kDash),
                   displayH, colors::kWhite);
  }
}

void drawActiveFlightPlanPage(Renderer& r, const FlightData& d,
                              const MapData& map, float x, float y, float w,
                              float h, float displayH) {
  // FPL Active Flight Plan: the leg list (WAYPOINT / DTK / DIS / CUM) on the
  // left with the active waypoint in magenta, route preview map on the right
  // (G1000 Pilot's Guide for Cessna Nav III, Section 5.6).
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const float colGap = w * 0.012f;
  const float leftW = (w - 2.0f * pad - colGap) * 0.54f;
  const float rightW = w - 2.0f * pad - colGap - leftW;
  const float rx = x + pad + leftW + colGap;

  const std::vector<MapLeg>& plan = map.flightPlan;
  std::string title = "ACTIVE FLIGHT PLAN";
  if (plan.size() >= 2) {
    title += "  " + plan.front().id + " / " + plan.back().id;
  }
  Rect listInner = drawWindow(r, Rect{x + pad, y + pad, leftW, h - 2.0f * pad},
                              title.c_str(), displayH);
  {
    const float hSize = mfdFontPx(kWtHeader, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    const float rowH = rowSize * 1.7f;
    const float colDtk = listInner.x + listInner.w * 0.52f;
    const float colDis = listInner.x + listInner.w * 0.76f;
    const float colCum = listInner.x + listInner.w;

    float fy = listInner.y;
    r.fillText(listInner.x, fy + rowH * 0.5f, "WAYPOINT", hSize,
               TextAlign::Left, colors::kLabelText);
    r.fillText(colDtk, fy + rowH * 0.5f, "DTK", hSize, TextAlign::Right,
               colors::kLabelText);
    r.fillText(colDis, fy + rowH * 0.5f, "DIS", hSize, TextAlign::Right,
               colors::kLabelText);
    r.fillText(colCum, fy + rowH * 0.5f, "CUM", hSize, TextAlign::Right,
               colors::kLabelText);
    fy += rowH;

    if (plan.empty()) {
      r.fillText(listInner.x + listInner.w * 0.5f, fy + rowH,
                 "NO ACTIVE FLIGHT PLAN", rowSize, TextAlign::Center,
                 colors::kLabelText);
    }

    char buf[24];
    double cumNm = 0.0;
    const int maxRows =
        std::max(1, static_cast<int>((listInner.h - rowH) / rowH));
    for (std::size_t i = 0;
         i < plan.size() && static_cast<int>(i) < maxRows; ++i) {
      const MapLeg& leg = plan[i];
      const float cy = fy + rowH * 0.5f;
      const bool active = !d.fmaToWpt.empty() && leg.id == d.fmaToWpt;
      const Color rowColor = active ? colors::kMagenta : colors::kWhite;
      if (active) {
        // Single-leg active arrow ahead of the waypoint ident, like the
        // magenta leg arrow in the real flight plan list.
        const float ax = listInner.x - listInner.w * 0.005f;
        const Point arrow[3] = {{ax + rowSize * 0.55f, cy},
                                {ax, cy - rowSize * 0.32f},
                                {ax, cy + rowSize * 0.32f}};
        r.fillPolygon(arrow, 3, colors::kMagenta);
      }
      r.fillText(listInner.x + rowSize * 0.8f, cy, leg.id, rowSize,
                 TextAlign::Left, rowColor);
      if (i > 0) {
        const MapLeg& prev = plan[i - 1];
        const double dtk =
            navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
        const double dis = navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
        cumNm += dis;
        std::snprintf(buf, sizeof(buf), "%03.0f%s", dtk, kDeg);
        r.fillText(colDtk, cy, buf, rowSize, TextAlign::Right, rowColor);
        std::snprintf(buf, sizeof(buf), "%.1fNM", dis);
        r.fillText(colDis, cy, buf, rowSize, TextAlign::Right, rowColor);
        std::snprintf(buf, sizeof(buf), "%.0fNM", cumNm);
        r.fillText(colCum, cy, buf, rowSize, TextAlign::Right, rowColor);
      }
      fy += rowH;
    }
  }

  // Right: route preview map around ownship, wide enough to show the plan.
  Rect mapInner =
      drawWindow(r, Rect{rx, y + pad, rightW, h - 2.0f * pad}, "MAP", displayH);
  float previewRangeNm = 25.0f;
  if (map.positionValid && plan.size() >= 2) {
    double maxNm = 0.0;
    for (const MapLeg& leg : plan) {
      maxNm = std::max(maxNm, navDistanceNm(map.ownshipLat, map.ownshipLon,
                                            leg.lat, leg.lon));
    }
    previewRangeNm =
        std::max(10.0f, std::min(150.0f, static_cast<float>(maxNm) * 1.2f));
  }
  drawEmbeddedMap(r, d, map, mapInner, previewRangeNm, nullptr, displayH);
}

namespace {

// Small checkbox with a green check when done, matching the NXi checklist item
// marker. Drawn left of the item description.
void drawCheckbox(Renderer& r, float x, float cy, float size, bool checked) {
  const Point box[5] = {{x, cy - size * 0.5f},
                        {x + size, cy - size * 0.5f},
                        {x + size, cy + size * 0.5f},
                        {x, cy + size * 0.5f},
                        {x, cy - size * 0.5f}};
  r.strokePolyline(box, 5, 1.2f, colors::kLabelText);
  if (checked) {
    r.strokeLine(x + size * 0.18f, cy + size * 0.02f, x + size * 0.42f,
                 cy + size * 0.30f, 2.0f, colors::kActiveGreen);
    r.strokeLine(x + size * 0.42f, cy + size * 0.30f, x + size * 0.82f,
                 cy - size * 0.32f, 2.0f, colors::kActiveGreen);
  }
}

}  // namespace

void drawChecklistPage(Renderer& r, const ChecklistData& checklist,
                       const MfdController& ui, float x, float y, float w,
                       float h, float displayH) {
  r.fillRect(x, y, w, h, colors::kBlack);
  const float pad = w * 0.012f;
  const Rect box{x + pad, y + pad, w - 2.0f * pad, h - 2.0f * pad};

  const int total = checklist.totalChecklists();
  const std::string* groupName = nullptr;
  const Checklist* cl =
      total > 0 ? checklist.at(ui.checklistIndex(), &groupName) : nullptr;

  if (cl == nullptr) {
    Rect inner = drawWindow(r, box, "CHECKLIST", displayH);
    r.fillText(inner.x + inner.w * 0.5f, inner.y + inner.h * 0.4f,
               "NO CHECKLIST AVAILABLE", mfdFontPx(kWtRow, displayH),
               TextAlign::Center, colors::kLabelText);
    r.fillText(inner.x + inner.w * 0.5f, inner.y + inner.h * 0.4f +
                                             mfdFontPx(kWtRow, displayH) * 1.6f,
               "ADD A CHECKLIST FILE FOR THIS AIRCRAFT",
               mfdFontPx(kWtHeader, displayH), TextAlign::Center,
               colors::kLabelText);
    return;
  }

  Rect inner = drawWindow(r, box, cl->title.c_str(), displayH);

  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float headerSize = mfdFontPx(kWtHeader, displayH);
  const float rowH = rowSize * 1.8f;

  // Header row: owning group name (left) and the "n OF m" position (right).
  float fy = inner.y;
  if (groupName != nullptr && !groupName->empty()) {
    r.fillText(inner.x, fy + rowH * 0.5f, *groupName, headerSize,
               TextAlign::Left, colors::kCyan);
  }
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "CHECKLIST %d OF %d",
                  ui.checklistIndex() + 1, total);
    r.fillText(inner.x + inner.w, fy + rowH * 0.5f, buf, headerSize,
               TextAlign::Right, colors::kLabelText);
  }
  fy += rowH;

  // The list area below the header holds the items plus the trailing
  // "go to next checklist?" prompt, scrolling to keep the cursor visible.
  const float listTop = fy;
  const float listH = inner.y + inner.h - listTop;
  const int itemCount = static_cast<int>(cl->items.size());
  const int totalRows = itemCount + 1;  // +1 for the prompt line
  const int visibleRows = std::max(1, static_cast<int>(listH / rowH));
  const int cursor = ui.checklistCursor();
  int start = 0;
  if (totalRows > visibleRows) {
    start = cursor - visibleRows / 2;
    start = std::max(0, std::min(start, totalRows - visibleRows));
  }
  const int end = std::min(totalRows, start + visibleRows);

  const float checkSize = rowSize * 0.85f;
  const float textX = inner.x + checkSize * 2.0f;
  for (int row = start; row < end; ++row) {
    const float ry = listTop + (row - start) * rowH;
    const float cy = ry + rowH * 0.5f;
    const bool isCursor = row == cursor;
    if (isCursor) {
      // Authentic G1000 cursor: a solid highlight box over the row.
      r.fillRect(inner.x - inner.w * 0.01f, ry, inner.w * 1.02f, rowH,
                 mfdAlpha(colors::kCyan, 0.85f));
    }
    const Color textColor = isCursor ? colors::kBlack : colors::kWhite;

    if (row == itemCount) {
      // Trailing prompt below the last item.
      r.fillText(inner.x, cy, "GO TO NEXT CHECKLIST?", rowSize, TextAlign::Left,
                 isCursor ? colors::kBlack : colors::kActiveGreen);
      continue;
    }

    const ChecklistItem& item = cl->items[static_cast<std::size_t>(row)];
    const bool checked = ui.checklistItemChecked(ui.checklistIndex(), row);
    drawCheckbox(r, inner.x, cy, checkSize, checked && !isCursor);
    if (isCursor && checked) {
      // On the highlight the green check is hard to read; show a dark tick.
      r.strokeLine(inner.x + checkSize * 0.18f, cy + checkSize * 0.02f,
                   inner.x + checkSize * 0.42f, cy + checkSize * 0.30f, 2.0f,
                   colors::kBlack);
      r.strokeLine(inner.x + checkSize * 0.42f, cy + checkSize * 0.30f,
                   inner.x + checkSize * 0.82f, cy - checkSize * 0.32f, 2.0f,
                   colors::kBlack);
    }
    r.fillText(textX, cy, item.text, rowSize, TextAlign::Left, textColor);
    if (!item.response.empty()) {
      r.fillText(inner.x + inner.w, cy, item.response, rowSize,
                 TextAlign::Right, textColor);
    }
  }
}

}  // namespace avionics::mfd
