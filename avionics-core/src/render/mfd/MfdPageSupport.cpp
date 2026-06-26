#include "render/mfd/MfdPageSupport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "avionics/NavMath.h"
#include "avionics/render/MapSymbols.h"
#include "render/map/MapViewInternal.h"

namespace avionics::mfd {

float directToInsetRangeNm(const MapData& map, const MapFeature& wpt) {
  float dtoRangeNm = 7.5f;
  if (map.positionValid) {
    const double dis =
        navDistanceNm(map.ownshipLat, map.ownshipLon, wpt.lat, wpt.lon);
    if (dis > 0.1) {
      dtoRangeNm = static_cast<float>(
          std::max(2.0, std::min(250.0, dis * 1.2)));
    }
  }
  return dtoRangeNm;
}

float directToInsetViewHalfExtentNm(float rangeNm) {
  MapViewConfig cfg{};
  cfg.w = 180.0f;
  cfg.h = 220.0f;
  const float mapRadiusPx = mapview::mapRangeSpanPx(cfg);
  if (mapRadiusPx <= 0.0f || rangeNm <= 0.0f) return rangeNm * 1.1f;
  const float pixelsPerNm = mapRadiusPx / rangeNm;
  const float halfW = cfg.w * 0.5f;
  const float halfH = cfg.h * 0.5f;
  return std::sqrt(halfW * halfW + halfH * halfH) / pixelsPerNm;
}

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

bool sunriseSunsetUtc(int dayOfYear, double latDeg, double lonDeg,
                      double& riseUtcHours, double& setUtcHours) {
  if (dayOfYear < 1) return false;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kRad = kPi / 180.0;
  // NOAA's fractional-year solar approximation (General Solar Position
  // Calculations, NOAA Global Monitoring Division): equation of time
  // (minutes) and solar declination (radians) from the fractional year.
  const double g = 2.0 * kPi / 365.0 * (dayOfYear - 1);
  const double eqTimeMin =
      229.18 * (0.000075 + 0.001868 * std::cos(g) - 0.032077 * std::sin(g) -
                0.014615 * std::cos(2.0 * g) - 0.040849 * std::sin(2.0 * g));
  const double decl = 0.006918 - 0.399912 * std::cos(g) +
                      0.070257 * std::sin(g) - 0.006758 * std::cos(2.0 * g) +
                      0.000907 * std::sin(2.0 * g) -
                      0.002697 * std::cos(3.0 * g) +
                      0.00148 * std::sin(3.0 * g);
  // Hour angle at the standard sunrise/sunset zenith (refraction + the solar
  // half-diameter). |cos| > 1 means the sun never crosses it that day.
  const double lat = latDeg * kRad;
  const double cosHa =
      std::cos(90.833 * kRad) / (std::cos(lat) * std::cos(decl)) -
      std::tan(lat) * std::tan(decl);
  if (cosHa < -1.0 || cosHa > 1.0) return false;
  const double haDeg = std::acos(cosHa) / kRad;
  // Minutes UTC; longitude positive east (4 minutes per degree).
  const double riseMin = 720.0 - 4.0 * (lonDeg + haDeg) - eqTimeMin;
  const double setMin = 720.0 - 4.0 * (lonDeg - haDeg) - eqTimeMin;
  riseUtcHours = std::fmod(riseMin / 60.0 + 48.0, 24.0);
  setUtcHours = std::fmod(setMin / 60.0 + 48.0, 24.0);
  return true;
}

// X-Plane US region codes K1-K7 follow the seven FAA enroute-chart regions
// (verified against earth_nav.dat: SEA=K1, ABQ/DEN=K2, ICT/HLC/TOP=K3,
// BVO/SPS=K4, CGT=K5, BOS=K6, MEM/ORL=K7), matching the names the G1000
// shows in the WPT information boxes (Fig 5-26: KHLC "N CEN USA").
std::string regionName(const std::string& code) {
  struct Entry {
    const char* code;
    const char* name;
  };
  static const Entry kEntries[] = {
      {"K1", "NW USA"},      {"K2", "SW USA"},   {"K3", "N CEN USA"},
      {"K4", "S CEN USA"},   {"K5", "E CEN USA"}, {"K6", "NE USA"},
      {"K7", "SE USA"},      {"PA", "ALASKA"},   {"PH", "HAWAII"},
      {"CY", "CANADA"},      {"CZ", "CANADA"},   {"MM", "MEXICO"},
  };
  for (const Entry& e : kEntries) {
    if (code == e.code) return e.name;
  }
  return code;
}

const char* runwaySurfaceName(RunwaySurface surface) {
  switch (surface) {
    case RunwaySurface::Hard:
      return "Hard Surface";
    case RunwaySurface::Turf:
      return "Turf";
    case RunwaySurface::Gravel:
      return "Gravel";
    case RunwaySurface::Dirt:
      return "Dirt";
    case RunwaySurface::Soft:
      return "Soft Surface";
    case RunwaySurface::Water:
      return "Water";
    case RunwaySurface::Unknown:
      break;
  }
  return "Unknown";
}

const char* airportUsageType(const MapFeature& apt) {
  switch (apt.airportKind) {
    case AirportFacilityKind::Private:
      return "Private";
    case AirportFacilityKind::Heliport:
      return "Heliport";
    case AirportFacilityKind::Land:
    case AirportFacilityKind::Seaplane:
      break;
  }
  return "Public";
}

// VOR service volumes: Terminal 25nm, Low Altitude 40nm, High Altitude 130nm
// (the classes named in the Pilot's Guide VOR information box).
const char* vorClassName(int rangeNm) {
  if (rangeNm <= 0) return "VOR";
  if (rangeNm <= 25) return "Terminal";
  if (rangeNm <= 50) return "Low Altitude";
  return "High Altitude";
}

std::string formatMagvar(const MapFeature& navaid) {
  if (!navaid.hasMagvar) return {};
  const int deg = static_cast<int>(
      std::lround(std::fabs(static_cast<double>(navaid.magvarDeg))));
  char buf[12];
  std::snprintf(buf, sizeof(buf), "%d%s%c", deg, kDeg,
                navaid.magvarDeg >= 0.0f ? 'E' : 'W');
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

PageFrame beginPanelPage(Renderer& r, float x, float y, float w, float h,
                         bool widePanel) {
  const float panelW = w * (widePanel ? kPanelWideWFrac : kPanelWFrac);
  PageFrame f;
  f.map = Rect{x, y, w - panelW, h};
  f.panel = Rect{x + w - panelW, y, panelW, h};
  r.fillRect(f.panel.x, f.panel.y, f.panel.w, f.panel.h, colors::kMfdPanelGray);
  return f;
}

void drawPageMap(Renderer& r, const FlightData& d, const MapData& map,
                 const Rect& area, float rangeNm, const MapFeature* center,
                 float displayH, bool showFixes,
                 const std::vector<MapLeg>* procedurePreview,
                 float displayRangeNm, TerrainDisplay terrain,
                 bool useInsetMapData) {
  MapViewConfig cfg;
  cfg.x = area.x;
  cfg.y = area.y;
  cfg.w = area.w;
  cfg.h = area.h;
  cfg.orientation = MapOrientation::NorthUp;
  cfg.rangeNm = rangeNm;
  cfg.displayRangeNm = displayRangeNm;
  cfg.style.showChrome = false;
  cfg.style.showOrientationLabel = true;
  cfg.style.showNorthArrow = true;
  cfg.style.terrain = terrain;
  cfg.style.showFixes = showFixes;
  cfg.style.labelFontWt = 16.0f;
  cfg.procedurePreview = procedurePreview;
  cfg.useInsetMapData = useInsetMapData;
  if (center != nullptr) {
    cfg.hasCenterOverride = true;
    cfg.centerLat = center->lat;
    cfg.centerLon = center->lon;
    cfg.centerFeature = center;
  }
  r.fillRect(area.x, area.y, area.w, area.h, colors::kBlack);
  MapView::render(r, map, d, cfg, displayH);
}

// ---- waypoint symbols (map-style vector glyphs for the list/header rows) ----

void drawWaypointIcon(Renderer& r, float cx, float cy, float size,
                      const MapFeature* feature, MapFeatureType type) {
  if (feature != nullptr) {
    drawMapFeatureSymbol(r, *feature, cx, cy, size * 0.5f);
    return;
  }
  drawMapFeatureSymbol(r, type, cx, cy, size * 0.5f, mapFeatureColor(type));
}

// The white selected-facility arrow from the WT nearest lists
// (path 'm 0 2 L 4 2 L 4 0 L 7 3 L 4 6 L 4 4 L 0 4 z').
void drawSelectArrow(Renderer& r, float x, float cy, float size) {
  const float u = size / 7.0f;
  const Point pts[7] = {{x, cy - u},          {x + 4.0f * u, cy - u},
                        {x + 4.0f * u, cy - 3.0f * u}, {x + 7.0f * u, cy},
                        {x + 4.0f * u, cy + 3.0f * u}, {x + 4.0f * u, cy + u},
                        {x, cy + u}};
  r.fillPolygon(pts, 7, colors::kWhite);
}

// ---- shared group-box contents ----

// Facility header inside a waypoint group box: cyan ident with the waypoint
// symbol beside it, and the usage type (airports, Fig 5-26 "Public") or
// station kind (VORs, Fig 5-35 "VOR-DME") top-right. Returns the y below the
// header.
float drawFacilityHeader(Renderer& r, const Rect& area, const MapFeature* f,
                         MapFeatureType type, float displayH) {
  const float identSize = mfdFontPx(kWtIdentLarge, displayH);
  const float cy = area.y + identSize * 0.62f;
  if (f == nullptr) {
    r.fillText(area.x, cy, "_ _ _ _ _", identSize, TextAlign::Left,
               colors::kCyan);
    return area.y + identSize * 1.5f;
  }
  r.fillText(area.x, cy, f->id, identSize, TextAlign::Left, colors::kCyan);
  drawWaypointIcon(r, area.x + mfdFontPx(120.0f, displayH), cy,
                   mfdFontPx(26.0f, displayH), f, type);
  const char* rightLabel = nullptr;
  if (type == MapFeatureType::Airport) {
    rightLabel = airportUsageType(*f);
  } else if (type == MapFeatureType::Vor && !f->navaidType.empty()) {
    rightLabel = f->navaidType.c_str();
  }
  if (rightLabel != nullptr) {
    r.fillText(area.x + area.w, cy, rightLabel, mfdFontPx(18.0f, displayH),
               TextAlign::Right, colors::kWhitesmoke);
  }
  return area.y + identSize * 1.5f;
}

float drawFacilityNameCity(Renderer& r, const Rect& area, float y,
                           const MapFeature* f, float displayH) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  auto drawRow = [&](const std::string& text) {
    std::string row = text;
    // Clip long facility names to the box (the G1000 truncates).
    while (row.size() > 4 &&
           r.measureTextWidth(row, rowSize) > area.w) {
      row.pop_back();
    }
    r.fillText(area.x, y + rowH * 0.5f, row, rowSize, TextAlign::Left,
               colors::kCyan);
    y += rowH;
  };
  if (f != nullptr && !f->name.empty()) drawRow(f->name);
  if (f != nullptr && !f->city.empty()) drawRow(f->city);
  return y;
}

// Runways group (Fig 5-26 "Runways" box): selector arrows around the cyan
// runway designation, then dimensions, surface type, and lighting rows.
void drawRunwayGroup(Renderer& r, const Rect& area,
                     const std::vector<AirportRunwayInfo>& runways,
                     int selectedIndex, int maxRows, float displayH) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  float cy = area.y + rowH * 0.5f;

  const AirportRunwayInfo* rwy = nullptr;
  if (!runways.empty()) {
    const int idx = std::max(
        0, std::min(selectedIndex, static_cast<int>(runways.size()) - 1));
    rwy = &runways[static_cast<std::size_t>(idx)];
  }

  // Designation row with prev/next arrows (visible when several runways).
  const float a = rowSize * 0.34f;
  if (runways.size() > 1) {
    const float designW =
        r.measureTextWidth(rwy->designation, rowSize) + 4.0f * a;
    const Point la[3] = {{area.x, cy},
                         {area.x + a, cy - a},
                         {area.x + a, cy + a}};
    r.fillPolygon(la, 3, colors::kCyan);
    const Point ra[3] = {{area.x + designW + 2.0f * a, cy},
                         {area.x + designW + a, cy - a},
                         {area.x + designW + a, cy + a}};
    r.fillPolygon(ra, 3, colors::kCyan);
  }
  r.fillText(area.x + 2.0f * a, cy,
             rwy != nullptr ? rwy->designation : std::string(kDash), rowSize,
             TextAlign::Left, colors::kCyan);
  cy += rowH;

  const float indent = mfdFontPx(10.0f, displayH);
  // Dimensions row: "5000FT x 75FT" with the FT suffixes at unit size.
  if (rwy != nullptr && rwy->lengthFt > 0) {
    char num[16];
    const float unitSize = rowSize * kUnitEm;
    float xx = area.x + indent;
    std::snprintf(num, sizeof(num), "%d", rwy->lengthFt);
    r.fillText(xx, cy, num, rowSize, TextAlign::Left, colors::kWhitesmoke);
    xx += r.measureTextWidth(num, rowSize);
    r.fillText(xx, cy, "FT", unitSize, TextAlign::Left, colors::kWhitesmoke);
    xx += r.measureTextWidth("FT", unitSize);
    r.fillText(xx, cy, " x ", rowSize, TextAlign::Left, colors::kWhitesmoke);
    xx += r.measureTextWidth(" x ", rowSize);
    if (rwy->widthFt > 0) {
      std::snprintf(num, sizeof(num), "%d", rwy->widthFt);
      r.fillText(xx, cy, num, rowSize, TextAlign::Left, colors::kWhitesmoke);
      xx += r.measureTextWidth(num, rowSize);
      r.fillText(xx, cy, "FT", unitSize, TextAlign::Left, colors::kWhitesmoke);
    } else {
      r.fillText(xx, cy, kDash, rowSize, TextAlign::Left, colors::kWhitesmoke);
    }
  } else {
    r.fillText(area.x + indent, cy, kDash, rowSize, TextAlign::Left,
               colors::kWhitesmoke);
  }
  cy += rowH;
  if (maxRows <= 2) return;

  // Surface type row.
  r.fillText(area.x + indent, cy,
             rwy != nullptr ? runwaySurfaceName(rwy->surface) : kDash, rowSize,
             TextAlign::Left, colors::kWhitesmoke);
  cy += rowH;
  if (maxRows <= 3) return;

  // Lighting row.
  if (rwy != nullptr) {
    r.fillText(area.x + indent, cy, rwy->lighted ? "Lights" : "No Lights",
               rowSize, TextAlign::Left, colors::kWhitesmoke);
  }
}

// Frequencies group: name left, frequency in the rounded pill right (WT
// FrequenciesGroup). With no frequency database the pills dash.
void drawApproachesGroup(Renderer& r, const Rect& area, float displayH,
                         const std::vector<MapProcedure>& procedures) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  if (procedures.empty()) {
    r.fillText(area.x, area.y + rowH * 0.5f, kDash, rowSize, TextAlign::Left,
               colors::kCyan);
    return;
  }
  const int fit = std::max(1, static_cast<int>(area.h / rowH));
  const int count = std::min(fit, static_cast<int>(procedures.size()));
  float yy = area.y;
  char buf[48];
  for (int i = 0; i < count; ++i) {
    const MapProcedure& proc = procedures[static_cast<std::size_t>(i)];
    const float cy = yy + rowH * 0.5f;
    if (!proc.runway.empty()) {
      std::snprintf(buf, sizeof(buf), "%s RW%s", proc.name.c_str(),
                    proc.runway.c_str());
    } else if (!proc.transition.empty() && proc.transition != proc.name) {
      std::snprintf(buf, sizeof(buf), "%s.%s", proc.name.c_str(),
                    proc.transition.c_str());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", proc.name.c_str());
    }
    r.fillText(area.x, cy, buf, rowSize, TextAlign::Left, colors::kCyan);
    if (proc.frequencyMhz > 0.0f) {
      std::snprintf(buf, sizeof(buf), "%.2f", proc.frequencyMhz);
      drawValueWithUnit(r, area.x + area.w, cy, buf, "MHz", rowSize,
                        colors::kCyan);
    }
    yy += rowH;
  }
}

namespace {

const char* commServiceName(AirportCommService service) {
  switch (service) {
    case AirportCommService::Atis:
      return "ATIS";
    case AirportCommService::Unicom:
      return "UNICOM";
    case AirportCommService::Clearance:
      return "CLEARANCE";
    case AirportCommService::Ground:
      return "GROUND";
    case AirportCommService::Tower:
      return "TOWER";
    case AirportCommService::Approach:
      return "APPROACH";
    case AirportCommService::Departure:
      return "DEPARTURE";
    case AirportCommService::Other:
      break;
  }
  return nullptr;
}

// Display order of the comm services in the Frequencies box (the published
// list order on the unit: information services first, then the control
// positions in the order a departure uses them).
int commServiceRank(AirportCommService service) {
  switch (service) {
    case AirportCommService::Atis:
      return 0;
    case AirportCommService::Unicom:
      return 1;
    case AirportCommService::Clearance:
      return 2;
    case AirportCommService::Ground:
      return 3;
    case AirportCommService::Tower:
      return 4;
    case AirportCommService::Approach:
      return 5;
    case AirportCommService::Departure:
      return 6;
    case AirportCommService::Other:
      break;
  }
  return 7;
}

}  // namespace

// Frequencies group (Fig 5-26): the airport's published frequencies, service
// name left and the frequency in a pill right; ATIS is flagged RX
// (receive-only) like the real unit. Dashes when nothing is published.
void drawFrequencyGroup(Renderer& r, const Rect& area, float displayH,
                        int maxRows,
                        const std::vector<MapAirportFrequency>& frequencies) {
  std::vector<const MapAirportFrequency*> rows;
  for (const MapAirportFrequency& f : frequencies) {
    if (commServiceName(f.service) != nullptr && f.mhz > 0.0f) {
      rows.push_back(&f);
    }
  }
  std::stable_sort(rows.begin(), rows.end(),
                   [](const MapAirportFrequency* a,
                      const MapAirportFrequency* b) {
                     return commServiceRank(a->service) <
                            commServiceRank(b->service);
                   });

  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float pillW = area.w * 0.34f;
  const float pillH = mfdFontPx(24.0f, displayH);
  const int fit =
      std::max(1, std::min(maxRows, static_cast<int>(area.h / rowH)));

  if (rows.empty()) {
    const float cy = area.y + rowH * 0.5f;
    r.fillText(area.x, cy, kDash, rowSize, TextAlign::Left,
               colors::kWhitesmoke);
    drawPill(r, Rect{area.x + area.w - pillW, cy - pillH * 0.5f, pillW, pillH},
             "___.___", mfdFontPx(18.0f, displayH), colors::kWhitesmoke);
    return;
  }

  char buf[16];
  float yy = area.y;
  const int count = std::min(fit, static_cast<int>(rows.size()));
  for (int i = 0; i < count; ++i) {
    const MapAirportFrequency& f = *rows[static_cast<std::size_t>(i)];
    const float cy = yy + rowH * 0.5f;
    r.fillText(area.x, cy, commServiceName(f.service), rowSize,
               TextAlign::Left, colors::kWhitesmoke);
    if (f.service == AirportCommService::Atis) {
      r.fillText(area.x + area.w - pillW - mfdFontPx(6.0f, displayH), cy, "RX",
                 rowSize * kUnitEm, TextAlign::Right, colors::kWhitesmoke);
    }
    std::snprintf(buf, sizeof(buf), "%.3f", f.mhz);
    drawPill(r, Rect{area.x + area.w - pillW, cy - pillH * 0.5f, pillW, pillH},
             buf, mfdFontPx(18.0f, displayH), colors::kWhitesmoke);
    yy += rowH;
  }
}

// Nearest list rows (WT FacilitiesGroup): selected-facility arrow, cyan ident,
// waypoint icon, bearing and distance right-aligned with small unit suffixes.
void drawNearestRows(Renderer& r, const Rect& area,
                     const std::vector<NearRow>& rows, int selected,
                     MapFeatureType type, float displayH) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  if (rows.empty()) {
    r.fillText(area.x + area.w * 0.5f, area.y + rowH, "NONE WITHIN RANGE",
               mfdFontPx(16.0f, displayH), TextAlign::Center,
               colors::kTitleGray);
    return;
  }
  char buf[24];
  float yy = area.y;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (yy + rowH > area.y + area.h + 1.0f) break;
    const NearRow& row = rows[i];
    const float cy = yy + rowH * 0.5f;
    if (static_cast<int>(i) == selected) {
      drawSelectArrow(r, area.x, cy, rowSize * 0.85f);
    }
    r.fillText(area.x + mfdFontPx(22.0f, displayH), cy, row.feature->id,
               rowSize, TextAlign::Left, colors::kCyan);
    drawWaypointIcon(r, area.x + mfdFontPx(118.0f, displayH), cy,
                     mfdFontPx(22.0f, displayH), row.feature, type);
    std::snprintf(buf, sizeof(buf), "%03.0f",
                  static_cast<double>(row.bearingDeg));
    drawValueWithUnit(r, area.x + area.w - mfdFontPx(88.0f, displayH), cy, buf,
                      kDeg, rowSize, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1f",
                  static_cast<double>(row.distanceNm));
    drawValueWithUnit(r, area.x + area.w, cy, buf, "NM", rowSize,
                      colors::kWhitesmoke);
    yy += rowH;
  }
}

const char* airspaceClassName(AirspaceClass c) {
  switch (c) {
    case AirspaceClass::ClassA:
      return "CLASS A";
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
    case AirspaceClass::Warning:
      return "WARNING";
    case AirspaceClass::Alert:
      return "ALERT";
    case AirspaceClass::Caution:
      return "CAUTION";
    case AirspaceClass::Training:
      return "TRAINING";
    case AirspaceClass::MOA:
      return "MOA";
    case AirspaceClass::TRSA:
      return "TRSA";
    case AirspaceClass::ADIZ:
      return "ADIZ";
    case AirspaceClass::TFR:
      return "TFR";
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


}  // namespace avionics::mfd
