#include "render/mfd/MfdPages.h"

#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/NavMath.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

int clampNrstSelected(const MfdController& ui, int rowCount) {
  if (rowCount <= 0) return 0;
  return std::max(0, std::min(ui.nrstSelected(), rowCount - 1));
}

}  // namespace

void drawNearestAirportsPage(Renderer& r, const FlightData& d,
                             const MapData& map, const MfdController& ui,
                             float x, float y, float w, float h,
                             float displayH) {
  // NRST Nearest Airports (WT MFDNearestAirportsPage): map left; the panel
  // stacks Nearest Airports / Information / Runways / Frequencies /
  // Approaches group boxes.
  std::vector<NearRow> airports =
      collectNearest(map, MapFeatureType::Airport, 5);
  const int selectedIdx = clampNrstSelected(ui, static_cast<int>(airports.size()));
  const MapFeature* selected =
      airports.empty() ? nullptr : airports[selectedIdx].feature;

  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, 25.0f, nullptr, displayH);

  PanelStack stack(f.panel, displayH);
  char buf[24];

  {
    Rect inner = drawGroupBox(r, stack.slot(170.0f), "Nearest Airports",
                              displayH);
    drawNearestRows(r, inner, airports, selectedIdx,
                    MapFeatureType::Airport, displayH);
  }

  // Information box (Fig 5-30): facility name, city, and field elevation.
  {
    Rect inner = drawGroupBox(r, stack.slot(96.0f), "Information", displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    float fy = drawFacilityNameCity(r, inner, inner.y, selected, displayH);
    if (selected != nullptr && selected->elevationFt != 0.0f) {
      const float cy =
          fy > inner.y ? fy - rowH * 0.5f : inner.y + rowH * 0.5f;
      std::snprintf(buf, sizeof(buf), "%d",
                    static_cast<int>(selected->elevationFt));
      drawValueWithUnit(r, inner.x + inner.w, cy, buf, "FT", rowSize,
                        colors::kWhitesmoke);
    }
    if (selected != nullptr && selected->name.empty() &&
        selected->city.empty()) {
      r.fillText(inner.x, inner.y + rowH * 0.5f, kDash, rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    }
  }

  // Runways box: compact designation + dimensions form.
  {
    Rect inner = drawGroupBox(r, stack.slot(82.0f), "Runways", displayH);
    drawRunwayGroup(r, inner,
                    selected != nullptr ? ui.airportRunways(selected->id)
                                        : std::vector<AirportRunwayInfo>{},
                    0, 2, displayH);
  }

  {
    Rect inner = drawGroupBox(r, stack.slot(112.0f), "Frequencies", displayH);
    drawFrequencyGroup(
        r, inner, displayH, 3,
        selected != nullptr ? ui.airportFrequencies(selected->id)
                            : std::vector<MapAirportFrequency>{});
  }

  {
    Rect inner = drawGroupBox(r, stack.slot(stack.remainingWt()),
                              "Approaches", displayH);
    const std::vector<MapProcedure> procedures =
        selected != nullptr ? ui.proceduresForAirport(selected->id,
                                                      ProcedureType::Approach)
                            : std::vector<MapProcedure>{};
    drawApproachesGroup(r, inner, displayH, procedures);
  }
}

void drawNearestFeaturePage(Renderer& r, const FlightData& d,
                            const MapData& map, const MfdController& ui,
                            MapFeatureType type, float x, float y, float w,
                            float h, float displayH) {
  // NRST Nearest Intersections/NDB/VOR (WT nearest pages): map left; the
  // panel stacks the tall facility list, Information, and the Reference VOR
  // (intersections) or Frequency (navaids) box.
  const bool isIntersection = type == MapFeatureType::Fix;
  const char* listTitle = isIntersection ? "Nearest Intersections"
                          : type == MapFeatureType::Ndb ? "Nearest NDB"
                                                        : "Nearest VOR";

  std::vector<NearRow> rows = collectNearest(map, type, 10);
  const int selectedIdx = clampNrstSelected(ui, static_cast<int>(rows.size()));
  const MapFeature* selected =
      rows.empty() ? nullptr : rows[selectedIdx].feature;

  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, 25.0f, nullptr, displayH, isIntersection);

  PanelStack stack(f.panel, displayH);

  {
    Rect inner = drawGroupBox(r, stack.slot(330.0f), listTitle, displayH);
    drawNearestRows(r, inner, rows, selectedIdx, type, displayH);
  }

  {
    const bool isVor = type == MapFeatureType::Vor;
    Rect inner = drawGroupBox(r, stack.slot(isVor ? 130.0f : 102.0f),
                              "Information", displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    float fy = inner.y;
    if (isVor) {
      r.fillText(inner.x, fy + rowH * 0.5f,
                 selected != nullptr ? "VOR" : kDash,
                 mfdFontPx(kWtRow, displayH), TextAlign::Left,
                 colors::kWhitesmoke);
      fy += rowH;
    }
    fy = drawField(r, inner, fy, rowH, "LAT",
                   selected != nullptr ? formatLatLon(selected->lat, true)
                                       : std::string(kDash),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "LON",
                   selected != nullptr ? formatLatLon(selected->lon, false)
                                       : std::string(kDash),
                   displayH, colors::kWhitesmoke);
  }

  if (isIntersection) {
    Rect inner = drawGroupBox(r, stack.slot(102.0f), "Reference VOR",
                              displayH);
    const MapFeature* vor = nearestFeature(map, MapFeatureType::Vor);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    float cy = inner.y + rowH * 0.5f;
    if (vor != nullptr && selected != nullptr) {
      char buf[24];
      r.fillText(inner.x, cy, vor->id, rowSize, TextAlign::Left,
                 colors::kWhitesmoke);
      drawWaypointIcon(r, inner.x + inner.w * 0.40f, cy,
                       mfdFontPx(22.0f, displayH), vor, MapFeatureType::Vor);
      r.fillText(inner.x + inner.w, cy,
                 formatFrequency(MapFeatureType::Vor, vor->frequency), rowSize,
                 TextAlign::Right, colors::kWhitesmoke);
      cy += rowH;
      std::snprintf(buf, sizeof(buf), "%03.0f",
                    navBearingDeg(selected->lat, selected->lon, vor->lat,
                                  vor->lon));
      drawValueWithUnit(r, inner.x + inner.w * 0.40f, cy, buf, kDeg, rowSize,
                        colors::kWhitesmoke);
      std::snprintf(buf, sizeof(buf), "%.1f",
                    navDistanceNm(selected->lat, selected->lon, vor->lat,
                                  vor->lon));
      drawValueWithUnit(r, inner.x + inner.w, cy, buf, "NM", rowSize,
                        colors::kWhitesmoke);
    } else {
      r.fillText(inner.x, cy, kDash, rowSize, TextAlign::Left,
                 colors::kWhitesmoke);
    }
  } else {
    Rect inner = drawGroupBox(r, stack.slot(64.0f), "Frequency", displayH);
    const std::string freq =
        selected != nullptr ? formatFrequency(type, selected->frequency)
                            : std::string(kDash);
    const float cy = inner.y + mfdFontPx(kWtListRow, displayH) * 0.5f;
    if (type == MapFeatureType::Vor) {
      const float pillW = inner.w * 0.32f;
      const float pillH = mfdFontPx(24.0f, displayH);
      drawPill(r,
               Rect{inner.x + inner.w * 0.5f - pillW * 0.5f,
                    cy - pillH * 0.5f, pillW, pillH},
               freq, mfdFontPx(18.0f, displayH), colors::kWhitesmoke);
    } else {
      r.fillText(inner.x + inner.w * 0.15f, cy, freq,
                 mfdFontPx(kWtRow, displayH), TextAlign::Left,
                 colors::kWhitesmoke);
    }
  }
}

void drawNearestFrequenciesPage(Renderer& r, const FlightData& d,
                                const MapData& map, float x, float y, float w,
                                float h, float displayH) {
  // NRST Nearest Frequencies (G1000 Pilot's Guide, Section 5, NRST - Nearest
  // Frequencies): map left; the panel stacks ARTCC / FSS / WX frequency
  // groups. With no communications-frequency database in this suite, the
  // identifiers and frequency pills dash, like the real unit before a database
  // is loaded.
  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, 60.0f, nullptr, displayH);

  PanelStack stack(f.panel, displayH);

  auto drawFreqColumn = [&](const char* title, float wtHeight, int rows) {
    Rect inner = drawGroupBox(r, stack.slot(wtHeight), title, displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    float yy = inner.y;
    for (int i = 0; i < rows; ++i) {
      if (yy + rowH > inner.y + inner.h + 1.0f) break;
      const float cy = yy + rowH * 0.5f;
      r.fillText(inner.x, cy, kDash, rowSize, TextAlign::Left,
                 colors::kCyan);
      const float pillW = inner.w * 0.34f;
      const float pillH = mfdFontPx(24.0f, displayH);
      drawPill(r,
               Rect{inner.x + inner.w - pillW, cy - pillH * 0.5f, pillW, pillH},
               "___.__", mfdFontPx(18.0f, displayH), colors::kWhitesmoke);
      yy += rowH;
    }
  };

  drawFreqColumn("ARTCC", 150.0f, 3);
  drawFreqColumn("FSS", 150.0f, 3);
  drawFreqColumn("WX", stack.remainingWt(), 3);
}

void drawNearestAirspacesPage(Renderer& r, const FlightData& d,
                              const MapData& map, const MfdController& ui,
                              float x, float y, float w, float h,
                              float displayH) {
  // NRST Nearest Airspaces: map left; the airspace list (name, class, and
  // proximity status) and the selected airspace's vertical limits on the
  // right (G1000 Pilot's Guide for Cessna Nav III, Section 7.15; not in WT).
  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, 25.0f, nullptr, displayH);

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
    if (rows.size() > 8) rows.resize(8);
  }
  const int selectedIdx = clampNrstSelected(ui, static_cast<int>(rows.size()));
  const AirspaceRow* selected =
      rows.empty() ? nullptr : &rows[selectedIdx];

  PanelStack stack(f.panel, displayH);

  {
    Rect inner = drawGroupBox(r, stack.slot(stack.remainingWt(140.0f)),
                              "Nearest Airspaces", displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(16.0f, displayH);
    float yy = inner.y;
    char buf[24];
    for (std::size_t i = 0; i < rows.size(); ++i) {
      if (yy + 2.0f * rowH > inner.y + inner.h + 1.0f) break;
      const AirspaceRow& row = rows[i];
      float cy = yy + rowH * 0.5f;
      if (static_cast<int>(i) == selectedIdx) {
        drawSelectArrow(r, inner.x, cy, rowSize * 0.85f);
      }
      // Name on its own line (long), class + status on the second.
      std::string name = row.airspace->name.empty() ? std::string("UNNAMED")
                                                    : row.airspace->name;
      if (name.size() > 24) name.resize(24);
      r.fillText(inner.x + mfdFontPx(22.0f, displayH), cy, name, rowSize,
                 TextAlign::Left, colors::kCyan);
      cy += rowH;
      r.fillText(inner.x + mfdFontPx(22.0f, displayH), cy,
                 airspaceClassName(row.airspace->airspaceClass), rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
      if (row.inside) {
        r.fillText(inner.x + inner.w, cy, "INSIDE", rowSize, TextAlign::Right,
                   colors::kBandYellow);
      } else {
        std::snprintf(buf, sizeof(buf), "%.1f", row.distanceNm);
        drawValueWithUnit(r, inner.x + inner.w, cy, buf, "NM", rowSize,
                          colors::kWhitesmoke);
      }
      yy += 2.0f * rowH;
    }
    if (rows.empty()) {
      r.fillText(inner.x + inner.w * 0.5f, inner.y + rowH,
                 "NONE WITHIN RANGE", rowSize, TextAlign::Center,
                 colors::kTitleGray);
    }
  }

  {
    Rect inner = drawGroupBox(r, stack.slot(130.0f), "Vertical Limits",
                              displayH);
    const float rowH = inner.h / 3.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "AIRSPACE",
                   selected != nullptr ? selected->airspace->name
                                       : std::string(kDash),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "CEILING",
                   selected != nullptr
                       ? formatAirspaceAltitude(selected->airspace->ceilingFt,
                                                true)
                       : std::string(kDash),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "FLOOR",
                   selected != nullptr
                       ? formatAirspaceAltitude(selected->airspace->floorFt,
                                                false)
                       : std::string(kDash),
                   displayH, colors::kWhitesmoke);
  }
}

}  // namespace avionics::mfd
