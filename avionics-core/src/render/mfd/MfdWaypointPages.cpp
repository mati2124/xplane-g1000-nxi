#include "render/mfd/MfdPages.h"

#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "avionics/NavMath.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

const MapFeature* wptFacility(const MapData& map, const MfdController& ui,
                              MapFeatureType type) {
  if (ui.wptHasSelection()) return &ui.wptSelectedFeature();
  return nearestFeature(map, type);
}

// Facility ident row with optional FMS entry cursor (Pilot's Guide waypoint
// ident search).
float drawWptFacilityHeader(Renderer& r, const Rect& area,
                            const MapFeature* f, MapFeatureType type,
                            const MfdController& ui, float displayH) {
  const float identSize = mfdFontPx(kWtIdentLarge, displayH);
  const float cy = area.y + identSize * 0.62f;
  if (ui.wptEntryActive()) {
    std::string ident = ui.wptEntryIdent();
    while (static_cast<int>(ident.size()) <= ui.wptEntryCursor()) {
      ident += '_';
    }
    drawCursorSelect(r, area.x, cy, ident, identSize, TextAlign::Left,
                     ui.blinkOn());
    if (ui.wptEntryNotFound()) {
      r.fillText(area.x + identSize * 3.5f, cy, "NOT FOUND", identSize * 0.55f,
                 TextAlign::Left, colors::kBandYellow);
    }
    return area.y + identSize * 1.5f;
  }
  return drawFacilityHeader(r, area, f, type, displayH);
}

}  // namespace

void drawWaypointPage(Renderer& r, const FlightData& d, const MapData& map,
                      const MfdController& ui, float x, float y, float w,
                      float h, float displayH) {
  // WPT Airport Information (Fig 5-26): full-height map on the left, the gray
  // panel on the right with the Airport / Runways / Frequencies group boxes.
  const MapFeature* apt = wptFacility(map, ui, MapFeatureType::Airport);
  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, kAirportDiagramRangeNm, apt, displayH);

  PanelStack stack(f.panel, displayH);
  char buf[24];

  // Airport box: ident/symbol/usage header, facility name and city, then the
  // region + elevation and location + fuel rows.
  {
    Rect inner = drawGroupBox(r, stack.slot(206.0f), "Airport", displayH);
    float fy = drawWptFacilityHeader(r, inner, apt, MapFeatureType::Airport, ui,
                                     displayH);
    fy = drawFacilityNameCity(r, inner, fy, apt, displayH);
    if (apt != nullptr) {
      const float rowH = mfdFontPx(kWtListRow, displayH);
      const float rowSize = mfdFontPx(kWtRow, displayH);

      // Region + field elevation.
      float cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy,
                 apt->region.empty() ? std::string(kDash)
                                     : regionName(apt->region),
                 rowSize, TextAlign::Left, colors::kWhitesmoke);
      if (apt->elevationFt != 0.0f) {
        std::snprintf(buf, sizeof(buf), "%d",
                      static_cast<int>(apt->elevationFt));
        drawValueWithUnit(r, inner.x + inner.w, cy, buf, "FT", rowSize,
                          colors::kWhitesmoke);
      } else {
        r.fillText(inner.x + inner.w, cy, kDash, rowSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
      fy += rowH;

      // Latitude + fuel availability, then longitude on its own row.
      cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy, formatLatLon(apt->lat, true), rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
      if (apt->airportServiced) {
        r.fillText(inner.x + inner.w, cy, "AVGAS", rowSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
      fy += rowH;
      cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy, formatLatLon(apt->lon, false), rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    }
  }

  // Runways box: designation / dimensions / surface / lighting.
  {
    Rect inner = drawGroupBox(r, stack.slot(138.0f), "Runways", displayH);
    drawRunwayGroup(r, inner,
                    apt != nullptr ? ui.airportRunways(apt->id)
                                   : std::vector<AirportRunwayInfo>{},
                    0, 4, displayH);
  }

  // Frequencies box fills the rest of the panel.
  {
    Rect inner = drawGroupBox(r, stack.slot(stack.remainingWt()),
                              "Frequencies", displayH);
    drawFrequencyGroup(r, inner, displayH, 7,
                       apt != nullptr ? ui.airportFrequencies(apt->id)
                                      : std::vector<MapAirportFrequency>{});
  }
}

namespace {

// "Nearest Airport" box contents (Fig 5-33/5-35): cyan ident with the airport
// symbol, then bearing/distance from the navaid to that airport.
void drawNearestAirportBox(Renderer& r, const Rect& inner, const MapData& map,
                           const MapFeature* navaid, float displayH) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  const MapFeature* apt = nearestFeature(map, MapFeatureType::Airport);
  float cy = inner.y + rowH * 0.5f;
  if (apt == nullptr || navaid == nullptr) {
    r.fillText(inner.x, cy, kDash, rowSize, TextAlign::Left, colors::kCyan);
    return;
  }
  char buf[24];
  r.fillText(inner.x, cy, apt->id, rowSize, TextAlign::Left, colors::kCyan);
  drawWaypointIcon(r, inner.x + inner.w * 0.5f, cy, mfdFontPx(22.0f, displayH),
                   apt, MapFeatureType::Airport);
  cy += rowH;
  std::snprintf(buf, sizeof(buf), "%03.0f",
                navBearingDeg(navaid->lat, navaid->lon, apt->lat, apt->lon));
  drawValueWithUnit(r, inner.x + inner.w * 0.40f, cy, buf, kDeg, rowSize,
                    colors::kWhitesmoke);
  std::snprintf(buf, sizeof(buf), "%.1f",
                navDistanceNm(navaid->lat, navaid->lon, apt->lat, apt->lon));
  drawValueWithUnit(r, inner.x + inner.w, cy, buf, "NM", rowSize,
                    colors::kWhitesmoke);
}

// Stacked region / latitude / longitude rows shared by the Information boxes
// (Fig 5-31/33/35 show them left-aligned without labels). Returns the next y.
float drawRegionLatLonRows(Renderer& r, const Rect& inner, float fy,
                           const MapFeature* navaid, float displayH) {
  const float rowH = mfdFontPx(kWtListRow, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  auto row = [&](const std::string& text) {
    r.fillText(inner.x, fy + rowH * 0.5f, text, rowSize, TextAlign::Left,
               colors::kWhitesmoke);
    fy += rowH;
  };
  row(navaid != nullptr && !navaid->region.empty()
          ? regionName(navaid->region)
          : std::string(kDash));
  row(navaid != nullptr ? formatLatLon(navaid->lat, true)
                        : std::string(kDash));
  row(navaid != nullptr ? formatLatLon(navaid->lon, false)
                        : std::string(kDash));
  return fy;
}

}  // namespace

void drawWaypointNavaidPage(Renderer& r, const FlightData& d,
                            const MapData& map, const MfdController& ui,
                            MapFeatureType type, float x, float y, float w,
                            float h, float displayH) {
  // WPT Intersection/NDB/VOR Information (Fig 5-31/5-33/5-35): map left,
  // panel right. Intersections stack Intersection / Information / Nearest
  // VOR; navaids stack the facility box (with name), Information, Frequency,
  // and Nearest Airport.
  const bool isIntersection = type == MapFeatureType::Fix;
  const bool isVor = type == MapFeatureType::Vor;
  const char* subject = isIntersection ? "Intersection"
                        : type == MapFeatureType::Ndb ? "NDB"
                                                      : "VOR";
  const MapFeature* navaid = wptFacility(map, ui, type);

  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, isVor ? 15.0f : 7.5f, navaid, displayH,
              isIntersection);

  PanelStack stack(f.panel, displayH);
  char buf[24];

  // Facility box: ident header; navaids add the facility name row.
  {
    Rect inner = drawGroupBox(r, stack.slot(isIntersection ? 64.0f : 96.0f),
                              subject, displayH);
    float fy = drawWptFacilityHeader(r, inner, navaid, type, ui, displayH);
    if (!isIntersection) {
      drawFacilityNameCity(r, inner, fy, navaid, displayH);
    }
  }

  // Information box: region + location; VORs lead with the class row and the
  // slaved magnetic variation; NDBs with the station kind.
  {
    Rect inner = drawGroupBox(r, stack.slot(isIntersection ? 112.0f : 140.0f),
                              "Information", displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    float fy = inner.y;
    if (isVor) {
      const float cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy,
                 navaid != nullptr ? vorClassName(navaid->rangeNm) : kDash,
                 rowSize, TextAlign::Left, colors::kWhitesmoke);
      const std::string magvar =
          navaid != nullptr ? formatMagvar(*navaid) : std::string();
      if (!magvar.empty()) {
        r.fillText(inner.x + inner.w, cy, magvar, rowSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
      fy += rowH;
    } else if (type == MapFeatureType::Ndb) {
      std::string kind = "NDB";
      if (navaid != nullptr && !navaid->navaidType.empty()) {
        if (navaid->navaidType == "LOM" || navaid->navaidType == "LMM") {
          kind = "Compass Locator (" + navaid->navaidType + ")";
        } else {
          kind = navaid->navaidType;
        }
      }
      r.fillText(inner.x, fy + rowH * 0.5f, navaid != nullptr ? kind : kDash,
                 rowSize, TextAlign::Left, colors::kWhitesmoke);
      fy += rowH;
    }
    drawRegionLatLonRows(r, inner, fy, navaid, displayH);
  }

  if (isIntersection) {
    // Nearest VOR box (Fig 5-31): ident + symbol, then the radial from the
    // VOR and the distance to it.
    Rect inner = drawGroupBox(r, stack.slot(112.0f), "Nearest VOR", displayH);
    const MapFeature* vor = nearestFeature(map, MapFeatureType::Vor);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
    float cy = inner.y + rowH * 0.5f;
    if (vor != nullptr && navaid != nullptr) {
      r.fillText(inner.x, cy, vor->id, rowSize, TextAlign::Left, colors::kCyan);
      drawWaypointIcon(r, inner.x + inner.w * 0.5f, cy,
                       mfdFontPx(22.0f, displayH), vor, MapFeatureType::Vor);
      cy += rowH;
      // Radial FROM the VOR to the intersection (magnetic when the VOR's
      // slaved variation is known).
      double radial = navBearingDeg(vor->lat, vor->lon, navaid->lat,
                                    navaid->lon);
      if (vor->hasMagvar) {
        radial -= static_cast<double>(vor->magvarDeg);
        while (radial < 0.0) radial += 360.0;
        while (radial >= 360.0) radial -= 360.0;
      }
      r.fillText(inner.x, cy, "RAD", labelSize, TextAlign::Left,
                 colors::kTitleGray);
      std::snprintf(buf, sizeof(buf), "%03.0f", radial);
      drawValueWithUnit(r, inner.x + inner.w * 0.60f, cy, buf, kDeg, rowSize,
                        colors::kWhitesmoke);
      cy += rowH;
      r.fillText(inner.x, cy, "DIS", labelSize, TextAlign::Left,
                 colors::kTitleGray);
      std::snprintf(buf, sizeof(buf), "%.1f",
                    navDistanceNm(navaid->lat, navaid->lon, vor->lat,
                                  vor->lon));
      drawValueWithUnit(r, inner.x + inner.w * 0.60f, cy, buf, "NM", rowSize,
                        colors::kWhitesmoke);
    } else {
      r.fillText(inner.x, cy, kDash, rowSize, TextAlign::Left, colors::kCyan);
    }
    return;
  }

  // Frequency box: VOR shows the pill (tunable), NDB plain text.
  {
    Rect inner = drawGroupBox(r, stack.slot(64.0f), "Frequency", displayH);
    const std::string freq =
        navaid != nullptr ? formatFrequency(type, navaid->frequency)
                          : std::string(kDash);
    const float cy = inner.y + mfdFontPx(kWtListRow, displayH) * 0.5f;
    if (isVor) {
      const float pillW = inner.w * 0.34f;
      const float pillH = mfdFontPx(24.0f, displayH);
      drawPill(r, Rect{inner.x, cy - pillH * 0.5f, pillW, pillH}, freq,
               mfdFontPx(18.0f, displayH), colors::kWhitesmoke);
    } else {
      r.fillText(inner.x + mfdFontPx(10.0f, displayH), cy, freq,
                 mfdFontPx(kWtRow, displayH), TextAlign::Left,
                 colors::kWhitesmoke);
    }
  }

  // Nearest Airport box (Fig 5-33/5-35).
  {
    Rect inner = drawGroupBox(r, stack.slot(102.0f), "Nearest Airport",
                              displayH);
    drawNearestAirportBox(r, inner, map, navaid, displayH);
  }
}

}  // namespace avionics::mfd
