#include "render/mfd/MfdPages.h"

#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/MapRange.h"
#include "avionics/MetarParser.h"
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
  drawPageMap(r, d, map, f.map, isVor ? kWptVorInfoRangeNm : kWptNavInfoRangeNm,
              navaid, displayH, isIntersection);

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

namespace {

// Vertically center a single value row in a selector box (matching the PROC
// loading form): drawGroupBox trims the content rect by its bottom pad, so a
// row centered on the content alone reads high.
float boxRowCenterY(const Rect& slot, const Rect& content) {
  return (content.y + slot.y + slot.h) * 0.5f;
}

// One selector box (Departure / Arrival / Approach / Runway / Transition): the
// box title with the cyan value, dashed when empty, like the trainer.
void drawProcSelectorBox(Renderer& r, const Rect& slot, const char* title,
                         const std::string& value, float displayH) {
  const Rect box = drawGroupBox(r, slot, title, displayH);
  const float valueSize = mfdFontPx(kWtFieldValue, displayH);
  const float cy = boxRowCenterY(slot, box);
  if (value.empty()) {
    r.fillText(box.x, cy, kDash, valueSize, TextAlign::Left, colors::kCyan);
  } else {
    r.fillText(box.x, cy, value, valueSize, TextAlign::Left, colors::kCyan);
  }
}

// Sequence leg rows for the WPT procedure pages: cyan ident with the white
// leg-role suffix (iaf/faf/map/...), then the per-leg course and distance
// columns. Display-only (no cursor/scroll focus); mirrors the PROC loading
// window's drawProcSequenceRows.
void drawWptSequenceRows(Renderer& r, const Rect& area, float displayH,
                         const std::vector<MapLeg>& legs) {
  if (legs.empty()) return;
  const float rowSize = mfdFontPx(kWtFieldValue, displayH);
  const float rowH = mfdFontPx(26.0f, displayH);
  const int visible = std::max(1, static_cast<int>(area.h / rowH));
  const int end = std::min(static_cast<int>(legs.size()), visible);
  char buf[32];
  for (int i = 0; i < end; ++i) {
    const MapLeg& leg = legs[static_cast<std::size_t>(i)];
    const float cy = area.y + rowH * (static_cast<float>(i) + 0.5f);
    r.fillText(area.x, cy, leg.id, rowSize, TextAlign::Left, colors::kCyan);
    const std::string role = fplLegDisplayRole(leg);
    if (!role.empty()) {
      const float roleX =
          area.x + r.measureTextWidth(leg.id, rowSize) + rowSize * 0.35f;
      r.fillText(roleX, cy, role, rowSize, TextAlign::Left, colors::kWhite);
    }
    // The first leg has no preceding fix, so no course/distance, like the unit.
    if (i > 0) {
      const MapLeg& prev = legs[static_cast<std::size_t>(i - 1)];
      const double dtk = navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
      const double dis = navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
      const float colDisR = area.x + area.w;
      const float disColW = r.measureTextWidth("00.0", rowSize) +
                            r.measureTextWidth("NM", rowSize * kUnitEm) +
                            rowSize * 0.5f;
      std::snprintf(buf, sizeof(buf), "%03.0f", dtk);
      drawValueWithUnit(r, colDisR - disColW, cy, buf, kDeg, rowSize,
                        colors::kWhite);
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, colors::kWhite);
    }
  }
}

// Frame the procedure preview map: range the airport-to-leg span so the whole
// procedure fits, snapped to the map range ladder (auto-fit, like the PROC
// preview). Falls back to a sensible default with no legs.
float procPreviewRangeNm(const MapFeature* apt,
                         const std::vector<MapLeg>& legs) {
  if (apt == nullptr || legs.empty()) return 25.0f;
  double maxNm = 0.0;
  for (const MapLeg& leg : legs) {
    maxNm = std::max(maxNm, navDistanceNm(apt->lat, apt->lon, leg.lat, leg.lon));
  }
  const float fit = std::max(2.0f, static_cast<float>(maxNm) * 1.3f);
  return mapRangeNmAt(mapRangeIndexForNm(fit));
}

}  // namespace

void drawWaypointProcedurePage(Renderer& r, const FlightData& d,
                               const MapData& map, const MfdController& ui,
                               ProcedureType type, float x, float y, float w,
                               float h, float displayH) {
  // WPT Departure/Arrival/Approach Information (trainer apt_056..058): the
  // procedure preview map on the left, the selector boxes + Sequence on the
  // right, pre-selecting the airport's first published procedure read-only.
  const MapFeature* apt = wptFacility(map, ui, MapFeatureType::Airport);
  const WptProcedureInfo info =
      apt != nullptr ? ui.wptProcedureInfo(*apt, type) : WptProcedureInfo{};

  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  const float rangeNm = procPreviewRangeNm(apt, info.legs);
  const std::vector<MapLeg>* preview =
      info.legs.empty() ? nullptr : &info.legs;
  drawPageMap(r, d, map, f.map, rangeNm, apt, displayH, /*showFixes=*/true,
              preview, rangeNm, ui.terrainDisplay(), false, AirwayDisplay::Off,
              false);

  PanelStack stack(f.panel, displayH);
  const bool isApproach = type == ProcedureType::Approach;
  const bool isDeparture = type == ProcedureType::Departure;

  // Airport box: ident/symbol/usage header, facility name and city.
  {
    Rect inner = drawGroupBox(r, stack.slot(96.0f), "Airport", displayH);
    float fy = drawWptFacilityHeader(r, inner, apt, MapFeatureType::Airport, ui,
                                     displayH);
    drawFacilityNameCity(r, inner, fy, apt, displayH);
  }

  if (isApproach) {
    // Approach Channel (GLS channel/ID — not modeled, dashed like the trainer).
    {
      Rect ch = drawGroupBox(r, stack.slot(50.0f), "Approach Channel", displayH);
      const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
      const float valueSize = mfdFontPx(kWtFieldValue, displayH);
      const float cy = ch.y + ch.h * 0.5f;
      const char* channel = "Channel";
      const float channelW = r.measureTextWidth(channel, labelSize);
      r.fillText(ch.x, cy, channel, labelSize, TextAlign::Left, colors::kWhite);
      r.fillText(ch.x + channelW + mfdFontPx(8.0f, displayH), cy, "_____",
                 valueSize, TextAlign::Left, colors::kCyan);
      const float idX = ch.x + ch.w * 0.55f;
      r.fillText(idX, cy, "ID", labelSize, TextAlign::Left, colors::kWhite);
      r.fillText(ch.x + ch.w, cy, "_____", valueSize, TextAlign::Right,
                 colors::kWhite);
    }
    drawProcSelectorBox(r, stack.slot(46.0f), "Approach", info.name, displayH);
    drawProcSelectorBox(r, stack.slot(46.0f), "Transition", info.transition,
                        displayH);
    // Minimums (display-only OFF state; not selectable on the WPT page).
    {
      Rect mc = drawGroupBox(r, stack.slot(86.0f), "Minimums", displayH);
      const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
      const float valueSize = mfdFontPx(kWtFieldValue, displayH);
      const float smallSize = valueSize * kUnitEm;
      const float cy1 = mc.y + valueSize * 0.6f;
      const float cy2 = cy1 + valueSize * 1.6f;
      r.fillText(mc.x, cy1, "OFF", labelSize, TextAlign::Left, colors::kCyan);
      r.fillText(mc.x + mc.w, cy1, "FT", smallSize, TextAlign::Right,
                 colors::kWhitesmoke);
      std::string tempLabel = "TEMP At ";
      tempLabel += apt != nullptr ? apt->id : std::string();
      r.fillText(mc.x, cy2, tempLabel, labelSize, TextAlign::Left,
                 colors::kWhite);
      r.fillText(mc.x + mc.w, cy2, "FT", smallSize, TextAlign::Right,
                 colors::kWhitesmoke);
    }
    // Primary Frequency: localizer/VOR/NDB ident + tunable pill when known.
    {
      Rect fc = drawGroupBox(r, stack.slot(46.0f), "Primary Frequency",
                             displayH);
      const float valueSize = mfdFontPx(kWtFieldValue, displayH);
      const float rowCy = fc.y + fc.h * 0.5f;
      if (info.primaryFreqMhz > 0.0f) {
        char buf[16];
        std::snprintf(buf, sizeof(buf),
                      info.primaryIsNdb ? "%.1f" : "%.2f", info.primaryFreqMhz);
        if (!info.primaryIdent.empty()) {
          r.fillText(fc.x, rowCy, info.primaryIdent, valueSize, TextAlign::Left,
                     colors::kCyan);
        }
        const float pillW =
            r.measureTextWidth(buf, valueSize) + valueSize * 1.1f;
        drawPill(r, Rect{fc.x + fc.w - pillW, rowCy - valueSize * 0.72f, pillW,
                         valueSize * 1.44f},
                 buf, valueSize, colors::kCyan);
      } else {
        r.fillText(fc.x, rowCy, kDash, valueSize, TextAlign::Left,
                   colors::kWhite);
      }
    }
  } else {
    drawProcSelectorBox(r, stack.slot(46.0f),
                        isDeparture ? "Departure" : "Arrival", info.name,
                        displayH);
    // Trainer field order: Departure shows Runway then Transition; Arrival
    // shows Transition then Runway.
    if (isDeparture) {
      drawProcSelectorBox(r, stack.slot(46.0f), "Runway", info.runway, displayH);
      drawProcSelectorBox(r, stack.slot(46.0f), "Transition", info.transition,
                          displayH);
    } else {
      drawProcSelectorBox(r, stack.slot(46.0f), "Transition", info.transition,
                          displayH);
      drawProcSelectorBox(r, stack.slot(46.0f), "Runway", info.runway, displayH);
    }
  }

  // Sequence box fills the rest of the panel.
  {
    Rect sc = drawGroupBox(r, stack.slot(stack.remainingWt()), "Sequence",
                           displayH);
    drawWptSequenceRows(r, sc, displayH, info.legs);
  }
}

// Draw one decoded METAR row: a gray label followed inline by its white value
// (no colon, matching the real unit's WPT Weather field list). Returns the
// next row's y.
float drawMetarRow(Renderer& r, const Rect& area, float y, float step,
                   float rowSize, const char* label, const std::string& value) {
  const float cy = y + step * 0.5f;
  r.fillText(area.x, cy, label, rowSize, TextAlign::Left, colors::kTitleGray);
  const float vx = area.x + r.measureTextWidth(label, rowSize) + rowSize * 0.4f;
  r.fillText(vx, cy, value, rowSize, TextAlign::Left, colors::kWhitesmoke);
  return y + step;
}

void drawWaypointWeatherPage(Renderer& r, const FlightData& d,
                             const MapData& map, const MfdController& ui,
                             float x, float y, float w, float h,
                             float displayH) {
  // WPT Weather Information (trainer apt_059): the airport diagram on the left
  // with the Airport / METAR / TAF group boxes on the right. The METAR box
  // decodes the full field list (time, wind, visibility, clouds, temperature,
  // dew point, altimeter) and then shows the verbatim "ORIGINAL METAR TEXT";
  // the TAF box shows the verbatim forecast. Both fall back to the dashed
  // "no data" state when the shell wires no station-weather source (e.g.
  // X-Plane provides METARs but no TAFs).
  const MapFeature* apt = wptFacility(map, ui, MapFeatureType::Airport);
  PageFrame f = beginPanelPage(r, x, y, w, h, false);
  drawPageMap(r, d, map, f.map, kAirportDiagramRangeNm, apt, displayH);

  PanelStack stack(f.panel, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float rowH = mfdFontPx(kWtListRow, displayH);
  // Decoded rows stack tightly so the full field list plus the raw text fit
  // the box without scrolling (the trainer scrolls when remarks overflow; the
  // typical report fits at this pitch).
  const float step = mfdFontPx(19.0f, displayH);
  const float lineH = mfdFontPx(19.0f, displayH);

  // Airport box (compact header + name/city, matching the trainer).
  {
    Rect inner = drawGroupBox(r, stack.slot(96.0f), "Airport", displayH);
    float fy = drawWptFacilityHeader(r, inner, apt, MapFeatureType::Airport, ui,
                                     displayH);
    drawFacilityNameCity(r, inner, fy, apt, displayH);
  }

  const std::optional<StationWeather> wx =
      apt != nullptr ? ui.stationWeather(apt->id) : std::nullopt;

  // METAR box: full decoded field list, then the verbatim report.
  {
    Rect inner = drawGroupBox(r, stack.slot(330.0f), "METAR", displayH);
    if (!wx.has_value() || !wx->hasMetar()) {
      r.fillText(inner.x, inner.y + rowH * 0.5f, kDash, rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    } else {
      const MetarDecoded m = parseMetar(wx->rawMetar);
      char buf[48];
      float fy = inner.y;

      if (m.observationTime.has_value()) {
        fy = drawMetarRow(r, inner, fy, step, rowSize, "TIME",
                          *m.observationTime);
      }
      if (m.windCalm) {
        fy = drawMetarRow(r, inner, fy, step, rowSize, "WIND", "CALM");
      } else {
        if (m.windVariable) {
          fy = drawMetarRow(r, inner, fy, step, rowSize, "WIND DIRECTION",
                            "VRB");
        } else if (m.windDirectionDeg.has_value()) {
          std::snprintf(buf, sizeof(buf), "%03d%s", *m.windDirectionDeg, kDeg);
          fy = drawMetarRow(r, inner, fy, step, rowSize, "WIND DIRECTION", buf);
        }
        if (m.windSpeedKt.has_value()) {
          if (m.windGustKt.has_value()) {
            std::snprintf(buf, sizeof(buf), "%dKT G%dKT", *m.windSpeedKt,
                          *m.windGustKt);
          } else {
            std::snprintf(buf, sizeof(buf), "%dKT", *m.windSpeedKt);
          }
          fy = drawMetarRow(r, inner, fy, step, rowSize, "WIND SPEED", buf);
        }
      }
      if (m.visibility.has_value()) {
        fy = drawMetarRow(r, inner, fy, step, rowSize, "VISIBILITY",
                          *m.visibility);
      }
      if (m.clouds.has_value()) {
        fy = drawMetarRow(r, inner, fy, step, rowSize, "CLOUDS", *m.clouds);
      }
      if (m.temperatureC.has_value()) {
        std::snprintf(buf, sizeof(buf), "%d%sC",
                      static_cast<int>(std::lround(*m.temperatureC)), kDeg);
        fy = drawMetarRow(r, inner, fy, step, rowSize, "TEMPERATURE", buf);
      }
      if (m.dewPointC.has_value()) {
        std::snprintf(buf, sizeof(buf), "%d%sC",
                      static_cast<int>(std::lround(*m.dewPointC)), kDeg);
        fy = drawMetarRow(r, inner, fy, step, rowSize, "DEW POINT", buf);
      }
      if (m.altimeterInHg.has_value()) {
        std::snprintf(buf, sizeof(buf), "%.2fIN",
                      static_cast<double>(*m.altimeterInHg));
        fy = drawMetarRow(r, inner, fy, step, rowSize, "ALTIMETER", buf);
      }

      // A blank line, then the verbatim report under its heading (word-wrapped).
      fy += step * 0.5f;
      r.fillText(inner.x, fy + step * 0.5f, "ORIGINAL METAR TEXT:", rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
      fy += step;
      drawWrappedText(r, inner, fy + lineH * 0.5f, wx->rawMetar, rowSize, lineH,
                      colors::kWhitesmoke);
    }
  }

  // TAF box fills the rest: the verbatim forecast, or dashed when unavailable.
  {
    Rect inner = drawGroupBox(r, stack.slot(stack.remainingWt()), "TAF",
                              displayH);
    if (!wx.has_value() || !wx->hasTaf()) {
      r.fillText(inner.x, inner.y + rowH * 0.5f, kDash, rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    } else {
      drawWrappedText(r, inner, inner.y + lineH * 0.5f, wx->rawTaf, rowSize,
                      lineH, colors::kWhitesmoke);
    }
  }
}

}  // namespace avionics::mfd
