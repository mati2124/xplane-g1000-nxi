#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "avionics/NavMath.h"
#include "avionics/Terrain.h"
#include "avionics/render/MapSymbols.h"
#include "render/map/MapProjection.h"
#include "render/map/MapViewInternal.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

// PROC approach preview auto-fit. The procedure's half-extent (from its bbox
// center) should fill this fraction of the labeled range ring when the map
// zooms to frame it; smaller leaves more margin around the legs.
constexpr float kProcPreviewFitFactor = 1.7f;
// The PROC window covers this fraction of the map body on the right; the
// preview view is nudged east by half of it so the legs sit in the uncovered
// area to the left rather than behind the window (matches the trainer).
constexpr float kProcPreviewWindowFrac = 0.40f;
constexpr double kProcPreviewDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kProcPreviewNmPerDegLat = 60.0;

Color obstaclePointerColor(const MapObstacle& ob, const FlightData& d) {
  if (!d.altitudeValid) return colors::kWhite;
  const float rel = ob.mslFt - d.altitudeFt;
  if (rel >= -100.0f) return colors::kBandRed;
  if (rel >= -1000.0f) return colors::kBandYellow;
  return colors::kWhite;
}

// Map rotation (degrees) for the active orientation, mirroring MapView's
// projection so the pointer's feature highlight lands on the same pixel.
float pointerRotationDeg(MapOrientation orientation, const FlightData& d) {
  switch (orientation) {
    case MapOrientation::HeadingUp:
      return d.headingDeg;
    case MapOrientation::TrackUp:
      return d.trackDeg;
    case MapOrientation::NorthUp:
      break;
  }
  return 0.0f;
}

// One vertical bound of an airspace as the NXi prints it: "Surface" at the
// ground, "Unlimited" for no published ceiling, else "<alt>FT msl".
std::string airspaceLevelText(float ft) {
  if (ft <= 0.5f) return "Surface";
  if (ft >= kAirspaceUnlimitedFt) return "Unlimited";
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%dFT msl",
                static_cast<int>(std::lround(ft)));
  return buf;
}

// One text line of the near-cursor selection box. `groupStart` adds a small gap
// above the line so distinct items (the feature, each airspace) read apart.
struct PointerInfoLine {
  std::string text;
  float sizePx;
  Color color;
  bool groupStart;
};

// Selection box beside the pan cursor (Pilot's Guide, Map Panning): describes
// whatever the pointer is over -- the highlighted feature (ident + facility
// name) and/or each airspace the cursor lies within (name, class, vertical
// limits) -- right at the cursor, the way the real unit does, rather than in the
// fixed top-left readout. The feature's cyan ring and the airspace boundary
// highlight are drawn elsewhere.
void drawPointerSelectionBox(Renderer& r, const MfdController& ui,
                             const MapFeature* feature, float x, float y,
                             float w, float h, float ptrX, float ptrY,
                             float displayH) {
  const std::vector<const MapAirspace*> airspaces = ui.mapPointerAirspaces();
  if (feature == nullptr && airspaces.empty()) return;

  const float textSize = mfdFontPx(15.0f, displayH);
  const float identSize = mfdFontPx(18.0f, displayH);

  // Flatten the selection into display lines up front so the box height and the
  // draw pass agree (a missing facility name simply drops that line).
  std::vector<PointerInfoLine> lines;
  if (feature != nullptr) {
    lines.push_back({feature->id, identSize, mapFeatureColor(*feature), true});
    if (!feature->name.empty()) {
      lines.push_back({feature->name, textSize, colors::kWhite, false});
    }
  }
  for (const MapAirspace* as : airspaces) {
    bool first = true;
    auto addLine = [&](std::string s) {
      lines.push_back({std::move(s), textSize, colors::kWhite, first});
      first = false;
    };
    if (!as->name.empty()) addLine(as->name);
    addLine(airspaceClassLabel(as->airspaceClass));
    addLine(airspaceLevelText(as->floorFt) + " to " +
            airspaceLevelText(as->ceilingFt));
  }
  if (lines.empty()) return;

  const float pad = mfdFontPx(7.0f, displayH);
  const float groupGap = mfdFontPx(6.0f, displayH);
  auto lineHeight = [](float sizePx) { return sizePx * 1.35f; };

  // Size the box to its widest line (measured in the bold face it draws in) so
  // it only spans the text, not a fixed slice of the map.
  float boxH = pad * 2.0f;
  float maxLineW = 0.0f;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i > 0 && lines[i].groupStart) boxH += groupGap;
    boxH += lineHeight(lines[i].sizePx);
    maxLineW = std::max(maxLineW,
                        r.measureTextWidth(lines[i].text, lines[i].sizePx,
                                           FontFace::RobotoBold));
  }
  const float boxW = std::min(maxLineW + pad * 2.0f, w * 0.45f);

  // Anchor beside the cursor tip, clamped to stay within the map viewport.
  float boxX = ptrX + mfdFontPx(16.0f, displayH);
  float boxY = ptrY - boxH * 0.5f;
  boxX = std::clamp(boxX, x + pad, x + w - boxW - pad);
  boxY = std::clamp(boxY, y + pad, y + h - boxH - pad);

  r.fillRect(boxX, boxY, boxW, boxH, Color{0.0f, 0.0f, 0.0f, 0.82f});
  r.strokeLine(boxX, boxY, boxX + boxW, boxY, 1.0f, colors::kPanelBorder);
  r.strokeLine(boxX, boxY + boxH, boxX + boxW, boxY + boxH, 1.0f,
               colors::kPanelBorder);
  r.strokeLine(boxX, boxY, boxX, boxY + boxH, 1.0f, colors::kPanelBorder);
  r.strokeLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.0f,
               colors::kPanelBorder);

  // The selection description renders in the bold face, like the real unit.
  FontScope boldScope(r, FontFace::RobotoBold);
  const float innerX = boxX + pad;
  float ty = boxY + pad;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i > 0 && lines[i].groupStart) ty += groupGap;
    const float lh = lineHeight(lines[i].sizePx);
    r.fillText(innerX, ty + lh * 0.5f, lines[i].text, lines[i].sizePx,
               TextAlign::Left, lines[i].color);
    ty += lh;
  }
}

}  // namespace

void drawMapPage(Renderer& r, const FlightData& d, const MapData& map,
                 MfdController& ui, float x, float y, float w, float h,
                 float displayH) {
  MapViewConfig config;
  config.x = x;
  config.y = y;
  config.w = w;
  config.h = h;
  config.orientation = ui.mapOrientation();
  config.rangeNm = ui.rangeNm();
  config.displayRangeNm = ui.displayRangeNm();
  config.style.showChrome = true;
  config.style.showNorthArrow = true;
  config.style.terrain = ui.terrainDisplay();
  config.style.airways = ui.airwayDisplay();
  config.style.showTraffic = ui.showTraffic();
  config.style.trafficSymbolsRangeNm =
      ui.mapSettingRangeNm(MapSetting::TrafficSymbolsRange);
  config.style.showTrafficLabels = ui.mapSettingOn(MapSetting::TrafficLabelsOn);
  config.style.trafficLabelsRangeNm =
      ui.mapSettingRangeNm(MapSetting::TrafficLabelsRange);
  config.style.showWeather = ui.showWeather();
  config.style.nexradRangeNm = ui.mapSettingRangeNm(MapSetting::NexradRange);
  // Map Setup "Map" group items, driven by the Map Settings window (Fig. 5-7).
  config.style.showTrackVector = ui.mapSettingOn(MapSetting::TrackVectorOn);
  config.style.showWindVector = ui.mapSettingOn(MapSetting::WindVectorOn);
  config.style.showFuelRing = ui.mapSettingOn(MapSetting::FuelRangeOn);
  config.style.showObstacles = ui.mapSettingOn(MapSetting::ObstacleOn);
  config.style.obstacleRangeNm = ui.mapSettingRangeNm(MapSetting::ObstacleRange);
  config.style.showFixes = ui.mapSettingOn(MapSetting::IntOn);
  // Map Setup "Aviation" group: per-size airport visibility + max display range
  // (Fig. 5-7), so the wide view keeps the major airports as the small ones
  // declutter off.
  config.style.showLargeAirports = ui.mapSettingOn(MapSetting::LargeAirportOn);
  config.style.showMediumAirports = ui.mapSettingOn(MapSetting::MediumAirportOn);
  config.style.showSmallAirports = ui.mapSettingOn(MapSetting::SmallAirportOn);
  config.style.largeAirportRangeNm =
      ui.mapSettingRangeNm(MapSetting::LargeAirportRange);
  config.style.mediumAirportRangeNm =
      ui.mapSettingRangeNm(MapSetting::MediumAirportRange);
  config.style.smallAirportRangeNm =
      ui.mapSettingRangeNm(MapSetting::SmallAirportRange);
  config.style.terrainMaxRangeNm = ui.mapSettingRangeNm(MapSetting::TerrainRange);
  config.style.labelFontWt = 18.0f;
  applyMapDetail(config.style, ui.mapDetail());
  const MapFeature* pointerFeature = ui.mapPointerActive() ? ui.mapPointerFeature()
                                                           : nullptr;
  const MapObstacle* pointerObstacle =
      ui.mapPointerActive() ? ui.mapPointerObstacle() : nullptr;
  const MapObstacle* selectedObstacle = nullptr;
  if (pointerObstacle != nullptr) {
    if (pointerFeature == nullptr) {
      selectedObstacle = pointerObstacle;
    } else {
      const double featNm = navDistanceNm(ui.mapPointerLat(), ui.mapPointerLon(),
                                          pointerFeature->lat, pointerFeature->lon);
      const double obNm = navDistanceNm(ui.mapPointerLat(), ui.mapPointerLon(),
                                        pointerObstacle->lat, pointerObstacle->lon);
      if (obNm <= featNm) {
        selectedObstacle = pointerObstacle;
      }
    }
  }
  config.selectedObstacle = selectedObstacle;
  if (ui.mapPointerActive()) {
    ui.setMapViewport(x, y, w, h, displayH);
    ui.mapPointerSyncScroll(d);
    config.hasCenterOverride = true;
    config.centerLat = ui.mapPanViewCenterLat();
    config.centerLon = ui.mapPanViewCenterLon();
    // Let the airspace layer highlight whatever the cursor is over.
    config.pointerActive = true;
    config.pointerLat = ui.mapPointerLat();
    config.pointerLon = ui.mapPointerLon();
  }

  // PROC -> Select Approach preview: while the Approach Loading window is up,
  // the MFD map frames the highlighted procedure's legs (Pilot's Guide 5.8),
  // auto-ranging and centering on them and drawing them North-Up like the unit.
  std::vector<MapLeg> procPreview;
  if (!ui.mapPointerActive() && ui.procMenuOpen() && ui.procSelectMode()) {
    procPreview = ui.procPreviewLegs();
  }
  if (procPreview.size() >= 2) {
    double minLat = procPreview.front().lat;
    double maxLat = minLat;
    double minLon = procPreview.front().lon;
    double maxLon = minLon;
    for (const MapLeg& leg : procPreview) {
      minLat = std::min(minLat, leg.lat);
      maxLat = std::max(maxLat, leg.lat);
      minLon = std::min(minLon, leg.lon);
      maxLon = std::max(maxLon, leg.lon);
    }
    const double previewCenterLat = (minLat + maxLat) * 0.5;
    const double previewCenterLon = (minLon + maxLon) * 0.5;
    double maxNm = 0.0;
    for (const MapLeg& leg : procPreview) {
      maxNm = std::max(maxNm, navDistanceNm(previewCenterLat, previewCenterLon,
                                            leg.lat, leg.lon));
    }
    config.orientation = MapOrientation::NorthUp;
    // The real unit shows no terrain shading on the approach preview.
    config.style.terrain = TerrainDisplay::Off;
    const float fitRingNm = static_cast<float>(maxNm) / kProcPreviewFitFactor;
    int idx = kMapRangeCloseRungCount;  // start at 0.5 NM, skip the foot steps
    while (idx < kMapRangeLadderCount - 1 && mapRangeNmAt(idx) < fitRingNm) {
      ++idx;
    }
    // The RANGE knob still zooms the preview (Pilot's Guide 5.8): auto-frame the
    // legs until the user turns the knob, then honor the manual range. Keep the
    // ladder synced to the fit while auto-ranging so the first turn steps from
    // the framed range.
    if (ui.procPreviewRangeManual()) {
      config.rangeNm = ui.rangeNm();
      config.displayRangeNm = ui.displayRangeNm();
    } else {
      ui.setProcPreviewFitRange(idx);
      config.rangeNm = mapRangeNmAt(idx);
      config.displayRangeNm = config.rangeNm;
    }
    config.hasCenterOverride = true;
    config.centerLat = previewCenterLat;
    // Shift the view east so the legs land in the map area to the left of the
    // PROC window instead of being hidden behind it.
    const float mapRadiusPx = mapview::mapRangeSpanPx(config);
    const float pixelsPerNm =
        mapRadiusPx / std::max(kMapRangeMinNm, config.displayRangeNm);
    const float shiftPx = w * kProcPreviewWindowFrac * 0.5f;
    const double shiftNm = pixelsPerNm > 0.0f ? shiftPx / pixelsPerNm : 0.0;
    const double cosLat =
        std::max(0.05, std::cos(previewCenterLat * kProcPreviewDegToRad));
    config.centerLon =
        previewCenterLon + shiftNm / (kProcPreviewNmPerDegLat * cosLat);
    config.procedurePreview = &procPreview;
  }
  MapView::render(r, map, d, config, displayH);

  // Map Panning (G1000 NXi Pilot's Guide, Map Pointer): when pointer mode is
  // active a flashing arrow cursor marks the pointer position on the map (the
  // view stays put until the cursor reaches the inner edge, then scrolls), the
  // feature under it is highlighted, and an information box shows the
  // bearing/distance from present position and the pointer's coordinates.
  if (ui.mapPointerActive()) {
    const float cx = x + w * 0.5f;
    const float cy = y + h * 0.5f;
    const double viewLat = ui.mapPanViewCenterLat();
    const double viewLon = ui.mapPanViewCenterLon();
    const double ptLat = ui.mapPointerLat();
    const double ptLon = ui.mapPointerLon();
    const MapFeature* sel = selectedObstacle != nullptr ? nullptr : pointerFeature;
    const float mapRadiusPx = mapview::mapRangeSpanPx(config);
    const float scaleRangeNm =
        std::max(kMapRangeMinNm, ui.displayRangeNm() > 0.0f ? ui.displayRangeNm()
                                                  : ui.rangeNm());
    const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
    const float rotation = pointerRotationDeg(ui.mapOrientation(), d);
    float ptrX = cx;
    float ptrY = cy;
    if (map.positionValid) {
      map::latLonToLocalPx(ptLat, ptLon, viewLat, viewLon, cx, cy, pixelsPerNm,
                           rotation, ptrX, ptrY);
    }

    // Highlight the selected feature with a cyan ring at its projected
    // position (near the pointer tip when the pointer is over it).
    if (sel != nullptr && map.positionValid) {
      float fx = 0.0f, fy = 0.0f;
      map::latLonToLocalPx(sel->lat, sel->lon, viewLat, viewLon, cx, cy,
                           pixelsPerNm, rotation, fx, fy);
      strokeCircle(r, fx, fy, std::min(w, h) * 0.024f, 2.0f, colors::kCyan);
    }

    if (selectedObstacle != nullptr && map.positionValid) {
      const Color obC = obstaclePointerColor(*selectedObstacle, d);
      const float obstacleSize =
          std::max(11.0f, mapview::fontPx(mapview::kObstacleSymbolWt, displayH));
      const float labelSize = mapview::fontPx(config.style.labelFontWt, displayH);
      mapview::drawObstacleSelectedTag(r, ptrX, ptrY, obstacleSize,
                                       selectedObstacle->mslFt,
                                       selectedObstacle->aglFt, labelSize, obC);
    }

    mapview::drawMapPointer(r, ptrX, ptrY, displayH,
                            ui.mapPointerFlashInverted());

    // Pointer information box, anchored top-left like the real unit (Pilot's
    // Guide, Map Panning): a fixed two-row readout of the cursor's bearing and
    // distance from present position, the ground elevation under it, and its
    // coordinates -- it does NOT float by the cursor. Whatever the cursor is
    // actually over (a feature or airspace) is described in a box at the cursor
    // instead (drawPointerSelectionBox); obstacles get their MSL/AGL tag there.
    if (map.positionValid) {
      const float pad = mfdFontPx(7.0f, displayH);
      const float labelSize = mfdFontPx(15.0f, displayH);
      const float rowH = labelSize * 1.4f;
      const float boxW = w * 0.34f;
      const float boxX = x + pad;
      const float boxY = y + pad;
      const float boxH = pad * 2.0f + rowH * 2.0f;

      r.fillRect(boxX, boxY, boxW, boxH, Color{0.0f, 0.0f, 0.0f, 0.82f});
      r.strokeLine(boxX, boxY, boxX + boxW, boxY, 1.0f, colors::kPanelBorder);
      r.strokeLine(boxX, boxY + boxH, boxX + boxW, boxY + boxH, 1.0f,
                   colors::kPanelBorder);
      r.strokeLine(boxX, boxY, boxX, boxY + boxH, 1.0f, colors::kPanelBorder);
      r.strokeLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.0f,
                   colors::kPanelBorder);

      // The whole readout renders in the bold face, like the real unit.
      FontScope boldScope(r, FontFace::RobotoBold);
      const float innerX = boxX + pad;
      const float innerW = boxW - 2.0f * pad;
      float ty = boxY + pad;

      const double brg =
          navBearingDeg(map.ownshipLat, map.ownshipLon, ptLat, ptLon);
      const double dis =
          navDistanceNm(map.ownshipLat, map.ownshipLon, ptLat, ptLon);

      // Column anchors within the box: the DIS/ELEV value column, the BRG
      // label+value column, then the lat/lon stacked at the right edge.
      const float col1ValX = innerX + innerW * 0.30f;
      const float col2LabelX = innerX + innerW * 0.37f;
      const float col2ValX = innerX + innerW * 0.62f;
      const float rightX = innerX + innerW;

      char buf[24];
      // Row 1: DIS .. BRG .. latitude.
      float cy = ty + rowH * 0.5f;
      r.fillText(innerX, cy, "DIS", labelSize, TextAlign::Left,
                 colors::kTitleGray);
      std::snprintf(buf, sizeof(buf), dis < 100.0 ? "%.1fNM" : "%.0fNM", dis);
      r.fillText(col1ValX, cy, buf, labelSize, TextAlign::Right, colors::kCyan);
      r.fillText(col2LabelX, cy, "BRG", labelSize, TextAlign::Left,
                 colors::kTitleGray);
      std::snprintf(buf, sizeof(buf), "%03d%s",
                    static_cast<int>(std::lround(brg)) % 360, kDeg);
      r.fillText(col2ValX, cy, buf, labelSize, TextAlign::Right, colors::kCyan);
      r.fillText(rightX, cy, formatLatLon(ptLat, true), labelSize,
                 TextAlign::Right, colors::kCyan);
      ty += rowH;

      // Row 2: ELEV (ground under the cursor) .. longitude.
      cy = ty + rowH * 0.5f;
      const float elevFt =
          map.terrain != nullptr
              ? map.terrain->elevationFt(ptLat, ptLon)
              : std::numeric_limits<float>::quiet_NaN();
      std::string elevStr;
      if (std::isnan(elevFt)) {
        elevStr = "___FT";
      } else {
        char elevBuf[16];
        std::snprintf(elevBuf, sizeof(elevBuf), "%dFT",
                      static_cast<int>(std::lround(elevFt)));
        elevStr = elevBuf;
      }
      r.fillText(innerX, cy, "ELEV", labelSize, TextAlign::Left,
                 colors::kTitleGray);
      r.fillText(col1ValX, cy, elevStr, labelSize, TextAlign::Right,
                 colors::kCyan);
      r.fillText(rightX, cy, formatLatLon(ptLon, false), labelSize,
                 TextAlign::Right, colors::kCyan);
    }

    // Whatever the cursor is over (feature and/or airspace): description box
    // beside the cursor tip (the feature ring and airspace boundary are
    // highlighted on the map itself).
    drawPointerSelectionBox(r, ui, sel, x, y, w, h, ptrX, ptrY, displayH);
  }
}

namespace {

// Dedicated Traffic Map ranges (NM): outer ring / inner ring, the NXi default
// traffic scope (Pilot's Guide, Hazard Avoidance - Traffic Map Page).
constexpr float kTrafficOuterNm = 6.0f;
constexpr float kTrafficInnerNm = 2.0f;

// Relative-altitude tag like the NXi (hundreds of feet, signed two digits).
std::string formatRelAlt(float relAltFt) {
  const int hundreds = static_cast<int>(std::lround(relAltFt / 100.0f));
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%+03d", hundreds);
  return buf;
}

}  // namespace

void drawTrafficMapPage(Renderer& r, const FlightData& d, const MapData& map,
                        const MfdController& ui, float x, float y, float w,
                        float h, float displayH) {
  (void)ui;
  r.fillRect(x, y, w, h, colors::kBlack);

  const float cx = x + w * 0.5f;
  const float cy = y + h * 0.5f;
  const float outerR = std::min(w, h) * 0.40f;
  const float innerR = outerR * (kTrafficInnerNm / kTrafficOuterNm);
  const float nmToPx = outerR / kTrafficOuterNm;

  strokeCircle(r, cx, cy, outerR, 1.0f, colors::kGroupBoxBorder);
  strokeCircle(r, cx, cy, innerR, 1.0f, colors::kGroupBoxBorder);

  // Ownship symbol at center (heading-up, so it points straight up).
  const float os = std::min(w, h) * 0.022f;
  const Point ship[4] = {{cx, cy - os},
                         {cx - os * 0.7f, cy + os * 0.7f},
                         {cx, cy + os * 0.35f},
                         {cx + os * 0.7f, cy + os * 0.7f}};
  r.fillPolygon(ship, 4, colors::kWhite);

  // Mode / status annunciations, NXi style.
  const float labelSize = mfdFontPx(16.0f, displayH);
  const bool operating = d.dataLinkValid && map.positionValid;
  r.fillText(x + mfdFontPx(12.0f, displayH), y + mfdFontPx(20.0f, displayH),
             operating ? "OPERATING" : "NO DATA", labelSize, TextAlign::Left,
             operating ? colors::kActiveGreen : colors::kBandYellow);
  r.fillText(x + w - mfdFontPx(12.0f, displayH), y + mfdFontPx(20.0f, displayH),
             "TAS", labelSize, TextAlign::Right, colors::kWhite);

  // Range-ring labels at the ring tops (the NXi labels both rings in NM).
  char rngBuf[12];
  std::snprintf(rngBuf, sizeof(rngBuf), "%d", static_cast<int>(kTrafficOuterNm));
  r.fillText(cx + outerR * 0.10f, cy - outerR + labelSize * 0.5f, rngBuf,
             mfdFontPx(14.0f, displayH), TextAlign::Left, colors::kTitleGray);
  std::snprintf(rngBuf, sizeof(rngBuf), "%d", static_cast<int>(kTrafficInnerNm));
  r.fillText(cx + innerR * 0.10f, cy - innerR + labelSize * 0.5f, rngBuf,
             mfdFontPx(14.0f, displayH), TextAlign::Left, colors::kTitleGray);

  if (!operating) return;

  // Plot traffic targets, heading-up: relative bearing = target bearing minus
  // ownship heading, distance scaled to the ring radii. Off-scale targets are
  // dropped (the inner two rings are the dedicated traffic scope).
  const float headingRad = d.headingDeg * 0.01745329f;
  const float symHalf = std::min(w, h) * 0.016f;
  for (const MapTraffic& t : map.traffic) {
    const double brg =
        navBearingDeg(map.ownshipLat, map.ownshipLon, t.lat, t.lon);
    const double dist =
        navDistanceNm(map.ownshipLat, map.ownshipLon, t.lat, t.lon);
    if (dist > kTrafficOuterNm) continue;
    const float a = static_cast<float>(brg) * 0.01745329f - headingRad;
    const float px = cx + static_cast<float>(dist) * nmToPx * std::sin(a);
    const float py = cy - static_cast<float>(dist) * nmToPx * std::cos(a);

    if (t.trafficAdvisory) {
      r.fillCircle(px, py, symHalf, colors::kBandYellow);
    } else {
      const Point diamond[5] = {{px, py - symHalf},
                                {px + symHalf, py},
                                {px, py + symHalf},
                                {px - symHalf, py},
                                {px, py - symHalf}};
      r.strokePolyline(diamond, 5, 1.5f, colors::kWhite);
    }

    const Color tagColor =
        t.trafficAdvisory ? colors::kBandYellow : colors::kWhite;
    const float tagSize = mfdFontPx(13.0f, displayH);
    const bool above = t.relAltFt >= 0.0f;
    const float tagY = above ? py - symHalf - tagSize * 0.5f
                             : py + symHalf + tagSize * 0.8f;
    r.fillText(px, tagY, formatRelAlt(t.relAltFt), tagSize, TextAlign::Center,
               tagColor);

    // Vertical trend arrow to the right of the symbol for >= 500 fpm.
    if (std::fabs(t.verticalSpeedFpm) >= 500.0f) {
      const float ax = px + symHalf + tagSize * 0.4f;
      const float dir = t.verticalSpeedFpm > 0.0f ? -1.0f : 1.0f;
      r.strokeLine(ax, py - symHalf * 0.8f, ax, py + symHalf * 0.8f, 1.5f,
                   tagColor);
      r.strokeLine(ax, py + dir * symHalf * 0.8f, ax - symHalf * 0.35f,
                   py + dir * symHalf * 0.3f, 1.5f, tagColor);
      r.strokeLine(ax, py + dir * symHalf * 0.8f, ax + symHalf * 0.35f,
                   py + dir * symHalf * 0.3f, 1.5f, tagColor);
    }
  }
}

}  // namespace avionics::mfd
