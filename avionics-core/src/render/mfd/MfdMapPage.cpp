#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/NavMath.h"
#include "avionics/render/MapSymbols.h"
#include "render/map/MapProjection.h"
#include "render/map/MapViewInternal.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

// Compact label-left / value-right row used by the Map Pointer information box.
void drawPointerRow(Renderer& r, float x, float w, float cy, const char* label,
                    const std::string& value, float sizePx,
                    const Color& valueColor) {
  r.fillText(x, cy, label, sizePx, TextAlign::Left, colors::kTitleGray);
  r.fillText(x + w, cy, value, sizePx, TextAlign::Right, valueColor);
}

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
        std::max(0.5f, ui.displayRangeNm() > 0.0f ? ui.displayRangeNm()
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

    // Information box, top-center (clear of the range and orientation labels).
    if (map.positionValid) {
      const float pad = mfdFontPx(8.0f, displayH);
      const float labelSize = mfdFontPx(15.0f, displayH);
      const float identSize = mfdFontPx(20.0f, displayH);
      const float rowH = labelSize * 1.4f;
      const bool hasFeat = sel != nullptr;
      const bool hasOb = selectedObstacle != nullptr;
      const float boxW = w * 0.30f;
      const float boxX = x + w * 0.5f - boxW * 0.5f;
      const float boxY = y + h * 0.04f;
      const float extraRows = hasOb ? 1.0f : 0.0f;
      const float boxH =
          pad * 2.0f + (hasFeat ? identSize * 1.35f : 0.0f) +
          rowH * (4.0f + extraRows);

      r.fillRect(boxX, boxY, boxW, boxH, Color{0.0f, 0.0f, 0.0f, 0.82f});
      r.strokeLine(boxX, boxY, boxX + boxW, boxY, 1.0f, colors::kPanelBorder);
      r.strokeLine(boxX, boxY + boxH, boxX + boxW, boxY + boxH, 1.0f,
                   colors::kPanelBorder);
      r.strokeLine(boxX, boxY, boxX, boxY + boxH, 1.0f, colors::kPanelBorder);
      r.strokeLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.0f,
                   colors::kPanelBorder);

      const float innerX = boxX + pad;
      const float innerW = boxW - 2.0f * pad;
      float ty = boxY + pad;
      if (hasFeat) {
        r.fillText(boxX + boxW * 0.5f, ty + identSize * 0.5f, sel->id,
                   identSize, TextAlign::Center, mapFeatureColor(*sel));
        ty += identSize * 1.35f;
      } else if (hasOb) {
        char elevBuf[24];
        std::snprintf(elevBuf, sizeof(elevBuf), "ELEV %dFT",
                      static_cast<int>(std::lround(selectedObstacle->mslFt)));
        r.fillText(boxX + boxW * 0.5f, ty + identSize * 0.5f, elevBuf,
                   identSize, TextAlign::Center,
                   obstaclePointerColor(*selectedObstacle, d));
        ty += identSize * 1.35f;
      }

      const double brg =
          navBearingDeg(map.ownshipLat, map.ownshipLon, ptLat, ptLon);
      const double dis =
          navDistanceNm(map.ownshipLat, map.ownshipLon, ptLat, ptLon);
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%03d%s",
                    static_cast<int>(std::lround(brg)) % 360, kDeg);
      drawPointerRow(r, innerX, innerW, ty + rowH * 0.5f, "BRG", buf, labelSize,
                     colors::kWhite);
      ty += rowH;
      std::snprintf(buf, sizeof(buf), dis < 100.0 ? "%.1fNM" : "%.0fNM", dis);
      drawPointerRow(r, innerX, innerW, ty + rowH * 0.5f, "DIS", buf, labelSize,
                     colors::kWhite);
      ty += rowH;

      r.fillText(boxX + boxW * 0.5f, ty + rowH * 0.5f,
                 formatLatLon(ptLat, true), labelSize, TextAlign::Center,
                 colors::kWhite);
      ty += rowH;
      r.fillText(boxX + boxW * 0.5f, ty + rowH * 0.5f,
                 formatLatLon(ptLon, false), labelSize, TextAlign::Center,
                 colors::kWhite);
      if (hasOb) {
        ty += rowH;
        char obBuf[32];
        std::snprintf(obBuf, sizeof(obBuf), "%dFT MSL / %dFT AGL",
                      static_cast<int>(std::lround(selectedObstacle->mslFt)),
                      static_cast<int>(std::lround(selectedObstacle->aglFt)));
        r.fillText(boxX + boxW * 0.5f, ty + rowH * 0.5f, obBuf, labelSize,
                   TextAlign::Center,
                   obstaclePointerColor(*selectedObstacle, d));
      }
    }
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
