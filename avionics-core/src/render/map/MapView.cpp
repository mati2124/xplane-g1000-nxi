#include "avionics/render/MapView.h"

#include <algorithm>
#include <cmath>

#include "avionics/Color.h"
#include "avionics/Terrain.h"
#include "avionics/WeatherRadar.h"
#include "avionics/render/MapSymbols.h"
#include "render/map/MapProjection.h"
#include "render/map/MapViewInternal.h"
#include "render/map/TerrainRaster.h"
#include "render/map/WeatherRaster.h"

namespace avionics {
namespace {

float orientationDeg(MapOrientation mode, const FlightData& flight) {
  switch (mode) {
    case MapOrientation::NorthUp:
      return 0.0f;
    case MapOrientation::HeadingUp:
      return flight.headingDeg;
    case MapOrientation::TrackUp:
      return flight.trackDeg;
  }
  return 0.0f;
}

// Map orientation annunciation, as on the real unit ("NORTH UP" / "TRK UP" /
// "HDG UP", G1000 NXi Pilot's Guide, Navigation Map).
const char* orientationLabel(MapOrientation mode) {
  switch (mode) {
    case MapOrientation::NorthUp:
      return "NORTH UP";
    case MapOrientation::HeadingUp:
      return "HDG UP";
    case MapOrientation::TrackUp:
      return "TRK UP";
  }
  return "";
}

}  // namespace

void MapView::render(Renderer& r, const MapData& map, const FlightData& flight,
                     const MapViewConfig& config, float displayH) {
  if (config.w <= 0.0f || config.h <= 0.0f) return;

  const float cx = config.x + config.w * 0.5f;
  const float cy = config.y + config.h * 0.5f;
  const float mapRadiusPx = mapview::mapRangeSpanPx(config);
  // A per-view range override lets the MFD MAP page zoom independently of the
  // PFD inset while reading the same MapData.
  const float rangeNm =
      std::max(kMapRangeMinNm, config.rangeNm > 0.0f ? config.rangeNm : map.rangeNm);
  // The on-screen scale follows the animated zoom value (when supplied) so the
  // map glides between ladder steps, while `rangeNm` -- the selected step --
  // still drives the readout, range rings, and symbol declutter so those don't
  // flicker through intermediate values during the zoom.
  const float scaleRangeNm =
      std::max(kMapRangeMinNm, config.displayRangeNm > 0.0f ? config.displayRangeNm
                                                  : rangeNm);
  const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
  const float viewHalfExtentNm =
      std::sqrt((config.w * 0.5f) * (config.w * 0.5f) +
                (config.h * 0.5f) * (config.h * 0.5f)) /
      pixelsPerNm;
  const float rotation = orientationDeg(config.orientation, flight);
  const float labelSize = mapview::fontPx(config.style.labelFontWt, displayH);

  const double viewCenterLat =
      config.hasCenterOverride ? config.centerLat : map.ownshipLat;
  const double viewCenterLon =
      config.hasCenterOverride ? config.centerLon : map.ownshipLon;

  const bool useInsetData = config.useInsetMapData && map.insetMapActive;
  const std::vector<MapLandLine>& landLines =
      useInsetData ? map.insetLandLines : map.landLines;
  const std::vector<MapLandCity>& cities =
      useInsetData ? map.insetCities : map.cities;

  r.save();
  r.clip(config.x, config.y, config.w, config.h);

  // When X-Plane DSF tiles are available, sample elevation for the black-land /
  // navy-water chart base instead of GSHHG Mercator chord fills (which leave
  // wedges and meridian seams). The topo TER softkey still switches to hillshade
  // or REL coloring on top of the same DEM path.
  // The DEM mask only pays off where the view spans a handful of DSF tiles;
  // past kChartLandMaxRangeNm the tile count explodes, so fall back to the
  // instant in-memory GSHHG vector coastline at continental scale.
  // Popup inset maps (Direct-To, FPL entry) center on arbitrary targets and rely
  // on the inset GSHHG query instead of DSF tiles prewarmed at ownship.
  const bool useDsfLandMask =
      !useInsetData && config.style.showLand &&
      config.style.terrain == TerrainDisplay::Off &&
      rangeNm <= map::kChartLandMaxRangeNm && map.terrain != nullptr &&
      map.terrain->hasElevationTiles();

  // NXi chart base. Default to black "land" and only paint the navy ocean base
  // once real GSHHG coastline data is available for the view: a map with no
  // coastline data (the GSHHG store still loading, a failed load, or a
  // DSF-elevation-only build) reads as land, not as a screen full of blank
  // ocean. DSF elevation tiles alone do NOT justify the navy base -- at wide
  // range (e.g. 150 NM) only a few tiles around ownship are resident, so the
  // rest of the view samples NaN and a navy base would show through as "blue
  // over land". With a black base the terrain raster instead paints navy only
  // where it has confirmed water (e <= 0) and topo on land, and uncovered
  // no-data pixels stay black. When the DSF land mask is active the base also
  // stays black for the same reason. The Direct-To / FPL popup inset always
  // uses the black land base.
  if (config.style.showLand && map.positionValid &&
      (config.style.showChrome || config.style.showLand)) {
    const bool landDataLoaded = !landLines.empty();
    const Color chartBase =
        (config.useInsetMapData || !landDataLoaded || useDsfLandMask)
            ? mapview::kMapLandFill
            : mapview::kMapOceanFill;
    r.fillRect(config.x, config.y, config.w, config.h, chartBase);
  }

  if (!map.positionValid) {
    if (config.style.showChrome) {
      // Amber "NO GPS POSITION" in a black chrome plate centered on the map
      // (G1000 NXi position-lost annunciation), reusing the boxed map-chrome
      // style used for the orientation / range / wind plates rather than plain
      // gray text.
      const char* kNoGpsText = "NO GPS POSITION";
      const float gpsSize = labelSize * 1.1f;
      const float padX = gpsSize * 0.55f;
      const float boxW = r.measureTextWidth(kNoGpsText, gpsSize) + 2.0f * padX;
      const float boxH = gpsSize * 1.5f;
      mapview::drawChromeBox(r, cx - boxW * 0.5f, cy - boxH * 0.5f, boxW, boxH);
      r.fillText(cx, cy + boxH * 0.02f, kNoGpsText, gpsSize, TextAlign::Center,
                 colors::kBandYellow, mapview::kMapLabelFace);
    }
    r.restore();
    return;
  }

  const float symSize =
      std::max(5.0f, mapview::fontPx(mapview::kFeatureSymbolWt, displayH));
  const float obstacleSize =
      std::max(11.0f, mapview::fontPx(mapview::kObstacleSymbolWt, displayH));

  mapview::Proj proj;
  proj.centerLat = viewCenterLat;
  proj.centerLon = viewCenterLon;
  proj.cx = cx;
  proj.cy = cy;
  proj.pixelsPerNm = pixelsPerNm;
  proj.rotation = rotation;
  proj.minX = config.x;
  proj.minY = config.y;
  proj.maxX = config.x + config.w;
  proj.maxY = config.y + config.h;
  proj.init();

  // The DSF land mask (decided above, before the chart base fill) samples
  // elevation for the black-land / navy-water chart base.
  if (useDsfLandMask) {
    map::drawTerrainRaster(
        r, *map.terrain, map::TerrainRasterMode::ChartLand, 0.0f, viewCenterLat,
        viewCenterLon, cx, cy, pixelsPerNm, rotation, rangeNm, scaleRangeNm,
        viewHalfExtentNm, map::kChartLandMaxRangeNm);
  }

  // Lakes, borders, roads, and labels. GSHHG land-mass chord fills are skipped
  // when the DSF mask is active; transparent raster pixels (tiles not yet
  // streamed in) show the black land base rather than navy ocean.
  if (config.style.showLand && (!landLines.empty() || !cities.empty())) {
    mapview::drawLandData(r, landLines, proj, rangeNm, useDsfLandMask,
                          viewHalfExtentNm, scaleRangeNm,
                          config.style.showLandData);
    // City dots are man-made land data, decluttered at Detail 3; water stays.
    if (config.style.showLabels && config.style.showLandData) {
      mapview::drawCityDots(r, cities, proj, rangeNm, symSize);
    }
  }

  // Terrain background (topo or relative), composited over the chart base.
  bool terrainDrawn = false;
  if (config.style.terrain != TerrainDisplay::Off && map.terrain != nullptr) {
    const map::TerrainRasterMode mode =
        config.style.terrain == TerrainDisplay::Rel
            ? map::TerrainRasterMode::Relative
            : map::TerrainRasterMode::Absolute;
    terrainDrawn = map::drawTerrainRaster(
        r, *map.terrain, mode, flight.altitudeValid ? flight.altitudeFt : 0.0f,
        viewCenterLat, viewCenterLon, cx, cy, pixelsPerNm, rotation, rangeNm,
        scaleRangeNm, viewHalfExtentNm, config.style.terrainMaxRangeNm);
  }

  // Rivers and political/state boundaries overlay on top of the topo raster so
  // the thin hydrography and the country/state lines stay visible whether
  // terrain is on or off (the raster otherwise paints over them), matching the
  // real NXi.
  if (config.style.showLand && !landLines.empty()) {
    mapview::drawRiverData(r, landLines, proj, rangeNm);
    mapview::drawBorderData(r, landLines, proj, rangeNm,
                            config.style.showLandData);
  }

  // Dim fallback when land styling is off and no terrain raster is shown.
  if (!terrainDrawn && !config.style.showLand) {
    if (config.style.showChrome) {
      r.fillRect(config.x, config.y, config.w, config.h,
                 Color{0.0f, 0.0f, 0.0f, 0.82f});
    }
  }

  if (config.style.showChrome) {
    r.strokeLine(config.x, config.y, config.x + config.w, config.y, 2.0f,
                 colors::kTapeTopBorder);
    r.strokeLine(config.x, config.y + config.h, config.x + config.w,
                 config.y + config.h, 2.0f, colors::kTapeTopBorder);
    r.strokeLine(config.x, config.y, config.x, config.y + config.h, 2.0f,
                 colors::kTapeTopBorder);
    r.strokeLine(config.x + config.w, config.y, config.x + config.w,
                 config.y + config.h, 2.0f, colors::kTapeTopBorder);
  }

  // Map precipitation overlay: prefer the datalink NEXRAD source (real ground
  // radar) when available, falling back to the onboard radar source. Drawn
  // here -- after the topo/land base (including lake/water fills) but before
  // the nav symbology -- so the returns sit on top of bodies of water like the
  // real G1000 NXi, rather than being painted over by them.
  const WeatherRadarSource* overlayWx =
      map.nexrad != nullptr ? map.nexrad : map.weather;
  if (config.style.showWeather && rangeNm <= config.style.nexradRangeNm &&
      overlayWx != nullptr && overlayWx->active()) {
    // The overlay is anchored to the aircraft: datalink NEXRAD is geo-referenced
    // and centered on ownship, and the onboard radar sweep emanates from it.
    // Draw it at ownship's *screen* location (not the view center) so it stays
    // fixed to the ground/aircraft when the map is panned away from ownship.
    const double rot = static_cast<double>(rotation) * map::kDegToRad;
    const double cosR = std::cos(rot);
    const double sinR = std::sin(rot);
    const float mercatorPxPerRad =
        pixelsPerNm * static_cast<float>(map::kNmPerEarthRad);
    double eastRad = 0.0;
    double northRad = 0.0;
    map::mercatorOffsetRad(map.ownshipLat, map.ownshipLon, viewCenterLat,
                           viewCenterLon, eastRad, northRad);
    float wx = 0.0f;
    float wy = 0.0f;
    map::mercatorToScreen(eastRad, northRad, cx, cy, mercatorPxPerRad, cosR,
                          sinR, wx, wy);
    map::drawWeatherRaster(r, *overlayWx, wx, wy, pixelsPerNm, rotation);
  }

  if (config.style.showRangeRings) {
    if (config.orientation == MapOrientation::NorthUp) {
      mapview::drawRangeRing(r, cx, cy, mapRadiusPx, colors::kLabelText);
    } else {
      mapview::drawRangeCompass(r, config, flight, cx, cy, mapRadiusPx,
                                rotation, labelSize, colors::kLabelText);
    }
  }

  // Airspace boundaries draw beneath the route and features.
  if (config.style.showAirspace && !map.airspaces.empty()) {
    mapview::drawAirspaces(r, map, proj, rangeNm, flight, config.pointerActive,
                           config.pointerLat, config.pointerLon);
  }

  // Airways draw above airspace but under the route and nav features.
  if (!map.airways.empty()) {
    mapview::drawAirways(r, map, proj, config.style.airways, rangeNm,
                         labelSize);
  }

  // VOR compass rose(s): a cyan ~2.5 NM rose around each VOR station on the map,
  // like the real NXi. Drawn here so the route line and feature symbols overlay
  // it.
  if (config.style.showFeatures) {
    mapview::drawVorRoses(r, map, proj, config, rangeNm, labelSize);
  }

  const bool procPreviewActive =
      config.procedurePreview != nullptr &&
      config.procedurePreview->size() >= 2;

  if (config.style.showFlightPlan && map.flightPlan.size() >= 2 &&
      !procPreviewActive) {
    mapview::drawFlightPlan(r, map, proj, config, flight, symSize);
  }

  if (procPreviewActive) {
    mapview::drawProcedurePreview(r, proj, config, symSize);
  }

  if (config.style.showFlightPlan && map.positionValid && !procPreviewActive) {
    mapview::drawDirectToCourse(r, map, flight, proj, config, symSize);
  }

  // Taxiway/apron pavement at very close range, under the runway quads.
  if (config.style.showTaxiways && config.style.showFeatures &&
      !map.taxiways.empty()) {
    const bool landDataLoaded = !landLines.empty();
    const Color taxiwayHoleFill =
        (config.useInsetMapData || !landDataLoaded || useDsfLandMask)
            ? mapview::kMapLandFill
            : mapview::kMapOceanFill;
    mapview::drawTaxiways(r, map, proj, rangeNm, taxiwayHoleFill);
  }

  // Runway pavement quads at close range, under the airport symbols/labels.
  if (config.style.showRunways && config.style.showFeatures &&
      !map.runways.empty()) {
    mapview::drawRunways(r, map, proj, rangeNm, labelSize);
  }

  // SafeTaxi taxiway identifier labels, on top of the pavement.
  if (config.style.showTaxiways && config.style.showFeatures &&
      config.style.showLabels && !map.taxiwayLabels.empty()) {
    mapview::drawTaxiwayLabels(r, map, proj, rangeNm, labelSize);
  }

  if (config.style.showFeatures) {
    mapview::drawNavFeatures(r, map, proj, config, rangeNm, symSize);
  }

  // Direct-To / WPT inset: the pinned target is always drawn (and labeled)
  // regardless of airport size-class range declutter (far-away targets zoom out
  // past 100 NM). It draws at its true projected position; for the WPT/NRST
  // insets that position is the view center, while the Direct-To leg frames the
  // midpoint so the target sits off-center toward the ownship.
  if (useInsetData && config.centerFeature != nullptr &&
      config.style.showFeatures) {
    const MapFeature& f = *config.centerFeature;
    float fx = cx, fy = cy;
    proj.toPx(f.lat, f.lon, fx, fy);
    drawMapFeatureSymbol(r, f, fx, fy, symSize);
    if (config.style.showLabels && !f.id.empty()) {
      const float textSize = labelSize * mapview::kMapIdentLabelScale;
      const float labelY =
          (f.type == MapFeatureType::Airport ? fy - symSize * 1.25f
                                             : fy - symSize * 1.12f) -
          mapview::kMapLabelLiftPx;
      r.fillText(fx, labelY, f.id, textSize, TextAlign::Center, colors::kWhite,
                 mapview::kMapLabelFace);
    }
  }

  // Boxed flight-plan idents draw above nav symbology (fixes share the same
  // coordinates as plan legs; the route overlay replaces their map icons).
  if (config.style.showFlightPlan && map.flightPlan.size() >= 2 &&
      !procPreviewActive) {
    mapview::drawFlightPlanLabels(r, map, proj, config, flight, symSize,
                                  labelSize);
  }
  // VNAV top/bottom-of-descent markers, on the active flight-plan course.
  if (config.style.showFlightPlan && !procPreviewActive) {
    mapview::drawTopOfDescent(r, proj, config, flight, symSize, labelSize);
    mapview::drawBottomOfDescent(r, proj, config, flight, symSize, labelSize);
  }
  if (config.style.showFlightPlan && map.positionValid && !procPreviewActive) {
    mapview::drawDirectToCourseLabel(r, map, flight, proj, config, symSize,
                                     labelSize);
  }
  // Previewed approach fixes get the same boxed idents as loaded plan legs.
  mapview::drawProcedurePreviewLabels(r, proj, config, symSize, labelSize);

  // Obstacles (FAA DOF): independent of nav-feature declutter (Table 5-4).
  if (config.style.showObstacles && !map.obstacles.empty()) {
    mapview::drawObstacles(r, map, proj, rangeNm,
                           config.style.obstacleRangeNm,
                           flight.altitudeValid ? flight.altitudeFt : 0.0f,
                           flight.altitudeValid, obstacleSize);
  }

  // Place and nav idents draw above symbology (white labels centered on top).
  if (config.style.showLabels) {
    if (config.style.showLand && !cities.empty()) {
      mapview::drawMapPlaceLabels(r, cities, proj, rangeNm, labelSize,
                                  config.style.showLandData);
    }
    if (config.style.showFeatures) {
      mapview::drawNavFeatureLabels(r, map, proj, config, rangeNm, symSize,
                                    labelSize);
    }
    // The procedure preview does not draw its own fix labels: the base map
    // already labels every fix in white, so cyan preview labels would just
    // duplicate them (and the real unit shows no extra labels here).
  }

  // Ownship sits at the view center unless the center is overridden (the WPT/
  // NRST airport maps center on the airport), in which case project it.
  float ownX = cx, ownY = cy;
  if (config.hasCenterOverride) {
    proj.toPx(map.ownshipLat, map.ownshipLon, ownX, ownY);
  }

  if (config.style.showFuelRing) {
    mapview::drawFuelRing(r, flight, ownX, ownY, pixelsPerNm, mapRadiusPx);
  }
  if (config.style.showTrackVector) {
    mapview::drawTrackVector(r, flight, ownX, ownY, pixelsPerNm, rotation);
  }

  // Traffic overlays everything except ownship and the chrome.
  if (config.style.showTraffic && rangeNm <= config.style.trafficSymbolsRangeNm &&
      !map.traffic.empty()) {
    const bool showLabels = config.style.showTrafficLabels &&
                            rangeNm <= config.style.trafficLabelsRangeNm;
    mapview::drawTraffic(r, map, proj, symSize, labelSize, showLabels);
  }

  const float ownSize =
      std::max(8.0f, mapview::fontPx(mapview::kOwnshipSymbolWt, displayH));
  mapview::drawOwnshipSymbol(r, ownX, ownY, ownSize, flight.headingDeg - rotation);

  if (config.style.showWindVector) {
    mapview::drawWindVector(r, flight, config, rotation, labelSize);
  }

  if (config.style.terrain == TerrainDisplay::Rel && terrainDrawn) {
    mapview::drawRelTerrainLegend(r, config, labelSize);
  }

  if (config.style.showChrome || config.style.showOrientationLabel) {
    // Boxed orientation annunciation in the top-left corner with the north
    // indicator below it (Fig 5-2 / Fig 5-26).
    const float ox = config.x + labelSize * 0.45f;
    const float oy = config.y + labelSize * 0.35f;
    const float boxH = mapview::drawChromeLabel(
        r, ox, oy, orientationLabel(config.orientation), labelSize,
        colors::kCyan);
    if (config.style.showNorthArrow) {
      mapview::drawNorthArrow(r, ox + labelSize * 1.2f,
                              oy + boxH + labelSize * 1.6f, labelSize * 1.2f,
                              rotation);
    }

    // Boxed range readout: on the outer range ring when rings are shown,
    // otherwise tucked into the lower-right corner (PFD inset).
    if (config.style.showRangeRings) {
      mapview::drawRangeLabel(r, cx - mapRadiusPx * 0.7071f,
                              cy - mapRadiusPx * 0.7071f, rangeNm, labelSize,
                              true);
    } else {
      mapview::drawRangeLabel(r, config.x + config.w - labelSize * 0.4f,
                              config.y + config.h - labelSize * 0.35f, rangeNm,
                              labelSize, false);
    }
  }

  r.restore();
}

}  // namespace avionics
