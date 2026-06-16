#include "avionics/render/MapView.h"

#include <algorithm>
#include <cmath>

#include "avionics/Color.h"
#include "avionics/WeatherRadar.h"
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
  const float mapRadiusPx = 0.45f * std::min(config.w, config.h);
  // A per-view range override lets the MFD MAP page zoom independently of the
  // PFD inset while reading the same MapData.
  const float rangeNm =
      std::max(0.5f, config.rangeNm > 0.0f ? config.rangeNm : map.rangeNm);
  // The on-screen scale follows the animated zoom value (when supplied) so the
  // map glides between ladder steps, while `rangeNm` -- the selected step --
  // still drives the readout, range rings, and symbol declutter so those don't
  // flicker through intermediate values during the zoom.
  const float scaleRangeNm =
      std::max(0.5f, config.displayRangeNm > 0.0f ? config.displayRangeNm
                                                  : rangeNm);
  const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
  const float rotation = orientationDeg(config.orientation, flight);
  const float labelSize = mapview::fontPx(config.style.labelFontWt, displayH);

  const double viewCenterLat =
      config.hasCenterOverride ? config.centerLat : map.ownshipLat;
  const double viewCenterLon =
      config.hasCenterOverride ? config.centerLon : map.ownshipLon;

  r.save();
  r.clip(config.x, config.y, config.w, config.h);

  // Terrain background (topo or relative), drawn first so everything else
  // overlays it. Falls back to the plain background when no terrain source is
  // available or the raster has not finished its first build yet.
  bool terrainDrawn = false;
  if (config.style.terrain != TerrainDisplay::Off && map.terrain != nullptr &&
      map.positionValid) {
    const map::TerrainRasterMode mode =
        config.style.terrain == TerrainDisplay::Rel
            ? map::TerrainRasterMode::Relative
            : map::TerrainRasterMode::Absolute;
    terrainDrawn = map::drawTerrainRaster(
        r, *map.terrain, mode, flight.altitudeValid ? flight.altitudeFt : 0.0f,
        viewCenterLat, viewCenterLon, cx, cy, pixelsPerNm, rotation, rangeNm);
  }

  // Plain background fill when no terrain raster is shown. Drawn here -- before
  // the weather overlay -- so it sits behind the NEXRAD returns rather than
  // dimming them.
  if (config.style.showChrome && !terrainDrawn) {
    r.fillRect(config.x, config.y, config.w, config.h,
               Color{0.0f, 0.0f, 0.0f, 0.82f});
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

  if (!map.positionValid) {
    if (config.style.showChrome) {
      r.fillText(cx, cy, "NO GPS POSITION", labelSize, TextAlign::Center,
                 colors::kLabelText);
    }
    r.restore();
    return;
  }

  const float symSize =
      std::max(5.0f, mapview::fontPx(mapview::kFeatureSymbolWt, displayH));

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

  // Land data sits directly on the background (under everything, including
  // the range rings); lakes/rivers are skipped over the terrain raster's
  // water-colored TOPO base only when terrain is off-screen -- they overlay
  // consistently either way.
  if (config.style.showLand &&
      (!map.landLines.empty() || !map.cities.empty())) {
    mapview::drawLandData(r, map, proj, rangeNm);
    if (config.style.showLabels) {
      mapview::drawCities(r, map, proj, rangeNm, symSize, labelSize);
    }
  }

  // Map precipitation overlay: prefer the datalink NEXRAD source (real ground
  // radar) when available, falling back to the onboard radar source. Drawn
  // here -- after the topo/land base (including lake/water fills) but before
  // the nav symbology -- so the returns sit on top of bodies of water like the
  // real G1000 NXi, rather than being painted over by them.
  const WeatherRadarSource* overlayWx =
      map.nexrad != nullptr ? map.nexrad : map.weather;
  if (config.style.showWeather && overlayWx != nullptr && overlayWx->active()) {
    // The overlay is anchored to the aircraft: datalink NEXRAD is geo-referenced
    // and centered on ownship, and the onboard radar sweep emanates from it.
    // Draw it at ownship's *screen* location (not the view center) so it stays
    // fixed to the ground/aircraft when the map is panned away from ownship.
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double rot = static_cast<double>(rotation) * kDegToRad;
    const double cosR = std::cos(rot);
    const double sinR = std::sin(rot);
    const double northNm = (map.ownshipLat - viewCenterLat) * map::kNmPerDegLat;
    const double eastNm =
        (map.ownshipLon - viewCenterLon) * map::nmPerDegLon(viewCenterLat);
    const double mapEast = eastNm * cosR - northNm * sinR;
    const double mapNorth = eastNm * sinR + northNm * cosR;
    const float wx =
        cx + static_cast<float>(mapEast * static_cast<double>(pixelsPerNm));
    const float wy =
        cy - static_cast<float>(mapNorth * static_cast<double>(pixelsPerNm));
    map::drawWeatherRaster(r, *overlayWx, wx, wy, pixelsPerNm, rotation);
  }

  if (config.style.showRangeRings) {
    mapview::drawRangeRing(r, cx, cy, mapRadiusPx, colors::kLabelText);
    mapview::drawRangeRing(r, cx, cy, mapRadiusPx * 0.5f, colors::kLabelText);
  }

  // Airspace boundaries draw beneath the route and features.
  if (config.style.showAirspace && !map.airspaces.empty()) {
    mapview::drawAirspaces(r, map, proj, rangeNm, flight);
  }

  // Airways draw above airspace but under the route and nav features.
  if (!map.airways.empty()) {
    mapview::drawAirways(r, map, proj, config.style.airways, rangeNm,
                         labelSize);
  }

  if (config.style.showFlightPlan && map.flightPlan.size() >= 2) {
    mapview::drawFlightPlan(r, map, proj, config, flight, symSize, labelSize);
  }

  if (config.procedurePreview != nullptr &&
      config.procedurePreview->size() >= 2) {
    mapview::drawProcedurePreview(r, proj, config, symSize, labelSize);
  }

  if (config.style.showFlightPlan && map.directToActive && map.positionValid) {
    mapview::drawDirectToCourse(r, map, proj, config, symSize, labelSize);
  }

  // Taxiway/apron pavement at very close range, under the runway quads.
  if (config.style.showTaxiways && config.style.showFeatures &&
      !map.taxiways.empty()) {
    mapview::drawTaxiways(r, map, proj, rangeNm);
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
    mapview::drawNavFeatures(r, map, proj, config, rangeNm, symSize, labelSize);
  }

  // Obstacles draw with the nav features (same declutter switch).
  if (config.style.showObstacles && config.style.showFeatures &&
      !map.obstacles.empty()) {
    mapview::drawObstacles(r, map, proj, rangeNm,
                           flight.altitudeValid ? flight.altitudeFt : 0.0f,
                           flight.altitudeValid, symSize);
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
  if (config.style.showTraffic && !map.traffic.empty()) {
    mapview::drawTraffic(r, map, proj, symSize, labelSize);
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
