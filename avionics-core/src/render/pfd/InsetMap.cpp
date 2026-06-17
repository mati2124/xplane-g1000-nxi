#include "render/pfd/PfdInternal.h"

#include "avionics/render/MapView.h"

namespace avionics::pfd {

void drawInsetMap(Renderer& r, const Layout& L, const MapData& map,
                  const FlightData& flight, const SoftkeyController& ui,
                  float displayH, bool forceVisible) {
  if (!forceVisible && !ui.insetMapVisible()) return;

  MapViewConfig config;
  config.x = L.insetMapX;
  config.y = L.insetMapY;
  config.w = L.insetMapW;
  config.h = L.insetMapH;
  config.orientation = MapOrientation::TrackUp;
  config.rangeNm = ui.insetRangeNm();  // driven by the bezel range rocker
  config.displayRangeNm = ui.insetDisplayRangeNm();  // smooth zoom animation
  config.style.showChrome = true;
  config.style.labelFontWt = wt::kHsiSource;

  // Map/HSI softkey overlays: Rel Ter wins over Topo when both are selected,
  // and the Detail key declutters the inset like the full map.
  config.style.terrain =
      ui.displayToggle(DisplayToggle::MapRelTer) ? TerrainDisplay::Rel
      : ui.displayToggle(DisplayToggle::MapTopo) ? TerrainDisplay::Topo
                                                 : TerrainDisplay::Off;
  config.style.showTraffic = ui.displayToggle(DisplayToggle::Traffic);
  applyMapDetail(config.style, ui.mapDetail());

  MapView::render(r, map, flight, config, displayH);
}

void drawHsiMap(Renderer& r, const Layout& L, const MapData& map,
                const FlightData& flight, const SoftkeyController& ui,
                float displayH) {
  if (!ui.hsiMapVisible()) return;

  // The HSI Map fills a square region centered on the compass rose; the rose is
  // drawn over it afterward with a translucent backing (G1000 NXi HSI Map). The
  // map uses the lower/larger HSI-map rose geometry.
  const float half = L.hsiMapRadius * 1.20f;
  MapViewConfig config;
  config.x = L.hsiMapCx - half;
  config.y = L.hsiMapCy - half;
  config.w = 2.0f * half;
  config.h = 2.0f * half;
  config.orientation = MapOrientation::TrackUp;
  config.rangeNm = ui.insetRangeNm();
  config.displayRangeNm = ui.insetDisplayRangeNm();  // smooth zoom animation
  config.style.showChrome = false;  // the rose supplies the heading reference
  config.style.showRangeRings = false;  // the rose replaces the range compass
  config.style.labelFontWt = wt::kHsiSource;

  config.style.terrain =
      ui.displayToggle(DisplayToggle::MapRelTer) ? TerrainDisplay::Rel
      : ui.displayToggle(DisplayToggle::MapTopo) ? TerrainDisplay::Topo
                                                 : TerrainDisplay::Off;
  config.style.showTraffic = ui.displayToggle(DisplayToggle::Traffic);
  applyMapDetail(config.style, ui.mapDetail());

  r.save();
  r.clip(config.x, config.y, config.w, config.h);
  // The HSI Map paints its own opaque black backdrop (like the WPT/NRST page
  // maps in drawPageMap) so that with terrain off the map reads on black rather
  // than letting the synthetic-vision ground show through -- on the real G1000
  // NXi the HSI Map area is black when TOPO/Rel Ter are off, even with SVT
  // terrain drawn around the rose. When terrain is on, the raster overlays this.
  r.fillRect(config.x, config.y, config.w, config.h, colors::kBlack);
  MapView::render(r, map, flight, config, displayH);
  // NanoVG only scissors to rectangles, so the map renders into the square
  // above. Restore everything outside the compass rose to the background the
  // map painted over, so the moving map only shows within the circular confines
  // of the HSI and the synthetic-vision ground shows around it (G1000 NXi). The
  // whole rose sits below the horizon, where the attitude background is a flat
  // fill of kGroundHorizon, so masking that color blends seamlessly with the
  // surrounding ground. The rose backing, ticks, and pointers are drawn over
  // this afterward in drawHsiSection. outerR reaches past the square's corners
  // (corner distance = half*sqrt2 ~= 1.70*radius) and the clip keeps the mask
  // inside the square so it never bleeds onto the attitude window above.
  drawArcBand(r, L.hsiMapCx, L.hsiMapCy, L.hsiMapRadius, L.hsiMapRadius * 1.8f,
              0.0f, 360.0f, colors::kGroundHorizon);
  r.restore();
}

}  // namespace avionics::pfd
