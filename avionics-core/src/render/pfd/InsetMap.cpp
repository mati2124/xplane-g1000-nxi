#include "render/pfd/PfdInternal.h"

#include "avionics/render/MapView.h"

namespace avionics::pfd {

void drawInsetMap(Renderer& r, const Layout& L, const MapData& map,
                  const FlightData& flight, const SoftkeyController& ui,
                  float displayH) {
  if (!ui.insetMapVisible()) return;

  MapViewConfig config;
  config.x = L.insetMapX;
  config.y = L.insetMapY;
  config.w = L.insetMapW;
  config.h = L.insetMapH;
  config.orientation = MapOrientation::TrackUp;
  config.rangeNm = ui.insetRangeNm();  // driven by the bezel range rocker
  config.style.showChrome = true;
  config.style.labelFontWt = wt::kHsiSource;

  MapView::render(r, map, flight, config, displayH);
}

}  // namespace avionics::pfd
