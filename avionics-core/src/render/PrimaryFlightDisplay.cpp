#include "avionics/render/PrimaryFlightDisplay.h"

#include "render/mfd/EisStrip.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/PfdInternal.h"

namespace avionics {

void PrimaryFlightDisplay::render(Renderer& r, const FlightData& d,
                                  const MapData& map,
                                  const SoftkeyController& ui,
                                  const EisLayout& eisLayout, bool reversionary,
                                  int widthPx, int heightPx, bool powerUp) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);

  // Display-backup (reversionary) mode mirrors the real unit (Pilot's Guide
  // Fig. 3-2): the EIS engine strip takes the full-height left edge and the PFD
  // symbology is compressed into the space to its right (the airspeed tape
  // butts against the strip with no overlap). We achieve that by laying the PFD
  // out in the reduced width (w - eisW) and translating every horizontal
  // position right by eisW; the top/softkey bars stay full width.
  const float eisW =
      reversionary ? w * mfd::eisStripWidthFrac(eisLayout.style) : 0.0f;
  pfd::Layout L = pfd::computeLayout(w - eisW, h);
  if (reversionary) {
    for (float* px : {&L.asiX, &L.altX, &L.vsiX, &L.attCx, &L.hsiCx,
                      &L.hsiMapCx, &L.vdiX, &L.markerX, &L.casAnnunLeft}) {
      *px += eisW;
    }
    // Inset map to the bottom-right (its normal bottom-left home is now the
    // engine column).
    L.insetMapX = w - L.insetMapW - L.insetMapX;
  }

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  pfd::drawAttitude(r, L, d, w, h, powerUp);
  if (reversionary) {
    // Drawn over the attitude background's left edge, like the engine column on
    // the MFD; opaque, so it covers the SVT/horizon fill beneath it. The strip
    // is reduced to its primary parameters: the dense turbofan synoptic grid
    // does not fit beside the flight instruments (the piston stack already
    // does, and ignores the flag).
    mfd::drawEisStrip(r, d, eisLayout,
                      mfd::Rect{0.0f, L.topBarH, eisW, L.infoPanelTop - L.topBarH},
                      h, /*reduced=*/true);
  }
  pfd::drawInsetMap(r, L, map, d, ui, h, /*forceVisible=*/reversionary);
  // HSI Map layout draws the moving map behind the compass rose.
  pfd::drawHsiMap(r, L, map, d, ui, h);
  pfd::drawAirspeedTape(r, L, d, ui, h);
  pfd::drawAltimeter(r, L, d, ui, h);
  pfd::drawVerticalSpeedIndicator(r, L, d, h);
  pfd::drawVerticalDeviation(r, L, d, h);
  pfd::drawHsiSection(r, L, d, ui, h, ui.hsiMapVisible(), powerUp);
  pfd::drawChrome(r, L, d, ui, w, h, powerUp);
}

}  // namespace avionics
