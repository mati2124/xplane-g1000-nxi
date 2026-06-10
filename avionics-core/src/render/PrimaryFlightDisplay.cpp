#include "avionics/render/PrimaryFlightDisplay.h"

#include "render/pfd/PfdInternal.h"

namespace avionics {

void PrimaryFlightDisplay::render(Renderer& r, const FlightData& d,
                                  const MapData& map,
                                  const SoftkeyController& ui, int widthPx,
                                  int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const pfd::Layout L = pfd::computeLayout(w, h);

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  pfd::drawAttitude(r, L, d, w, h);
  pfd::drawInsetMap(r, L, map, d, ui, h);
  pfd::drawAirspeedTape(r, L, d, ui, h);
  pfd::drawAltimeter(r, L, d, ui, h);
  pfd::drawVerticalSpeedIndicator(r, L, d, h);
  pfd::drawVerticalDeviation(r, L, d, h);
  pfd::drawHsiSection(r, L, d, ui, h);
  pfd::drawChrome(r, L, d, ui, w, h);
}

}  // namespace avionics
