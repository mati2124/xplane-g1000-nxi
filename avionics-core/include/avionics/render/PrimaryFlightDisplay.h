#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"

namespace avionics {

// Renderer for the PFD page. The drawing itself is stateless; all persistent
// state is passed in -- flight data (smoothed in the DataSource) and the
// interactive softkey/window state owned by the AvionicsEngine.
class PrimaryFlightDisplay {
 public:
  static void render(Renderer& r, const FlightData& data, const MapData& map,
                     const SoftkeyController& ui, int widthPx, int heightPx);
};

}  // namespace avionics
