#pragma once

#include "avionics/Eis.h"
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
  // `eisLayout` + `reversionary` drive G1000 display-backup mode: when the MFD
  // is unpowered/failed, the PFD adds the EIS engine strip down its left edge
  // and relocates the inset map to the bottom-right, mirroring the real unit
  // (Pilot's Guide Fig. 1-5). In normal operation `reversionary` is false and
  // the EIS layout is ignored.
  // `powerUp` suppresses readouts that are not shown while the system is still
  // initializing (the selected-heading, desired-track, and magnetic-heading
  // boxes around the HSI); used by the PFD power-up annunciation screen.
  static void render(Renderer& r, const FlightData& data, const MapData& map,
                     const SoftkeyController& ui, const EisLayout& eisLayout,
                     bool reversionary, int widthPx, int heightPx,
                     bool powerUp = false);
};

}  // namespace avionics
