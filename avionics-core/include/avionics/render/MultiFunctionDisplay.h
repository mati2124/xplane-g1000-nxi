#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"

namespace avionics {

// Renderer for the MFD page (a second GDU, typically its own window). Like the
// PFD it is stateless: the page group and map range live in the MfdController
// owned by the AvionicsEngine, and the nav data comes from the same
// DataSource::mapSnapshot() that feeds the PFD inset map.
//
// For now the MAP page renders a full-screen moving map (reusing MapView);
// the WPT/AUX/NRST page groups draw a titled placeholder.
class MultiFunctionDisplay {
 public:
  static void render(Renderer& r, const FlightData& data, const MapData& map,
                     const MfdController& ui, int widthPx, int heightPx);
};

}  // namespace avionics
