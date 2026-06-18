#pragma once

#include "avionics/Checklist.h"
#include "avionics/Eis.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"

namespace avionics {

// Renderer for the MFD page (a second GDU, typically its own window). Like the
// PFD it is stateless: the page group and map range live in the MfdController
// owned by the AvionicsEngine, and the nav data comes from the same
// DataSource::mapSnapshot() that feeds the PFD inset map.
//
// MAP renders a full-screen moving map (reusing MapView); WPT shows the active
// leg and flight plan; AUX shows system status; NRST lists nearest navaids.
class MultiFunctionDisplay {
 public:
  static void render(Renderer& r, const FlightData& data, const MapData& map,
                     const ChecklistData& checklist, const EisLayout& eisLayout,
                     MfdController& ui, const SoftkeyController& radios,
                     int widthPx, int heightPx);
};

}  // namespace avionics
