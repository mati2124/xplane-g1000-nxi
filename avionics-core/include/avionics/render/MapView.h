#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/Renderer.h"

namespace avionics {

// How the map is rotated inside its viewport. The PFD inset and MFD MAP page
// share the same renderer; each caller picks orientation via MapViewConfig.
enum class MapOrientation { NorthUp, HeadingUp, TrackUp };

struct MapViewStyle {
  bool showRangeRings = true;
  bool showFlightPlan = true;
  bool showFeatures = true;
  bool showAirspace = true;
  // Topographic terrain background (requires MapData::terrain). When off, the
  // map keeps its plain dark background.
  bool showTerrain = false;
  // Border, range label, and orientation annunciation. The inset map shows
  // these; a full-screen MFD page may supply its own chrome instead.
  bool showChrome = true;
  float labelFontWt = 14.0f;
};

// Rectangle and display options for a single map instance. Any screen region
// (PFD inset box, MFD full map, traffic overlay) passes the same struct.
struct MapViewConfig {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
  MapOrientation orientation = MapOrientation::TrackUp;
  // Per-view range, in NM. When > 0 it overrides MapData::rangeNm so several
  // map instances (PFD inset vs. MFD MAP page) can show the same nav data at
  // independent zooms. Defaults to 0 = follow the shared MapData range.
  float rangeNm = 0.0f;
  MapViewStyle style;
};

// Shared moving-map renderer. PFD inset and future MFD MAP page both call
// MapView::render with different viewport rects and styles.
class MapView {
 public:
  static void render(Renderer& r, const MapData& map, const FlightData& flight,
                     const MapViewConfig& config, float displayH);
};

}  // namespace avionics
