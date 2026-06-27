#pragma once

#include <memory>
#include <string>
#include <vector>

#include "avionics/SimBrief.h"  // NavigraphLoginPhase

namespace avionics {

// Navigraph airport-chart category, mirroring the v2 Charts API `category`
// field (ARR, DEP, REF, APT, APP). Drives the list grouping/coloring on the
// AUX - Charts page; unknown values map to Other.
enum class ChartCategory {
  Departure,   // DEP (SID)
  Arrival,     // ARR (STAR)
  Approach,    // APP
  Airport,     // APT (taxi / airport diagram)
  Reference,   // REF
  Other,
};

// Pixel rectangle on a chart PNG (Navigraph origin: bottom-left, +y up).
struct ChartPixelRect {
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};

// Geo-referencing metadata from the Navigraph Charts API (planview + insets).
struct ChartGeoref {
  bool isGeoreferenced = false;
  int imageWidth = 0;
  int imageHeight = 0;
  ChartPixelRect planviewPixels;
  double planviewLat1 = 0.0;
  double planviewLng1 = 0.0;
  double planviewLat2 = 0.0;
  double planviewLng2 = 0.0;
  std::vector<ChartPixelRect> insetPixels;
};

// One chart's metadata as shown in the page's selectable list. The downloadable
// image URL stays in the shell (it carries the access token); the page only
// needs enough to label and select a chart.
struct ChartListItem {
  std::string id;           // unique chart id (e.g. "NZWN102")
  std::string indexNumber;  // per-airport index (e.g. "10-2")
  std::string name;         // human-readable name
  ChartCategory category = ChartCategory::Other;
  ChartGeoref georef;
};

// Lifecycle of the chart index fetch for the selected airport.
enum class ChartsStatus {
  Idle,     // signed out / nothing requested yet
  Loading,  // chart index (or the selected image) in flight
  Ready,    // index loaded; charts available
  Empty,    // index loaded but the airport has no charts
  Error,    // index fetch failed (see ChartsState::error)
};

// The currently displayed chart image. Per Navigraph's terms charts must not be
// cached or stored, so the bytes live only in memory for the active selection.
// The shell worker downloads the PNG; the page decodes + uploads it lazily,
// re-uploading only when `generation` changes (0 = nothing to show yet).
struct ChartImage {
  unsigned generation = 0;
  std::shared_ptr<const std::vector<unsigned char>> pngBytes;
};

// Snapshot of the Navigraph charts integration shown on the AUX - Charts page.
// The shell publishes it into the MfdController each frame (the shell owns the
// HTTPS client + token); the page draws it. Charts are only fetched while
// signed in and the sim link is up (commAllowed), per Navigraph's terms.
struct ChartsState {
  // Navigraph data is only permitted with a connected sim session (the same
  // gate the SimBrief page uses); when false the page prompts to connect.
  bool commAllowed = true;
  NavigraphLoginPhase loginPhase = NavigraphLoginPhase::LoggedOut;

  // The airport whose chart index is currently published (origin or destination
  // of the active flight plan, per the page's Origin/Dest softkeys).
  std::string airportIcao;
  ChartsStatus status = ChartsStatus::Idle;
  std::string error;  // human-readable failure summary when status == Error
  std::vector<ChartListItem> charts;
  ChartImage image;  // the selected chart's image (matches the page selection)
};

}  // namespace avionics
