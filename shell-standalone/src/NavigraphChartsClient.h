#pragma once

#include <string>
#include <vector>

// Navigraph Charts API v2 client
// (https://developers.navigraph.com/docs/charts/airport-charts).
//
// Two blocking HTTPS calls, both run off the render thread (see NavigraphStore)
// and authorized with the Bearer access token from the OAuth flow:
//   1. NavigraphFetchChartIndex -> the airport's chart index (metadata + image
//      URLs) from GET /v2/charts/{ICAO}.
//   2. NavigraphFetchChartImage -> the high-resolution chart PNG bytes from the
//      index entry's image_day_url / image_night_url.
//
// Charts are gated to a signed-in account (the API rejects anonymous calls) and
// to a Navigraph Ultimate subscription for non-demo airports; without one the
// API serves the two demo airports (NZWN / YBBN). Per Navigraph's terms the
// bytes must not be cached to disk -- callers keep them in memory only.

#include "avionics/Charts.h"

namespace avionics {

// One chart's metadata from the airport chart index.
struct NavigraphChartMeta {
  std::string id;            // unique chart id (e.g. "NZWN102")
  std::string indexNumber;   // per-airport index (e.g. "10-2")
  std::string name;          // human-readable name
  std::string category;      // raw API category: ARR / DEP / REF / APT / APP
  std::string imageDayUrl;   // absolute URL of the day PNG
  std::string imageNightUrl; // absolute URL of the night PNG
  ChartGeoref georef;
};

struct ChartIndexResult {
  bool ok = false;
  std::string error;  // human-readable summary when !ok
  std::string icao;
  std::vector<NavigraphChartMeta> charts;
};

struct ChartImageResult {
  bool ok = false;
  std::string error;
  std::string chartId;  // echoes the requested chart id
  bool night = false;   // echoes the requested variant
  std::vector<unsigned char> pngBytes;
};

// Fetch the chart index for an ICAO airport. Blocking.
ChartIndexResult NavigraphFetchChartIndex(const std::string& accessToken,
                                          const std::string& icao);

// Download a chart image by its absolute URL (from the index entry). `chartId`
// and `night` are echoed back so the caller can match the result to the
// selection that requested it. Blocking.
ChartImageResult NavigraphFetchChartImage(const std::string& accessToken,
                                          const std::string& chartId, bool night,
                                          const std::string& imageUrl);

// Maps Navigraph index metadata into the avionics-core chart list item.
ChartListItem NavigraphChartToListItem(const NavigraphChartMeta& meta);

}  // namespace avionics
