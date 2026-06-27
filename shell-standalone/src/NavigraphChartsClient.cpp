#include "NavigraphChartsClient.h"

#include <curl/curl.h>

#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>

#include "NavigraphClient.h"  // EnsureCurlGlobalInit

namespace avionics {
namespace {

using nlohmann::json;

constexpr const char* kChartsBaseUrl = "https://api.navigraph.com/v2/charts/";
constexpr long kHttpTimeoutSeconds = 30;

std::size_t WriteToString(char* data, std::size_t size, std::size_t nmemb,
                          void* userData) {
  static_cast<std::string*>(userData)->append(data, size * nmemb);
  return size * nmemb;
}

std::size_t WriteToBytes(char* data, std::size_t size, std::size_t nmemb,
                           void* userData) {
  auto* out = static_cast<std::vector<unsigned char>*>(userData);
  const auto* bytes = reinterpret_cast<const unsigned char*>(data);
  out->insert(out->end(), bytes, bytes + size * nmemb);
  return size * nmemb;
}

// GETs `url` with a Bearer token, writing the body via `writeFn` into `userData`.
// Returns false only on transport failure; `httpStatus` carries the HTTP code.
bool HttpGetBearer(const std::string& url, const std::string& accessToken,
                   curl_write_callback writeFn, void* userData,
                   long& httpStatus) {
  EnsureCurlGlobalInit();
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;

  curl_slist* headers = nullptr;
  const std::string authHeader = "Authorization: Bearer " + accessToken;
  headers = curl_slist_append(headers, authHeader.c_str());

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFn);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, userData);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, kHttpTimeoutSeconds);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "XPlaneAvionics");

  const CURLcode rc = curl_easy_perform(curl);
  httpStatus = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpStatus);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK;
}

ChartPixelRect ParsePixelRect(const json& j) {
  ChartPixelRect rect;
  if (!j.is_object()) return rect;
  rect.x1 = j.value("x1", 0.0f);
  rect.y1 = j.value("y1", 0.0f);
  rect.x2 = j.value("x2", 0.0f);
  rect.y2 = j.value("y2", 0.0f);
  return rect;
}

ChartGeoref ParseGeoref(const json& j) {
  ChartGeoref georef;
  georef.isGeoreferenced = j.value("is_georeferenced", false);
  georef.imageWidth = j.value("width", 0);
  georef.imageHeight = j.value("height", 0);
  if (!georef.isGeoreferenced) return georef;

  const json* boxes = nullptr;
  if (j.is_object()) {
    const auto it = j.find("bounding_boxes");
    if (it != j.end() && it->is_object()) boxes = &(*it);
  }
  if (boxes == nullptr) return georef;

  const auto planIt = boxes->find("planview");
  if (planIt == boxes->end() || !planIt->is_object()) return georef;

  const auto pixelsIt = planIt->find("pixels");
  if (pixelsIt != planIt->end()) {
    georef.planviewPixels = ParsePixelRect(*pixelsIt);
  }
  const auto latlngIt = planIt->find("latlng");
  if (latlngIt != planIt->end() && latlngIt->is_object()) {
    georef.planviewLat1 = latlngIt->value("lat1", 0.0);
    georef.planviewLng1 = latlngIt->value("lng1", 0.0);
    georef.planviewLat2 = latlngIt->value("lat2", 0.0);
    georef.planviewLng2 = latlngIt->value("lng2", 0.0);
  }

  const auto insetsIt = boxes->find("insets");
  if (insetsIt != boxes->end() && insetsIt->is_array()) {
    for (const json& inset : *insetsIt) {
      if (!inset.is_object()) continue;
      const auto insetPixelsIt = inset.find("pixels");
      if (insetPixelsIt == inset.end()) continue;
      georef.insetPixels.push_back(ParsePixelRect(*insetPixelsIt));
    }
  }
  return georef;
}

// Maps the v2 charts API `category` code to a known group; falls back to the
// raw string so the page can still display it.
NavigraphChartMeta ParseChart(const json& j, const std::string& icao) {
  NavigraphChartMeta meta;
  meta.id = j.value("id", "");
  meta.indexNumber = j.value("index_number", "");
  meta.name = j.value("name", "");
  meta.category = j.value("category", "");
  meta.imageDayUrl = j.value("image_day_url", "");
  meta.imageNightUrl = j.value("image_night_url", "");
  // Fall back to building the image URL from the file name when the absolute
  // URL field is absent (older API responses expose only image_day/_night).
  if (meta.imageDayUrl.empty()) {
    const std::string file = j.value("image_day", "");
    if (!file.empty()) meta.imageDayUrl = kChartsBaseUrl + icao + "/" + file;
  }
  if (meta.imageNightUrl.empty()) {
    const std::string file = j.value("image_night", "");
    if (!file.empty()) meta.imageNightUrl = kChartsBaseUrl + icao + "/" + file;
  }
  meta.georef = ParseGeoref(j);
  return meta;
}

}  // namespace

ChartListItem NavigraphChartToListItem(const NavigraphChartMeta& meta) {
  ChartListItem item;
  item.id = meta.id;
  item.indexNumber = meta.indexNumber;
  item.name = meta.name;
  item.georef = meta.georef;
  if (meta.category == "DEP") {
    item.category = ChartCategory::Departure;
  } else if (meta.category == "ARR") {
    item.category = ChartCategory::Arrival;
  } else if (meta.category == "APP") {
    item.category = ChartCategory::Approach;
  } else if (meta.category == "APT") {
    item.category = ChartCategory::Airport;
  } else if (meta.category == "REF") {
    item.category = ChartCategory::Reference;
  } else {
    item.category = ChartCategory::Other;
  }
  return item;
}

ChartIndexResult NavigraphFetchChartIndex(const std::string& accessToken,
                                          const std::string& icao) {
  ChartIndexResult result;
  result.icao = icao;
  if (accessToken.empty()) {
    result.error = "NOT SIGNED IN";
    return result;
  }
  if (icao.empty()) {
    result.error = "NO AIRPORT";
    return result;
  }

  // rules=ANY returns both IFR and VFR charts so the page lists everything the
  // subscription allows; version=STD is the standard (non-CAO) chart set.
  const std::string url =
      std::string(kChartsBaseUrl) + icao + "?version=STD&rules=ANY";
  std::string body;
  long httpStatus = 0;
  if (!HttpGetBearer(url, accessToken, WriteToString, &body, httpStatus)) {
    result.error = "NO CONNECTION";
    return result;
  }
  if (httpStatus == 401 || httpStatus == 403) {
    if (!NavigraphTokenHasChartsSubscription(accessToken)) {
      result.error = "SIGN OUT AND SIGN IN";
    } else if (httpStatus == 403) {
      result.error = "ULTIMATE REQUIRED";
    } else {
      result.error = "ACCESS DENIED";
    }
    return result;
  }
  if (httpStatus == 404) {
    // No chart document for this airport: report an empty (but successful) set.
    result.ok = true;
    return result;
  }
  if (httpStatus != 200) {
    result.error = "HTTP " + std::to_string(httpStatus);
    return result;
  }

  const json j = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) {
    result.error = "BAD RESPONSE";
    return result;
  }
  // The document is either { "charts": [...] } or a bare array, depending on
  // API version; accept both.
  const json* charts = nullptr;
  if (j.is_object()) {
    const auto it = j.find("charts");
    if (it != j.end() && it->is_array()) charts = &(*it);
  } else if (j.is_array()) {
    charts = &j;
  }
  if (charts == nullptr) {
    result.error = "BAD RESPONSE";
    return result;
  }
  for (const json& entry : *charts) {
    if (!entry.is_object()) continue;
    NavigraphChartMeta meta = ParseChart(entry, icao);
    if (!meta.id.empty()) result.charts.push_back(std::move(meta));
  }
  result.ok = true;
  return result;
}

ChartImageResult NavigraphFetchChartImage(const std::string& accessToken,
                                          const std::string& chartId, bool night,
                                          const std::string& imageUrl) {
  ChartImageResult result;
  result.chartId = chartId;
  result.night = night;
  if (accessToken.empty()) {
    result.error = "NOT SIGNED IN";
    return result;
  }
  if (imageUrl.empty()) {
    result.error = "NO IMAGE";
    return result;
  }
  long httpStatus = 0;
  if (!HttpGetBearer(imageUrl, accessToken, WriteToBytes, &result.pngBytes,
                     httpStatus)) {
    result.pngBytes.clear();
    result.error = "NO CONNECTION";
    return result;
  }
  if (httpStatus != 200 || result.pngBytes.empty()) {
    result.pngBytes.clear();
    if (httpStatus == 401 || httpStatus == 403) {
      if (!NavigraphTokenHasChartsSubscription(accessToken)) {
        result.error = "SIGN OUT AND SIGN IN";
      } else if (httpStatus == 403) {
        result.error = "ULTIMATE REQUIRED";
      } else {
        result.error = "ACCESS DENIED";
      }
    } else {
      result.error = "HTTP " + std::to_string(httpStatus);
    }
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace avionics
