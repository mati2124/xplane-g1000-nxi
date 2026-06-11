#include "SimBriefClient.h"

#include <curl/curl.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <string>

namespace avionics {
namespace {

using nlohmann::json;

// SimBrief's public OFP fetcher (Navigraph developer docs). The Pilot ID
// selects the account; json=v2 is the stable JSON layout recommended for new
// integrations.
constexpr const char* kFetchUrlBase =
    "https://www.simbrief.com/api/xml.fetcher.php?userid=";
constexpr const char* kFetchUrlJsonSuffix = "&json=v2";
constexpr long kHttpTimeoutSeconds = 20;

// Navlog pseudo-fixes computed by SimBrief rather than filed (top of climb /
// descent). They are not flight-plan waypoints, so they are skipped.
constexpr const char* kIdentTopOfClimb = "TOC";
constexpr const char* kIdentTopOfDescent = "TOD";

std::size_t WriteToString(const char* data, std::size_t size,
                          std::size_t nmemb, void* userData) {
  static_cast<std::string*>(userData)->append(data, size * nmemb);
  return size * nmemb;
}

// SimBrief v1 served every scalar as a string; v2 mostly keeps that but is
// not guaranteed to. Accept either representation.
std::string asString(const json& v) {
  if (v.is_string()) return v.get<std::string>();
  if (v.is_number_integer()) return std::to_string(v.get<long long>());
  if (v.is_number_float()) return std::to_string(v.get<double>());
  return {};
}

bool asDouble(const json& v, double& out) {
  if (v.is_number()) {
    out = v.get<double>();
    return true;
  }
  if (v.is_string()) {
    const std::string s = v.get<std::string>();
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end != s.c_str();
  }
  return false;
}

// Reads an {icao_code, pos_lat, pos_long} block (origin/destination) into a
// MapLeg. Returns false when any field is missing/unparsable.
bool readAirport(const json& j, MapLeg& out) {
  if (!j.is_object()) return false;
  out.id = asString(j.value("icao_code", json()));
  return !out.id.empty() && asDouble(j.value("pos_lat", json()), out.lat) &&
         asDouble(j.value("pos_long", json()), out.lon);
}

// "2026-06-04T21:22:08Z" -> "04JUN 21:22Z" (G1000-style date/time field).
std::string formatGeneratedTime(const std::string& iso) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0;
  if (std::sscanf(iso.c_str(), "%d-%d-%dT%d:%d", &year, &month, &day, &hour,
                  &minute) != 5 ||
      month < 1 || month > 12) {
    return iso;  // unexpected layout: show it raw rather than nothing
  }
  static constexpr const char* kMonths[] = {"JAN", "FEB", "MAR", "APR",
                                            "MAY", "JUN", "JUL", "AUG",
                                            "SEP", "OCT", "NOV", "DEC"};
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%02d%s %02d:%02dZ", day, kMonths[month - 1],
                hour, minute);
  return buf;
}

SimBriefFetchResult parseOfp(const std::string& body, long httpStatus) {
  SimBriefFetchResult result;

  json j = json::parse(body, /*cb=*/nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    result.error = httpStatus == 200 ? "UNEXPECTED RESPONSE"
                                     : "HTTP " + std::to_string(httpStatus);
    return result;
  }

  // The fetcher reports failures (e.g. unknown Pilot ID) as HTTP 400 with a
  // status string like "Error: Unknown UserID".
  const std::string status = asString(j["fetch"]["status"]);
  if (status != "Success") {
    result.error = status.empty()
                       ? "HTTP " + std::to_string(httpStatus)
                       : status;
    // Uppercase for the page's caution-style message.
    for (char& c : result.error) c = static_cast<char>(std::toupper(c));
    return result;
  }

  MapLeg origin;
  MapLeg destination;
  if (!readAirport(j["origin"], origin) ||
      !readAirport(j["destination"], destination)) {
    result.error = "OFP MISSING AIRPORTS";
    return result;
  }

  // v2 serves the navlog as a flat array; v1 wrapped it as {"fix": [...]}.
  const json* fixes = nullptr;
  const json& nav = j["navlog"];
  if (nav.is_array()) {
    fixes = &nav;
  } else if (nav.is_object() && nav["fix"].is_array()) {
    fixes = &nav["fix"];
  }

  result.legs.push_back(origin);
  if (fixes != nullptr) {
    for (const json& fix : *fixes) {
      MapLeg leg;
      leg.id = asString(fix.value("ident", json()));
      if (leg.id.empty() || leg.id == kIdentTopOfClimb ||
          leg.id == kIdentTopOfDescent) {
        continue;
      }
      if (!asDouble(fix.value("pos_lat", json()), leg.lat) ||
          !asDouble(fix.value("pos_long", json()), leg.lon)) {
        continue;
      }
      result.legs.push_back(std::move(leg));
    }
  }
  // The navlog normally ends with the destination airport; append it when an
  // OFP variant leaves it out so the drawn route always reaches the field.
  if (result.legs.back().id != destination.id) {
    result.legs.push_back(destination);
  }
  if (result.legs.size() < 2) {
    result.error = "OFP HAS NO ROUTE";
    result.legs.clear();
    return result;
  }

  result.originIcao = origin.id;
  result.destinationIcao = destination.id;
  result.route = asString(j["general"]["route"]);
  result.generatedUtc =
      formatGeneratedTime(asString(j["params"]["time_generated"]));
  result.ok = true;
  return result;
}

}  // namespace

SimBriefFetchResult FetchSimBriefOfp(const std::string& pilotId) {
  SimBriefFetchResult result;

  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    result.error = "HTTP CLIENT UNAVAILABLE";
    return result;
  }

  const std::string url = kFetchUrlBase + pilotId + kFetchUrlJsonSuffix;
  std::string body;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, kHttpTimeoutSeconds);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "XPlaneAvionics");

  const CURLcode rc = curl_easy_perform(curl);
  long httpStatus = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpStatus);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) {
    // e.g. no network, DNS failure, timeout.
    result.error = "NO CONNECTION";
    return result;
  }
  return parseOfp(body, httpStatus);
}

}  // namespace avionics
