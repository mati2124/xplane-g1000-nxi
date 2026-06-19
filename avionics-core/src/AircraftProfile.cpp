#include "avionics/AircraftProfile.h"

#include <algorithm>
#include <cctype>

namespace avionics {
namespace {

std::string toUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool contains(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}

std::string trim(const std::string& s) {
  std::size_t start = 0;
  while (start < s.size() &&
         std::isspace(static_cast<unsigned char>(s[start]))) {
    ++start;
  }
  std::size_t end = s.size();
  while (end > start &&
         std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(start, end - start);
}

// Reduce an aircraft type code to a safe filename stem: lowercase alphanumerics
// only. Drops spaces, slashes, and anything that could escape the assets folder
// so a malformed acf_ICAO can never resolve outside assets/eis|checklists.
std::string sanitizeTypeKey(const std::string& icaoType) {
  std::string out;
  out.reserve(icaoType.size());
  for (unsigned char c : icaoType) {
    if (std::isalnum(c)) out.push_back(static_cast<char>(std::tolower(c)));
  }
  return out;
}

AircraftProfile c172Profile() {
  return {"c172", aircraft_assets::kC172Eis, aircraft_assets::kC172Checklist};
}

AircraftProfile sf50Profile() {
  return {"sf50", aircraft_assets::kSf50Eis, aircraft_assets::kSf50Checklist};
}

}  // namespace

AircraftProfile resolveAircraftProfile(const std::string& icaoType,
                                       const std::string& acfRelativePath) {
  // 1. ICAO type code is authoritative when present.
  const std::string icao = toUpper(trim(icaoType));
  if (icao == "SF50") return sf50Profile();
  if (icao == "C172" || icao == "C72R" || icao == "C172SP") return c172Profile();

  // 2. Fall back to the .acf path for aircraft that leave acf_ICAO blank or
  //    generic. Match on the model name as it appears in the install folder.
  const std::string path = toLower(acfRelativePath);
  if (contains(path, "sf50") || contains(path, "vision jet") ||
      contains(path, "visionjet") || contains(path, "cirrus")) {
    return sf50Profile();
  }
  if (contains(path, "c172") || contains(path, "cessna_172") ||
      contains(path, "skyhawk")) {
    return c172Profile();
  }

  // 3. Default to the Cessna piston fit (the project's baseline aircraft).
  return c172Profile();
}

std::string typeKeyedEisAsset(const std::string& icaoType) {
  const std::string key = sanitizeTypeKey(icaoType);
  if (key.empty()) return std::string();
  return std::string(aircraft_assets::kEisDir) + "/" + key +
         aircraft_assets::kEisExt;
}

std::string typeKeyedChecklistAsset(const std::string& icaoType) {
  const std::string key = sanitizeTypeKey(icaoType);
  if (key.empty()) return std::string();
  return std::string(aircraft_assets::kChecklistDir) + "/" + key +
         aircraft_assets::kChecklistExt;
}

}  // namespace avionics
