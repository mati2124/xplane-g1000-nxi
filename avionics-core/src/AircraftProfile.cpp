#include "avionics/AircraftProfile.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <vector>

#include "avionics/AssetPaths.h"

#ifndef AVIONICS_CORE_ASSET_DIR
#define AVIONICS_CORE_ASSET_DIR ""
#endif

namespace avionics {
namespace {

std::string toUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

bool contains(const std::string &haystack, const char *needle) {
  return haystack.find(needle) != std::string::npos;
}

std::string trim(const std::string &s) {
  std::size_t start = 0;
  while (start < s.size() &&
         std::isspace(static_cast<unsigned char>(s[start]))) {
    ++start;
  }
  std::size_t end = s.size();
  while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(start, end - start);
}

// Reduce an aircraft type code to a safe filename stem: lowercase alphanumerics
// only. Drops spaces, slashes, and anything that could escape the assets folder
// so a malformed acf_ICAO can never resolve outside assets/eis|checklists|boot.
std::string sanitizeTypeKey(const std::string &icaoType) {
  std::string out;
  out.reserve(icaoType.size());
  for (unsigned char c : icaoType) {
    if (std::isalnum(c))
      out.push_back(static_cast<char>(std::tolower(c)));
  }
  return out;
}

std::string devFallback(const std::string &assetRel) {
  const std::string base = AVIONICS_CORE_ASSET_DIR;
  if (base.empty())
    return std::string();
  return base + "/" + assetRel;
}

AircraftProfile c172Profile() {
  return {"c172", aircraft_assets::kC172Eis, aircraft_assets::kC172Checklist,
          std::string(), "Cessna 172S", 15.0, 1};
}

AircraftProfile sf50Profile() {
  // X-Plane's Vision Jet lateral AP banks steeper than the G1000 NXi 15° piston
  // default; a higher assumed bank shrinks fly-by lead so outbound steering and
  // leg sequencing align with when the sim AP actually begins the turn.
  return {"sf50", aircraft_assets::kSf50Eis, aircraft_assets::kSf50Checklist,
          std::string(), "Cirrus Vision SF50", 25.0, 1};
}

AircraftProfile pa46tProfile() {
  return {"pa46t", aircraft_assets::kPa46tEis, aircraft_assets::kPa46tChecklist,
          std::string(), "Piper PA-46T", 15.0, 2};
}

AircraftProfile c208Profile() {
  return {"c208", aircraft_assets::kC208Eis, aircraft_assets::kC208Checklist,
          std::string(), "Cessna 208B Grand Caravan", 15.0, 2};
}

std::vector<std::string>
candidateBootHeroPaths(const std::string &aircraftAcfRelativePath,
                       const std::string &typeKeyedPath,
                       const std::string &profileBundledPath) {
  std::vector<std::string> paths;
  namespace fs = std::filesystem;

  auto pushIfFile = [&](const std::string &path) {
    if (path.empty())
      return;
    std::error_code ec;
    if (fs::is_regular_file(path, ec))
      paths.push_back(path);
  };

  if (!aircraftAcfRelativePath.empty()) {
    const fs::path acf = fs::path(aircraftAcfRelativePath);
    const fs::path dir = acf.parent_path();
    pushIfFile((dir / "g1000_boot.png").string());
    if (acf.has_stem()) {
      pushIfFile((dir / (acf.stem().string() + "_boot.png")).string());
    }
  }

  pushIfFile(typeKeyedPath);
  pushIfFile(profileBundledPath);
  return paths;
}

} // namespace

AircraftProfile resolveAircraftProfile(const std::string &icaoType,
                                       const std::string &acfRelativePath) {
  // 1. ICAO type code is authoritative when present.
  const std::string icao = toUpper(trim(icaoType));
  if (icao == "SF50")
    return sf50Profile();
  if (icao == "C208" || icao == "C208B")
    return c208Profile();
  if (icao == "PA46T" || icao == "PA46")
    return pa46tProfile();
  if (icao == "C172" || icao == "C72R" || icao == "C172SP")
    return c172Profile();

  // 2. Fall back to the .acf path for aircraft that leave acf_ICAO blank or
  //    generic. Match on the model name as it appears in the install folder.
  const std::string path = toLower(acfRelativePath);
  if (contains(path, "sf50") || contains(path, "vision jet") ||
      contains(path, "visionjet") || contains(path, "cirrus")) {
    return sf50Profile();
  }
  if (contains(path, "c208") || contains(path, "caravan")) {
    return c208Profile();
  }
  if (contains(path, "pa46t") || contains(path, "pa46") ||
      contains(path, "meridian") || contains(path, "malibu")) {
    return pa46tProfile();
  }
  if (contains(path, "c172") || contains(path, "cessna_172") ||
      contains(path, "skyhawk")) {
    return c172Profile();
  }

  // 3. Default to the Cessna piston fit (the project's baseline aircraft).
  return c172Profile();
}

double resolveTurnLeadBankDeg(const std::string &icaoType,
                              const std::string &acfRelativePath) {
  return resolveAircraftProfile(icaoType, acfRelativePath).flyByBankDeg;
}

std::string typeKeyedEisAsset(const std::string &icaoType) {
  const std::string key = sanitizeTypeKey(icaoType);
  if (key.empty())
    return std::string();
  return std::string(aircraft_assets::kEisDir) + "/" + key +
         aircraft_assets::kEisExt;
}

std::string typeKeyedChecklistAsset(const std::string &icaoType) {
  const std::string key = sanitizeTypeKey(icaoType);
  if (key.empty())
    return std::string();
  return std::string(aircraft_assets::kChecklistDir) + "/" + key +
         aircraft_assets::kChecklistExt;
}

std::string typeKeyedBootHeroAsset(const std::string &icaoType) {
  const std::string key = sanitizeTypeKey(icaoType);
  if (key.empty())
    return std::string();
  return std::string(aircraft_assets::kBootDir) + "/" + key +
         aircraft_assets::kBootHeroExt;
}

std::string resolveBootHeroAsset(const std::string &icaoType,
                                 const std::string &acfRelativePath) {
  const AircraftProfile profile =
      resolveAircraftProfile(icaoType, acfRelativePath);

  const std::string typeAsset = typeKeyedBootHeroAsset(icaoType);
  const std::string typeKeyed =
      typeAsset.empty() ? std::string()
                        : assets::resolve(typeAsset, devFallback(typeAsset));

  const std::string bundledRel =
      profile.bootHeroAsset.empty() && !profile.id.empty()
          ? typeKeyedBootHeroAsset(profile.id)
          : profile.bootHeroAsset;
  const std::string bundled =
      bundledRel.empty() ? std::string()
                         : assets::resolve(bundledRel, devFallback(bundledRel));

  const std::vector<std::string> candidates =
      candidateBootHeroPaths(acfRelativePath, typeKeyed, bundled);
  if (candidates.empty())
    return std::string();
  return candidates.front();
}

} // namespace avionics
