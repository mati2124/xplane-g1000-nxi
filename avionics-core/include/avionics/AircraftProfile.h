#pragma once

#include <string>

namespace avionics {

// Maps the currently loaded X-Plane aircraft to the avionics assets that should
// follow the airframe: which EIS (engine page) layout and which checklist file
// to display. The G1000 NXi shows a different engine page and a different set of
// checklists per aircraft, so these are data-driven and swap automatically when
// the user changes aircraft instead of being hardcoded into the avionics.
//
// Detection order (see resolveAircraftProfile):
//   1. acf_ICAO type code (e.g. "C172", "SF50") - the authoritative identifier.
//   2. acf_relative_path substring fallback when the ICAO code is missing or
//      generic (third-party aircraft sometimes leave acf_ICAO blank).
// Anything unrecognized falls back to the Cessna piston profile, matching the
// project's default fit.
struct AircraftProfile {
  // Stable short id for logging / debugging (e.g. "c172", "sf50").
  std::string id;
  // Asset-relative paths (resolved through assets::resolve against the bundled
  // assets, with a beside-the-.acf override taking precedence in the stores).
  std::string eisAsset;
  std::string checklistAsset;
  // MFD power-up left panel (PNG under assets/boot/). Empty when the profile
  // has no bundled art; ICAO-keyed boot/<icao>.png is tried first via
  // typeKeyedBootHeroAsset().
  std::string bootHeroAsset;
  // Marketing / type string on the MFD power-up database row (e.g. "Cessna 172S").
  std::string bootAirframeName;
  // Assumed fly-by bank angle (degrees) for turn-anticipation lead distance and
  // leg sequencing. Matched to the native X-Plane lateral autopilot for each
  // airframe so NAV coupling starts outbound steering when the sim AP would.
  double flyByBankDeg = 15.0;
};

// Relative asset paths for the built-in profiles, also used as the dev-tree
// fallback keys (AVIONICS_CORE_ASSET_DIR + "/" + <asset>).
namespace aircraft_assets {
inline constexpr const char* kEisDir = "eis";
inline constexpr const char* kEisExt = ".eis";
inline constexpr const char* kChecklistDir = "checklists";
inline constexpr const char* kChecklistExt = ".checklist";
inline constexpr const char* kBootDir = "boot";
inline constexpr const char* kBootHeroExt = ".png";
inline constexpr const char* kC172Eis = "eis/c172s.eis";
inline constexpr const char* kC172Checklist = "checklists/c172.checklist";
inline constexpr const char* kSf50Eis = "eis/sf50.eis";
inline constexpr const char* kSf50Checklist = "checklists/sf50.checklist";
inline constexpr const char* kPa46tEis = "eis/pa46t.eis";
inline constexpr const char* kPa46tChecklist = "checklists/pa46t.checklist";
}  // namespace aircraft_assets

// Relative asset paths (under the runtime assets/ tree) for a user-droppable,
// ICAO-keyed override that lets users add support for ANY aircraft without a
// rebuild or any code change: dropping assets/eis/<icao>.eis and
// assets/checklists/<icao>.checklist makes the avionics load them for that
// airframe. Keyed off X-Plane's acf_ICAO type code, sanitized to lowercase
// alphanumerics (e.g. "TBM9" -> "eis/tbm9.eis"). Returns empty when `icaoType`
// has no usable characters, so callers can skip the lookup.
std::string typeKeyedEisAsset(const std::string& icaoType);
std::string typeKeyedChecklistAsset(const std::string& icaoType);

// User-droppable MFD power-up panel image: assets/boot/<icao>.png (sanitized
// lowercase alphanumerics from acf_ICAO, e.g. C172 -> boot/c172.png).
std::string typeKeyedBootHeroAsset(const std::string& icaoType);

// Resolve the first existing MFD boot hero PNG using the same precedence as
// EIS/checklists: beside-the-.acf g1000_boot.png, ICAO-keyed assets/boot file,
// then the bundled profile default.
std::string resolveBootHeroAsset(const std::string& icaoType,
                                 const std::string& acfRelativePath);

// Resolve the profile for the loaded aircraft. `icaoType` is X-Plane's
// sim/aircraft/view/acf_ICAO (may be empty); `acfRelativePath` is
// sim/aircraft/view/acf_relative_path (may be empty in tools/tests). Either may
// be supplied on its own.
AircraftProfile resolveAircraftProfile(const std::string& icaoType,
                                       const std::string& acfRelativePath);

// Fly-by bank used by turnLeadDistanceNm / FmsNavigator sequencing for the
// loaded aircraft (see AircraftProfile::flyByBankDeg).
double resolveTurnLeadBankDeg(const std::string& icaoType,
                              const std::string& acfRelativePath);

}  // namespace avionics
