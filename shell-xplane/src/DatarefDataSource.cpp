#include "DatarefDataSource.h"
#include "FmsDebugOverlay.h"
#include "FmsRouteProgrammer.h"
#include "avionics/EisLegacy.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavigationComputer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include "avionics/AptDatGeometryCache.h"
#include "avionics/AptDatParser.h"
#include "avionics/AssetPaths.h"
#include "avionics/Datarefs.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/MfdController.h"
#include "avionics/OpenAirParser.h"
#include "avionics/nav/NearbyFeatureSelect.h"

namespace avionics {
namespace {

// Inset-map display range and the cap on features handed to the renderer.
constexpr float kMapRangeNm = 10.0f;
// Query radius for the nearby-feature/airspace scans: must cover the longest
// MFD map range and any Map-Pointer pan, not just the inset default (mirrors
// the standalone shell's kMapQueryRangeNm). Using the 10 NM display range here
// left zoomed-out and panned views empty.
constexpr float kMapQueryRangeNm = 160.0f;
// Deep enough for the per-type reserves in filterNearby (airports + navaids) to
// fully populate a wide MFD MAP view before fixes fill the remaining budget.
constexpr std::size_t kMaxMapFeatures = 500;
constexpr std::size_t kMaxMapAirspaces = 60;
constexpr float kRunwayQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapRunways = 120;
constexpr float kTaxiwayQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapTaxiways = 600;
constexpr float kTaxiwayLabelQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapTaxiwayLabels = 400;
// Obstacles only draw at low ranges (and the DOF is dense).
constexpr float kObstacleQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapObstacles = 300;
constexpr std::size_t kMaxMapLandLines = 8000;
constexpr std::size_t kMaxMapCities = 600;

#ifndef AVIONICS_LAND_DATA
#define AVIONICS_LAND_DATA ""
#endif
constexpr const char* kLandDataAssetPath = AVIONICS_LAND_DATA;

// X-Plane's bundled OpenAir airspace file, relative to the system path
// (XPLMGetSystemPath). User-updated data under Custom Data wins when present.
const char* kAirspaceRelPaths[] = {
    "Custom Data/Airspaces/airspace.txt",
    "Resources/default data/airspaces/airspace.txt",
};

const char* kAptDatRelPaths[] = {
    "Global Scenery/Global Airports/Earth nav data/apt.dat",
    "Custom Scenery/Global Airports/Earth nav data/apt.dat",
};

// Global Scenery "Earth nav data" directory holding the 1°x1° DSF DEM tiles the
// terrain background samples, relative to the install root (XPLMGetSystemPath).
// Mirrors the candidates xplane_install::earthNavDataDir() probes.
const char* kEarthNavDataRelPaths[] = {
    "Global Scenery/X-Plane 12 Global Scenery/Earth nav data",
    "Global Scenery/X-Plane 11 Global Scenery/Earth nav data",
    "Global Scenery/Earth nav data",
};
// Nearby-feature list is range-filtered from the cache on this cadence rather
// than every frame (ownship position itself still updates every frame).
constexpr double kMapRebuildIntervalSeconds = 1.0;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

// X-Plane's NAV/COM frequency datarefs are integers of MHz x 100 (e.g. 11800
// == 118.00 MHz), so convert in both directions around the FlightData MHz.
constexpr float kRadioHzToMhz = 0.01f;
constexpr float kMhzToRadioHz = 100.0f;

// X-Plane failure_enum: 0 = working, 6 = inoperative (failed now); intermediate
// values are armed conditions that have not yet tripped. An instrument is shown
// failed (red X) only once its dataref reads inoperative.
constexpr int kFailureInop = 6;

// True when a failure_enum dataref reports the system inoperative. A missing
// dataref (null) reads as healthy so the glass is never stuck red-X'd on
// airframes that don't expose that failure.
bool failedInop(XPLMDataRef ref) {
  return ref != nullptr && XPLMGetDatai(ref) == kFailureInop;
}

// X-Plane 12 transponder_mode enum (off=0, stdby=1, on=2, alt=3, test=4, with
// 6/7 the TCAS traffic modes). The G1000 annunciates Mode C as ALT, and the
// traffic modes still squawk altitude, so they map to ALT / TA / TA-RA rather
// than a bare "ON". Mirrors the standalone shell's decode.
// Read a null-terminated byte[] nav ident and trim trailing whitespace.
std::string readNavIdentDataref(XPLMDataRef ref) {
  if (ref == nullptr) return {};
  char buf[8] = {};
  int n = XPLMGetDatab(ref, buf, 0, static_cast<int>(sizeof(buf)) - 1);
  if (n < 0) n = 0;
  buf[n] = '\0';
  std::string id(buf);
  while (!id.empty() &&
         std::isspace(static_cast<unsigned char>(id.back()))) {
    id.pop_back();
  }
  return id;
}

// navN_nav_id for VOR/LOC; navN_dme_id for standalone DME/TACAN (XP12).
std::string readNavStationIdent(XPLMDataRef navId, XPLMDataRef dmeId) {
  std::string id = readNavIdentDataref(navId);
  if (!id.empty()) return id;
  return readNavIdentDataref(dmeId);
}

const char* xpdrModeString(int mode) {
  switch (mode) {
    case 0:
      return "OFF";
    case 1:
      return "STBY";
    case 2:
      return "ON";
    case 4:
      return "TEST";
    case 6:
      return "TA";
    case 7:
      return "TA/RA";
    default:
      return "ALT";
  }
}

// Resolve a (possibly array-element) dataref path. The "[index]" suffix is RREF
// wire syntax that the in-process SDK's XPLMFindDataRef does not accept, so it
// is stripped here and returned separately; the element is then read with
// XPLMGetDatavf. A scalar path yields index -1 (read with XPLMGetDataf).
struct ResolvedDataRef {
  XPLMDataRef ref;
  int arrayIndex;
};

ResolvedDataRef resolveDataRef(const char* path) {
  std::string p(path);
  int arrayIndex = -1;
  const std::size_t open = p.find('[');
  if (open != std::string::npos && !p.empty() && p.back() == ']') {
    arrayIndex = std::atoi(p.c_str() + open + 1);
    p.resize(open);
  }
  return {XPLMFindDataRef(p.c_str()), arrayIndex};
}

// Navaid identifier buffer: the SDK recommends >= 6 chars; 32 is generous.
constexpr int kNavIdBufferSize = 32;

// Return the features from `src` within rangeNm of (lat, lon), capped at
// maxCount. Shared shape with the standalone NavDataStore::nearby.
std::vector<MapFeature> filterNearby(const std::vector<MapFeature>& src,
                                     double lat, double lon, float rangeNm,
                                     std::size_t maxCount) {
  return assembleNearbyMapFeaturesMixed(src, lat, lon, rangeNm, maxCount);
}

// A single decoded FMS flight-plan entry: identifier plus lat/lon. The flight
// plan is only reachable through the FMS SDK, not datarefs. The SDK fills outID
// with the waypoint identifier; the buffer must be generous (lat/lon entries
// can be long), so use the documented 256-byte size and force null-termination.
struct FmsEntry {
  std::string id;
  float lat = 0.0f;
  float lon = 0.0f;
};

FmsEntry fmsEntry(int index) {
  XPLMNavType type = xplm_Nav_Unknown;
  char id[256] = {};
  XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
  int altitude = 0;
  FmsEntry entry;
  XPLMGetFMSEntryInfo(index, &type, id, &ref, &altitude, &entry.lat,
                      &entry.lon);
  id[sizeof(id) - 1] = '\0';
  entry.id = id;
  return entry;
}

std::string fmsEntryId(int index) { return fmsEntry(index).id; }

std::string flightPlanIdentAt(const std::vector<MapLeg>& plan, int index) {
  if (index < 0 || index >= static_cast<int>(plan.size())) return {};
  return plan[static_cast<std::size_t>(index)].id;
}

bool idsEqual(const std::string& a, const std::string& b) {
  if (a == b) return true;
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const char ca = static_cast<char>(std::tolower(
        static_cast<unsigned char>(a[i])));
    const char cb = static_cast<char>(std::tolower(
        static_cast<unsigned char>(b[i])));
    if (ca != cb) return false;
  }
  return true;
}

int legIndexInPlan(const std::vector<MapLeg>& plan, const std::string& id) {
  if (id.empty()) return -1;
  for (int i = 0; i < static_cast<int>(plan.size()); ++i) {
    if (idsEqual(plan[static_cast<std::size_t>(i)].id, id)) return i;
  }
  return -1;
}

std::vector<MapLeg> fmsFlightPlanSnapshot(int count) {
  std::vector<MapLeg> plan;
  plan.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const FmsEntry entry = fmsEntry(i);
    if (entry.lat == 0.0f && entry.lon == 0.0f) continue;
    plan.push_back({static_cast<double>(entry.lat), static_cast<double>(entry.lon),
                    entry.id});
  }
  return plan;
}

bool dirExists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::is_directory(path, ec);
}

bool fileExists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(path, ec);
}

bool installRootLooksValid(const std::string& root) {
  if (root.empty()) return false;
  return dirExists(root + "Resources") || dirExists(root + "Global Scenery");
}

// Derive the install root from this plugin's path, walking up directory
// components until one looks like an X-Plane root. The plugin lives at
// .../Resources/plugins/<name>/mac_x64/xplane-avionics.xpl, so the root is a
// few levels up, but the exact depth (and whether the path is absolute) can
// vary, so probe each ancestor rather than assume a fixed count.
std::string installRootFromPluginPath() {
  char pluginPath[2048] = {};
  XPLMGetPluginInfo(XPLMGetMyID(), nullptr, pluginPath, nullptr, nullptr);
  std::string path(pluginPath);
  for (int i = 0; i < 8 && !path.empty(); ++i) {
    const std::size_t slash = path.find_last_of("/\\");
    if (slash == std::string::npos) break;
    path.resize(slash);
    const std::string candidate = path + "/";
    if (installRootLooksValid(candidate)) return candidate;
  }
  return {};
}

std::string resolveInstallRoot() {
  char systemPath[1024] = {};
  XPLMGetSystemPath(systemPath);
  std::string root(systemPath);
  if (!root.empty() && root.back() != '/' && root.back() != '\\') {
    root += '/';
  }
  {
    char msg[1200];
    std::snprintf(msg, sizeof(msg),
                  "G1000 NXi: XPLMGetSystemPath = '%s'\n", root.c_str());
    XPLMDebugString(msg);
  }
  if (installRootLooksValid(root)) return root;

  const std::string fromPlugin = installRootFromPluginPath();
  {
    char msg[1200];
    std::snprintf(msg, sizeof(msg),
                  "G1000 NXi: install root from plugin path = '%s'\n",
                  fromPlugin.c_str());
    XPLMDebugString(msg);
  }
  if (installRootLooksValid(fromPlugin)) return fromPlugin;
  return {};
}

const char* firstExistingFile(const std::string& root, const char* const* relPaths,
                              std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    if (fileExists(root + relPaths[i])) return relPaths[i];
  }
  return nullptr;
}

}  // namespace

DatarefDataSource::DatarefDataSource(EisSource* eisSource)
    : eisSource_(eisSource) {
  airspeed_ = XPLMFindDataRef(datarefs::kAirspeedKts);
  altitude_ = XPLMFindDataRef(datarefs::kAltitudeFt);
  heading_ = XPLMFindDataRef(datarefs::kHeadingDegMag);
  pitch_ = XPLMFindDataRef(datarefs::kPitchDeg);
  roll_ = XPLMFindDataRef(datarefs::kRollDeg);
  flightDirectorPitch_ = XPLMFindDataRef(datarefs::kFlightDirectorPitch);
  flightDirectorRoll_ = XPLMFindDataRef(datarefs::kFlightDirectorRoll);
  flightDirectorMode_ = XPLMFindDataRef(datarefs::kFlightDirectorMode);
  verticalSpeed_ = XPLMFindDataRef(datarefs::kVerticalSpeedFpm);
  slip_ = XPLMFindDataRef(datarefs::kSlipDeg);

  latitude_ = XPLMFindDataRef(datarefs::kLatitudeDeg);
  longitude_ = XPLMFindDataRef(datarefs::kLongitudeDeg);

  zuluTimeSec_ = XPLMFindDataRef(datarefs::kZuluTimeSec);
  localDateDays_ = XPLMFindDataRef(datarefs::kLocalDateDays);

  gpsDistance_ = XPLMFindDataRef(datarefs::kGpsDistanceNm);
  gpsBearing_ = XPLMFindDataRef(datarefs::kGpsBearingDegMag);
  gpsNavId_ = XPLMFindDataRef(datarefs::kGpsNavId);
  gpsHdef_ = XPLMFindDataRef(datarefs::kGpsHdefNmPerDot);
  hsiObsCourse_ = XPLMFindDataRef(datarefs::kHsiObsCourseDegMag);
  hsiSourceSelect_ = XPLMFindDataRef(datarefs::kHsiSourceSelect);
  overrideGps_ = XPLMFindDataRef(datarefs::kOverrideGps);
  gpsCourseDegMag_ = XPLMFindDataRef(datarefs::kGpsCourseDegMag);
  gpsHdefDot_ = XPLMFindDataRef(datarefs::kGpsHdefDot);
  gpsDmeDistOverride_ = XPLMFindDataRef(datarefs::kGpsDmeDistOverride);
  nav1NavId_ = XPLMFindDataRef(datarefs::kNav1NavId);
  nav2NavId_ = XPLMFindDataRef(datarefs::kNav2NavId);
  nav1DmeId_ = XPLMFindDataRef(datarefs::kNav1DmeId);
  nav2DmeId_ = XPLMFindDataRef(datarefs::kNav2DmeId);

  radios_[static_cast<int>(RadioUnit::Nav1)] = {
      XPLMFindDataRef(datarefs::kNav1FrequencyHz),
      XPLMFindDataRef(datarefs::kNav1StandbyFrequencyHz),
      &FlightData::nav1ActiveMhz, &FlightData::nav1StandbyMhz,
      XPLMFindDataRef(datarefs::kNav1Volume), &FlightData::nav1Volume,
      XPLMFindDataRef(datarefs::kNav1IdentAudio), &FlightData::nav1IdentAudio};
  radios_[static_cast<int>(RadioUnit::Nav2)] = {
      XPLMFindDataRef(datarefs::kNav2FrequencyHz),
      XPLMFindDataRef(datarefs::kNav2StandbyFrequencyHz),
      &FlightData::nav2ActiveMhz, &FlightData::nav2StandbyMhz,
      XPLMFindDataRef(datarefs::kNav2Volume), &FlightData::nav2Volume,
      XPLMFindDataRef(datarefs::kNav2IdentAudio), &FlightData::nav2IdentAudio};
  radios_[static_cast<int>(RadioUnit::Com1)] = {
      XPLMFindDataRef(datarefs::kCom1FrequencyHz),
      XPLMFindDataRef(datarefs::kCom1StandbyFrequencyHz),
      &FlightData::com1ActiveMhz, &FlightData::com1StandbyMhz,
      XPLMFindDataRef(datarefs::kCom1Volume), &FlightData::com1Volume};
  radios_[static_cast<int>(RadioUnit::Com2)] = {
      XPLMFindDataRef(datarefs::kCom2FrequencyHz),
      XPLMFindDataRef(datarefs::kCom2StandbyFrequencyHz),
      &FlightData::com2ActiveMhz, &FlightData::com2StandbyMhz,
      XPLMFindDataRef(datarefs::kCom2Volume), &FlightData::com2Volume};

  transponderCode_ = XPLMFindDataRef(datarefs::kTransponderCode);
  transponderMode_ = XPLMFindDataRef(datarefs::kTransponderMode);
  batteryMasterOn_ = XPLMFindDataRef(datarefs::kBatteryMasterOn);
  avionicsPowerOn_ = XPLMFindDataRef(datarefs::kAvionicsPowerOn);
  efisMapRangeNm_ = XPLMFindDataRef(datarefs::kEfisMapRangeNm);

  failAttitude_ = XPLMFindDataRef(datarefs::kFailAttitude);
  failHeading_ = XPLMFindDataRef(datarefs::kFailHeading);
  failAirspeed_ = XPLMFindDataRef(datarefs::kFailAirspeed);
  failAltimeter_ = XPLMFindDataRef(datarefs::kFailAltimeter);
  failVerticalSpeed_ = XPLMFindDataRef(datarefs::kFailVerticalSpeed);
  failNav1_ = XPLMFindDataRef(datarefs::kFailNav1);
  failNav2_ = XPLMFindDataRef(datarefs::kFailNav2);
  failCom1_ = XPLMFindDataRef(datarefs::kFailCom1);
  failCom2_ = XPLMFindDataRef(datarefs::kFailCom2);
  failTransponder_ = XPLMFindDataRef(datarefs::kFailTransponder);
  acfRelativePath_ = XPLMFindDataRef(datarefs::kAcfRelativePath);
  acfIcao_ = XPLMFindDataRef(datarefs::kAcfIcao);
  acfVso_ = XPLMFindDataRef(datarefs::kAcfVso);
  acfVs_ = XPLMFindDataRef(datarefs::kAcfVs);
  acfVfe_ = XPLMFindDataRef(datarefs::kAcfVfe);
  acfVno_ = XPLMFindDataRef(datarefs::kAcfVno);
  acfVne_ = XPLMFindDataRef(datarefs::kAcfVne);

  rebuildEisBindings();

  // Natural Earth land overlay (coastlines, borders, cities). The plugin
  // assets search dir is registered in plugin_main before this constructor runs.
  landData_ = std::make_unique<LandDataStore>(
      assets::resolve("land_data.bin", kLandDataAssetPath));
}

void DatarefDataSource::rebuildEisBindings() {
  eisBindings_.clear();
  if (eisSource_ == nullptr || !eisSource_->ready()) return;

  for (const EisDataBinding& spec : eisSource_->layout().bindings) {
    const ResolvedDataRef resolved = resolveDataRef(spec.datarefPath.c_str());
    if (resolved.ref) {
      eisBindings_.push_back({resolved.ref, resolved.arrayIndex, spec.scale,
                              spec.offset, spec.channel});
    }
  }
}

void DatarefDataSource::refreshAirspeedEnvelope() {
  if (acfVso_) data_.airspeedEnvelopeVsoKt = XPLMGetDataf(acfVso_);
  if (acfVs_) data_.airspeedEnvelopeVs1Kt = XPLMGetDataf(acfVs_);
  if (acfVfe_) data_.airspeedEnvelopeVfeKt = XPLMGetDataf(acfVfe_);
  if (acfVno_) data_.airspeedEnvelopeVnoKt = XPLMGetDataf(acfVno_);
  if (acfVne_) data_.airspeedEnvelopeVneKt = XPLMGetDataf(acfVne_);
  applyAirspeedEnvelopeDefaults(data_);
}

void DatarefDataSource::setAircraftOverride(AircraftOverride override) {
  if (aircraftOverride_ == override) return;
  aircraftOverride_ = override;
  lastAircraftAcfPath_.clear();
  lastAircraftIcao_.clear();
}

void DatarefDataSource::updateAircraftProfile() {
  std::string acfPath;
  if (acfRelativePath_ != nullptr) {
    char buf[1024] = {};
    const int n =
        XPLMGetDatab(acfRelativePath_, buf, 0, static_cast<int>(sizeof(buf)) - 1);
    if (n > 0) {
      buf[n] = '\0';
      acfPath = buf;
    }
  }

  std::string icao;
  if (acfIcao_ != nullptr) {
    // acf_ICAO is a fixed 40-byte string field, NUL- or space-padded.
    char buf[64] = {};
    const int n =
        XPLMGetDatab(acfIcao_, buf, 0, static_cast<int>(sizeof(buf)) - 1);
    if (n > 0) {
      buf[n] = '\0';
      icao = buf;
      while (!icao.empty() &&
             std::isspace(static_cast<unsigned char>(icao.back()))) {
        icao.pop_back();
      }
    }
  }

  if (aircraftOverride_ == AircraftOverride::C172) {
    icao = "C172";
    acfPath = "c172";
  } else if (aircraftOverride_ == AircraftOverride::SF50) {
    icao = "SF50";
    acfPath = "sf50";
  }

  if (acfPath == lastAircraftAcfPath_ && icao == lastAircraftIcao_) return;
  lastAircraftAcfPath_ = acfPath;
  lastAircraftIcao_ = icao;

  // Re-probe the radar fit for the newly loaded airframe.
  weather_.resetEquipment();

  // Swap the EIS engine page and the checklists to the new aircraft's profile.
  if (eisSource_ != nullptr) {
    eisSource_->setAircraftIdentity(icao, acfPath);
    eisWasReady_ = false;
  }
  if (checklistSource_ != nullptr) {
    checklistSource_->setAircraftIdentity(icao, acfPath);
  }
}

void DatarefDataSource::ensureInstallDataLoaded() {
  if (installDataStarted_) return;
  installDataStarted_ = true;

  const std::string root = resolveInstallRoot();
  if (root.empty()) {
    XPLMDebugString(
        "G1000 NXi: could not resolve X-Plane install root for map data\n");
    airspaceLoaded_.store(true, std::memory_order_release);
    aptDatLoaded_.store(true, std::memory_order_release);
    terrain_ = std::make_unique<DsfTerrainStore>(std::string{});
    return;
  }

  if (const char* airspaceRel = firstExistingFile(
          root, kAirspaceRelPaths,
          sizeof(kAirspaceRelPaths) / sizeof(kAirspaceRelPaths[0]))) {
    char msg[256];
    std::snprintf(msg, sizeof(msg),
                  "G1000 NXi: loading airspace (%s)...\n", airspaceRel);
    XPLMDebugString(msg);
    loadAirspaceAsync(root + airspaceRel);
  } else {
    XPLMDebugString("G1000 NXi: airspace file not found\n");
    airspaceLoaded_.store(true, std::memory_order_release);
  }

  if (const char* aptRel = firstExistingFile(
          root, kAptDatRelPaths,
          sizeof(kAptDatRelPaths) / sizeof(kAptDatRelPaths[0]))) {
    aptDatPath_ = root + aptRel;
    aptGeometryCachePath_ =
        root + "Resources/plugins/xplane-avionics/apt_geometry.cache";
    XPLMDebugString(
        "G1000 NXi: loading airport diagram geometry (apt.dat)...\n");
    loadAptDatAsync();
  } else {
    XPLMDebugString("G1000 NXi: apt.dat not found\n");
    aptDatLoaded_.store(true, std::memory_order_release);
  }

  // X-Plane 12 downloads real-weather METARs to Output/real weather; the store
  // indexes the newest file by ICAO for the WPT - Weather page.
  metar_.start(root);

  std::string earthNavDir;
  for (const char* rel : kEarthNavDataRelPaths) {
    const std::string candidate = root + rel;
    if (dirExists(candidate)) {
      earthNavDir = candidate;
      break;
    }
  }
  terrain_ = std::make_unique<DsfTerrainStore>(std::move(earthNavDir));
}

DatarefDataSource::~DatarefDataSource() {
  // Stop the map-query worker first: it reads navCache_/runwayCells_/
  // airspaceCache_, so it must be joined before those members are destroyed.
  stopMapQueryWorker();
  if (airspaceThread_.joinable()) airspaceThread_.join();
  if (aptDatThread_.joinable()) aptDatThread_.join();
}

void DatarefDataSource::loadAirspaceAsync(std::string airspaceFilePath) {
  airspaceThread_ = std::thread([this, path = std::move(airspaceFilePath)] {
    std::ifstream in(path);
    if (in.good()) airspaceCache_ = parseOpenAir(in);
    airspaceLoaded_.store(true, std::memory_order_release);
  });
}

void DatarefDataSource::loadAptDatAsync() {
  aptDatThread_ = std::thread([this] {
    AptDatParseResult parsed;
    bool loaded = !aptGeometryCachePath_.empty() &&
                  loadAptDatGeometryCache(aptGeometryCachePath_, aptDatPath_,
                                          parsed);
    if (!loaded) {
      std::ifstream in(aptDatPath_);
      if (in.good()) {
        parsed = parseAptDat(in);
        if (!aptGeometryCachePath_.empty()) {
          saveAptDatGeometryCache(aptGeometryCachePath_, aptDatPath_, parsed);
        }
      }
    }
    aptMetaByIcao_ = std::move(parsed.metaByIcao);
    runwayCells_ = std::move(parsed.runwayCells);
    pavementCells_ = std::move(parsed.pavementCells);
    taxiwayLabelCells_ = std::move(parsed.taxiwayLabelCells);
    aptMapDirty_.store(true, std::memory_order_release);
    aptDatLoaded_.store(true, std::memory_order_release);
  });
}

void DatarefDataSource::update(double dtSeconds) {
  ensureInstallDataLoaded();
  metar_.poll(dtSeconds);
  updateAircraftProfile();
  refreshAirspeedEnvelope();
  if (eisSource_ != nullptr) {
    eisSource_->refreshIfChanged();
    const bool ready = eisSource_->ready();
    if (ready && (!eisWasReady_ ||
                  eisBindings_.size() != eisSource_->layout().bindings.size())) {
      rebuildEisBindings();
    }
    eisWasReady_ = ready;
  }
  if (checklistSource_ != nullptr) {
    checklistSource_->refreshIfChanged();
  }

  if (airspeed_) data_.airspeedKts = XPLMGetDataf(airspeed_);
  if (altitude_) data_.altitudeFt = XPLMGetDataf(altitude_);
  if (heading_) data_.headingDeg = XPLMGetDataf(heading_);
  if (pitch_) data_.pitchDeg = XPLMGetDataf(pitch_);
  if (roll_) data_.rollDeg = XPLMGetDataf(roll_);
  if (flightDirectorPitch_)
    data_.fdPitchDeg = XPLMGetDataf(flightDirectorPitch_);
  if (flightDirectorRoll_) data_.fdRollDeg = XPLMGetDataf(flightDirectorRoll_);
  if (flightDirectorMode_) {
    const int fdMode = XPLMGetDatai(flightDirectorMode_);
    data_.flightDirectorActive = fdMode >= 1;
    data_.apEngaged = fdMode == 2;
  }
  if (verticalSpeed_) data_.verticalSpeedFpm = XPLMGetDataf(verticalSpeed_);
  if (slip_) data_.slipSkidDeg = XPLMGetDataf(slip_);

  // Nav status box: distance + magnetic bearing to the active GPS destination.
  if (gpsDistance_) data_.fmaLegDistanceNm = XPLMGetDataf(gpsDistance_);
  if (gpsBearing_) data_.fmaLegBearingDeg = XPLMGetDataf(gpsBearing_);

  // Sim UTC clock (chrome clock readout) and date (Trip Planning
  // sunrise/sunset).
  if (zuluTimeSec_) {
    const int total = static_cast<int>(XPLMGetDataf(zuluTimeSec_));
    data_.utcHour = (total / 3600) % 24;
    data_.utcMinute = (total / 60) % 60;
    data_.utcSecond = total % 60;
  }
  if (localDateDays_) {
    // X-Plane reports 0-based day-of-year; FlightData carries 1-based.
    data_.utcDayOfYear = XPLMGetDatai(localDateDays_) + 1;
  }

  // NAV/COM active + standby frequencies and the transponder, so the glass
  // tracks the live radios (and reflects bezel tuning we wrote back). These are
  // integer datarefs, so read with XPLMGetDatai.
  for (const RadioRef& r : radios_) {
    if (r.active) data_.*(r.activeMember) = XPLMGetDatai(r.active) * kRadioHzToMhz;
    if (r.standby)
      data_.*(r.standbyMember) = XPLMGetDatai(r.standby) * kRadioHzToMhz;
    // Audio volume is a float dataref (0..1).
    if (r.volume) data_.*(r.volumeMember) = XPLMGetDataf(r.volume);
    // NAV Morse-ident audio selection is an int dataref (0/1).
    if (r.identAudio) data_.*(r.identMember) = XPLMGetDatai(r.identAudio) != 0;
  }
  data_.nav1Ident = readNavStationIdent(nav1NavId_, nav1DmeId_);
  data_.nav2Ident = readNavStationIdent(nav2NavId_, nav2DmeId_);
  if (transponderCode_) data_.transponderCode = XPLMGetDatai(transponderCode_);
  if (transponderMode_)
    data_.transponderMode = xpdrModeString(XPLMGetDatai(transponderMode_));

  // Per-instrument failures: each gauge/box red-X's when its X-Plane failure
  // dataref trips (matching the standalone UDP shell). The AHRS feeds attitude
  // + heading; the ADC feeds the air-data tapes; radios/transponder fail on
  // their own.
  data_.attitudeValid = !failedInop(failAttitude_);
  data_.headingValid = !failedInop(failHeading_);
  data_.airspeedValid = !failedInop(failAirspeed_);
  data_.altitudeValid = !failedInop(failAltimeter_);
  data_.verticalSpeedValid = !failedInop(failVerticalSpeed_);
  data_.nav1Valid = !failedInop(failNav1_);
  data_.nav2Valid = !failedInop(failNav2_);
  data_.com1Valid = !failedInop(failCom1_);
  data_.com2Valid = !failedInop(failCom2_);
  data_.transponderValid = !failedInop(failTransponder_);

  // GDU power: the PFD follows the master switch, the MFD the avionics master.
  // Default to powered when a switch dataref is missing so the glass is never
  // stuck dark on airframes that don't expose it.
  data_.masterPowerOn =
      batteryMasterOn_ == nullptr || XPLMGetDatai(batteryMasterOn_) != 0;
  data_.avionicsPowerOn =
      avionicsPowerOn_ == nullptr || XPLMGetDatai(avionicsPowerOn_) != 0;

  // EIS engine/fuel/electrical indicators for the MFD engine strip.
  for (const EisBinding& b : eisBindings_) {
    float raw = 0.0f;
    if (b.arrayIndex < 0) {
      raw = XPLMGetDataf(b.ref);
    } else {
      XPLMGetDatavf(b.ref, &raw, b.arrayIndex, 1);
    }
    data_.eisChannels[b.channel] = raw * b.scale + b.offset;
  }
  syncEisLegacyFields(data_);

  if (routeOverrideSet_ && routeOverride_.empty() &&
      !(directToActive_ && !directTo_.id.empty())) {
    directToActive_ = false;
    directTo_ = {};
  }

  updateMap(dtSeconds);
  weather_.update(dtSeconds);
  syncDisplayBackup(data_, dtSeconds);
}

void DatarefDataSource::resetGpsCouplingState() {
  lastSentGpsCourseDeg_ = -999.0f;
  lastSentGpsHdefDots_ = 999.0f;
  lastPushedCourseDeg_ = -999.0f;
  lastGpsCoupledLegIndex_ = -1;
  lastGpsCoupledToWpt_.clear();
  lastSentGpsNavId_.clear();
  lastSentGpsDmeDistNm_ = -1.0f;
  lastSentGpsHdefNmPerDot_ = -1.0f;
  lastSentHsiSource_ = -1;
  lastNavigatorDirectTo_ = false;
}

void DatarefDataSource::ensureSimCdiSource(CdiSource source) {
  if (hsiSourceSelect_ == nullptr) return;
  const int simVal =
      source == CdiSource::Nav1 ? 0 : source == CdiSource::Nav2 ? 1 : 2;
  if (simVal == lastSentHsiSource_) return;
  XPLMSetDatai(hsiSourceSelect_, simVal);
  lastSentHsiSource_ = simVal;
}

void DatarefDataSource::applyGpsNavigation(FmsNavigator& navigator, bool obsMode,
                                           CdiSource cdiSource, float nmPerDot) {
  float scale = nmPerDot;
  if (scale <= 0.01f && gpsHdef_ != nullptr) {
    scale = XPLMGetDataf(gpsHdef_);
  }
  if (scale <= 0.01f) scale = 0.5f;

  NavigationCallbacks callbacks;
  callbacks.onDirectToCaptured =
      [this](int legIdx) { onNavigatorDirectToCaptured(legIdx); };

  if (routeOverrideSet_ && routeOverride_.empty() &&
      !(directToActive_ && !directTo_.id.empty())) {
    clearNavigationFields(data_);
    navigator.setFlightPlan({});
  } else {
    runNavigationFrame(navigator, map_, data_, obsMode, cdiSource, scale,
                       callbacks);
  }

  // Do not sync the sim FMS destination on automatic leg sequencing: pushing
  // XPLMSetDestinationFMSEntry at each waypoint capture makes X-Plane briefly
  // lose GPS nav validity and drop autopilot NAV mode. Leg sync is reserved for
  // explicit Activate Leg / Direct-To capture (syncSimulatorActiveLeg).

  FmsDebugNavState dbg{};
  dbg.activeLegIndex = data_.fmaActiveLegIndex;
  dbg.planLegCount = static_cast<int>(map_.flightPlan.size());
  std::snprintf(dbg.fmaFrom, sizeof(dbg.fmaFrom), "%s", data_.fmaFromWpt.c_str());
  std::snprintf(dbg.fmaTo, sizeof(dbg.fmaTo), "%s", data_.fmaToWpt.c_str());
  dbg.courseDeg = data_.courseDeg;
  dbg.cdiDots = data_.cdiDeviationDots;
  dbg.gpsOverride = gpsOverrideActive_;
  dbg.obsMode = obsMode;
  dbg.directTo = navigator.directToActive();
  FmsDebugOverlay::updateNavState(dbg);

  ensureSimCdiSource(cdiSource);

  if (data_.fmaToWpt.empty() || obsMode || cdiSource != CdiSource::Gps) {
    if (gpsOverrideActive_ && overrideGps_ != nullptr) {
      XPLMSetDatai(overrideGps_, 0);
      gpsOverrideActive_ = false;
      lastSentGpsCourseDeg_ = -999.0f;
      lastSentGpsHdefDots_ = 999.0f;
      lastSentGpsNavId_.clear();
      lastGpsCoupledLegIndex_ = -1;
      lastGpsCoupledToWpt_.clear();
      FmsDebugOverlay::recordWrite("GPS", "override OFF");
    }
    return;
  }

  // FMS is programmed in-process (programFmsDirectTo / programFmsRoute); native
  // sim GPS drives the autopilot. Lateral override_gps broke AP NAV coupling.
  if (gpsOverrideActive_ && overrideGps_ != nullptr) {
    XPLMSetDatai(overrideGps_, 0);
    gpsOverrideActive_ = false;
    lastSentGpsCourseDeg_ = -999.0f;
    lastSentGpsHdefDots_ = 999.0f;
    lastSentGpsNavId_.clear();
    lastGpsCoupledLegIndex_ = -1;
    lastGpsCoupledToWpt_.clear();
    FmsDebugOverlay::recordWrite("GPS", "override OFF (FMS native)");
  }
}

void DatarefDataSource::syncSimulatorActiveLeg(int legIndex) {
  const std::vector<MapLeg>& plan =
      routeOverrideSet_ ? routeOverride_ : map_.flightPlan;
  if (plan.empty()) return;
  if (directToActive_) clearDirectTo();
  programFmsActiveLeg(legIndex, plan);
  lastSyncedFmsLegIndex_ = legIndex;
}

void DatarefDataSource::syncWeatherRadar(const MfdController& ui) {
  weather_.syncFromController(ui);
}

void DatarefDataSource::syncMapRangeFromSim(MfdController& ui) {
  if (efisMapRangeNm_ == nullptr) return;
  const float simNm = XPLMGetDataf(efisMapRangeNm_);
  if (simNm <= 0.0f) return;
  if (lastPushedMapRangeNm_ >= 0.0f &&
      std::fabs(simNm - lastPushedMapRangeNm_) < 0.01f) {
    return;
  }
  const float uiNm = ui.rangeNm();
  if (std::fabs(std::log(simNm) - std::log(uiNm)) > 0.02f) {
    ui.setRangeFromNm(simNm);
    chartRangeNm_ = ui.rangeNm();
    mapPanDirty_ = true;
  }
}

void DatarefDataSource::pushMapRangeToSim(float rangeNm) {
  if (efisMapRangeNm_ == nullptr || rangeNm <= 0.0f) return;
  if (lastPushedMapRangeNm_ >= 0.0f &&
      std::fabs(rangeNm - lastPushedMapRangeNm_) < 0.001f) {
    return;
  }
  XPLMSetDataf(efisMapRangeNm_, rangeNm);
  lastPushedMapRangeNm_ = rangeNm;
}

bool DatarefDataSource::stepMapRangeFromSim(int direction,
                                            MfdController* ui) {
  if (direction == 0) return false;
  float simNm = 0.0f;
  if (efisMapRangeNm_ != nullptr) {
    simNm = XPLMGetDataf(efisMapRangeNm_);
  }
  if (simNm <= 0.0f && ui != nullptr) {
    simNm = ui->rangeNm();
  }
  if (simNm <= 0.0f) {
    simNm = mapRangeNmAt(kMapRangeDefaultIndex);
  }
  int idx = mapRangeIndexForNm(simNm);
  if (direction > 0) {
    idx = std::min(kMapRangeLadderCount - 1, idx + 1);
  } else {
    idx = std::max(0, idx - 1);
  }
  const float newNm = mapRangeNmAt(idx);
  pushMapRangeToSim(newNm);
  if (ui != nullptr) {
    ui->setRangeFromNm(newNm);
  }
  chartRangeNm_ = newNm;
  mapPanDirty_ = true;
  return newNm != simNm;
}

void DatarefDataSource::tuneRadioStandby(RadioUnit unit, float standbyMhz) {
  const RadioRef& r = radios_[static_cast<int>(unit)];
  if (r.standby) {
    XPLMSetDatai(r.standby,
                 static_cast<int>(std::lround(standbyMhz * kMhzToRadioHz)));
  }
  data_.*(r.standbyMember) = standbyMhz;
}

void DatarefDataSource::transferRadio(RadioUnit unit) {
  const RadioRef& r = radios_[static_cast<int>(unit)];
  const float active = data_.*(r.activeMember);
  const float standby = data_.*(r.standbyMember);
  if (r.active) {
    XPLMSetDatai(r.active,
                 static_cast<int>(std::lround(standby * kMhzToRadioHz)));
  }
  if (r.standby) {
    XPLMSetDatai(r.standby,
                 static_cast<int>(std::lround(active * kMhzToRadioHz)));
  }
  data_.*(r.activeMember) = standby;
  data_.*(r.standbyMember) = active;
}

void DatarefDataSource::setRadioVolume(RadioUnit unit, float volume) {
  const RadioRef& r = radios_[static_cast<int>(unit)];
  if (r.volume) XPLMSetDataf(r.volume, volume);
  data_.*(r.volumeMember) = volume;
}

void DatarefDataSource::setNavIdent(RadioUnit unit, bool on) {
  const RadioRef& r = radios_[static_cast<int>(unit)];
  if (r.identAudio) XPLMSetDatai(r.identAudio, on ? 1 : 0);
  if (r.identMember) data_.*(r.identMember) = on;
}

void DatarefDataSource::setTransponderCode(int code) {
  if (transponderCode_) XPLMSetDatai(transponderCode_, code);
  data_.transponderCode = code;
}

void DatarefDataSource::setTransponderMode(int mode) {
  if (transponderMode_) XPLMSetDatai(transponderMode_, mode);
  data_.transponderMode = xpdrModeString(mode);
}

void DatarefDataSource::setLocalFlightPlan(std::vector<MapLeg> route) {
  routeOverride_ = std::move(route);
  routeOverrideSet_ = true;
  map_.flightPlan = routeOverride_;
}

void DatarefDataSource::setRouteOverride(std::vector<MapLeg> route,
                                           bool programSimulator) {
  routeOverride_ = std::move(route);
  routeOverrideSet_ = true;
  map_.flightPlan = routeOverride_;
  if (routeOverride_.empty()) setDirectTo({});
  lastSyncedFmsLegIndex_ = -1;
  if (programSimulator) programFmsRoute(routeOverride_);
}

void DatarefDataSource::clearRouteOverride() {
  routeOverride_ = {};
  routeOverrideSet_ = false;
}

void DatarefDataSource::setDirectTo(MapLeg target, bool flyHold) {
  directTo_ = std::move(target);
  directToActive_ = !directTo_.id.empty();
  directToHold_ = flyHold && directToActive_;
  if (directToActive_) {
    // Each Direct-To activation snapshots present position as the course origin.
    if (map_.positionValid) {
      directToOriginLat_ = map_.ownshipLat;
      directToOriginLon_ = map_.ownshipLon;
      directToOriginValid_ = true;
      directToOriginPending_ = false;
    } else {
      directToOriginValid_ = false;
      directToOriginPending_ = true;
    }
    if (!map_.flightPlan.empty()) {
      applyInPlanDirectToRouteSlice(map_, map_.flightPlan, directTo_);
    }
    resetGpsCouplingState();
    programFmsDirectTo(true, directTo_);
  } else {
    directToOriginPending_ = false;
    directToOriginValid_ = false;
    directToHold_ = false;
    clearFlightPlanRouteSlice(map_);
    programFmsDirectTo(false, directTo_);
  }
}

void DatarefDataSource::clearDirectTo() {
  directToActive_ = false;
  directToHold_ = false;
  directTo_ = {};
  directToOriginPending_ = false;
  directToOriginValid_ = false;
  map_.directToActive = false;
  map_.directToHold = false;
  map_.directTo = {};
  map_.directToOriginValid = false;
  programFmsDirectTo(false, directTo_);
}

void DatarefDataSource::onNavigatorDirectToCaptured(int activeLegIndex) {
  clearDirectTo();
  if (activeLegIndex >= 0) {
    syncSimulatorActiveLeg(activeLegIndex);
  }
  resetGpsCouplingState();
}

void DatarefDataSource::buildNavCache() {
  navCacheBuilt_ = true;  // set first so a navaid-less DB isn't rescanned

  // Walk the database one nav-aid type at a time. Like-typed nav-aids are
  // grouped contiguously, so iterating from the first to the last of each type
  // visits exactly that type. Fixes are intentionally excluded: there are far
  // too many to scan/draw usefully at inset-map scale.
  struct Kind {
    XPLMNavType xpType;
    MapFeatureType mapType;
  };
  const Kind kinds[] = {
      {xplm_Nav_Airport, MapFeatureType::Airport},
      {xplm_Nav_VOR, MapFeatureType::Vor},
      {xplm_Nav_NDB, MapFeatureType::Ndb},
  };

  for (const Kind& kind : kinds) {
    XPLMNavRef ref = XPLMFindFirstNavAidOfType(kind.xpType);
    if (ref == XPLM_NAV_NOT_FOUND) continue;
    const XPLMNavRef last = XPLMFindLastNavAidOfType(kind.xpType);
    while (ref != XPLM_NAV_NOT_FOUND) {
      XPLMNavType type = xplm_Nav_Unknown;
      float lat = 0.0f;
      float lon = 0.0f;
      int freq = 0;
      char id[kNavIdBufferSize] = {};
      char name[256] = {};
      XPLMGetNavAidInfo(ref, &type, &lat, &lon, nullptr, &freq, nullptr, id,
                        name, nullptr);
      id[sizeof(id) - 1] = '\0';
      name[sizeof(name) - 1] = '\0';
      if (type == kind.xpType) {
        MapFeature f;
        f.type = kind.mapType;
        f.lat = static_cast<double>(lat);
        f.lon = static_cast<double>(lon);
        f.id = id;
        f.name = name;
        if (kind.mapType == MapFeatureType::Vor ||
            kind.mapType == MapFeatureType::Ndb) {
          f.navaidType = splitNavaidTypeSuffix(f.name);
          if (f.navaidType.empty()) {
            f.navaidType = kind.mapType == MapFeatureType::Vor ? "VOR" : "NDB";
          }
          // XPLM frequencies: VORs in 10 kHz units (11390 == 113.90 MHz),
          // NDBs directly in kHz.
          f.frequency = kind.mapType == MapFeatureType::Vor
                            ? static_cast<float>(freq) / 100.0f
                            : static_cast<float>(freq);
        }
        navCache_.push_back(std::move(f));
      }
      if (ref == last) break;
      ref = XPLMGetNextNavAid(ref);
    }
  }
}

void DatarefDataSource::updateMap(double dtSeconds) {
  map_.rangeNm = kMapRangeNm;
  map_.terrain = terrain_.get();
  map_.weather = &weather_;

  // Ownship position at full double precision (in-process, no RREF truncation).
  if (latitude_ && longitude_) {
    map_.ownshipLat = XPLMGetDatad(latitude_);
    map_.ownshipLon = XPLMGetDatad(longitude_);
    map_.positionValid = true;
  }

  // Live datalink NEXRAD overlay centered on the aircraft (real ground radar,
  // independent of whether this airframe has an onboard radar). Pointed at the
  // source unconditionally so the overlay is real datalink weather (or nothing
  // until tiles load / when offline), never the onboard radar texture; the
  // latter still drives the dedicated Weather Radar page via map_.weather.
  if (map_.positionValid) {
    nexrad_.setCenter(map_.ownshipLat, map_.ownshipLon);
  }
  nexrad_.advance(dtSeconds);
  map_.nexrad = &nexrad_;

  map_.directToActive = directToActive_;
  map_.directToHold = directToHold_;
  map_.directTo = directTo_;
  map_.directToOriginValid = directToOriginValid_;
  map_.directToOriginLat = directToOriginLat_;
  map_.directToOriginLon = directToOriginLon_;
  if (directToOriginPending_ && map_.positionValid) {
    directToOriginLat_ = map_.ownshipLat;
    directToOriginLon_ = map_.ownshipLon;
    directToOriginValid_ = true;
    directToOriginPending_ = false;
    map_.directToOriginValid = true;
    map_.directToOriginLat = directToOriginLat_;
    map_.directToOriginLon = directToOriginLon_;
  }

  // Active flight-plan route: session-only in the plugin. The NXi FPL editor and
  // inset map show what the pilot built this X-Plane session (routeOverride_),
  // not X-Plane's persisted FMS file from a previous launch. The sim FMS is
  // programmed when the route is complete enough for navigation, but a fresh
  // X-Plane start should present a blank FPL page like a cold avionics boot.
  map_.flightPlan.clear();
  if (routeOverrideSet_) {
    map_.flightPlan = routeOverride_;
  }

  // Nearby navaids/airports: built once from the nav database, then range-
  // filtered around ownship on a throttled timer (the cache is static, only the
  // filtering follows the aircraft). The filtering and the apt-geometry /
  // airspace scans run on a background worker (see submitMapQuery); here we only
  // adopt finished results and decide when to kick off the next scan.
  if (!navCacheBuilt_) buildNavCache();
  adoptMapQueryResult();

  if (landData_ && landData_->loaded() && !landEverLoaded_) {
    landEverLoaded_ = true;
    sinceMapRebuildSeconds_ = kMapRebuildIntervalSeconds;
    XPLMDebugString("G1000 NXi: land chart data ready\n");
  }

  sinceMapRebuildSeconds_ += dtSeconds;
  if (aptMapDirty_.load(std::memory_order_acquire)) {
    sinceMapRebuildSeconds_ = kMapRebuildIntervalSeconds;
  }
  // When the MFD Map Pointer is active the queries follow the pointer instead
  // of ownship so the panned-to area has data (see setMapPanCenter()).
  const double queryLat = mapPanActive_ ? mapPanLat_ : map_.ownshipLat;
  const double queryLon = mapPanActive_ ? mapPanLon_ : map_.ownshipLon;
  if (map_.positionValid &&
      (map_.features.empty() || mapPanDirty_ ||
       sinceMapRebuildSeconds_ >= kMapRebuildIntervalSeconds)) {
    if (submitMapQuery(queryLat, queryLon)) {
      // A scan is now in flight; don't re-submit every frame while we wait.
      mapPanDirty_ = false;
      sinceMapRebuildSeconds_ = 0.0;
    }
  }
  rebuildInsetMap();
}

void DatarefDataSource::startMapQueryWorker() {
  if (!mapQueryThread_.joinable()) {
    mapQueryThread_ = std::thread([this] { mapQueryWorkerMain(); });
  }
}

void DatarefDataSource::stopMapQueryWorker() {
  {
    std::lock_guard<std::mutex> lock(mapQueryMu_);
    mapQueryStop_ = true;
  }
  mapQueryCv_.notify_all();
  if (mapQueryThread_.joinable()) mapQueryThread_.join();
}

bool DatarefDataSource::submitMapQuery(double lat, double lon) {
  startMapQueryWorker();
  std::lock_guard<std::mutex> lock(mapQueryMu_);
  // Only one scan at a time; a still-moving ownship simply triggers a fresh
  // scan once the in-flight one is adopted.
  if (mapQueryPhase_ != MapQueryPhase::Idle) return false;
  mapQueryReqLat_ = lat;
  mapQueryReqLon_ = lon;
  mapQueryReqApt_ = aptDatLoaded_.load(std::memory_order_acquire);
  mapQueryReqAirspace_ = airspaceLoaded_.load(std::memory_order_acquire);
  mapQueryReqObstacles_ =
      obstacles_ != nullptr && obstacles_->loaded();
  mapQueryReqLand_ = landData_ != nullptr && landData_->loaded();
  mapQueryLandRangeNm_ = chartRangeNm_;
  mapQueryLandViewHalfExtentNm_ = mapViewHalfExtentNm_;
  mapQueryPhase_ = MapQueryPhase::Running;
  mapQueryCv_.notify_one();
  return true;
}

bool DatarefDataSource::adoptMapQueryResult() {
  MapQueryResult result;
  {
    std::lock_guard<std::mutex> lock(mapQueryMu_);
    if (mapQueryPhase_ != MapQueryPhase::Done) return false;
    result = std::move(mapQueryResult_);
    mapQueryResult_ = MapQueryResult{};
    mapQueryPhase_ = MapQueryPhase::Idle;
  }

  map_.features = std::move(result.features);

  if (result.aptGeometryIncluded) {
    const std::size_t prevRunways = map_.runways.size();
    const std::size_t prevTaxiways = map_.taxiways.size();
    const std::size_t prevTaxiwayLabels = map_.taxiwayLabels.size();
    map_.runways = std::move(result.runways);
    map_.taxiways = std::move(result.taxiways);
    map_.taxiwayLabels = std::move(result.taxiwayLabels);
    if (aptMapDirty_.exchange(false, std::memory_order_acq_rel) ||
        map_.runways.size() != prevRunways ||
        map_.taxiways.size() != prevTaxiways ||
        map_.taxiwayLabels.size() != prevTaxiwayLabels) {
      ++map_.geometryEpoch;
      char msg[160];
      std::snprintf(
          msg, sizeof(msg),
          "G1000 NXi: airport diagram ready (%zu runways, %zu taxiway polys "
          "near aircraft)\n",
          map_.runways.size(), map_.taxiways.size());
      XPLMDebugString(msg);
    }
  }

  if (result.airspaceIncluded) {
    const std::size_t prevAirspaces = map_.airspaces.size();
    map_.airspaces = std::move(result.airspaces);
    if (map_.airspaces.size() != prevAirspaces) {
      ++map_.geometryEpoch;
    }
  }

  if (result.obstaclesIncluded) {
    map_.obstacles = std::move(result.obstacles);
  }

  if (result.landIncluded) {
    map_.landLines = std::move(result.landLines);
    map_.cities = std::move(result.cities);
  }
  return true;
}

// Background worker: pure reads of the immutable-after-load nav / apt.dat /
// airspace / obstacle caches plus the query center handed in by submitMapQuery.
// Never touches map_ or the sim/XPLM API.
void DatarefDataSource::mapQueryWorkerMain() {
  for (;;) {
    double lat = 0.0;
    double lon = 0.0;
    bool wantApt = false;
    bool wantAirspace = false;
    bool wantObstacles = false;
    bool wantLand = false;
    float landRangeNm = 0.0f;
    float landViewHalfExtentNm = 0.0f;
    const ObstacleStore* obstacles = nullptr;
    const LandDataStore* land = nullptr;
    {
      std::unique_lock<std::mutex> lock(mapQueryMu_);
      mapQueryCv_.wait(lock, [this] {
        return mapQueryStop_ || mapQueryPhase_ == MapQueryPhase::Running;
      });
      if (mapQueryStop_) return;
      lat = mapQueryReqLat_;
      lon = mapQueryReqLon_;
      wantApt = mapQueryReqApt_;
      wantAirspace = mapQueryReqAirspace_;
      wantObstacles = mapQueryReqObstacles_;
      wantLand = mapQueryReqLand_;
      landRangeNm = mapQueryLandRangeNm_;
      landViewHalfExtentNm = mapQueryLandViewHalfExtentNm_;
      obstacles = obstacles_;
      land = landData_.get();
    }

    MapQueryResult res;
    res.features =
        filterNearby(navCache_, lat, lon, kMapQueryRangeNm, kMaxMapFeatures);
    if (wantApt) {
      for (MapFeature& f : res.features) {
        enrichAirportFromMeta(f, aptMetaByIcao_);
      }
      res.runways = nearbyRunwaysFromCells(runwayCells_, lat, lon,
                                           kRunwayQueryRangeNm, kMaxMapRunways);
      res.taxiways = nearbyPavementFromCells(
          pavementCells_, lat, lon, kTaxiwayQueryRangeNm, kMaxMapTaxiways);
      res.taxiwayLabels = nearbyTaxiwayLabelsFromCells(
          taxiwayLabelCells_, lat, lon, kTaxiwayLabelQueryRangeNm,
          kMaxMapTaxiwayLabels);
      res.aptGeometryIncluded = true;
    }
    if (wantAirspace) {
      res.airspaces = airspacesNear(airspaceCache_, lat, lon, kMapQueryRangeNm,
                                    kMaxMapAirspaces);
      res.airspaceIncluded = true;
    }
    if (wantObstacles && obstacles != nullptr) {
      res.obstacles = obstacles->nearby(lat, lon, kObstacleQueryRangeNm,
                                        kMaxMapObstacles);
      res.obstaclesIncluded = true;
    }
    if (wantLand && land != nullptr) {
      res.landLines =
          land->nearbyLines(lat, lon, landRangeNm, kMaxMapLandLines,
                            landViewHalfExtentNm);
      res.cities = land->nearbyCities(lat, lon, landRangeNm, kMaxMapCities,
                                      landViewHalfExtentNm);
      res.landIncluded = true;
    }

    {
      std::lock_guard<std::mutex> lock(mapQueryMu_);
      if (mapQueryStop_) return;
      mapQueryResult_ = std::move(res);
      mapQueryPhase_ = MapQueryPhase::Done;
    }
  }
}

void DatarefDataSource::setMapPanCenter(bool active, double lat, double lon) {
  if (active != mapPanActive_ ||
      (active && (lat != mapPanLat_ || lon != mapPanLon_))) {
    mapPanDirty_ = true;  // pointer toggled/moved: re-scan around the new center
  }
  mapPanActive_ = active;
  mapPanLat_ = lat;
  mapPanLon_ = lon;
}

void DatarefDataSource::setInsetMapQuery(bool active, double lat, double lon,
                                          float rangeNm, float viewHalfExtentNm,
                                          const std::string& targetIdent) {
  if (active != insetMapActive_ ||
      (active && (lat != insetMapLat_ || lon != insetMapLon_ ||
                  rangeNm != insetMapRangeNm_ ||
                  viewHalfExtentNm != insetMapHalfExtentNm_ ||
                  targetIdent != insetMapTargetIdent_))) {
    insetMapDirty_ = true;
  }
  insetMapActive_ = active;
  insetMapLat_ = lat;
  insetMapLon_ = lon;
  insetMapRangeNm_ = rangeNm;
  insetMapHalfExtentNm_ = viewHalfExtentNm;
  insetMapTargetIdent_ = targetIdent;
}

void DatarefDataSource::rebuildInsetMap() {
  if (!insetMapActive_) {
    if (map_.insetMapActive) {
      map_.insetMapActive = false;
      map_.insetLandLines.clear();
      map_.insetCities.clear();
      map_.insetFeatures.clear();
    }
    return;
  }
  if (!insetMapDirty_ && !map_.insetLandLines.empty()) return;
  insetMapDirty_ = false;
  map_.insetMapActive = true;
  map_.insetMapLat = insetMapLat_;
  map_.insetMapLon = insetMapLon_;
  if (!navCacheBuilt_) return;
  const float featRange = std::max(insetMapRangeNm_, kMapQueryRangeNm * 0.25f);
  map_.insetFeatures =
      filterNearby(navCache_, insetMapLat_, insetMapLon_, featRange,
                   kMaxMapFeatures);
  for (MapFeature& f : map_.insetFeatures) {
    enrichAirportFromMeta(f, aptMetaByIcao_);
  }
  if (!insetMapTargetIdent_.empty()) {
    bool found = false;
    for (const MapFeature& f : map_.insetFeatures) {
      if (f.id == insetMapTargetIdent_) {
        found = true;
        break;
      }
    }
    if (!found) {
      std::vector<MapFeature> exact =
          lookupNavIdentNear(insetMapTargetIdent_, insetMapLat_, insetMapLon_, 1);
      if (!exact.empty()) map_.insetFeatures.push_back(exact.front());
    }
  }
  if (landData_ != nullptr && landData_->loaded()) {
    map_.insetLandLines =
        landData_->nearbyLines(insetMapLat_, insetMapLon_, insetMapRangeNm_,
                               kMaxMapLandLines, insetMapHalfExtentNm_);
    map_.insetCities =
        landData_->nearbyCities(insetMapLat_, insetMapLon_, insetMapRangeNm_,
                                kMaxMapCities, insetMapHalfExtentNm_);
  }
}

void DatarefDataSource::setChartRangeNm(float rangeNm) {
  if (chartRangeNm_ != rangeNm) {
    chartRangeNm_ = rangeNm;
    mapPanDirty_ = true;
  }
  pushMapRangeToSim(rangeNm);
}

void DatarefDataSource::setMapViewHalfExtentNm(float halfExtentNm) {
  if (mapViewHalfExtentNm_ != halfExtentNm) {
    mapViewHalfExtentNm_ = halfExtentNm;
    mapPanDirty_ = true;
  }
}

std::vector<MapAirportFrequency> DatarefDataSource::airportFrequencies(
    const std::string& icao) const {
  const auto it = aptMetaByIcao_.find(icao);
  if (it == aptMetaByIcao_.end()) return {};
  return it->second.frequencies;
}

std::vector<MapFeature> DatarefDataSource::lookupNavIdent(
    const std::string& ident, std::size_t maxCount) const {
  return lookupNavIdentNear(ident, 0.0, 0.0, maxCount);
}

std::vector<MapFeature> DatarefDataSource::lookupNavIdentNear(
    const std::string& ident, double refLat, double refLon,
    std::size_t maxCount) const {
  if (ident.empty() || maxCount == 0) return {};
  std::vector<MapFeature> out;
  out.reserve(maxCount);
  for (const MapFeature& feature : navCache_) {
    if (feature.id != ident) continue;
    out.push_back(feature);
    if (out.size() >= maxCount) break;
  }
  if (out.size() >= maxCount) return out;

  auto navIdentEquals = [](const char* found, const std::string& expected) {
    if (found == nullptr || expected.empty()) return false;
    std::string a(found);
    while (!a.empty() && std::isspace(static_cast<unsigned char>(a.back()))) {
      a.pop_back();
    }
    if (a.size() != expected.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
      if (std::toupper(static_cast<unsigned char>(a[i])) !=
          std::toupper(static_cast<unsigned char>(expected[i]))) {
        return false;
      }
    }
    return true;
  };

  auto mapNavType = [](XPLMNavType type) -> MapFeatureType {
    switch (type) {
      case xplm_Nav_Airport:
        return MapFeatureType::Airport;
      case xplm_Nav_VOR:
        return MapFeatureType::Vor;
      case xplm_Nav_NDB:
        return MapFeatureType::Ndb;
      case xplm_Nav_Fix:
        return MapFeatureType::Fix;
      default:
        return MapFeatureType::Waypoint;
    }
  };

  auto tryNavAid = [&](XPLMNavType type, float* latInOut, float* lonInOut) {
    const XPLMNavRef ref =
        XPLMFindNavAid(nullptr, ident.c_str(), latInOut, lonInOut, nullptr, type);
    if (ref == XPLM_NAV_NOT_FOUND) return;

    XPLMNavType outType = xplm_Nav_Unknown;
    float lat = 0.0f;
    float lon = 0.0f;
    char foundId[32] = {};
    XPLMGetNavAidInfo(ref, &outType, &lat, &lon, nullptr, nullptr, nullptr,
                      foundId, nullptr, nullptr);
    foundId[sizeof(foundId) - 1] = '\0';
    if (!navIdentEquals(foundId, ident)) return;

    for (const MapFeature& existing : out) {
      if (existing.lat == static_cast<double>(lat) &&
          existing.lon == static_cast<double>(lon)) {
        return;
      }
    }

    MapFeature feature;
    feature.id = ident;
    feature.lat = static_cast<double>(lat);
    feature.lon = static_cast<double>(lon);
    feature.type = mapNavType(outType);
    out.push_back(std::move(feature));
  };

  const bool haveRef =
      std::isfinite(refLat) && std::isfinite(refLon) &&
      (refLat != 0.0 || refLon != 0.0);
  float latHint = static_cast<float>(refLat);
  float lonHint = static_cast<float>(refLon);
  float* latPtr = haveRef ? &latHint : nullptr;
  float* lonPtr = haveRef ? &lonHint : nullptr;

  const XPLMNavType types[] = {xplm_Nav_Fix, xplm_Nav_VOR, xplm_Nav_NDB,
                               xplm_Nav_Airport};
  for (XPLMNavType type : types) {
    if (out.size() >= maxCount) break;
    tryNavAid(type, latPtr, lonPtr);
  }
  return out;
}

std::string DatarefDataSource::firstNavIdentWithPrefix(
    const std::string& prefix) const {
  if (prefix.empty()) return {};
  std::string best;
  for (const MapFeature& feature : navCache_) {
    if (feature.id.size() < prefix.size()) continue;
    if (feature.id.compare(0, prefix.size(), prefix) != 0) continue;
    if (best.empty() || feature.id < best) best = feature.id;
  }
  return best;
}

}  // namespace avionics
