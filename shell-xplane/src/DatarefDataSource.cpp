#include "DatarefDataSource.h"
#include "avionics/EisLegacy.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include "avionics/AptDatGeometryCache.h"
#include "avionics/AptDatParser.h"
#include "avionics/Datarefs.h"
#include "avionics/OpenAirParser.h"

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

// Return the features from `src` within rangeNm of (lat, lon), nearest first,
// capped at maxCount. Shared shape with the standalone NavDataStore::nearby.
std::vector<MapFeature> filterNearby(const std::vector<MapFeature>& src,
                                     double lat, double lon, float rangeNm,
                                     std::size_t maxCount) {
  std::vector<MapFeature> result;
  if (maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  struct Scored {
    MapFeature feature;
    double distSq;
  };
  std::vector<Scored> scored;
  for (const MapFeature& f : src) {
    if (std::fabs(f.lat - lat) > dLat) continue;
    if (std::fabs(f.lon - lon) > dLon) continue;
    const double north = (f.lat - lat) * kNmPerDeg;
    const double east = (f.lon - lon) * kNmPerDeg * cosLat;
    scored.push_back({f, north * north + east * east});
  }
  std::sort(scored.begin(), scored.end(),
            [](const Scored& a, const Scored& b) { return a.distSq < b.distSq; });

  // Reserve airports and navaids before fixes (mirrors NavDataStore::nearby) so
  // a wide MFD MAP view keeps the far airports instead of letting the dense fix
  // class fill the whole budget with the nearest cluster. The map renderer then
  // declutters airports per-size against the Map Setup "Aviation" ranges.
  constexpr std::size_t kMaxAirports = 200;
  constexpr std::size_t kMaxNavaids = 100;
  result.reserve(std::min(scored.size(), maxCount));
  std::size_t airports = 0;
  std::size_t navaids = 0;
  for (const Scored& s : scored) {
    if (result.size() >= maxCount) break;
    const MapFeatureType t = s.feature.type;
    if (t == MapFeatureType::Airport) {
      if (airports >= kMaxAirports) continue;
      ++airports;
    } else if (t == MapFeatureType::Vor || t == MapFeatureType::Ndb) {
      if (navaids >= kMaxNavaids) continue;
      ++navaids;
    } else {
      continue;  // fixes/waypoints fill the remaining budget below
    }
    result.push_back(s.feature);
  }
  for (const Scored& s : scored) {
    if (result.size() >= maxCount) break;
    const MapFeatureType t = s.feature.type;
    if (t == MapFeatureType::Fix || t == MapFeatureType::Waypoint) {
      result.push_back(s.feature);
    }
  }
  return result;
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
  verticalSpeed_ = XPLMFindDataRef(datarefs::kVerticalSpeedFpm);
  slip_ = XPLMFindDataRef(datarefs::kSlipDeg);

  latitude_ = XPLMFindDataRef(datarefs::kLatitudeDeg);
  longitude_ = XPLMFindDataRef(datarefs::kLongitudeDeg);

  zuluTimeSec_ = XPLMFindDataRef(datarefs::kZuluTimeSec);
  localDateDays_ = XPLMFindDataRef(datarefs::kLocalDateDays);

  gpsDistance_ = XPLMFindDataRef(datarefs::kGpsDistanceNm);
  gpsBearing_ = XPLMFindDataRef(datarefs::kGpsBearingDegMag);
  gpsNavId_ = XPLMFindDataRef(datarefs::kGpsNavId);
  nav1NavId_ = XPLMFindDataRef(datarefs::kNav1NavId);
  nav2NavId_ = XPLMFindDataRef(datarefs::kNav2NavId);
  nav1DmeId_ = XPLMFindDataRef(datarefs::kNav1DmeId);
  nav2DmeId_ = XPLMFindDataRef(datarefs::kNav2DmeId);

  radios_[static_cast<int>(RadioUnit::Nav1)] = {
      XPLMFindDataRef(datarefs::kNav1FrequencyHz),
      XPLMFindDataRef(datarefs::kNav1StandbyFrequencyHz),
      &FlightData::nav1ActiveMhz, &FlightData::nav1StandbyMhz};
  radios_[static_cast<int>(RadioUnit::Nav2)] = {
      XPLMFindDataRef(datarefs::kNav2FrequencyHz),
      XPLMFindDataRef(datarefs::kNav2StandbyFrequencyHz),
      &FlightData::nav2ActiveMhz, &FlightData::nav2StandbyMhz};
  radios_[static_cast<int>(RadioUnit::Com1)] = {
      XPLMFindDataRef(datarefs::kCom1FrequencyHz),
      XPLMFindDataRef(datarefs::kCom1StandbyFrequencyHz),
      &FlightData::com1ActiveMhz, &FlightData::com1StandbyMhz};
  radios_[static_cast<int>(RadioUnit::Com2)] = {
      XPLMFindDataRef(datarefs::kCom2FrequencyHz),
      XPLMFindDataRef(datarefs::kCom2StandbyFrequencyHz),
      &FlightData::com2ActiveMhz, &FlightData::com2StandbyMhz};

  transponderCode_ = XPLMFindDataRef(datarefs::kTransponderCode);
  transponderMode_ = XPLMFindDataRef(datarefs::kTransponderMode);
  acfRelativePath_ = XPLMFindDataRef("sim/aircraft/view/acf_relative_path");

  rebuildEisBindings();
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

void DatarefDataSource::updateAircraftEisPath() {
  if (acfRelativePath_ == nullptr) return;

  char buf[1024] = {};
  const int n = XPLMGetDatab(acfRelativePath_, buf, 0, static_cast<int>(sizeof(buf)) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  const std::string acfPath(buf);
  if (acfPath == lastAircraftAcfPath_) return;
  lastAircraftAcfPath_ = acfPath;

  // Re-probe the radar fit for the newly loaded airframe.
  weather_.resetEquipment();

  if (eisSource_ != nullptr) {
    eisSource_->setAircraftAcfRelativePath(acfPath);
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
  updateAircraftEisPath();
  if (eisSource_ != nullptr) {
    eisSource_->refreshIfChanged();
    if (eisSource_->ready() &&
        eisBindings_.size() != eisSource_->layout().bindings.size()) {
      rebuildEisBindings();
    }
  }

  if (airspeed_) data_.airspeedKts = XPLMGetDataf(airspeed_);
  if (altitude_) data_.altitudeFt = XPLMGetDataf(altitude_);
  if (heading_) data_.headingDeg = XPLMGetDataf(heading_);
  if (pitch_) data_.pitchDeg = XPLMGetDataf(pitch_);
  if (roll_) data_.rollDeg = XPLMGetDataf(roll_);
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
  }
  data_.nav1Ident = readNavStationIdent(nav1NavId_, nav1DmeId_);
  data_.nav2Ident = readNavStationIdent(nav2NavId_, nav2DmeId_);
  if (transponderCode_) data_.transponderCode = XPLMGetDatai(transponderCode_);
  if (transponderMode_)
    data_.transponderMode = xpdrModeString(XPLMGetDatai(transponderMode_));

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

  // Active flight-plan leg (FROM -> TO). The entry the FMS is flying toward is
  // the TO waypoint; the one before it is FROM (e.g. KFMY -> KLAL).
  const int fmsCount = XPLMCountFMSEntries();
  if (fmsCount > 0) {
    int dest = XPLMGetDestinationFMSEntry();
    if (dest < 0) dest = 0;
    if (dest >= fmsCount) dest = fmsCount - 1;
    data_.fmaToWpt = fmsEntryId(dest);
    data_.fmaFromWpt = (dest > 0) ? fmsEntryId(dest - 1) : std::string();
  } else if (gpsNavId_) {
    // No flight plan entered: fall back to the active GPS destination id (a
    // null-terminated byte string), rendering as a direct-to "->KXXX".
    char buf[32] = {};
    int n = XPLMGetDatab(gpsNavId_, buf, 0, static_cast<int>(sizeof(buf)) - 1);
    if (n < 0) n = 0;
    buf[n] = '\0';
    data_.fmaToWpt = buf;
    data_.fmaFromWpt.clear();
  }

  updateMap(dtSeconds);
  weather_.update(dtSeconds);
}

void DatarefDataSource::syncWeatherRadar(const MfdController& ui) {
  weather_.syncFromController(ui);
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

void DatarefDataSource::setTransponderCode(int code) {
  if (transponderCode_) XPLMSetDatai(transponderCode_, code);
  data_.transponderCode = code;
}

void DatarefDataSource::setTransponderMode(int mode) {
  if (transponderMode_) XPLMSetDatai(transponderMode_, mode);
  data_.transponderMode = xpdrModeString(mode);
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

  // Active flight-plan route: every FMS entry as a lat/lon leg with its id. The
  // PFD inset and the future MFD MAP page both render this via the shared
  // MapView, so the route is built once here regardless of which page is up.
  map_.flightPlan.clear();
  const int count = XPLMCountFMSEntries();
  map_.flightPlan.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const FmsEntry entry = fmsEntry(i);
    // Skip empty / unpopulated entries (a 0/0 fix would draw a spurious leg).
    if (entry.lat == 0.0f && entry.lon == 0.0f) continue;
    map_.flightPlan.push_back(
        {static_cast<double>(entry.lat), static_cast<double>(entry.lon),
         entry.id});
  }

  // Nearby navaids/airports: built once from the nav database, then range-
  // filtered around ownship on a throttled timer (the cache is static, only the
  // filtering follows the aircraft).
  if (!navCacheBuilt_) buildNavCache();
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
    map_.features = filterNearby(navCache_, queryLat, queryLon,
                                 kMapQueryRangeNm, kMaxMapFeatures);
    if (aptDatLoaded_.load(std::memory_order_acquire)) {
      for (MapFeature& f : map_.features) {
        enrichAirportFromMeta(f, aptMetaByIcao_);
      }
      const std::size_t prevRunways = map_.runways.size();
      const std::size_t prevTaxiways = map_.taxiways.size();
      const std::size_t prevTaxiwayLabels = map_.taxiwayLabels.size();
      map_.runways = nearbyRunwaysFromCells(runwayCells_, queryLat, queryLon,
                                            kRunwayQueryRangeNm, kMaxMapRunways);
      map_.taxiways = nearbyPavementFromCells(
          pavementCells_, queryLat, queryLon, kTaxiwayQueryRangeNm,
          kMaxMapTaxiways);
      map_.taxiwayLabels = nearbyTaxiwayLabelsFromCells(
          taxiwayLabelCells_, queryLat, queryLon, kTaxiwayLabelQueryRangeNm,
          kMaxMapTaxiwayLabels);
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
    // Airspace boundaries from the (background-parsed) OpenAir file, filtered to
    // the same neighborhood as the features.
    if (airspaceLoaded_.load(std::memory_order_acquire)) {
      const std::size_t prevAirspaces = map_.airspaces.size();
      map_.airspaces = airspacesNear(airspaceCache_, queryLat, queryLon,
                                     kMapQueryRangeNm, kMaxMapAirspaces);
      if (map_.airspaces.size() != prevAirspaces) {
        ++map_.geometryEpoch;
      }
    }
    mapPanDirty_ = false;
    sinceMapRebuildSeconds_ = 0.0;
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

}  // namespace avionics
