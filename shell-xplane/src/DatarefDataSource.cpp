#include "DatarefDataSource.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>
#include <utility>

#include "XPLMNavigation.h"
#include "XPLMUtilities.h"
#include "avionics/Datarefs.h"
#include "avionics/OpenAirParser.h"

namespace avionics {
namespace {

// Inset-map display range and the cap on features handed to the renderer.
constexpr float kMapRangeNm = 10.0f;
constexpr std::size_t kMaxMapFeatures = 250;
constexpr std::size_t kMaxMapAirspaces = 60;

// X-Plane's bundled OpenAir airspace file, relative to the system path
// (XPLMGetSystemPath). User-updated data under Custom Data wins when present.
const char* kAirspaceRelPaths[] = {
    "Custom Data/Airspaces/airspace.txt",
    "Resources/default data/airspaces/airspace.txt",
};
// Nearby-feature list is range-filtered from the cache on this cadence rather
// than every frame (ownship position itself still updates every frame).
constexpr double kMapRebuildIntervalSeconds = 1.0;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

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
  if (scored.size() > maxCount) scored.resize(maxCount);
  result.reserve(scored.size());
  for (const Scored& s : scored) result.push_back(s.feature);
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

}  // namespace

DatarefDataSource::DatarefDataSource() {
  airspeed_ = XPLMFindDataRef(datarefs::kAirspeedKts);
  altitude_ = XPLMFindDataRef(datarefs::kAltitudeFt);
  heading_ = XPLMFindDataRef(datarefs::kHeadingDegMag);
  pitch_ = XPLMFindDataRef(datarefs::kPitchDeg);
  roll_ = XPLMFindDataRef(datarefs::kRollDeg);
  verticalSpeed_ = XPLMFindDataRef(datarefs::kVerticalSpeedFpm);
  slip_ = XPLMFindDataRef(datarefs::kSlipDeg);

  latitude_ = XPLMFindDataRef(datarefs::kLatitudeDeg);
  longitude_ = XPLMFindDataRef(datarefs::kLongitudeDeg);

  gpsDistance_ = XPLMFindDataRef(datarefs::kGpsDistanceNm);
  gpsBearing_ = XPLMFindDataRef(datarefs::kGpsBearingDegMag);
  gpsNavId_ = XPLMFindDataRef(datarefs::kGpsNavId);

  // Resolve the airspace file path on the sim thread (XPLMGetSystemPath returns
  // the install root, native separator, trailing slash), then parse it off the
  // sim thread since the global file can be large.
  char systemPath[1024] = {};
  XPLMGetSystemPath(systemPath);
  std::string root(systemPath);
  std::string airspacePath;
  for (const char* rel : kAirspaceRelPaths) {
    std::string candidate = root + rel;
    std::ifstream probe(candidate);
    if (probe.good()) {
      airspacePath = std::move(candidate);
      break;
    }
  }
  if (!airspacePath.empty()) {
    loadAirspaceAsync(std::move(airspacePath));
  } else {
    airspaceLoaded_.store(true, std::memory_order_release);
  }
}

DatarefDataSource::~DatarefDataSource() {
  if (airspaceThread_.joinable()) airspaceThread_.join();
}

void DatarefDataSource::loadAirspaceAsync(std::string airspaceFilePath) {
  airspaceThread_ = std::thread([this, path = std::move(airspaceFilePath)] {
    std::ifstream in(path);
    if (in.good()) airspaceCache_ = parseOpenAir(in);
    airspaceLoaded_.store(true, std::memory_order_release);
  });
}

void DatarefDataSource::update(double dtSeconds) {
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
      char id[kNavIdBufferSize] = {};
      XPLMGetNavAidInfo(ref, &type, &lat, &lon, nullptr, nullptr, nullptr, id,
                        nullptr, nullptr);
      id[sizeof(id) - 1] = '\0';
      if (type == kind.xpType) {
        navCache_.push_back({kind.mapType, static_cast<double>(lat),
                             static_cast<double>(lon), std::string(id)});
      }
      if (ref == last) break;
      ref = XPLMGetNextNavAid(ref);
    }
  }
}

void DatarefDataSource::updateMap(double dtSeconds) {
  map_.rangeNm = kMapRangeNm;

  // Ownship position at full double precision (in-process, no RREF truncation).
  if (latitude_ && longitude_) {
    map_.ownshipLat = XPLMGetDatad(latitude_);
    map_.ownshipLon = XPLMGetDatad(longitude_);
    map_.positionValid = true;
  }

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
  if (map_.positionValid &&
      (map_.features.empty() ||
       sinceMapRebuildSeconds_ >= kMapRebuildIntervalSeconds)) {
    map_.features = filterNearby(navCache_, map_.ownshipLat, map_.ownshipLon,
                                 map_.rangeNm, kMaxMapFeatures);
    // Airspace boundaries from the (background-parsed) OpenAir file, filtered to
    // the same neighborhood as the features.
    if (airspaceLoaded_.load(std::memory_order_acquire)) {
      map_.airspaces = airspacesNear(airspaceCache_, map_.ownshipLat,
                                     map_.ownshipLon, map_.rangeNm,
                                     kMaxMapAirspaces);
    }
    sinceMapRebuildSeconds_ = 0.0;
  }
}

}  // namespace avionics
