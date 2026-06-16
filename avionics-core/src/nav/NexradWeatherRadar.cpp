#include "avionics/NexradWeatherRadar.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#if defined(AVIONICS_HAS_CURL) && defined(AVIONICS_HAS_STBIMAGE)
#define AVIONICS_NEXRAD_LIVE 1
#else
#define AVIONICS_NEXRAD_LIVE 0
#endif

#if AVIONICS_NEXRAD_LIVE
#include <mutex>
#include <string>

#include <curl/curl.h>

// STB_IMAGE_STATIC keeps every stb symbol local to this translation unit, so it
// doesn't collide with the copy NanoVG already compiles into the renderer.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include "stb_image.h"
#endif

namespace avionics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNmPerDegLat = 60.0;

// Refetch tiles once the aircraft has drifted this far from the fetch center,
// or once the cached frame is this old (RainViewer publishes ~every 10 min).
constexpr double kRefetchDistanceNm = 40.0;
constexpr double kRefetchAgeSeconds = 300.0;
// Rebuild the ownship-centered grid once the aircraft moves this far, so the
// (geo-fixed) weather appears to stay put on the ground as ownship moves.
constexpr double kResampleDistanceNm = 1.0;

double clampLat(double lat) { return std::max(-85.0, std::min(85.0, lat)); }

double cosLatClamped(double latDeg) {
  return std::max(0.05, std::cos(latDeg * kPi / 180.0));
}

// Rough planar NM between two coordinates (fine over the tens-of-NM scales here).
double approxDistanceNm(double lat1, double lon1, double lat2, double lon2) {
  const double dLat = (lat2 - lat1) * kNmPerDegLat;
  const double dLon =
      (lon2 - lon1) * kNmPerDegLat * cosLatClamped((lat1 + lat2) * 0.5);
  return std::hypot(dLat, dLon);
}

// RainViewer's live tilecache serves a single fixed "Universal Blue" palette
// (the {color}/{snow} URL params are ignored by the deployed endpoint), which
// encodes composite reflectivity as: tan (very light) -> cyan -> blue -> yellow
// -> orange -> red -> pink (extreme). To drive the real G1000 legend we invert
// the exact published palette (rainviewer_api_colors_table.csv, "Universal Blue"
// column) back to dBZ, then the shared overlay recolors by dBZ. Doing the exact
// inversion (rather than guessing from hue) is what keeps light drizzle (tan)
// from being mistaken for heavy precip.
struct UbStop {
  float dbz;
  unsigned char r, g, b;
};
// Universal Blue dBZ -> RGB (opaque entries, dBZ -10..64; the 65+ white/green
// table sentinels are dropped so they can't capture near-white smoothed edges).
constexpr UbStop kUb[] = {
    {-10.0f, 99, 97, 89},    {-9.0f, 102, 99, 90},    {-8.0f, 105, 102, 92},
    {-7.0f, 108, 104, 93},   {-6.0f, 111, 107, 95},   {-5.0f, 114, 110, 97},
    {-4.0f, 117, 112, 98},   {-3.0f, 120, 115, 100},  {-2.0f, 124, 117, 101},
    {-1.0f, 127, 120, 103},  {0.0f, 130, 123, 105},   {1.0f, 133, 125, 106},
    {2.0f, 136, 128, 108},   {3.0f, 139, 130, 109},   {4.0f, 142, 133, 111},
    {5.0f, 146, 136, 113},   {6.0f, 158, 147, 117},   {7.0f, 170, 158, 121},
    {8.0f, 182, 169, 126},   {9.0f, 194, 180, 130},   {10.0f, 206, 192, 135},
    {11.0f, 210, 196, 139},  {12.0f, 214, 200, 143},  {13.0f, 218, 204, 147},
    {14.0f, 222, 208, 151},  {15.0f, 136, 221, 238},  {16.0f, 108, 209, 235},
    {17.0f, 81, 197, 232},   {18.0f, 54, 186, 229},   {19.0f, 27, 174, 226},
    {20.0f, 0, 163, 224},    {21.0f, 0, 154, 213},    {22.0f, 0, 145, 202},
    {23.0f, 0, 136, 191},    {24.0f, 0, 127, 180},    {25.0f, 0, 119, 170},
    {26.0f, 0, 112, 163},    {27.0f, 0, 105, 156},    {28.0f, 0, 98, 149},
    {29.0f, 0, 91, 142},     {30.0f, 0, 85, 136},     {31.0f, 0, 81, 128},
    {32.0f, 0, 78, 120},     {33.0f, 0, 74, 112},     {34.0f, 0, 71, 104},
    {35.0f, 255, 238, 0},    {36.0f, 255, 224, 0},    {37.0f, 255, 210, 0},
    {38.0f, 255, 197, 0},    {39.0f, 255, 183, 0},    {40.0f, 255, 170, 0},
    {41.0f, 255, 159, 0},    {42.0f, 255, 149, 0},    {43.0f, 255, 139, 0},
    {44.0f, 255, 129, 0},    {45.0f, 255, 68, 0},     {46.0f, 242, 54, 0},
    {47.0f, 230, 40, 0},     {48.0f, 217, 27, 0},     {49.0f, 205, 13, 0},
    {50.0f, 193, 0, 0},      {51.0f, 168, 0, 0},      {52.0f, 143, 0, 0},
    {53.0f, 118, 0, 0},      {54.0f, 93, 0, 0},       {55.0f, 255, 170, 255},
    {56.0f, 255, 159, 255},  {57.0f, 255, 149, 255},  {58.0f, 255, 139, 255},
    {59.0f, 255, 129, 255},  {60.0f, 255, 119, 255},  {61.0f, 255, 108, 255},
    {62.0f, 255, 98, 255},   {63.0f, 255, 88, 255},   {64.0f, 255, 78, 255},
};

// Nearest-color inversion of a tile pixel to a reflectivity dBZ.
float universalBlueDbz(unsigned char R, unsigned char G, unsigned char B) {
  int bestIdx = 0;
  long bestDist = 1L << 30;
  for (int i = 0; i < static_cast<int>(sizeof(kUb) / sizeof(kUb[0])); ++i) {
    const long dr = static_cast<long>(R) - kUb[i].r;
    const long dg = static_cast<long>(G) - kUb[i].g;
    const long db = static_cast<long>(B) - kUb[i].b;
    const long d = dr * dr + dg * dg + db * db;
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  }
  return kUb[bestIdx].dbz;
}

// Web-mercator (EPSG:3857) global pixel coordinate at a zoom level, with a
// 256 px tile size. x increases east, y increases south.
double mercatorGlobalX(double lonDeg, int zoom) {
  const double world = 256.0 * std::pow(2.0, zoom);
  return (lonDeg + 180.0) / 360.0 * world;
}

double mercatorGlobalY(double latDeg, int zoom) {
  const double world = 256.0 * std::pow(2.0, zoom);
  const double lat = clampLat(latDeg) * kPi / 180.0;
  const double y = (1.0 - std::asinh(std::tan(lat)) / kPi) / 2.0;
  return y * world;
}

}  // namespace

#if AVIONICS_NEXRAD_LIVE
namespace {

std::once_flag g_curlInitFlag;
void ensureCurlInit() {
  std::call_once(g_curlInitFlag, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

std::size_t appendToString(char* data, std::size_t size, std::size_t nmemb,
                           void* userData) {
  static_cast<std::string*>(userData)->append(data, size * nmemb);
  return size * nmemb;
}

std::size_t appendToBytes(char* data, std::size_t size, std::size_t nmemb,
                          void* userData) {
  auto* out = static_cast<std::vector<unsigned char>*>(userData);
  out->insert(out->end(), data, data + size * nmemb);
  return size * nmemb;
}

// Aborts the transfer when the owning radar's abort flag is set, so destruction
// doesn't wait out the network timeout.
int abortProgress(void* clientp, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
  const auto* abort = static_cast<const std::atomic<bool>*>(clientp);
  return (abort != nullptr && abort->load(std::memory_order_acquire)) ? 1 : 0;
}

void applyCommonOptions(CURL* curl, const std::atomic<bool>* abort) {
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "G1000-NXi-NEXRAD");
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 6L);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, abortProgress);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA, abort);
}

bool httpGet(const std::string& url, const std::atomic<bool>* abort,
             std::string& body) {
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  applyCommonOptions(curl, abort);
  long httpCode = 0;
  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK && httpCode == 200 && !body.empty();
}

bool httpGetBytes(const std::string& url, const std::atomic<bool>* abort,
                  std::vector<unsigned char>& body) {
  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendToBytes);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  applyCommonOptions(curl, abort);
  long httpCode = 0;
  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK && httpCode == 200 && !body.empty();
}

// Extract the first `"key":"value"` string after `from` in a JSON blob.
std::string jsonStringAfter(const std::string& json, const char* key,
                            std::size_t from = 0) {
  const std::string needle = std::string("\"") + key + "\"";
  std::size_t pos = json.find(needle, from);
  if (pos == std::string::npos) return {};
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) return {};
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) return {};
  const std::size_t end = json.find('"', pos + 1);
  if (end == std::string::npos) return {};
  return json.substr(pos + 1, end - pos - 1);
}

// Latest past radar frame path: the last "path" entry inside the radar/past
// array (frames are oldest-first, so the most recent is last).
std::string latestRadarPath(const std::string& json) {
  const std::size_t past = json.find("\"past\"");
  std::size_t end = json.find("\"nowcast\"", past == std::string::npos ? 0 : past);
  if (end == std::string::npos) end = json.size();
  std::string path;
  std::size_t search = (past == std::string::npos) ? 0 : past;
  for (;;) {
    const std::size_t at = json.find("\"path\"", search);
    if (at == std::string::npos || at >= end) break;
    path = jsonStringAfter(json, "path", at);
    search = at + 6;
  }
  return path;
}

}  // namespace

void NexradWeatherRadar::fetchMosaic(double lat, double lon,
                                     const std::atomic<bool>* abort,
                                     Mosaic& out) {
  ensureCurlInit();
  out.valid = false;
  out.zoom = kZoom;
  out.tiles.clear();

  std::string index;
  if (!httpGet("https://api.rainviewer.com/public/weather-maps.json", abort,
               index)) {
    return;
  }
  const std::string host = jsonStringAfter(index, "host");
  const std::string path = latestRadarPath(index);
  if (host.empty() || path.empty()) return;

  // Tile range covering center +/- the overlay radius.
  const double dLat = kRadiusNm / kNmPerDegLat;
  const double dLon = kRadiusNm / kNmPerDegLat / cosLatClamped(lat);
  const int worldTiles = 1 << kZoom;
  int xMin = static_cast<int>(std::floor(mercatorGlobalX(lon - dLon, kZoom) / 256.0));
  int xMax = static_cast<int>(std::floor(mercatorGlobalX(lon + dLon, kZoom) / 256.0));
  int yMin = static_cast<int>(std::floor(mercatorGlobalY(lat + dLat, kZoom) / 256.0));
  int yMax = static_cast<int>(std::floor(mercatorGlobalY(lat - dLat, kZoom) / 256.0));
  yMin = std::max(0, yMin);
  yMax = std::min(worldTiles - 1, yMax);
  // Cap how many tiles we pull regardless of latitude.
  xMax = std::min(xMax, xMin + 7);
  yMax = std::min(yMax, yMin + 7);

  // The index fetch succeeded -> the mosaic is usable even if a region has no
  // returns (clear weather decodes to transparent tiles).
  out.valid = true;

  for (int tx = xMin; tx <= xMax; ++tx) {
    if (abort != nullptr && abort->load(std::memory_order_acquire)) return;
    const int wrappedX = ((tx % worldTiles) + worldTiles) % worldTiles;
    for (int ty = yMin; ty <= yMax; ++ty) {
      // options 0_0 = no smoothing, so each pixel is an exact palette color we
      // can invert precisely to dBZ (the live endpoint ignores the {color}
      // param and always serves the Universal Blue palette regardless).
      const std::string url = host + path + "/256/" + std::to_string(kZoom) +
                              "/" + std::to_string(wrappedX) + "/" +
                              std::to_string(ty) + "/6/0_0.png";
      std::vector<unsigned char> png;
      if (!httpGetBytes(url, abort, png)) continue;
      int w = 0, h = 0, comp = 0;
      unsigned char* pixels = stbi_load_from_memory(
          png.data(), static_cast<int>(png.size()), &w, &h, &comp, 4);
      if (pixels == nullptr || w != 256 || h != 256) {
        if (pixels != nullptr) stbi_image_free(pixels);
        continue;
      }
      std::vector<unsigned char> rgba(pixels, pixels + 256 * 256 * 4);
      stbi_image_free(pixels);
      const std::uint64_t key =
          (static_cast<std::uint64_t>(wrappedX) << 32) |
          static_cast<std::uint32_t>(ty);
      out.tiles.emplace(key, std::move(rgba));
    }
  }
}
#else   // !AVIONICS_NEXRAD_LIVE
void NexradWeatherRadar::fetchMosaic(double, double, const std::atomic<bool>*,
                                     Mosaic& out) {
  out.valid = false;
}
#endif  // AVIONICS_NEXRAD_LIVE

NexradWeatherRadar::NexradWeatherRadar() = default;

NexradWeatherRadar::~NexradWeatherRadar() {
  abortFetch_.store(true, std::memory_order_release);
  if (worker_.joinable()) worker_.join();
  stopResampleWorker();
}

void NexradWeatherRadar::setCenter(double lat, double lon) {
  centerLat_ = lat;
  centerLon_ = lon;
  haveCenter_ = true;
}

void NexradWeatherRadar::maybeStartFetch() {
#if AVIONICS_NEXRAD_LIVE
  if (!haveCenter_) return;
  if (fetching_.load(std::memory_order_acquire)) return;

  const bool stale = secondsSinceFetch_ >= kRefetchAgeSeconds;
  const bool moved =
      approxDistanceNm(centerLat_, centerLon_, fetchCenterLat_, fetchCenterLon_) >
      kRefetchDistanceNm;
  const bool haveMosaic = activeMosaic_ && activeMosaic_->valid;
  const bool first = !haveMosaic && !resultReady_.load();
  if (!stale && !moved && !first) return;

  if (worker_.joinable()) worker_.join();  // reap the previous fetch
  fetchCenterLat_ = centerLat_;
  fetchCenterLon_ = centerLon_;
  secondsSinceFetch_ = 0.0;
  fetching_.store(true, std::memory_order_release);
  const double lat = centerLat_;
  const double lon = centerLon_;
  worker_ = std::thread([this, lat, lon] {
    Mosaic m;
    // Never let an exception escape the worker (that would call std::terminate);
    // a failed fetch simply publishes an invalid mosaic.
    try {
      fetchMosaic(lat, lon, &abortFetch_, m);
    } catch (...) {
      m = Mosaic{};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = std::move(m);
    }
    resultReady_.store(true, std::memory_order_release);
    fetching_.store(false, std::memory_order_release);
  });
#endif
}

void NexradWeatherRadar::advance(double dtSeconds) {
  secondsSinceFetch_ += dtSeconds;
  maybeStartFetch();

  bool newData = false;
  if (resultReady_.exchange(false, std::memory_order_acq_rel)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.valid) {
      std::fprintf(stderr, "NEXRAD: fetched %zu radar tiles\n",
                   pending_.tiles.size());
      activeMosaic_ = std::make_shared<const Mosaic>(std::move(pending_));
      newData = true;
    } else {
      std::fprintf(stderr, "NEXRAD: fetch failed (no network / no data)\n");
    }
    pending_ = Mosaic{};
  }

  // Publish a finished off-thread resample. strength_ is only ever written here
  // (on the consumer thread), so returnStrength()/revision() readers never race
  // the worker.
  {
    std::lock_guard<std::mutex> lock(resampleMu_);
    if (resamplePhase_ == ResamplePhase::Done) {
      strength_.swap(resampleScratch_);
      resampledLat_ = resampleDoneLat_;
      resampledLon_ = resampleDoneLon_;
      resamplePhase_ = ResamplePhase::Idle;
      active_ = true;
      ++revision_;
    }
  }

  if (!activeMosaic_ || !activeMosaic_->valid || !haveCenter_) return;

  const bool moved =
      approxDistanceNm(centerLat_, centerLon_, resampledLat_, resampledLon_) >
      kResampleDistanceNm;
  if (newData || moved || strength_.empty()) submitResample();
}

void NexradWeatherRadar::startResampleWorker() {
  if (!resampleThread_.joinable()) {
    resampleThread_ = std::thread([this] { resampleWorkerMain(); });
  }
}

void NexradWeatherRadar::stopResampleWorker() {
  {
    std::lock_guard<std::mutex> lock(resampleMu_);
    resampleStop_ = true;
  }
  resampleCv_.notify_all();
  if (resampleThread_.joinable()) resampleThread_.join();
}

// Hand the latest mosaic + center to the worker, unless one is already running
// (a still-moving ownship simply triggers a fresh job once that one is adopted).
void NexradWeatherRadar::submitResample() {
  startResampleWorker();
  std::lock_guard<std::mutex> lock(resampleMu_);
  if (resamplePhase_ != ResamplePhase::Idle) return;
  resampleMosaic_ = activeMosaic_;  // shared_ptr keeps the tiles alive for the worker
  resampleReqLat_ = centerLat_;
  resampleReqLon_ = centerLon_;
  resamplePhase_ = ResamplePhase::Running;
  resampleCv_.notify_one();
}

void NexradWeatherRadar::resampleWorkerMain() {
  for (;;) {
    std::shared_ptr<const Mosaic> mosaic;
    double lat = 0.0;
    double lon = 0.0;
    {
      std::unique_lock<std::mutex> lock(resampleMu_);
      resampleCv_.wait(lock, [this] {
        return resampleStop_ || resamplePhase_ == ResamplePhase::Running;
      });
      if (resampleStop_) return;
      mosaic = resampleMosaic_;
      lat = resampleReqLat_;
      lon = resampleReqLon_;
    }

    std::vector<unsigned char> out;
    if (mosaic && mosaic->valid) {
      resampleInto(*mosaic, lat, lon, out);
    } else {
      out.assign(static_cast<std::size_t>(kGrid) * kGrid, 0);
    }

    {
      std::lock_guard<std::mutex> lock(resampleMu_);
      if (resampleStop_) return;
      resampleScratch_.swap(out);
      resampleDoneLat_ = lat;
      resampleDoneLon_ = lon;
      resamplePhase_ = ResamplePhase::Done;
    }
  }
}

void NexradWeatherRadar::resampleInto(const Mosaic& mosaic, double centerLat,
                                      double centerLon,
                                      std::vector<unsigned char>& out) {
  out.assign(static_cast<std::size_t>(kGrid) * kGrid, 0);

  const int zoom = mosaic.zoom;
  const int worldTiles = 1 << zoom;
  const double cosLat = cosLatClamped(centerLat);

  for (int row = 0; row < kGrid; ++row) {
    // row 0 is north (top of the texture); +north NM at the top edge.
    const double northNm =
        (0.5 - (row + 0.5) / static_cast<double>(kGrid)) * 2.0 * kRadiusNm;
    const double cellLat = centerLat + northNm / kNmPerDegLat;
    unsigned char* strRow =
        out.data() + static_cast<std::size_t>(row) * kGrid;
    for (int col = 0; col < kGrid; ++col) {
      const double eastNm =
          ((col + 0.5) / static_cast<double>(kGrid) * 2.0 - 1.0) * kRadiusNm;
      const double cellLon =
          centerLon + eastNm / kNmPerDegLat / cosLat;

      const double gx = mercatorGlobalX(cellLon, zoom);
      const double gy = mercatorGlobalY(cellLat, zoom);
      int tileX = static_cast<int>(std::floor(gx / 256.0));
      const int tileY = static_cast<int>(std::floor(gy / 256.0));
      tileX = ((tileX % worldTiles) + worldTiles) % worldTiles;
      if (tileY < 0 || tileY >= worldTiles) continue;

      const std::uint64_t key =
          (static_cast<std::uint64_t>(tileX) << 32) |
          static_cast<std::uint32_t>(tileY);
      const auto it = mosaic.tiles.find(key);
      if (it == mosaic.tiles.end()) continue;

      int px = static_cast<int>(gx) - tileX * 256;
      int py = static_cast<int>(gy) - tileY * 256;
      px = std::max(0, std::min(255, px));
      py = std::max(0, std::min(255, py));
      const unsigned char* src =
          it->second.data() + (static_cast<std::size_t>(py) * 256 + px) * 4;

      // Transparent tile pixel == no return. Otherwise invert the fixed
      // RainViewer palette to reflectivity (dBZ) and pack it as a fraction of
      // full scale; the shared overlay recolors by dBZ with the G1000 legend.
      if (src[3] == 0) continue;
      const float dbz = universalBlueDbz(src[0], src[1], src[2]);
      const float frac =
          std::max(0.0f, std::min(1.0f, dbz / kNexradFullScaleDbz));
      strRow[col] = static_cast<unsigned char>(std::lround(frac * 255.0f));
    }
  }
}

}  // namespace avionics
