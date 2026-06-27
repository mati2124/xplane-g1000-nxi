#include "render/map/TerrainRaster.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "avionics/Terrain.h"
#include "render/map/MapProjection.h"
#include "render/map/MapViewInternal.h"

namespace avionics::map {
namespace {

// Raster edge length in pixels at close range. Halved above
// kFullDetailTerrainMaxNm where each pixel already spans several NM.
constexpr int kFullRasterSize = kTerrainFullRasterSize;
constexpr int kHiResRasterSize = kTerrainHiResRasterSize;
constexpr int kCoarseRasterSize = kTerrainCoarseRasterSize;

int rasterSizeFor(bool coarseSample, float halfNm, float pixelsPerNm) {
  if (coarseSample) return kCoarseRasterSize;
  const float minSize = static_cast<float>(kFullRasterSize);
  const float target =
      kTerrainMinRasterCellsPerScreenPixel * 2.0f * halfNm * pixelsPerNm;
  return static_cast<int>(std::min(
      static_cast<float>(kHiResRasterSize),
      std::max(minSize, std::ceil(target))));
}

bool allowTerrainStagingSwap(float rangeNm, bool zoomSettled, bool chartLand) {
  // ChartLand swaps even mid-zoom: a fast zoom-out otherwise keeps showing the
  // old, smaller-footprint raster (its edges fall outside the new viewport and
  // flash as the navy base) until the animation settles. The binary land/water
  // texture has no hillshade to make a mid-zoom swap look jarring.
  return zoomSettled || chartLand || rangeNm >= mapview::kContinentalPerfRangeNm;
}

// Raster half-width as a multiple of the map range. The viewport's rotated
// corner reaches ~1.9x range on a square map, but the MFD MAP window is wider
// than tall so east/west edges need more; drawTerrainRaster also expands the
// footprint to viewHalfExtentNm when that exceeds this factor.
constexpr float kCoverageRangeFactor = 2.4f;
constexpr float kMfdMapCornerFactor = kTerrainCornerRangeFactor;

// Small margin beyond the farthest visible map corner so drift between rebuilds
// does not expose an un-rastered strip at the viewport edge.
constexpr float kViewExtentMargin = 1.06f;

// halfNm tolerance when comparing snapshots so a smooth zoom animation does not
// cancel an in-flight async build every frame.
constexpr float kHalfNmGeometryTolerance = 0.08f;

// Rebuild once the view center drifts this fraction of the range from the
// raster center.
constexpr float kRecenterDriftFactor = 0.30f;

// Minimum draw calls between terrain rebuilds that are triggered *only* by
// newer DEM data (a tile finishing load bumps the source revision). Geometry
// changes (pan/zoom/recenter) always rebuild immediately; this rate-limits the
// data-driven refresh so a burst of streaming tiles doesn't recolor and
// re-upload the whole raster every few frames -- which lands a 512x512
// hillshade plus a texture upload on a live frame as a visible hitch in shells
// that render every frame (the standalone). ~1 s at 60 fps; terrain detail
// still fills in a few seconds after the tiles load.
constexpr int kRevisionRebuildIntervalFrames = 60;
constexpr int kCoarseRevisionRebuildIntervalFrames = 6;

// DEM rows sampled per frame during an incremental rebuild. Kept small enough
// that a live-every-frame shell (the standalone) stays near 60 fps while a
// rebuild is in flight (~48 rows ≈ 10 ms on typical hardware); a full 512-row
// build then spreads over ~11 frames. The plugin only hits this on its full-
// render frames (every Nth), so the wider spread is fine there too.
constexpr int kRowsPerFrame = 48;
constexpr int kCoarseRowsPerFrame = 128;

constexpr float kFeetPerNm = 6076.12f;

// Relative-terrain (TER REL) thresholds per the NXi terrain proximity scheme:
// terrain at/above 100 ft below the aircraft is red, within 1000 ft yellow.
constexpr float kRelRedBelowFt = 100.0f;
constexpr float kRelYellowBelowFt = 1000.0f;
// Quantize ownship altitude so REL rebuilds happen per 100 ft step, not
// continuously while climbing.
constexpr float kRelAltBucketFt = 100.0f;

// Topographic color ramp (ft MSL -> color). Lowland green and the water/land
// step match the Garmin PC Trainer capture MFD Terrain Colors.bmp; higher
// breakpoints follow the NXi TOPO SCALE (Pilot's Guide Fig 5-14). Water at or
// below MSL uses the same navy as the navigation-map ocean base so topo does not
// tint the Gulf with a separate teal ramp.
struct TerrainStop {
  float ft;
  Color color;
};

constexpr TerrainStop kTerrainStops[] = {
    {1.0f, {0.282f, 0.471f, 0.282f, 1.0f}},      // lowland green (72,120,72)
    {500.0f, {0.698f, 0.624f, 0.420f, 1.0f}},    // tan (178,159,107)
    {2000.0f, {0.753f, 0.553f, 0.357f, 1.0f}},   // clay (192,141,91)
    {3000.0f, {0.612f, 0.396f, 0.196f, 1.0f}},   // burnt orange (156,101,50)
    {6000.0f, {0.573f, 0.310f, 0.192f, 1.0f}},  // brown (146,79,49)
    {8000.0f, {0.561f, 0.267f, 0.145f, 1.0f}},  // brick (143,68,37)
    {10500.0f, {0.553f, 0.573f, 0.584f, 1.0f}}, // grey rock (141,146,149)
    {27000.0f, {0.761f, 0.780f, 0.792f, 1.0f}}, // snow (194,199,202)
};

Color terrainColor(float ft) {
  constexpr int n =
      static_cast<int>(sizeof(kTerrainStops) / sizeof(kTerrainStops[0]));
  if (ft <= kTerrainStops[0].ft) return kTerrainStops[0].color;
  if (ft >= kTerrainStops[n - 1].ft) return kTerrainStops[n - 1].color;
  for (int i = 1; i < n; ++i) {
    if (ft <= kTerrainStops[i].ft) {
      const Color& a = kTerrainStops[i - 1].color;
      const Color& b = kTerrainStops[i].color;
      const float t = (ft - kTerrainStops[i - 1].ft) /
                      (kTerrainStops[i].ft - kTerrainStops[i - 1].ft);
      return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
              a.b + (b.b - a.b) * t, 1.0f};
    }
  }
  return kTerrainStops[n - 1].color;
}

// Geographic snapshot a raster was (or is being) built for.
struct Snapshot {
  double centerLat = 0.0;
  double centerLon = 0.0;
  float halfNm = 0.0f;
  TerrainRasterMode mode = TerrainRasterMode::Absolute;
  int relAltBucket = 0;
  unsigned sourceRevision = 0;
  bool coarseSample = false;
  float detailHalfNm = 0.0f;
  float builtRangeNm = 0.0f;
  int rasterSize = kFullRasterSize;

  // Geometry-only match (ignores sourceRevision): two snapshots that cover the
  // same ground at the same scale/mode, even if newer DEM tiles have since
  // loaded. Used to decide whether an in-flight build can keep going -- a tile
  // load bumps the revision every frame while flying, and restarting the build
  // each time would mean it never finishes and the map resamples the whole
  // raster on every redraw.
  bool sameGeometry(const Snapshot& o) const {
    const float halfTol =
        std::max(1.0f, halfNm * kHalfNmGeometryTolerance);
    return std::abs(halfNm - o.halfNm) <= halfTol && mode == o.mode &&
           relAltBucket == o.relAltBucket && coarseSample == o.coarseSample &&
           rasterSize == o.rasterSize;
  }

  float driftNm(const Snapshot& o) const {
    const double dN = (centerLat - o.centerLat) * kNmPerDegLat;
    const double dE = (centerLon - o.centerLon) * nmPerDegLon(o.centerLat);
    return static_cast<float>(std::sqrt(dN * dN + dE * dE));
  }
};

// Per-map-view raster cache entry. Keyed by (renderer, view center px) so each
// on-screen map instance owns one raster and one GPU image.
struct ViewRaster {
  Renderer* renderer = nullptr;
  int keyX = 0;
  int keyY = 0;
  std::uint64_t lastUse = 0;

  int imageId = -1;
  int texSizePx = 0;  // actual GL texture dimension backing imageId (authoritative)
  Snapshot front;
  bool frontValid = false;

  bool building = false;
  Snapshot target;
  int rowsDone = 0;
  std::vector<float> elevFt;
  std::vector<unsigned char> rgba;

  // Draw calls since the last completed build, used to rate-limit rebuilds that
  // are driven only by newer DEM data (see kRevisionRebuildIntervalFrames).
  int framesSinceBuild = 0;

  // Completed async/sync build held until zoom animation settles so the
  // on-screen texture is not swapped mid-zoom.
  bool stagingValid = false;
  Snapshot stagingFront;
  std::vector<unsigned char> stagingRgba;
};

void applyStagingToFront(ViewRaster& v, Renderer& r) {
  if (!v.stagingValid || v.stagingRgba.empty()) return;
  const int size = v.stagingFront.rasterSize;
  // The GL upload (glTexImage2D / glTexSubImage2D) reads size*size*4 bytes from
  // the staging buffer. If a build/upload size desync ever leaves the buffer
  // smaller than its declared raster size, uploading would read past the end of
  // the heap allocation and crash the GPU driver (seen as a page-aligned read
  // fault in glTexSubImage2D). Drop such a frame rather than feed the driver a
  // short buffer; the next completed build replaces it.
  const std::size_t needBytes =
      static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 4u;
  if (size <= 0 || v.stagingRgba.size() < needBytes) {
    v.stagingValid = false;
    v.stagingRgba.clear();
    return;
  }
  // Use the actual texture dimension (texSizePx), not front.rasterSize, to decide
  // update-vs-recreate: an in-place update must match the existing texture size
  // exactly, otherwise the driver uploads the old (larger) dimensions from the
  // new (smaller) buffer.
  if (v.imageId < 0 || v.texSizePx != size) {
    if (v.imageId >= 0) r.deleteImage(v.imageId);
    v.imageId = r.createImageRGBA(size, size, v.stagingRgba.data());
    v.texSizePx = (v.imageId >= 0) ? size : 0;
  } else {
    r.updateImageRGBA(v.imageId, v.stagingRgba.data());
  }
  v.front = v.stagingFront;
  v.frontValid = v.imageId >= 0;
  v.building = false;
  v.framesSinceBuild = 0;
  v.stagingValid = false;
  v.stagingRgba.clear();
}

constexpr std::size_t kMaxViewRasters = 8;

std::vector<std::unique_ptr<ViewRaster>>& registry() {
  static std::vector<std::unique_ptr<ViewRaster>> views;
  return views;
}

ViewRaster& viewFor(Renderer& r, float cx, float cy) {
  static std::uint64_t useCounter = 0;
  const int kx = static_cast<int>(std::lround(cx));
  const int ky = static_cast<int>(std::lround(cy));
  auto& views = registry();
  for (auto& v : views) {
    if (v->renderer == &r && v->keyX == kx && v->keyY == ky) {
      v->lastUse = ++useCounter;
      return *v;
    }
  }
  if (views.size() >= kMaxViewRasters) {
    auto oldest = std::min_element(
        views.begin(), views.end(),
        [](const auto& a, const auto& b) { return a->lastUse < b->lastUse; });
    // Only reclaim the GPU image when the evicted entry belongs to the same
    // renderer (whose GL context is current right now, since it is drawing).
    if ((*oldest)->renderer == &r && (*oldest)->imageId >= 0) {
      r.deleteImage((*oldest)->imageId);
    }
    views.erase(oldest);
  }
  views.push_back(std::make_unique<ViewRaster>());
  ViewRaster& v = *views.back();
  v.renderer = &r;
  v.keyX = kx;
  v.keyY = ky;
  v.lastUse = ++useCounter;
  return v;
}

// Samples a horizontal band of DEM rows into the elevation grid. Within a row
// latitude is fixed and longitude steps uniformly, so the per-cell lon is
// lonStart + lonStep*j; handing the whole row to elevationFtRow lets the DSF
// store resolve (and lock) the tile once per row instead of once per pixel.
// True while a background terrain worker is doing the sampling (standalone).
// Defined below; forward-declared so prefetchTerrain can decide whether it is
// safe to block waiting for tiles (only when off the render/sim thread).
extern bool g_asyncBuilds;

void prefetchTerrain(const TerrainSource& terrain, const Snapshot& s) {
  const double nmLon = nmPerDegLon(s.centerLat);
  const double minLat = s.centerLat - s.halfNm / kNmPerDegLat;
  const double maxLat = s.centerLat + s.halfNm / kNmPerDegLat;
  const double minLon = s.centerLon - s.halfNm / nmLon;
  const double maxLon = s.centerLon + s.halfNm / nmLon;
  // The ChartLand mask covers only a few tiles (capped at close range) and its
  // first frame is useless when sampled before tiles load -- it builds an all-
  // water raster that only self-corrects on the next geometry change (the pilot
  // nudging the range knob). When the sampling runs on the async worker, block
  // until the handful of coarse summaries are ready so the very first published
  // raster already shows land. Never block the synchronous (plugin) path.
  const bool waitForTiles =
      s.mode == TerrainRasterMode::ChartLand && g_asyncBuilds;
  terrain.ensureCoverage(minLat, maxLat, minLon, maxLon, waitForTiles);
}

void sampleRows(ViewRaster& v, const TerrainSource& terrain, int rows) {
  const Snapshot& s = v.target;
  const int rasterSize = s.rasterSize;
  if (v.rowsDone == 0) {
    terrain.setBulkTerrainSample(true);
    terrain.setCoarseTerrainSample(s.coarseSample);
    terrain.setTerrainViewCenter(s.centerLat, s.centerLon, s.detailHalfNm);
    prefetchTerrain(terrain, s);
  }
  const float stepNm = 2.0f * s.halfNm / rasterSize;
  const double nmLon = nmPerDegLon(s.centerLat);
  const double lonStart =
      s.centerLon + (-s.halfNm + 0.5 * stepNm) / nmLon;
  const double lonStep = static_cast<double>(stepNm) / nmLon;
  const int endRow = std::min(rasterSize, v.rowsDone + rows);
  for (int i = v.rowsDone; i < endRow; ++i) {
    const double northNm = s.halfNm - (i + 0.5) * stepNm;
    const double lat = s.centerLat + northNm / kNmPerDegLat;
    float* out =
        v.elevFt.data() + static_cast<std::size_t>(i) * rasterSize;
    terrain.elevationFtRow(lat, lonStart, lonStep, rasterSize, out);
  }
  v.rowsDone = endRow;
  if (v.rowsDone >= rasterSize) {
    terrain.setBulkTerrainSample(false);
    terrain.setCoarseTerrainSample(false);
  }
}

void writePixel(unsigned char* px, const Color& c) {
  px[0] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.r)) * 255.0f));
  px[1] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.g)) * 255.0f));
  px[2] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.b)) * 255.0f));
  px[3] = static_cast<unsigned char>(
      std::lround(std::min(1.0f, std::max(0.0f, c.a)) * 255.0f));
}

// One separable triangular-weighted low-pass of the given cell radius. NaN
// (water / no-data) samples are skipped so coastlines are not pulled inland.
void smoothElevationPass(const std::vector<float>& src, std::vector<float>& dst,
                         int n, int radius) {
  dst.resize(src.size());
  std::vector<float> tmp(src.size());
  const auto readRow = [&](const std::vector<float>& grid, int r) {
    return grid.data() + static_cast<std::size_t>(r) * n;
  };
  const auto writeRow = [&](std::vector<float>& grid, int r) {
    return grid.data() + static_cast<std::size_t>(r) * n;
  };
  for (int i = 0; i < n; ++i) {
    const float* in = readRow(src, i);
    float* out = writeRow(tmp, i);
    for (int j = 0; j < n; ++j) {
      const int j0 = std::max(j - radius, 0);
      const int j1 = std::min(j + radius, n - 1);
      float sum = 0.0f;
      float weight = 0.0f;
      for (int k = j0; k <= j1; ++k) {
        if (std::isnan(in[k])) continue;
        const float w = static_cast<float>(radius + 1 - std::abs(k - j));
        sum += in[k] * w;
        weight += w;
      }
      out[j] = weight > 0.0f ? sum / weight : in[j];
    }
  }
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      const int i0 = std::max(i - radius, 0);
      const int i1 = std::min(i + radius, n - 1);
      float sum = 0.0f;
      float weight = 0.0f;
      for (int k = i0; k <= i1; ++k) {
        const float v = readRow(tmp, k)[j];
        if (std::isnan(v)) continue;
        const float w = static_cast<float>(radius + 1 - std::abs(k - i));
        sum += v * w;
        weight += w;
      }
      writeRow(dst, i)[j] =
          weight > 0.0f ? sum / weight : readRow(tmp, i)[j];
    }
  }
}

void smoothElevationForHillshade(const std::vector<float>& src,
                                 std::vector<float>& smooth, int n, int radius,
                                 int passes) {
  if (passes <= 0 || radius <= 0) {
    smooth = src;
    return;
  }
  std::vector<float> a;
  std::vector<float> b;
  smoothElevationPass(src, a, n, radius);
  for (int p = 1; p < passes; ++p) {
    smoothElevationPass(a, b, n, radius);
    a.swap(b);
  }
  smooth = std::move(a);
}

// Converts the sampled elevation grid into RGBA. Absolute mode applies the
// topo ramp plus a NW-lit hillshade (slope from the DEM gradient) so relief
// reads like the real TOPO map; Relative mode applies the TER REL proximity
// colors against the snapshot's altitude bucket.
void colorize(ViewRaster& v) {
  const Snapshot& s = v.target;
  const int rasterSize = s.rasterSize;

  if (s.mode == TerrainRasterMode::ChartLand) {
    for (int i = 0; i < rasterSize; ++i) {
      const float* row =
          v.elevFt.data() + static_cast<std::size_t>(i) * rasterSize;
      unsigned char* px =
          v.rgba.data() + static_cast<std::size_t>(i) * rasterSize * 4;
      for (int j = 0; j < rasterSize; ++j, px += 4) {
        const float e = row[j];
        if (std::isnan(e)) {
          writePixel(px, Color{0.0f, 0.0f, 0.0f, 0.0f});
        } else if (e <= 0.0f) {
          writePixel(px, mapview::kMapOceanFill);
        } else {
          writePixel(px, mapview::kMapLandFill);
        }
      }
    }
    return;
  }

  const float cellFt = (2.0f * s.halfNm / rasterSize) * kFeetPerNm;
  // Light from the northwest, above (x = east, y = south, z = up).
  constexpr float kLx = -0.45f, kLy = -0.45f, kLz = 0.77f;
  // Slope exaggeration so ~90 m cells still produce visible relief.
  constexpr float kSlopeGain = 3.0f;

  const float ownAltFt =
      static_cast<float>(s.relAltBucket) * kRelAltBucketFt;

  std::vector<float> shadeElev;
  const std::vector<float>* hillshadeGrid = &v.elevFt;
  if (s.mode == TerrainRasterMode::Absolute && !s.coarseSample) {
    // Size the kernel from ground distance so the whole-meter DEM quantization
    // is bridged consistently at every range (texel spacing varies with range).
    const float cellNm = 2.0f * s.halfNm / static_cast<float>(rasterSize);
    const int smoothRadius = std::max(
        kTerrainHillshadeSmoothMinRadiusCells,
        std::min(kTerrainHillshadeSmoothMaxRadiusCells,
                 static_cast<int>(std::lround(
                     kTerrainHillshadeSmoothRadiusNm / cellNm))));
    smoothElevationForHillshade(v.elevFt, shadeElev, rasterSize, smoothRadius,
                                kTerrainHillshadeSmoothPasses);
    hillshadeGrid = &shadeElev;
  }

  for (int i = 0; i < rasterSize; ++i) {
    const float* row =
        v.elevFt.data() + static_cast<std::size_t>(i) * rasterSize;
    const float* shadeRow =
        hillshadeGrid->data() + static_cast<std::size_t>(i) * rasterSize;
    const float* rowN = hillshadeGrid->data() +
        static_cast<std::size_t>(std::max(i - 1, 0)) * rasterSize;
    const float* rowS = hillshadeGrid->data() +
        static_cast<std::size_t>(std::min(i + 1, rasterSize - 1)) * rasterSize;
    unsigned char* px =
        v.rgba.data() + static_cast<std::size_t>(i) * rasterSize * 4;
    for (int j = 0; j < rasterSize; ++j, px += 4) {
      const float e = row[j];

      if (std::isnan(e)) {
        writePixel(px, Color{0.0f, 0.0f, 0.0f, 0.0f});
        continue;
      }

      if (s.mode == TerrainRasterMode::Relative) {
        const float rel = e - ownAltFt;
        if (rel >= -kRelRedBelowFt) {
          writePixel(px, colors::kBandRed);
        } else if (rel >= -kRelYellowBelowFt) {
          writePixel(px, colors::kBandYellow);
        } else {
          writePixel(px, colors::kBlack);
        }
        continue;
      }

      if (e <= 0.0f) {
        writePixel(px, mapview::kMapOceanFill);
        continue;
      }

      const float colorFt = hillshadeGrid == &v.elevFt ? e : shadeRow[j];
      Color c = terrainColor(colorFt);
      if (e > 0.5f && !s.coarseSample) {
        const int jW = std::max(j - 1, 0);
        const int jE = std::min(j + 1, rasterSize - 1);
        const float dzdx =
            kSlopeGain * (shadeRow[jE] - shadeRow[jW]) / (2.0f * cellFt);
        const float dzdy = kSlopeGain * (rowS[j] - rowN[j]) / (2.0f * cellFt);
        const float invLen =
            1.0f / std::sqrt(dzdx * dzdx + dzdy * dzdy + 1.0f);
        const float dot =
            (-dzdx * kLx - dzdy * kLy + kLz) * invLen;  // normal . light
        // Normalized so flat ground keeps the ramp color exactly.
        const float bright =
            std::min(1.30f, std::max(0.40f, 0.25f + 0.75f * (dot / kLz)));
        c.r *= bright;
        c.g *= bright;
        c.b *= bright;
      }
      writePixel(px, c);
    }
  }
}

// Background terrain builder for live-every-frame shells (standalone). DEM
// sampling and hillshade colorize are CPU-heavy (~100 ms for a full 512x512
// raster); doing them on the render thread pins the whole suite below 10 fps.
// The worker never touches GL -- the render thread uploads the finished RGBA
// buffer when the job completes.
struct AsyncTerrainWorker {
  std::mutex mu;
  std::condition_variable cv;
  std::thread thread;
  bool stop = false;

  enum class Phase { Idle, Running, Done };
  Phase phase = Phase::Idle;
  bool cancel = false;

  const TerrainSource* terrain = nullptr;
  Renderer* renderer = nullptr;
  int keyX = 0;
  int keyY = 0;
  Snapshot target;       // most recently requested build
  Snapshot builtTarget;  // snapshot the published `rgba` was actually built for
  std::vector<float> elevFt;
  std::vector<unsigned char> rgba;

  void ensureStarted() {
    if (!thread.joinable()) {
      thread = std::thread([this] { run(); });
    }
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mu);
      stop = true;
      cancel = true;
    }
    cv.notify_all();
    if (thread.joinable()) {
      thread.join();
    }
    stop = false;
    phase = Phase::Idle;
  }

  void run() {
    for (;;) {
      const TerrainSource* ter = nullptr;
      Snapshot snap;
      {
        std::unique_lock<std::mutex> lock(mu);
        cv.wait(lock, [this] { return stop || phase == Phase::Running; });
        if (stop) return;
        ter = terrain;
        snap = target;
      }

      ViewRaster scratch;
      scratch.target = snap;
      scratch.rowsDone = 0;
      const int rasterSize = snap.rasterSize;
      scratch.elevFt.resize(static_cast<std::size_t>(rasterSize) * rasterSize);
      scratch.rgba.resize(static_cast<std::size_t>(rasterSize) * rasterSize * 4);

      bool cancelled = false;
      while (scratch.rowsDone < rasterSize) {
        {
          std::lock_guard<std::mutex> lock(mu);
          if (cancel) {
            cancelled = true;
            break;
          }
        }
        sampleRows(scratch, *ter,
                   snap.coarseSample ? kCoarseRowsPerFrame : kRowsPerFrame);
      }

      if (!cancelled) {
        colorize(scratch);
        std::lock_guard<std::mutex> lock(mu);
        if (!cancel) {
          elevFt = std::move(scratch.elevFt);
          rgba = std::move(scratch.rgba);
          // Record the snapshot this buffer was built for so the consumer pairs
          // the RGBA with matching dimensions even if `target` was updated by a
          // newer submit() mid-build (otherwise it could upload a larger raster
          // size from this smaller buffer and overrun it).
          builtTarget = snap;
          phase = Phase::Done;
        } else {
          phase = Phase::Idle;
        }
      } else {
        std::lock_guard<std::mutex> lock(mu);
        phase = Phase::Idle;
      }
      cancel = false;
      cv.notify_all();
    }
  }

  bool busyFor(Renderer& r, int kx, int ky) {
    std::lock_guard<std::mutex> lock(mu);
    return phase == Phase::Running && renderer == &r && keyX == kx &&
           keyY == ky;
  }

  void submit(const TerrainSource& ter, Renderer& r, int kx, int ky,
              const Snapshot& snap) {
    std::lock_guard<std::mutex> lock(mu);
    if (phase == Phase::Running) {
      cancel = true;
    }
    terrain = &ter;
    renderer = &r;
    keyX = kx;
    keyY = ky;
    target = snap;
    elevFt.clear();
    rgba.clear();
    phase = Phase::Running;
    cancel = false;
    cv.notify_one();
  }

  bool tryConsume(Renderer& r, int kx, int ky, Snapshot& outSnap,
                  std::vector<unsigned char>& outRgba) {
    std::lock_guard<std::mutex> lock(mu);
    if (phase != Phase::Done || renderer != &r || keyX != kx || keyY != ky) {
      return false;
    }
    outSnap = builtTarget;
    outRgba = std::move(rgba);
    phase = Phase::Idle;
    return true;
  }
};

bool g_asyncBuilds = false;
AsyncTerrainWorker g_async;

}  // namespace

bool drawTerrainRaster(Renderer& r, const TerrainSource& terrain,
                       TerrainRasterMode mode, float ownAltFt,
                       double viewCenterLat, double viewCenterLon, float cx,
                       float cy, float pixelsPerNm, float rotationDeg,
                       float rangeNm, float displayRangeNm,
                       float viewHalfExtentNm, float terrainMaxRangeNm) {
  if (rangeNm > terrainMaxRangeNm) return false;

  ViewRaster& v = viewFor(r, cx, cy);

  const bool chartLand = mode == TerrainRasterMode::ChartLand;
  const float zoomSettled = mapRangeZoomSettled(displayRangeNm, rangeNm);
  const bool stagingSwapOk =
      allowTerrainStagingSwap(rangeNm, zoomSettled, chartLand);
  const float minDrawHalfNm = viewHalfExtentNm * 1.01f;
  const bool rangeStepChanged =
      v.frontValid && v.front.builtRangeNm > 0.5f &&
      std::abs(rangeNm - v.front.builtRangeNm) > 0.5f;

  if (g_asyncBuilds) {
    Snapshot completedSnap;
    std::vector<unsigned char> completedRgba;
    if (g_async.tryConsume(r, v.keyX, v.keyY, completedSnap, completedRgba)) {
      v.stagingFront = completedSnap;
      v.stagingRgba = std::move(completedRgba);
      v.stagingValid = true;
      v.building = false;
      if (stagingSwapOk) {
        applyStagingToFront(v, r);
      }
    }
  }

  if (stagingSwapOk && v.stagingValid) {
    applyStagingToFront(v, r);
  }

  // Ladder range jumps immediately on RNG+/− while displayRangeNm eases. Size
  // the rebuild footprint from the target step so halfNm does not drift every
  // animation frame (which was canceling the async worker and flickering).
  const float viewHalfAtLadder =
      viewHalfExtentNm *
      (rangeNm / std::max(kMapRangeMinNm, displayRangeNm));

  Snapshot desired;
  desired.centerLat = viewCenterLat;
  desired.centerLon = viewCenterLon;
  const float viewHalfMargin =
      viewHalfAtLadder > 0.5f ? viewHalfAtLadder * kViewExtentMargin : 0.0f;
  // Over-fetch beyond the viewport (kCoverageRangeFactor) so a pan or one zoom-
  // out step stays inside the cached raster instead of exposing un-rastered
  // edges that flash as the navy base until the next rebuild. ChartLand shares
  // this: it is range-capped to 15 NM (a handful of tiles) and prewarmed, so the
  // wider footprint is affordable.
  if (viewHalfMargin > 0.0f) {
    desired.halfNm =
        std::max(rangeNm * kCoverageRangeFactor, viewHalfMargin);
  } else {
    desired.halfNm = std::max(rangeNm * kCoverageRangeFactor,
                              rangeNm * kMfdMapCornerFactor);
  }
  desired.mode = mode;
  desired.relAltBucket =
      mode == TerrainRasterMode::Relative
          ? static_cast<int>(std::lround(ownAltFt / kRelAltBucketFt))
          : 0;
  // ChartLand renders a crisp coastline from the full-resolution DEM tiles
  // rather than the coarse 16x16-per-tile summary (whose ~3.75 NM cells look
  // blocky). It is range-capped to a handful of tiles, and those tiles must be
  // read in full to build a coarse summary anyway, so full detail costs no
  // extra disk I/O -- only a per-pixel threshold, which is far cheaper than the
  // topo hillshade it skips.
  desired.coarseSample = !chartLand && rangeNm > kFullDetailTerrainMaxNm;
  desired.rasterSize =
      rasterSizeFor(desired.coarseSample, desired.halfNm, pixelsPerNm);
  // Zoomed-in ChartLand: lift the raster cap so the texture keeps ~1 texel per
  // screen pixel instead of stretching the 1152 cap across the large MFD map
  // (which softens the coast). Only a single tile is in view here, so the
  // bigger raster is affordable.
  if (chartLand && rangeNm <= kChartLandHiResRangeNm) {
    const float target =
        kTerrainMinRasterCellsPerScreenPixel * 2.0f * desired.halfNm *
        pixelsPerNm;
    desired.rasterSize = static_cast<int>(
        std::min(static_cast<float>(kChartLandHiResRasterSize),
                 std::max(static_cast<float>(desired.rasterSize),
                          std::ceil(target))));
  }
  // detailHalfNm drives which tiles the store upgrades to full DEM; cover the
  // whole footprint at full detail unless on the wide coarse-sample tier.
  desired.detailHalfNm =
      desired.coarseSample ? rangeNm * 0.25f : desired.halfNm;
  desired.builtRangeNm = rangeNm;
  desired.sourceRevision = terrain.revision();

  if (v.stagingValid &&
      std::abs(v.stagingFront.builtRangeNm - rangeNm) > 0.5f) {
    v.stagingValid = false;
    v.stagingRgba.clear();
  }

  const int revisionRebuildInterval =
      desired.coarseSample ? kCoarseRevisionRebuildIntervalFrames
                           : kRevisionRebuildIntervalFrames;

  ++v.framesSinceBuild;

  const float driftLimitNm = rangeNm * kRecenterDriftFactor;
  // Split freshness into geometry (the ground/scale/mode the raster covers) and
  // data (the DEM revision). Geometry staleness must rebuild now; data-only
  // staleness (a streaming tile bumped the revision) is rate-limited so it
  // doesn't recolor/upload every few frames.
  const bool geometryFresh = v.frontValid && v.front.sameGeometry(desired) &&
                             v.front.driftNm(desired) <= driftLimitNm;
  const bool revisionFresh = v.front.sourceRevision == desired.sourceRevision;
  bool needRebuild =
      !geometryFresh ||
      (!revisionFresh &&
       v.framesSinceBuild >= revisionRebuildInterval);
  if (!zoomSettled) {
    const bool modeChange =
        v.frontValid && (v.front.mode != desired.mode ||
                         v.front.relAltBucket != desired.relAltBucket);
    needRebuild = modeChange || (!v.frontValid && !v.building);
    // ChartLand must rebuild as soon as the footprint changes (even mid-zoom)
    // so a fast zoom-out gets a raster that covers the new, larger viewport
    // rather than scaling up the old one and exposing navy edges. Tiles are
    // prewarmed, so the rebuild lands quickly.
    if (chartLand) needRebuild = needRebuild || !geometryFresh;
  }

  const bool asyncBusy =
      g_asyncBuilds && g_async.busyFor(r, v.keyX, v.keyY);

  // Pre-build the target-range raster in the background while the zoom eases,
  // but only swap it to the screen once the animation has settled.
  const bool stagingReadyForTarget =
      v.stagingValid &&
      std::abs(v.stagingFront.builtRangeNm - rangeNm) <= 0.5f;
  const bool speculativeZoomBuild =
      !zoomSettled && rangeStepChanged && !v.building && !asyncBusy &&
      !stagingReadyForTarget;

  if (needRebuild || speculativeZoomBuild) {
    // Start a new build when idle, or restart mid-build only when zoom/mode/range
    // changes. Position drift while a build is in flight must NOT reset
    // rowsDone -- on a live-every-frame shell the view center moves every frame
    // (track-up / ownship-centered map), so treating drift as a stale target
    // restarted the build from row 0 every frame, sampled tens of thousands of
    // DEM points at ~100 ms, and never reached the colorize/upload finish line.
    if (!v.building && !asyncBusy) {
      v.building = true;
      v.target = desired;
      v.rowsDone = 0;
      const std::size_t cells =
          static_cast<std::size_t>(desired.rasterSize) * desired.rasterSize;
      if (v.elevFt.size() != cells) {
        v.elevFt.resize(cells);
        v.rgba.resize(cells * 4);
      }
      if (g_asyncBuilds) {
        g_async.submit(terrain, r, v.keyX, v.keyY, desired);
      }
    } else if (zoomSettled && !v.target.sameGeometry(desired)) {
      v.target = desired;
      v.rowsDone = 0;
      const std::size_t cells =
          static_cast<std::size_t>(desired.rasterSize) * desired.rasterSize;
      if (v.elevFt.size() != cells) {
        v.elevFt.resize(cells);
        v.rgba.resize(cells * 4);
      }
      if (g_asyncBuilds) {
        g_async.submit(terrain, r, v.keyX, v.keyY, desired);
      }
    }
  }

  if (v.building && !g_asyncBuilds) {
    // Plugin path: spread rebuild work across full-render frames.
    sampleRows(v, terrain,
               v.target.coarseSample ? kCoarseRowsPerFrame : kRowsPerFrame);
    if (v.rowsDone >= v.target.rasterSize) {
      colorize(v);
      v.stagingFront = v.target;
      v.stagingRgba = v.rgba;
      v.stagingValid = true;
      v.building = false;
      v.rowsDone = 0;
      if (stagingSwapOk) {
        applyStagingToFront(v, r);
      }
    }
  }

  if (!v.frontValid || v.imageId < 0) return false;
  if (zoomSettled && v.front.halfNm < minDrawHalfNm) return false;

  // The raster is north-up around its own snapshot center: rotate into the map
  // orientation and offset by the snapshot-vs-view center displacement so the
  // image stays geographically pinned while the aircraft drifts between
  // rebuilds.
  const float mercatorPxPerRad =
      pixelsPerNm * static_cast<float>(kNmPerEarthRad);
  double eastRad = 0.0;
  double northRad = 0.0;
  mercatorOffsetRad(v.front.centerLat, v.front.centerLon, viewCenterLat,
                    viewCenterLon, eastRad, northRad);
  const float dxPx = static_cast<float>(eastRad * mercatorPxPerRad);
  const float dyPx = -static_cast<float>(northRad * mercatorPxPerRad);
  // Scale with displayRangeNm only -- the cached texture's geographic halfNm is
  // fixed until zoom settles; pixelsPerNm already tracks the log-space zoom
  // easing used by chart land and symbology.
  const float halfPx = v.front.halfNm * pixelsPerNm;

  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(-rotationDeg);
  r.drawImage(v.imageId, dxPx - halfPx, dyPx - halfPx, 2.0f * halfPx,
              2.0f * halfPx, 1.0f);
  r.restore();
  return true;
}

void setAsyncTerrainBuilds(bool enabled) {
  if (enabled == g_asyncBuilds) return;
  if (!enabled) {
    g_async.shutdown();
  }
  g_asyncBuilds = enabled;
  if (enabled) {
    g_async.ensureStarted();
  }
}

}  // namespace avionics::map
