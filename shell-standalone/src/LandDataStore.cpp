#include "LandDataStore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <utility>

#include "avionics/MapRange.h"

namespace avionics {
namespace {

constexpr double kNmPerDeg = 60.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Roads/rivers are decluttered past ~150 NM in the renderer. Lakes are an
// exception: major inland water (Great Lakes tier) stays visible on the
// continental chart, so they are queried out to the full map range.
constexpr float kNearLandRangeNm = 160.0f;
// High-res GSHHG land fill only at close range; wider charts use low-res silhouettes.
// Coast polylines are kept in the asset for tooling but never drawn — the NXi
// shoreline is the landmass fill edge against navy ocean.
constexpr float kDetailLandMaxRangeNm = 50.0f;
// High-res lon-band land slices from the lowest ladder step through mid range.
constexpr float kRegionalLandMinRangeNm = kMapRangeLadderNm[0];
constexpr float kRegionalLandMaxRangeNm = 120.0f;
constexpr float kChartLakeMinRangeNm = 500.0f;
// At 1000 NM only Great-Lakes-scale water bodies are visible (not every
// northern reservoir that would project into a Mercator wedge).
constexpr float kWideChartLakeMinSpanDeg = 2.0f;
// Wider overlap so borders and landmass fills reach the map edge at max zoom
// (the US–Mexico border reaches ~117°W; 1.35× still clipped the western
// segment when centered over Florida at 1000 NM).
constexpr float kWideLandOverlapScale = 1.85f;
constexpr float kStateBorderQueryMaxRangeNm = 200.0f;

// Lines that remain meaningful across a continental view and so are always
// collected out to the full query range.
bool isWideLandClass(LandClass c) {
  return c == LandClass::Border || c == LandClass::StateBorder ||
         c == LandClass::Coast || c == LandClass::LandMass;
}

constexpr std::uint32_t kMagic = 0x444C5641;  // "AVLD"
constexpr std::uint32_t kVersionLatest = 2;

// Asset line classes, by converter convention (tools/convert_natural_earth.py).
LandClass lineClassFor(std::uint8_t code) {
  switch (code) {
    case 0:
      return LandClass::River;
    case 1:
      return LandClass::Lake;
    case 2:
      return LandClass::Road;
    case 3:
      return LandClass::Border;
    case 4:
      return LandClass::Coast;
    case 5:
      return LandClass::StateBorder;
    case 6:
      return LandClass::Railroad;
    case 7:
      return LandClass::LandMass;
    default:
      return LandClass::Border;
  }
}

struct Reader {
  const std::uint8_t* p = nullptr;
  const std::uint8_t* end = nullptr;

  bool need(std::size_t n) const {
    return static_cast<std::size_t>(end - p) >= n;
  }
  std::uint8_t u8() { return *p++; }
  std::uint16_t u16() {
    const std::uint16_t v = static_cast<std::uint16_t>(p[0]) |
                            (static_cast<std::uint16_t>(p[1]) << 8);
    p += 2;
    return v;
  }
  std::uint32_t u32() {
    const std::uint32_t v = static_cast<std::uint32_t>(p[0]) |
                            (static_cast<std::uint32_t>(p[1]) << 8) |
                            (static_cast<std::uint32_t>(p[2]) << 16) |
                            (static_cast<std::uint32_t>(p[3]) << 24);
    p += 4;
    return v;
  }
  float f32() {
    const std::uint32_t bits = u32();
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
  }
};

}  // namespace

void LandDataStore::addToSpatialCells(
    const Bounds& b, std::size_t idx,
    std::vector<std::vector<std::size_t>>& grid) const {
  const int la0 = std::max(
      0, static_cast<int>(std::floor((b.minLat + 90.0f) / kSpatialCellDeg)));
  const int la1 = std::min(
      kSpatialLatCells - 1,
      static_cast<int>(std::floor((b.maxLat + 90.0f) / kSpatialCellDeg)));
  const int lo0 = std::max(
      0, static_cast<int>(std::floor((b.minLon + 180.0f) / kSpatialCellDeg)));
  const int lo1 = std::min(
      kSpatialLonCells - 1,
      static_cast<int>(std::floor((b.maxLon + 180.0f) / kSpatialCellDeg)));
  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      grid[static_cast<std::size_t>(la * kSpatialLonCells + lo)].push_back(idx);
    }
  }
}

void LandDataStore::forEachSpatialIndex(
    double lat, double lon, float rangeNm, float marginScale,
    const std::vector<std::vector<std::size_t>>& grid,
    std::vector<std::size_t>& scratch,
    const std::function<void(std::size_t)>& fn) const {
  scratch.clear();
  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const float latSpan =
      static_cast<float>((rangeNm / kNmPerDeg) * marginScale);
  const double northCos =
      std::max(0.05, std::cos((lat + static_cast<double>(latSpan)) * kDegToRad));
  const double southCos =
      std::max(0.05, std::cos((lat - static_cast<double>(latSpan)) * kDegToRad));
  const double cosEdge = std::min({cosLat, northCos, southCos});
  const float dLon = static_cast<float>(
      (rangeNm / (kNmPerDeg * cosEdge)) * marginScale);
  const float qMinLat = static_cast<float>(lat) - latSpan;
  const float qMaxLat = static_cast<float>(lat) + latSpan;
  const float qMinLon = static_cast<float>(lon) - dLon;
  const float qMaxLon = static_cast<float>(lon) + dLon;
  const int la0 = std::max(
      0, static_cast<int>(std::floor((qMinLat + 90.0f) / kSpatialCellDeg)));
  const int la1 = std::min(
      kSpatialLatCells - 1,
      static_cast<int>(std::floor((qMaxLat + 90.0f) / kSpatialCellDeg)));
  const int lo0 = std::max(
      0, static_cast<int>(std::floor((qMinLon + 180.0f) / kSpatialCellDeg)));
  const int lo1 = std::min(
      kSpatialLonCells - 1,
      static_cast<int>(std::floor((qMaxLon + 180.0f) / kSpatialCellDeg)));
  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      const std::vector<std::size_t>& cell =
          grid[static_cast<std::size_t>(la * kSpatialLonCells + lo)];
      scratch.insert(scratch.end(), cell.begin(), cell.end());
    }
  }
  if (scratch.empty()) return;
  std::sort(scratch.begin(), scratch.end());
  scratch.erase(std::unique(scratch.begin(), scratch.end()), scratch.end());
  for (std::size_t idx : scratch) fn(idx);
}

LandDataStore::LandDataStore(std::string path) : path_(std::move(path)) {
  thread_ = std::thread([this] { load(); });
}

LandDataStore::~LandDataStore() {
  if (thread_.joinable()) thread_.join();
}

void LandDataStore::load() {
  std::ifstream in(path_, std::ios::binary);
  if (in.good()) {
    std::vector<std::uint8_t> buf(
        (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    Reader r{buf.data(), buf.data() + buf.size()};

    if (r.need(16) && r.u32() == kMagic) {
      const std::uint32_t fileVersion = r.u32();
      if (fileVersion < 1 || fileVersion > kVersionLatest) {
        loaded_.store(true, std::memory_order_release);
        return;
      }
      const std::uint32_t lineCount = r.u32();
      const std::uint32_t cityCount = r.u32();

      lines_.reserve(lineCount);
      lineBounds_.reserve(lineCount);
      for (std::uint32_t i = 0; i < lineCount && r.need(3); ++i) {
        MapLandLine line;
        line.landClass = lineClassFor(r.u8());
        const std::uint16_t npts = r.u16();
        if (!r.need(static_cast<std::size_t>(npts) * 8)) break;
        line.points.reserve(npts);
        Bounds b{90.0f, -90.0f, 180.0f, -180.0f};
        for (std::uint16_t k = 0; k < npts; ++k) {
          const float lat = r.f32();
          const float lon = r.f32();
          line.points.push_back({lat, lon});
          b.minLat = std::min(b.minLat, lat);
          b.maxLat = std::max(b.maxLat, lat);
          b.minLon = std::min(b.minLon, lon);
          b.maxLon = std::max(b.maxLon, lon);
        }
        lines_.push_back(std::move(line));
        lineBounds_.push_back(b);
      }

      silhouetteLandIdx_.reserve(8192);
      islandLandLowResIdx_.reserve(16384);
      regionalLandIdx_.reserve(64);
      detailLandIdx_.reserve(131072);
      coastIdx_.reserve(16384);
      constexpr float kSilhouetteLandSpanDeg = 30.0f;
      constexpr float kIslandLandMaxSpanDeg = 28.0f;
      constexpr float kRegionalLandMinSpanDeg = 20.0f;
      constexpr float kMaxSilhouetteSpanDeg = 180.0f;
      constexpr std::size_t kSilhouetteLandMaxPts = 8000;
      for (std::size_t i = 0; i < lines_.size(); ++i) {
        const LandClass c = lines_[i].landClass;
        if (c == LandClass::Coast) {
          coastIdx_.push_back(i);
          continue;
        }
        if (c != LandClass::LandMass) continue;
        const Bounds& b = lineBounds_[i];
        const float span = (b.maxLat - b.minLat) + (b.maxLon - b.minLon);
        const std::size_t npts = lines_[i].points.size();
        if (span >= kSilhouetteLandSpanDeg && npts <= kSilhouetteLandMaxPts &&
            span <= kMaxSilhouetteSpanDeg) {
          silhouetteLandIdx_.push_back(i);
        } else if (span <= kIslandLandMaxSpanDeg &&
                   npts <= kSilhouetteLandMaxPts) {
          islandLandLowResIdx_.push_back(i);
        } else if (span >= kRegionalLandMinSpanDeg &&
                   npts > kSilhouetteLandMaxPts) {
          regionalLandIdx_.push_back(i);
        } else {
          detailLandIdx_.push_back(i);
        }
      }

      islandCellIdx_.resize(
          static_cast<std::size_t>(kSpatialLatCells * kSpatialLonCells));
      detailCellIdx_.resize(
          static_cast<std::size_t>(kSpatialLatCells * kSpatialLonCells));
      for (std::size_t i : islandLandLowResIdx_) {
        addToSpatialCells(lineBounds_[i], i, islandCellIdx_);
      }
      for (std::size_t i : detailLandIdx_) {
        addToSpatialCells(lineBounds_[i], i, detailCellIdx_);
      }

      cities_.reserve(cityCount);
      for (std::uint32_t i = 0; i < cityCount && r.need(10); ++i) {
        MapLandCity city;
        city.lat = r.f32();
        city.lon = r.f32();
        city.rank = r.u8();
        if (fileVersion >= 2) {
          if (!r.need(1)) break;
          city.labelKind = static_cast<LandLabelKind>(r.u8());
        }
        const std::uint8_t nameLen = r.u8();
        if (!r.need(nameLen)) break;
        city.name.assign(reinterpret_cast<const char*>(r.p), nameLen);
        r.p += nameLen;
        cities_.push_back(std::move(city));
      }
    }
  }
  loaded_.store(true, std::memory_order_release);
}

std::vector<MapLandLine> LandDataStore::nearbyLines(
    double lat, double lon, float rangeNm, std::size_t maxCount) const {
  std::vector<MapLandLine> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));

  // Landmass fills must win the budget: they define the black-land / blue-ocean
  // base and are stored after borders in the asset, so a wide-class scan that
  // grabs borders first would otherwise starve them.
  const float nearRangeNm = std::min(rangeNm, kNearLandRangeNm);

  auto overlaps = [&](const Bounds& b, float queryRangeNm,
                      float marginScale) {
    const float latSpan =
        static_cast<float>((queryRangeNm / kNmPerDeg) * marginScale);
    const float dLat = latSpan;
    // Mercator stretches north/south; the top of a north-up chart sits at a
    // higher latitude than an equirectangular range circle would suggest, so
    // use the smallest cos(lat) across the query span for east/west reach.
    const double northCos =
        std::max(0.05, std::cos((lat + static_cast<double>(latSpan)) * kDegToRad));
    const double southCos =
        std::max(0.05, std::cos((lat - static_cast<double>(latSpan)) * kDegToRad));
    const double cosEdge = std::min({cosLat, northCos, southCos});
    const float dLon = static_cast<float>(
        (queryRangeNm / (kNmPerDeg * cosEdge)) * marginScale);
    return !(lat + dLat < b.minLat || lat - dLat > b.maxLat ||
             lon + dLon < b.minLon || lon - dLon > b.maxLon);
  };

  // Mercator landmass fills use straight chords between projected vertices.
  // Hemisphere-scale rings (e.g. Eurasia) can still pass the inflated bbox test
  // when centered over the Americas and then bogus-fill half the chart black.
  // Keep distant hemispheres out via centroid distance, but also admit any ring
  // whose bounding box actually contains the view center so panning toward the
  // Arctic (Americas centroid near the equator) still draws North America.
  auto lonInsideBounds = [](double lonDeg, float minLon, float maxLon) {
    const double span = static_cast<double>(maxLon) - minLon;
    if (span >= 360.0) return true;
    double d = lonDeg - minLon;
    while (d < 0.0) d += 360.0;
    while (d >= 360.0) d -= 360.0;
    return d <= span;
  };

  auto viewInsideBounds = [&](const Bounds& b) {
    if (lat < b.minLat || lat > b.maxLat) return false;
    return lonInsideBounds(lon, b.minLon, b.maxLon);
  };

  auto centroidWithinRange = [&](const Bounds& b, float queryRangeNm,
                                 float marginScale) {
    const double clat = (static_cast<double>(b.minLat) + b.maxLat) * 0.5;
    const double clon = (static_cast<double>(b.minLon) + b.maxLon) * 0.5;
    const double dLatNm = (clat - lat) * kNmPerDeg;
    double dLon = clon - lon;
    while (dLon > 180.0) dLon -= 360.0;
    while (dLon < -180.0) dLon += 360.0;
    const double midCos =
        std::max(0.05, std::cos((lat + clat) * 0.5 * kDegToRad));
    const double dLonNm = dLon * kNmPerDeg * midCos;
    const double distNm =
        std::sqrt(dLatNm * dLatNm + dLonNm * dLonNm);
    return distNm <= static_cast<double>(queryRangeNm) * marginScale;
  };

  constexpr float kSilhouetteLandSpanDeg = 30.0f;
  constexpr float kIslandLandMaxSpanDeg = 28.0f;
  constexpr std::size_t kSilhouetteLandMaxPts = 8000;

  auto landMassRelevant = [&](const Bounds& b, float span, float queryRangeNm,
                              float marginScale) {
    if (span <= kIslandLandMaxSpanDeg) {
      return overlaps(b, queryRangeNm, marginScale);
    }
    if (viewInsideBounds(b)) return true;
    if (centroidWithinRange(b, queryRangeNm, marginScale)) return true;
    // Mercator panning can place the pointer over water while a continental
    // landmass still fills much of the chart (Mexico visible while centered
    // off Ecuador). Use center-latitude lon reach — not the polar-stretched
    // overlaps() box — so Eurasia does not pass when centered over Hudson.
    constexpr float kContinentalLandSpanDeg = 40.0f;
    if (span < kContinentalLandSpanDeg) return false;
    const float latSpan =
        static_cast<float>((queryRangeNm / kNmPerDeg) * marginScale);
    if (lat + latSpan < b.minLat || lat - latSpan > b.maxLat) return false;
    const float dLon = static_cast<float>(
        (queryRangeNm / (kNmPerDeg * cosLat)) * marginScale);
    return !(lon + dLon < b.minLon || lon - dLon > b.maxLon);
  };

  constexpr std::size_t kLakeWideBudget = 800;
  constexpr std::size_t kBorderBudget = 3500;

  auto landMassBudgetFor = [](float queryRangeNm, bool closeDetail) -> std::size_t {
    if (closeDetail) {
      if (queryRangeNm <= 2.5f) return 140;
      if (queryRangeNm <= 5.0f) return 120;
      if (queryRangeNm <= 15.0f) return 100;
      return 60;
    }
    return 0;
  };

  struct LandCand {
    std::size_t idx = 0;
    float span = 0.0f;
    float dist2 = 0.0f;
  };
  std::vector<LandCand> continentalCands;
  std::vector<LandCand> islandCands;
  std::vector<LandCand> regionalCands;
  std::vector<LandCand> detailCands;
  continentalCands.reserve(32);
  islandCands.reserve(4096);
  regionalCands.reserve(32);
  detailCands.reserve(512);
  const bool continentalLandQuery = rangeNm >= 500.0f;
  const bool closeDetailQuery = rangeNm <= kDetailLandMaxRangeNm;

  auto islandBudgetFor = [](float queryRangeNm) -> std::size_t {
    if (queryRangeNm >= 500.0f) return 450;
    if (queryRangeNm >= 150.0f) return 320;
    if (queryRangeNm >= 50.0f) return 220;
    // Regional lon-band fills carry the peninsula at 25–120 NM; below 25 NM
    // keep a full island budget so coastal keys/islands beat micro artifacts.
    if (queryRangeNm >= 25.0f) return 80;
    return 200;
  };

  auto minIslandSpanFor = [](float queryRangeNm) -> float {
    // GSHHG decimation leaves tiny artifact rings whose centroids sit on the
    // map center and would otherwise consume the entire island budget.
    if (queryRangeNm >= 500.0f) return 0.35f;
    if (queryRangeNm >= 150.0f) return 0.15f;
    if (queryRangeNm >= 50.0f) return 0.025f;
    if (queryRangeNm >= 25.0f) return 0.008f;
    return 0.012f;
  };

  auto considerLand = [&](std::size_t i) {
    if (!overlaps(lineBounds_[i], rangeNm, kWideLandOverlapScale)) return;
    const Bounds& b = lineBounds_[i];
    const float span = (b.maxLat - b.minLat) + (b.maxLon - b.minLon);
    if (!landMassRelevant(b, span, rangeNm, kWideLandOverlapScale)) {
      return;
    }
    const double clat = (b.minLat + b.maxLat) * 0.5;
    const double clon = (b.minLon + b.maxLon) * 0.5;
    const double dlat = clat - lat;
    const double dlon = (clon - lon) * cosLat;
    LandCand cand{
        i, span, static_cast<float>(dlat * dlat + dlon * dlon)};
    const std::size_t npts = lines_[i].points.size();
    const bool isSilhouette =
        span >= kSilhouetteLandSpanDeg && npts <= kSilhouetteLandMaxPts;
    const bool isIslandLowRes =
        span <= kIslandLandMaxSpanDeg && npts <= kSilhouetteLandMaxPts;
    if (isSilhouette) {
      continentalCands.push_back(cand);
    } else if (isIslandLowRes) {
      if (span >= minIslandSpanFor(rangeNm)) {
        islandCands.push_back(cand);
      }
    } else {
      detailCands.push_back(cand);
    }
  };

  auto considerRegional = [&](std::size_t i) {
    if (!overlaps(lineBounds_[i], rangeNm, kWideLandOverlapScale)) return;
    const Bounds& b = lineBounds_[i];
    const float span = (b.maxLat - b.minLat) + (b.maxLon - b.minLon);
    if (!landMassRelevant(b, span, rangeNm, kWideLandOverlapScale)) {
      return;
    }
    const double clat = (b.minLat + b.maxLat) * 0.5;
    const double clon = (b.minLon + b.maxLon) * 0.5;
    const double dlat = clat - lat;
    const double dlon = (clon - lon) * cosLat;
    regionalCands.push_back(
        {i, span, static_cast<float>(dlat * dlat + dlon * dlon)});
  };

  for (std::size_t i : silhouetteLandIdx_) {
    if (!closeDetailQuery) {
      considerLand(i);
    } else if (viewInsideBounds(lineBounds_[i])) {
      // Skip the coarse continental silhouette whenever regional lon-bands
      // are active; they layer over the silhouette at draw time when needed.
      const bool regionalActive =
          rangeNm >= kRegionalLandMinRangeNm &&
          rangeNm <= kRegionalLandMaxRangeNm;
      if (!(regionalActive && rangeNm <= 15.0f)) {
        considerLand(i);
      }
    }
  }
  static thread_local std::vector<std::size_t> spatialScratch;
  if (closeDetailQuery) {
    forEachSpatialIndex(lat, lon, rangeNm, 1.2f, detailCellIdx_, spatialScratch,
                        considerLand);
    forEachSpatialIndex(lat, lon, rangeNm, 1.2f, islandCellIdx_, spatialScratch,
                        considerLand);
  } else {
    for (std::size_t i : islandLandLowResIdx_) considerLand(i);
  }
  if (rangeNm >= kRegionalLandMinRangeNm && rangeNm <= kRegionalLandMaxRangeNm) {
    for (std::size_t i : regionalLandIdx_) considerRegional(i);
  }
  std::sort(continentalCands.begin(), continentalCands.end(),
            [](const LandCand& a, const LandCand& b) {
              if (a.dist2 != b.dist2) return a.dist2 < b.dist2;
              return a.span > b.span;
            });
  std::sort(islandCands.begin(), islandCands.end(),
            [rangeNm](const LandCand& a, const LandCand& b) {
              // Regional zoom: large islands (Cuba, Hispaniola) must beat the
              // hundreds of micro artifact rings near the map center.
              if (rangeNm >= 150.0f && rangeNm < 500.0f) {
                if (a.span != b.span) return a.span > b.span;
                return a.dist2 < b.dist2;
              }
              // Close range: same problem — span-first keeps Sanibel-scale
              // islands ahead of 4-point GSHHG noise near ownship.
              if (rangeNm < 50.0f) {
                if (a.span != b.span) return a.span > b.span;
                return a.dist2 < b.dist2;
              }
              if (a.dist2 != b.dist2) return a.dist2 < b.dist2;
              return a.span > b.span;
            });
  std::sort(regionalCands.begin(), regionalCands.end(),
            [](const LandCand& a, const LandCand& b) {
              if (a.dist2 != b.dist2) return a.dist2 < b.dist2;
              return a.span > b.span;
            });
  if (continentalLandQuery) {
    std::sort(detailCands.begin(), detailCands.end(),
              [](const LandCand& a, const LandCand& b) {
                if (a.span != b.span) return a.span > b.span;
                return a.dist2 < b.dist2;
              });
  } else if (closeDetailQuery && rangeNm <= 15.0f) {
    std::sort(detailCands.begin(), detailCands.end(),
              [](const LandCand& a, const LandCand& b) {
                if (a.span != b.span) return a.span > b.span;
                return a.dist2 < b.dist2;
              });
  } else {
    std::sort(detailCands.begin(), detailCands.end(),
              [](const LandCand& a, const LandCand& b) {
                if (a.dist2 != b.dist2) return a.dist2 < b.dist2;
                return a.span < b.span;
              });
  }
  for (const LandCand& cand : continentalCands) {
    if (result.size() >= maxCount) break;
    result.push_back(lines_[cand.idx]);
  }
  for (const LandCand& cand : regionalCands) {
    if (result.size() >= maxCount) break;
    result.push_back(lines_[cand.idx]);
  }
  const std::size_t islandBudget = islandBudgetFor(rangeNm);
  for (std::size_t n = 0;
       n < islandCands.size() && n < islandBudget && result.size() < maxCount;
       ++n) {
    result.push_back(lines_[islandCands[n].idx]);
  }
  const std::size_t detailBudget = landMassBudgetFor(rangeNm, closeDetailQuery);
  for (std::size_t n = 0;
       n < detailCands.size() && n < detailBudget && result.size() < maxCount;
       ++n) {
    result.push_back(lines_[detailCands[n].idx]);
  }

  if (rangeNm >= kChartLakeMinRangeNm) {
    std::size_t lakeCount = 0;
    for (std::size_t i = 0; i < lines_.size(); ++i) {
      if (lines_[i].landClass != LandClass::Lake) continue;
      if (!overlaps(lineBounds_[i], rangeNm, kWideLandOverlapScale)) continue;
      if (rangeNm >= kMapRangeMaxNm) {
        const Bounds& b = lineBounds_[i];
        const float span = (b.maxLat - b.minLat) + (b.maxLon - b.minLon);
        if (span < kWideChartLakeMinSpanDeg) continue;
      }
      result.push_back(lines_[i]);
      if (++lakeCount >= kLakeWideBudget || result.size() >= maxCount) {
        break;
      }
    }
    if (result.size() >= maxCount) return result;
  }

  struct BorderCand {
    std::size_t idx = 0;
    float dist2 = 0.0f;
    float span = 0.0f;
  };
  std::vector<BorderCand> borderCands;
  borderCands.reserve(512);
  for (std::size_t i = 0; i < lines_.size(); ++i) {
    if (lines_[i].landClass != LandClass::Border) continue;
    if (!overlaps(lineBounds_[i], rangeNm, kWideLandOverlapScale)) continue;
    const Bounds& b = lineBounds_[i];
    const double clat = (b.minLat + b.maxLat) * 0.5;
    const double clon = (b.minLon + b.maxLon) * 0.5;
    const double dlat = clat - lat;
    const double dlon = (clon - lon) * cosLat;
    const float span =
        (b.maxLat - b.minLat) + (b.maxLon - b.minLon);
    borderCands.push_back(
        {i, static_cast<float>(dlat * dlat + dlon * dlon), span});
  }
  const bool continentalQuery = rangeNm >= 500.0f;
  const bool borderOverBudget = borderCands.size() > kBorderBudget;
  if (continentalQuery && borderOverBudget) {
    std::sort(borderCands.begin(), borderCands.end(),
              [](const BorderCand& a, const BorderCand& b) {
                if (a.span != b.span) return a.span > b.span;
                return a.dist2 < b.dist2;
              });
  } else {
    std::sort(borderCands.begin(), borderCands.end(),
              [](const BorderCand& a, const BorderCand& b) {
                return a.dist2 < b.dist2;
              });
  }
  for (std::size_t n = 0;
       n < borderCands.size() && n < kBorderBudget && result.size() < maxCount;
       ++n) {
    result.push_back(lines_[borderCands[n].idx]);
  }
  if (result.size() >= maxCount) return result;

  for (std::size_t i = 0; i < lines_.size(); ++i) {
    const LandClass c = lines_[i].landClass;
    if (c != LandClass::StateBorder) continue;
    if (rangeNm > kStateBorderQueryMaxRangeNm) continue;
    if (!overlaps(lineBounds_[i], rangeNm, kWideLandOverlapScale)) continue;
    result.push_back(lines_[i]);
    if (result.size() >= maxCount) return result;
  }

  for (std::size_t i = 0; i < lines_.size(); ++i) {
    const LandClass c = lines_[i].landClass;
    if (isWideLandClass(c)) continue;
    if (rangeNm >= kChartLakeMinRangeNm && c == LandClass::Lake) continue;
    if (!overlaps(lineBounds_[i], nearRangeNm, 1.2f)) continue;
    result.push_back(lines_[i]);
    if (result.size() >= maxCount) return result;
  }
  return result;
}

std::vector<MapLandCity> LandDataStore::nearbyCities(
    double lat, double lon, float rangeNm, std::size_t maxCount) const {
  std::vector<MapLandCity> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const float latSpan =
      static_cast<float>((rangeNm / kNmPerDeg) * kWideLandOverlapScale);
  const double northCos =
      std::max(0.05, std::cos((lat + static_cast<double>(latSpan)) * kDegToRad));
  const double southCos =
      std::max(0.05, std::cos((lat - static_cast<double>(latSpan)) * kDegToRad));
  const double cosEdge = std::min({cosLat, northCos, southCos});
  const double dLat = latSpan;
  const double dLon = (rangeNm / (kNmPerDeg * cosEdge)) * kWideLandOverlapScale;

  auto maxLabelRangeNm = [](const MapLandCity& label) -> float {
    switch (label.labelKind) {
      case LandLabelKind::Hydro:
        if (label.rank >= 10) return 2000.0f;
        if (label.rank >= 8) return kLandQueryRangeNm;
        if (label.rank >= 6) return 500.0f;
        if (label.rank >= 5) return 200.0f;
        return 25.0f;
      case LandLabelKind::Region:
        if (label.rank >= 10) return 2000.0f;
        if (label.rank >= 8) return kLandQueryRangeNm;
        if (label.rank >= 6) return 200.0f;
        return 100.0f;
      case LandLabelKind::City:
        break;
    }
    if (label.rank >= 9) return 500.0f;
    if (label.rank >= 8) return 150.0f;
    if (label.rank >= 4) return 50.0f;
    return 20.0f;
  };

  std::vector<MapLandCity> candidates;
  candidates.reserve(512);
  for (const MapLandCity& city : cities_) {
    if (std::fabs(city.lat - lat) > dLat || std::fabs(city.lon - lon) > dLon) {
      continue;
    }
    if (rangeNm > maxLabelRangeNm(city)) continue;
    if (rangeNm >= 1000.0f) {
      if (city.labelKind == LandLabelKind::City && city.rank < 8) continue;
      if (city.labelKind == LandLabelKind::Hydro && city.rank < 8) continue;
      if (city.labelKind == LandLabelKind::Region && city.rank < 8) continue;
    } else if (rangeNm > 500.0f) {
      if (city.labelKind == LandLabelKind::City && city.rank < 8) continue;
      if (city.labelKind == LandLabelKind::Hydro && city.rank < 6) continue;
      if (city.labelKind == LandLabelKind::Region && city.rank < 8) continue;
    } else if (rangeNm > 200.0f) {
      if (city.labelKind == LandLabelKind::Region && city.rank < 6) continue;
    }
    candidates.push_back(city);
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const MapLandCity& a, const MapLandCity& b) {
              if (a.rank != b.rank) return a.rank > b.rank;
              return a.name < b.name;
            });
  if (candidates.size() > maxCount) candidates.resize(maxCount);
  return candidates;
}

}  // namespace avionics
