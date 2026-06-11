#include "LandDataStore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <utility>

namespace avionics {
namespace {

constexpr double kNmPerDeg = 60.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Roads/rivers/lakes are decluttered past ~150 NM in the renderer, so there is
// no point returning them from a continental-scale query; pulling them only
// from this tight inner radius keeps the wide query's line budget for the
// classes that actually draw at range (nation/state borders, coastlines).
constexpr float kNearLandRangeNm = 160.0f;

// Lines that remain meaningful across a continental view and so are always
// collected out to the full query range.
bool isWideLandClass(LandClass c) {
  return c == LandClass::Border || c == LandClass::StateBorder ||
         c == LandClass::Coast;
}

constexpr std::uint32_t kMagic = 0x444C5641;  // "AVLD"
constexpr std::uint32_t kVersion = 1;

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

    if (r.need(16) && r.u32() == kMagic && r.u32() == kVersion) {
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

      cities_.reserve(cityCount);
      for (std::uint32_t i = 0; i < cityCount && r.need(10); ++i) {
        MapLandCity city;
        city.lat = r.f32();
        city.lon = r.f32();
        city.rank = r.u8();
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

  // Wide classes (borders/coastlines) are collected to the full query range;
  // near classes (roads/rivers/lakes) only out to kNearLandRangeNm. Two passes
  // so a road-dense area cannot exhaust the line budget before the borders that
  // define a continental view are ever reached (the asset stores roads before
  // borders, so a single in-order scan would starve them).
  const float nearRangeNm = std::min(rangeNm, kNearLandRangeNm);

  // Each group gets its own budget so the (large) wide set cannot exhaust the
  // shared cap and starve the near set, or vice versa.
  auto collect = [&](bool wantWide, float queryRangeNm) {
    const float dLat = static_cast<float>((queryRangeNm / kNmPerDeg) * 1.2);
    const float dLon =
        static_cast<float>((queryRangeNm / (kNmPerDeg * cosLat)) * 1.2);
    std::size_t added = 0;
    for (std::size_t i = 0; i < lines_.size(); ++i) {
      if (isWideLandClass(lines_[i].landClass) != wantWide) continue;
      const Bounds& b = lineBounds_[i];
      if (lat + dLat < b.minLat || lat - dLat > b.maxLat ||
          lon + dLon < b.minLon || lon - dLon > b.maxLon) {
        continue;
      }
      result.push_back(lines_[i]);
      if (++added >= maxCount) return;
    }
  };

  collect(/*wantWide=*/true, rangeNm);
  collect(/*wantWide=*/false, nearRangeNm);
  return result;
}

std::vector<MapLandCity> LandDataStore::nearbyCities(
    double lat, double lon, float rangeNm, std::size_t maxCount) const {
  std::vector<MapLandCity> result;
  if (!loaded() || maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  for (const MapLandCity& city : cities_) {
    if (std::fabs(city.lat - lat) > dLat || std::fabs(city.lon - lon) > dLon) {
      continue;
    }
    result.push_back(city);
    if (result.size() >= maxCount) break;
  }
  return result;
}

}  // namespace avionics
