#include "DsfTerrainStore.h"

#include "XPlaneInstall.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#endif

namespace avionics {
namespace {

constexpr double kMetersToFeet = 3.280839895013123;
constexpr char kDsfMagic[] = "XPLNEDSF";
constexpr std::uint32_t kDsfFooterBytes = 16;
constexpr std::uint8_t k7zMagic[] = {0x37, 0x7A, 0xBC, 0xAF, 0x27, 0x1C};

bool tagIs(const char* disk, const char* logical) {
  // DSF atom IDs are uint32 constants; on little-endian hosts most tags are
  // stored byte-reversed (GEOD -> DOEG), but some (PORP) appear as literal
  // four-character strings in the file.
  if (std::memcmp(disk, logical, 4) == 0) return true;
  return disk[0] == logical[3] && disk[1] == logical[2] &&
         disk[2] == logical[1] && disk[3] == logical[0];
}

std::uint32_t readU32(const std::uint8_t* p) {
  std::uint32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

float readF32(const std::uint8_t* p) {
  float v = 0.0f;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::int16_t readI16(const std::uint8_t* p) {
  std::int16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

int floorLatSouth(double lat) {
  return static_cast<int>(std::floor(lat));
}

// DSF file +LAT-LON uses the tile's western/southern integer edge. For western
// hemisphere longitudes the LON component is ceil(|lon|) (e.g. -122.4° -> 123).
int tileLonIndex(double lon) {
  if (lon < 0.0) {
    return static_cast<int>(std::ceil(-lon - 1e-9));
  }
  return static_cast<int>(std::floor(lon));
}

int latBucket(double lat) {
  return static_cast<int>(std::floor(lat / 10.0)) * 10;
}

int lonBucket(double lon) {
  return static_cast<int>(std::ceil(std::abs(lon) / 10.0)) * 10;
}

std::string joinPath(const std::string& dir, const std::string& leaf) {
#if defined(_WIN32)
  const char sep = '\\';
#else
  const char sep = '/';
#endif
  if (dir.empty()) return leaf;
  if (dir.back() == '/' || dir.back() == '\\') return dir + leaf;
  return dir + sep + leaf;
}

bool startsWith7z(const std::uint8_t* data, std::size_t n) {
  return n >= sizeof(k7zMagic) &&
         std::memcmp(data, k7zMagic, sizeof(k7zMagic)) == 0;
}

std::vector<std::uint8_t> readFileBytes(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  in.seekg(0, std::ios::end);
  const std::streamoff size = in.tellg();
  if (size <= 0) return {};
  in.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> out(static_cast<std::size_t>(size));
  if (!in.read(reinterpret_cast<char*>(out.data()), size)) return {};
  return out;
}

std::vector<std::uint8_t> decompress7zToStdout(const std::string& path) {
  const char* tools[] = {"bsdtar", "tar"};
  for (const char* tool : tools) {
    std::string cmd;
#if defined(_WIN32)
    cmd = std::string(tool) + " -xOf \"" + path + "\"";
    FILE* pipe = _popen(cmd.c_str(), "rb");
#else
    cmd = std::string("/usr/bin/") + tool + " -xOf '" + path + "' 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      cmd = std::string(tool) + " -xOf '" + path + "' 2>/dev/null";
      pipe = popen(cmd.c_str(), "r");
    }
#endif
    if (!pipe) continue;

    std::vector<std::uint8_t> out;
    std::array<char, 65536> buf{};
    std::size_t n = 0;
    while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
      const auto prev = out.size();
      out.resize(prev + n);
      std::memcpy(out.data() + prev, buf.data(), n);
    }
#if defined(_WIN32)
    _pclose(pipe);
#else
    pclose(pipe);
#endif
    if (out.size() >= 12 && std::memcmp(out.data(), kDsfMagic, 8) == 0) {
      return out;
    }
  }
  return {};
}

std::vector<std::uint8_t> loadDsfBytes(const std::string& path) {
  std::vector<std::uint8_t> raw = readFileBytes(path);
  if (raw.empty()) return raw;
  if (startsWith7z(raw.data(), raw.size())) {
    return decompress7zToStdout(path);
  }
  if (raw.size() >= 8 && std::memcmp(raw.data(), kDsfMagic, 8) == 0) {
    return raw;
  }
  return {};
}

bool parsePropDoubles(const std::uint8_t* props, std::size_t size, double& west,
                      double& east, double& south, double& north) {
  bool haveWest = false, haveEast = false, haveSouth = false, haveNorth = false;
  std::size_t i = 0;
  while (i < size) {
    const char* key = reinterpret_cast<const char*>(props + i);
    const std::size_t keyLen = std::strlen(key);
    if (keyLen == 0) break;
    i += keyLen + 1;
    if (i >= size) break;
    const char* val = reinterpret_cast<const char*>(props + i);
    const std::size_t valLen = std::strlen(val);
    if (valLen == 0) break;
    if (std::strcmp(key, "sim/west") == 0) {
      west = std::atof(val);
      haveWest = true;
    } else if (std::strcmp(key, "sim/east") == 0) {
      east = std::atof(val);
      haveEast = true;
    } else if (std::strcmp(key, "sim/south") == 0) {
      south = std::atof(val);
      haveSouth = true;
    } else if (std::strcmp(key, "sim/north") == 0) {
      north = std::atof(val);
      haveNorth = true;
    }
    i += valLen + 1;
  }
  return haveWest && haveEast && haveSouth && haveNorth;
}

const std::uint8_t* findAtomPayload(const std::uint8_t* data, std::size_t start,
                                    std::size_t end, const char* tag,
                                    std::size_t& payloadSize) {
  std::size_t off = start;
  while (off + 8 <= end) {
    const char* disk = reinterpret_cast<const char*>(data + off);
    const std::uint32_t size = readU32(data + off + 4);
    if (size < 8 || off + size > end) return nullptr;
    const std::size_t payloadStart = off + 8;
    const std::size_t payloadEnd = off + size;
    if (tagIs(disk, tag)) {
      payloadSize = size - 8;
      return data + payloadStart;
    }
    if (tagIs(disk, "HEAD") || tagIs(disk, "DAEH") || tagIs(disk, "DEFN") ||
        tagIs(disk, "DEMS")) {
      const std::uint8_t* nested = findAtomPayload(
          data, payloadStart, payloadEnd, tag, payloadSize);
      if (nested != nullptr) return nested;
    }
    off += size;
  }
  return nullptr;
}

std::vector<std::string> parseStringTable(const std::uint8_t* data,
                                          std::size_t size) {
  std::vector<std::string> out;
  std::size_t i = 0;
  while (i < size) {
    const char* s = reinterpret_cast<const char*>(data + i);
    const std::size_t len = std::strlen(s);
    if (len == 0) break;
    out.emplace_back(s);
    i += len + 1;
  }
  return out;
}

struct DemLayer {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint8_t bytesPerPixel = 0;
  std::uint16_t flags = 0;
  float scale = 1.0f;
  float offset = 0.0f;
  std::vector<std::uint8_t> pixels;
};

constexpr std::uint16_t kDemiDataMask = 0x0003;
constexpr std::uint16_t kDemiDataFp32 = 0;
constexpr std::uint16_t kDemiDataSint = 1;
constexpr std::uint16_t kDemiDataUint = 2;

bool parseDemi(const std::uint8_t* payload, std::size_t size, DemLayer& out) {
  if (size < 20) return false;
  out.bytesPerPixel = payload[1];
  out.flags = static_cast<std::uint16_t>(payload[2] |
                                         (static_cast<std::uint16_t>(payload[3])
                                          << 8));
  out.width = readU32(payload + 4);
  out.height = readU32(payload + 8);
  out.scale = readF32(payload + 12);
  out.offset = readF32(payload + 16);
  return out.width > 0 && out.height > 0 && out.bytesPerPixel > 0;
}

bool parseDemsLayers(const std::uint8_t* data, std::size_t start,
                     std::size_t end, std::vector<DemLayer>& layers) {
  std::size_t off = start;
  DemLayer current;
  bool haveDemi = false;
  while (off + 8 <= end) {
    const char* disk = reinterpret_cast<const char*>(data + off);
    const std::uint32_t size = readU32(data + off + 4);
    if (size < 8 || off + size > end) return false;
    const std::uint8_t* payload = data + off + 8;
    const std::size_t payloadSize = size - 8;
    if (tagIs(disk, "DEMI")) {
      current = DemLayer{};
      if (!parseDemi(payload, payloadSize, current)) return false;
      haveDemi = true;
    } else if (tagIs(disk, "DEMD")) {
      if (!haveDemi) return false;
      current.pixels.assign(payload, payload + payloadSize);
      layers.push_back(std::move(current));
      haveDemi = false;
    }
    off += size;
  }
  return !layers.empty();
}

double samplePixel(const DemLayer& layer, std::uint32_t row,
                   std::uint32_t col) {
  const std::uint32_t w = layer.width;
  const std::uint32_t h = layer.height;
  if (row >= h || col >= w) return 0.0;
  const std::size_t idx = static_cast<std::size_t>(row) * w + col;
  const std::uint8_t* base = layer.pixels.data();
  double v = 0.0;
  switch (layer.flags & kDemiDataMask) {
    case kDemiDataFp32: {
      if (layer.bytesPerPixel != 4) return 0.0;
      v = static_cast<double>(readF32(base + idx * 4));
      break;
    }
    case kDemiDataSint:
      if (layer.bytesPerPixel == 1) {
        std::int8_t raw = 0;
        std::memcpy(&raw, base + idx, 1);
        v = static_cast<double>(raw);
      } else if (layer.bytesPerPixel == 2) {
        v = static_cast<double>(readI16(base + idx * 2));
      } else if (layer.bytesPerPixel == 4) {
        std::int32_t raw = 0;
        std::memcpy(&raw, base + idx * 4, 4);
        v = static_cast<double>(raw);
      } else {
        return 0.0;
      }
      break;
    case kDemiDataUint:
      if (layer.bytesPerPixel == 1) {
        v = static_cast<double>(base[idx]);
      } else if (layer.bytesPerPixel == 2) {
        std::uint16_t raw = 0;
        std::memcpy(&raw, base + idx * 2, 2);
        v = static_cast<double>(raw);
      } else if (layer.bytesPerPixel == 4) {
        std::uint32_t raw = 0;
        std::memcpy(&raw, base + idx * 4, 4);
        v = static_cast<double>(raw);
      } else {
        return 0.0;
      }
      break;
    default:
      return 0.0;
  }
  return v * static_cast<double>(layer.scale) +
         static_cast<double>(layer.offset);
}

struct DemTile {
  int southLat = 0;
  int lonIndex = 0;
  double west = 0.0;
  double east = 0.0;
  double south = 0.0;
  double north = 0.0;
  DemLayer elevation;

  double elevationMeters(double lat, double lon) const {
    const std::uint32_t w = elevation.width;
    const std::uint32_t h = elevation.height;
    if (w < 2 || h < 2) return 0.0;
    const double colF =
        (lon - west) / (east - west) * static_cast<double>(w - 1);
    const double rowF =
        (north - lat) / (north - south) * static_cast<double>(h - 1);
    const int c0 = static_cast<int>(std::floor(colF));
    const int r0 = static_cast<int>(std::floor(rowF));
    const int c1 = std::min(c0 + 1, static_cast<int>(w) - 1);
    const int r1 = std::min(r0 + 1, static_cast<int>(h) - 1);
    const double fc = colF - static_cast<double>(c0);
    const double fr = rowF - static_cast<double>(r0);
    const double v00 = samplePixel(elevation, static_cast<std::uint32_t>(r0),
                                   static_cast<std::uint32_t>(c0));
    const double v10 = samplePixel(elevation, static_cast<std::uint32_t>(r0),
                                   static_cast<std::uint32_t>(c1));
    const double v01 = samplePixel(elevation, static_cast<std::uint32_t>(r1),
                                   static_cast<std::uint32_t>(c0));
    const double v11 = samplePixel(elevation, static_cast<std::uint32_t>(r1),
                                   static_cast<std::uint32_t>(c1));
    const double v0 = v00 * (1.0 - fc) + v10 * fc;
    const double v1 = v01 * (1.0 - fc) + v11 * fc;
    return v0 * (1.0 - fr) + v1 * fr;
  }
};

std::unique_ptr<DemTile> loadTileFromDsf(const std::vector<std::uint8_t>& dsf) {
  if (dsf.size() < 12 || std::memcmp(dsf.data(), kDsfMagic, 8) != 0) {
    return nullptr;
  }
  const std::size_t atomEnd =
      dsf.size() > kDsfFooterBytes ? dsf.size() - kDsfFooterBytes : dsf.size();
  const std::size_t atomStart = 12;

  std::size_t propSize = 0;
  const std::uint8_t* propPayload =
      findAtomPayload(dsf.data(), atomStart, atomEnd, "PORP", propSize);
  if (propPayload == nullptr) return nullptr;

  auto tile = std::make_unique<DemTile>();
  if (!parsePropDoubles(propPayload, propSize, tile->west, tile->east,
                        tile->south, tile->north)) {
    return nullptr;
  }

  std::size_t demnSize = 0;
  const std::uint8_t* demnPayload =
      findAtomPayload(dsf.data(), atomStart, atomEnd, "DEMN", demnSize);
  if (demnPayload == nullptr) return nullptr;
  const std::vector<std::string> layerNames =
      parseStringTable(demnPayload, demnSize);
  int elevIndex = -1;
  for (std::size_t i = 0; i < layerNames.size(); ++i) {
    if (layerNames[i] == "elevation") {
      elevIndex = static_cast<int>(i);
      break;
    }
  }
  if (elevIndex < 0) return nullptr;

  std::size_t off = atomStart;
  std::size_t demsPayloadStart = 0;
  std::size_t demsPayloadEnd = 0;
  while (off + 8 <= atomEnd) {
    const char* disk = reinterpret_cast<const char*>(dsf.data() + off);
    const std::uint32_t size = readU32(dsf.data() + off + 4);
    if (size < 8 || off + size > atomEnd) break;
    if (tagIs(disk, "DEMS")) {
      demsPayloadStart = off + 8;
      demsPayloadEnd = off + size;
      break;
    }
    off += size;
  }
  if (demsPayloadEnd <= demsPayloadStart) return nullptr;

  std::vector<DemLayer> layers;
  if (!parseDemsLayers(dsf.data(), demsPayloadStart, demsPayloadEnd, layers)) {
    return nullptr;
  }
  if (static_cast<std::size_t>(elevIndex) >= layers.size()) return nullptr;

  tile->elevation = std::move(layers[static_cast<std::size_t>(elevIndex)]);
  tile->southLat = static_cast<int>(std::floor(tile->south));
  tile->lonIndex = tileLonIndex(tile->west < 0.0 ? tile->west : tile->east);
  return tile;
}

std::string formatTileFile(int southLat, int lonIndex) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%+d-%03d.dsf", southLat, lonIndex);
  return buf;
}

std::string formatTileFolder(int latB, int lonB) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%+03d-%03d", latB, lonB);
  return buf;
}

}  // namespace

struct DsfTerrainStore::TileCacheEntry {
  int southLat = 0;
  int lonIndex = 0;
  std::chrono::steady_clock::time_point lastHit{};
  DemTile dem;
};

DsfTerrainStore::DsfTerrainStore() {
  earthNavDir_ = xplane_install::earthNavDataDir();
  cache_.reserve(kMaxCachedTiles);
}

DsfTerrainStore::~DsfTerrainStore() {
  for (TileCacheEntry* e : cache_) delete e;
}

std::string DsfTerrainStore::tilePath(double lat, double lon) const {
  if (earthNavDir_.empty()) return {};
  const int south = floorLatSouth(lat);
  const int lonIdx = tileLonIndex(lon);
  const std::string folder =
      joinPath(earthNavDir_, formatTileFolder(latBucket(lat), lonBucket(lon)));
  return joinPath(folder, formatTileFile(south, lonIdx));
}

const DsfTerrainStore::TileCacheEntry* DsfTerrainStore::tileFor(
    double lat, double lon) const {
  if (earthNavDir_.empty()) return nullptr;

  const int south = floorLatSouth(lat);
  const int lonIdx = tileLonIndex(lon);

  const auto now = std::chrono::steady_clock::now();

  // LRU hit: move the entry to the front so wide rasters that touch many tiles
  // keep their working set resident.
  for (std::size_t i = 0; i < cache_.size(); ++i) {
    TileCacheEntry* e = cache_[i];
    if (e->southLat == south && e->lonIndex == lonIdx) {
      e->lastHit = now;
      if (i != 0) {
        cache_.erase(cache_.begin() + static_cast<std::ptrdiff_t>(i));
        cache_.insert(cache_.begin(), e);
      }
      return e;
    }
  }

  // Known-absent tile (ocean / not installed): don't touch the filesystem
  // again for it.
  for (const std::pair<int, int>& m : misses_) {
    if (m.first == south && m.second == lonIdx) return nullptr;
  }

  // Throttle loads: tile loads can take long (7z decompress via subprocess),
  // and one raster pass can request many uncached tiles. Allow one load per
  // cooldown window; everything else falls back and retries on a later frame.
  if (now - lastLoad_ < std::chrono::milliseconds(20)) return nullptr;

  // With the cache full of tiles that were all hit within the last frame or
  // two, the visible footprint is bigger than the cache. Evicting would just
  // thrash (reload the same tiles every frame), so leave the working set
  // resident and let out-of-cache areas keep the procedural fallback.
  if (cache_.size() >= static_cast<std::size_t>(kMaxCachedTiles) &&
      now - cache_.back()->lastHit < std::chrono::milliseconds(250)) {
    return nullptr;
  }

  lastLoad_ = now;

  const std::string path = tilePath(lat, lon);
  std::unique_ptr<::avionics::DemTile> loaded;
  if (!path.empty()) {
    const std::vector<std::uint8_t> bytes = loadDsfBytes(path);
    if (!bytes.empty()) loaded = loadTileFromDsf(bytes);
  }
  if (!loaded) {
    if (misses_.size() >= static_cast<std::size_t>(kMaxMissEntries)) {
      misses_.erase(misses_.begin());
    }
    misses_.push_back({south, lonIdx});
    return nullptr;
  }

  auto* slot = new TileCacheEntry{};
  slot->southLat = south;
  slot->lonIndex = lonIdx;
  slot->lastHit = now;
  slot->dem = std::move(*loaded);

  if (cache_.size() >= static_cast<std::size_t>(kMaxCachedTiles)) {
    delete cache_.back();
    cache_.pop_back();
  }
  cache_.insert(cache_.begin(), slot);
  return slot;
}

float DsfTerrainStore::elevationFt(double lat, double lon) const {
  const TileCacheEntry* tile = tileFor(lat, lon);
  if (tile == nullptr) return fallback_.elevationFt(lat, lon);
  return static_cast<float>(tile->dem.elevationMeters(lat, lon) * kMetersToFeet);
}

}  // namespace avionics
