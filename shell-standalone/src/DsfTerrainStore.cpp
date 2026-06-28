#include "DsfTerrainStore.h"

#include "XPlaneInstall.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
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

#if defined(_WIN32)
std::wstring utf8ToWide(const std::string& s) {
  if (s.empty()) return {};
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(),
                                    static_cast<int>(s.size()), nullptr, 0);
  if (n <= 0) return {};
  std::wstring w(static_cast<std::size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()),
                      w.data(), n);
  return w;
}

// Launch tar.exe directly (not via cmd.exe / _popen) so GUI apps do not flash
// console windows and paths with spaces (e.g. C:\X-Plane 12\...) stay intact.
std::vector<std::uint8_t> runHiddenProcessCaptureStdout(
    const std::wstring& exe, const std::wstring& args) {
  SECURITY_ATTRIBUTES sa{};
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = TRUE;

  HANDLE readPipe = nullptr;
  HANDLE writePipe = nullptr;
  if (!CreatePipe(&readPipe, &writePipe, &sa, 0)) return {};
  SetHandleInformation(readPipe, HANDLE_FLAG_INHERIT, 0);

  HANDLE nul =
      CreateFileW(L"NUL", GENERIC_WRITE, FILE_SHARE_WRITE, &sa, OPEN_EXISTING,
                  FILE_ATTRIBUTE_NORMAL, nullptr);

  STARTUPINFOW si{};
  si.cb = sizeof(si);
  si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
  si.wShowWindow = SW_HIDE;
  si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  si.hStdOutput = writePipe;
  si.hStdError = nul != INVALID_HANDLE_VALUE ? nul : writePipe;

  std::wstring cmd = L"\"" + exe + L"\" " + args;
  std::vector<wchar_t> cmdBuf(cmd.begin(), cmd.end());
  cmdBuf.push_back(L'\0');

  PROCESS_INFORMATION pi{};
  const BOOL ok = CreateProcessW(exe.c_str(), cmdBuf.data(), nullptr, nullptr,
                                 TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si,
                                 &pi);
  CloseHandle(writePipe);
  if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
  if (!ok) {
    CloseHandle(readPipe);
    return {};
  }

  std::vector<std::uint8_t> out;
  std::array<std::uint8_t, 65536> buf{};
  for (;;) {
    DWORD n = 0;
    if (!ReadFile(readPipe, buf.data(), static_cast<DWORD>(buf.size()), &n,
                  nullptr)) {
      break;
    }
    if (n == 0) break;
    const auto prev = out.size();
    out.resize(prev + n);
    std::memcpy(out.data() + prev, buf.data(), n);
  }

  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD exitCode = 1;
  GetExitCodeProcess(pi.hProcess, &exitCode);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);
  CloseHandle(readPipe);

  if (exitCode != 0) return {};
  return out;
}
#endif

std::vector<std::uint8_t> decompress7zToStdout(const std::string& path) {
#if defined(_WIN32)
  static std::mutex decompressMu;
  std::lock_guard<std::mutex> lock(decompressMu);

  const char* systemRoot = std::getenv("SystemRoot");
  if (systemRoot == nullptr) return {};
  const std::wstring tar =
      utf8ToWide(std::string(systemRoot) + "\\System32\\tar.exe");
  const std::wstring args = L"-xOf \"" + utf8ToWide(path) + L"\"";
  const std::vector<std::uint8_t> out =
      runHiddenProcessCaptureStdout(tar, args);
  if (out.size() >= 12 && std::memcmp(out.data(), kDsfMagic, 8) == 0) {
    return out;
  }
  return {};
#else
  const char* tools[] = {"bsdtar", "tar"};
  for (const char* tool : tools) {
    std::string cmd =
        std::string("/usr/bin/") + tool + " -xOf '" + path + "' 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      cmd = std::string(tool) + " -xOf '" + path + "' 2>/dev/null";
      pipe = popen(cmd.c_str(), "r");
    }
    if (!pipe) continue;

    std::vector<std::uint8_t> out;
    std::array<char, 65536> buf{};
    std::size_t n = 0;
    while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
      const auto prev = out.size();
      out.resize(prev + n);
      std::memcpy(out.data() + prev, buf.data(), n);
    }
    pclose(pipe);
    if (out.size() >= 12 && std::memcmp(out.data(), kDsfMagic, 8) == 0) {
      return out;
    }
  }
  return {};
#endif
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
    // DSF raster rows are stored south-to-north (row 0 is the tile's southern
    // edge), so map increasing latitude to increasing row. Using (north - lat)
    // here flips the DEM N-S about the tile center and lands points on the
    // wrong elevation (e.g. KFMY reads 0 m and renders as water).
    const double rowF =
        (lat - south) / (north - south) * static_cast<double>(h - 1);
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

double maxElevationMeters(const DemLayer& layer) {
  const std::uint32_t w = layer.width;
  const std::uint32_t h = layer.height;
  if (w == 0 || h == 0) return 0.0;
  double maxV = -1.0e30;
  for (std::uint32_t row = 0; row < h; ++row) {
    for (std::uint32_t col = 0; col < w; ++col) {
      maxV = std::max(maxV, samplePixel(layer, row, col));
    }
  }
  return maxV;
}

void buildCoarseGrid(const DemTile& dem, float* out, int gridSize) {
  const std::uint32_t w = dem.elevation.width;
  const std::uint32_t h = dem.elevation.height;
  if (w == 0 || h == 0 || gridSize <= 0) return;

  for (int gy = 0; gy < gridSize; ++gy) {
    const std::uint32_t row0 =
        static_cast<std::uint32_t>((gy * static_cast<int>(h)) / gridSize);
    const std::uint32_t row1 =
        static_cast<std::uint32_t>(((gy + 1) * static_cast<int>(h)) / gridSize);
    const std::uint32_t rEnd = std::max(row0 + 1, row1);
    for (int gx = 0; gx < gridSize; ++gx) {
      const std::uint32_t col0 =
          static_cast<std::uint32_t>((gx * static_cast<int>(w)) / gridSize);
      const std::uint32_t col1 =
          static_cast<std::uint32_t>(((gx + 1) * static_cast<int>(w)) / gridSize);
      const std::uint32_t cEnd = std::max(col0 + 1, col1);
      double maxM = -1.0e30;
      for (std::uint32_t row = row0; row < rEnd && row < h; ++row) {
        for (std::uint32_t col = col0; col < cEnd && col < w; ++col) {
          maxM = std::max(maxM, samplePixel(dem.elevation, row, col));
        }
      }
      out[static_cast<std::size_t>(gy) * static_cast<std::size_t>(gridSize) +
          static_cast<std::size_t>(gx)] =
          static_cast<float>(maxM * kMetersToFeet);
    }
  }
}

void tileBoundsDeg(int southLat, int lonIndex, double lon, double& south,
                   double& north, double& west, double& east) {
  south = static_cast<double>(southLat);
  north = south + 1.0;
  west = (lon < 0.0) ? -static_cast<double>(lonIndex)
                       : static_cast<double>(lonIndex);
  east = west + 1.0;
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

double representativeLonForTile(int lonIndex, double lonHint) {
  const int idxFromHint =
      (lonHint < 0.0) ? static_cast<int>(std::ceil(-lonHint - 1e-9))
                      : static_cast<int>(std::floor(lonHint));
  if (idxFromHint == lonIndex) return lonHint;
  return (lonHint < 0.0) ? (-static_cast<double>(lonIndex) + 0.5)
                         : (static_cast<double>(lonIndex) + 0.5);
}

}  // namespace

struct DsfTerrainStore::TileCacheEntry {
  int southLat = 0;
  int lonIndex = 0;
  std::chrono::steady_clock::time_point lastHit{};
  DemTile dem;
};

DsfTerrainStore::DsfTerrainStore()
    : DsfTerrainStore(xplane_install::earthNavDataDir()) {}

DsfTerrainStore::DsfTerrainStore(std::string earthNavDir)
    : earthNavDir_(std::move(earthNavDir)) {
  cache_.reserve(kMaxCachedTiles);
  if (!earthNavDir_.empty()) {
    for (std::thread& worker : workers_) {
      worker = std::thread([this] { workerMain(); });
    }
  }
}

DsfTerrainStore::~DsfTerrainStore() {
  {
    std::lock_guard<std::mutex> lock(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  for (std::thread& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  for (TileCacheEntry* e : cache_) delete e;
}

std::uint64_t DsfTerrainStore::tileKey(int southLat, int lonIndex) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(southLat + 128))
          << 32) |
         static_cast<std::uint32_t>(lonIndex);
}

std::string DsfTerrainStore::tilePath(int southLat, int lonIndex, double lat,
                                      double lon) const {
  if (earthNavDir_.empty()) return {};
  const double repLat = static_cast<double>(southLat) + 0.5;
  const double repLon = representativeLonForTile(lonIndex, lon);
  const std::string folder =
      joinPath(earthNavDir_, formatTileFolder(latBucket(repLat), lonBucket(repLon)));
  return joinPath(folder, formatTileFile(southLat, lonIndex));
}

void DsfTerrainStore::workerMain() {
  for (;;) {
    PendingLoad job{};
    {
      std::unique_lock<std::mutex> lock(mu_);
      cv_.wait(lock, [this] { return stop_ || !pending_.empty(); });
      if (stop_) return;
      popPendingLoadLocked(job);
    }

    std::unique_ptr<::avionics::DemTile> loaded;
    const std::string path =
        tilePath(job.southLat, job.lonIndex, job.lat, job.lon);
    if (!path.empty()) {
      const std::vector<std::uint8_t> bytes = loadDsfBytes(path);
      if (!bytes.empty()) loaded = loadTileFromDsf(bytes);
    }

    if (!loaded) {
      std::lock_guard<std::mutex> lock(mu_);
      misses_.insert(tileKey(job.southLat, job.lonIndex));
      revision_.fetch_add(1, std::memory_order_relaxed);
      cv_.notify_all();
      continue;
    }

    const std::uint64_t key = tileKey(job.southLat, job.lonIndex);
    std::uint8_t existingGrid = 0;
    bool hasFull = false;
    bool wantFull = false;
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (const auto it = summaries_.find(key); it != summaries_.end()) {
        existingGrid = it->second.gridSize;
      }
      hasFull =
          findResidentTileLocked(job.southLat, job.lonIndex,
                                 std::chrono::steady_clock::now()) != nullptr;
      wantFull =
          !coarseSample_ || tileInDetailZoneLocked(job.southLat, job.lonIndex);
    }

    auto publishSummary = [&](std::uint8_t gridSize) {
      CoarseSummary summary;
      summary.gridSize = gridSize;
      buildCoarseGrid(*loaded, summary.elevFt, gridSize);
      std::lock_guard<std::mutex> lock(mu_);
      summaries_[key] = summary;
      revision_.fetch_add(1, std::memory_order_relaxed);
      cv_.notify_all();
    };

    if (existingGrid < kQuickGridSize) {
      publishSummary(kQuickGridSize);
    }
    if (existingGrid < kCoarseGridSize) {
      publishSummary(kCoarseGridSize);
    }
    if (wantFull && !hasFull) {
      std::lock_guard<std::mutex> lock(mu_);
      auto* slot = new TileCacheEntry{};
      slot->southLat = job.southLat;
      slot->lonIndex = job.lonIndex;
      slot->lastHit = std::chrono::steady_clock::now();
      slot->dem = std::move(*loaded);
      if (cache_.size() >= maxCacheTilesLocked()) {
        delete cache_.back();
        cache_.pop_back();
      }
      cache_.insert(cache_.begin(), slot);
      revision_.fetch_add(1, std::memory_order_relaxed);
      cv_.notify_all();
    }
  }
}

const DsfTerrainStore::CoarseSummary* DsfTerrainStore::findSummaryLocked(
    int southLat, int lonIndex) const {
  const auto it = summaries_.find(tileKey(southLat, lonIndex));
  if (it == summaries_.end()) return nullptr;
  return &it->second;
}

float DsfTerrainStore::sampleCoarseGridLocked(const CoarseSummary& summary,
                                              int southLat, int lonIndex,
                                              double lat, double lon) const {
  double south = 0.0;
  double north = 0.0;
  double west = 0.0;
  double east = 0.0;
  tileBoundsDeg(southLat, lonIndex, lon, south, north, west, east);
  const int n = std::max(2, static_cast<int>(summary.gridSize));
  const double colF =
      (lon - west) / (east - west) * static_cast<double>(n - 1);
  const double rowF =
      (lat - south) / (north - south) * static_cast<double>(n - 1);
  const int c0 = static_cast<int>(std::floor(colF));
  const int r0 = static_cast<int>(std::floor(rowF));
  const int c1 = std::min(c0 + 1, n - 1);
  const int r1 = std::min(r0 + 1, n - 1);
  const double fc = colF - static_cast<double>(c0);
  const double fr = rowF - static_cast<double>(r0);
  const auto at = [&](int r, int c) -> float {
    return summary.elevFt[static_cast<std::size_t>(r) * static_cast<std::size_t>(n) +
                          static_cast<std::size_t>(c)];
  };
  const float v00 = at(r0, c0);
  const float v10 = at(r0, c1);
  const float v01 = at(r1, c0);
  const float v11 = at(r1, c1);
  const float v0 = v00 * static_cast<float>(1.0 - fc) + v10 * static_cast<float>(fc);
  const float v1 = v01 * static_cast<float>(1.0 - fc) + v11 * static_cast<float>(fc);
  return v0 * static_cast<float>(1.0 - fr) + v1 * static_cast<float>(fr);
}

float DsfTerrainStore::sampleElevationLocked(
    int southLat, int lonIndex, double lat, double lon,
    std::chrono::steady_clock::time_point now) const {
  if (TileCacheEntry* tile = findResidentTileLocked(southLat, lonIndex, now)) {
    return static_cast<float>(tile->dem.elevationMeters(lat, lon) *
                              kMetersToFeet);
  }
  if (const CoarseSummary* summary = findSummaryLocked(southLat, lonIndex)) {
    return sampleCoarseGridLocked(*summary, southLat, lonIndex, lat, lon);
  }
  return tileAbsentElevationFt();
}

DsfTerrainStore::TileCacheEntry* DsfTerrainStore::findResidentTileLocked(
    int southLat, int lonIndex,
    std::chrono::steady_clock::time_point now) const {
  // LRU hit: move the entry to the front so wide rasters that touch many tiles
  // keep their working set resident.
  for (std::size_t i = 0; i < cache_.size(); ++i) {
    TileCacheEntry* e = cache_[i];
    if (e->southLat == southLat && e->lonIndex == lonIndex) {
      e->lastHit = now;
      if (i != 0) {
        cache_.erase(cache_.begin() + static_cast<std::ptrdiff_t>(i));
        cache_.insert(cache_.begin(), e);
      }
      return e;
    }
  }
  return nullptr;
}

void DsfTerrainStore::queueTileLocked(
    int southLat, int lonIndex, double lat, double lon,
    std::chrono::steady_clock::time_point now) const {
  queueTileLocked(southLat, lonIndex, lat, lon, now, false);
}

bool DsfTerrainStore::tileMissedLocked(int southLat, int lonIndex) const {
  return misses_.count(tileKey(southLat, lonIndex)) != 0;
}

bool DsfTerrainStore::tileInDetailZoneLocked(int southLat,
                                             int lonIndex) const {
  if (detailHalfNm_ <= 0.0f) return !coarseSample_;
  const double tileLat = static_cast<double>(southLat) + 0.5;
  const double tileLon =
      representativeLonForTile(lonIndex, viewCenterLon_);
  const double dN = (tileLat - viewCenterLat_) * 60.0;
  const double nmLon =
      60.0 * std::cos(viewCenterLat_ * 3.14159265358979323846 / 180.0);
  const double dE = (tileLon - viewCenterLon_) * nmLon;
  return (dN * dN + dE * dE) <=
         static_cast<double>(detailHalfNm_) * static_cast<double>(detailHalfNm_);
}

bool DsfTerrainStore::popPendingLoadLocked(PendingLoad& out) {
  if (pending_.empty()) return false;
  std::size_t best = 0;
  double bestScore = 1.0e30;
  for (std::size_t i = 0; i < pending_.size(); ++i) {
    const PendingLoad& p = pending_[i];
    const double tileLat = static_cast<double>(p.southLat) + 0.5;
    const double tileLon =
        representativeLonForTile(p.lonIndex, viewCenterLon_);
    const double dN = (tileLat - viewCenterLat_) * 60.0;
    const double nmLon =
        60.0 * std::cos(viewCenterLat_ * 3.14159265358979323846 / 180.0);
    const double dE = (tileLon - viewCenterLon_) * nmLon;
    const double score = dN * dN + dE * dE;
    if (score < bestScore) {
      bestScore = score;
      best = i;
    }
  }
  out = pending_[best];
  pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(best));
  return true;
}

bool DsfTerrainStore::tileReadyLocked(
    int southLat, int lonIndex,
    std::chrono::steady_clock::time_point now) const {
  if (findResidentTileLocked(southLat, lonIndex, now) != nullptr ||
      tileMissedLocked(southLat, lonIndex)) {
    return true;
  }
  if (findSummaryLocked(southLat, lonIndex) == nullptr) return false;
  if (!coarseSample_ || !tileInDetailZoneLocked(southLat, lonIndex)) {
    return true;
  }
  return false;
}

void DsfTerrainStore::queueTileLocked(
    int southLat, int lonIndex, double lat, double lon,
    std::chrono::steady_clock::time_point now, bool force) const {
  const std::uint64_t key = tileKey(southLat, lonIndex);
  if (force) {
    misses_.erase(key);
  } else if (tileMissedLocked(southLat, lonIndex)) {
    return;
  }

  // A tile with a coarse summary but no resident full DEM is an "upgrade"
  // request (replace the blocky summary with the full tile), as opposed to a
  // first-time fill of an untouched tile.
  const bool haveSummary = findSummaryLocked(southLat, lonIndex) != nullptr;
  if (!force && (!bulkSample_ || haveSummary)) {
    // With the cache full of tiles that were all hit within the last frame or
    // two, the visible footprint is bigger than the cache. Evicting would just
    // thrash (reload the same tiles every frame), so leave the working set
    // resident and let out-of-cache areas read as sea level until their tile
    // arrives (see tileAbsentElevationFt). The bulk first-fill is exempt so a
    // wide view still streams every tile in, but full-detail *upgrades* honor
    // this guard even during bulk sampling: a footprint larger than the cache
    // would otherwise reload-and-evict the same tiles every frame.
    const bool cacheSaturated =
        cache_.size() >= maxCacheTilesLocked() &&
        now - cache_.back()->lastHit < std::chrono::milliseconds(250);
    if (cacheSaturated || pending_.size() >= maxPendingLoadsLocked()) return;
  }

  for (const PendingLoad& p : pending_) {
    if (p.southLat == southLat && p.lonIndex == lonIndex) return;
  }
  if (!force) {
    if (findResidentTileLocked(southLat, lonIndex, now) != nullptr) return;
    if (haveSummary) {
      // Upgrade summary -> full whenever this tile should be full-detail: at
      // close/terminal range (coarseSample_ off) every visible tile qualifies,
      // and on the continental zoom only tiles inside the detail zone do.
      const bool needsFull =
          !coarseSample_ || tileInDetailZoneLocked(southLat, lonIndex);
      if (!needsFull) return;
    }
  }
  pending_.push_back({southLat, lonIndex, lat, lon});
  cv_.notify_all();
}

void DsfTerrainStore::ensureCoverage(double minLat, double maxLat, double minLon,
                                     double maxLon,
                                     bool waitForTiles) const {
  if (earthNavDir_.empty()) return;

  const double loLat = std::min(minLat, maxLat);
  const double hiLat = std::max(minLat, maxLat);
  const double loLon = std::min(minLon, maxLon);
  const double hiLon = std::max(minLon, maxLon);
  const int latSouthMin = floorLatSouth(loLat);
  const int latSouthMax = floorLatSouth(hiLat);
  // tileLonIndex increases westward in the western hemisphere, so min/max lon
  // do not map to ascending index order without an explicit swap.
  const int lonIdxMin =
      std::min(tileLonIndex(loLon), tileLonIndex(hiLon));
  const int lonIdxMax =
      std::max(tileLonIndex(loLon), tileLonIndex(hiLon));
  const double probeLat = (loLat + hiLat) * 0.5;
  const double probeLon = (loLon + hiLon) * 0.5;

  std::unique_lock<std::mutex> lock(mu_);
  const int tileCount =
      (latSouthMax - latSouthMin + 1) * (lonIdxMax - lonIdxMin + 1);
  const auto deadline =
      std::chrono::steady_clock::now() +
      (waitForTiles && tileCount > kMaxBlockingEnsureTiles
           ? std::chrono::seconds(180)
           : std::chrono::seconds(30));

  auto queueOutstanding = [&](std::chrono::steady_clock::time_point now) {
    for (int south = latSouthMin; south <= latSouthMax; ++south) {
      for (int lonIdx = lonIdxMin; lonIdx <= lonIdxMax; ++lonIdx) {
        if (tileReadyLocked(south, lonIdx, now)) continue;
        const double repLat = static_cast<double>(south) + 0.5;
        const double repLon = representativeLonForTile(lonIdx, probeLon);
        queueTileLocked(south, lonIdx, repLat, repLon, now, true);
      }
    }
  };

  if (tileCount > kMaxBlockingEnsureTiles && !waitForTiles) {
    queueOutstanding(std::chrono::steady_clock::now());
    return;
  }

  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    queueOutstanding(now);

    bool ready = true;
    for (int south = latSouthMin; south <= latSouthMax && ready; ++south) {
      for (int lonIdx = lonIdxMin; lonIdx <= lonIdxMax; ++lonIdx) {
        if (!tileReadyLocked(south, lonIdx, now)) {
          ready = false;
          break;
        }
      }
    }
    if (ready && pending_.empty()) return;
    if (now >= deadline) return;
    cv_.wait_for(lock, std::chrono::milliseconds(50));
  }
}

void DsfTerrainStore::setBulkTerrainSample(bool enabled) const {
  std::lock_guard<std::mutex> lock(mu_);
  bulkSample_ = enabled;
}

void DsfTerrainStore::setCoarseTerrainSample(bool enabled) const {
  std::lock_guard<std::mutex> lock(mu_);
  coarseSample_ = enabled;
}

void DsfTerrainStore::setTerrainViewCenter(double lat, double lon,
                                           float detailHalfNm) const {
  std::lock_guard<std::mutex> lock(mu_);
  viewCenterLat_ = lat;
  viewCenterLon_ = lon;
  detailHalfNm_ = detailHalfNm;
}

float DsfTerrainStore::elevationFt(double lat, double lon) const {
  if (earthNavDir_.empty()) return fallback_.elevationFt(lat, lon);

  const int south = floorLatSouth(lat);
  const int lonIdx = tileLonIndex(lon);

  std::lock_guard<std::mutex> lock(mu_);
  const auto now = std::chrono::steady_clock::now();
  const float elev = sampleElevationLocked(south, lonIdx, lat, lon, now);
  if (!std::isnan(elev)) {
    // Background-upgrade a summary-served sample to full DEM when this tile
    // should be full-detail (see elevationFtRow); throttled so it is harmless
    // off the render path.
    const bool wantFull =
        !coarseSample_ || tileInDetailZoneLocked(south, lonIdx);
    if (wantFull && findResidentTileLocked(south, lonIdx, now) == nullptr) {
      queueTileLocked(south, lonIdx, lat, lon, now, false);
    }
    return elev;
  }
  if (tileMissedLocked(south, lonIdx)) return tileAbsentElevationFt();
  queueTileLocked(south, lonIdx, lat, lon, now, bulkSample_ || coarseSample_);
  return tileAbsentElevationFt();
}

void DsfTerrainStore::elevationFtRow(double lat, double lonStart,
                                     double lonStep, int count,
                                     float* out) const {
  if (count <= 0) return;
  if (earthNavDir_.empty()) {
    for (int i = 0; i < count; ++i) {
      out[i] = fallback_.elevationFt(
          lat, lonStart + lonStep * static_cast<double>(i));
    }
    return;
  }

  // Take the lock once for the whole row. Latitude is constant, so the tile
  // changes only when longitude crosses a 1-degree boundary; we re-resolve the
  // serving tile (or fallback) only at those crossings instead of per sample,
  // which removes the per-pixel lock + cache scan that dominated rebuilds.
  std::lock_guard<std::mutex> lock(mu_);
  const auto now = std::chrono::steady_clock::now();
  const int south = floorLatSouth(lat);

  int resolvedLonIdx = INT_MIN;
  for (int i = 0; i < count; ++i) {
    const double lon = lonStart + lonStep * static_cast<double>(i);
    const int lonIdx = tileLonIndex(lon);
    const float elev = sampleElevationLocked(south, lonIdx, lat, lon, now);
    if (!std::isnan(elev)) {
      out[i] = elev;
      // The sample came from a resident full DEM tile or a coarse summary. When
      // it is only the summary but this tile should be full-detail, queue a
      // background upgrade so the blocky summary (16x16 cells ~ 3.75 NM) is
      // replaced by the full DEM. Without this a tile whose full DEM was evicted
      // (or only ever summarized while in coarse mode) stays pixelated forever:
      // the summary read is non-NaN, so the first-load path below never runs.
      // coarseSample upgrades are forced (prioritized near the view center);
      // full-detail upgrades take the throttled path so a footprint larger than
      // the cache does not thrash.
      const bool wantFull =
          !coarseSample_ || tileInDetailZoneLocked(south, lonIdx);
      if (wantFull && lonIdx != resolvedLonIdx &&
          findResidentTileLocked(south, lonIdx, now) == nullptr) {
        resolvedLonIdx = lonIdx;
        queueTileLocked(south, lonIdx, lat, lon, now, coarseSample_);
      }
      continue;
    }
    if (tileMissedLocked(south, lonIdx)) {
      out[i] = tileAbsentElevationFt();
      continue;
    }
    if (lonIdx != resolvedLonIdx) {
      resolvedLonIdx = lonIdx;
      queueTileLocked(south, lonIdx, lat, lon, now,
                      bulkSample_ || coarseSample_);
    }
    out[i] = tileAbsentElevationFt();
  }
}

}  // namespace avionics
