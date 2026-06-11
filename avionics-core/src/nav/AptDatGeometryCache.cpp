#include "avionics/AptDatGeometryCache.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sys/stat.h>

namespace avionics {
namespace {

constexpr char kMagic[4] = {'A', 'P', 'T', 'G'};
// v2 adds runway-end designators and the taxiway-label cells.
// v3 adds airport facility name/city and per-airport runway info.
constexpr std::uint32_t kVersion = 3;

bool aptDatIdentity(const std::string& aptDatPath, std::int64_t& sizeOut,
                    std::int64_t& mtimeOut) {
  struct stat st {};
  if (stat(aptDatPath.c_str(), &st) != 0) return false;
  sizeOut = static_cast<std::int64_t>(st.st_size);
  mtimeOut = static_cast<std::int64_t>(st.st_mtime);
  return true;
}

bool writeString(std::ostream& out, const std::string& s) {
  const auto len = static_cast<std::uint32_t>(s.size());
  out.write(reinterpret_cast<const char*>(&len), sizeof(len));
  if (len > 0) out.write(s.data(), static_cast<std::streamsize>(len));
  return out.good();
}

bool readString(std::istream& in, std::string& s) {
  std::uint32_t len = 0;
  in.read(reinterpret_cast<char*>(&len), sizeof(len));
  if (!in.good()) return false;
  s.assign(len, '\0');
  if (len > 0) {
    in.read(s.data(), static_cast<std::streamsize>(len));
  }
  return in.good();
}

bool writeGeoPoint(std::ostream& out, const GeoPoint& g) {
  out.write(reinterpret_cast<const char*>(&g.lat), sizeof(g.lat));
  out.write(reinterpret_cast<const char*>(&g.lon), sizeof(g.lon));
  return out.good();
}

bool readGeoPoint(std::istream& in, GeoPoint& g) {
  in.read(reinterpret_cast<char*>(&g.lat), sizeof(g.lat));
  in.read(reinterpret_cast<char*>(&g.lon), sizeof(g.lon));
  return in.good();
}

bool writeRunwayCells(std::ostream& out,
                      const std::unordered_map<int, std::vector<MapRunway>>& cells) {
  const auto cellCount = static_cast<std::uint32_t>(cells.size());
  out.write(reinterpret_cast<const char*>(&cellCount), sizeof(cellCount));
  for (const auto& [key, runways] : cells) {
    const auto count = static_cast<std::uint32_t>(runways.size());
    out.write(reinterpret_cast<const char*>(&key), sizeof(key));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const MapRunway& rwy : runways) {
      if (!writeGeoPoint(out, rwy.a) || !writeGeoPoint(out, rwy.b)) return false;
      out.write(reinterpret_cast<const char*>(&rwy.widthM), sizeof(rwy.widthM));
      if (!writeString(out, rwy.idA) || !writeString(out, rwy.idB)) return false;
    }
  }
  return out.good();
}

bool readRunwayCells(std::istream& in,
                     std::unordered_map<int, std::vector<MapRunway>>& cells) {
  std::uint32_t cellCount = 0;
  in.read(reinterpret_cast<char*>(&cellCount), sizeof(cellCount));
  if (!in.good()) return false;
  cells.clear();
  for (std::uint32_t c = 0; c < cellCount; ++c) {
    int key = 0;
    std::uint32_t count = 0;
    in.read(reinterpret_cast<char*>(&key), sizeof(key));
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!in.good()) return false;
    std::vector<MapRunway> runways;
    runways.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      MapRunway rwy;
      if (!readGeoPoint(in, rwy.a) || !readGeoPoint(in, rwy.b)) return false;
      in.read(reinterpret_cast<char*>(&rwy.widthM), sizeof(rwy.widthM));
      if (!in.good()) return false;
      if (!readString(in, rwy.idA) || !readString(in, rwy.idB)) return false;
      runways.push_back(rwy);
    }
    cells.emplace(key, std::move(runways));
  }
  return true;
}

bool writePavementCells(
    std::ostream& out,
    const std::unordered_map<int, std::vector<MapPavement>>& cells) {
  const auto cellCount = static_cast<std::uint32_t>(cells.size());
  out.write(reinterpret_cast<const char*>(&cellCount), sizeof(cellCount));
  for (const auto& [key, pavements] : cells) {
    const auto count = static_cast<std::uint32_t>(pavements.size());
    out.write(reinterpret_cast<const char*>(&key), sizeof(key));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const MapPavement& pav : pavements) {
      const auto vertCount = static_cast<std::uint32_t>(pav.outline.size());
      out.write(reinterpret_cast<const char*>(&vertCount), sizeof(vertCount));
      for (const GeoPoint& g : pav.outline) {
        if (!writeGeoPoint(out, g)) return false;
      }
    }
  }
  return out.good();
}

bool readPavementCells(std::istream& in,
                       std::unordered_map<int, std::vector<MapPavement>>& cells) {
  std::uint32_t cellCount = 0;
  in.read(reinterpret_cast<char*>(&cellCount), sizeof(cellCount));
  if (!in.good()) return false;
  cells.clear();
  for (std::uint32_t c = 0; c < cellCount; ++c) {
    int key = 0;
    std::uint32_t count = 0;
    in.read(reinterpret_cast<char*>(&key), sizeof(key));
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!in.good()) return false;
    std::vector<MapPavement> pavements;
    pavements.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      MapPavement pav;
      std::uint32_t vertCount = 0;
      in.read(reinterpret_cast<char*>(&vertCount), sizeof(vertCount));
      if (!in.good()) return false;
      pav.outline.reserve(vertCount);
      for (std::uint32_t v = 0; v < vertCount; ++v) {
        GeoPoint g;
        if (!readGeoPoint(in, g)) return false;
        pav.outline.push_back(g);
      }
      pavements.push_back(std::move(pav));
    }
    cells.emplace(key, std::move(pavements));
  }
  return true;
}

bool writeTaxiwayLabelCells(
    std::ostream& out,
    const std::unordered_map<int, std::vector<MapTaxiwayLabel>>& cells) {
  const auto cellCount = static_cast<std::uint32_t>(cells.size());
  out.write(reinterpret_cast<const char*>(&cellCount), sizeof(cellCount));
  for (const auto& [key, labels] : cells) {
    const auto count = static_cast<std::uint32_t>(labels.size());
    out.write(reinterpret_cast<const char*>(&key), sizeof(key));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const MapTaxiwayLabel& label : labels) {
      if (!writeGeoPoint(out, label.pos)) return false;
      if (!writeString(out, label.text)) return false;
    }
  }
  return out.good();
}

bool readTaxiwayLabelCells(
    std::istream& in,
    std::unordered_map<int, std::vector<MapTaxiwayLabel>>& cells) {
  std::uint32_t cellCount = 0;
  in.read(reinterpret_cast<char*>(&cellCount), sizeof(cellCount));
  if (!in.good()) return false;
  cells.clear();
  for (std::uint32_t c = 0; c < cellCount; ++c) {
    int key = 0;
    std::uint32_t count = 0;
    in.read(reinterpret_cast<char*>(&key), sizeof(key));
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!in.good()) return false;
    std::vector<MapTaxiwayLabel> labels;
    labels.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      MapTaxiwayLabel label;
      if (!readGeoPoint(in, label.pos)) return false;
      if (!readString(in, label.text)) return false;
      labels.push_back(std::move(label));
    }
    cells.emplace(key, std::move(labels));
  }
  return true;
}

bool writeMeta(std::ostream& out,
               const std::unordered_map<std::string, AirportMeta>& meta) {
  const auto count = static_cast<std::uint32_t>(meta.size());
  out.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& [icao, m] : meta) {
    if (!writeString(out, icao)) return false;
    const auto kind = static_cast<std::uint8_t>(m.kind);
  out.write(reinterpret_cast<const char*>(&m.hasControlTower),
            sizeof(m.hasControlTower));
    out.write(reinterpret_cast<const char*>(&m.hasFuelServices),
              sizeof(m.hasFuelServices));
    out.write(reinterpret_cast<const char*>(&kind), sizeof(kind));
    if (!writeString(out, m.name) || !writeString(out, m.city)) return false;
    const auto freqCount = static_cast<std::uint32_t>(m.frequencies.size());
    out.write(reinterpret_cast<const char*>(&freqCount), sizeof(freqCount));
    for (const MapAirportFrequency& f : m.frequencies) {
      const auto service = static_cast<std::uint8_t>(f.service);
      out.write(reinterpret_cast<const char*>(&service), sizeof(service));
      out.write(reinterpret_cast<const char*>(&f.mhz), sizeof(f.mhz));
      if (!writeString(out, f.label)) return false;
    }
    const auto rwyCount = static_cast<std::uint32_t>(m.runways.size());
    out.write(reinterpret_cast<const char*>(&rwyCount), sizeof(rwyCount));
    for (const AirportRunwayInfo& r : m.runways) {
      if (!writeString(out, r.designation)) return false;
      const auto lengthFt = static_cast<std::int32_t>(r.lengthFt);
      const auto widthFt = static_cast<std::int32_t>(r.widthFt);
      const auto surface = static_cast<std::uint8_t>(r.surface);
      out.write(reinterpret_cast<const char*>(&lengthFt), sizeof(lengthFt));
      out.write(reinterpret_cast<const char*>(&widthFt), sizeof(widthFt));
      out.write(reinterpret_cast<const char*>(&surface), sizeof(surface));
      out.write(reinterpret_cast<const char*>(&r.lighted), sizeof(r.lighted));
    }
  }
  return out.good();
}

bool readMeta(std::istream& in, std::unordered_map<std::string, AirportMeta>& meta) {
  std::uint32_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(count));
  if (!in.good()) return false;
  meta.clear();
  for (std::uint32_t i = 0; i < count; ++i) {
    std::string icao;
  AirportMeta m;
    if (!readString(in, icao)) return false;
    std::uint8_t kind = 0;
    in.read(reinterpret_cast<char*>(&m.hasControlTower), sizeof(m.hasControlTower));
    in.read(reinterpret_cast<char*>(&m.hasFuelServices), sizeof(m.hasFuelServices));
    in.read(reinterpret_cast<char*>(&kind), sizeof(kind));
    m.kind = static_cast<AirportFacilityKind>(kind);
    if (!readString(in, m.name) || !readString(in, m.city)) return false;
    std::uint32_t freqCount = 0;
    in.read(reinterpret_cast<char*>(&freqCount), sizeof(freqCount));
    if (!in.good()) return false;
    m.frequencies.reserve(freqCount);
    for (std::uint32_t f = 0; f < freqCount; ++f) {
      MapAirportFrequency freq;
      std::uint8_t service = 0;
      in.read(reinterpret_cast<char*>(&service), sizeof(service));
      in.read(reinterpret_cast<char*>(&freq.mhz), sizeof(freq.mhz));
      if (!readString(in, freq.label)) return false;
      freq.service = static_cast<AirportCommService>(service);
      m.frequencies.push_back(std::move(freq));
    }
    std::uint32_t rwyCount = 0;
    in.read(reinterpret_cast<char*>(&rwyCount), sizeof(rwyCount));
    if (!in.good()) return false;
    m.runways.reserve(rwyCount);
    for (std::uint32_t r = 0; r < rwyCount; ++r) {
      AirportRunwayInfo rwy;
      if (!readString(in, rwy.designation)) return false;
      std::int32_t lengthFt = 0;
      std::int32_t widthFt = 0;
      std::uint8_t surface = 0;
      in.read(reinterpret_cast<char*>(&lengthFt), sizeof(lengthFt));
      in.read(reinterpret_cast<char*>(&widthFt), sizeof(widthFt));
      in.read(reinterpret_cast<char*>(&surface), sizeof(surface));
      in.read(reinterpret_cast<char*>(&rwy.lighted), sizeof(rwy.lighted));
      if (!in.good()) return false;
      rwy.lengthFt = lengthFt;
      rwy.widthFt = widthFt;
      rwy.surface = static_cast<RunwaySurface>(surface);
      m.runways.push_back(std::move(rwy));
    }
    meta.emplace(std::move(icao), std::move(m));
  }
  return true;
}

}  // namespace

bool loadAptDatGeometryCache(const std::string& cachePath,
                             const std::string& aptDatPath,
                             AptDatParseResult& out) {
  std::int64_t aptSize = 0;
  std::int64_t aptMtime = 0;
  if (!aptDatIdentity(aptDatPath, aptSize, aptMtime)) return false;

  std::ifstream in(cachePath, std::ios::binary);
  if (!in.good()) return false;

  char magic[4] = {};
  std::uint32_t version = 0;
  std::int64_t cachedSize = 0;
  std::int64_t cachedMtime = 0;
  in.read(magic, sizeof(magic));
  in.read(reinterpret_cast<char*>(&version), sizeof(version));
  in.read(reinterpret_cast<char*>(&cachedSize), sizeof(cachedSize));
  in.read(reinterpret_cast<char*>(&cachedMtime), sizeof(cachedMtime));
  if (!in.good()) return false;
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0 || version != kVersion ||
      cachedSize != aptSize || cachedMtime != aptMtime) {
    return false;
  }

  return readRunwayCells(in, out.runwayCells) &&
         readPavementCells(in, out.pavementCells) &&
         readTaxiwayLabelCells(in, out.taxiwayLabelCells) &&
         readMeta(in, out.metaByIcao);
}

bool saveAptDatGeometryCache(const std::string& cachePath,
                             const std::string& aptDatPath,
                             const AptDatParseResult& in) {
  std::int64_t aptSize = 0;
  std::int64_t aptMtime = 0;
  if (!aptDatIdentity(aptDatPath, aptSize, aptMtime)) return false;

  std::ofstream out(cachePath, std::ios::binary | std::ios::trunc);
  if (!out.good()) return false;

  const std::uint32_t version = kVersion;
  out.write(kMagic, sizeof(kMagic));
  out.write(reinterpret_cast<const char*>(&version), sizeof(version));
  out.write(reinterpret_cast<const char*>(&aptSize), sizeof(aptSize));
  out.write(reinterpret_cast<const char*>(&aptMtime), sizeof(aptMtime));
  return writeRunwayCells(out, in.runwayCells) &&
         writePavementCells(out, in.pavementCells) &&
         writeTaxiwayLabelCells(out, in.taxiwayLabelCells) &&
         writeMeta(out, in.metaByIcao);
}

}  // namespace avionics
