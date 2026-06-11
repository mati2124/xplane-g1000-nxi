#include "avionics/AptDatParser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace avionics {
namespace {

int cellKey(double lat, double lon) {
  const int la = static_cast<int>(std::floor(lat)) + 90;
  const int lo = static_cast<int>(std::floor(lon)) + 180;
  return la * 360 + lo;
}

std::string trimCopy(const std::string& s) {
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::string upperTrim(const std::string& s) {
  std::string out = trimCopy(s);
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
}

bool looksLikeIcao(const std::string& id) {
  if (id.size() < 3 || id.size() > 7) return false;
  for (char c : id) {
    if (!std::isalnum(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

// Normalize a runway-end designator for the SafeTaxi label: zero-pad a single
// leading digit so "5" reads "05" like the painted runway number, preserving
// any L/R/C suffix ("9L" -> "09L").
std::string normalizeRunwayId(const std::string& raw) {
  if (raw.empty()) return raw;
  std::size_t digits = 0;
  while (digits < raw.size() && std::isdigit(static_cast<unsigned char>(raw[digits]))) {
    ++digits;
  }
  if (digits == 1) return "0" + raw;
  return raw;
}

// apt.dat surface code -> G1000 surface vocabulary. X-Plane 12 added the
// 20-38 block for asphalt/concrete appearance variants, all hard surface.
RunwaySurface surfaceFromCode(long code) {
  if (code == 1 || code == 2 || (code >= 20 && code <= 38)) {
    return RunwaySurface::Hard;
  }
  switch (code) {
    case 3:
      return RunwaySurface::Turf;
    case 4:
      return RunwaySurface::Dirt;
    case 5:
      return RunwaySurface::Gravel;
    case 12:  // dry lakebed
      return RunwaySurface::Dirt;
    case 13:
      return RunwaySurface::Water;
    case 14:  // snow/ice
      return RunwaySurface::Soft;
    default:
      return RunwaySurface::Unknown;
  }
}

bool parseRunwayRow(const char* p, MapRunway& out, AirportRunwayInfo& info) {
  char* end = nullptr;
  const double width = std::strtod(p, &end);
  if (end == p || width <= 0.0) return false;
  p = end;

  // The runway-end designator is the last of the seven tokens skipped before
  // each threshold's lat/lon. The first group is the runway-wide properties
  // (surface, shoulder, smoothness, centerline lights, edge lights, signs),
  // then the end number; the second group is end-1 attributes then the end-2
  // number.
  std::string lastToken;
  auto skipToken = [&]() {
    while (*p == ' ' || *p == '\t') ++p;
    const char* start = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    lastToken.assign(start, static_cast<std::size_t>(p - start));
  };
  std::string surfaceTok, edgeLightsTok;
  for (int i = 0; i < 7; ++i) {
    skipToken();
    if (i == 0) surfaceTok = lastToken;
    if (i == 4) edgeLightsTok = lastToken;
  }
  const std::string id1 = lastToken;

  const double lat1 = std::strtod(p, &end);
  if (end == p) return false;
  p = end;
  const double lon1 = std::strtod(p, &end);
  if (end == p) return false;
  p = end;

  for (int i = 0; i < 7; ++i) skipToken();
  const std::string id2 = lastToken;

  const double lat2 = std::strtod(p, &end);
  if (end == p) return false;
  p = end;
  const double lon2 = std::strtod(p, &end);
  if (end == p) return false;

  if (std::fabs(lat1) > 90.0 || std::fabs(lat2) > 90.0 ||
      std::fabs(lon1) > 180.0 || std::fabs(lon2) > 180.0) {
    return false;
  }

  out.a = {lat1, lon1};
  out.b = {lat2, lon2};
  out.widthM = static_cast<float>(width);
  out.idA = normalizeRunwayId(id1);
  out.idB = normalizeRunwayId(id2);

  constexpr double kDegToRadLocal = 3.14159265358979323846 / 180.0;
  constexpr double kFtPerNm = 6076.115;
  const double midLat = (lat1 + lat2) * 0.5;
  const double cosLat = std::max(0.05, std::cos(midLat * kDegToRadLocal));
  const double dLatNm = (lat2 - lat1) * 60.0;
  const double dLonNm = (lon2 - lon1) * 60.0 * cosLat;
  const double lengthNm = std::sqrt(dLatNm * dLatNm + dLonNm * dLonNm);

  info.designation = out.idA + "-" + out.idB;
  info.lengthFt = static_cast<int>(lengthNm * kFtPerNm + 0.5);
  info.widthFt = static_cast<int>(width * 3.28084 + 0.5);
  info.surface = surfaceFromCode(std::strtol(surfaceTok.c_str(), nullptr, 10));
  info.lighted = std::strtol(edgeLightsTok.c_str(), nullptr, 10) > 0;
  return true;
}

// Paved surfaces shown on the taxiway diagram: asphalt (1) and concrete (2).
// Grass/dirt/gravel ramps are skipped so the close-range diagram stays a clean
// SafeTaxi-style hard-surface outline.
bool isHardSurface(long surface) { return surface == 1 || surface == 2; }

// Surface code from a row-110 pavement header ("110 <surface> <smoothness>
// <orientation> ...").
bool parsePavementSurface(const std::string& line, long& surface) {
  const char* p = line.c_str();
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);  // row code 110
  if (end == p) return false;
  p = end;
  surface = std::strtol(p, &end, 10);
  return end != p;
}

// Lat/lon from a pavement node row (111-116). Bezier control points and any
// trailing attributes are ignored: curves become straight segments.
bool parseNodeLatLon(const std::string& line, double& lat, double& lon) {
  const char* p = line.c_str();
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);  // node row code 111-116
  if (end == p) return false;
  p = end;
  lat = std::strtod(p, &end);
  if (end == p) return false;
  p = end;
  lon = std::strtod(p, &end);
  if (end == p) return false;
  return std::fabs(lat) <= 90.0 && std::fabs(lon) <= 180.0;
}

void commitPavement(std::unordered_map<int, std::vector<MapPavement>>& out,
                    MapPavement& pavement) {
  if (pavement.outline.size() >= 3) {
    double sumLat = 0.0;
    double sumLon = 0.0;
    for (const GeoPoint& g : pavement.outline) {
      sumLat += g.lat;
      sumLon += g.lon;
    }
    const double cLat = sumLat / static_cast<double>(pavement.outline.size());
    const double cLon = sumLon / static_cast<double>(pavement.outline.size());
    out[cellKey(cLat, cLon)].push_back(std::move(pavement));
  }
  pavement = MapPavement{};
}

struct PendingAirport {
  std::string icao;
  AirportMeta meta;
};

void commitAirport(std::unordered_map<std::string, AirportMeta>& out,
                   PendingAirport& pending) {
  if (pending.icao.empty()) return;
  out[pending.icao] = pending.meta;
  pending = PendingAirport{};
}

void parseAirportHeader(int rowCode, const char* line, PendingAirport& pending) {
  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;

  std::strtod(p, &end);
  if (end == p) return;
  p = end;

  const long field2 = std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;
  const long field3 = std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;

  while (*p == ' ' || *p == '\t') ++p;
  const char* idStart = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  const std::string id(idStart, static_cast<std::size_t>(p - idStart));

  // Remainder of the header row is the facility name ("Hill City Muni"),
  // kept mixed-case the way the G1000 shows it.
  while (*p == ' ' || *p == '\t') ++p;
  const char* nameStart = p;
  while (*p && *p != '\r' && *p != '\n') ++p;
  pending.meta.name =
      trimCopy(std::string(nameStart, static_cast<std::size_t>(p - nameStart)));

  if (rowCode == 16) {
    pending.meta.kind = AirportFacilityKind::Seaplane;
  } else if (rowCode == 17) {
    pending.meta.kind = AirportFacilityKind::Heliport;
  } else {
    pending.meta.kind = AirportFacilityKind::Land;
  }

  if (!(field2 == 0 && field3 == 0) && (field2 == 0 || field2 == 1)) {
    pending.meta.hasControlTower = (field2 == 1);
  }

  if (looksLikeIcao(id)) {
    pending.icao = upperTrim(id);
  }
}

void parseMetadataRow(const char* line, PendingAirport& pending) {
  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  const long code = std::strtol(p, &end, 10);
  if (code != 1302 || end == p) return;
  p = end;

  while (*p == ' ' || *p == '\t') ++p;
  const char* keyStart = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  const std::string key(keyStart, static_cast<std::size_t>(p - keyStart));
  while (*p == ' ' || *p == '\t') ++p;
  const char* valStart = p;
  while (*p && *p != '\r' && *p != '\n') ++p;
  const std::string value(valStart, static_cast<std::size_t>(p - valStart));

  const std::string keyUpper = upperTrim(key);
  if (keyUpper == "ICAO_CODE" || keyUpper == "ICAO_ID") {
    const std::string icao = upperTrim(value);
    if (!icao.empty()) pending.icao = icao;
  } else if (keyUpper == "CITY") {
    // Keep any state/country suffix already in place; the page appends the
    // state only when it is a short abbreviation ("Hill City KS").
    const std::string city = trimCopy(value);
    if (!city.empty()) {
      pending.meta.city =
          pending.meta.city.empty() ? city : city + " " + pending.meta.city;
    }
  } else if (keyUpper == "STATE") {
    // Short postal abbreviations only ("KS"); full state names would crowd
    // the info box the G1000 keeps to "City ST".
    const std::string state = trimCopy(value);
    if (state.size() <= 3) {
      pending.meta.city = pending.meta.city.empty()
                              ? state
                              : pending.meta.city + " " + state;
    }
  }
}

AirportCommService commServiceFromRow(long code) {
  int type = 0;
  if (code >= 1050 && code <= 1056) {
    type = static_cast<int>(code - 1000);
  } else if (code >= 50 && code <= 56) {
    type = static_cast<int>(code);
  }
  switch (type) {
    case 50:
      return AirportCommService::Atis;
    case 51:
      return AirportCommService::Unicom;
    case 52:
      return AirportCommService::Clearance;
    case 53:
      return AirportCommService::Ground;
    case 54:
      return AirportCommService::Tower;
    case 55:
      return AirportCommService::Approach;
    case 56:
      return AirportCommService::Departure;
    default:
      return AirportCommService::Other;
  }
}

void parseCommFrequencyRow(long code, const char* line, PendingAirport& pending) {
  const AirportCommService service = commServiceFromRow(code);
  if (service == AirportCommService::Other) return;

  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;
  const double raw = std::strtod(p, &end);
  if (end == p || raw <= 0.0) return;
  p = end;
  while (*p == ' ' || *p == '\t') ++p;
  const char* labelStart = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  const std::string label(labelStart, static_cast<std::size_t>(p - labelStart));

  MapAirportFrequency freq;
  freq.service = service;
  // Legacy rows (50–56) store MHz x 100; global-airport rows (1050–1056) use
  // MHz x 1000 (e.g. 119800 == 119.800 MHz).
  freq.mhz = static_cast<float>((code >= 1050 ? raw / 1000.0 : raw / 100.0));
  freq.label = label;
  pending.meta.frequencies.push_back(std::move(freq));

  if (service == AirportCommService::Tower) {
    pending.meta.hasControlTower = true;
  }
}

bool rowCodeAt(const std::string& line, long& code) {
  const char* p = line.c_str();
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  code = std::strtol(p, &end, 10);
  return end != p;
}

// Accumulates one airport's ATC taxi-route network: row-1201 nodes (id ->
// position) and the named row-1202 edges, so each taxiway identifier gets a
// single label at the centroid of its segments.
struct TaxiNetwork {
  struct NameAccum {
    double sumLat = 0.0;
    double sumLon = 0.0;
    int count = 0;
  };
  std::unordered_map<int, GeoPoint> nodes;
  std::unordered_map<std::string, NameAccum> byName;
};

// Row 1201: "1201 <lat> <lon> <usage> <node_id> [name]".
void parseTaxiNode(const char* line, TaxiNetwork& net) {
  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);  // 1201
  if (end == p) return;
  p = end;
  const double lat = std::strtod(p, &end);
  if (end == p) return;
  p = end;
  const double lon = std::strtod(p, &end);
  if (end == p) return;
  p = end;
  while (*p == ' ' || *p == '\t') ++p;  // skip usage token (both/dest/...)
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  const long id = std::strtol(p, &end, 10);
  if (end == p) return;
  if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) return;
  net.nodes[static_cast<int>(id)] = {lat, lon};
}

// Row 1202: "1202 <node1> <node2> <oneway|twoway> <group> [<name>]". Only
// taxiway-group edges carrying an ATC identifier (the optional name token) are
// labeled; runway edges and unnamed edges are skipped.
void parseTaxiEdge(const char* line, TaxiNetwork& net) {
  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  std::strtol(p, &end, 10);  // 1202
  if (end == p) return;
  p = end;
  const long n1 = std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;
  const long n2 = std::strtol(p, &end, 10);
  if (end == p) return;
  p = end;

  auto nextToken = [&](std::string& out) {
    while (*p == ' ' || *p == '\t') ++p;
    const char* start = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    out.assign(start, static_cast<std::size_t>(p - start));
    return !out.empty();
  };
  std::string dir, group, name;
  if (!nextToken(dir)) return;
  if (!nextToken(group)) return;
  if (group.rfind("taxiway", 0) != 0) return;
  if (!nextToken(name)) return;

  auto add = [&](long id) {
    const auto it = net.nodes.find(static_cast<int>(id));
    if (it == net.nodes.end()) return;
    TaxiNetwork::NameAccum& acc = net.byName[name];
    acc.sumLat += it->second.lat;
    acc.sumLon += it->second.lon;
    acc.count += 1;
  };
  add(n1);
  add(n2);
}

void flushTaxiNetwork(
    std::unordered_map<int, std::vector<MapTaxiwayLabel>>& out,
    TaxiNetwork& net) {
  for (const auto& [name, acc] : net.byName) {
    if (acc.count == 0) continue;
    const double cLat = acc.sumLat / static_cast<double>(acc.count);
    const double cLon = acc.sumLon / static_cast<double>(acc.count);
    MapTaxiwayLabel label;
    label.pos = {cLat, cLon};
    label.text = name;
    out[cellKey(cLat, cLon)].push_back(std::move(label));
  }
  net = TaxiNetwork{};
}

}  // namespace

AptDatParseResult parseAptDat(std::istream& in) {
  AptDatParseResult result;
  PendingAirport pending;
  TaxiNetwork taxi;

  // Row-110 pavement chunk accumulation state. A chunk runs from its 110
  // header until the next non-node row; only the first (outer) boundary loop is
  // kept and only when the surface is hard.
  bool inPavement = false;
  bool pavementKeep = false;
  bool firstLoopClosed = false;
  MapPavement pavement;
  auto endPavement = [&]() {
    if (inPavement && pavementKeep) {
      commitPavement(result.pavementCells, pavement);
    } else {
      pavement = MapPavement{};
    }
    inPavement = false;
    pavementKeep = false;
    firstLoopClosed = false;
  };

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;

    long code = 0;
    if (!rowCodeAt(line, code)) continue;

    // Pavement node rows (111-116) belong to the current 110 chunk; any other
    // row terminates it.
    if (inPavement) {
      if (code >= 111 && code <= 116) {
        if (pavementKeep && !firstLoopClosed) {
          double lat = 0.0;
          double lon = 0.0;
          if (parseNodeLatLon(line, lat, lon)) {
            pavement.outline.push_back({lat, lon});
          }
        }
        if (code >= 113) firstLoopClosed = true;  // loop/chain terminated
        continue;
      }
      endPavement();
    }

    if (code == 110) {
      long surface = 0;
      inPavement = true;
      firstLoopClosed = false;
      pavementKeep = parsePavementSurface(line, surface) && isHardSurface(surface);
      pavement = MapPavement{};
      continue;
    }

    if (code == 1 || code == 16 || code == 17) {
      flushTaxiNetwork(result.taxiwayLabelCells, taxi);
      commitAirport(result.metaByIcao, pending);
      parseAirportHeader(static_cast<int>(code), line.c_str(), pending);
      continue;
    }

    if (code == 1201) {
      parseTaxiNode(line.c_str(), taxi);
      continue;
    }

    if (code == 1202) {
      parseTaxiEdge(line.c_str(), taxi);
      continue;
    }

    if (code == 1302) {
      parseMetadataRow(line.c_str(), pending);
      continue;
    }

    if (code == 14) {
      pending.meta.hasControlTower = true;
      continue;
    }

    if ((code >= 50 && code <= 56) || (code >= 1050 && code <= 1056)) {
      parseCommFrequencyRow(code, line.c_str(), pending);
      continue;
    }

    if (code == 1400) {
      const std::string upper = upperTrim(line);
      if (upper.find("FUEL_") != std::string::npos) {
        pending.meta.hasFuelServices = true;
      }
      continue;
    }

    if (code != 100) continue;

    MapRunway rwy;
    AirportRunwayInfo info;
    const char* p = line.c_str();
    while (*p && *p != ' ' && *p != '\t') ++p;
    while (*p == ' ' || *p == '\t') ++p;
    if (!parseRunwayRow(p, rwy, info)) continue;
    pending.meta.runways.push_back(std::move(info));
    const double midLat = (rwy.a.lat + rwy.b.lat) * 0.5;
    const double midLon = (rwy.a.lon + rwy.b.lon) * 0.5;
    result.runwayCells[cellKey(midLat, midLon)].push_back(rwy);
  }
  endPavement();
  flushTaxiNetwork(result.taxiwayLabelCells, taxi);
  commitAirport(result.metaByIcao, pending);
  return result;
}

namespace {

constexpr double kNmPerDeg = 60.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

std::vector<MapRunway> nearbyRunwaysFromCells(
    const std::unordered_map<int, std::vector<MapRunway>>& cells, double lat,
    double lon, float rangeNm, std::size_t maxCount) {
  std::vector<MapRunway> result;
  if (maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  const int la0 = static_cast<int>(std::floor(lat - dLat));
  const int la1 = static_cast<int>(std::floor(lat + dLat));
  const int lo0 = static_cast<int>(std::floor(lon - dLon));
  const int lo1 = static_cast<int>(std::floor(lon + dLon));

  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      const auto it = cells.find((la + 90) * 360 + (lo + 180));
      if (it == cells.end()) continue;
      for (const MapRunway& rwy : it->second) {
        const double midLat = (rwy.a.lat + rwy.b.lat) * 0.5;
        const double midLon = (rwy.a.lon + rwy.b.lon) * 0.5;
        if (std::fabs(midLat - lat) > dLat || std::fabs(midLon - lon) > dLon) {
          continue;
        }
        result.push_back(rwy);
        if (result.size() >= maxCount) return result;
      }
    }
  }
  return result;
}

std::vector<MapPavement> nearbyPavementFromCells(
    const std::unordered_map<int, std::vector<MapPavement>>& cells, double lat,
    double lon, float rangeNm, std::size_t maxCount) {
  std::vector<MapPavement> result;
  if (maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  const int la0 = static_cast<int>(std::floor(lat - dLat));
  const int la1 = static_cast<int>(std::floor(lat + dLat));
  const int lo0 = static_cast<int>(std::floor(lon - dLon));
  const int lo1 = static_cast<int>(std::floor(lon + dLon));

  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      const auto it = cells.find((la + 90) * 360 + (lo + 180));
      if (it == cells.end()) continue;
      for (const MapPavement& pav : it->second) {
        if (pav.outline.empty()) continue;
        double sumLat = 0.0;
        double sumLon = 0.0;
        for (const GeoPoint& g : pav.outline) {
          sumLat += g.lat;
          sumLon += g.lon;
        }
        const double n = static_cast<double>(pav.outline.size());
        if (std::fabs(sumLat / n - lat) > dLat ||
            std::fabs(sumLon / n - lon) > dLon) {
          continue;
        }
        result.push_back(pav);
        if (result.size() >= maxCount) return result;
      }
    }
  }
  return result;
}

std::vector<MapTaxiwayLabel> nearbyTaxiwayLabelsFromCells(
    const std::unordered_map<int, std::vector<MapTaxiwayLabel>>& cells,
    double lat, double lon, float rangeNm, std::size_t maxCount) {
  std::vector<MapTaxiwayLabel> result;
  if (maxCount == 0) return result;

  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double dLat = (rangeNm / kNmPerDeg) * 1.2;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.2;

  const int la0 = static_cast<int>(std::floor(lat - dLat));
  const int la1 = static_cast<int>(std::floor(lat + dLat));
  const int lo0 = static_cast<int>(std::floor(lon - dLon));
  const int lo1 = static_cast<int>(std::floor(lon + dLon));

  for (int la = la0; la <= la1; ++la) {
    for (int lo = lo0; lo <= lo1; ++lo) {
      const auto it = cells.find((la + 90) * 360 + (lo + 180));
      if (it == cells.end()) continue;
      for (const MapTaxiwayLabel& label : it->second) {
        if (std::fabs(label.pos.lat - lat) > dLat ||
            std::fabs(label.pos.lon - lon) > dLon) {
          continue;
        }
        result.push_back(label);
        if (result.size() >= maxCount) return result;
      }
    }
  }
  return result;
}

void enrichAirportFromMeta(
    MapFeature& feature,
    const std::unordered_map<std::string, AirportMeta>& metaByIcao) {
  if (feature.type != MapFeatureType::Airport) return;
  const auto it = metaByIcao.find(upperTrim(feature.id));
  if (it == metaByIcao.end()) return;
  const AirportMeta& meta = it->second;
  feature.airportTowered = meta.hasControlTower;
  feature.airportServiced = meta.hasFuelServices;
  if (meta.kind != AirportFacilityKind::Land) {
    feature.airportKind = meta.kind;
  }
  if (feature.name.empty()) feature.name = meta.name;
  if (feature.city.empty()) feature.city = meta.city;
  // Longest runway from the parsed row-100 data when the nav data didn't
  // already provide one.
  if (feature.longestRunwayFt == 0) {
    for (const AirportRunwayInfo& rwy : meta.runways) {
      feature.longestRunwayFt = std::max(feature.longestRunwayFt, rwy.lengthFt);
    }
  }
}

}  // namespace avionics
