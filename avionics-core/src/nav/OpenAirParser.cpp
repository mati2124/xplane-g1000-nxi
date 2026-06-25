#include "avionics/OpenAirParser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>

namespace avionics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kNmPerDeg = 60.0;

// "Unlimited" ceiling sentinel and the step (degrees) used to tessellate arcs.
constexpr float kUnlimitedFt = kAirspaceUnlimitedFt;
constexpr double kArcStepDeg = 5.0;

double cosLatClamped(double latDeg) {
  return std::max(0.05, std::cos(latDeg * kDegToRad));
}

std::string upperTrim(const std::string& s) {
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  std::string out = s.substr(a, b - a);
  for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return out;
}

AirspaceClass classFromToken(const std::string& token) {
  const std::string t = upperTrim(token);
  if (t == "A") return AirspaceClass::ClassA;
  if (t == "B") return AirspaceClass::ClassB;
  if (t == "C") return AirspaceClass::ClassC;
  if (t == "D") return AirspaceClass::ClassD;
  if (t == "R") return AirspaceClass::Restricted;
  if (t == "P") return AirspaceClass::Prohibited;
  if (t == "Q") return AirspaceClass::Danger;
  if (t == "W") return AirspaceClass::Warning;
  return AirspaceClass::Other;
}

// Refine OpenAir AC=W entries and named areas to the NXi airspace groups.
void refineAirspaceClass(AirspaceClass& cls, const std::string& name) {
  const std::string n = upperTrim(name);
  if (n.find("TFR") != std::string::npos) {
    cls = AirspaceClass::TFR;
    return;
  }
  if (n.find("ADIZ") != std::string::npos) {
    cls = AirspaceClass::ADIZ;
    return;
  }
  if (n.find("TRSA") != std::string::npos) {
    cls = AirspaceClass::TRSA;
    return;
  }
  if (n.find("MOA") != std::string::npos ||
      n.find("MIL") != std::string::npos) {
    cls = AirspaceClass::MOA;
    return;
  }
  if (n.find("TRAINING") != std::string::npos || n.find(" TRA ") != std::string::npos) {
    cls = AirspaceClass::Training;
    return;
  }
  if (n.find("ALERT") != std::string::npos) {
    cls = AirspaceClass::Alert;
    return;
  }
  if (n.find("CAUTION") != std::string::npos) {
    cls = AirspaceClass::Caution;
    return;
  }
  if (n.find("WARN") != std::string::npos) {
    cls = AirspaceClass::Warning;
    return;
  }
}

// Altitude strings: "SFC"/"GND" -> 0, "UNLIM[ITED]" -> sentinel, "FL180" ->
// 18000, "8400 ft"/"8400ft"/"100 Agl" -> the leading number. Unparseable text
// (e.g. "Ask on 122.8") falls back to the supplied default.
float parseAltitudeFt(const std::string& raw, float fallback) {
  const std::string s = upperTrim(raw);
  if (s.empty()) return fallback;
  if (s.rfind("SFC", 0) == 0 || s.rfind("GND", 0) == 0) return 0.0f;
  if (s.rfind("UNLIM", 0) == 0) return kUnlimitedFt;
  if (s.rfind("FL", 0) == 0) {
    return static_cast<float>(std::atof(s.c_str() + 2) * 100.0);
  }
  const char* p = s.c_str();
  while (*p && !(std::isdigit(static_cast<unsigned char>(*p)) || *p == '.')) ++p;
  if (*p == '\0') return fallback;
  return static_cast<float>(std::atof(p));
}

// Reads one angle as DEG[:MIN[:SEC]] (minutes/seconds may carry a decimal).
// Advances p past the consumed characters; returns degrees as a positive value.
double readAngle(const char*& p) {
  while (*p == ' ' || *p == '\t') ++p;
  char* end = nullptr;
  const double deg = std::strtod(p, &end);
  if (end == p) return -1.0;
  p = end;
  double minutes = 0.0;
  double seconds = 0.0;
  if (*p == ':') {
    ++p;
    minutes = std::strtod(p, &end);
    p = end;
    if (*p == ':') {
      ++p;
      seconds = std::strtod(p, &end);
      p = end;
    }
  }
  return deg + minutes / 60.0 + seconds / 3600.0;
}

// Parses an OpenAir coordinate pair "LAT[N|S] LON[E|W]" (spaces optional before
// the hemisphere letters). Returns false if either half is malformed.
bool parseCoordinate(const char* p, GeoPoint& out) {
  const double latMag = readAngle(p);
  if (latMag < 0.0) return false;
  while (*p == ' ' || *p == '\t') ++p;
  const char latHem = static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
  if (latHem != 'N' && latHem != 'S') return false;
  ++p;

  const double lonMag = readAngle(p);
  if (lonMag < 0.0) return false;
  while (*p == ' ' || *p == '\t') ++p;
  const char lonHem = static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
  if (lonHem != 'E' && lonHem != 'W') return false;

  out.lat = (latHem == 'S') ? -latMag : latMag;
  out.lon = (lonHem == 'W') ? -lonMag : lonMag;
  return std::fabs(out.lat) <= 90.0 && std::fabs(out.lon) <= 180.0;
}

GeoPoint pointFromBearing(const GeoPoint& center, double bearingDeg,
                          double radiusNm) {
  const double northNm = radiusNm * std::cos(bearingDeg * kDegToRad);
  const double eastNm = radiusNm * std::sin(bearingDeg * kDegToRad);
  GeoPoint p;
  p.lat = center.lat + northNm / kNmPerDeg;
  p.lon = center.lon + eastNm / (kNmPerDeg * cosLatClamped(center.lat));
  return p;
}

double bearingFromCenter(const GeoPoint& center, const GeoPoint& pt,
                         double& radiusNm) {
  const double northNm = (pt.lat - center.lat) * kNmPerDeg;
  const double eastNm = (pt.lon - center.lon) * kNmPerDeg * cosLatClamped(center.lat);
  radiusNm = std::sqrt(northNm * northNm + eastNm * eastNm);
  double deg = std::atan2(eastNm, northNm) / kDegToRad;
  if (deg < 0.0) deg += 360.0;
  return deg;
}

void appendArc(std::vector<GeoPoint>& out, const GeoPoint& center,
               double radiusNm, double startDeg, double endDeg, bool clockwise) {
  double sweep = endDeg - startDeg;
  if (clockwise) {
    while (sweep < 0.0) sweep += 360.0;
  } else {
    while (sweep > 0.0) sweep -= 360.0;
  }
  const int steps =
      std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / kArcStepDeg)));
  for (int i = 0; i <= steps; ++i) {
    const double a = startDeg + sweep * (static_cast<double>(i) / steps);
    out.push_back(pointFromBearing(center, a, radiusNm));
  }
}

void appendCircle(std::vector<GeoPoint>& out, const GeoPoint& center,
                  double radiusNm) {
  const int steps =
      std::max(8, static_cast<int>(std::ceil(360.0 / kArcStepDeg)));
  for (int i = 0; i <= steps; ++i) {
    const double a = 360.0 * (static_cast<double>(i) / steps);
    out.push_back(pointFromBearing(center, a, radiusNm));
  }
}

// In-progress airspace accumulated across records until the next AC / EOF.
struct Builder {
  MapAirspace airspace;
  GeoPoint center;
  bool clockwise = true;
};

void flush(std::vector<MapAirspace>& out, Builder& b) {
  if (b.airspace.airspaceClass != AirspaceClass::Other &&
      b.airspace.airspaceClass != AirspaceClass::ClassA &&
      b.airspace.boundary.size() >= 2) {
    out.push_back(std::move(b.airspace));
  }
  b.airspace = MapAirspace{};
  b.center = GeoPoint{};
  b.clockwise = true;
}

// Splits "a,b,c" on commas into trimmed pieces.
std::vector<std::string> splitCommas(const std::string& s) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= s.size()) {
    const std::size_t comma = s.find(',', start);
    const std::size_t end = (comma == std::string::npos) ? s.size() : comma;
    parts.push_back(s.substr(start, end - start));
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return parts;
}

}  // namespace

std::vector<MapAirspace> parseOpenAir(std::istream& in) {
  std::vector<MapAirspace> out;
  Builder b;
  std::string line;

  while (std::getline(in, line)) {
    // Strip a trailing CR (Windows line endings) and leading whitespace.
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }
    std::size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) {
      ++i;
    }
    if (i >= line.size()) continue;
    if (line[i] == '*') continue;  // comment

    // Two-character record key followed by its argument.
    const std::string rest = line.substr(i);
    if (rest.size() < 2) continue;
    const char k0 = static_cast<char>(std::toupper(static_cast<unsigned char>(rest[0])));
    const char k1 = static_cast<char>(std::toupper(static_cast<unsigned char>(rest[1])));
    std::string arg = (rest.size() > 2) ? rest.substr(2) : std::string();
    // Trim a single leading space/= so "AC B" and "V X=.." both work.
    std::size_t a = 0;
    while (a < arg.size() && (arg[a] == ' ' || arg[a] == '\t')) ++a;
    arg = arg.substr(a);

    if (k0 == 'A' && k1 == 'C') {
      flush(out, b);
      b.airspace.airspaceClass = classFromToken(arg);
    } else if (k0 == 'A' && k1 == 'N') {
      b.airspace.name = upperTrim(arg);
      refineAirspaceClass(b.airspace.airspaceClass, b.airspace.name);
    } else if (k0 == 'A' && k1 == 'H') {
      b.airspace.ceilingFt = parseAltitudeFt(arg, kUnlimitedFt);
    } else if (k0 == 'A' && k1 == 'L') {
      b.airspace.floorFt = parseAltitudeFt(arg, 0.0f);
    } else if (k0 == 'V') {
      // Variable assignment: "X=<coord>" (arc center) or "D=<+|->" (direction).
      std::size_t eq = arg.find('=');
      if (eq != std::string::npos) {
        const char var = static_cast<char>(std::toupper(static_cast<unsigned char>(arg[0])));
        const std::string val = arg.substr(eq + 1);
        if (var == 'X') {
          GeoPoint c;
          if (parseCoordinate(val.c_str(), c)) b.center = c;
        } else if (var == 'D') {
          const std::string v = upperTrim(val);
          b.clockwise = !(!v.empty() && v[0] == '-');
        }
      }
    } else if (k0 == 'D' && k1 == 'P') {
      GeoPoint p;
      if (parseCoordinate(arg.c_str(), p)) b.airspace.boundary.push_back(p);
    } else if (k0 == 'D' && k1 == 'C') {
      const double radius = std::atof(arg.c_str());
      if (radius > 0.0) appendCircle(b.airspace.boundary, b.center, radius);
    } else if (k0 == 'D' && k1 == 'A') {
      const std::vector<std::string> parts = splitCommas(arg);
      if (parts.size() >= 3) {
        const double radius = std::atof(parts[0].c_str());
        const double start = std::atof(parts[1].c_str());
        const double end = std::atof(parts[2].c_str());
        if (radius > 0.0) {
          appendArc(b.airspace.boundary, b.center, radius, start, end,
                    b.clockwise);
        }
      }
    } else if (k0 == 'D' && k1 == 'B') {
      const std::vector<std::string> parts = splitCommas(arg);
      if (parts.size() >= 2) {
        GeoPoint p1;
        GeoPoint p2;
        if (parseCoordinate(parts[0].c_str(), p1) &&
            parseCoordinate(parts[1].c_str(), p2)) {
          double r1 = 0.0;
          double r2 = 0.0;
          const double startDeg = bearingFromCenter(b.center, p1, r1);
          const double endDeg = bearingFromCenter(b.center, p2, r2);
          appendArc(b.airspace.boundary, b.center, r1, startDeg, endDeg,
                    b.clockwise);
        }
      }
    }
    // Any other record (AT, SP, SB, DY, ...) is ignored.
  }
  flush(out, b);
  return out;
}

std::vector<MapAirspace> airspacesNear(const std::vector<MapAirspace>& src,
                                       double lat, double lon, float rangeNm,
                                       std::size_t maxCount) {
  std::vector<MapAirspace> result;
  if (maxCount == 0) return result;

  const double cosLat = cosLatClamped(lat);
  // Generous box (range + 25% margin) so partially-visible airspace is kept.
  const double dLat = (rangeNm / kNmPerDeg) * 1.25;
  const double dLon = (rangeNm / (kNmPerDeg * cosLat)) * 1.25;
  const double minLat = lat - dLat;
  const double maxLat = lat + dLat;
  const double minLon = lon - dLon;
  const double maxLon = lon + dLon;

  // Candidates that overlap the box, tagged with their distance (nm^2, scaled
  // by latitude) from ownship to the nearest point of their bounding box. We
  // rank by distance and keep the closest maxCount: in dense areas (e.g.
  // Florida) far more than maxCount airspaces overlap a wide query box, so a
  // file-order cut would silently drop airspace right at the aircraft (the
  // KRSW Class C case). Distance ranking guarantees the nearest are kept.
  struct Candidate {
    const MapAirspace* airspace;
    double distSq;
  };
  std::vector<Candidate> candidates;

  for (const MapAirspace& as : src) {
    if (as.airspaceClass == AirspaceClass::Other ||
        as.airspaceClass == AirspaceClass::ClassA || as.boundary.empty()) {
      continue;
    }
    double bMinLat = as.boundary.front().lat;
    double bMaxLat = bMinLat;
    double bMinLon = as.boundary.front().lon;
    double bMaxLon = bMinLon;
    for (const GeoPoint& g : as.boundary) {
      bMinLat = std::min(bMinLat, g.lat);
      bMaxLat = std::max(bMaxLat, g.lat);
      bMinLon = std::min(bMinLon, g.lon);
      bMaxLon = std::max(bMaxLon, g.lon);
    }
    const bool overlaps = bMinLat <= maxLat && bMaxLat >= minLat &&
                          bMinLon <= maxLon && bMaxLon >= minLon;
    if (!overlaps) continue;

    // Distance from ownship to the nearest point of the bounding box (zero when
    // ownship is inside it), in nm, so a large airspace whose edge is close
    // ranks ahead of a small one whose center is far.
    const double clampedLat = std::min(std::max(lat, bMinLat), bMaxLat);
    const double clampedLon = std::min(std::max(lon, bMinLon), bMaxLon);
    const double dNorthNm = (lat - clampedLat) * kNmPerDeg;
    const double dEastNm = (lon - clampedLon) * kNmPerDeg * cosLat;
    candidates.push_back({&as, dNorthNm * dNorthNm + dEastNm * dEastNm});
  }

  if (candidates.size() > maxCount) {
    std::nth_element(candidates.begin(), candidates.begin() + maxCount,
                     candidates.end(),
                     [](const Candidate& a, const Candidate& b) {
                       return a.distSq < b.distSq;
                     });
    candidates.resize(maxCount);
  }

  result.reserve(candidates.size());
  for (const Candidate& c : candidates) result.push_back(*c.airspace);
  return result;
}

}  // namespace avionics
