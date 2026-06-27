#include "avionics/MetarParser.h"

#include <cctype>
#include <sstream>

namespace avionics {
namespace {

bool allDigits(const std::string& s, std::size_t from, std::size_t to) {
  if (from >= to) return false;
  for (std::size_t i = from; i < to; ++i) {
    if (std::isdigit(static_cast<unsigned char>(s[i])) == 0) return false;
  }
  return true;
}

int parseInt(const std::string& s, std::size_t from, std::size_t to) {
  int value = 0;
  for (std::size_t i = from; i < to; ++i) value = value * 10 + (s[i] - '0');
  return value;
}

// Parse a METAR temperature/dew-point field ("24", "M03", "08") to degrees C.
// The "M" prefix marks a negative value (e.g. "M05" = -5 C). Returns false for
// an empty or non-numeric field so cloud / visibility groups never match.
bool parseTempField(const std::string& s, float& out) {
  std::size_t i = 0;
  bool negative = false;
  if (!s.empty() && (s[0] == 'M' || s[0] == 'm')) {
    negative = true;
    i = 1;
  }
  if (i >= s.size() || !allDigits(s, i, s.size())) return false;
  const int value = parseInt(s, i, s.size());
  out = negative ? -static_cast<float>(value) : static_cast<float>(value);
  return true;
}

// "ddhhmmZ" observation-time group (e.g. "161453Z").
bool isObservationTime(const std::string& t) {
  return t.size() == 7 && t.back() == 'Z' && allDigits(t, 0, 6);
}

// "Annnn" US altimeter group (inches Hg in hundredths, e.g. "A3009" = 30.09").
bool parseAltimeter(const std::string& t, float& inHg) {
  if (t.size() != 5 || t[0] != 'A' || !allDigits(t, 1, 5)) return false;
  inHg = static_cast<float>(parseInt(t, 1, 5)) / 100.0f;
  return true;
}

// Wind group "<dir|VRB><spd>[Gust]<unit>" with unit KT / MPS / KMH (e.g.
// "08005KT", "34015G25KT", "VRB03KT", "00000KT").
bool parseWind(const std::string& t, MetarDecoded& out) {
  std::size_t unitLen = 0;
  if (t.size() > 2 && t.compare(t.size() - 2, 2, "KT") == 0) {
    unitLen = 2;
  } else if (t.size() > 3 && (t.compare(t.size() - 3, 3, "MPS") == 0 ||
                              t.compare(t.size() - 3, 3, "KMH") == 0)) {
    unitLen = 3;
  } else {
    return false;
  }
  const std::string body = t.substr(0, t.size() - unitLen);
  if (body.size() < 5) return false;

  bool variable = false;
  const std::string dir = body.substr(0, 3);
  if (dir == "VRB") {
    variable = true;
  } else if (!allDigits(dir, 0, 3)) {
    return false;
  }

  // Speed digits, then an optional "G<gust>".
  const std::size_t gPos = body.find('G', 3);
  const std::size_t spdEnd = gPos == std::string::npos ? body.size() : gPos;
  if (!allDigits(body, 3, spdEnd)) return false;
  const int speed = parseInt(body, 3, spdEnd);
  int gust = 0;
  if (gPos != std::string::npos) {
    if (!allDigits(body, gPos + 1, body.size())) return false;
    gust = parseInt(body, gPos + 1, body.size());
  }

  if (!variable && dir == "000" && speed == 0) {
    out.windCalm = true;
    return true;
  }
  out.windVariable = variable;
  if (!variable) out.windDirectionDeg = parseInt(dir, 0, 3);
  out.windSpeedKt = speed;
  if (gPos != std::string::npos) out.windGustKt = gust;
  return true;
}

// Visibility: a statute-mile group ("10SM", "1/2SM", "M1/4SM") or "CAVOK".
bool isVisibility(const std::string& t) {
  if (t == "CAVOK") return true;
  return t.size() > 2 && t.compare(t.size() - 2, 2, "SM") == 0;
}

// Sky-condition group: a coverage prefix + height ("FEW014", "OVC250///"),
// vertical visibility ("VV002"), or a clear-sky token.
bool isCloud(const std::string& t) {
  if (t == "SKC" || t == "CLR" || t == "NSC" || t == "NCD") return true;
  static const char* const kCover[] = {"FEW", "SCT", "BKN", "OVC", "VV"};
  for (const char* c : kCover) {
    const std::size_t n = std::char_traits<char>::length(c);
    if (t.size() >= n + 3 && t.compare(0, n, c) == 0 && allDigits(t, n, n + 3)) {
      return true;
    }
  }
  return false;
}

}  // namespace

MetarDecoded parseMetar(const std::string& rawMetar) {
  MetarDecoded out;
  std::istringstream tokens(rawMetar);
  std::string tok;
  bool windSeen = false;
  while (tokens >> tok) {
    // The remarks section repeats some groups at higher precision but is not
    // shown decoded by the unit, so stop before it.
    if (tok == "RMK") break;

    if (!out.observationTime && isObservationTime(tok)) {
      out.observationTime = tok;
      continue;
    }
    if (!windSeen && parseWind(tok, out)) {
      windSeen = true;
      continue;
    }
    float inHg = 0.0f;
    if (parseAltimeter(tok, inHg)) {
      out.altimeterInHg = inHg;
      continue;
    }
    if (!out.visibility && isVisibility(tok)) {
      out.visibility = tok;
      continue;
    }
    if (isCloud(tok)) {
      out.clouds = out.clouds ? *out.clouds + " " + tok : tok;
      continue;
    }
    // Temperature/dew-point group "<temp>/<dew>" (e.g. "24/21", "M03/M05").
    // The temperature parse gates false positives from RVR ("R06/2000") and
    // fractional visibility ("1/2SM", already consumed above).
    const std::size_t slash = tok.find('/');
    if (slash != std::string::npos && slash > 0 && slash + 1 < tok.size()) {
      float temp = 0.0f;
      float dew = 0.0f;
      if (parseTempField(tok.substr(0, slash), temp) &&
          parseTempField(tok.substr(slash + 1), dew)) {
        out.temperatureC = temp;
        out.dewPointC = dew;
      }
    }
  }
  return out;
}

}  // namespace avionics
