#pragma once

#include <optional>
#include <string>

namespace avionics {

// The decoded fields of a raw METAR shown on the WPT - Weather Information page
// (G1000 NXi Pilot's Guide §5.2). The real unit decodes the full report into a
// labelled list and then shows the verbatim "ORIGINAL METAR TEXT" below it, so
// this carries every field the page lists. Each value is empty when its group
// is absent or unparseable (the page then omits that row).
struct MetarDecoded {
  // Observation time group, verbatim ("161453Z" = day 16, 1453 Zulu).
  std::optional<std::string> observationTime;

  // Wind. Direction is the true heading in degrees (empty when variable/calm);
  // windVariable marks a "VRB" direction; windCalm marks "00000KT". Speed and
  // the optional gust are in knots.
  std::optional<int> windDirectionDeg;
  bool windVariable = false;
  bool windCalm = false;
  std::optional<int> windSpeedKt;
  std::optional<int> windGustKt;

  // Prevailing visibility, verbatim with its unit ("10SM", "1/2SM", "CAVOK").
  std::optional<std::string> visibility;

  // Sky-condition groups, space-joined verbatim ("FEW014", "SCT025 BKN040",
  // "SKC", "CLR"). Empty when none were reported.
  std::optional<std::string> clouds;

  std::optional<float> temperatureC;
  std::optional<float> dewPointC;
  std::optional<float> altimeterInHg;  // from the "Annnn" group, inches Hg
};

// Extract the decoded fields from a raw METAR string. Tolerant of a leading WMO
// data-type designator (e.g. "SA "), the station id, arbitrary whitespace, and
// a trailing "RMK" remarks section (ignored, since its higher-precision T-group
// duplicates the body the page displays). Keeps no reference to the input.
MetarDecoded parseMetar(const std::string& rawMetar);

}  // namespace avionics
