#pragma once

#include <optional>
#include <string>

namespace avionics {

// Datalink weather for one reporting station (the WPT - Weather Information
// page, G1000 NXi Pilot's Guide §5.2). The page decodes the METAR into a
// labelled field list (via avionics::parseMetar) and shows the verbatim
// "ORIGINAL METAR TEXT" and raw TAF below it, so this only needs to carry the
// raw report strings; decoding happens in the renderer.
struct StationWeather {
  std::string icao;       // station identifier, e.g. "KFMY"
  std::string rawMetar;   // verbatim METAR report (empty when none)
  std::string rawTaf;     // verbatim TAF forecast (empty when none)

  bool hasMetar() const { return !rawMetar.empty(); }
  bool hasTaf() const { return !rawTaf.empty(); }
};

// "Where station weather comes from" for the WPT - Weather Information page,
// mirroring the WeatherRadarSource / NavFeatureSource seams: the platform-
// agnostic core asks for weather by airport ICAO and a shell supplies it (the
// X-Plane plugin reads X-Plane 12's downloaded METAR files; the standalone
// mock returns a canned report for screenshots). Returns nullopt when no report
// is available for the station, so the page shows the dashed "no data" state.
class StationWeatherSource {
 public:
  virtual ~StationWeatherSource() = default;

  virtual std::optional<StationWeather> weatherForStation(
      const std::string& icao) const = 0;
};

}  // namespace avionics
