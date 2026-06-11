#pragma once

#include <string>
#include <vector>

#include "avionics/Color.h"

namespace avionics {

// Logical channel ids referenced by EIS layout gauges and BIND lines. Aircraft
// authors use these ids consistently so one layout file can be shared across
// shells; the BIND section maps each channel to a sim-specific dataref path.
namespace eis_channels {
inline constexpr const char* kEngRpm = "eng.rpm";
inline constexpr const char* kFuelFlow = "eng.fuel_flow";
inline constexpr const char* kOilPres = "eng.oil_pres";
inline constexpr const char* kOilTemp = "eng.oil_temp";
inline constexpr const char* kEgt = "eng.egt";
inline constexpr const char* kVacuum = "eng.vacuum";
inline constexpr const char* kFuelQtyLeft = "fuel.qty_left";
inline constexpr const char* kFuelQtyRight = "fuel.qty_right";
inline constexpr const char* kEngHours = "eng.hours";
inline constexpr const char* kBusVoltsMain = "elec.bus_main";
inline constexpr const char* kBusVoltsEss = "elec.bus_ess";
inline constexpr const char* kBattAmpsMain = "elec.batt_main";
inline constexpr const char* kBattAmpsStandby = "elec.batt_standby";
}  // namespace eis_channels

// Per-aircraft engine display definition loaded from a plain-text file (see
// parseEisText). Mirrors how the real G1000 NXi reads its engine configuration
// from the airframe rather than hardcoding one layout in avionics firmware.

enum class EisGaugeType {
  RpmDial,
  Bar,
  Readout,
  Electrical,
};

enum class EisBandColor {
  Green,
  Yellow,
  Red,
};

struct EisBand {
  EisBandColor color = EisBandColor::Green;
  float lo = 0.0f;
  float hi = 0.0f;
};

struct EisGauge {
  EisGaugeType type = EisGaugeType::Bar;
  std::string label;
  std::string channel;
  std::string channelRight;
  std::string leftTag;
  std::string rightTag;
  float min = 0.0f;
  float max = 100.0f;
  float redline = 0.0f;
  bool hasRedline = false;
  std::vector<EisBand> bands;
  std::string format = "%.1f";
};

struct EisSection {
  std::string title;
  std::vector<EisGauge> gauges;
};

struct EisDataBinding {
  std::string channel;
  std::string datarefPath;
  float scale = 1.0f;
  float offset = 0.0f;
};

struct EisLayout {
  std::string stripTitle = "ENGINE";
  std::vector<EisSection> sections;
  std::vector<EisDataBinding> bindings;

  bool empty() const {
    for (const EisSection& s : sections) {
      if (!s.gauges.empty()) return false;
    }
    return true;
  }
};

// Resolve a band color token from an EIS file to the shared palette.
Color eisBandColor(EisBandColor color);

// Parses the line-oriented EIS definition format into an EisLayout.
EisLayout parseEisText(const std::string& text);

// Candidate paths for a per-aircraft EIS file, in search order. When
// `aircraftAcfRelativePath` is non-empty (X-Plane's sim/aircraft/view/
// acf_relative_path), looks beside the .acf for g1000_eis.txt and
// <acf_stem>_eis.txt. `explicitSelector` wins when it points at a regular file.
// `defaultBundledPath` is the build-time sample (AVIONICS_DEFAULT_EIS).
std::vector<std::string> candidateEisPaths(const std::string& explicitSelector,
                                            const std::string& aircraftAcfRelativePath,
                                            const std::string& defaultBundledPath);

// "Where the aircraft's EIS layout comes from." Shells implement this by
// loading the author-editable file from disk (mirroring ChecklistSource).
class EisSource {
 public:
  virtual ~EisSource() = default;
  virtual bool ready() const = 0;
  virtual const EisLayout& layout() const = 0;
  virtual void setAircraftAcfRelativePath(const std::string& acfRelativePath) {
    (void)acfRelativePath;
  }
  virtual void refreshIfChanged() {}
};

}  // namespace avionics
