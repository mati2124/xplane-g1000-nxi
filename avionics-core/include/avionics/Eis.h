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

// Turbofan (Cirrus Vision SF50 / Perspective Touch+) EIS channels. The jet
// engine page (Pilot's Guide for the Vision SF50, 190-02470-02, Fig. 3-2)
// shows turbine parameters, a two-tank fuel system, a four-source electrical
// system, and airframe synoptics (gear, trim, flaps, pressurization) that the
// piston Cessna fit has no equivalent for.
inline constexpr const char* kThrustPct = "eng.thrust_pct";   // % thrust
inline constexpr const char* kN1Pct = "eng.n1";              // fan speed %
inline constexpr const char* kN2Pct = "eng.n2";              // HP spool %
inline constexpr const char* kIttC = "eng.itt_c";            // turbine temp degC
inline constexpr const char* kOilTempC = "eng.oil_temp_c";   // oil temp degC
inline constexpr const char* kFuelTempC = "fuel.temp_c";     // fuel temp degC
inline constexpr const char* kFuelSelected = "fuel.selected"; // 0=L,1=R tank
inline constexpr const char* kEmerBusVolts = "elec.emer_bus";
inline constexpr const char* kBatt1Amps = "elec.batt1_amps";
inline constexpr const char* kBatt2Amps = "elec.batt2_amps";
inline constexpr const char* kGen1Amps = "elec.gen1_amps";
inline constexpr const char* kGen2Amps = "elec.gen2_amps";
inline constexpr const char* kGearNose = "gear.nose";        // 0=up,1=down/lkd
inline constexpr const char* kGearLeft = "gear.left";
inline constexpr const char* kGearRight = "gear.right";
inline constexpr const char* kPitchTrim = "trim.pitch";      // -1..1 (-=nose dn)
inline constexpr const char* kRollTrim = "trim.roll";        // -1..1 (-=left)
inline constexpr const char* kRudderTrim = "trim.rudder";      // -1..1 (-=left)
inline constexpr const char* kFlapsActual = "flaps.actual";  // 0..1
inline constexpr const char* kFlapsCommanded = "flaps.commanded";  // 0..1
inline constexpr const char* kCabinRateFpm = "cabin.rate_fpm";
inline constexpr const char* kCabinAltFt = "cabin.alt_ft";
inline constexpr const char* kCabinDiffPsi = "cabin.diff_psi";
inline constexpr const char* kDestElevFt = "cabin.dest_elev_ft";
}  // namespace eis_channels

// Per-aircraft engine display definition loaded from a plain-text file (see
// parseEisText). Mirrors how the real G1000 NXi reads its engine configuration
// from the airframe rather than hardcoding one layout in avionics firmware.

enum class EisGaugeType {
  RpmDial,
  Bar,
  // Two-tank fuel quantity on a single shared horizontal bar with the L pointer
  // riding above the track and the R pointer below it (G1000 NXi Pilot's Guide
  // Fig. 3-17). Reads LEFT/RIGHT channels and tags.
  FuelQty,
  Readout,
  Electrical,
  FuelQtyVert,
  Dial,
};

// Overall engine-page layout family. Piston is the single-column Cessna Nav III
// stack (RPM dial + bars + electrical rows). Turbofan is the dense Cirrus
// Vision SF50 grid (Perspective Touch+ Pilot's Guide Fig. 3-2): a % thrust arc,
// a row of vertical turbine bars, a two-tank fuel block, a four-source
// electrical block, and the gear/trim/flaps/pressurization synoptics. The
// renderer dispatches on this; gauge limits/bands/bindings stay data-driven.
enum class EisStripStyle {
  Piston,
  Turbofan,
  Turboprop,
  Caravan,
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
  // Cyan reference bug on the gauge (turbofan N1 / % thrust takeoff setting).
  float bug = 0.0f;
  bool hasBug = false;
  // Optional live channel for a moving cyan reference bug.
  std::string bugChannel;
  // Optional live channel for a moving redline marker.
  std::string redlineChannel;
  // Number of graduation intervals drawn beneath a Bar track (0 = none). The
  // real Cessna FFLOW and EGT bars carry a combed scale; OIL/VAC bars do not.
  int ticks = 0;
  // When set, a Bar draws its min/max numerals at the track ends (FFLOW "0"/
  // "20"); OIL/EGT/VAC bars omit them on the real unit.
  bool scale = false;
  // When set, this gauge is one of the primary engine indications shown on the
  // PFD's reversionary (display-backup) strip when the MFD is dark. The
  // turbofan renderer's reduced strip draws only the PFD-flagged gauges (the
  // dense synoptic grid does not fit beside the flight instruments); the piston
  // strip fits in full and ignores this. Authored with the `PFD` keyword.
  bool pfd = false;
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
  EisStripStyle style = EisStripStyle::Piston;
  std::vector<EisSection> sections;
  std::vector<EisDataBinding> bindings;

  bool empty() const {
    for (const EisSection& s : sections) {
      if (!s.gauges.empty()) return false;
    }
    return true;
  }

  // First gauge bound to `channel`, or null. The turbofan renderer uses this to
  // pull a parameter's limits/bands/bug from the data-driven layout while
  // placing it at a fixed position in the SF50 grid.
  const EisGauge* gaugeForChannel(const std::string& channel) const {
    for (const EisSection& s : sections) {
      for (const EisGauge& g : s.gauges) {
        if (g.channel == channel) return &g;
      }
    }
    return nullptr;
  }
};

// Resolve a band color token from an EIS file to the shared palette.
Color eisBandColor(EisBandColor color);

// Parses the line-oriented EIS definition format into an EisLayout.
EisLayout parseEisText(const std::string& text);

// Candidate paths for a per-aircraft EIS file, in search order:
//   1. `explicitSelector` (a CLI/path override) when it points at a regular
//      file.
//   2. Beside the .acf when `aircraftAcfRelativePath` is non-empty (X-Plane's
//      sim/aircraft/view/acf_relative_path): g1000_eis.txt and <acf_stem>_eis.txt.
//   3. `typeKeyedPath` — a user-droppable, ICAO-keyed asset (assets/eis/<icao>.eis)
//      resolved by the store; lets users add any aircraft without a rebuild.
//   4. `defaultBundledPath` — the profile's bundled sample (AVIONICS_DEFAULT_EIS).
// Empty arguments are skipped, as are paths that are not regular files.
std::vector<std::string> candidateEisPaths(const std::string& explicitSelector,
                                            const std::string& aircraftAcfRelativePath,
                                            const std::string& typeKeyedPath,
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
  // Preferred hook: the loaded aircraft's ICAO type code (acf_ICAO) plus its
  // .acf relative path. Lets the store pick the per-aircraft EIS profile by
  // type. The default forwards the path to setAircraftAcfRelativePath so older
  // sources keep working.
  virtual void setAircraftIdentity(const std::string& icaoType,
                                   const std::string& acfRelativePath) {
    (void)icaoType;
    setAircraftAcfRelativePath(acfRelativePath);
  }
  virtual void refreshIfChanged() {}
};

}  // namespace avionics
