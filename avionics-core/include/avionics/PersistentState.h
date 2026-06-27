#pragma once

// Durable display preferences that survive between flights (i.e. across
// power/restart cycles), mirroring the real G1000 NXi's user settings: the
// softkey-selectable display options the pilot expects to find unchanged the
// next time the avionics power up.
//
// Only genuinely durable display preferences are captured here. Transient,
// flight-specific state (timers, the in-progress XPDR code entry, the active
// flight plan, the live CDI source that tracks the nav feed, etc.) is left out
// so it always starts fresh, as on the real unit.
//
// Both shells share one capture/apply path and one text format: each shell
// owns where the bytes live (the standalone settings file, X-Plane's
// preferences directory) but the field set and serialization are defined once
// here.

#include <array>
#include <map>
#include <string>

#include "avionics/MapRange.h"
#include "avionics/MfdController.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/MapView.h"

namespace avionics {

// Durable PFD softkey display options (Map/HSI, PFD Opt, and the display
// toggles), plus the inset-map range.
struct PfdPersistentState {
  MapLayout mapLayout = MapLayout::Inset;
  MapDetail mapDetail = MapDetail::All;
  WindOption windOption = WindOption::Option2;
  std::array<bool, kDisplayToggleCount> toggles{};
  int insetRangeIndex = kMapRangeDefaultIndex;
  int insetRangeSavedVersion = 0;
};

// Durable MFD navigation-map display options and range.
struct MfdPersistentState {
  TerrainDisplay terrain = TerrainDisplay::Topo;
  AirwayDisplay airways = AirwayDisplay::Off;
  bool showTraffic = false;
  bool showWeather = false;
  MapDetail detail = MapDetail::All;
  MapOrientation orientation = MapOrientation::NorthUp;
  int rangeIndex = kMapRangeDefaultIndex;
  int rangeSavedVersion = 0;
};

// Pilot-editable V-speed reference bugs (Glide, Vr, Vx, Vy) from the
// Timer/References window, captured so they can be remembered per aircraft.
// X-Plane does not publish these (unlike the Vne/Vno envelope), so the pilot's
// entries are stored here keyed by aircraft type and restored on the next load
// of that airframe. Defaults match the delivered Cessna 172S.
struct VspeedProfile {
  std::array<bool, kVspeedRefCount> on{true, true, true, true};
  std::array<float, kVspeedRefCount> kt{kDefaultVspeedKt[0], kDefaultVspeedKt[1],
                                        kDefaultVspeedKt[2], kDefaultVspeedKt[3]};
};

bool operator==(const VspeedProfile& a, const VspeedProfile& b);
inline bool operator!=(const VspeedProfile& a, const VspeedProfile& b) {
  return !(a == b);
}

// The full persisted avionics display state (both screens).
struct AvionicsPersistentState {
  PfdPersistentState pfd;
  MfdPersistentState mfd;
  // V-speed reference bugs keyed by aircraft ICAO type (e.g. "C172", "SF50").
  std::map<std::string, VspeedProfile> vspeedByAircraft;
};

bool operator==(const PfdPersistentState& a, const PfdPersistentState& b);
bool operator==(const MfdPersistentState& a, const MfdPersistentState& b);
bool operator==(const AvionicsPersistentState& a,
                const AvionicsPersistentState& b);
inline bool operator!=(const AvionicsPersistentState& a,
                       const AvionicsPersistentState& b) {
  return !(a == b);
}

// Read the durable options out of a live controller into `out`.
void capturePfdState(const SoftkeyController& controller,
                     PfdPersistentState& out);
void captureMfdState(const MfdController& controller, MfdPersistentState& out);

// Restore the durable options into a live controller (refreshing any
// state-carrying softkey labels so the bar reflects the loaded values).
void applyPfdState(SoftkeyController& controller, const PfdPersistentState& s);
void applyMfdState(MfdController& controller, const MfdPersistentState& s);

// Read / restore the pilot's V-speed reference bugs (values + On/Off) from a
// controller, used to remember them per aircraft.
void captureVspeeds(const SoftkeyController& controller, VspeedProfile& out);
void applyVspeeds(SoftkeyController& controller, const VspeedProfile& s);

// Per-aircraft V-speed memory. Call once per frame with the live PFD controller
// and the current aircraft ICAO type. On an aircraft change it restores that
// airframe's saved bugs (or the defaults); otherwise it captures the pilot's
// current bugs into `store`. Returns true when `store` changed and the shell
// should persist it.
class VspeedAircraftMemory {
 public:
  bool sync(SoftkeyController& controller, const std::string& aircraftIcao,
            std::map<std::string, VspeedProfile>& store);

 private:
  std::string lastIcao_;
  bool primed_ = false;
};

// Append the state as `key=value` text lines (one per field) to `out`, so a
// shell can embed them in its own preferences file.
void appendStateLines(const AvionicsPersistentState& state, std::string& out);

// Apply one parsed `key=value` line to `state`. Returns true if the key is one
// of ours (so a shell can fall through to its own keys when it is not).
bool applyStateLine(const std::string& key, const std::string& value,
                    AvionicsPersistentState& state);

}  // namespace avionics
