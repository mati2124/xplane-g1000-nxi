#pragma once

namespace avionics {

// NAV/COM units on the PFD/MFD top bar (Pilot's Guide, Audio Panel).
enum class RadioUnit { Nav1, Nav2, Com1, Com2 };

// Which side of the NAV/COM bar a unit belongs to. The G1000 tunes COM and NAV
// independently (separate knobs and separate cyan tuning cursors), so the bar
// shows one selected COM and one selected NAV at the same time.
enum class RadioBand { None, Com, Nav };

inline constexpr RadioBand radioBandOf(RadioUnit unit) {
  return (unit == RadioUnit::Com1 || unit == RadioUnit::Com2) ? RadioBand::Com
                                                              : RadioBand::Nav;
}

inline constexpr float kNavFreqMinMhz = 108.0f;
inline constexpr float kNavFreqMaxMhz = 117.95f;
inline constexpr float kComFreqMinMhz = 118.0f;
inline constexpr float kComFreqMaxMhz = 136.975f;

// Steps the NAV standby frequency one channel (50 kHz) in the given direction,
// wrapping inside the VOR band. This is the small (inner) tuning knob.
float stepNavStandbyMhz(float mhz, int direction);

// Steps the COM standby frequency one channel (25 kHz) in the given direction,
// wrapping inside the aviation COM band. This is the small (inner) tuning knob.
float stepComStandbyMhz(float mhz, int direction);

// Steps the whole-MHz part of the standby frequency by one in the given
// direction, preserving the kHz fraction and wrapping inside the band. These
// are the large (outer) tuning knobs (Pilot's Guide, NAV/COM tuning).
float stepNavStandbyMhzCoarse(float mhz, int direction);
float stepComStandbyMhzCoarse(float mhz, int direction);

}  // namespace avionics
