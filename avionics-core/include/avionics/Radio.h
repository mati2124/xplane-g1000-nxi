#pragma once

#include "avionics/FlightData.h"

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

// Audio volume range (0..1) and one-click step for the VOL/SQ and VOL/ID knobs.
inline constexpr float kRadioVolumeMin = 0.0f;
inline constexpr float kRadioVolumeMax = 1.0f;
inline constexpr float kRadioVolumeStep = 0.05f;
// How long the volume percentage stays shown in the NavCom box after a change
// (Pilot's Guide: "Volume level indication remains for two seconds").
inline constexpr double kRadioVolumeShownSeconds = 2.0;

// The FlightData volume field backing each radio unit's audio level.
inline constexpr float FlightData::* radioVolumeMember(RadioUnit unit) {
  switch (unit) {
    case RadioUnit::Nav1:
      return &FlightData::nav1Volume;
    case RadioUnit::Nav2:
      return &FlightData::nav2Volume;
    case RadioUnit::Com1:
      return &FlightData::com1Volume;
    case RadioUnit::Com2:
      return &FlightData::com2Volume;
  }
  return &FlightData::com1Volume;
}

// The FlightData Morse-ident audio flag backing each NAV unit (the NAV VOL/ID
// knob press). COM units have no ident audio, so they map to NAV1's flag (the
// COM VOL/SQ press is inert -- X-Plane has no squelch dataref).
inline constexpr bool FlightData::* navIdentAudioMember(RadioUnit unit) {
  return unit == RadioUnit::Nav2 ? &FlightData::nav2IdentAudio
                                 : &FlightData::nav1IdentAudio;
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
