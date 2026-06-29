#pragma once

#include <string>
#include <unordered_map>

namespace avionics {

// Standard ISA barometric pressure (29.92 in Hg / 1013 hPa). When the current
// setting matches this value the PFD BARO box displays "STD BARO" (G1000 Pilot's
// Guide, Standard Barometric Setting).
inline constexpr float kBaroStandardInHg = 29.92f;
inline constexpr float kBaroStandardEpsilonInHg = 0.005f;

inline bool isBaroStandard(float baroInHg) {
  const float delta = baroInHg - kBaroStandardInHg;
  return delta <= kBaroStandardEpsilonInHg && delta >= -kBaroStandardEpsilonInHg;
}

// Cessna 172S defaults for the PFD airspeed-tape color bands (KIAS). X-Plane
// replaces these from sim/aircraft/view/acf_V* when a live feed is connected.
inline constexpr float kDefaultAirspeedVsoKt = 33.0f;
inline constexpr float kDefaultAirspeedVfeKt = 85.0f;
inline constexpr float kDefaultAirspeedVs1Kt = 48.0f;
inline constexpr float kDefaultAirspeedVnoKt = 129.0f;
inline constexpr float kDefaultAirspeedVneKt = 163.0f;

inline float sanitizeAirspeedEnvelopeKt(float kt, float fallbackKt) {
  return kt > 1.0f ? kt : fallbackKt;
}

// Active navigation source annunciated on the HSI / CDI.
enum class CdiSource { Gps, Nav1, Nav2 };

// Vertical deviation indicator (VDI) shown on a scale to the LEFT of the
// altimeter (G1000 NXi Pilot's Guide, Flight Instruments). The NXi shows a
// green diamond for an ILS Glideslope, a magenta diamond for a GPS Glidepath,
// and a magenta pointer for VNAV vertical deviation. When an ILS localizer is
// tuned but no glideslope is being received, "NO GS" is shown in its place.
enum class VerticalDeviationKind { None, Glideslope, Glidepath, Vnav };

// Marker beacon receiver state, annunciated to the left of the altimeter:
// outer (cyan "O"), middle (amber "M"), inner (white "I").
enum class MarkerBeacon { None, Outer, Middle, Inner };

// Active VNAV profile (G1000 NXi Pilot's Guide, Section 6 "Vertical
// Navigation"). The FMS builds a descent path to the active altitude constraint
// at the default flight-path angle, computing the required vertical speed to
// track it, the top-of-descent point, and the deviation from the path. Drives
// the FPL page "Active VNV Profile" box and the PFD vertical deviation pointer.
struct VnvProfile {
  bool active = false;               // a valid VNAV target exists ahead
  std::string targetWpt;             // ident of the constrained waypoint
  int targetAltFt = 0;               // its altitude constraint
  float vsTargetFpm = 0.0f;          // path descent rate at current GS (down<0)
  float vsRequiredFpm = 0.0f;        // VS needed now to make the constraint
  float fpaDeg = 0.0f;               // path flight-path angle
  float distanceToTodNm = 0.0f;      // distance to top of descent (<=0 past TOD)
  int timeToTodSec = 0;              // time to TOD at current ground speed
  bool capturing = false;            // past TOD: descending on the path
  float verticalDeviationFt = 0.0f;  // current altitude minus path altitude
  // Geographic position of the top of descent along the route, for the MFD map
  // "TOD" marker (G1000 NXi Pilot's Guide, Section 5). Valid only while the TOD
  // is still ahead of the aircraft (distanceToTodNm > 0).
  bool todValid = false;
  double todLat = 0.0;
  double todLon = 0.0;
  // Geographic position of the bottom of descent (where the path levels at the
  // target constraint, i.e. the target waypoint), for the MFD map "BOD" marker.
  // Valid while the target leg is still ahead of the aircraft.
  bool bodValid = false;
  double bodLat = 0.0;
  double bodLon = 0.0;
};

// Decoded, platform-agnostic flight state consumed by the gauges.
// Units are chosen to match what the displays render directly, so neither
// shell has to do conversions in the hot path.
struct FlightData {
  float airspeedKts = 0.0f;
  float altitudeFt = 0.0f;
  float headingDeg = 0.0f;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  float verticalSpeedFpm = 0.0f;
  float slipSkidDeg = 0.0f;

  // Sensor/system validity for reversionary red-X annunciation. Real glass
  // cockpits group these by source: the AHRS drives attitude and heading, and
  // the air data computer (ADC) drives airspeed, altitude, and vertical speed.
  // When a source fails, the affected instrument(s) draw a red X instead of
  // (potentially misleading) data.
  bool attitudeValid = true;       // AHRS attitude (pitch/roll)
  bool headingValid = true;        // AHRS heading / HSI compass
  bool airspeedValid = true;       // ADC
  bool altitudeValid = true;       // ADC
  bool verticalSpeedValid = true;  // ADC

  // Per-radio and transponder health, each backed by its own X-Plane failure
  // dataref so an individual box can fail on its own (e.g. COM2 fails while
  // COM1 keeps working). A failed radio row draws a red X over its frequency
  // cell; a failed transponder shows "XPDR FAIL" in the bottom info panel.
  bool nav1Valid = true;
  bool nav2Valid = true;
  bool com1Valid = true;
  bool com2Valid = true;
  bool transponderValid = true;

  // Whole-feed health, distinct from the per-sensor flags above. The engine
  // clears this (alongside every sensor flag) when the data link stops
  // delivering fresh data, so the non-sensor readouts in the top NAV/COM bar
  // and bottom info panel (radios, FMA, transponder, OAT, clock) blank out /
  // show amber dashes instead of stale values, matching the red-X'd gauges.
  bool dataLinkValid = true;

  // GDU electrical power, mirroring the real Cessna Nav III G1000 power-up: the
  // PFD lives on the battery/master bus and lights up when the master switch is
  // on; the MFD is downstream on the avionics bus and also needs the avionics
  // master. With no power the GDU screen is simply black, and switching it on
  // runs the Garmin power-up self-test. Both default true so synthetic /
  // network feeds that don't model the switches (mock, standalone) keep the
  // glass lit.
  bool masterPowerOn = true;
  bool avionicsPowerOn = true;

  // Manual display-backup (reversionary) mode from the audio panel's red
  // DISPLAY BACKUP button. When true the PFD adds the EIS strip and the MFD
  // presents PFD instruments (automatic reversionary from a dark MFD also sets
  // this false and uses avionicsPowerOn instead).
  bool displayBackupActive = false;

  // Crew Alerting System (CAS) conditions, sourced from X-Plane's annunciators.
  // Each is true while its annunciator is lit; SoftkeyController turns the set
  // into the warning/caution text shown in the PFD Alerts window.
  bool casLowVacuum = false;
  bool casLowVoltage = false;
  bool casFuelLow = false;
  bool casOilPressureLow = false;
  bool casOilTempHigh = false;
  bool casFuelPressureLow = false;
  bool casPitotHeatOff = false;
  bool casIcing = false;
  bool casGearUnsafe = false;
  bool casStallWarning = false;

  // Current track over the ground (true/mag deg), shown as the magenta diamond
  // on the HSI compass card. Turn rate (deg/sec, + = right) drives the magenta
  // turn-rate trend vector above the rose; the G1000 caps the prediction at a
  // standard-rate (3 deg/sec) turn and shows an arrowhead beyond 4 deg/sec.
  float trackDeg = 0.0f;
  float turnRateDegPerSec = 0.0f;

  // NAV/COM radio frequencies (MHz). Active is the in-use frequency; standby is
  // the preselected one. Defaults are representative values shown until a live
  // radio source fills them in.
  float nav1ActiveMhz = 113.00f;
  float nav1StandbyMhz = 110.30f;
  float nav2ActiveMhz = 116.80f;
  float nav2StandbyMhz = 109.50f;
  float com1ActiveMhz = 118.000f;
  float com1StandbyMhz = 121.700f;
  float com2ActiveMhz = 119.250f;
  float com2StandbyMhz = 124.850f;

  // Which COM radio is currently transmitting (active frequency shown green).
  bool com1Transmitting = true;
  bool com2Transmitting = false;

  // Per-radio audio volume (0..1), set by the COM VOL/SQ and NAV VOL/ID knobs.
  // The PFD NavCom box shows the level as a percentage in place of the standby
  // frequency for two seconds after a change (Pilot's Guide Fig. 4-3 / 4-8).
  float com1Volume = 1.0f;
  float com2Volume = 1.0f;
  float nav1Volume = 1.0f;
  float nav2Volume = 1.0f;

  // Whether each NAV receiver's audio (its Morse identifier) is selected,
  // toggled by the NAV VOL/ID knob press. A white "ID" annunciates beside the
  // active NAV frequency while on (Pilot's Guide Fig. 4-8).
  bool nav1IdentAudio = false;
  bool nav2IdentAudio = false;

  // Decoded Morse identifier of the active NAV station (up to 3 chars, e.g.
  // "PSP"), shown to the right of the active NAV frequency. Empty when no
  // station is being received.
  std::string nav1Ident;
  std::string nav2Ident;

  // Decoded COM station identifier for each active frequency (e.g. "KTRM
  // UNICOM"), shown in a green bar beneath the active COM row when the tuned
  // frequency matches a published airport comm frequency in the nav database.
  std::string com1Ident;
  std::string com2Ident;

  // Active CDI navigation source, annunciated on the HSI.
  CdiSource cdiSource = CdiSource::Gps;

  // GPS flight phase annunciated inside the HSI rose (ENR/TERM/DPRT/APR/OCN).
  std::string gpsFlightPhase = "ENR";
  // Automatic waypoint sequencing suspended at the MAPt (SUSP annunciation).
  bool gpsSequencingSuspended = false;
  // Assumed fly-by bank for turn lead / leg sequencing (see AircraftProfile).
  float turnLeadBankDeg = 15.0f;
  // Missed approach segment is active (PROC Activate Missed or equivalent).
  bool missedApproachActive = false;

  // Transponder status box (bottom-right of the PFD): squawk code, mode label
  // (STBY/ON/ALT/GND), and a brief reply ("R") indication.
  int transponderCode = 1200;
  std::string transponderMode = "ALT";
  bool transponderReply = false;

  // Pilot-selected references (cyan bugs/readouts on the tapes and HSI) and the
  // current altimeter barometric setting.
  float selectedAltitudeFt = 6000.0f;
  float selectedHeadingDeg = 45.0f;
  // Autopilot FLC target airspeed (kt). Shown beside FLC on the FMA and as a
  // cyan bug on the airspeed tape while speed-by-pitch is the active vertical
  // mode (G1000 NXi Pilot's Guide, AFCS / Airspeed Indicator).
  bool selectedAirspeedValid = false;
  float selectedAirspeedKts = 0.0f;
  float baroSettingInHg = 29.92f;

  // Baro Transition Alert: while set, the BARO setting box flashes to prompt
  // the pilot to change to/from standard pressure (G1000 NXi Pilot's Guide,
  // Flight Instruments). Cleared once the pilot adjusts the setting.
  bool baroTransitionAlert = false;

  // HSI lateral guidance for the active CDI source: the selected course and the
  // lateral deviation (in dots; + = the course line lies to the right) plus the
  // TO/FROM sense. navSignalValid gates the deviation bar and TO/FROM flag.
  float courseDeg = 45.0f;
  float cdiDeviationDots = 0.0f;
  bool cdiToFlag = true;
  bool navSignalValid = true;

  // Signed GPS cross-track distance in NM (+ = left of course), unclamped unlike
  // cdiDeviationDots. Shown as the numeric XTK readout below the HSI aircraft
  // symbol when GPS is the active source and the CDI is off-scale (>2 dots).
  float gpsCrossTrackNm = 0.0f;

  // Bearing pointers on the HSI. BRG1 is a single-line needle, BRG2 a
  // double-line needle, each with an info window (pointer icon, source, station
  // /waypoint identifier, and GPS-derived slant-range) displayed below the HSI.
  bool bearing1Valid = false;
  float bearing1Deg = 0.0f;
  float bearing1DistanceNm = 0.0f;
  std::string bearing1Source = "GPS";
  std::string bearing1Ident;
  bool bearing2Valid = false;
  float bearing2Deg = 0.0f;
  float bearing2DistanceNm = 0.0f;
  std::string bearing2Source = "VOR1";
  std::string bearing2Ident;

  // Vertical deviation indicator (left of the altimeter). vdiDeviationDots is
  // positive when the aircraft is ABOVE the path (the diamond/pointer rides
  // low); the scale is +/-2 dots. vdiValid gates the diamond -- when the kind
  // is Glideslope and the signal is invalid (LOC tuned, no GS), "NO GS" shows.
  VerticalDeviationKind vdiKind = VerticalDeviationKind::None;
  bool vdiValid = false;
  float vdiDeviationDots = 0.0f;

  // Required Vertical Speed (fpm) to reach the active VNV target; drawn as a
  // magenta chevron on the VSI scale when valid.
  bool requiredVsValid = false;
  float requiredVsFpm = 0.0f;

  // Autopilot Selected Vertical Speed (fpm), drawn as a cyan bug on the VSI
  // scale when the VS reference is active (G1000 NXi Pilot's Guide, VSI).
  bool selectedVsValid = false;
  float selectedVerticalSpeedFpm = 0.0f;

  // Marker beacon receiver annunciation (left of the altimeter).
  MarkerBeacon markerBeacon = MarkerBeacon::None;

  // Active VNAV profile, computed each frame from the flight plan's altitude
  // constraints, ownship position, and ground speed.
  VnvProfile vnv;

  // DME Information Window (PFD Opt > DME), shown above the BRG1 window when the
  // DME display option is on: tuned source/mode, frequency, and slant-range.
  std::string dmeMode = "NAV1";
  float dmeFreqMhz = 113.00f;
  float dmeDistanceNm = 0.0f;
  bool dmeValid = false;

  // Flight director single-cue command bars on the attitude indicator. The bars
  // displace by the steering error (commanded minus actual) so the pilot flies
  // the aircraft symbol into them.
  bool flightDirectorActive = true;
  float fdPitchDeg = 0.0f;
  float fdRollDeg = 0.0f;

  // Tape trend vectors: the value projected six seconds ahead at the current
  // rate (magenta bar on the airspeed and altitude tapes).
  float airspeedTrendKts = 0.0f;
  float altitudeTrendFt = 0.0f;

  // Airspeed-tape color-band limits (KIAS), sourced from X-Plane's acf_V*
  // datarefs when connected. Defaults match the Cessna 172S envelope.
  float airspeedEnvelopeVsoKt = kDefaultAirspeedVsoKt;
  float airspeedEnvelopeVfeKt = kDefaultAirspeedVfeKt;
  float airspeedEnvelopeVs1Kt = kDefaultAirspeedVs1Kt;
  float airspeedEnvelopeVnoKt = kDefaultAirspeedVnoKt;
  float airspeedEnvelopeVneKt = kDefaultAirspeedVneKt;

  // Wind for the HSI wind box: direction the wind is coming FROM (true deg).
  bool windValid = true;
  float windDirectionDeg = 0.0f;
  float windSpeedKts = 0.0f;

  // FMA (center NavCom panel): active leg, lateral/vertical modes.
  std::string fmaFromWpt;
  std::string fmaToWpt = "KRSW";
  // Index of fmaToWpt in the active flight plan (-1 when unknown / Direct-To).
  int fmaActiveLegIndex = -1;
  float fmaLegDistanceNm = 12.4f;
  float fmaLegBearingDeg = 315.0f;
  // Active leg is a holding pattern: the Navigation Status Box shows a racetrack
  // symbol + fix instead of FROM -> TO (G1000 NXi Pilot's Guide Fig 5-3 symbols).
  bool fmaLegIsHold = false;
  bool fmaLegHoldRightTurn = true;
  // Turn-anticipation annunciation for the Navigation Status Box (replaces the
  // active-leg field when non-empty; G1000 NXi Pilot's Guide Section 5.1).
  std::string navStatusAnnunciation;
  bool navStatusAnnunciationFlash = false;
  std::string fmaLateralActive = "HDG";
  std::string fmaLateralArmed = "GPS";
  std::string fmaVerticalActive = "VS";
  std::string fmaVerticalArmed = "ALTS";
  std::string fmaVerticalApproachArmed;
  // Armed (white) VPTH before capture; flashes when re-ack is required near TOD.
  bool fmaVerticalPathArmed = false;
  bool fmaVerticalPathArmedFlash = false;
  int fmaVerticalValue = 500;
  std::string fmaVerticalUnits = "FPM";
  // Temporary RNAV glidepath/AP coupling debug (MFD AUX GPS Status).
  std::string gpCouplingDebug;
  bool apEngaged = true;
  bool ydEngaged = true;

  // Bottom info panel (above the softkey bar).
  float tasKts = 112.0f;
  float groundSpeedKts = 108.0f;
  float oatCelsius = 12.5f;
  int timerSeconds = 0;
  int utcHour = 18;
  int utcMinute = 42;
  int utcSecond = 15;
  bool clockIsUtc = true;
  // UTC day of year (1..366), for the Trip Planning sunrise/sunset
  // computation; 0 = unknown (those rows dash).
  int utcDayOfYear = 0;

  // Engine Indication System (EIS) strip on the left edge of the MFD, modeled
  // on the single-engine Cessna Nav III fit (G1000 Pilot's Guide for Cessna
  // Nav III, Section 3): tachometer, fuel flow, oil pressure/temperature, EGT,
  // standby vacuum, per-tank fuel quantity, engine hours, and the
  // voltmeter/ammeter rows.
  float engineRpm = 2400.0f;
  float fuelFlowGph = 9.8f;
  float oilPressurePsi = 62.0f;
  float oilTempDegF = 180.0f;
  float egtDegF = 1450.0f;
  float vacuumInHg = 4.9f;
  float fuelQtyLeftGal = 22.0f;
  float fuelQtyRightGal = 23.0f;
  float engineHours = 0.0f;
  float busVoltsMain = 27.9f;
  float busVoltsEssential = 27.8f;
  float battAmpsMain = 2.0f;
  float battAmpsStandby = 0.0f;

  // Generic EIS channel bag populated from the active layout's BIND lines.
  // Gauges reference channels by id; syncEisLegacyFields() mirrors well-known
  // channels into the scalar fields above for CAS and trip-planning consumers.
  std::unordered_map<std::string, float> eisChannels;
};

// Replaces any unset / non-positive airspeed-envelope limits on `d` with the
// Cessna 172S defaults (defined above struct FlightData so the helper can see
// the completed type).
inline void applyAirspeedEnvelopeDefaults(FlightData& d) {
  d.airspeedEnvelopeVsoKt =
      sanitizeAirspeedEnvelopeKt(d.airspeedEnvelopeVsoKt, kDefaultAirspeedVsoKt);
  d.airspeedEnvelopeVfeKt =
      sanitizeAirspeedEnvelopeKt(d.airspeedEnvelopeVfeKt, kDefaultAirspeedVfeKt);
  d.airspeedEnvelopeVs1Kt =
      sanitizeAirspeedEnvelopeKt(d.airspeedEnvelopeVs1Kt, kDefaultAirspeedVs1Kt);
  d.airspeedEnvelopeVnoKt =
      sanitizeAirspeedEnvelopeKt(d.airspeedEnvelopeVnoKt, kDefaultAirspeedVnoKt);
  d.airspeedEnvelopeVneKt =
      sanitizeAirspeedEnvelopeKt(d.airspeedEnvelopeVneKt, kDefaultAirspeedVneKt);
}

}  // namespace avionics
