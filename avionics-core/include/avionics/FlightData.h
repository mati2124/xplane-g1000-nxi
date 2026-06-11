#pragma once

#include <string>
#include <unordered_map>

namespace avionics {

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

  // Whole-feed health, distinct from the per-sensor flags above. The engine
  // clears this (alongside every sensor flag) when the data link stops
  // delivering fresh data, so the non-sensor readouts in the top NAV/COM bar
  // and bottom info panel (radios, FMA, transponder, OAT, clock) blank out /
  // show amber dashes instead of stale values, matching the red-X'd gauges.
  bool dataLinkValid = true;

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

  // Active CDI navigation source, annunciated on the HSI.
  CdiSource cdiSource = CdiSource::Gps;

  // GPS flight phase annunciated inside the HSI rose (ENR/TERM/DPRT/APR/OCN).
  std::string gpsFlightPhase = "ENR";

  // Transponder status box (bottom-right of the PFD): squawk code, mode label
  // (STBY/ON/ALT/GND), and a brief reply ("R") indication.
  int transponderCode = 1200;
  std::string transponderMode = "ALT";
  bool transponderReply = false;

  // Pilot-selected references (cyan bugs/readouts on the tapes and HSI) and the
  // current altimeter barometric setting.
  float selectedAltitudeFt = 6000.0f;
  float selectedHeadingDeg = 45.0f;
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

  // Wind for the HSI wind box: direction the wind is coming FROM (true deg).
  bool windValid = true;
  float windDirectionDeg = 0.0f;
  float windSpeedKts = 0.0f;

  // FMA (center of the top NAV/COM bar): active leg, lateral/vertical modes.
  std::string fmaFromWpt;
  std::string fmaToWpt = "KFMY";
  float fmaLegDistanceNm = 12.4f;
  float fmaLegBearingDeg = 315.0f;
  std::string fmaLateralActive = "GPS";
  std::string fmaLateralArmed;
  std::string fmaVerticalActive = "ALT";
  std::string fmaVerticalArmed = "ALTS";
  std::string fmaVerticalApproachArmed;
  int fmaVerticalValue = 6000;
  std::string fmaVerticalUnits = "FT";
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

}  // namespace avionics
