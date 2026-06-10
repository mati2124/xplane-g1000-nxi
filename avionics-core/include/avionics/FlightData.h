#pragma once

#include <string>

namespace avionics {

// Active navigation source annunciated on the HSI / CDI.
enum class CdiSource { Gps, Nav1, Nav2 };

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

  // HSI lateral guidance for the active CDI source: the selected course and the
  // lateral deviation (in dots; + = the course line lies to the right) plus the
  // TO/FROM sense. navSignalValid gates the deviation bar and TO/FROM flag.
  float courseDeg = 45.0f;
  float cdiDeviationDots = 0.0f;
  bool cdiToFlag = true;
  bool navSignalValid = true;

  // Bearing pointers on the HSI. BRG1 is a single-line needle, BRG2 a
  // double-line needle, each with an info window (source + slant-range).
  bool bearing1Valid = false;
  float bearing1Deg = 0.0f;
  float bearing1DistanceNm = 0.0f;
  std::string bearing1Source = "GPS";
  bool bearing2Valid = false;
  float bearing2Deg = 0.0f;
  float bearing2DistanceNm = 0.0f;
  std::string bearing2Source = "VOR1";

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
  std::string fmaToWpt = "KPAO";
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
};

}  // namespace avionics
