#pragma once

namespace avionics {
namespace datarefs {

// Canonical X-Plane dataref paths, kept in one place so neither the plugin
// shell nor any future tooling has to hardcode these strings inline.
// See: https://developer.x-plane.com/datarefs/
inline constexpr const char* kAirspeedKts =
    "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";
inline constexpr const char* kAltitudeFt =
    "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
inline constexpr const char* kHeadingDegMag =
    "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot";
inline constexpr const char* kPitchDeg =
    "sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot";
inline constexpr const char* kRollDeg =
    "sim/cockpit2/gauges/indicators/roll_electric_deg_pilot";
inline constexpr const char* kVerticalSpeedFpm =
    "sim/cockpit2/gauges/indicators/vvi_fpm_pilot";
inline constexpr const char* kSlipDeg =
    "sim/cockpit2/gauges/indicators/slip_deg";

// Track / turn rate (deg, deg/sec).
inline constexpr const char* kGroundTrackDegMag =
    "sim/cockpit2/gauges/indicators/ground_track_mag_pilot";
inline constexpr const char* kTurnRateDegPerSec =
    "sim/cockpit2/gauges/indicators/turn_rate_heading_deg_pilot";

// Wind (direction the wind is FROM, deg mag; speed in knots).
inline constexpr const char* kWindDirectionDegMag =
    "sim/cockpit2/gauges/indicators/wind_heading_deg_mag";
inline constexpr const char* kWindSpeedKts =
    "sim/cockpit2/gauges/indicators/wind_speed_kts";

// Speeds. groundspeed and true_airspeed are reported in m/s and must be scaled
// to knots before they reach FlightData.
inline constexpr const char* kGroundSpeedMs =
    "sim/flightmodel/position/groundspeed";
inline constexpr const char* kTrueAirspeedMs =
    "sim/flightmodel/position/true_airspeed";

// Outside air temperature (deg C).
inline constexpr const char* kOatDegC =
    "sim/cockpit2/temperature/outside_air_temp_degc";

// Zulu (UTC) wall-clock time as a continuous count of seconds since midnight
// (fractional). Preferred over the split hours/minutes/seconds integer datarefs
// because a single monotonic value can be ticked locally between packets, so the
// displayed clock advances smoothly and never skips a second on UDP jitter/loss.
inline constexpr const char* kZuluTimeSec = "sim/time/zulu_time_sec";

// Pilot-selected references and barometric setting.
inline constexpr const char* kSelectedAltitudeFt =
    "sim/cockpit/autopilot/altitude";
inline constexpr const char* kSelectedHeadingDegMag =
    "sim/cockpit/autopilot/heading_mag";
inline constexpr const char* kBaroSettingInHg =
    "sim/cockpit/misc/barometer_setting";

// NAV/COM radio frequencies. These are integers in units of 10 Hz where the
// value is the frequency in MHz x 100 (e.g. 11030 == 110.30 MHz), so multiply
// by 0.01 to get MHz.
inline constexpr const char* kNav1FrequencyHz =
    "sim/cockpit2/radios/actuators/nav1_frequency_hz";
inline constexpr const char* kNav1StandbyFrequencyHz =
    "sim/cockpit2/radios/actuators/nav1_standby_frequency_hz";
inline constexpr const char* kNav2FrequencyHz =
    "sim/cockpit2/radios/actuators/nav2_frequency_hz";
inline constexpr const char* kNav2StandbyFrequencyHz =
    "sim/cockpit2/radios/actuators/nav2_standby_frequency_hz";
inline constexpr const char* kCom1FrequencyHz =
    "sim/cockpit2/radios/actuators/com1_frequency_hz";
inline constexpr const char* kCom1StandbyFrequencyHz =
    "sim/cockpit2/radios/actuators/com1_standby_frequency_hz";
inline constexpr const char* kCom2FrequencyHz =
    "sim/cockpit2/radios/actuators/com2_frequency_hz";
inline constexpr const char* kCom2StandbyFrequencyHz =
    "sim/cockpit2/radios/actuators/com2_standby_frequency_hz";

// HSI / CDI for the pilot's selected nav source. The hsi_* indicators always
// reflect whatever source HSI_source_select_pilot has selected.
inline constexpr const char* kHsiSourceSelect =
    "sim/cockpit2/radios/actuators/HSI_source_select_pilot";  // 0=Nav1 1=Nav2 2=GPS
inline constexpr const char* kHsiObsCourseDegMag =
    "sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot";
inline constexpr const char* kHsiDeviationDots =
    "sim/cockpit2/radios/indicators/hsi_hdef_dots_pilot";
inline constexpr const char* kHsiFromTo =
    "sim/cockpit2/radios/indicators/hsi_flag_from_to_pilot";  // 0=flag 1=to 2=from

// GPS/FMS active-leg indicators for the navigation status box: slant-range
// distance (nm) and magnetic bearing to the selected GPS destination waypoint.
// The destination's identifier string lives in gps_nav_id (a byte[] string),
// which the float-only RREF protocol cannot carry, so it is not bound here.
inline constexpr const char* kGpsDistanceNm =
    "sim/cockpit2/radios/indicators/gps_dme_distance_nm";
inline constexpr const char* kGpsBearingDegMag =
    "sim/cockpit2/radios/indicators/gps_bearing_deg_mag";
// GPS lateral CDI sensitivity in nautical miles per dot of deflection. X-Plane
// has no direct ENR/TERM/APR enum; the G1000 derives the annunciated flight
// phase from the active CDI scale (the GPS auto-slews it by phase of flight).
// The G1000 uses a 2-dot full scale, so full-scale NM = 2 x this value:
// OCN = 4 NM, ENR = 2 NM, TERM = 1 NM, APR = 0.3 NM.
inline constexpr const char* kGpsHdefNmPerDot =
    "sim/cockpit/radios/gps_hdef_nm_per_dot";
// GPS destination waypoint identifier. This is a byte[] string dataref, so it
// is readable only in-process (the X-Plane plugin shell via XPLMGetDatab); the
// float-only RREF protocol used by the standalone shell cannot carry it.
inline constexpr const char* kGpsNavId =
    "sim/cockpit2/radios/indicators/gps_nav_id";

// Transponder code (0000-7777) and mode. The X-Plane 12 mode enum is
// off=0, stdby=1, on=2, alt=3, test=4, with 6/7 being the TCAS traffic modes.
inline constexpr const char* kTransponderCode =
    "sim/cockpit/radios/transponder_code";
inline constexpr const char* kTransponderMode =
    "sim/cockpit/radios/transponder_mode";

// Autopilot / flight director. flight_director_mode: 0=off, 1=FD on,
// 2=FD on with autopilot servos.
inline constexpr const char* kFlightDirectorMode =
    "sim/cockpit2/autopilot/flight_director_mode";
inline constexpr const char* kYawDamperOn =
    "sim/cockpit/switches/yaw_damper_on";

// Autopilot mode statuses driving the FMA (top-bar flight mode annunciator).
// Each is an int enum: 0 = off, 1 = armed, 2 = active/captured. The lateral
// group (roll/heading/nav/backcourse) and the vertical group (pitch/altitude/
// vertical-speed/speed/vnav/glideslope) each annunciate one captured mode and
// one armed mode.
inline constexpr const char* kApRollStatus =
    "sim/cockpit2/autopilot/roll_status";
inline constexpr const char* kApHeadingStatus =
    "sim/cockpit2/autopilot/heading_status";
inline constexpr const char* kApNavStatus =
    "sim/cockpit2/autopilot/nav_status";
inline constexpr const char* kApBackcourseStatus =
    "sim/cockpit2/autopilot/backcourse_status";
inline constexpr const char* kApPitchStatus =
    "sim/cockpit2/autopilot/pitch_status";
inline constexpr const char* kApAltitudeHoldStatus =
    "sim/cockpit2/autopilot/altitude_hold_status";
inline constexpr const char* kApVerticalSpeedStatus =
    "sim/cockpit2/autopilot/vvi_status";
inline constexpr const char* kApSpeedStatus =
    "sim/cockpit2/autopilot/speed_status";
inline constexpr const char* kApVnavStatus =
    "sim/cockpit2/autopilot/vnav_status";
inline constexpr const char* kApGlideslopeStatus =
    "sim/cockpit2/autopilot/glideslope_status";
// Altitude preselect armed (drives the ALTS armed annunciation): 0 or 1.
inline constexpr const char* kApAltitudeHoldArmed =
    "sim/cockpit2/autopilot/altitude_hold_armed";

// Per-instrument failure state (pilot side). These are failure_enum ints where
// 0 = working and 6 = inoperative (failed now); intermediate values are armed
// failure conditions that have not yet tripped. Used to drive the reversionary
// red-X annunciations: AHRS feeds attitude + heading, ADC feeds the air-data
// instruments.
inline constexpr const char* kFailAttitude =
    "sim/operation/failures/rel_ss_ahz";  // artificial horizon (AHRS)
inline constexpr const char* kFailHeading =
    "sim/operation/failures/rel_ss_dgy";  // directional gyro (heading)
inline constexpr const char* kFailAirspeed =
    "sim/operation/failures/rel_ss_asi";
inline constexpr const char* kFailAltimeter =
    "sim/operation/failures/rel_ss_alt";
inline constexpr const char* kFailVerticalSpeed =
    "sim/operation/failures/rel_ss_vvi";

// Crew Alerting System (CAS) sources. X-Plane lights its annunciators under
// sim/cockpit2/annunciators/ as boolean ints (0 = off, 1 = lit). The per-engine
// annunciators are int[8] arrays; we read element [0] (engine 1), which suits
// the single-engine GA aircraft this PFD models. RREF accepts the "[index]"
// suffix to subscribe to a single array element.
inline constexpr const char* kAnnunLowVacuum =
    "sim/cockpit2/annunciators/low_vacuum";
inline constexpr const char* kAnnunLowVoltage =
    "sim/cockpit2/annunciators/low_voltage";
inline constexpr const char* kAnnunFuelLow =
    "sim/cockpit2/annunciators/fuel_quantity";
inline constexpr const char* kAnnunOilPressureLow =
    "sim/cockpit2/annunciators/oil_pressure[0]";
inline constexpr const char* kAnnunOilTempHigh =
    "sim/cockpit2/annunciators/oil_temperature[0]";
inline constexpr const char* kAnnunFuelPressureLow =
    "sim/cockpit2/annunciators/fuel_pressure[0]";
inline constexpr const char* kAnnunPitotHeat =
    "sim/cockpit2/annunciators/pitot_heat";
inline constexpr const char* kAnnunIce = "sim/cockpit2/annunciators/ice";
inline constexpr const char* kAnnunGearUnsafe =
    "sim/cockpit2/annunciators/gear_unsafe";
inline constexpr const char* kAnnunStallWarning =
    "sim/cockpit2/annunciators/stall_warning";

}  // namespace datarefs
}  // namespace avionics
