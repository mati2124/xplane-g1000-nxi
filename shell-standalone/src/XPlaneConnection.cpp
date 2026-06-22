#include "XPlaneConnection.h"

#include "avionics/EisLegacy.h"

#include <cmath>
#include <cstring>

#include "avionics/Datarefs.h"
#include "avionics/GlidepathGuidance.h"
#include "avionics/MapRange.h"
#include "avionics/NavMath.h"
#include "avionics/Radio.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using SocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
#endif

namespace avionics {

namespace {

constexpr int kSubscribeFrequencyHz = 20;
constexpr double kStaleTimeoutSeconds = 2.0;
constexpr double kResubscribeIntervalSeconds = 3.0;
constexpr float kMetersPerSecondToKnots = 1.943844f;
// X-Plane radio frequency datarefs are MHz x 100 (11030 == 110.30 MHz).
constexpr float kRadioHzToMhz = 0.01f;
// X-Plane failure_enum value meaning the instrument is currently inoperative.
constexpr int kFailureInop = 6;

// Autopilot mode-status enum values shared by all *_status datarefs.
constexpr int kApModeArmed = 1;
constexpr int kApModeActive = 2;

// X-Plane transponder_mode enum. Note this is NOT the legacy off/stdby/on/test
// set: X-Plane 12 inserted ALT (Mode C) at 3 and pushed Test to 4, and the
// high values 6/7 are the TCAS traffic modes (which still report altitude).
enum XpdrMode {
  kXpdrOff = 0,
  kXpdrStandby = 1,
  kXpdrOn = 2,
  kXpdrAlt = 3,
  kXpdrTest = 4,
  kXpdrTaOnly = 6,
  kXpdrTaRa = 7,
};

// Time constant of the per-field smoothing filter. X-Plane streams at ~20 Hz
// but we render at ~60 fps; easing each displayed value toward the latest
// received value over this many seconds removes the visible stepping without
// adding much lag.
constexpr double kSmoothingTimeConstantSeconds = 0.06;
// The airspeed trend vector is a 6-second projection of IAS rate. It is
// derived from packet-to-packet sim values (not the per-frame display easing)
// and low-pass filtered with a longer time constant so takeoff acceleration
// reads smoothly instead of jittering on UDP / pitot noise.
constexpr double kAirspeedTrendTimeConstantSeconds = 0.75;

// Zulu clock (sim/time/zulu_time_sec) handling. The displayed clock free-runs at
// real time between packets and is eased toward the sim's value with this time
// constant, so normal UDP jitter/loss never makes the seconds skip. A delta
// larger than the resync threshold (sim time set, a pause/resume gap, or the
// midnight rollover) is snapped instead of eased.
constexpr double kZuluClockTimeConstantSeconds = 0.5;
constexpr double kZuluResyncThresholdSeconds = 3.0;
constexpr double kSecondsPerDay = 86400.0;
constexpr double kDirectToArrivalNm = 0.45;

int legIndexInPlan(const std::vector<MapLeg>& plan, const std::string& id) {
  if (id.empty()) return -1;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (plan[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

// RREF request wire format (little endian):
//   "RREF\0" + int32 frequency + int32 index + char[400] dataref name.
constexpr int kDatarefNameSize = 400;
constexpr int kRrefRequestSize = 5 + 4 + 4 + kDatarefNameSize;  // 413 bytes
// DREF (write) wire format is distinct from RREF: X-Plane expects a 500-byte
// dataref-path field, so the packet is "DREF\0" + float value + char[500] =
// 509 bytes. Sending the shorter RREF path size (404-byte payload) makes
// X-Plane reject the write ("received 404 bytes but needed 504 bytes").
constexpr int kDrefNameSize = 500;
constexpr int kDrefMessageSize = 5 + 4 + kDrefNameSize;  // 509 bytes
// RREF reply payload after the 5-byte "RREF\0" header is a sequence of
// (int32 index, float32 value) records.
constexpr int kRrefHeaderSize = 5;
constexpr int kRrefRecordSize = 8;
constexpr int kReceiveBufferSize = 8192;

// How a field is eased toward its newly received value each frame.
enum class Smooth {
  Snap,    // discrete / pilot-set value: jump straight to it (no filtering)
  Linear,  // continuous scalar: low-pass filter toward the target
  Angle,   // continuous heading-like value: filter along the shortest arc
};

// Binding from a subscription index -> X-Plane dataref -> FlightData field.
// All bound fields are floats, so a member pointer keeps this data-driven (used
// for both decoding and smoothing). `scale` and `offset` convert the dataref's
// native units to the field's units (value * scale + offset, e.g. deg C ->
// deg F); the array position is the index sent to (and echoed back by)
// X-Plane.
struct DatarefBinding {
  const char* path;
  float scale;
  float FlightData::* member;
  Smooth smooth;
  float offset = 0.0f;
};

const DatarefBinding kBindings[] = {
    {datarefs::kAirspeedKts, 1.0f, &FlightData::airspeedKts, Smooth::Linear},
    {datarefs::kAltitudeFt, 1.0f, &FlightData::altitudeFt, Smooth::Linear},
    {datarefs::kHeadingDegMag, 1.0f, &FlightData::headingDeg, Smooth::Angle},
    {datarefs::kPitchDeg, 1.0f, &FlightData::pitchDeg, Smooth::Linear},
    {datarefs::kRollDeg, 1.0f, &FlightData::rollDeg, Smooth::Linear},
    {datarefs::kVerticalSpeedFpm, 1.0f, &FlightData::verticalSpeedFpm,
     Smooth::Linear},
    {datarefs::kSlipDeg, 1.0f, &FlightData::slipSkidDeg, Smooth::Linear},
    {datarefs::kGroundTrackDegMag, 1.0f, &FlightData::trackDeg, Smooth::Angle},
    {datarefs::kTurnRateDegPerSec, 1.0f, &FlightData::turnRateDegPerSec,
     Smooth::Linear},
    {datarefs::kWindDirectionDegMag, 1.0f, &FlightData::windDirectionDeg,
     Smooth::Angle},
    {datarefs::kWindSpeedKts, 1.0f, &FlightData::windSpeedKts, Smooth::Linear},
    {datarefs::kGroundSpeedMs, kMetersPerSecondToKnots,
     &FlightData::groundSpeedKts, Smooth::Linear},
    {datarefs::kTrueAirspeedMs, kMetersPerSecondToKnots, &FlightData::tasKts,
     Smooth::Linear},
    {datarefs::kOatDegC, 1.0f, &FlightData::oatCelsius, Smooth::Linear},
    {datarefs::kSelectedAltitudeFt, 1.0f, &FlightData::selectedAltitudeFt,
     Smooth::Snap},
    {datarefs::kSelectedHeadingDegMag, 1.0f, &FlightData::selectedHeadingDeg,
     Smooth::Snap},
    {datarefs::kSelectedAirspeedKts, 1.0f, &FlightData::selectedAirspeedKts,
     Smooth::Snap},
    {datarefs::kSelectedVerticalSpeedFpm, 1.0f,
     &FlightData::selectedVerticalSpeedFpm, Smooth::Snap},
    {datarefs::kBaroSettingInHg, 1.0f, &FlightData::baroSettingInHg,
     Smooth::Snap},

    // GPS active-leg distance + bearing for the nav status box (DIS / BRG).
    {datarefs::kGpsDistanceNm, 1.0f, &FlightData::fmaLegDistanceNm,
     Smooth::Linear},
    {datarefs::kGpsBearingDegMag, 1.0f, &FlightData::fmaLegBearingDeg,
     Smooth::Angle},

    // HSI course + lateral deviation for the selected nav source.
    {datarefs::kHsiObsCourseDegMag, 1.0f, &FlightData::courseDeg, Smooth::Snap},
    {datarefs::kHsiDeviationDots, 1.0f, &FlightData::cdiDeviationDots,
     Smooth::Linear},

    // NAV/COM frequencies (discrete tuner steps, so they snap).
    {datarefs::kNav1FrequencyHz, kRadioHzToMhz, &FlightData::nav1ActiveMhz,
     Smooth::Snap},
    {datarefs::kNav1StandbyFrequencyHz, kRadioHzToMhz,
     &FlightData::nav1StandbyMhz, Smooth::Snap},
    {datarefs::kNav2FrequencyHz, kRadioHzToMhz, &FlightData::nav2ActiveMhz,
     Smooth::Snap},
    {datarefs::kNav2StandbyFrequencyHz, kRadioHzToMhz,
     &FlightData::nav2StandbyMhz, Smooth::Snap},
    {datarefs::kCom1FrequencyHz, kRadioHzToMhz, &FlightData::com1ActiveMhz,
     Smooth::Snap},
    {datarefs::kCom1StandbyFrequencyHz, kRadioHzToMhz,
     &FlightData::com1StandbyMhz, Smooth::Snap},
    {datarefs::kCom2FrequencyHz, kRadioHzToMhz, &FlightData::com2ActiveMhz,
     Smooth::Snap},
    {datarefs::kCom2StandbyFrequencyHz, kRadioHzToMhz,
     &FlightData::com2StandbyMhz, Smooth::Snap},

    // Per-radio audio volume (0..1), so the NavCom box shows the live level
    // (and reflects the VOL/SQ / VOL/ID knob writes we send back).
    {datarefs::kCom1Volume, 1.0f, &FlightData::com1Volume, Smooth::Snap},
    {datarefs::kCom2Volume, 1.0f, &FlightData::com2Volume, Smooth::Snap},
    {datarefs::kNav1Volume, 1.0f, &FlightData::nav1Volume, Smooth::Snap},
    {datarefs::kNav2Volume, 1.0f, &FlightData::nav2Volume, Smooth::Snap},
};
constexpr int kBindingCount =
    static_cast<int>(sizeof(kBindings) / sizeof(kBindings[0]));

// Discrete (non-float) fields. RREF only carries floats, so these datarefs are
// rounded and interpreted into FlightData's enum/bool/int/string fields. Their
// subscription indices continue after the float bindings above.
enum DiscreteRef {
  kDiscCdiSource = 0,
  kDiscTransponderCode,
  kDiscTransponderMode,
  kDiscFlightDirectorMode,
  kDiscYawDamper,
  kDiscFromTo,
  kDiscFailAttitude,
  kDiscFailHeading,
  kDiscFailAirspeed,
  kDiscFailAltitude,
  kDiscFailVerticalSpeed,
  kDiscFailNav1,
  kDiscFailNav2,
  kDiscFailCom1,
  kDiscFailCom2,
  kDiscFailTransponder,
  kDiscCasLowVacuum,
  kDiscCasLowVoltage,
  kDiscCasFuelLow,
  kDiscCasOilPressureLow,
  kDiscCasOilTempHigh,
  kDiscCasFuelPressureLow,
  kDiscCasPitotHeat,
  kDiscCasIce,
  kDiscCasGearUnsafe,
  kDiscCasStallWarning,
  kDiscNav1IdentAudio,
  kDiscNav2IdentAudio,
  kDiscMasterPower,
  kDiscAvionicsPower,
  kDiscreteCount,
};

const char* const kDiscretePaths[kDiscreteCount] = {
    datarefs::kHsiSourceSelect,      datarefs::kTransponderCode,
    datarefs::kTransponderMode,      datarefs::kFlightDirectorMode,
    datarefs::kYawDamperOn,          datarefs::kHsiFromTo,
    datarefs::kFailAttitude,         datarefs::kFailHeading,
    datarefs::kFailAirspeed,         datarefs::kFailAltimeter,
    datarefs::kFailVerticalSpeed,    datarefs::kFailNav1,
    datarefs::kFailNav2,             datarefs::kFailCom1,
    datarefs::kFailCom2,             datarefs::kFailTransponder,
    datarefs::kAnnunLowVacuum,
    datarefs::kAnnunLowVoltage,      datarefs::kAnnunFuelLow,
    datarefs::kAnnunOilPressureLow,  datarefs::kAnnunOilTempHigh,
    datarefs::kAnnunFuelPressureLow, datarefs::kAnnunPitotHeat,
    datarefs::kAnnunIce,             datarefs::kAnnunGearUnsafe,
    datarefs::kAnnunStallWarning,    datarefs::kNav1IdentAudio,
    datarefs::kNav2IdentAudio,       datarefs::kBatteryMasterOn,
    datarefs::kAvionicsPowerOn,
};

// The continuous zulu-time subscription rides one index past the float and
// discrete bindings; it is decoded specially (local clock) rather than into a
// FlightData field directly.
constexpr int kZuluSubscriptionIndex = kBindingCount + kDiscreteCount;

// Autopilot mode statuses for the FMA. These are accumulated into a raw int
// array (rather than a FlightData field) and combined into the lateral/vertical
// mode strings each frame, since several datarefs collapse into one annunciator
// cell. Their subscription indices continue past the zulu subscription.
enum ApMode {
  kApRoll = 0,
  kApHeading,
  kApNav,
  kApBackcourse,
  kApPitch,
  kApAltitudeHold,
  kApVerticalSpeed,
  kApSpeed,
  kApVnav,
  kApGlideslope,
  kApAltitudeArmed,
  kApModeCount,
};

const char* const kApModePaths[kApModeCount] = {
    datarefs::kApRollStatus,          datarefs::kApHeadingStatus,
    datarefs::kApNavStatus,           datarefs::kApBackcourseStatus,
    datarefs::kApPitchStatus,         datarefs::kApAltitudeHoldStatus,
    datarefs::kApVerticalSpeedStatus, datarefs::kApSpeedStatus,
    datarefs::kApVnavStatus,          datarefs::kApGlideslopeStatus,
    datarefs::kApAltitudeHoldArmed,
};

constexpr int kApModeBaseIndex = kZuluSubscriptionIndex + 1;

// The GPS CDI-sensitivity subscription rides one index past the AP-mode block.
// It is decoded into the gpsFlightPhase string rather than a float field.
constexpr int kGpsSensitivityIndex = kApModeBaseIndex + kApModeCount;

// Ownship latitude/longitude subscriptions for the moving map. Like the zulu
// clock they decode into dedicated members (not a FlightData field) since the
// map consumes them separately.
constexpr int kLatitudeIndex = kGpsSensitivityIndex + 1;
constexpr int kLongitudeIndex = kLatitudeIndex + 1;

// TCAS traffic targets for the map overlay: per-element subscriptions into the
// target position arrays (element 0 is ownship, so targets start at 1).
constexpr int kTrafficTargetCount = 8;
constexpr int kTrafficFieldCount = 4;  // lat, lon, ele, vertical_speed
constexpr int kTrafficBaseIndex = kLongitudeIndex + 1;

enum NavInstrRef {
  kNav1Vdef = 0,
  kNav2Vdef,
  kNav1GsFlag,
  kNav2GsFlag,
  kOuterMarker,
  kMiddleMarker,
  kInnerMarker,
  kNav1Dme,
  kNav2Dme,
  kNavInstrFieldCount,
};
static_assert(kNavInstrFieldCount == 9,
              "nav instrumentation subscription count must match storage");
constexpr int kNavInstrBaseIndex =
    kTrafficBaseIndex + kTrafficTargetCount * kTrafficFieldCount;
const char* const kNavInstrPaths[kNavInstrFieldCount] = {
    datarefs::kNav1VdefDotsPilot,
    datarefs::kNav2VdefDotsPilot,
    datarefs::kNav1GsFlag,
    datarefs::kNav2GsFlag,
    datarefs::kOuterMarkerLit,
    datarefs::kMiddleMarkerLit,
    datarefs::kInnerMarkerLit,
    datarefs::kNav1DmeDistanceNm,
    datarefs::kNav2DmeDistanceNm,
};

// Sim date (day of year, 0-based) for the Trip Planning sunrise/sunset rows;
// rides one index past the nav-instrumentation block.
constexpr int kDateDaysIndex = kNavInstrBaseIndex + kNavInstrFieldCount;
constexpr int kEisSubscriptionBase = kDateDaysIndex + 1;

const char* const kTrafficFieldPaths[kTrafficFieldCount] = {
    datarefs::kTcasTargetLat,
    datarefs::kTcasTargetLon,
    datarefs::kTcasTargetEleMeters,
    datarefs::kTcasTargetVerticalSpeedFpm,
};

// How often the nearby-feature list is rebuilt from the nav database. Ownship
// position updates every frame; the (range-filtered) feature scan is throttled.
constexpr double kMapRebuildIntervalSeconds = 1.0;
// Display range for the inset map (the MFD overrides per view).
constexpr float kMapRangeNm = 10.0f;
// Query radius for the nearby-data scans: must cover the longest MFD range,
// not just the inset default.
constexpr float kMapQueryRangeNm = 160.0f;
// Deep enough for the per-type reserves in NavDataStore::nearby (airports +
// navaids) to fully populate a wide MFD MAP view before fixes fill the rest.
constexpr std::size_t kMaxMapFeatures = 500;
constexpr std::size_t kMaxMapAirspaces = 60;
constexpr std::size_t kMaxMapAirways = 500;
// Runway diagrams only draw at short ranges, so their query stays tight.
constexpr float kRunwayQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapRunways = 120;
// Taxiway pavement only draws very close in (<= 2.5 NM), so its query is
// tighter still and the polygon count is capped to keep the rebuild cheap.
constexpr float kTaxiwayQueryRangeNm = 10.0f;
constexpr std::size_t kMaxMapTaxiways = 600;
constexpr float kTaxiwayLabelQueryRangeNm = 10.0f;
constexpr std::size_t kMaxMapTaxiwayLabels = 400;
// Obstacles likewise only draw at low ranges (and the DOF is dense).
constexpr float kObstacleQueryRangeNm = 30.0f;
constexpr std::size_t kMaxMapObstacles = 300;

// Traffic display filtering and the simple TA threat heuristic (TIS-style:
// proximate traffic within 1 NM and 1200 ft is upgraded to an advisory).
constexpr float kTrafficMaxRangeNm = 40.0f;
constexpr float kTrafficTaRangeNm = 1.0f;
constexpr float kTrafficTaAltFt = 1200.0f;
constexpr float kMetersToFeet = 3.28084f;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kNmPerDegLat = 60.0;

constexpr std::size_t kMaxMapLandLines = 8000;
constexpr std::size_t kMaxMapCities = 600;

// Map the active GPS CDI sensitivity (NM per dot; the G1000 uses a 2-dot full
// scale) to the flight-phase annunciation shown in the HSI. Full-scale NM is
// 2 x the per-dot value. A non-positive value means there is no usable GPS
// scale (no active flight plan), which blanks the annunciation.
const char* gpsPhaseFromSensitivity(float nmPerDot) {
  if (nmPerDot <= 0.0f) return "";
  const float fullScaleNm = nmPerDot * 2.0f;
  if (fullScaleNm >= 3.0f) return "OCN";   // oceanic, 4 NM
  if (fullScaleNm >= 1.5f) return "ENR";   // enroute, 2 NM
  if (fullScaleNm >= 0.6f) return "TERM";  // terminal, 1 NM
  return "APR";                            // approach, 0.3 NM
}

void applyDiscrete(FlightData& d, int discrete, float value) {
  const int v = static_cast<int>(std::lround(value));
  switch (discrete) {
    case kDiscCdiSource:
      d.cdiSource = (v == 0) ? CdiSource::Nav1
                  : (v == 1) ? CdiSource::Nav2
                             : CdiSource::Gps;
      break;
    case kDiscTransponderCode:
      d.transponderCode = v;
      break;
    case kDiscTransponderMode:
      // The G1000 annunciates Mode C (altitude reporting) as ALT, and the TCAS
      // traffic modes still squawk altitude, so they fall through to ALT/TA-RA
      // rather than the bare "ON".
      switch (v) {
        case kXpdrOff:
          d.transponderMode = "OFF";
          break;
        case kXpdrStandby:
          d.transponderMode = "STBY";
          break;
        case kXpdrOn:
          d.transponderMode = "ON";
          break;
        case kXpdrTest:
          d.transponderMode = "TEST";
          break;
        case kXpdrTaOnly:
          d.transponderMode = "TA";
          break;
        case kXpdrTaRa:
          d.transponderMode = "TA/RA";
          break;
        case kXpdrAlt:
        default:
          d.transponderMode = "ALT";
          break;
      }
      break;
    case kDiscFlightDirectorMode:
      d.flightDirectorActive = v >= 1;
      d.apEngaged = v == 2;  // servos engaged
      break;
    case kDiscYawDamper:
      d.ydEngaged = v != 0;
      break;
    case kDiscFromTo:
      // 0 = flag (no usable signal), 1 = TO, 2 = FROM.
      d.navSignalValid = v != 0;
      d.cdiToFlag = v == 1;
      break;
    case kDiscFailAttitude:
      d.attitudeValid = v != kFailureInop;
      break;
    case kDiscFailHeading:
      d.headingValid = v != kFailureInop;
      break;
    case kDiscFailAirspeed:
      d.airspeedValid = v != kFailureInop;
      break;
    case kDiscFailAltitude:
      d.altitudeValid = v != kFailureInop;
      break;
    case kDiscFailVerticalSpeed:
      d.verticalSpeedValid = v != kFailureInop;
      break;
    case kDiscFailNav1:
      d.nav1Valid = v != kFailureInop;
      break;
    case kDiscFailNav2:
      d.nav2Valid = v != kFailureInop;
      break;
    case kDiscFailCom1:
      d.com1Valid = v != kFailureInop;
      break;
    case kDiscFailCom2:
      d.com2Valid = v != kFailureInop;
      break;
    case kDiscFailTransponder:
      d.transponderValid = v != kFailureInop;
      break;
    case kDiscCasLowVacuum:
      d.casLowVacuum = v != 0;
      break;
    case kDiscCasLowVoltage:
      d.casLowVoltage = v != 0;
      break;
    case kDiscCasFuelLow:
      d.casFuelLow = v != 0;
      break;
    case kDiscCasOilPressureLow:
      d.casOilPressureLow = v != 0;
      break;
    case kDiscCasOilTempHigh:
      d.casOilTempHigh = v != 0;
      break;
    case kDiscCasFuelPressureLow:
      d.casFuelPressureLow = v != 0;
      break;
    case kDiscCasPitotHeat:
      d.casPitotHeatOff = v != 0;
      break;
    case kDiscCasIce:
      d.casIcing = v != 0;
      break;
    case kDiscCasGearUnsafe:
      d.casGearUnsafe = v != 0;
      break;
    case kDiscCasStallWarning:
      d.casStallWarning = v != 0;
      break;
    case kDiscNav1IdentAudio:
      d.nav1IdentAudio = v != 0;
      break;
    case kDiscNav2IdentAudio:
      d.nav2IdentAudio = v != 0;
      break;
    case kDiscMasterPower:
      // GDU power: master gates the PFD, avionics master gates the MFD. Mirrors
      // the in-sim DatarefDataSource so the standalone honors the same switches.
      d.masterPowerOn = v != 0;
      break;
    case kDiscAvionicsPower:
      d.avionicsPowerOn = v != 0;
      break;
    default:
      break;
  }
}

void copyDiscreteFields(FlightData& dst, const FlightData& src) {
  dst.cdiSource = src.cdiSource;
  dst.transponderCode = src.transponderCode;
  dst.transponderMode = src.transponderMode;
  dst.flightDirectorActive = src.flightDirectorActive;
  dst.apEngaged = src.apEngaged;
  dst.ydEngaged = src.ydEngaged;
  dst.navSignalValid = src.navSignalValid;
  dst.cdiToFlag = src.cdiToFlag;
  dst.attitudeValid = src.attitudeValid;
  dst.headingValid = src.headingValid;
  dst.airspeedValid = src.airspeedValid;
  dst.altitudeValid = src.altitudeValid;
  dst.verticalSpeedValid = src.verticalSpeedValid;
  dst.nav1Valid = src.nav1Valid;
  dst.nav2Valid = src.nav2Valid;
  dst.com1Valid = src.com1Valid;
  dst.com2Valid = src.com2Valid;
  dst.transponderValid = src.transponderValid;
  dst.casLowVacuum = src.casLowVacuum;
  dst.casLowVoltage = src.casLowVoltage;
  dst.casFuelLow = src.casFuelLow;
  dst.casOilPressureLow = src.casOilPressureLow;
  dst.casOilTempHigh = src.casOilTempHigh;
  dst.casFuelPressureLow = src.casFuelPressureLow;
  dst.casPitotHeatOff = src.casPitotHeatOff;
  dst.casIcing = src.casIcing;
  dst.casGearUnsafe = src.casGearUnsafe;
  dst.casStallWarning = src.casStallWarning;
  dst.nav1IdentAudio = src.nav1IdentAudio;
  dst.nav2IdentAudio = src.nav2IdentAudio;
  dst.masterPowerOn = src.masterPowerOn;
  dst.avionicsPowerOn = src.avionicsPowerOn;
}

float wrap360(float deg) {
  deg = std::fmod(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

// Shortest signed difference target-current in degrees, in [-180, 180].
float angleDelta(float target, float current) {
  return std::fmod(target - current + 540.0f, 360.0f) - 180.0f;
}

void writeLe32(unsigned char* p, std::int32_t v) {
  p[0] = static_cast<unsigned char>(v & 0xFF);
  p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
  p[2] = static_cast<unsigned char>((v >> 16) & 0xFF);
  p[3] = static_cast<unsigned char>((v >> 24) & 0xFF);
}

std::int32_t readLe32(const unsigned char* p) {
  return static_cast<std::int32_t>(
      static_cast<std::uint32_t>(p[0]) |
      (static_cast<std::uint32_t>(p[1]) << 8) |
      (static_cast<std::uint32_t>(p[2]) << 16) |
      (static_cast<std::uint32_t>(p[3]) << 24));
}

float readLeFloat(const unsigned char* p) {
  std::int32_t bits = readLe32(p);
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

}  // namespace

XPlaneConnection::XPlaneConnection(std::string host, std::uint16_t port,
                                   NavDataStore& navData,
                                   AirspaceStore& airspace, AirwayStore& airways,
                                   AptDatStore& aptData, LandDataStore& landData,
                                   FmsPlanStore& fmsPlan,
                                   const TerrainSource* terrain,
                                   ChecklistSource* checklists,
                                   EisSource* eis,
                                   const ObstacleStore* obstacles,
                                   std::uint16_t bridgePort,
                                   bool fmsWriteEnabled)
    : terrain_(terrain),
      navData_(navData),
      airspace_(airspace),
      airways_(airways),
      aptData_(aptData),
      landData_(landData),
      obstacles_(obstacles),
      fmsPlan_(fmsPlan),
      checklists_(checklists),
      eisSource_(eis),
      host_(std::move(host)),
      port_(port),
      webApi_(host_, XPlaneWebApi::kDefaultPort),
      fmsBridge_(host_, bridgePort),
      fmsWriteEnabled_(fmsWriteEnabled) {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
  if (sock == INVALID_SOCKET) {
    socketHandle_ = -1;
    return;
  }
  u_long nonblocking = 1;
  ioctlsocket(sock, FIONBIO, &nonblocking);
#else
  if (sock < 0) {
    socketHandle_ = -1;
    return;
  }
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif

  socketHandle_ = static_cast<std::intptr_t>(sock);
  rebuildEisSubscriptions();
  sendSubscriptions(kSubscribeFrequencyHz);
}

XPlaneConnection::~XPlaneConnection() {
  if (socketHandle_ != -1) {
    // Politely cancel the subscriptions (frequency 0 = stop) before closing.
    sendSubscriptions(0);
    SocketHandle sock = static_cast<SocketHandle>(socketHandle_);
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
#else
    ::close(sock);
#endif
    socketHandle_ = -1;
  }
}

void XPlaneConnection::sendSubscriptions(int frequencyHz) {
  if (socketHandle_ == -1) return;
  SocketHandle sock = static_cast<SocketHandle>(socketHandle_);

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port_);
#ifdef _WIN32
  inet_pton(AF_INET, host_.c_str(), &dest.sin_addr);
#else
  dest.sin_addr.s_addr = inet_addr(host_.c_str());
#endif

  auto subscribe = [&](int index, const char* path) {
    unsigned char msg[kRrefRequestSize] = {0};
    std::memcpy(msg, "RREF", 4);  // byte 4 stays 0 (null terminator)
    writeLe32(msg + 5, frequencyHz);
    writeLe32(msg + 9, index);
    std::strncpy(reinterpret_cast<char*>(msg + 13), path, kDatarefNameSize - 1);
    ::sendto(sock, reinterpret_cast<const char*>(msg), kRrefRequestSize, 0,
             reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
  };

  for (int i = 0; i < kBindingCount; ++i) subscribe(i, kBindings[i].path);
  for (int j = 0; j < kDiscreteCount; ++j) {
    subscribe(kBindingCount + j, kDiscretePaths[j]);
  }
  subscribe(kZuluSubscriptionIndex, datarefs::kZuluTimeSec);
  for (int k = 0; k < kApModeCount; ++k) {
    subscribe(kApModeBaseIndex + k, kApModePaths[k]);
  }
  subscribe(kGpsSensitivityIndex, datarefs::kGpsHdefNmPerDot);
  subscribe(kLatitudeIndex, datarefs::kLatitudeDeg);
  subscribe(kLongitudeIndex, datarefs::kLongitudeDeg);

  // TCAS target arrays, one subscription per element ("path[i]").
  char path[128];
  for (int t = 0; t < kTrafficTargetCount; ++t) {
    for (int f = 0; f < kTrafficFieldCount; ++f) {
      std::snprintf(path, sizeof(path), "%s[%d]", kTrafficFieldPaths[f], t + 1);
      subscribe(kTrafficBaseIndex + t * kTrafficFieldCount + f, path);
    }
  }
  for (int n = 0; n < kNavInstrFieldCount; ++n) {
    subscribe(kNavInstrBaseIndex + n, kNavInstrPaths[n]);
  }
  subscribe(kDateDaysIndex, datarefs::kLocalDateDays);
  subscribeEisBindings(frequencyHz);
}

void XPlaneConnection::rebuildEisSubscriptions() {
  eisBindings_.clear();
  if (eisSource_ == nullptr || !eisSource_->ready()) {
    eisLayoutBindingCount_ = 0;
    return;
  }

  const EisLayout& layout = eisSource_->layout();
  eisLayoutBindingCount_ = layout.bindings.size();
  int subIndex = kEisSubscriptionBase;
  for (const EisDataBinding& spec : layout.bindings) {
    RuntimeEisBinding binding;
    binding.channel = spec.channel;
    binding.datarefPath = spec.datarefPath;
    binding.scale = spec.scale;
    binding.offset = spec.offset;
    binding.subIndex = subIndex++;
    eisBindings_.push_back(std::move(binding));
  }
}

void XPlaneConnection::subscribeEisBindings(int frequencyHz) {
  if (socketHandle_ == -1) return;
  SocketHandle sock = static_cast<SocketHandle>(socketHandle_);

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port_);
#ifdef _WIN32
  inet_pton(AF_INET, host_.c_str(), &dest.sin_addr);
#else
  dest.sin_addr.s_addr = inet_addr(host_.c_str());
#endif

  unsigned char msg[kRrefRequestSize] = {0};
  auto subscribe = [&](int index, const char* path) {
    std::memcpy(msg, "RREF", 4);
    writeLe32(msg + 5, frequencyHz);
    writeLe32(msg + 9, index);
    std::strncpy(reinterpret_cast<char*>(msg + 13), path, kDatarefNameSize - 1);
    ::sendto(sock, reinterpret_cast<const char*>(msg), kRrefRequestSize, 0,
             reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
  };

  for (const RuntimeEisBinding& binding : eisBindings_) {
    subscribe(binding.subIndex, binding.datarefPath.c_str());
  }
}

void XPlaneConnection::drainSocket() {
  if (socketHandle_ == -1) return;
  SocketHandle sock = static_cast<SocketHandle>(socketHandle_);

  unsigned char buf[kReceiveBufferSize];
  for (;;) {
#ifdef _WIN32
    int n = ::recvfrom(sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                       nullptr, nullptr);
#else
    ssize_t n =
        ::recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
#endif
    if (n <= 0) break;  // EWOULDBLOCK / no more packets
    if (n < kRrefHeaderSize || std::memcmp(buf, "RREF", 4) != 0) continue;

    for (int off = kRrefHeaderSize; off + kRrefRecordSize <= n;
         off += kRrefRecordSize) {
      const std::int32_t index = readLe32(buf + off);
      const float value = readLeFloat(buf + off + 4);
      // Decode into the target state; update() eases the displayed state toward
      // it so the gauges move smoothly between the ~20 Hz packets.
      if (index >= 0 && index < kBindingCount) {
        target_.*(kBindings[index].member) =
            value * kBindings[index].scale + kBindings[index].offset;
      } else if (index >= kBindingCount &&
                 index < kBindingCount + kDiscreteCount) {
        applyDiscrete(target_, index - kBindingCount, value);
      } else if (index == kZuluSubscriptionIndex) {
        zuluTargetSec_ = value;
        zuluHasTarget_ = true;
      } else if (index >= kApModeBaseIndex &&
                 index < kApModeBaseIndex + kApModeCount) {
        apModeStatus_[index - kApModeBaseIndex] =
            static_cast<int>(std::lround(value));
      } else if (index == kGpsSensitivityIndex) {
        gpsHdefNmPerDot_ = value;
      } else if (index == kLatitudeIndex) {
        ownshipLatDeg_ = value;
        haveLat_ = true;
      } else if (index == kLongitudeIndex) {
        ownshipLonDeg_ = value;
        haveLon_ = true;
      } else if (index >= kTrafficBaseIndex &&
                 index < kTrafficBaseIndex +
                             kTrafficTargetCount * kTrafficFieldCount) {
        static_assert(kTrafficTargetCount == kTrafficSlotCount &&
                          kTrafficFieldCount == kTrafficSlotFields,
                      "subscription layout must match the raw storage");
        const int rel = index - kTrafficBaseIndex;
        trafficRaw_[rel / kTrafficFieldCount][rel % kTrafficFieldCount] =
            value;
      } else if (index >= kNavInstrBaseIndex &&
                 index < kNavInstrBaseIndex + kNavInstrFieldCount) {
        navInstr_[index - kNavInstrBaseIndex] = value;
      } else if (index == kDateDaysIndex) {
        // X-Plane reports 0-based day-of-year; FlightData carries 1-based.
        target_.utcDayOfYear = static_cast<int>(std::lround(value)) + 1;
      } else if (index >= kEisSubscriptionBase) {
        const int rel = index - kEisSubscriptionBase;
        if (rel >= 0 && rel < static_cast<int>(eisBindings_.size())) {
          eisBindings_[static_cast<std::size_t>(rel)].target = value;
        }
      }
    }
    lastPacketSeconds_ = elapsedSeconds_;
    everConnected_ = true;
  }
}

void XPlaneConnection::updateAircraftProfile() {
  const std::string icao = webApi_.aircraftIcao();
  const std::string acfPath = webApi_.aircraftAcfRelativePath();
  if (icao.empty() && acfPath.empty()) return;
  if (icao == lastAircraftIcao_ && acfPath == lastAircraftAcfPath_) return;
  lastAircraftIcao_ = icao;
  lastAircraftAcfPath_ = acfPath;

  if (eisSource_ != nullptr) {
    eisSource_->setAircraftIdentity(icao, acfPath);
    eisWasReady_ = false;
  }
  if (checklists_ != nullptr) {
    checklists_->setAircraftIdentity(icao, acfPath);
  }
}

void XPlaneConnection::update(double dtSeconds) {
  elapsedSeconds_ += dtSeconds;
  sinceResubscribeSeconds_ += dtSeconds;

  updateAircraftProfile();
  if (eisSource_ != nullptr) {
    eisSource_->refreshIfChanged();
    const bool ready = eisSource_->ready();
    if (ready && (!eisWasReady_ ||
                  eisSource_->layout().bindings.size() != eisLayoutBindingCount_)) {
      rebuildEisSubscriptions();
      sendSubscriptions(kSubscribeFrequencyHz);
    }
    eisWasReady_ = ready;
  }

  drainSocket();

  // Airspeed trend: measure rate when a new IAS packet arrives (~20 Hz), then
  // ease the displayed trend toward that rate each render frame.
  if (target_.airspeedKts != prevTargetAirspeedKts_) {
    const double packetDt = elapsedSeconds_ - lastAirspeedTargetSeconds_;
    if (packetDt > 1e-4) {
      lastInstTrendKts_ = (target_.airspeedKts - prevTargetAirspeedKts_) /
                          static_cast<float>(packetDt) * 6.0f;
    }
    prevTargetAirspeedKts_ = target_.airspeedKts;
    lastAirspeedTargetSeconds_ = elapsedSeconds_;
  }

  const ConnectionState state = connectionState();

  if (state == ConnectionState::Connected) {
    if (!primed_) {
      // First fresh data after (re)connecting: jump straight to it so the
      // gauges don't visibly ease in from their defaults / last-known values.
      data_ = target_;
      prevTargetAirspeedKts_ = target_.airspeedKts;
      lastAirspeedTargetSeconds_ = elapsedSeconds_;
      lastInstTrendKts_ = 0.0f;
      data_.airspeedTrendKts = 0.0f;
      data_.altitudeTrendFt = data_.verticalSpeedFpm * (6.0f / 60.0f);
      for (const RuntimeEisBinding& b : eisBindings_) {
        data_.eisChannels[b.channel] = b.target * b.scale + b.offset;
      }
      syncEisLegacyFields(data_);
      primed_ = true;
    } else {
      // alpha = 1 - e^(-dt/tau): the fraction of the remaining gap to close
      // this frame, framerate-independent.
      const float alpha = static_cast<float>(
          1.0 - std::exp(-dtSeconds / kSmoothingTimeConstantSeconds));
      for (int i = 0; i < kBindingCount; ++i) {
        float& cur = data_.*(kBindings[i].member);
        const float tgt = target_.*(kBindings[i].member);
        switch (kBindings[i].smooth) {
          case Smooth::Snap:
            cur = tgt;
            break;
          case Smooth::Linear:
            cur += (tgt - cur) * alpha;
            break;
          case Smooth::Angle:
            cur = wrap360(cur + angleDelta(tgt, cur) * alpha);
            break;
        }
      }

      // Discrete fields aren't smoothed; copy them straight through.
      copyDiscreteFields(data_, target_);

      // Trend vectors (6-second projection). Altitude tracks the VSI; airspeed
      // eases toward the packet-derived rate computed above.
      data_.altitudeTrendFt = data_.verticalSpeedFpm * (6.0f / 60.0f);
      if (dtSeconds > 1e-4) {
        const float trendAlpha = static_cast<float>(
            1.0 - std::exp(-dtSeconds / kAirspeedTrendTimeConstantSeconds));
        data_.airspeedTrendKts +=
            (lastInstTrendKts_ - data_.airspeedTrendKts) * trendAlpha;
      }

      for (RuntimeEisBinding& b : eisBindings_) {
        const float converted = b.target * b.scale + b.offset;
        float& cur = data_.eisChannels[b.channel];
        cur += (converted - cur) * alpha;
      }
      syncEisLegacyFields(data_);
    }
    updateFmaModes();
    updateNavInstrumentation();
    updateGpsGlidepathCoupling();
    // GPS flight phase (ENR/TERM/APR/OCN) is a discrete annunciation derived
    // from the live CDI sensitivity, so it is set straight through (no easing).
    data_.gpsFlightPhase = gpsPhaseFromSensitivity(gpsHdefNmPerDot_);
    // Active destination identifier comes from the Web API (string dataref).
    // When unavailable it is empty, which clears the placeholder rather than
    // showing a stale demo waypoint. X-Plane exposes no FROM-waypoint string
    // dataref, so the leg renders direct-to ("->KXXX") unless we infer FROM
    // from the displayed flight plan.
    const std::string simDest = webApi_.destinationId();
    const std::vector<MapLeg> plan = displayedFlightPlan();
    syncDirectToWithSimulator(simDest, plan);

    data_.fmaToWpt = simDest;
    data_.fmaFromWpt.clear();

    if (directToActive_ && !directTo_.id.empty()) {
      data_.fmaToWpt = directTo_.id;
      if (map_.positionValid) {
        data_.fmaLegBearingDeg = static_cast<float>(
            navBearingDeg(map_.ownshipLat, map_.ownshipLon, directTo_.lat,
                          directTo_.lon));
        data_.fmaLegDistanceNm = static_cast<float>(
            navDistanceNm(map_.ownshipLat, map_.ownshipLon, directTo_.lat,
                          directTo_.lon));
      }
    } else {
      const int dtoIdx = legIndexInPlan(plan, directTo_.id);
      if (!directTo_.id.empty() && dtoIdx >= 0 &&
          dtoIdx + 1 < static_cast<int>(plan.size()) &&
          (simDest.empty() || simDest == directTo_.id)) {
        data_.fmaFromWpt = directTo_.id;
        data_.fmaToWpt = plan[static_cast<std::size_t>(dtoIdx + 1)].id;
      } else {
        const int toIdx = legIndexInPlan(plan, data_.fmaToWpt);
        if (toIdx > 0) {
          data_.fmaFromWpt = plan[static_cast<std::size_t>(toIdx - 1)].id;
        }
      }
    }
    data_.nav1Ident = webApi_.nav1Ident();
    data_.nav2Ident = webApi_.nav2Ident();
    // Sim date passes straight through (it only changes at midnight).
    data_.utcDayOfYear = target_.utcDayOfYear;
    updateZuluClock(dtSeconds);
    updateMap(dtSeconds);
    syncDisplayBackup(data_, dtSeconds);
  } else {
    // Link down: re-prime on the next reconnect, and periodically re-subscribe
    // so we recover if X-Plane was started after us (or restarted).
    primed_ = false;
    zuluPrimed_ = false;
    map_.positionValid = false;
    if (sinceResubscribeSeconds_ >= kResubscribeIntervalSeconds) {
      sendSubscriptions(kSubscribeFrequencyHz);
      sinceResubscribeSeconds_ = 0.0;
    }
  }
}

void XPlaneConnection::updateZuluClock(double dtSeconds) {
  if (!zuluHasTarget_) return;  // keep defaults until the first zulu packet
  if (!zuluPrimed_) {
    zuluDisplaySec_ = zuluTargetSec_;
    zuluPrimed_ = true;
  } else {
    // Free-run at real time, then ease toward the sim's value. Between packets
    // (or when one is lost) the clock keeps ticking instead of holding, so the
    // displayed seconds never skip. During a pause the sim value stops while the
    // free-run keeps advancing; the easing balances the two so the clock simply
    // settles rather than running away.
    zuluDisplaySec_ += dtSeconds;
    double diff = zuluTargetSec_ - zuluDisplaySec_;
    // Take the shortest path so the midnight rollover wraps cleanly.
    if (diff > kSecondsPerDay / 2.0) diff -= kSecondsPerDay;
    if (diff < -kSecondsPerDay / 2.0) diff += kSecondsPerDay;
    if (std::fabs(diff) > kZuluResyncThresholdSeconds) {
      zuluDisplaySec_ = zuluTargetSec_;  // time set / large gap: snap
    } else {
      const double alpha =
          1.0 - std::exp(-dtSeconds / kZuluClockTimeConstantSeconds);
      zuluDisplaySec_ += diff * alpha;
    }
  }

  zuluDisplaySec_ = std::fmod(zuluDisplaySec_, kSecondsPerDay);
  if (zuluDisplaySec_ < 0.0) zuluDisplaySec_ += kSecondsPerDay;
  const int total = static_cast<int>(zuluDisplaySec_);
  data_.utcHour = (total / 3600) % 24;
  data_.utcMinute = (total / 60) % 60;
  data_.utcSecond = total % 60;
}

std::vector<MapLeg> XPlaneConnection::displayedFlightPlan() const {
  if (routeOverrideSet_) return routeOverride_;
  bool bridgeAvailable = false;
  std::vector<MapLeg> bridgePlan = fmsBridge_.flightPlan(bridgeAvailable);
  if (bridgeAvailable && !bridgePlan.empty()) return bridgePlan;
  fmsPlan_.refreshIfChanged();
  if (fmsPlan_.loaded()) return fmsPlan_.flightPlan();
  return {};
}

void XPlaneConnection::releaseDirectToOverride() {
  directToActive_ = false;
}

void XPlaneConnection::syncDirectToWithSimulator(
    const std::string& simDestination, const std::vector<MapLeg>& plan) {
  if (!directToActive_ || directTo_.id.empty()) return;

  const int dtoIdx = legIndexInPlan(plan, directTo_.id);
  const int simIdx = legIndexInPlan(plan, simDestination);

  if (!simDestination.empty() && simDestination != directTo_.id) {
    if (dtoIdx >= 0 && simIdx > dtoIdx) {
      releaseDirectToOverride();
      return;
    }
    if (dtoIdx >= 0 && simIdx >= 0 && simIdx < dtoIdx) {
      // Sim still names a leg before our Direct-To target; keep the override.
      return;
    }
    if (dtoIdx < 0) {
      // Direct-To was not a listed plan leg; trust the sim when it disagrees.
      releaseDirectToOverride();
      return;
    }
  }

  if (dtoIdx < 0 || dtoIdx + 1 >= static_cast<int>(plan.size())) return;
  if (!haveLat_ || !haveLon_) return;

  const double distNm =
      navDistanceNm(static_cast<double>(ownshipLatDeg_),
                    static_cast<double>(ownshipLonDeg_), directTo_.lat,
                    directTo_.lon);
  if (distNm > kDirectToArrivalNm) return;

  // Passed the Direct-To fix; resume sequencing through the remaining plan even
  // if the gps_nav_id string has not caught up yet.
  releaseDirectToOverride();
}

void XPlaneConnection::updateMap(double dtSeconds) {
  map_.rangeNm = chartRangeNm_;
  map_.terrain = terrain_;
  map_.positionValid = haveLat_ && haveLon_;
  if (!map_.positionValid) return;

  map_.ownshipLat = static_cast<double>(ownshipLatDeg_);
  map_.ownshipLon = static_cast<double>(ownshipLonDeg_);

  // Live datalink NEXRAD overlay centered on the aircraft (real ground radar).
  nexrad_.setCenter(map_.ownshipLat, map_.ownshipLon);
  nexrad_.advance(dtSeconds);
  map_.nexrad = &nexrad_;

  // Flight plan precedence:
  //   1. An externally supplied route (SimBrief OFP / FPL page edits) wins.
  //   2. Otherwise the live FMS from the in-sim plugin bridge, when reachable
  //      and non-empty -- this is the real in-cockpit route over UDP.
  //   3. Otherwise an X-Plane .fms file (see FmsPlanStore), the offline source
  //      that works without the plugin installed.
  // Display-only Direct-To course (the live sim navigation is unchanged).
  map_.directToActive = directToActive_;
  map_.directTo = directTo_;

  if (routeOverrideSet_) {
    map_.flightPlan = routeOverride_;
  } else {
    bool bridgeAvailable = false;
    std::vector<MapLeg> bridgePlan = fmsBridge_.flightPlan(bridgeAvailable);
    if (bridgeAvailable && !bridgePlan.empty()) {
      map_.flightPlan = std::move(bridgePlan);
    } else {
      fmsPlan_.refreshIfChanged();
      if (fmsPlan_.loaded() && !fmsPlan_.flightPlan().empty()) {
        map_.flightPlan = fmsPlan_.flightPlan();
      }
    }
  }

  // Nearby navaids/fixes from the parsed nav database. Rebuild on a throttled
  // timer rather than every frame, since the database spans the whole world.
  // When the MFD Map Pointer is active the queries follow the pointer instead
  // of ownship so the panned-to area has data (see setMapPanCenter()).
  const double queryLat = mapPanActive_ ? mapPanLat_ : map_.ownshipLat;
  const double queryLon = mapPanActive_ ? mapPanLon_ : map_.ownshipLon;
  sinceMapRebuildSeconds_ += dtSeconds;
  const bool due = map_.features.empty() || mapPanDirty_ ||
                   sinceMapRebuildSeconds_ >= kMapRebuildIntervalSeconds;
  if (due) {
    if (navData_.loaded()) {
      map_.features = navData_.nearby(queryLat, queryLon, kMapQueryRangeNm,
                                      kMaxMapFeatures);
      map_.navDatabase = navData_.navDatabaseInfo();
      if (aptData_.loaded()) {
        for (MapFeature& f : map_.features) {
          aptData_.enrichAirport(f);
        }
      }
    }
    if (airspace_.loaded()) {
      map_.airspaces = airspace_.nearby(queryLat, queryLon, kMapQueryRangeNm,
                                        kMaxMapAirspaces);
    }
    if (airways_.loaded()) {
      map_.airways =
          airways_.nearby(queryLat, queryLon, kMapQueryRangeNm, kMaxMapAirways);
    }
    if (aptData_.loaded()) {
      map_.runways = aptData_.nearby(queryLat, queryLon, kRunwayQueryRangeNm,
                                     kMaxMapRunways);
      map_.taxiways = aptData_.nearbyTaxiways(
          queryLat, queryLon, kTaxiwayQueryRangeNm, kMaxMapTaxiways);
      map_.taxiwayLabels = aptData_.nearbyTaxiwayLabels(
          queryLat, queryLon, kTaxiwayLabelQueryRangeNm, kMaxMapTaxiwayLabels);
    }
    if (landData_.loaded()) {
      map_.landLines = landData_.nearbyLines(queryLat, queryLon,
                                             chartRangeNm_, kMaxMapLandLines,
                                             mapViewHalfExtentNm_);
      map_.cities = landData_.nearbyCities(queryLat, queryLon, chartRangeNm_,
                                           kMaxMapCities);
    }
    if (obstacles_ != nullptr && obstacles_->loaded()) {
      map_.obstacles = obstacles_->nearby(queryLat, queryLon,
                                           kObstacleQueryRangeNm, kMaxMapObstacles);
    }
    mapPanDirty_ = false;
    sinceMapRebuildSeconds_ = 0.0;
  }

  // Traffic: decoded every frame (only a handful of slots) so targets track
  // the 20 Hz RREF stream instead of the 1 Hz database rebuild.
  map_.traffic.clear();
  const double cosLat = std::cos(map_.ownshipLat * kDegToRad);
  for (int t = 0; t < kTrafficSlotCount; ++t) {
    const float lat = trafficRaw_[t][0];
    const float lon = trafficRaw_[t][1];
    if (lat == 0.0f && lon == 0.0f) continue;  // unused TCAS slot
    const double dLatNm = (lat - map_.ownshipLat) * kNmPerDegLat;
    const double dLonNm = (lon - map_.ownshipLon) * kNmPerDegLat * cosLat;
    const double distNm = std::sqrt(dLatNm * dLatNm + dLonNm * dLonNm);
    if (distNm > kTrafficMaxRangeNm) continue;

    MapTraffic tgt;
    tgt.lat = lat;
    tgt.lon = lon;
    tgt.relAltFt = trafficRaw_[t][2] * kMetersToFeet - data_.altitudeFt;
    tgt.verticalSpeedFpm = trafficRaw_[t][3];
    tgt.trafficAdvisory = distNm <= kTrafficTaRangeNm &&
                          std::fabs(tgt.relAltFt) <= kTrafficTaAltFt;
    map_.traffic.push_back(tgt);
  }
}

void XPlaneConnection::updateFmaModes() {
  static_assert(kApModeCount == kApModeStatusCount,
                "AP mode subscription table and status array must stay in sync");

  // With the flight director (and therefore the autopilot) off, the G1000
  // blanks the lateral/vertical mode annunciators entirely.
  if (!data_.flightDirectorActive) {
    data_.fmaLateralActive.clear();
    data_.fmaLateralArmed.clear();
    data_.fmaVerticalActive.clear();
    data_.fmaVerticalArmed.clear();
    data_.fmaVerticalApproachArmed.clear();
    data_.fmaVerticalValue = 0;
    data_.fmaVerticalUnits.clear();
    data_.selectedAirspeedValid = false;
    data_.selectedVsValid = false;
    return;
  }

  auto mode = [&](int m) { return apModeStatus_[m]; };
  // The lateral nav label follows the selected CDI source (GPS vs a VOR/LOC).
  const std::string navLabel =
      (data_.cdiSource == CdiSource::Gps) ? "GPS" : "VOR";

  // Lateral: the highest-priority captured mode is green, an armed mode is the
  // white prefix beside it.
  std::string latActive;
  if (mode(kApBackcourse) == kApModeActive) {
    latActive = "BC";
  } else if (mode(kApNav) == kApModeActive) {
    latActive = navLabel;
  } else if (mode(kApHeading) == kApModeActive) {
    latActive = "HDG";
  } else if (mode(kApRoll) == kApModeActive) {
    latActive = "ROL";
  }

  std::string latArmed;
  if (mode(kApBackcourse) == kApModeArmed) {
    latArmed = "BC";
  } else if (mode(kApNav) == kApModeArmed) {
    latArmed = navLabel;
  } else if (mode(kApHeading) == kApModeArmed) {
    latArmed = "HDG";
  }
  data_.fmaLateralActive = latActive;
  data_.fmaLateralArmed = latArmed;

  // Vertical: the captured mode is green; altitude hold and FLC carry a cyan
  // reference value (preselected altitude or target airspeed).
  std::string vertActive;
  int vertValue = 0;
  std::string vertUnits;
  if (mode(kApGlideslope) == kApModeActive) {
    vertActive = "GS";
  } else if (mode(kApAltitudeHold) == kApModeActive) {
    vertActive = "ALT";
    vertValue = static_cast<int>(std::lround(data_.selectedAltitudeFt));
    vertUnits = "FT";
  } else if (mode(kApVnav) == kApModeActive) {
    vertActive = "VPTH";
  } else if (mode(kApSpeed) == kApModeActive) {
    vertActive = "FLC";
    vertValue = static_cast<int>(std::lround(data_.selectedAirspeedKts));
    vertUnits = "KT";
  } else if (mode(kApVerticalSpeed) == kApModeActive) {
    vertActive = "VS";
    vertValue = static_cast<int>(std::lround(data_.selectedVerticalSpeedFpm));
    vertUnits = "FPM";
  } else if (mode(kApPitch) == kApModeActive) {
    vertActive = "PIT";
  }
  data_.fmaVerticalActive = vertActive;
  data_.fmaVerticalValue = vertValue;
  data_.fmaVerticalUnits = vertUnits;
  data_.selectedAirspeedValid =
      mode(kApSpeed) == kApModeActive && data_.selectedAirspeedKts > 0.5f;
  data_.selectedVsValid = mode(kApVerticalSpeed) == kApModeActive;

  // Vertical armed: ALTS (altitude preselect) whenever altitude capture is
  // armed; an armed glideslope occupies the rightmost approach-armed slot.
  data_.fmaVerticalArmed =
      (mode(kApAltitudeArmed) != 0) ? "ALTS" : std::string();
  data_.fmaVerticalApproachArmed =
      (mode(kApGlideslope) == kApModeArmed) ? "GS" : std::string();
}

void XPlaneConnection::updateNavInstrumentation() {
  if (navInstr_[kInnerMarker] > 0.5f) {
    data_.markerBeacon = MarkerBeacon::Inner;
  } else if (navInstr_[kMiddleMarker] > 0.5f) {
    data_.markerBeacon = MarkerBeacon::Middle;
  } else if (navInstr_[kOuterMarker] > 0.5f) {
    data_.markerBeacon = MarkerBeacon::Outer;
  } else {
    data_.markerBeacon = MarkerBeacon::None;
  }

  if (data_.cdiSource == CdiSource::Nav1 || data_.cdiSource == CdiSource::Nav2) {
    const bool nav1 = data_.cdiSource == CdiSource::Nav1;
    data_.vdiKind = VerticalDeviationKind::Glideslope;
    data_.vdiValid = navInstr_[nav1 ? kNav1GsFlag : kNav2GsFlag] < 0.5f;
    data_.vdiDeviationDots = navInstr_[nav1 ? kNav1Vdef : kNav2Vdef];
    const float dme = navInstr_[nav1 ? kNav1Dme : kNav2Dme];
    data_.dmeValid = dme > 0.05f;
    data_.dmeDistanceNm = dme;
    data_.dmeMode = nav1 ? "NAV1" : "NAV2";
    data_.dmeFreqMhz = nav1 ? data_.nav1ActiveMhz : data_.nav2ActiveMhz;
  } else {
    const float gpsDme = data_.fmaLegDistanceNm;
    data_.vdiKind = VerticalDeviationKind::None;
    data_.vdiValid = false;
    data_.vdiDeviationDots = 0.0f;
    data_.dmeValid = gpsDme > 0.05f;
    data_.dmeDistanceNm = gpsDme;
    data_.dmeMode = "GPS";
    data_.dmeFreqMhz = 0.0f;
  }
}

void XPlaneConnection::updateGpsGlidepathCoupling() {
  if (connectionState() != ConnectionState::Connected) {
    gpsGlidepathHasSignal_ = false;
    return;
  }

  // Only synthesize a GPS glidepath for RNAV; ILS GS comes from the NAV radios.
  if (data_.cdiSource != CdiSource::Gps) {
    gpsGlidepathHasSignal_ = false;
    return;
  }

  const int gsStatus = apModeStatus_[kApGlideslope];
  if (gsStatus == 0) {
    gpsGlidepathHasSignal_ = false;
    return;
  }

  const GlidepathSolution gp = computeGlidepath(map_, data_);
  if (!gp.valid) {
    gpsGlidepathHasSignal_ = false;
    return;
  }

  // Tell X-Plane a WAAS/LPV vertical path is available and publish deviation
  // dots so APP/GS can capture and track like an ILS glideslope.
  sendDataref(datarefs::kGpsHasGlideslope, 1.0f);
  if (std::fabs(gp.deviationDots - lastSentGpsVdefDots_) > 0.02f) {
    sendDataref(datarefs::kGpsVdefDots, gp.deviationDots);
    lastSentGpsVdefDots_ = gp.deviationDots;
  }
  gpsGlidepathHasSignal_ = true;

  bool gsStatusChanged = false;
  if (gsStatus == kApModeArmed) {
    // X-Plane captures GS when crossing the path from below with a valid signal.
    const bool interceptable =
        gp.altitudeErrorFt <= 100.0f && gp.altitudeErrorFt >= -2000.0f;
    const bool inEnvelope =
        std::fabs(gp.deviationDots) <= kGlidepathCaptureMaxDots;
    if (interceptable && inEnvelope) {
      sendDataref(datarefs::kApGlideslopeStatus,
                  static_cast<float>(kApModeActive));
      apModeStatus_[kApGlideslope] = kApModeActive;
      gsStatusChanged = true;
    }
  }

  if (apModeStatus_[kApGlideslope] == kApModeActive && data_.apEngaged) {
    // Some aircraft APs need an explicit VS target while GS is active; the sim
    // may ignore this when GS pitch tracking is working from the injected vdef.
    if (std::fabs(gp.targetVerticalSpeedFpm - lastSentGsTrackVsFpm_) > 10.0f) {
      sendDataref(datarefs::kSelectedVerticalSpeedFpm, gp.targetVerticalSpeedFpm);
      lastSentGsTrackVsFpm_ = gp.targetVerticalSpeedFpm;
    }
  }

  if (gsStatusChanged) {
    updateFmaModes();
  }
}

ConnectionState XPlaneConnection::connectionState() const {
  if (!everConnected_) return ConnectionState::Connecting;
  if (elapsedSeconds_ - lastPacketSeconds_ > kStaleTimeoutSeconds) {
    return ConnectionState::Disconnected;
  }
  return ConnectionState::Connected;
}

namespace {

struct RadioPaths {
  const char* active;
  const char* standby;
  float FlightData::* activeMember;
  float FlightData::* standbyMember;
};

RadioPaths radioPaths(RadioUnit unit) {
  switch (unit) {
    case RadioUnit::Nav1:
      return {datarefs::kNav1FrequencyHz, datarefs::kNav1StandbyFrequencyHz,
              &FlightData::nav1ActiveMhz, &FlightData::nav1StandbyMhz};
    case RadioUnit::Nav2:
      return {datarefs::kNav2FrequencyHz, datarefs::kNav2StandbyFrequencyHz,
              &FlightData::nav2ActiveMhz, &FlightData::nav2StandbyMhz};
    case RadioUnit::Com1:
      return {datarefs::kCom1FrequencyHz, datarefs::kCom1StandbyFrequencyHz,
              &FlightData::com1ActiveMhz, &FlightData::com1StandbyMhz};
    case RadioUnit::Com2:
      return {datarefs::kCom2FrequencyHz, datarefs::kCom2StandbyFrequencyHz,
              &FlightData::com2ActiveMhz, &FlightData::com2StandbyMhz};
  }
  return {datarefs::kNav1FrequencyHz, datarefs::kNav1StandbyFrequencyHz,
          &FlightData::nav1ActiveMhz, &FlightData::nav1StandbyMhz};
}

constexpr float kMhzToRadioHz = 100.0f;

// Radio frequency datarefs are integers (MHz x 100). A float MHz like 114.15f
// is actually ~114.1499996, so 114.15 * 100 = 11414.9996; writing that to the
// integer dataref truncates to 11414 (-> 114.14). Round to the nearest 10 kHz
// channel first so the active frequency lands exactly where the pilot tuned it.
float radioMhzToHz(float mhz) {
  return static_cast<float>(std::lround(mhz * kMhzToRadioHz));
}

}  // namespace

void XPlaneConnection::sendDataref(const char* path, float value) {
  if (socketHandle_ == -1) return;
  SocketHandle sock = static_cast<SocketHandle>(socketHandle_);

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port_);
#ifdef _WIN32
  inet_pton(AF_INET, host_.c_str(), &dest.sin_addr);
#else
  dest.sin_addr.s_addr = inet_addr(host_.c_str());
#endif

  unsigned char msg[kDrefMessageSize] = {0};
  std::memcpy(msg, "DREF", 4);
  std::int32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe32(msg + 5, bits);
  std::strncpy(reinterpret_cast<char*>(msg + 9), path, kDrefNameSize - 1);
  ::sendto(sock, reinterpret_cast<const char*>(msg),
           static_cast<int>(sizeof(msg)), 0,
           reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
}

void XPlaneConnection::setMapPanCenter(bool active, double lat, double lon) {
  if (active != mapPanActive_ ||
      (active && (lat != mapPanLat_ || lon != mapPanLon_))) {
    mapPanDirty_ = true;  // pointer toggled/moved: re-scan around the new center
  }
  mapPanActive_ = active;
  mapPanLat_ = lat;
  mapPanLon_ = lon;
}

void XPlaneConnection::setChartRangeNm(float rangeNm) {
  if (chartRangeNm_ != rangeNm) {
    chartRangeNm_ = rangeNm;
    mapPanDirty_ = true;
  }
}

void XPlaneConnection::setMapViewHalfExtentNm(float halfExtentNm) {
  if (mapViewHalfExtentNm_ != halfExtentNm) {
    mapViewHalfExtentNm_ = halfExtentNm;
    mapPanDirty_ = true;
  }
}

void XPlaneConnection::tuneRadioStandby(RadioUnit unit, float standbyMhz) {
  const RadioPaths paths = radioPaths(unit);
  sendDataref(paths.standby, radioMhzToHz(standbyMhz));
  target_.*(paths.standbyMember) = standbyMhz;
  data_.*(paths.standbyMember) = standbyMhz;
}

void XPlaneConnection::transferRadio(RadioUnit unit) {
  const RadioPaths paths = radioPaths(unit);
  const float active = target_.*(paths.activeMember);
  const float standby = target_.*(paths.standbyMember);
  sendDataref(paths.active, radioMhzToHz(standby));
  sendDataref(paths.standby, radioMhzToHz(active));
  target_.*(paths.activeMember) = standby;
  target_.*(paths.standbyMember) = active;
  data_.*(paths.activeMember) = standby;
  data_.*(paths.standbyMember) = active;
}

void XPlaneConnection::setRadioVolume(RadioUnit unit, float volume) {
  const char* path = nullptr;
  float FlightData::* member = nullptr;
  switch (unit) {
    case RadioUnit::Nav1:
      path = datarefs::kNav1Volume;
      member = &FlightData::nav1Volume;
      break;
    case RadioUnit::Nav2:
      path = datarefs::kNav2Volume;
      member = &FlightData::nav2Volume;
      break;
    case RadioUnit::Com1:
      path = datarefs::kCom1Volume;
      member = &FlightData::com1Volume;
      break;
    case RadioUnit::Com2:
      path = datarefs::kCom2Volume;
      member = &FlightData::com2Volume;
      break;
  }
  sendDataref(path, volume);
  target_.*member = volume;
  data_.*member = volume;
}

void XPlaneConnection::setNavIdent(RadioUnit unit, bool on) {
  const char* path =
      unit == RadioUnit::Nav2 ? datarefs::kNav2IdentAudio
                              : datarefs::kNav1IdentAudio;
  bool FlightData::* member = navIdentAudioMember(unit);
  sendDataref(path, on ? 1.0f : 0.0f);
  target_.*member = on;
  data_.*member = on;
}

void XPlaneConnection::setTransponderCode(int code) {
  sendDataref(datarefs::kTransponderCode, static_cast<float>(code));
  target_.transponderCode = code;
  data_.transponderCode = code;
}

void XPlaneConnection::setTransponderMode(int mode) {
  sendDataref(datarefs::kTransponderMode, static_cast<float>(mode));
  applyDiscrete(target_, kDiscTransponderMode, static_cast<float>(mode));
  applyDiscrete(data_, kDiscTransponderMode, static_cast<float>(mode));
}

void XPlaneConnection::setHeadingBug(float deg) {
  sendDataref(datarefs::kSelectedHeadingDegMag, deg);
  target_.selectedHeadingDeg = deg;
  data_.selectedHeadingDeg = deg;
}

void XPlaneConnection::setSelectedCourse(float deg) {
  sendDataref(datarefs::kHsiObsCourseDegMag, deg);
  target_.courseDeg = deg;
  data_.courseDeg = deg;
}

void XPlaneConnection::setBaroInHg(float inHg) {
  sendDataref(datarefs::kBaroSettingInHg, inHg);
  target_.baroSettingInHg = inHg;
  data_.baroSettingInHg = inHg;
}

}  // namespace avionics
