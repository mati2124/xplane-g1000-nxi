#include "XPlaneConnection.h"

#include <cmath>
#include <cstring>

#include "avionics/Datarefs.h"

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
// Avgas mass-to-volume conversions for the EIS fuel readouts (6.01 lb/gal).
constexpr float kKgPerGallonAvgas = 2.72155f;
constexpr float kKgSecToGph = 3600.0f / kKgPerGallonAvgas;
constexpr float kKgToGallons = 1.0f / kKgPerGallonAvgas;
// Celsius -> Fahrenheit (the EIS oil/EGT readouts are in deg F).
constexpr float kCToFScale = 1.8f;
constexpr float kCToFOffset = 32.0f;
// X-Plane's vacuum dataref is a 0..1 ratio of maximum pump output; a healthy
// GA suction system reads ~5 inHg at the gauge.
constexpr float kVacuumRatioToInHg = 5.0f;
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

// Zulu clock (sim/time/zulu_time_sec) handling. The displayed clock free-runs at
// real time between packets and is eased toward the sim's value with this time
// constant, so normal UDP jitter/loss never makes the seconds skip. A delta
// larger than the resync threshold (sim time set, a pause/resume gap, or the
// midnight rollover) is snapped instead of eased.
constexpr double kZuluClockTimeConstantSeconds = 0.5;
constexpr double kZuluResyncThresholdSeconds = 3.0;
constexpr double kSecondsPerDay = 86400.0;

// RREF request wire format (little endian):
//   "RREF\0" + int32 frequency + int32 index + char[400] dataref name.
constexpr int kDatarefNameSize = 400;
constexpr int kRrefRequestSize = 5 + 4 + 4 + kDatarefNameSize;  // 413 bytes
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

    // EIS engine/fuel/electrical indicators for the MFD engine strip.
    {datarefs::kEngineRpm, 1.0f, &FlightData::engineRpm, Smooth::Linear},
    {datarefs::kFuelFlowKgSec, kKgSecToGph, &FlightData::fuelFlowGph,
     Smooth::Linear},
    {datarefs::kOilPressurePsi, 1.0f, &FlightData::oilPressurePsi,
     Smooth::Linear},
    {datarefs::kOilTemperatureDegC, kCToFScale, &FlightData::oilTempDegF,
     Smooth::Linear, kCToFOffset},
    {datarefs::kEgtDegC, kCToFScale, &FlightData::egtDegF, Smooth::Linear,
     kCToFOffset},
    {datarefs::kVacuumRatio, kVacuumRatioToInHg, &FlightData::vacuumInHg,
     Smooth::Linear},
    {datarefs::kFuelQuantityLeftKg, kKgToGallons, &FlightData::fuelQtyLeftGal,
     Smooth::Linear},
    {datarefs::kFuelQuantityRightKg, kKgToGallons,
     &FlightData::fuelQtyRightGal, Smooth::Linear},
    {datarefs::kHobbsTimeHours, 1.0f, &FlightData::engineHours, Smooth::Snap},
    {datarefs::kBusVoltsMain, 1.0f, &FlightData::busVoltsMain, Smooth::Linear},
    {datarefs::kBusVoltsEssential, 1.0f, &FlightData::busVoltsEssential,
     Smooth::Linear},
    {datarefs::kBatteryAmpsMain, 1.0f, &FlightData::battAmpsMain,
     Smooth::Linear},
    {datarefs::kBatteryAmpsStandby, 1.0f, &FlightData::battAmpsStandby,
     Smooth::Linear},
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
  kDiscreteCount,
};

const char* const kDiscretePaths[kDiscreteCount] = {
    datarefs::kHsiSourceSelect,      datarefs::kTransponderCode,
    datarefs::kTransponderMode,      datarefs::kFlightDirectorMode,
    datarefs::kYawDamperOn,          datarefs::kHsiFromTo,
    datarefs::kFailAttitude,         datarefs::kFailHeading,
    datarefs::kFailAirspeed,         datarefs::kFailAltimeter,
    datarefs::kFailVerticalSpeed,    datarefs::kAnnunLowVacuum,
    datarefs::kAnnunLowVoltage,      datarefs::kAnnunFuelLow,
    datarefs::kAnnunOilPressureLow,  datarefs::kAnnunOilTempHigh,
    datarefs::kAnnunFuelPressureLow, datarefs::kAnnunPitotHeat,
    datarefs::kAnnunIce,             datarefs::kAnnunGearUnsafe,
    datarefs::kAnnunStallWarning,
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

// How often the nearby-feature list is rebuilt from the nav database. Ownship
// position updates every frame; the (range-filtered) feature scan is throttled.
constexpr double kMapRebuildIntervalSeconds = 1.0;
// Display range for the inset map, and the cap on features fed to the renderer.
constexpr float kMapRangeNm = 10.0f;
constexpr std::size_t kMaxMapFeatures = 250;
constexpr std::size_t kMaxMapAirspaces = 60;

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
                                   NavDataStore& navData, FmsPlanStore& fmsPlan,
                                   const TerrainSource* terrain,
                                   const ChecklistSource* checklists)
    : navData_(navData),
      fmsPlan_(fmsPlan),
      terrain_(terrain),
      checklists_(checklists),
      host_(std::move(host)),
      port_(port),
      webApi_(host_, XPlaneWebApi::kDefaultPort) {
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
      }
    }
    lastPacketSeconds_ = elapsedSeconds_;
    everConnected_ = true;
  }
}

void XPlaneConnection::update(double dtSeconds) {
  elapsedSeconds_ += dtSeconds;
  sinceResubscribeSeconds_ += dtSeconds;

  drainSocket();

  const ConnectionState state = connectionState();

  if (state == ConnectionState::Connected) {
    if (!primed_) {
      // First fresh data after (re)connecting: jump straight to it so the
      // gauges don't visibly ease in from their defaults / last-known values.
      data_ = target_;
      prevAirspeedKts_ = data_.airspeedKts;
      data_.airspeedTrendKts = 0.0f;
      data_.altitudeTrendFt = data_.verticalSpeedFpm * (6.0f / 60.0f);
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
      // is the filtered rate of change of the smoothed IAS.
      data_.altitudeTrendFt = data_.verticalSpeedFpm * (6.0f / 60.0f);
      if (dtSeconds > 1e-4) {
        const float instTrend = (data_.airspeedKts - prevAirspeedKts_) /
                                static_cast<float>(dtSeconds) * 6.0f;
        data_.airspeedTrendKts += (instTrend - data_.airspeedTrendKts) * alpha;
      }
      prevAirspeedKts_ = data_.airspeedKts;
    }
    updateFmaModes();
    // GPS flight phase (ENR/TERM/APR/OCN) is a discrete annunciation derived
    // from the live CDI sensitivity, so it is set straight through (no easing).
    data_.gpsFlightPhase = gpsPhaseFromSensitivity(gpsHdefNmPerDot_);
    // Active destination identifier comes from the Web API (string dataref).
    // When unavailable it is empty, which clears the placeholder rather than
    // showing a stale demo waypoint. X-Plane exposes no FROM-waypoint string
    // dataref, so the leg renders direct-to ("->KXXX").
    data_.fmaToWpt = webApi_.destinationId();
    data_.fmaFromWpt.clear();
    updateZuluClock(dtSeconds);
    updateMap(dtSeconds);
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

void XPlaneConnection::updateMap(double dtSeconds) {
  map_.rangeNm = kMapRangeNm;
  map_.terrain = terrain_;
  map_.positionValid = haveLat_ && haveLon_;
  if (!map_.positionValid) return;

  map_.ownshipLat = static_cast<double>(ownshipLatDeg_);
  map_.ownshipLon = static_cast<double>(ownshipLonDeg_);

  // Flight plan: parsed from an X-Plane .fms file (see FmsPlanStore). The live
  // FMS is not available over UDP, so this tracks the exported/loaded plan file
  // rather than in-cockpit edits until a plugin bridge exists.
  fmsPlan_.refreshIfChanged();
  if (fmsPlan_.loaded() && !fmsPlan_.flightPlan().empty()) {
    map_.flightPlan = fmsPlan_.flightPlan();
  }

  // Nearby navaids/fixes from the parsed nav database. Rebuild on a throttled
  // timer rather than every frame, since the database spans the whole world.
  sinceMapRebuildSeconds_ += dtSeconds;
  const bool due = map_.features.empty() ||
                   sinceMapRebuildSeconds_ >= kMapRebuildIntervalSeconds;
  if (due) {
    if (navData_.loaded()) {
      map_.features = navData_.nearby(map_.ownshipLat, map_.ownshipLon,
                                      map_.rangeNm, kMaxMapFeatures);
    }
    if (airspace_.loaded()) {
      map_.airspaces = airspace_.nearby(map_.ownshipLat, map_.ownshipLon,
                                        map_.rangeNm, kMaxMapAirspaces);
    }
    sinceMapRebuildSeconds_ = 0.0;
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

  // Vertical: the captured mode is green; only altitude hold carries the cyan
  // reference value (the preselected/captured altitude).
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
  } else if (mode(kApVerticalSpeed) == kApModeActive) {
    vertActive = "VS";
  } else if (mode(kApPitch) == kApModeActive) {
    vertActive = "PIT";
  }
  data_.fmaVerticalActive = vertActive;
  data_.fmaVerticalValue = vertValue;
  data_.fmaVerticalUnits = vertUnits;

  // Vertical armed: ALTS (altitude preselect) whenever altitude capture is
  // armed; an armed glideslope occupies the rightmost approach-armed slot.
  data_.fmaVerticalArmed =
      (mode(kApAltitudeArmed) != 0) ? "ALTS" : std::string();
  data_.fmaVerticalApproachArmed =
      (mode(kApGlideslope) == kApModeArmed) ? "GS" : std::string();
}

ConnectionState XPlaneConnection::connectionState() const {
  if (!everConnected_) return ConnectionState::Connecting;
  if (elapsedSeconds_ - lastPacketSeconds_ > kStaleTimeoutSeconds) {
    return ConnectionState::Disconnected;
  }
  return ConnectionState::Connected;
}

}  // namespace avionics
