#include "avionics/MockDataSource.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace avionics {
namespace {

constexpr double kKpaoLat = 37.4611;
constexpr double kKpaoLon = -122.1153;

// Build a tessellated circular airspace ring (radius in NM) for the demo map.
std::vector<GeoPoint> demoCircle(double lat, double lon, double radiusNm,
                                 int segments = 48) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kNmPerDeg = 60.0;
  const double cosLat = std::max(0.05, std::cos(lat * kPi / 180.0));
  std::vector<GeoPoint> ring;
  ring.reserve(static_cast<std::size_t>(segments) + 1);
  for (int i = 0; i <= segments; ++i) {
    const double a = 2.0 * kPi * (static_cast<double>(i) / segments);
    ring.push_back({lat + (radiusNm / kNmPerDeg) * std::cos(a),
                    lon + (radiusNm / (kNmPerDeg * cosLat)) * std::sin(a)});
  }
  return ring;
}

void seedDemoAirspace(MapData& map) {
  // Demo only: ceilings are set well above the mock cruise altitude so all
  // three rings stay visible through the altitude declutter.
  MapAirspace classC;
  classC.airspaceClass = AirspaceClass::ClassC;
  classC.name = "DEMO-C";
  classC.floorFt = 0.0f;
  classC.ceilingFt = 8400.0f;
  classC.boundary = demoCircle(kKpaoLat, kKpaoLon, 4.5);

  MapAirspace classD;
  classD.airspaceClass = AirspaceClass::ClassD;
  classD.name = "KSQL-D";
  classD.floorFt = 0.0f;
  classD.ceilingFt = 9000.0f;
  classD.boundary = demoCircle(37.5111, -122.2495, 3.0);

  MapAirspace classB;
  classB.airspaceClass = AirspaceClass::ClassB;
  classB.name = "DEMO-B";
  classB.floorFt = 0.0f;
  classB.ceilingFt = 10000.0f;
  classB.boundary = {
      {37.40, -122.26}, {37.55, -122.26}, {37.56, -122.02}, {37.41, -122.00},
  };

  map.airspaces = {classB, classC, classD};
}

void seedDemoMap(MapData& map) {
  map.ownshipLat = kKpaoLat;
  map.ownshipLon = kKpaoLon;
  map.rangeNm = 10.0f;
  map.positionValid = true;
  map.flightPlan = {
      {kKpaoLat, kKpaoLon, "KPAO"},
      {37.5930, -121.8810, "SUNOL"},
      {36.9357, -121.7896, "KWVI"},
  };
  map.features = {
      {MapFeatureType::Airport, 37.5111, -122.2495, "KSQL"},
      {MapFeatureType::Vor, 37.3925, -122.2808, "OSI"},
      {MapFeatureType::Ndb, 37.5100, -122.0300, "OAK"},
      {MapFeatureType::Fix, 37.4200, -122.0500, "MENLO"},
      {MapFeatureType::Fix, 37.5300, -122.1700, "EDDYY"},
      {MapFeatureType::Airport, 37.6213, -122.3790, "KSFO"},
  };
  seedDemoAirspace(map);
}

}  // namespace

void MockDataSource::update(double dtSeconds) {
  elapsedSeconds_ += dtSeconds;
  const float t = static_cast<float>(elapsedSeconds_);

  data_.airspeedKts = 110.0f + 15.0f * std::sin(t * 0.20f);
  data_.altitudeFt = 5500.0f + 250.0f * std::sin(t * 0.10f);
  data_.headingDeg = std::fmod(45.0f + t * 2.0f, 360.0f);
  data_.pitchDeg = 6.0f * std::sin(t * 0.35f);
  data_.rollDeg = 20.0f * std::sin(t * 0.25f);
  data_.verticalSpeedFpm = 500.0f * std::sin(t * 0.10f) * 0.10f * 60.0f;
  data_.slipSkidDeg = 2.0f * std::sin(t * 0.5f);

  // Turbulence: sum a few incommensurate high-frequency sinusoids per axis so
  // the bumps look chaotic rather than periodic, then add them to the smooth
  // base states. kTurbulence scales the overall intensity (0 = calm air).
  constexpr float kTurbulence = 1.0f;
  auto bump = [&](float a, float b, float c) {
    return std::sin(t * a) + 0.6f * std::sin(t * b) + 0.35f * std::sin(t * c);
  };
  data_.pitchDeg += kTurbulence * 1.3f * bump(3.3f, 7.9f, 17.3f);
  data_.rollDeg += kTurbulence * 2.6f * bump(2.9f, 6.7f, 13.1f);
  data_.airspeedKts += kTurbulence * 2.2f * bump(2.3f, 5.7f, 11.9f);
  data_.verticalSpeedFpm += kTurbulence * 210.0f * bump(3.1f, 6.3f, 12.7f);
  data_.slipSkidDeg += kTurbulence * 0.7f * bump(4.1f, 9.3f, 15.1f);
  data_.altitudeFt += kTurbulence * 16.0f * bump(2.7f, 5.3f, 10.1f);
  data_.headingDeg = std::fmod(
      data_.headingDeg + kTurbulence * 1.0f * bump(2.1f, 4.9f, 9.7f) + 360.0f,
      360.0f);

  data_.tasKts = data_.airspeedKts + 4.0f;
  data_.groundSpeedKts = data_.airspeedKts - 2.0f;
  data_.oatCelsius = 12.0f + 3.0f * std::sin(t * 0.05f);

  // Turn rate (deg/sec) is the analytic derivative of the smooth heading sweep
  // (45 + t*2.0 -> 2.0 deg/sec base) modulated by the bank, so the HSI turn-rate
  // trend tracks the roll. Track lags heading by a small wind-driven crab angle.
  data_.turnRateDegPerSec = 2.0f + 0.05f * data_.rollDeg;
  data_.trackDeg =
      std::fmod(data_.headingDeg + 2.0f + 1.5f * std::sin(t * 0.07f) + 360.0f,
                360.0f);

  // Trend vectors: airspeed = d/dt of its sinusoid over 6 s; altitude tracks
  // the VSI (6-second projection) so the trend and VSI agree.
  data_.airspeedTrendKts = 18.0f * std::cos(t * 0.20f);
  data_.altitudeTrendFt = data_.verticalSpeedFpm * (6.0f / 60.0f);

  // HSI course + CDI: hold a fixed course while the lateral deviation drifts.
  data_.courseDeg = 45.0f;
  data_.cdiDeviationDots = 1.3f * std::sin(t * 0.13f);
  data_.cdiToFlag = true;
  data_.navSignalValid = true;
  // Enroute leg (well outside terminal airspace), so the GPS annunciates ENR.
  data_.gpsFlightPhase = "ENR";

  // Flight director command (slightly off the current attitude so the magenta
  // bars are visibly displaced from the aircraft symbol).
  data_.flightDirectorActive = true;
  data_.fdRollDeg = 12.0f * std::sin(t * 0.20f + 0.6f);
  data_.fdPitchDeg = 4.0f + 3.0f * std::sin(t * 0.30f + 0.4f);

  // Wind and bearing pointers (bearings rotate with the aircraft heading).
  data_.windValid = true;
  data_.windDirectionDeg =
      std::fmod(280.0f + 12.0f * std::sin(t * 0.04f) + 360.0f, 360.0f);
  data_.windSpeedKts = 14.0f + 4.0f * std::sin(t * 0.06f);
  data_.bearing1Valid = true;
  data_.bearing1Deg = std::fmod(data_.headingDeg + 65.0f, 360.0f);
  data_.bearing1DistanceNm = 12.4f;
  data_.bearing2Valid = true;
  data_.bearing2Deg = std::fmod(data_.headingDeg - 40.0f + 360.0f, 360.0f);
  data_.bearing2DistanceNm = 23.7f;
  data_.fmaLegDistanceNm = 12.4f - t * 0.02f;
  data_.fmaLegBearingDeg =
      std::fmod(data_.headingDeg + 15.0f + t * 0.5f, 360.0f);
  data_.transponderCode = 1200;
  data_.transponderMode = "ALT";
  data_.transponderReply = std::fmod(t, 9.0) < 0.4f;

  // Demo CAS conditions so the Alerts window shows live messages in the mock
  // build (driven by the real annunciator datarefs when connected to X-Plane).
  // A mix of cautions (amber) and warnings (red) on different cycles also
  // exercises the severity sorting -- warnings always sort above cautions.
  data_.casLowVacuum = std::fmod(t, 20.0f) > 6.0f;          // steady caution
  data_.casFuelLow = std::fmod(t, 30.0f) > 18.0f;           // intermittent
  data_.casPitotHeatOff = std::fmod(t, 16.0f) > 9.0f;       // caution
  data_.casLowVoltage = std::fmod(t, 24.0f) > 16.0f;        // warning (red)
  data_.casOilPressureLow = std::fmod(t, 40.0f) > 33.0f;    // warning (red)
  data_.fmaVerticalValue = static_cast<int>(std::lround(data_.selectedAltitudeFt));
  data_.timerSeconds = static_cast<int>(t) % 36000;
  const int totalSec = 18 * 3600 + static_cast<int>(t) % 86400;
  data_.utcHour = (totalSec / 3600) % 24;
  data_.utcMinute = (totalSec / 60) % 60;
  data_.utcSecond = totalSec % 60;

  if (map_.flightPlan.empty()) seedDemoMap(map_);

  // Drift ownship along track for a live inset map in mock mode.
  const double hdgRad =
      static_cast<double>(data_.headingDeg) * 3.14159265358979323846 / 180.0;
  const double nm = static_cast<double>(data_.groundSpeedKts) * dtSeconds / 3600.0;
  map_.ownshipLat += nm * std::cos(hdgRad) / 60.0;
  map_.ownshipLon +=
      nm * std::sin(hdgRad) / (60.0 * std::cos(kKpaoLat * 3.14159265358979323846 / 180.0));
}

}  // namespace avionics
