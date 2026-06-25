#include "avionics/MockDataSource.h"

#include "avionics/Eis.h"
#include "avionics/EisLegacy.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "avionics/GpsLegCourse.h"
#include "avionics/NavMath.h"
#include "avionics/NavigationComputer.h"
#include "avionics/MapRange.h"

namespace avionics {
namespace {

constexpr double kKfmyLat = 26.5862;
constexpr double kKfmyLon = -81.8632;

// Runway 31 threshold at Page Field (KFMY) for the on-ground mock mode. The
// threshold position and true runway heading are the published FAA values
// (26-34.988253N / 081-51.406542W, 310 true); the field elevation matches the
// airport's surveyed elevation.
constexpr double kKfmyRwy31ThresholdLat = 26.583138;
constexpr double kKfmyRwy31ThresholdLon = -81.856776;
constexpr float kKfmyRwy31HeadingDeg = 310.0f;
constexpr float kKfmyFieldElevationFt = 17.0f;

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

void seedDemoAirspace(MapData& map, double lat, double lon) {
  // Demo only: rings are placed relative to the given center (the route
  // start) so they stay in view whatever route is flown, and ceilings are set
  // well above the mock cruise altitude so all three stay visible through the
  // altitude declutter.
  MapAirspace classC;
  classC.airspaceClass = AirspaceClass::ClassC;
  classC.name = "DEMO-C";
  classC.floorFt = 0.0f;
  classC.ceilingFt = 8400.0f;
  classC.boundary = demoCircle(lat, lon, 4.5);

  MapAirspace classD;
  classD.airspaceClass = AirspaceClass::ClassD;
  classD.name = "DEMO-D";
  classD.floorFt = 0.0f;
  classD.ceilingFt = 9000.0f;
  classD.boundary = demoCircle(lat + 0.05, lon - 0.134, 3.0);

  MapAirspace classB;
  classB.airspaceClass = AirspaceClass::ClassB;
  classB.name = "DEMO-B";
  classB.floorFt = 0.0f;
  classB.ceilingFt = 10000.0f;
  classB.boundary = {
      {lat - 0.061, lon - 0.145},
      {lat + 0.089, lon - 0.145},
      {lat + 0.099, lon + 0.095},
      {lat - 0.051, lon + 0.115},
  };

  MapAirspace moa;
  moa.airspaceClass = AirspaceClass::MOA;
  moa.name = "DEMO MOA";
  moa.floorFt = 0.0f;
  moa.ceilingFt = 18000.0f;
  moa.boundary = demoCircle(lat - 0.08, lon + 0.12, 6.0);

  MapAirspace alert;
  alert.airspaceClass = AirspaceClass::Alert;
  alert.name = "DEMO ALERT";
  alert.floorFt = 0.0f;
  alert.ceilingFt = 5000.0f;
  alert.boundary = demoCircle(lat + 0.11, lon + 0.08, 2.5);

  MapAirspace restricted;
  restricted.airspaceClass = AirspaceClass::Restricted;
  restricted.name = "R-2901A";
  restricted.floorFt = 0.0f;
  restricted.ceilingFt = 12000.0f;
  restricted.boundary = {
      {lat - 0.14, lon + 0.02}, {lat - 0.05, lon + 0.02},
      {lat - 0.05, lon + 0.16}, {lat - 0.14, lon + 0.16},
  };

  map.airspaces = {classB, classC, classD, moa, alert, restricted};
}

// Built-in route flown when the shell does not supply a real flight plan.
// A short loop in Southwest Florida, sized to frame nicely at the default 10 NM
// map range, so the demo still looks plausible. The demo starts on a GPS
// Direct-To straight to KRSW (the first leg), so the PFD top bar shows the
// Direct-To icon -> KRSW (see ensureRoute()).
const std::vector<MapLeg>& defaultRoute() {
  static const std::vector<MapLeg> kRoute = {
      {26.5362, -81.7552, "KRSW"},
      // ESTRO carries a demo VNAV altitude constraint so the Active VNV Profile
      // box and PFD vertical deviation have something to track in the standalone
      // demo (the field is designated -> drawn cyan).
      {26.4700, -81.8100, "ESTRO", 3000, AltConstraintType::At, true},
      {26.6700, -81.9500, "CCRAL"},
  };
  return kRoute;
}

// Hand-placed nav features used only when no real NavFeatureSource is wired in.
void seedDemoFeatures(MapData& map) {
  MapFeature kfmy;
  kfmy.type = MapFeatureType::Airport;
  kfmy.lat = kKfmyLat;
  kfmy.lon = kKfmyLon;
  kfmy.id = "KFMY";
  kfmy.name = "Page Field";
  kfmy.city = "Ft Myers";
  kfmy.region = "FL";
  kfmy.airportTowered = true;
  kfmy.airportServiced = true;

  MapFeature krsw;
  krsw.type = MapFeatureType::Airport;
  krsw.lat = 26.5362;
  krsw.lon = -81.7552;
  krsw.id = "KRSW";
  krsw.name = "Southwest Florida Intl";
  krsw.city = "Ft Myers";
  krsw.region = "FL";
  krsw.airportTowered = true;
  krsw.airportServiced = true;

  MapFeature kpgd;
  kpgd.type = MapFeatureType::Airport;
  kpgd.lat = 26.9202;
  kpgd.lon = -81.9906;
  kpgd.id = "KPGD";
  kpgd.airportTowered = false;
  kpgd.airportServiced = false;

  map.features = {
      kfmy,
      krsw,
      {MapFeatureType::Vor, 26.5806, -81.8714, "RSW"},
      {MapFeatureType::Ndb, 26.5850, -81.8650, "FM"},
      {MapFeatureType::Fix, 26.4700, -81.8100, "ESTRO"},
      {MapFeatureType::Fix, 26.6700, -81.9500, "CCRAL"},
      kpgd,
  };
}

// Synthetic overlay layers (airways, runways, land data, obstacles) around the
// demo route. The mock feed has no offline source for these databases, so they
// are always seeded; without them the AWY softkey, runway diagrams, land
// styling, and obstacle symbols could never be exercised offline.
void seedDemoOverlays(MapData& map, double lat, double lon) {
  map.airways = {
      {AirwayLevel::Low, "V521", {lat - 0.9, lon - 0.45}, {lat + 0.1, lon - 0.1}},
      {AirwayLevel::Low, "V521", {lat + 0.1, lon - 0.1}, {lat + 1.1, lon + 0.3}},
      {AirwayLevel::Low, "V157", {lat - 0.7, lon + 0.5}, {lat + 0.2, lon + 0.05}},
      {AirwayLevel::Low, "V157", {lat + 0.2, lon + 0.05}, {lat + 1.0, lon - 0.5}},
      {AirwayLevel::High, "J79", {lat - 1.2, lon + 0.2}, {lat + 1.4, lon + 0.6}},
  };

  // KFMY 5/23 and KRSW 6/24, close to their published positions so the
  // diagrams sit on the airport symbols at low range.
  map.runways = {
      {{26.5800, -81.8700}, {26.5930, -81.8565}, 46.0f},
      {{26.5245, -81.7610}, {26.5430, -81.7415}, 46.0f},
  };

  // The Caloosahatchee river through Fort Myers, plus an I-75-like highway and
  // a county-border-like line, so each land style is drawn.
  MapLandLine river;
  river.landClass = LandClass::River;
  river.points = {{lat + 0.12, lon - 0.30}, {lat + 0.06, lon - 0.16},
                  {lat + 0.075, lon - 0.05}, {lat + 0.13, lon + 0.06},
                  {lat + 0.20, lon + 0.14},  {lat + 0.30, lon + 0.22}};
  MapLandLine road;
  road.landClass = LandClass::Road;
  road.points = {{lat - 0.9, lon + 0.085}, {lat - 0.3, lon + 0.08},
                 {lat + 0.3, lon + 0.10},  {lat + 0.9, lon + 0.16}};
  MapLandLine border;
  border.landClass = LandClass::Border;
  border.points = {{lat + 0.255, lon - 0.9}, {lat + 0.255, lon + 0.0},
                   {lat + 0.32, lon + 0.9}};
  map.landLines = {river, road, border};

  map.cities = {
      {"FORT MYERS", lat + 0.055, lon + 0.005, 3},
      {"CAPE CORAL", lat - 0.015, lon - 0.125, 2},
      {"LEHIGH ACRES", lat + 0.03, lon + 0.24, 1},
  };

  map.obstacles = {
      {lat - 0.04, lon + 0.06, 1549.0f, 520.0f, true, false, false, 1},
      {lat + 0.09, lon - 0.04, 360.0f, 340.0f, false, false, false, 1},
      {lat + 0.05, lon + 0.02, 120.0f, 60.0f, true, false, false, 1},
      {lat + 0.07, lon - 0.01, 280.0f, 250.0f, true, false, false, 2},
      {lat + 0.03, lon - 0.05, 80.0f, 75.0f, false, false, true, 1},
  };
}

// Map layers are pulled within this radius (and capped) so both the small PFD
// inset and a zoomed-out MFD MAP page have data to draw, refreshed on a timer
// since the world databases are large. Mirrors the live X-Plane feed's query
// envelope so both feeds show the same nearby data.
constexpr float kFeatureQueryRangeNm = 160.0f;
constexpr std::size_t kMaxFeatures = 500;
constexpr double kFeatureRebuildIntervalSeconds = 2.0;
constexpr float kAirspaceQueryRangeNm = 160.0f;
constexpr std::size_t kMaxAirspaces = 60;
constexpr float kAirwayQueryRangeNm = 160.0f;
constexpr std::size_t kMaxAirways = 500;
constexpr float kRunwayQueryRangeNm = 30.0f;
constexpr std::size_t kMaxRunways = 120;
constexpr float kTaxiwayQueryRangeNm = 10.0f;
constexpr std::size_t kMaxTaxiways = 600;
constexpr std::size_t kMaxLandLines = 8000;
constexpr std::size_t kMaxCities = 600;
constexpr float kObstacleQueryRangeNm = 30.0f;
constexpr std::size_t kMaxObstacles = 300;

void applyReversionaryAlertsDemo(FlightData& data) {
  data.attitudeValid = false;
  data.headingValid = false;
  data.airspeedValid = false;
  data.altitudeValid = false;
  data.verticalSpeedValid = false;
  data.navSignalValid = false;
}

}  // namespace

void MockDataSource::update(double dtSeconds) {
  if (groundMode_) {
    updateOnGround(dtSeconds);
    if (reversionaryAlertsDemo_) applyReversionaryAlertsDemo(data_);
    return;
  }

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
  // base states. kTurbulence scales the overall intensity (0 = calm air); the
  // bumps are only applied when the user opts into simulated turbulence.
  const float kTurbulence = turbulenceEnabled_ ? 1.0f : 0.0f;
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

  // Fly the route: this sets ownship position, heading, course and the FMA
  // active-leg fields, so the heading-dependent fields below follow the route.
  ensureRoute();
  navigateRoute(dtSeconds);

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

  // HSI course follows the active leg (set in navigateRoute); the lateral
  // deviation drifts gently so the CDI shows some life.
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
  data_.bearing1Source = "GPS";
  data_.bearing1Ident = "OSI";
  data_.bearing2Valid = true;
  data_.bearing2Deg = std::fmod(data_.headingDeg - 40.0f + 360.0f, 360.0f);
  data_.bearing2DistanceNm = 23.7f;
  data_.bearing2Source = "VOR2";
  data_.bearing2Ident = "SFO";

  // Decoded idents of the active NAV stations, shown beside their frequencies.
  data_.nav1Ident = "SAU";
  data_.nav2Ident = "PYE";

  // Vertical deviation: demo a GPS Glidepath that drifts gently around centre
  // (magenta diamond), plus the matching VNAV required-VS chevron on the VSI.
  data_.vdiKind = VerticalDeviationKind::Glidepath;
  data_.vdiValid = true;
  data_.vdiDeviationDots = 1.2f * std::sin(t * 0.12f);
  data_.requiredVsValid = true;
  data_.requiredVsFpm = -700.0f + 200.0f * std::sin(t * 0.09f);

  // Autopilot Selected Vertical Speed reference (cyan VSI bug).
  data_.selectedVsValid = true;
  data_.selectedVerticalSpeedFpm = -500.0f;

  // Marker beacons: sweep through outer -> middle -> inner so each annunciator
  // colour is exercised in the demo.
  const float markerPhase = std::fmod(t, 24.0f);
  data_.markerBeacon = markerPhase < 3.0f    ? MarkerBeacon::Outer
                       : markerPhase < 6.0f  ? MarkerBeacon::Middle
                       : markerPhase < 9.0f  ? MarkerBeacon::Inner
                                             : MarkerBeacon::None;

  // DME information window.
  data_.dmeValid = true;
  data_.dmeMode = "NAV1";
  data_.dmeFreqMhz = 113.70f;
  data_.dmeDistanceNm = 18.2f + 2.0f * std::sin(t * 0.08f);

  // Baro transition alert flashes intermittently (as it would crossing the
  // transition altitude/level).
  data_.baroTransitionAlert = std::fmod(t, 28.0f) > 22.0f;

  data_.transponderCode = 1200;
  data_.transponderMode = "ALT";
  data_.transponderReply = std::fmod(t, 9.0) < 0.4f;

  // Demo CAS conditions so the Alerts window shows live messages in the mock
  // build (driven by the real annunciator datarefs when connected to X-Plane).
  // A mix of cautions (amber) and warnings (red) on different cycles also
  // exercises the severity sorting -- warnings always sort above cautions.
  // The whole set is gated by casMessagesEnabled_ so the demo can be flown
  // without the periodic alerts (a Debug menu toggle in the standalone shell).
  const bool cas = casMessagesEnabled_;
  data_.casLowVacuum = cas && std::fmod(t, 20.0f) > 6.0f;    // steady caution
  data_.casFuelLow = cas && std::fmod(t, 30.0f) > 18.0f;     // intermittent
  data_.casPitotHeatOff = cas && std::fmod(t, 16.0f) > 9.0f; // caution
  data_.casLowVoltage = cas && std::fmod(t, 24.0f) > 16.0f;  // warning (red)
  data_.casOilPressureLow =
      cas && std::fmod(t, 40.0f) > 33.0f;                    // warning (red)
  // EIS engine strip: cruise-power values with gentle drift so the gauges show
  // life. Fuel burns down at the indicated fuel flow; the ammeter shows a
  // small charging load.
  data_.engineRpm = 2400.0f + 35.0f * std::sin(t * 0.18f) +
                    8.0f * std::sin(t * 1.7f);
  data_.fuelFlowGph = 9.8f + 0.5f * std::sin(t * 0.11f);
  data_.oilPressurePsi = 61.0f + 2.0f * std::sin(t * 0.07f);
  data_.oilTempDegF = 181.0f + 4.0f * std::sin(t * 0.03f);
  data_.egtDegF = 1448.0f + 18.0f * std::sin(t * 0.09f);
  data_.vacuumInHg = 4.9f + 0.15f * std::sin(t * 0.13f);
  const float burnedGal =
      data_.fuelFlowGph * static_cast<float>(elapsedSeconds_) / 3600.0f;
  data_.fuelQtyLeftGal = std::max(0.0f, 21.5f - burnedGal * 0.5f);
  data_.fuelQtyRightGal = std::max(0.0f, 22.5f - burnedGal * 0.5f);
  data_.engineHours = 1234.5f;
  data_.busVoltsMain = 27.9f + 0.1f * std::sin(t * 0.21f);
  data_.busVoltsEssential = 27.8f + 0.1f * std::sin(t * 0.19f);
  data_.battAmpsMain = 2.0f + 1.0f * std::sin(t * 0.15f);
  data_.battAmpsStandby = 0.0f;

  publishEisChannels();
  advanceClockFields();

  refreshFeatures(dtSeconds);

  // Demo traffic orbits ownship so targets stay on the map as the route is
  // flown: one proximate (open diamond) and one close advisory (yellow TA).
  {
    const float orbit = t * 0.05f;
    MapTraffic prox;
    prox.lat = map_.ownshipLat + 0.06 * std::cos(orbit);
    prox.lon = map_.ownshipLon + 0.07 * std::sin(orbit);
    prox.relAltFt = 700.0f + 200.0f * std::sin(t * 0.08f);
    prox.verticalSpeedFpm = 600.0f * std::sin(t * 0.08f);
    prox.trafficAdvisory = false;

    MapTraffic ta;
    ta.lat = map_.ownshipLat - 0.012 * std::cos(orbit * 1.7f);
    ta.lon = map_.ownshipLon + 0.012 * std::sin(orbit * 1.7f);
    ta.relAltFt = -300.0f;
    ta.verticalSpeedFpm = -400.0f;
    ta.trafficAdvisory = true;

    map_.traffic = {prox, ta};
  }

  publishMapBackground(dtSeconds);
  if (reversionaryAlertsDemo_) applyReversionaryAlertsDemo(data_);
  syncDisplayBackup(data_, dtSeconds);
}

void MockDataSource::updateOnGround(double dtSeconds) {
  elapsedSeconds_ += dtSeconds;

  // Parked on KFMY runway 31 with the engine idling and the brakes set: every
  // dynamic state is pinned to its stationary value so the PFD shows a
  // believable on-ground picture (zero speeds, wings level, field elevation).
  data_.airspeedKts = 0.0f;
  data_.altitudeFt = kKfmyFieldElevationFt;
  data_.pitchDeg = 0.0f;
  data_.rollDeg = 0.0f;
  data_.verticalSpeedFpm = 0.0f;
  data_.slipSkidDeg = 0.0f;
  data_.tasKts = 0.0f;
  data_.groundSpeedKts = 0.0f;
  data_.oatCelsius = 28.0f;  // typical Southwest Florida ramp temperature
  data_.turnRateDegPerSec = 0.0f;
  data_.airspeedTrendKts = 0.0f;
  data_.altitudeTrendFt = 0.0f;

  // Seed the flight plan / nearby nav data, then pin the position to the
  // runway 31 threshold and the heading down the runway (ensureRoute starts
  // the aircraft at the airport reference point pointed at the first leg, so
  // these must be set after it on the first frame).
  ensureRoute();
  data_.headingDeg = kKfmyRwy31HeadingDeg;
  data_.trackDeg = kKfmyRwy31HeadingDeg;
  map_.ownshipLat = kKfmyRwy31ThresholdLat;
  map_.ownshipLon = kKfmyRwy31ThresholdLon;
  map_.positionValid = true;
  map_.directToActive = false;
  map_.traffic.clear();  // no orbiting demo traffic while parked

  // Sitting in the departure runway environment: GPS in terminal mode with no
  // active-leg guidance, and lateral/vertical deviation, flight director and
  // autopilot references all stowed.
  data_.gpsFlightPhase = "TERM";
  data_.cdiDeviationDots = 0.0f;
  data_.cdiToFlag = false;
  data_.navSignalValid = false;
  data_.flightDirectorActive = false;
  data_.fmaFromWpt.clear();
  data_.fmaToWpt.clear();
  data_.fmaLegDistanceNm = 0.0f;
  data_.fmaLegBearingDeg = 0.0f;
  data_.vdiKind = VerticalDeviationKind::None;
  data_.vdiValid = false;
  data_.requiredVsValid = false;
  data_.selectedVsValid = false;

  // Surface wind still shows, but the nav bearing pointers, marker beacon and
  // DME have nothing to track on the ground.
  data_.windValid = true;
  data_.windDirectionDeg = 290.0f;
  data_.windSpeedKts = 7.0f;
  data_.bearing1Valid = false;
  data_.bearing2Valid = false;
  data_.markerBeacon = MarkerBeacon::None;
  data_.dmeValid = false;
  data_.baroTransitionAlert = false;

  data_.transponderCode = 1200;
  data_.transponderMode = "GND";
  data_.transponderReply = false;

  // A healthy idling aircraft raises no CAS messages.
  data_.casLowVacuum = false;
  data_.casFuelLow = false;
  data_.casPitotHeatOff = false;
  data_.casLowVoltage = false;
  data_.casOilPressureLow = false;

  // Engine idling: low RPM and fuel flow, oil pressure / temperature in the
  // normal idle band, alternator online and charging the battery.
  data_.engineRpm = 1000.0f;
  data_.fuelFlowGph = 2.6f;
  data_.oilPressurePsi = 45.0f;
  data_.oilTempDegF = 120.0f;
  data_.egtDegF = 1150.0f;
  data_.vacuumInHg = 5.0f;
  data_.fuelQtyLeftGal = 21.5f;
  data_.fuelQtyRightGal = 22.5f;
  data_.engineHours = 1234.5f;
  data_.busVoltsMain = 27.9f;
  data_.busVoltsEssential = 27.8f;
  data_.battAmpsMain = 2.0f;
  data_.battAmpsStandby = 0.0f;

  publishEisChannels();
  advanceClockFields();

  refreshFeatures(dtSeconds);
  publishMapBackground(dtSeconds);
  syncDisplayBackup(data_, dtSeconds);
}

void MockDataSource::publishEisChannels() {
  data_.eisChannels[eis_channels::kEngRpm] = data_.engineRpm;
  data_.eisChannels[eis_channels::kFuelFlow] = data_.fuelFlowGph;
  data_.eisChannels[eis_channels::kOilPres] = data_.oilPressurePsi;
  data_.eisChannels[eis_channels::kOilTemp] = data_.oilTempDegF;
  data_.eisChannels[eis_channels::kEgt] = data_.egtDegF;
  data_.eisChannels[eis_channels::kVacuum] = data_.vacuumInHg;
  data_.eisChannels[eis_channels::kFuelQtyLeft] = data_.fuelQtyLeftGal;
  data_.eisChannels[eis_channels::kFuelQtyRight] = data_.fuelQtyRightGal;
  data_.eisChannels[eis_channels::kEngHours] = data_.engineHours;
  data_.eisChannels[eis_channels::kBusVoltsMain] = data_.busVoltsMain;
  data_.eisChannels[eis_channels::kBusVoltsEss] = data_.busVoltsEssential;
  data_.eisChannels[eis_channels::kBattAmpsMain] = data_.battAmpsMain;
  data_.eisChannels[eis_channels::kBattAmpsStandby] = data_.battAmpsStandby;

  // When the active aircraft profile is the turbofan (Cirrus Vision SF50), also
  // publish believable turbine + synoptic channel values so the jet EIS page
  // shows live gauges in the demo/screenshot feed (the real plugin reads these
  // from the sim datarefs declared in sf50.eis). Values mirror Fig. 3-2.
  if (eisSource_ != nullptr && eisSource_->ready() &&
      eisSource_->layout().style == EisStripStyle::Turbofan) {
    const float t = static_cast<float>(elapsedSeconds_);
    const float n1 = groundMode_ ? 30.0f : 85.0f + 1.5f * std::sin(t * 0.2f);
    data_.eisChannels[eis_channels::kThrustPct] = groundMode_ ? 8.0f : 69.0f;
    data_.eisChannels[eis_channels::kN1Pct] = n1;
    data_.eisChannels[eis_channels::kN2Pct] = groundMode_ ? 55.0f : 75.0f;
    data_.eisChannels[eis_channels::kIttC] = groundMode_ ? 380.0f : 500.0f;
    data_.eisChannels[eis_channels::kOilTempC] = 41.0f;
    data_.eisChannels[eis_channels::kFuelTempC] = 20.0f;
    data_.eisChannels[eis_channels::kFuelQtyLeft] = 150.0f;
    data_.eisChannels[eis_channels::kFuelQtyRight] = 150.0f;
    data_.eisChannels[eis_channels::kFuelFlow] = groundMode_ ? 12.0f : 78.0f;
    data_.eisChannels[eis_channels::kOilPres] = 50.0f;
    data_.eisChannels[eis_channels::kEmerBusVolts] = 28.1f;
    data_.eisChannels[eis_channels::kBatt1Amps] = 49.0f;
    data_.eisChannels[eis_channels::kBatt2Amps] = 10.0f;
    data_.eisChannels[eis_channels::kGen1Amps] = 200.0f;
    data_.eisChannels[eis_channels::kGen2Amps] = 49.0f;
    // Airframe synoptics: gear up in cruise / down on the ground, takeoff trim,
    // 50% flaps, and a small cabin climb.
    const float gear = groundMode_ ? 1.0f : 0.0f;
    data_.eisChannels[eis_channels::kGearNose] = gear;
    data_.eisChannels[eis_channels::kGearLeft] = gear;
    data_.eisChannels[eis_channels::kGearRight] = gear;
    data_.eisChannels[eis_channels::kPitchTrim] = 0.5f;
    data_.eisChannels[eis_channels::kRollTrim] = 0.0f;
    data_.eisChannels[eis_channels::kFlapsActual] = groundMode_ ? 0.5f : 0.0f;
    data_.eisChannels[eis_channels::kFlapsCommanded] =
        groundMode_ ? 0.5f : 0.0f;
    data_.eisChannels[eis_channels::kCabinRateFpm] = groundMode_ ? 0.0f : 300.0f;
    data_.eisChannels[eis_channels::kCabinAltFt] = groundMode_ ? 0.0f : 3000.0f;
    data_.eisChannels[eis_channels::kCabinDiffPsi] = groundMode_ ? 0.1f : 3.0f;
    data_.eisChannels[eis_channels::kDestElevFt] = 13000.0f;
  }

  syncEisLegacyFields(data_);
}

void MockDataSource::advanceClockFields() {
  const float t = static_cast<float>(elapsedSeconds_);
  data_.timerSeconds = static_cast<int>(t) % 36000;
  const int totalSec = 18 * 3600 + static_cast<int>(t) % 86400;
  data_.utcHour = (totalSec / 3600) % 24;
  data_.utcMinute = (totalSec / 60) % 60;
  data_.utcSecond = totalSec % 60;
  // Fixed mid-June date for the Trip Planning sunrise/sunset rows (the mock
  // clock starts at 18:00 UTC; any real date works, a constant keeps
  // screenshots deterministic).
  data_.utcDayOfYear = 167;
}

void MockDataSource::publishMapBackground(double dtSeconds) {
  map_.terrain = terrainSource_ != nullptr ? terrainSource_ : &terrain_;
  if (weatherSource_ != nullptr) {
    map_.weather = weatherSource_;
  } else {
    weather_.advance(dtSeconds);
    map_.weather = &weather_;
  }

  // Live datalink NEXRAD overlay, centered on the aircraft -- real ground radar
  // even on the mock feed. Pointed at the source unconditionally so the overlay
  // shows real weather (or nothing until tiles load / when offline) rather than
  // a synthetic field. The procedural weather_ above remains only for the
  // dedicated onboard Weather Radar page.
  if (map_.positionValid) {
    nexrad_.setCenter(map_.ownshipLat, map_.ownshipLon);
  }
  nexrad_.advance(dtSeconds);
  map_.nexrad = &nexrad_;
}

void MockDataSource::setRoute(std::vector<MapLeg> route) {
  route_ = std::move(route);
  routeInitialized_ = false;  // re-seed position from the new route start
}

void MockDataSource::updateRoute(std::vector<MapLeg> route) {
  if (!routeInitialized_) {
    // Nothing is flying yet; treat the edit as the initial plan.
    setRoute(std::move(route));
    return;
  }

  const std::string target =
      (route_.size() >= 2 && legIndex_ < route_.size()) ? route_[legIndex_].id
                                                        : std::string();
  route_ = std::move(route);
  map_.flightPlan = route_;

  if (route_.size() < 2) {
    // Plan deleted (or down to one waypoint): navigation is suspended, so the
    // active-leg readouts go away with it.
    data_.fmaFromWpt.clear();
    data_.fmaToWpt.clear();
    data_.fmaLegDistanceNm = 0.0f;
    data_.fmaLegBearingDeg = 0.0f;
    legIndex_ = 1;
    return;
  }

  // Keep flying toward the same waypoint when the edit kept it; otherwise
  // start the new plan from its first leg.
  legIndex_ = 1;
  for (std::size_t i = 1; i < route_.size(); ++i) {
    if (route_[i].id == target) {
      legIndex_ = i;
      break;
    }
  }
}

void MockDataSource::ensureRoute() {
  if (routeInitialized_) return;

  bool usingDefault = false;
  if (route_.size() < 2) {
    route_ = defaultRoute();
    usingDefault = true;
  }
  map_.flightPlan = route_;
  map_.rangeNm = 10.0f;
  if (!navigationPinned_) {
    legIndex_ = 1;
  }

  if (usingDefault) {
    // Built-in demo: start near Page Field and fly a GPS Direct-To straight to
    // KRSW (the first leg) so the PFD top bar shows the Direct-To icon -> KRSW
    // with no FROM waypoint. KFMY is only the geographic start, not a leg.
    map_.ownshipLat = kKfmyLat;
    map_.ownshipLon = kKfmyLon;
    directToActive_ = true;
    directToTarget_ = route_.front();  // KRSW
    map_.directToOriginValid = true;
    map_.directToOriginLat = kKfmyLat;
    map_.directToOriginLon = kKfmyLon;
    data_.headingDeg = static_cast<float>(navBearingDeg(
        kKfmyLat, kKfmyLon, route_.front().lat, route_.front().lon));
  } else if (!navigationPinned_ || !map_.positionValid) {
    map_.ownshipLat = route_.front().lat;
    map_.ownshipLon = route_.front().lon;
    data_.headingDeg = static_cast<float>(navBearingDeg(
        route_[0].lat, route_[0].lon, route_[1].lat, route_[1].lon));
  }
  if (!map_.positionValid) {
    map_.positionValid = true;
  }

  // Hand-placed demo nav data is used ONLY when no real nav source is wired in
  // (e.g. a bare unit test). When a source is set -- which the standalone shell
  // always does -- refreshFeatures fills in the actual nearby features,
  // airspaces, airways, runways, land vectors, and obstacles from the X-Plane
  // databases, so the mock never fabricates navigation data.
  if (navFeatures_ == nullptr) {
    seedDemoFeatures(map_);
    seedDemoAirspace(map_, map_.ownshipLat, map_.ownshipLon);
    seedDemoOverlays(map_, map_.ownshipLat, map_.ownshipLon);
  }

  // Demo database currency: pretend the current AIRAC cycle is installed so
  // the power-up page and AUX status look like a freshly updated unit. With a
  // real nav source wired in, refreshFeatures swaps in the actual cycle info
  // (including the amber expired state when the installed data is stale).
  map_.navDatabase = navDatabaseInfoForCycle(currentAiracCycle());

  routeInitialized_ = true;
  if (navigationPinned_) {
    syncFmaFromLegIndex();
  }
}

void MockDataSource::directTo(MapLeg target) {
  directToActive_ = true;
  directToTarget_ = std::move(target);
  if (!map_.flightPlan.empty()) {
    applyInPlanDirectToRouteSlice(map_, map_.flightPlan, directToTarget_);
  }
  routeInitialized_ = true;
  if (map_.positionValid) {
    map_.directToOriginValid = true;
    map_.directToOriginLat = map_.ownshipLat;
    map_.directToOriginLon = map_.ownshipLon;
  }
}

void MockDataSource::restoreDirectTo(MapLeg target, double originLat,
                                     double originLon, bool originValid) {
  if (target.id.empty()) {
    cancelDirectTo();
    return;
  }
  directToActive_ = true;
  directToTarget_ = std::move(target);
  if (!map_.flightPlan.empty()) {
    applyInPlanDirectToRouteSlice(map_, map_.flightPlan, directToTarget_);
  }
  routeInitialized_ = true;
  map_.directToOriginValid = originValid;
  map_.directToOriginLat = originLat;
  map_.directToOriginLon = originLon;
}

void MockDataSource::cancelDirectTo() {
  directToActive_ = false;
  map_.directToActive = false;
  clearFlightPlanRouteSlice(map_);
}

void MockDataSource::setOwnshipPosition(double lat, double lon, float headingDeg) {
  map_.ownshipLat = lat;
  map_.ownshipLon = lon;
  map_.positionValid = true;
  data_.headingDeg = headingDeg;
  if (navFeatures_ != nullptr) {
    mapPanDirty_ = true;
  }
}

void MockDataSource::setActiveLegIndex(int index) {
  if (index < 0 || index >= static_cast<int>(route_.size())) return;
  legIndex_ = static_cast<std::size_t>(index);
  syncFmaFromLegIndex();
}

void MockDataSource::syncFmaFromLegIndex() {
  if (route_.empty()) return;
  data_.fmaActiveLegIndex = static_cast<int>(legIndex_);
  if (legIndex_ > 0) {
    data_.fmaFromWpt = route_[legIndex_ - 1].id;
    data_.fmaToWpt = route_[legIndex_].id;
  } else {
    data_.fmaFromWpt.clear();
    data_.fmaToWpt = route_.front().id;
  }
}

void MockDataSource::navigateRoute(double dt) {
  if (navigationPinned_) {
    syncFmaFromLegIndex();
    return;
  }
  // GPS Direct-To overrides route sequencing: fly straight to the target and
  // publish the magenta direct course for the map.
  if (directToActive_) {
    map_.directToActive = true;
    map_.directTo = directToTarget_;

    const double brg = navBearingDeg(map_.ownshipLat, map_.ownshipLon,
                                     directToTarget_.lat, directToTarget_.lon);
    const double distNm = navDistanceNm(
        map_.ownshipLat, map_.ownshipLon, directToTarget_.lat,
        directToTarget_.lon);

    const double diff =
        std::fmod(brg - data_.headingDeg + 540.0, 360.0) - 180.0;
    const double turnStep = 3.0 * dt;
    double heading = std::fabs(diff) <= turnStep
                         ? brg
                         : data_.headingDeg + (diff > 0.0 ? turnStep : -turnStep);
    heading = std::fmod(heading + 360.0, 360.0);
    data_.headingDeg = static_cast<float>(heading);
    data_.courseDeg = static_cast<float>(brg);

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kNmPerDeg = 60.0;
    const double cosLat =
        std::max(0.05, std::cos(map_.ownshipLat * kPi / 180.0));
    const double moveNm =
        std::max(40.0, static_cast<double>(data_.groundSpeedKts)) * dt / 3600.0;
    const double hdgRad = heading * kPi / 180.0;
    map_.ownshipLat += moveNm * std::cos(hdgRad) / kNmPerDeg;
    map_.ownshipLon += moveNm * std::sin(hdgRad) / (kNmPerDeg * cosLat);
    return;
  }
  map_.directToActive = false;

  if (route_.size() < 2) return;

  const MapLeg& target = route_[legIndex_];
  const double brg =
      navBearingDeg(map_.ownshipLat, map_.ownshipLon, target.lat, target.lon);
  const double distNm =
      navDistanceNm(map_.ownshipLat, map_.ownshipLon, target.lat, target.lon);

  // Ease the heading toward the bearing (shortest direction) at a standard-rate
  // turn, so course changes at waypoints look like real turns, not snaps.
  const double diff =
      std::fmod(brg - data_.headingDeg + 540.0, 360.0) - 180.0;
  const double turnStep = 3.0 * dt;  // ~standard rate
  double heading = std::fabs(diff) <= turnStep
                       ? brg
                       : data_.headingDeg + (diff > 0.0 ? turnStep : -turnStep);
  heading = std::fmod(heading + 360.0, 360.0);
  data_.headingDeg = static_cast<float>(heading);
  data_.courseDeg = static_cast<float>(brg);

  // Move along the current heading at ground speed.
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kNmPerDeg = 60.0;
  const double cosLat = std::max(0.05, std::cos(map_.ownshipLat * kPi / 180.0));
  const double moveNm =
      std::max(40.0, static_cast<double>(data_.groundSpeedKts)) * dt / 3600.0;
  const double hdgRad = heading * kPi / 180.0;
  map_.ownshipLat += moveNm * std::cos(hdgRad) / kNmPerDeg;
  map_.ownshipLon += moveNm * std::sin(hdgRad) / (kNmPerDeg * cosLat);
}

void MockDataSource::applyGpsNavigation(FmsNavigator& navigator, bool obsMode,
                                        CdiSource cdiSource, float nmPerDot) {
  map_.directToActive = directToActive_;
  map_.directTo = directToTarget_;

  if (navigationPinned_) {
    syncFmaFromLegIndex();
    return;
  }

  NavigationCallbacks callbacks;
  callbacks.onDirectToCaptured = [this](int /*activeLegIndex*/) {
    directToActive_ = false;
    directToTarget_ = {};
    map_.directToActive = false;
    map_.directTo = {};
  };

  const float scale = nmPerDot > 0.01f ? nmPerDot : 0.5f;
  runNavigationFrame(navigator, map_, data_, obsMode, cdiSource, scale, callbacks);

  if (data_.fmaActiveLegIndex >= 0) {
    legIndex_ = static_cast<std::size_t>(data_.fmaActiveLegIndex);
  }
}

void MockDataSource::setMapPanCenter(bool active, double lat, double lon) {
  if (active != mapPanActive_ ||
      (active && (lat != mapPanLat_ || lon != mapPanLon_))) {
    mapPanDirty_ = true;  // pointer toggled/moved: re-scan around the new center
  }
  mapPanActive_ = active;
  mapPanLat_ = lat;
  mapPanLon_ = lon;
}

void MockDataSource::setChartRangeNm(float rangeNm) {
  if (map_.rangeNm != rangeNm) {
    map_.rangeNm = rangeNm;
    mapPanDirty_ = true;
  }
}

void MockDataSource::setMapViewHalfExtentNm(float halfExtentNm) {
  if (mapViewHalfExtentNm_ != halfExtentNm) {
    mapViewHalfExtentNm_ = halfExtentNm;
    mapPanDirty_ = true;
  }
}

void MockDataSource::refreshFeatures(double dt) {
  if (navFeatures_ == nullptr) return;  // demo nav data already seeded

  // Rebuild every map layer from the real X-Plane databases on a throttled
  // timer (the world databases are large). Each layer is sourced independently:
  // the backend loads them on separate background threads, so some may still be
  // empty (not yet loaded) while others are populated -- the query simply
  // returns what is available so far, never fabricated data.
  sinceFeatureRebuild_ += dt;
  const bool due = map_.features.empty() || mapPanDirty_ ||
                   sinceFeatureRebuild_ >= kFeatureRebuildIntervalSeconds;
  if (!due) return;
  sinceFeatureRebuild_ = 0.0;
  mapPanDirty_ = false;

  // When the Map Pointer is active the view (and therefore the queries) center
  // on the panned map view center rather than ownship; see applyMapPanToDataSource().
  const double lat = mapPanActive_ ? mapPanLat_ : map_.ownshipLat;
  const double lon = mapPanActive_ ? mapPanLon_ : map_.ownshipLon;
  if (navFeatures_->ready()) {
    map_.features =
        navFeatures_->nearby(lat, lon, kFeatureQueryRangeNm, kMaxFeatures);
    const NavDatabaseInfo info = navFeatures_->navDatabaseInfo();
    if (info.available) map_.navDatabase = info;
  }
  map_.airspaces =
      navFeatures_->nearbyAirspaces(lat, lon, kAirspaceQueryRangeNm,
                                    kMaxAirspaces);
  map_.airways =
      navFeatures_->nearbyAirways(lat, lon, kAirwayQueryRangeNm, kMaxAirways);
  map_.runways =
      navFeatures_->nearbyRunways(lat, lon, kRunwayQueryRangeNm, kMaxRunways);
  map_.taxiways =
      navFeatures_->nearbyTaxiways(lat, lon, kTaxiwayQueryRangeNm, kMaxTaxiways);
  map_.landLines =
      navFeatures_->nearbyLandLines(lat, lon, map_.rangeNm, kMaxLandLines,
                                      mapViewHalfExtentNm_);
  map_.cities =
      navFeatures_->nearbyCities(lat, lon, map_.rangeNm, kMaxCities,
                                   mapViewHalfExtentNm_);
  map_.obstacles = navFeatures_->nearbyObstacles(lat, lon, kObstacleQueryRangeNm,
                                                 kMaxObstacles);
}

namespace {

float* standbyMhzPtr(FlightData& d, RadioUnit unit) {
  switch (unit) {
    case RadioUnit::Nav1:
      return &d.nav1StandbyMhz;
    case RadioUnit::Nav2:
      return &d.nav2StandbyMhz;
    case RadioUnit::Com1:
      return &d.com1StandbyMhz;
    case RadioUnit::Com2:
      return &d.com2StandbyMhz;
  }
  return &d.nav1StandbyMhz;
}

float* activeMhzPtr(FlightData& d, RadioUnit unit) {
  switch (unit) {
    case RadioUnit::Nav1:
      return &d.nav1ActiveMhz;
    case RadioUnit::Nav2:
      return &d.nav2ActiveMhz;
    case RadioUnit::Com1:
      return &d.com1ActiveMhz;
    case RadioUnit::Com2:
      return &d.com2ActiveMhz;
  }
  return &d.nav1ActiveMhz;
}

void applyXpdrModeString(FlightData& d, int mode) {
  switch (mode) {
    case 0:
      d.transponderMode = "OFF";
      break;
    case 1:
      d.transponderMode = "STBY";
      break;
    case 2:
      d.transponderMode = "ON";
      break;
    default:
      d.transponderMode = "ALT";
      break;
  }
}

}  // namespace

void MockDataSource::tuneRadioStandby(RadioUnit unit, float standbyMhz) {
  *standbyMhzPtr(data_, unit) = standbyMhz;
}

void MockDataSource::transferRadio(RadioUnit unit) {
  float* active = activeMhzPtr(data_, unit);
  float* standby = standbyMhzPtr(data_, unit);
  const float tmp = *active;
  *active = *standby;
  *standby = tmp;
}

void MockDataSource::setRadioVolume(RadioUnit unit, float volume) {
  data_.*radioVolumeMember(unit) = volume;
}

void MockDataSource::setNavIdent(RadioUnit unit, bool on) {
  data_.*navIdentAudioMember(unit) = on;
}

void MockDataSource::setTransponderCode(int code) {
  data_.transponderCode = code;
}

void MockDataSource::setTransponderMode(int mode) {
  applyXpdrModeString(data_, mode);
}

void MockDataSource::setHeadingBug(float deg) { data_.selectedHeadingDeg = deg; }

void MockDataSource::setSelectedCourse(float deg) { data_.courseDeg = deg; }

void MockDataSource::setBaroInHg(float inHg) { data_.baroSettingInHg = inHg; }

}  // namespace avionics
