#include "avionics/MockDataSource.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "avionics/NavMath.h"

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

  map.airspaces = {classB, classC, classD};
}

// Built-in route flown when the shell does not supply a real flight plan.
// These are real Bay Area waypoints, so the demo still looks plausible.
const std::vector<MapLeg>& defaultRoute() {
  static const std::vector<MapLeg> kRoute = {
      {kKpaoLat, kKpaoLon, "KPAO"},
      {37.5930, -121.8810, "SUNOL"},
      {36.9357, -121.7896, "KWVI"},
      {37.3925, -122.2808, "OSI"},
  };
  return kRoute;
}

// Hand-placed nav features used only when no real NavFeatureSource is wired in.
void seedDemoFeatures(MapData& map) {
  map.features = {
      {MapFeatureType::Airport, 37.5111, -122.2495, "KSQL"},
      {MapFeatureType::Vor, 37.3925, -122.2808, "OSI"},
      {MapFeatureType::Ndb, 37.5100, -122.0300, "OAK"},
      {MapFeatureType::Fix, 37.4200, -122.0500, "MENLO"},
      {MapFeatureType::Fix, 37.5300, -122.1700, "EDDYY"},
      {MapFeatureType::Airport, 37.6213, -122.3790, "KSFO"},
  };
}

// Features are pulled within this radius (and capped) so both the small PFD
// inset and a zoomed-out MFD MAP page have data to draw, refreshed on a timer
// since the world database is large.
constexpr float kFeatureQueryRangeNm = 160.0f;
constexpr std::size_t kMaxFeatures = 500;
constexpr double kFeatureRebuildIntervalSeconds = 2.0;

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

  // Vertical deviation: demo a GPS Glidepath that drifts gently around centre
  // (magenta diamond), plus the matching VNAV required-VS chevron on the VSI.
  data_.vdiKind = VerticalDeviationKind::Glidepath;
  data_.vdiValid = true;
  data_.vdiDeviationDots = 1.2f * std::sin(t * 0.12f);
  data_.requiredVsValid = true;
  data_.requiredVsFpm = -700.0f + 200.0f * std::sin(t * 0.09f);

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
  data_.casLowVacuum = std::fmod(t, 20.0f) > 6.0f;          // steady caution
  data_.casFuelLow = std::fmod(t, 30.0f) > 18.0f;           // intermittent
  data_.casPitotHeatOff = std::fmod(t, 16.0f) > 9.0f;       // caution
  data_.casLowVoltage = std::fmod(t, 24.0f) > 16.0f;        // warning (red)
  data_.casOilPressureLow = std::fmod(t, 40.0f) > 33.0f;    // warning (red)
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

  data_.fmaVerticalValue = static_cast<int>(std::lround(data_.selectedAltitudeFt));
  data_.timerSeconds = static_cast<int>(t) % 36000;
  const int totalSec = 18 * 3600 + static_cast<int>(t) % 86400;
  data_.utcHour = (totalSec / 3600) % 24;
  data_.utcMinute = (totalSec / 60) % 60;
  data_.utcSecond = totalSec % 60;

  refreshFeatures(dtSeconds);
  map_.terrain = terrainSource_ != nullptr ? terrainSource_ : &terrain_;
}

void MockDataSource::setRoute(std::vector<MapLeg> route) {
  route_ = std::move(route);
  routeInitialized_ = false;  // re-seed position from the new route start
}

void MockDataSource::ensureRoute() {
  if (routeInitialized_) return;

  if (route_.size() < 2) route_ = defaultRoute();
  map_.flightPlan = route_;
  map_.rangeNm = 10.0f;
  map_.ownshipLat = route_.front().lat;
  map_.ownshipLon = route_.front().lon;
  map_.positionValid = true;
  legIndex_ = 1;
  data_.headingDeg = static_cast<float>(
      navBearingDeg(route_[0].lat, route_[0].lon, route_[1].lat, route_[1].lon));

  // Hand-placed demo features only when no real nav source is wired in (with
  // one, refreshFeatures fills in the actual nearby navaids/fixes). The demo
  // airspace rings are always seeded: there is no offline airspace database in
  // the mock path, and without them the airspace declutter, the NRST airspaces
  // page, and the map boundary styles would never be exercised.
  if (navFeatures_ == nullptr) seedDemoFeatures(map_);
  seedDemoAirspace(map_, map_.ownshipLat, map_.ownshipLon);

  routeInitialized_ = true;
}

void MockDataSource::navigateRoute(double dt) {
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

  // Active-leg readout on the FMA (top bar).
  data_.fmaFromWpt = route_[(legIndex_ + route_.size() - 1) % route_.size()].id;
  data_.fmaToWpt = target.id;
  data_.fmaLegDistanceNm = static_cast<float>(distNm);
  data_.fmaLegBearingDeg = static_cast<float>(brg);

  // Sequence to the next leg on arrival, looping back to the start.
  if (distNm <= std::max(0.4, moveNm * 1.5)) {
    legIndex_ = (legIndex_ + 1) % route_.size();
  }
}

void MockDataSource::refreshFeatures(double dt) {
  if (navFeatures_ == nullptr) return;  // demo features already seeded

  sinceFeatureRebuild_ += dt;
  const bool due = map_.features.empty() ||
                   sinceFeatureRebuild_ >= kFeatureRebuildIntervalSeconds;
  if (due && navFeatures_->ready()) {
    map_.features = navFeatures_->nearby(map_.ownshipLat, map_.ownshipLon,
                                         kFeatureQueryRangeNm, kMaxFeatures);
    sinceFeatureRebuild_ = 0.0;
  }
}

}  // namespace avionics
