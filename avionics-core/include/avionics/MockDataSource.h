#pragma once

#include <cstddef>
#include <vector>

#include "avionics/Checklist.h"
#include "avionics/Eis.h"
#include "avionics/DataSource.h"
#include "avionics/FmsNavigator.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/NexradWeatherRadar.h"
#include "avionics/Radio.h"
#include "avionics/Terrain.h"
#include "avionics/WeatherRadar.h"

namespace avionics {

// Animated, dependency-free data source for bring-up and tests: lets the
// standalone shell render something believable before (or instead of) a live
// X-Plane link. The aircraft flies a looping route, so the moving map shows
// realistic motion along a flight plan.
//
// Only the aircraft motion is simulated: the moving-map navigation data comes
// from the real X-Plane databases. setNavFeatureSource() supplies the actual
// nearby features, airspaces, airways, runways, land vectors, and obstacles
// near the aircraft, and setRoute() flies an actual flight plan (e.g. parsed
// from an .fms file). The hand-placed demo nav data is a fallback used only
// when no source is wired in (e.g. a bare unit test).
class MockDataSource : public DataSource {
 public:
  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }
  const ChecklistData& checklistSnapshot() const override {
    return (checklists_ != nullptr && checklists_->ready())
               ? checklists_->checklists()
               : emptyChecklists_;
  }
  std::string checklistSourcePath() const override {
    return checklists_ != nullptr ? checklists_->sourcePath() : std::string();
  }
  const EisLayout& eisLayoutSnapshot() const override {
    return (eisSource_ != nullptr && eisSource_->ready()) ? eisSource_->layout()
                                                           : emptyEis_;
  }

  // The mock is a synthetic feed: bring the live pages up automatically rather
  // than gating bring-up / screenshots on an ENT keypress.
  bool requiresPowerUpAcknowledge() const override { return true; }

  std::string aircraftIcaoType() const override { return "C172"; }
  std::string aircraftAcfRelativePath() const override { return {}; }

  void setMapPanCenter(bool active, double lat, double lon) override;
  void setChartRangeNm(float rangeNm) override;
  void setMapViewHalfExtentNm(float halfExtentNm) override;

  // Fly along this route (needs >= 2 waypoints), looping back to the start.
  // Replaces the built-in demo route. Safe to call at runtime (e.g. once a
  // flight plan finishes loading) -- the aircraft repositions to the new start.
  void setRoute(std::vector<MapLeg> route);

  // Replace the route in flight without repositioning the aircraft (an FPL
  // page edit): keeps flying toward the same waypoint when it survived the
  // edit, otherwise picks up the new plan from its start. An empty route
  // clears the plan (the aircraft holds its heading).
  void updateRoute(std::vector<MapLeg> route);

  // Engage GPS Direct-To: fly straight from the present position to `target`.
  // Overlays a magenta direct course on the map; on arrival the direct-to
  // clears and route navigation resumes (sequencing past the target if it is
  // part of the loaded route).
  void directTo(MapLeg target);
  // Cancel an active Direct-To without changing the loaded route.
  void cancelDirectTo();

  void applyGpsNavigation(FmsNavigator& navigator, bool obsMode,
                          CdiSource cdiSource, float nmPerDot) override;

  // Source of real nearby navigation data (features, airspaces, airways,
  // runways, land vectors, obstacles). When set it replaces the hand-placed
  // demo data so the mock map matches the X-Plane databases.
  void setNavFeatureSource(const NavFeatureSource* source) {
    navFeatures_ = source;
  }

  // Topographic map background. When unset the built-in procedural terrain is
  // used; the standalone shell typically passes a DsfTerrainStore here.
  void setTerrainSource(const TerrainSource* source) { terrainSource_ = source; }

  // Weather-radar / NEXRAD overlay. When unset the built-in procedural weather
  // is used; the plugin passes a DatarefWeatherRadar here.
  void setWeatherSource(const WeatherRadarSource* source) {
    weatherSource_ = source;
  }

  // Author-supplied checklists for the MFD Checklist page group. Optional; when
  // unset the Checklist page shows "no checklist available".
  void setChecklistSource(const ChecklistSource* source) {
    checklists_ = source;
  }

  void setEisSource(const EisSource* source) { eisSource_ = source; }

  // Adds simulated turbulence (chaotic bumps on attitude, airspeed, vertical
  // speed, etc.) on top of the smooth base motion. Off by default; the
  // standalone shell exposes this as a Data Source menu toggle.
  void setTurbulenceEnabled(bool enabled) { turbulenceEnabled_ = enabled; }
  bool turbulenceEnabled() const { return turbulenceEnabled_; }

  // Freezes the feed on the ground at KFMY runway 31 -- stationary, wings
  // level, at field elevation, with the engine idling -- instead of flying the
  // demo route. Off by default; the standalone shell exposes this as a Data
  // Source menu option.
  void setGroundMode(bool onGround) { groundMode_ = onGround; }
  bool groundMode() const { return groundMode_; }

  // Whether the demo feed raises the cycling CAS annunciations (the amber/red
  // OIL PRESSURE, LOW VOLTS, FUEL LOW, etc. messages that toggle on a timer to
  // exercise the alerting window). On by default; the standalone shell exposes
  // this as a Debug menu toggle so the demo can be flown without the periodic
  // alerts. Only affects the flying demo -- a healthy idling aircraft on the
  // ground raises no CAS messages regardless.
  void setCasMessagesEnabled(bool enabled) { casMessagesEnabled_ = enabled; }
  bool casMessagesEnabled() const { return casMessagesEnabled_; }

  // Screenshot / debug: populate the Alerts window with reversionary sensor
  // failures (AHRS/ADC/NAV messages) without taking down the whole data link.
  void setReversionaryAlertsDemo(bool enabled) {
    reversionaryAlertsDemo_ = enabled;
  }

  // Local stand-in for sim commands while the mock feed is active.
  void tuneRadioStandby(RadioUnit unit, float standbyMhz);
  void transferRadio(RadioUnit unit);
  void setRadioVolume(RadioUnit unit, float volume);
  void setNavIdent(RadioUnit unit, bool on);
  void setTransponderCode(int code);
  void setTransponderMode(int mode);
  // HDG bug / selected course / baro knobs (the demo has no autopilot or weather
  // model, so these just hold the value the knob sets). The course tracks the
  // active leg while navigating, so a manual course set is transient in demo.
  void setHeadingBug(float deg);
  void setSelectedCourse(float deg);
  void setBaroInHg(float inHg);

  // GDU power switches (the demo has no electrical model, so these just hold the
  // state). Used to exercise the power-up / reversionary (display-backup)
  // behavior: master gates the PFD, avionics gates the MFD.
  void setMasterPowerOn(bool on) { data_.masterPowerOn = on; }
  void setAvionicsPowerOn(bool on) { data_.avionicsPowerOn = on; }

 private:
  void ensureRoute();              // lazily seed route + initial position
  void navigateRoute(double dt);   // advance the aircraft along the route
  void refreshFeatures(double dt); // pull nearby features (real or demo)
  void updateOnGround(double dt);  // parked-at-KFMY stationary state
  void publishEisChannels();       // copy engine fields into the EIS channels
  void advanceClockFields();       // tick the UTC clock / timer fields
  void publishMapBackground(double dt);  // attach terrain + weather overlays

  FlightData data_;
  MapData map_;
  ProceduralTerrain terrain_;  // synthetic topo background for the mock map
  ProceduralWeatherRadar weather_;  // onboard radar (WX page + offline overlay)
  NexradWeatherRadar nexrad_;       // live datalink NEXRAD map overlay
  double elapsedSeconds_ = 0.0;

  std::vector<MapLeg> route_;
  std::size_t legIndex_ = 1;  // route_ index the aircraft is flying toward
  bool routeInitialized_ = false;
  bool turbulenceEnabled_ = false;  // chaotic bumps on top of the base motion
  bool groundMode_ = false;         // parked at KFMY rwy 31 instead of flying
  bool casMessagesEnabled_ = true;  // demo CAS annunciations cycle on a timer
  bool reversionaryAlertsDemo_ = false;

  // Active Direct-To: when set, navigateRoute flies straight to directToTarget_
  // instead of sequencing the route.
  bool directToActive_ = false;
  MapLeg directToTarget_;

  const NavFeatureSource* navFeatures_ = nullptr;
  const TerrainSource* terrainSource_ = nullptr;
  const WeatherRadarSource* weatherSource_ = nullptr;
  const ChecklistSource* checklists_ = nullptr;
  const EisSource* eisSource_ = nullptr;
  double sinceFeatureRebuild_ = 0.0;

  // MFD Map Pointer (pan) state pushed by the shell. When active, the nearby-
  // feature scans center on (mapPanLat_, mapPanLon_) instead of ownship so the
  // panned-to area has data. mapPanDirty_ forces an immediate (un-throttled)
  // rebuild when the pointer is toggled or moved, so panning feels responsive.
  bool mapPanActive_ = false;
  double mapPanLat_ = 0.0;
  double mapPanLon_ = 0.0;
  bool mapPanDirty_ = false;
  float mapViewHalfExtentNm_ = 0.0f;

  static inline const ChecklistData emptyChecklists_{};
  static inline const EisLayout emptyEis_{};
};

}  // namespace avionics
