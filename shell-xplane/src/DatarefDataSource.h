#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <array>

#include "DsfTerrainStore.h"
#include "DatarefWeatherRadar.h"
#include "LandDataStore.h"
#include "MetarFileStore.h"
#include "ObstacleStore.h"
#include "XPLMDataAccess.h"
#include "avionics/AptDatParser.h"
#include "avionics/Checklist.h"
#include "avionics/DataSource.h"
#include "avionics/Eis.h"
#include "avionics/EisLegacy.h"
#include "avionics/FmsNavigator.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/NexradWeatherRadar.h"
#include "avionics/Radio.h"

namespace avionics {

class MfdController;

// In-process DataSource for the X-Plane plugin: resolves dataref handles once
// and reads them each frame. Dataref reads inside the sim are cheap, so no
// interpolation is needed here (unlike the standalone/network source).
class DatarefDataSource : public DataSource {
 public:
  explicit DatarefDataSource(EisSource* eisSource = nullptr);
  ~DatarefDataSource() override;

  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }
  std::uint32_t mapGeometryEpoch() const override { return map_.geometryEpoch; }
  const EisLayout& eisLayoutSnapshot() const override {
    return (eisSource_ != nullptr && eisSource_->ready()) ? eisSource_->layout()
                                                          : emptyEis_;
  }
  const ChecklistData& checklistSnapshot() const override {
    return (checklistSource_ != nullptr && checklistSource_->ready())
               ? checklistSource_->checklists()
               : emptyChecklists_;
  }
  std::string checklistSourcePath() const override {
    return checklistSource_ != nullptr ? checklistSource_->sourcePath()
                                       : std::string();
  }

  void setEisSource(EisSource* source) { eisSource_ = source; }
  void setChecklistSource(ChecklistSource* source) {
    checklistSource_ = source;
  }
  void setObstacleStore(const ObstacleStore* store) { obstacles_ = store; }

  // Called from the plugin draw path after the MFD engine state is known. Pushes
  // the EFIS weather mode, antenna tilt, and sector width to the sim based on
  // the MFD's NEXRAD overlay state and dedicated Weather Radar page controls.
  void syncWeatherRadar(const MfdController& ui);

  // Bidirectional sync with sim/cockpit2/EFIS/map_range_nm so GCU range (and
  // stock G1000 when our handler passes through) updates the NXi map ladder.
  void syncMapRangeFromSim(MfdController& ui);
  void pushMapRangeToSim(float rangeNm);
  // Step sim/cockpit2/EFIS/map_range_nm and optionally mirror onto the MFD UI.
  bool stepMapRangeFromSim(int direction, MfdController* ui);

  // Whether the current airframe carries a weather radar (probed from the sim's
  // radar return texture; see DatarefWeatherRadar::equipped). The MFD uses this
  // to hide its dedicated Weather Radar page on unequipped aircraft.
  bool weatherRadarEquipped() const { return weather_.equipped(); }

  // Station weather for the WPT - Weather Information page, read from X-Plane
  // 12's downloaded real-weather METAR files (no TAF source exists in X-Plane).
  // The pointer is stable for the source's lifetime; the store starts loading
  // once the install root is resolved (first update()).
  StationWeatherSource& stationWeatherSource() { return metar_; }

  void setMapPanCenter(bool active, double lat, double lon) override;
  void setInsetMapQuery(bool active, double lat, double lon, float rangeNm,
                        float viewHalfExtentNm,
                        const std::string& targetIdent = {}) override;
  void setChartRangeNm(float rangeNm) override;
  void setMapViewHalfExtentNm(float halfExtentNm) override;

  std::string aircraftIcaoType() const override { return lastAircraftIcao_; }
  std::string aircraftAcfRelativePath() const override {
    return lastAircraftAcfPath_;
  }

  // NAV/COM bezel tuning written straight back to the sim's radio datarefs so
  // the stock radios follow the glass. tuneRadioStandby sets the standby
  // frequency; transferRadio swaps active and standby (the cyan transfer
  // arrow). These mirror the standalone shell's XPlaneConnection writers.
  void tuneRadioStandby(RadioUnit unit, float standbyMhz);
  void transferRadio(RadioUnit unit);
  void setRadioVolume(RadioUnit unit, float volume);
  void setNavIdent(RadioUnit unit, bool on);

  // Transponder commits from the XPDR softkeys. Mode uses the X-Plane
  // transponder_mode enum (off=0, stdby=1, on=2, alt=3).
  void setTransponderCode(int code);
  void setTransponderMode(int mode);

  // HDG / CRS / BARO knob commits from the bezel (mirrors standalone shell).
  void setHeadingBug(float deg);
  void setSelectedCourse(float deg, CdiSource source);
  void setBaroInHg(float inHg);

  // Flight-plan edits from the PFD Active Flight Plan window or the MFD FPL
  // page. Programs the sim FMS when programSimulator is true; otherwise the
  // route is kept on the in-plugin display feed only.
  void setLocalFlightPlan(std::vector<MapLeg> route);
  void setRouteOverride(std::vector<MapLeg> route, bool programSimulator = true);
  void clearRouteOverride();
  void setDirectTo(MapLeg target, bool flyHold = false);
  void clearDirectTo();
  void onNavigatorDirectToCaptured(int activeLegIndex);

  void applyGpsNavigation(FmsNavigator& navigator, bool obsMode,
                          CdiSource cdiSource, float nmPerDot) override;
  void syncSimulatorActiveLeg(int legIndex) override;

  // apt.dat airport metadata (tower/fuel/kind and published comm frequencies).
  // Used by the plugin NavFeatureSource for COM frequency decode and WPT/NRST
  // frequency lists once the background apt.dat load finishes.
  bool aptDatReady() const { return aptDatLoaded_.load(); }
  std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const;
  std::vector<MapFeature> lookupNavIdent(const std::string& ident,
                                         std::size_t maxCount) const;
  std::vector<MapFeature> lookupNavIdentNear(const std::string& ident,
                                             double refLat, double refLon,
                                             std::size_t maxCount) const;
  std::string firstNavIdentWithPrefix(const std::string& prefix) const;

 private:
  // Rebuild the moving-map snapshot (ownship position, active flight plan, and
  // the range-filtered nearby-feature layer) each frame; the feature scan is
  // throttled by dtSeconds rather than run every frame.
  void updateMap(double dtSeconds);

  // One-time walk of X-Plane's in-RAM navigation database (airports, VORs,
  // NDBs) into a flat cache, range-filtered into map_.features as ownship
  // moves. The XPLMNavigation API must be called on the sim thread, which is
  // where update() runs, so no synchronization is needed.
  void buildNavCache();

  // One-time discovery of apt.dat, airspace.txt, and terrain tiles. Deferred
  // from the constructor until the first update() so XPLMGetSystemPath and file
  // probes run after the sim is fully up (constructor-time probes can fail).
  void ensureInstallDataLoaded();

  // Background load of X-Plane's OpenAir airspace file. The XPLMNavigation API
  // doesn't expose airspace, so the plugin parses the same file the standalone
  // does. Parsing runs off the sim thread; airspaceLoaded_ publishes the result.
  void loadAirspaceAsync(std::string airspaceFilePath);

  // Background load of apt.dat airport metadata (tower/fuel/kind) and runway /
  // taxiway pavement geometry for the close-range airport diagram.
  void loadAptDatAsync();

  // Background moving-map query worker. filterNearby + the apt-geometry /
  // airspace / obstacle scans are pure reads of the nav / apt.dat / airspace /
  // obstacle caches (each immutable once its loader thread publishes it) plus a
  // query center, so they run off the sim thread. updateMap() submits a job when
  // a rebuild is due and, when the worker finishes, swaps the result into map_
  // and bumps the geometry epoch -- so every map_ write stays on the sim thread.
  struct MapQueryResult {
    std::vector<MapFeature> features;
    std::vector<MapRunway> runways;
    std::vector<MapPavement> taxiways;
    std::vector<MapTaxiwayLabel> taxiwayLabels;
    std::vector<MapAirspace> airspaces;
    std::vector<MapObstacle> obstacles;
    std::vector<MapLandLine> landLines;
    std::vector<MapLandCity> cities;
    bool aptGeometryIncluded = false;  // runways/taxiways/labels were scanned
    bool airspaceIncluded = false;
    bool obstaclesIncluded = false;
    bool landIncluded = false;
  };
  void startMapQueryWorker();
  void stopMapQueryWorker();
  void mapQueryWorkerMain();
  // Hand the current query center + which caches are ready to the worker, unless
  // a job is already running. Returns true if a job was submitted.
  bool submitMapQuery(double lat, double lon);
  // If the worker finished, move its result into map_ (sim thread). Returns true
  // when a result was adopted.
  bool adoptMapQueryResult();

  void rebuildEisBindings();
  // Detects an aircraft change (acf_ICAO + acf_relative_path) and points the
  // EIS and checklist stores at the matching per-aircraft profile.
  void updateAircraftProfile();

  // Plane Maker V-speed limits for the loaded aircraft (PFD tape color bands).
  void refreshAirspeedEnvelope();

  FlightData data_;
  MapData map_;

  // Real-world topographic background: samples X-Plane's Global Scenery DSF DEM
  // tiles (with a procedural fallback). Pointed at map_.terrain each frame so
  // MapView can draw the TER TOPO/REL layer.
  std::unique_ptr<DsfTerrainStore> terrain_;

  bool installDataStarted_ = false;

  // X-Plane 12.3 onboard weather-radar return-strength texture (dedicated MFD
  // Weather Radar page, and the map overlay fallback before datalink arrives).
  DatarefWeatherRadar weather_;

  // Live datalink NEXRAD (real ground radar) for the map precipitation overlay.
  NexradWeatherRadar nexrad_;

  // X-Plane 12 real-weather METAR files (WPT - Weather Information page).
  MetarFileStore metar_;

  std::vector<MapFeature> navCache_;
  bool navCacheBuilt_ = false;
  double sinceMapRebuildSeconds_ = 0.0;

  // MFD Map Pointer (pan) state pushed from the draw path. When active, the
  // nearby-data scans center on (mapPanLat_, mapPanLon_) instead of ownship so
  // the panned-to area has data. mapPanDirty_ forces an immediate rebuild when
  // the pointer is toggled or moved so panning feels responsive.
  bool mapPanActive_ = false;
  double mapPanLat_ = 0.0;
  double mapPanLon_ = 0.0;
  bool mapPanDirty_ = false;
  float chartRangeNm_ = mapRangeNmAt(kMapRangeDefaultIndex);
  float mapViewHalfExtentNm_ = 0.0f;

  bool insetMapActive_ = false;
  double insetMapLat_ = 0.0;
  double insetMapLon_ = 0.0;
  float insetMapRangeNm_ = 0.0f;
  float insetMapHalfExtentNm_ = 0.0f;
  std::string insetMapTargetIdent_;
  bool insetMapDirty_ = false;
  void rebuildInsetMap();

  XPLMDataRef efisMapRangeNm_ = nullptr;
  float lastPushedMapRangeNm_ = -1.0f;

  std::vector<MapAirspace> airspaceCache_;
  std::atomic<bool> airspaceLoaded_{false};
  std::thread airspaceThread_;

  std::unordered_map<std::string, AirportMeta> aptMetaByIcao_;
  std::unordered_map<int, std::vector<MapRunway>> runwayCells_;
  std::unordered_map<int, std::vector<MapPavement>> pavementCells_;
  std::unordered_map<int, std::vector<MapTaxiwayLabel>> taxiwayLabelCells_;
  std::atomic<bool> aptDatLoaded_{false};
  std::atomic<bool> aptMapDirty_{false};
  std::thread aptDatThread_;
  std::string aptDatPath_;
  std::string aptGeometryCachePath_;

  // Map-query worker handoff (see MapQueryResult above).
  enum class MapQueryPhase { Idle, Running, Done };
  std::thread mapQueryThread_;
  std::mutex mapQueryMu_;
  std::condition_variable mapQueryCv_;
  MapQueryPhase mapQueryPhase_ = MapQueryPhase::Idle;
  bool mapQueryStop_ = false;
  double mapQueryReqLat_ = 0.0;
  double mapQueryReqLon_ = 0.0;
  bool mapQueryReqApt_ = false;
  bool mapQueryReqAirspace_ = false;
  bool mapQueryReqObstacles_ = false;
  bool mapQueryReqLand_ = false;
  float mapQueryLandRangeNm_ = 0.0f;
  float mapQueryLandViewHalfExtentNm_ = 0.0f;
  MapQueryResult mapQueryResult_;  // worker output, guarded by mapQueryMu_

  // Bundled Natural Earth coastlines/borders/cities (land_data.bin). Loaded on
  // a background thread; the map-query worker reads it once loaded().
  std::unique_ptr<LandDataStore> landData_;
  bool landEverLoaded_ = false;

  XPLMDataRef airspeed_ = nullptr;
  XPLMDataRef altitude_ = nullptr;
  XPLMDataRef heading_ = nullptr;
  XPLMDataRef pitch_ = nullptr;
  XPLMDataRef roll_ = nullptr;
  XPLMDataRef flightDirectorPitch_ = nullptr;
  XPLMDataRef flightDirectorRoll_ = nullptr;
  XPLMDataRef flightDirectorMode_ = nullptr;
  XPLMDataRef verticalSpeed_ = nullptr;
  XPLMDataRef slip_ = nullptr;

  // Ownship geographic position for the moving map (read as doubles).
  XPLMDataRef latitude_ = nullptr;
  XPLMDataRef longitude_ = nullptr;

  // TCAS target arrays for the map traffic overlay (element 0 is ownship).
  XPLMDataRef tcasTargetLat_ = nullptr;
  XPLMDataRef tcasTargetLon_ = nullptr;
  XPLMDataRef tcasTargetEleMeters_ = nullptr;
  XPLMDataRef tcasTargetVerticalSpeedFpm_ = nullptr;

  // Sim UTC clock and date for the chrome clock and the Trip Planning
  // sunrise/sunset rows.
  XPLMDataRef zuluTimeSec_ = nullptr;
  XPLMDataRef localDateDays_ = nullptr;

  // GPS active-leg navigation status box (destination identifier, distance and
  // magnetic bearing). The identifier is a byte[] string read via XPLMGetDatab.
  XPLMDataRef gpsDistance_ = nullptr;
  XPLMDataRef gpsBearing_ = nullptr;
  XPLMDataRef gpsNavId_ = nullptr;

  // Decoded Morse idents of the stations being received on NAV1/2 (byte[]
  // strings). navN_dme_id is the fallback for standalone DME/TACAN (XP12).
  XPLMDataRef nav1NavId_ = nullptr;
  XPLMDataRef nav2NavId_ = nullptr;
  XPLMDataRef nav1DmeId_ = nullptr;
  XPLMDataRef nav2DmeId_ = nullptr;

  // NAV/COM active + standby frequency datarefs (int) read each frame so the
  // glass shows the live radios, and written by the bezel tuning above. NAV
  // uses the MHz x 100 datarefs; COM uses the 8.33 kHz-capable datarefs (channel
  // in kHz) so .x25/.x75 channels are not truncated. Indexed by RadioUnit;
  // activeMember/standbyMember point at the matching FlightData fields, and
  // mhzToInt records the dataref's MHz scale.
  struct RadioRef {
    XPLMDataRef active = nullptr;
    XPLMDataRef standby = nullptr;
    float FlightData::* activeMember = nullptr;
    float FlightData::* standbyMember = nullptr;
    // Dataref-integer value per MHz: 100 for the legacy NAV MHz x 100 datarefs,
    // 1000 for the 8.33 kHz-capable COM datarefs (channel in kHz). MHz =
    // value / mhzToInt; value = lround(MHz * mhzToInt).
    float mhzToInt = 100.0f;
    // Per-radio audio volume (float 0..1), read each frame and written by the
    // VOL/SQ / VOL/ID knobs.
    XPLMDataRef volume = nullptr;
    float FlightData::* volumeMember = nullptr;
    // NAV Morse-ident audio selection (int 0/1), toggled by the NAV VOL/ID
    // push. Null for COM rows (the COM VOL/SQ push is inert).
    XPLMDataRef identAudio = nullptr;
    bool FlightData::* identMember = nullptr;
  };
  std::array<RadioRef, 4> radios_{};

  // Transponder code (0000-7777) and mode enum, read each frame and written by
  // the XPDR softkeys.
  XPLMDataRef transponderCode_ = nullptr;
  XPLMDataRef transponderMode_ = nullptr;

  // Master (battery) and avionics master switch states, gating the PFD/MFD
  // power-up to mirror the real-world G1000 (master -> PFD, avionics -> MFD).
  XPLMDataRef batteryMasterOn_ = nullptr;
  XPLMDataRef avionicsPowerOn_ = nullptr;

  // Per-instrument X-Plane failure datarefs (failure_enum, 6 = inoperative).
  // Each drives one FlightData validity flag so the affected gauge/box draws
  // its red-X annunciation (AHRS feeds attitude+heading; the ADC feeds the
  // air-data tapes; the radios and transponder fail independently).
  XPLMDataRef failAttitude_ = nullptr;
  XPLMDataRef failHeading_ = nullptr;
  XPLMDataRef failAirspeed_ = nullptr;
  XPLMDataRef failAltimeter_ = nullptr;
  XPLMDataRef failVerticalSpeed_ = nullptr;
  XPLMDataRef failNav1_ = nullptr;
  XPLMDataRef failNav2_ = nullptr;
  XPLMDataRef failCom1_ = nullptr;
  XPLMDataRef failCom2_ = nullptr;
  XPLMDataRef failTransponder_ = nullptr;

  // Engine Indication System (EIS) strip: tachometer, fuel flow, oil
  // pressure/temperature, EGT, vacuum, fuel quantity, engine hours and the
  // volt/ammeter rows. Resolved once into a data-driven table (mirroring the
  // standalone's UDP bindings) and read each frame. Several of these are
  // per-engine/per-tank/per-bus float[] arrays, so each binding records the
  // element index (-1 for a scalar dataref) and a unit conversion applied as
  // value * scale + offset (e.g. deg C -> deg F, kg/sec -> GPH).
  struct EisBinding {
    XPLMDataRef ref = nullptr;
    int arrayIndex = -1;
    float scale = 1.0f;
    float offset = 0.0f;
    std::string channel;
  };
  std::vector<EisBinding> eisBindings_;

  EisSource* eisSource_ = nullptr;
  ChecklistSource* checklistSource_ = nullptr;
  const ObstacleStore* obstacles_ = nullptr;  // optional bundled FAA DDOF (US)
  static inline const EisLayout emptyEis_{};
  static inline const ChecklistData emptyChecklists_{};
  std::string lastAircraftAcfPath_;
  std::string lastAircraftIcao_;
  bool eisWasReady_ = false;
  XPLMDataRef acfRelativePath_ = nullptr;
  XPLMDataRef acfIcao_ = nullptr;
  XPLMDataRef acfVso_ = nullptr;
  XPLMDataRef acfVs_ = nullptr;
  XPLMDataRef acfVfe_ = nullptr;
  XPLMDataRef acfVno_ = nullptr;
  XPLMDataRef acfVne_ = nullptr;

  // Display-only Direct-To course (the live FMS route is unchanged).
  bool directToActive_ = false;
  bool directToHold_ = false;
  MapLeg directTo_;
  bool directToOriginPending_ = false;
  bool directToOriginValid_ = false;
  double directToOriginLat_ = 0.0;
  double directToOriginLon_ = 0.0;

  // Authoritative route from PFD/MFD edits until cleared. Without this the sim
  // FMS readback replaces typed idents with coordinate strings (+27-81) for
  // fixes stored as lat/lon entries.
  std::vector<MapLeg> routeOverride_;
  bool routeOverrideSet_ = false;

  XPLMDataRef gpsHdef_ = nullptr;
  XPLMDataRef hsiObsCourse_ = nullptr;
  XPLMDataRef nav1ObsCourse_ = nullptr;
  XPLMDataRef nav2ObsCourse_ = nullptr;
  XPLMDataRef selectedHeading_ = nullptr;
  XPLMDataRef baroSetting_ = nullptr;
  XPLMDataRef hsiSourceSelect_ = nullptr;
  XPLMDataRef overrideGps_ = nullptr;
  XPLMDataRef gpsCourseDegMag_ = nullptr;
  XPLMDataRef gpsHdefDot_ = nullptr;
  XPLMDataRef gpsDmeDistOverride_ = nullptr;
  float lastPushedCourseDeg_ = -999.0f;
  float lastSentGpsCourseDeg_ = -999.0f;
  float lastSentGpsHdefDots_ = 999.0f;
  float lastSentGpsDmeDistNm_ = -1.0f;
  float lastSentGpsHdefNmPerDot_ = -1.0f;
  std::string lastSentGpsNavId_;
  int lastSyncedFmsLegIndex_ = -1;
  int lastGpsCoupledLegIndex_ = -1;
  std::string lastGpsCoupledToWpt_;
  int lastSentHsiSource_ = -1;
  bool gpsOverrideActive_ = false;
  bool lastNavigatorDirectTo_ = false;

  void resetGpsCouplingState();
  void ensureSimCdiSource(CdiSource source);
};

}  // namespace avionics
