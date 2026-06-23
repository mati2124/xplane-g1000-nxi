#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "AirspaceStore.h"
#include "AirwayStore.h"
#include "AptDatStore.h"
#include "FlightPlanBridgeClient.h"
#include "FmsPlanStore.h"
#include "LandDataStore.h"
#include "ObstacleStore.h"
#include "NavData.h"
#include "XPlaneWebApi.h"
#include "avionics/Checklist.h"
#include "avionics/Eis.h"
#include "avionics/MapData.h"
#include "avionics/MapRange.h"
#include "avionics/NexradWeatherRadar.h"
#include "avionics/Radio.h"
#include "avionics/SimulatorConnection.h"
#include "avionics/Terrain.h"

namespace avionics {

struct GlidepathSolution;

// Live link to X-Plane over its UDP RREF protocol.
//
// Why UDP RREF instead of the 12.1+ Web API (WebSocket/JSON): RREF needs no
// third-party WebSocket/JSON dependency (keeping the project dependency-light),
// works on X-Plane 11 and 12, and makes connection-loss detection trivial -- if
// no packets arrive within the stale timeout the link is considered down and
// the display shows the red X.
//
// On construction we open a non-blocking UDP socket and send RREF subscription
// requests for the datarefs we care about. X-Plane then streams back
// (index, float value) pairs which we decode into FlightData. All socket I/O is
// drained on the render thread inside update(), so no extra thread is needed.
//
// This is a SimulatorConnection so a future Microsoft Flight Simulator backend
// (SimConnect) can drop in behind the same interface.
class XPlaneConnection : public SimulatorConnection {
 public:
  // The database stores and fmsPlan are owned by the caller and shared with the
  // mock feed (via ShellNavMapData) so the large databases -- notably the
  // Global Airports apt.dat -- and the flight plan are parsed only once.
  XPlaneConnection(std::string host, std::uint16_t port, NavDataStore& navData,
                   AirspaceStore& airspace, AirwayStore& airways,
                   AptDatStore& aptData, LandDataStore& landData,
                   FmsPlanStore& fmsPlan, const TerrainSource* terrain = nullptr,
                   ChecklistSource* checklists = nullptr,
                   EisSource* eis = nullptr,
                   const ObstacleStore* obstacles = nullptr,
                   std::uint16_t bridgePort = fpbridge::kDefaultPort,
                   bool fmsWriteEnabled = true);
  ~XPlaneConnection() override;

  XPlaneConnection(const XPlaneConnection&) = delete;
  XPlaneConnection& operator=(const XPlaneConnection&) = delete;

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
  ConnectionState connectionState() const override;
  const char* simulatorName() const override { return "X-PLANE"; }

  // Show a pilot-built route on the map feed without programming X-Plane's FMS.
  void setLocalFlightPlan(std::vector<MapLeg> route) {
    routeOverride_ = std::move(route);
    routeOverrideSet_ = true;
    map_.flightPlan = routeOverride_;
  }

  // Replaces the .fms-file flight plan with an externally supplied route
  // (a SimBrief OFP or a completed FPL edit). Once set it stays authoritative --
  // an empty vector shows an empty plan (a deleted flight plan) rather than
  // falling back to the .fms file. When FMS write-back is enabled and the
  // plugin bridge is reachable, the route is also programmed into X-Plane's
  // FMS (an empty route clears the FMS plan).
  void setRouteOverride(std::vector<MapLeg> route) {
    routeOverride_ = std::move(route);
    routeOverrideSet_ = true;
    map_.flightPlan = routeOverride_;
    if (routeOverride_.empty()) setDirectTo({});
    if (fmsWriteEnabled_) fmsBridge_.writePlan(routeOverride_);
  }

  void clearRouteOverride() {
    routeOverride_ = {};
    routeOverrideSet_ = false;
  }

  // Active GPS Direct-To target for the map's magenta direct course. When FMS
  // write-back is enabled and the plugin bridge is reachable, this also engages
  // a present-position Direct-To in X-Plane's FMS; otherwise it drives the
  // display only. An empty id clears the direct course.
  void setDirectTo(MapLeg target);
  void clearDirectTo();

  void setMapPanCenter(bool active, double lat, double lon) override;
  void setChartRangeNm(float rangeNm) override;
  void setMapViewHalfExtentNm(float halfExtentNm) override;

  std::string aircraftIcaoType() const override { return lastAircraftIcao_; }
  std::string aircraftAcfRelativePath() const override {
    return lastAircraftAcfPath_;
  }

  // Pilot commands from the PFD bezel / softkeys (UDP DREF writes).
  void tuneRadioStandby(RadioUnit unit, float standbyMhz);
  void transferRadio(RadioUnit unit);
  void setRadioVolume(RadioUnit unit, float volume);
  void setNavIdent(RadioUnit unit, bool on);
  void setTransponderCode(int code);
  void setTransponderMode(int mode);
  // Dedicated HDG / CRS / BARO knob commits (degrees magnetic, inches Hg).
  void setHeadingBug(float deg);
  void setSelectedCourse(float deg);
  void setBaroInHg(float inHg);

  void applyGpsNavigation(bool obsMode, CdiSource cdiSource,
                          float nmPerDot) override;

 private:
  void sendDataref(const char* path, float value);
  void sendSubscriptions(int frequencyHz);
  void drainSocket();
  void rebuildEisSubscriptions();
  void subscribeEisBindings(int frequencyHz);
  void updateAircraftProfile();

  // Refresh the moving-map snapshot (ownship position + nearby features). The
  // feature list is range-filtered from the nav database on a throttled timer,
  // not every frame, since the database holds the whole world.
  void updateMap(double dtSeconds);

  // Advance the locally-ticked zulu (UTC) clock and write it into data_'s
  // hour/minute/second fields. Decouples the displayed clock from packet
  // cadence so the seconds never skip on UDP jitter/loss.
  void updateZuluClock(double dtSeconds);

  // Turn the latest raw autopilot mode statuses into the FMA strings/value
  // (lateral + vertical active/armed modes and the cyan altitude reference)
  // shown on the top bar.
  void updateFmaModes();

  // Glideslope, marker beacon, and DME fields from the nav radio indicators.
  void updateNavInstrumentation();

  // Standalone-only: feed CIFP-computed GPS glidepath into X-Plane and capture
  // GS when APP mode is armed but the sim has no RNAV vertical signal.
  void updateGpsGlidepathCoupling();

  // Flight plan shown on the map/FPL (same precedence as updateMap).
  std::vector<MapLeg> displayedFlightPlan() const;

  // Drop the local Direct-To display override without reprogramming the FMS
  // (X-Plane may already have sequenced past the DTO fix).
  void releaseDirectToOverride();

  // Fill directTo_ lat/lon from the nav database when the Direct-To window
  // supplied only an ident (needed for map course + DIS/BRG away from the target).
  void ensureDirectToCoords();

  // Keep directToActive_ in sync with X-Plane sequencing / local arrival.
  void syncDirectToWithSimulator(const std::string& simDestination,
                                 const std::vector<MapLeg>& plan);

  // data_ is the smoothed state returned by snapshot(); target_ holds the most
  // recent values decoded from packets, which data_ is eased toward each frame.
  FlightData data_;
  FlightData target_;
  bool primed_ = false;  // false until the first data snaps data_ to target_
  float prevTargetAirspeedKts_ = 0.0f;  // last IAS target from RREF packets
  double lastAirspeedTargetSeconds_ = 0.0;  // elapsed time at that packet
  float lastInstTrendKts_ = 0.0f;  // 6 s projection from latest packet delta

  // Continuous zulu (UTC) seconds-since-midnight. zuluTargetSec_ is the latest
  // value from X-Plane; zuluDisplaySec_ free-runs locally and is eased toward it
  // so the rendered clock ticks smoothly between (and through lost) packets.
  double zuluTargetSec_ = 0.0;
  double zuluDisplaySec_ = 0.0;
  bool zuluHasTarget_ = false;  // false until the first zulu packet arrives
  bool zuluPrimed_ = false;     // false until display snaps to the first value

  // Latest raw autopilot mode-status ints (0=off, 1=armed, 2=active), indexed
  // by the ApMode enum in the .cpp. Accumulated across packets and decoded into
  // the FMA fields each frame by updateFmaModes(). The count is mirrored by a
  // static_assert against the subscription table.
  static constexpr int kApModeStatusCount = 11;
  int apModeStatus_[kApModeStatusCount] = {};

  // Latest GPS lateral CDI sensitivity (NM per dot). The annunciated GPS flight
  // phase (ENR/TERM/APR/OCN) is derived from it each frame; 0 means no usable
  // GPS scale (no active flight plan), which blanks the phase annunciation.
  float gpsHdefNmPerDot_ = 0.0f;

  // Ownship position decoded from RREF (float precision; ~1-2 m at these
  // magnitudes, fine for the inset map). havePosition_ stays false until the
  // first lat AND lon packets arrive so the map shows "NO GPS POSITION".
  float ownshipLatDeg_ = 0.0f;
  float ownshipLonDeg_ = 0.0f;
  bool haveLat_ = false;
  bool haveLon_ = false;

  // Latest raw TCAS target fields straight off the RREF stream, one row per
  // tracked target slot: [lat deg, lon deg, elevation m, vertical speed fpm].
  // Decoded into MapTraffic entries by updateMap(). Slots X-Plane is not using
  // report (0, 0) lat/lon and are skipped.
  static constexpr int kTrafficSlotCount = 8;
  static constexpr int kTrafficSlotFields = 4;
  float trafficRaw_[kTrafficSlotCount][kTrafficSlotFields] = {};

  static constexpr int kNavInstrCount = 9;
  float navInstr_[kNavInstrCount] = {};

  // Moving-map snapshot and its nearby-feature rebuild timer. navData_ and
  // fmsPlan_ are shared (owned by the shell, also used by the mock feed).
  MapData map_;
  const TerrainSource* terrain_ = nullptr;

  // Live datalink NEXRAD for the map precipitation overlay (real ground radar).
  NexradWeatherRadar nexrad_;

  // Database stores are owned by the shell and shared (also used by the mock
  // feed through ShellNavMapData) so each is loaded only once.
  NavDataStore& navData_;
  AirspaceStore& airspace_;
  AirwayStore& airways_;
  AptDatStore& aptData_;
  LandDataStore& landData_;
  const ObstacleStore* obstacles_ = nullptr;  // optional, owned by the shell
  FmsPlanStore& fmsPlan_;
  std::vector<MapLeg> routeOverride_;  // takes precedence over fmsPlan_
  bool routeOverrideSet_ = false;      // override active (even when empty)
  MapLeg directTo_;                    // display-only Direct-To target
  bool directToActive_ = false;
  bool directToOriginPending_ = false;
  bool directToOriginValid_ = false;
  double directToOriginLat_ = 0.0;
  double directToOriginLon_ = 0.0;
  ChecklistSource* checklists_ = nullptr;
  EisSource* eisSource_ = nullptr;
  static inline const ChecklistData emptyChecklists_{};
  static inline const EisLayout emptyEis_{};

  struct RuntimeEisBinding {
    std::string channel;
    std::string datarefPath;
    float scale = 1.0f;
    float offset = 0.0f;
    int subIndex = -1;
    float target = 0.0f;
  };
  std::vector<RuntimeEisBinding> eisBindings_;
  std::size_t eisLayoutBindingCount_ = 0;
  bool eisWasReady_ = false;
  std::string lastAircraftIcao_;
  std::string lastAircraftAcfPath_;

  double sinceMapRebuildSeconds_ = 0.0;

  // MFD Map Pointer (pan) state pushed by the shell. When active, the nearby-
  // data scans center on (mapPanLat_, mapPanLon_) instead of ownship so the
  // panned-to area has data. mapPanDirty_ forces an immediate rebuild when the
  // pointer is toggled or moved so panning feels responsive.
  bool mapPanActive_ = false;
  double mapPanLat_ = 0.0;
  double mapPanLon_ = 0.0;
  bool mapPanDirty_ = false;
  float chartRangeNm_ = mapRangeNmAt(kMapRangeDefaultIndex);
  float mapViewHalfExtentNm_ = 0.0f;

  std::string host_;
  std::uint16_t port_;

  // String datarefs (the GPS destination identifier) can't ride the float-only
  // RREF stream, so they come from X-Plane's Web API on a background thread.
  XPlaneWebApi webApi_;

  // Live FMS flight plan from the in-sim plugin's UDP bridge (the FMS route is
  // not on the RREF stream or the Web API). When the bridge is reachable and
  // has a non-empty route it supersedes the .fms-file plan; otherwise the
  // .fms fallback still applies, so installing the bridge is purely additive.
  // The bridge is also the write path: shell-side route / Direct-To edits are
  // programmed back into X-Plane's FMS through it (unless fmsWriteEnabled_ is
  // false, e.g. --no-fms-write).
  FlightPlanBridgeClient fmsBridge_;
  bool fmsWriteEnabled_ = true;

  float lastSentGpsVdefDots_ = 999.0f;
  float lastSentGpsHdefDots_ = 999.0f;
  float lastSentGpsCourseDeg_ = -999.0f;
  float lastSentGsTrackVsFpm_ = 99999.0f;
  bool gpsGlidepathHasSignal_ = false;
  bool gpsGlidepathCaptured_ = false;
  bool gpsGlidepathPitchSteering_ = false;
  bool gpsOverrideActive_ = false;
  bool apOverrideForGsActive_ = false;
  float lastPushedCourseDeg_ = -999.0f;

  void setGpsOverride(bool active);
  void setApOverrideForGs(bool active);
  void engageGsCapture(const GlidepathSolution& gp);

  // Native socket handle stored width-safe: -1 is "invalid" on both POSIX (int
  // fd) and Windows (SOCKET, where INVALID_SOCKET is all-ones == -1).
  std::intptr_t socketHandle_ = -1;

  double elapsedSeconds_ = 0.0;
  double lastPacketSeconds_ = -1.0;       // < 0 until the first packet arrives
  double sinceResubscribeSeconds_ = 0.0;
  bool everConnected_ = false;
};

}  // namespace avionics
