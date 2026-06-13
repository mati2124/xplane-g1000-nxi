#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <array>

#include "DsfTerrainStore.h"
#include "DatarefWeatherRadar.h"
#include "XPLMDataAccess.h"
#include "avionics/AptDatParser.h"
#include "avionics/DataSource.h"
#include "avionics/Eis.h"
#include "avionics/EisLegacy.h"
#include "avionics/MapData.h"
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

  void setEisSource(EisSource* source) { eisSource_ = source; }

  // Called from the plugin draw path after the MFD engine state is known. Pushes
  // the EFIS weather mode, antenna tilt, and sector width to the sim based on
  // the MFD's NEXRAD overlay state and dedicated Weather Radar page controls.
  void syncWeatherRadar(const MfdController& ui);

  // Whether the current airframe carries a weather radar (probed from the sim's
  // radar return texture; see DatarefWeatherRadar::equipped). The MFD uses this
  // to hide its dedicated Weather Radar page on unequipped aircraft.
  bool weatherRadarEquipped() const { return weather_.equipped(); }

  void setMapPanCenter(bool active, double lat, double lon) override;

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

  // apt.dat airport metadata (tower/fuel/kind and published comm frequencies).
  // Used by the plugin NavFeatureSource for COM frequency decode and WPT/NRST
  // frequency lists once the background apt.dat load finishes.
  bool aptDatReady() const { return aptDatLoaded_.load(); }
  std::vector<MapAirportFrequency> airportFrequencies(
      const std::string& icao) const;

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

  void rebuildEisBindings();
  void updateAircraftEisPath();

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

  XPLMDataRef airspeed_ = nullptr;
  XPLMDataRef altitude_ = nullptr;
  XPLMDataRef heading_ = nullptr;
  XPLMDataRef pitch_ = nullptr;
  XPLMDataRef roll_ = nullptr;
  XPLMDataRef verticalSpeed_ = nullptr;
  XPLMDataRef slip_ = nullptr;

  // Ownship geographic position for the moving map (read as doubles).
  XPLMDataRef latitude_ = nullptr;
  XPLMDataRef longitude_ = nullptr;

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

  // NAV/COM active + standby frequency datarefs (int, value = MHz x 100) read
  // each frame so the glass shows the live radios, and written by the bezel
  // tuning above. Indexed by RadioUnit; activeMember/standbyMember point at the
  // matching FlightData fields.
  struct RadioRef {
    XPLMDataRef active = nullptr;
    XPLMDataRef standby = nullptr;
    float FlightData::* activeMember = nullptr;
    float FlightData::* standbyMember = nullptr;
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
  static inline const EisLayout emptyEis_{};
  std::string lastAircraftAcfPath_;
  XPLMDataRef acfRelativePath_ = nullptr;
};

}  // namespace avionics
