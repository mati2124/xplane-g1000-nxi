#pragma once

#include <cstdint>

#include "avionics/Checklist.h"
#include "avionics/ConnectionState.h"
#include "avionics/DisplayBackup.h"
#include "avionics/Eis.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

class FmsNavigator;

// Abstraction over "where flight data comes from".
//
// The X-Plane shell implements this by reading datarefs in-process; the
// standalone shell implements it over the network (X-Plane Web API / UDP)
// with client-side interpolation. The core never knows which it is talking to.
class DataSource {
 public:
  virtual ~DataSource() = default;

  // Advance/refresh the cached state. dtSeconds is wall-clock time since the
  // previous call, which network-backed sources use to interpolate.
  virtual void update(double dtSeconds) = 0;

  // The latest decoded state. Must be cheap; called once per rendered frame.
  virtual const FlightData& snapshot() const = 0;

  // Slow-changing map/navigation snapshot for the inset map and future MFD MAP
  // page. Default is empty; shells that can populate nav data override this.
  virtual const MapData& mapSnapshot() const { return emptyMap_; }

  // Airport diagram geometry generation; shells with a render cache compare
  // this each frame and invalidate when it changes.
  virtual std::uint32_t mapGeometryEpoch() const { return 0; }

  // Author-supplied checklists for the MFD Checklist page group. Default is
  // empty; shells that can locate the aircraft's checklist file override this.
  virtual const ChecklistData& checklistSnapshot() const {
    return emptyChecklist_;
  }

  // Resolved path of the loaded checklist file, when the shell found and read
  // one. Empty when no checklist file was located.
  virtual std::string checklistSourcePath() const { return {}; }

  // Per-aircraft engine display layout (g1000_eis.txt beside the .acf).
  virtual const EisLayout& eisLayoutSnapshot() const { return emptyEis_; }

  // Map panning: when the MFD MAP page's Map Pointer is active the map view
  // recenters on the pointer instead of ownship, so the feature/airspace/etc.
  // layers must be queried around that pointer too -- otherwise the data stays
  // around the aircraft and the panned-to area comes up empty. Shells push the
  // MFD controller's pointer state here each frame; sources that build the map
  // snapshot query around this center (when active) instead of ownship.
  // Sources without a spatial map layer ignore it (the default no-op).
  virtual void setMapPanCenter(bool /*active*/, double /*lat*/,
                               double /*lon*/) {}

  // MFD Direct-To inset: when the window is open on a far-away target, land and
  // nav features must be queried around that target so the inset chart is not
  // blank ocean. Sources populate MapData::inset*; MapView reads them when the
  // inset requests useInsetMapData.
  virtual void setInsetMapQuery(bool /*active*/, double /*lat*/, double /*lon*/,
                                float /*rangeNm*/, float /*viewHalfExtentNm*/,
                                const std::string& /*targetIdent*/ = {}) {}

  // MFD map range ladder step (NM). Land-layer queries use this for range-tier
  // selection (continental silhouettes vs high-res detail) and geographic overlap.
  virtual void setChartRangeNm(float /*rangeNm*/) {}

  // Corner reach of the MFD map viewport in NM (half the diagonal). Land queries
  // use this so GSHHG lon-band overlap matches what MapView draws above ~100 NM.
  virtual void setMapViewHalfExtentNm(float /*halfExtentNm*/) {}

  // Health of this source. Sources that are always available (the mock feed,
  // the in-process dataref reader) keep the default; network-backed sources
  // override it so the display can show the boot / connection-lost screens.
  virtual ConnectionState connectionState() const {
    return ConnectionState::Connected;
  }

  // Whether the MFD power-up page must be acknowledged with the ENT key before
  // the live pages appear, as on a real unit (the pilot confirms database
  // currency). Live sim links keep the default; synthetic feeds (the mock)
  // override it to false so the boot sequence advances on its own without a
  // keypress.
  virtual bool requiresPowerUpAcknowledge() const { return true; }

  // Loaded-aircraft identity for per-airframe assets (boot hero, EIS, checklists).
  virtual std::string aircraftIcaoType() const { return {}; }
  virtual std::string aircraftAcfRelativePath() const { return {}; }

  // Manual display-backup (reversionary) mode from the audio panel's red
  // DISPLAY BACKUP button. Shared by both GDU engines when they read the same
  // DataSource.
  void toggleDisplayBackup() { displayBackup_.toggle(); }

  // Reconcile GPS leg DTK/CDI with the active flight plan via FmsNavigator.
  // Called from the avionics engine after each source update.
  virtual void applyGpsNavigation(FmsNavigator& /*navigator*/, bool /*obsMode*/,
                                  CdiSource /*cdiSource*/, float /*nmPerDot*/) {}

  // Push the active flight-plan leg index into the simulator FMS destination
  // (XPLMSetDestinationFMSEntry). Default no-op; X-Plane shells override.
  virtual void syncSimulatorActiveLeg(int /*legIndex*/) {}

 protected:
  // Call from update() to advance the exit-delay timer and publish the flag on
  // `data`.
  void syncDisplayBackup(FlightData& data, double dtSeconds) {
    displayBackup_.update(dtSeconds);
    data.displayBackupActive = displayBackup_.active();
  }

 private:
  DisplayBackupState displayBackup_;
  static inline const MapData emptyMap_{};
  static inline const ChecklistData emptyChecklist_{};
  static inline const EisLayout emptyEis_{};
};

}  // namespace avionics
