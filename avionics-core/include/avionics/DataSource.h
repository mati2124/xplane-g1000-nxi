#pragma once

#include "avionics/Checklist.h"
#include "avionics/ConnectionState.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

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

  // Author-supplied checklists for the MFD Checklist page group. Default is
  // empty; shells that can locate the aircraft's checklist file override this.
  virtual const ChecklistData& checklistSnapshot() const {
    return emptyChecklist_;
  }

  // Health of this source. Sources that are always available (the mock feed,
  // the in-process dataref reader) keep the default; network-backed sources
  // override it so the display can show the boot / connection-lost screens.
  virtual ConnectionState connectionState() const {
    return ConnectionState::Connected;
  }

 private:
  static inline const MapData emptyMap_{};
  static inline const ChecklistData emptyChecklist_{};
};

}  // namespace avionics
