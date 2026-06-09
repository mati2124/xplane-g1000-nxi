#pragma once

#include "avionics/DataSource.h"

namespace avionics {

// Generic, transport-agnostic link to an external flight simulator.
//
// This is the seam that keeps the rest of the app from depending on any one
// simulator. X-Plane (via the UDP RREF protocol / the 12.1+ Web API) implements
// it today; a future Microsoft Flight Simulator backend would implement the
// same interface over SimConnect. Each backend decodes the simulator's native
// data into the shared FlightData and reports its own ConnectionState, so the
// engine, boot screen, and connection-lost screen work unchanged regardless of
// which simulator is attached.
class SimulatorConnection : public DataSource {
 public:
  // Short, human-readable label for the boot / status screens, e.g. "X-PLANE"
  // or "MSFS".
  virtual const char* simulatorName() const = 0;
};

}  // namespace avionics
