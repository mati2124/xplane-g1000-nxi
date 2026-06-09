#pragma once

namespace avionics {

// Health of the link to whatever is feeding flight data. Kept separate from the
// instrument-level validity flags in FlightData (navSignalValid, windValid,
// ...) because this describes the data pipe itself, not an individual gauge.
enum class ConnectionState {
  // Trying to reach the source; no data has arrived yet (e.g. waiting for the
  // simulator to start).
  Connecting,
  // Receiving fresh data.
  Connected,
  // Data was flowing but has gone stale / the link dropped. Drives the red-X
  // failure annunciation on the display.
  Disconnected,
};

}  // namespace avionics
