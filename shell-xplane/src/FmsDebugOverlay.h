#pragma once

namespace avionics {

// Live navigation state mirrored from the NXi data source each frame.
struct FmsDebugNavState {
  int activeLegIndex = -1;
  int planLegCount = 0;
  char fmaFrom[16] = {};
  char fmaTo[16] = {};
  float courseDeg = 0.0f;
  float cdiDots = 0.0f;
  bool gpsOverride = false;
  bool obsMode = false;
  bool directTo = false;
};

// On-screen overlay for flight-plan / FMS integration debugging. Records
// outbound writes to the sim FMS and GPS overrides, and compares them to the
// live X-Plane FMS destination each frame.
class FmsDebugOverlay {
 public:
  static void setEnabled(bool enabled);
  static bool enabled();

  static void registerDrawCallback();
  static void unregisterDrawCallback();

  static void recordWrite(const char* kind, const char* detail);
  static void updateNavState(const FmsDebugNavState& state);
};

}  // namespace avionics
