#pragma once

namespace avionics {

// Manual display-backup (reversionary) mode driven by the audio panel's red
// DISPLAY BACKUP button (G1000 NXi Pilot's Guide / X-Plane
// sim/GPS/G1000_display_reversion). When active the PFD adds the EIS strip and
// the MFD presents PFD instruments; deactivating waits five seconds before
// returning to normal (pressing again during the delay resets the timer).
inline constexpr double kDisplayBackupExitDelaySeconds = 5.0;

class DisplayBackupState {
 public:
  void update(double dtSeconds);
  void toggle();
  bool active() const { return active_; }

 private:
  bool active_ = false;
  double exitDelayRemaining_ = 0.0;
};

}  // namespace avionics
