#include "avionics/DisplayBackup.h"

namespace avionics {

void DisplayBackupState::update(double dtSeconds) {
  if (exitDelayRemaining_ <= 0.0) return;
  exitDelayRemaining_ -= dtSeconds;
  if (exitDelayRemaining_ <= 0.0) {
    active_ = false;
    exitDelayRemaining_ = 0.0;
  }
}

void DisplayBackupState::toggle() {
  if (active_) {
    // Exiting: start or reset the five-second hold before normal mode returns.
    exitDelayRemaining_ = kDisplayBackupExitDelaySeconds;
    return;
  }
  active_ = true;
  exitDelayRemaining_ = 0.0;
}

}  // namespace avionics
