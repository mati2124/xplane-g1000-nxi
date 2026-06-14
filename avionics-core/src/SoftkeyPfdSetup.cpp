#include "avionics/SoftkeyController.h"

#include <algorithm>

// PFD Setup Menu (MENU key, Pilot's Guide Fig. 1-18): adjusts the PFD/MFD
// display and key backlighting. The large FMS knob moves the field cursor
// (skipping a row's intensity unless it is in Manual), the small knob edits the
// highlighted field, and ENT confirms.
namespace avionics {

bool SoftkeyController::pfdSetupFieldReachable(PfdSetupField field) const {
  // A row's intensity is only a cursor stop while that row is in Manual (in
  // Auto the photocell drives it, so it is shown but not editable).
  if (field == PfdSetupField::PfdValue) {
    return setupMode_[static_cast<int>(PfdSetupRow::Pfd)] == BacklightMode::Manual;
  }
  if (field == PfdSetupField::MfdValue) {
    return setupMode_[static_cast<int>(PfdSetupRow::Mfd)] == BacklightMode::Manual;
  }
  return true;
}

void SoftkeyController::movePfdSetupCursor(int step) {
  const int n = static_cast<int>(PfdSetupField::Count);
  const int dir = step >= 0 ? 1 : -1;
  int cur = static_cast<int>(setupCursor_);
  // Walk to the next reachable field in the requested direction, wrapping.
  for (int i = 0; i < n; ++i) {
    cur = (cur + dir + n) % n;
    if (pfdSetupFieldReachable(static_cast<PfdSetupField>(cur))) break;
  }
  setupCursor_ = static_cast<PfdSetupField>(cur);
}

void SoftkeyController::adjustPfdSetupValue(int step) {
  const auto toggleTarget = [this](PfdSetupRow row) {
    const int r = static_cast<int>(row);
    setupTarget_[r] = setupTarget_[r] == BacklightTarget::Display
                          ? BacklightTarget::Key
                          : BacklightTarget::Display;
    // The key backlight is Auto-only on the real unit, so selecting Key drops
    // the row back to Auto.
    if (setupTarget_[r] == BacklightTarget::Key) {
      setupMode_[r] = BacklightMode::Auto;
    }
  };
  const auto toggleMode = [this](PfdSetupRow row) {
    const int r = static_cast<int>(row);
    if (setupTarget_[r] == BacklightTarget::Key) return;  // Key: Auto only
    setupMode_[r] = setupMode_[r] == BacklightMode::Auto ? BacklightMode::Manual
                                                         : BacklightMode::Auto;
  };
  const auto adjustValue = [this](PfdSetupRow row, int s) {
    const int r = static_cast<int>(row);
    setupIntensity_[r] = std::max(
        kBacklightMinPct,
        std::min(kBacklightMaxPct, setupIntensity_[r] + s * kBacklightStepPct));
  };

  switch (setupCursor_) {
    case PfdSetupField::PfdTarget:
      toggleTarget(PfdSetupRow::Pfd);
      break;
    case PfdSetupField::MfdTarget:
      toggleTarget(PfdSetupRow::Mfd);
      break;
    case PfdSetupField::PfdMode:
      toggleMode(PfdSetupRow::Pfd);
      break;
    case PfdSetupField::MfdMode:
      toggleMode(PfdSetupRow::Mfd);
      break;
    case PfdSetupField::PfdValue:
      adjustValue(PfdSetupRow::Pfd, step);
      break;
    case PfdSetupField::MfdValue:
      adjustValue(PfdSetupRow::Mfd, step);
      break;
    case PfdSetupField::Count:
      break;
  }
}

void SoftkeyController::activatePfdSetupField() {
  // ENT on a Manual mode field highlights that row's intensity (Pilot's Guide:
  // "select 'Manual' and press the ENT Key. The intensity value is now
  // highlighted."); ENT on the intensity accepts it and steps back to the mode.
  switch (setupCursor_) {
    case PfdSetupField::PfdMode:
      if (setupMode_[static_cast<int>(PfdSetupRow::Pfd)] ==
          BacklightMode::Manual) {
        setupCursor_ = PfdSetupField::PfdValue;
      }
      break;
    case PfdSetupField::MfdMode:
      if (setupMode_[static_cast<int>(PfdSetupRow::Mfd)] ==
          BacklightMode::Manual) {
        setupCursor_ = PfdSetupField::MfdValue;
      }
      break;
    case PfdSetupField::PfdValue:
      setupCursor_ = PfdSetupField::PfdMode;
      break;
    case PfdSetupField::MfdValue:
      setupCursor_ = PfdSetupField::MfdMode;
      break;
    default:
      break;
  }
}

}  // namespace avionics
