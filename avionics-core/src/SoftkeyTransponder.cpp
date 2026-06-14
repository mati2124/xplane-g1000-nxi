#include "avionics/SoftkeyController.h"

// Transponder: the IDNT annunciation start (Pilot's Guide, Ident Function) and
// the mode/code commits the sim feed consumes. Mode values use the X-Plane
// transponder_mode enum (off=0, stdby=1, on=2, alt=3).
namespace avionics {
namespace {

// IDNT annunciation duration after the Ident key (Pilot's Guide, Ident
// Function).
constexpr double kIdentSeconds = 18.0;

}  // namespace

void SoftkeyController::startIdent() {
  // In Standby the Ident key is inoperative (Pilot's Guide, Ident Function).
  if (xpdrMode_ == XpdrMode::Standby) return;
  identSecondsLeft_ = kIdentSeconds;
  // From the Mode or Code selection softkeys, Ident reverts to the top level.
  if (currentMenu() == SoftkeyMenu::Xpdr ||
      currentMenu() == SoftkeyMenu::XpdrCode) {
    xpdrPending_.clear();
    menuStack_.resize(1);
    rebuildLabels();
  }
}

int SoftkeyController::xpdrModeToSim(XpdrMode mode) {
  switch (mode) {
    case XpdrMode::Standby:
      return 1;
    case XpdrMode::On:
      return 2;
    case XpdrMode::Alt:
      return 3;
    case XpdrMode::Ground:
      return 1;
  }
  return 3;
}

void SoftkeyController::queueXpdrModeCommit() {
  xpdrModeCommit_ = xpdrModeToSim(xpdrMode_);
  xpdrModeCommitPending_ = true;
}

bool SoftkeyController::consumeXpdrCodeCommit(int& code) {
  if (!xpdrCodeCommitPending_) return false;
  xpdrCodeCommitPending_ = false;
  code = xpdrCodeCommit_;
  return true;
}

bool SoftkeyController::consumeXpdrModeCommit(int& mode) {
  if (!xpdrModeCommitPending_) return false;
  xpdrModeCommitPending_ = false;
  mode = xpdrModeCommit_;
  return true;
}

}  // namespace avionics
