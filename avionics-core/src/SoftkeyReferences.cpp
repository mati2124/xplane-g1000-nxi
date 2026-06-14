#include "avionics/SoftkeyController.h"

#include <algorithm>

// Timer/References window (Pilot's Guide Fig. 2-6 / Table 2-1): the generic
// timer command, the V-speed reference bugs, and the Minimums (MDA/DH) source
// and altitude. The FMS knob moves the field cursor and edits the highlighted
// value; ENT activates a field.
namespace avionics {
namespace {

// Minimums altitude entry: small-knob step and the settable range (Pilot's
// Guide: "from zero to 16,000 feet").
constexpr float kMinsStepFt = 100.0f;
constexpr float kMinsMaxFt = 16000.0f;

}  // namespace

const char* SoftkeyController::timerCommandLabel() const {
  if (timerRunning_) return "Stop?";
  return timerSeconds() > 0 ? "Reset?" : "Start?";
}

void SoftkeyController::moveReferencesCursor(int step) {
  // The cursor walks every field except MinsValue, which is only reachable
  // via ENT from the MINS mode field once BARO is selected (mirroring the
  // real unit, where ENT highlights the next field after a selection).
  // Turning the large knob while on the altitude steps off it.
  const int lastField = static_cast<int>(RefField::MinsMode);
  int cur = (refCursor_ == RefField::MinsValue ||
             refCursor_ == RefField::MinsTemp)
                ? static_cast<int>(RefField::MinsMode)
                : static_cast<int>(refCursor_);
  cur += step;
  if (cur < 0) cur = lastField;
  if (cur > lastField) cur = 0;
  refCursor_ = static_cast<RefField>(cur);
}

void SoftkeyController::adjustReferencesValue(int step) {
  // Small FMS knob: the MINS altitude and the V-speed reference values are the
  // numeric fields (the On/Off selections are activated with ENT).
  if (refCursor_ == RefField::MinsValue) {
    minsAltFt_ = std::max(
        0.0f, std::min(kMinsMaxFt, minsAltFt_ + step * kMinsStepFt));
    return;
  }
  if (refCursor_ == RefField::MinsTemp) {
    minsTempC_ = std::max(
        kMinsTempMinC,
        std::min(kMinsTempMaxC, minsTempC_ + step * kMinsTempStepC));
    return;
  }
  if (refCursor_ == RefField::Glide || refCursor_ == RefField::Vr ||
      refCursor_ == RefField::Vx || refCursor_ == RefField::Vy) {
    const int v = static_cast<int>(refCursor_) - static_cast<int>(RefField::Glide);
    vspeedKt_[v] = std::max(
        kVspeedMinKt,
        std::min(kVspeedMaxKt, vspeedKt_[v] + step * kVspeedStepKt));
  }
}

void SoftkeyController::activateReferencesField() {
  switch (refCursor_) {
    case RefField::TimerCmd:
      // Start? -> Stop? -> Reset? cycle (Pilot's Guide, Generic Timer).
      if (timerRunning_) {
        timerRunning_ = false;
      } else if (timerSeconds() > 0) {
        timerSeconds_ = 0.0;
      } else {
        timerRunning_ = true;
      }
      break;
    case RefField::Glide:
    case RefField::Vr:
    case RefField::Vx:
    case RefField::Vy: {
      const int v = static_cast<int>(refCursor_) - static_cast<int>(RefField::Glide);
      vspeedOn_[v] = !vspeedOn_[v];
      break;
    }
    case RefField::MinsMode:
      // Cycle the minimum source Off -> BARO -> TEMP -> Off. Selecting a source
      // highlights the next field (the altitude), per the real unit.
      switch (minsMode_) {
        case MinimumsMode::Off:
          minsMode_ = MinimumsMode::Baro;
          refCursor_ = RefField::MinsValue;
          break;
        case MinimumsMode::Baro:
          minsMode_ = MinimumsMode::Temp;
          refCursor_ = RefField::MinsValue;
          break;
        case MinimumsMode::Temp:
          minsMode_ = MinimumsMode::Off;
          break;
      }
      break;
    case RefField::MinsValue:
      // In TEMP COMP, ENT advances to the destination-temperature field.
      refCursor_ = (minsMode_ == MinimumsMode::Temp) ? RefField::MinsTemp
                                                     : RefField::TimerCmd;
      break;
    case RefField::MinsTemp:
      refCursor_ = RefField::TimerCmd;  // entry accepted
      break;
    case RefField::Count:
      break;
  }
}

}  // namespace avionics
