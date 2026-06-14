#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>

// Selected Altitude alerting (Pilot's Guide, Altitude Alerting, Fig. 2-32): the
// state machine that flashes the Selected Altitude box for five seconds at each
// transition (within 1000 ft, within 200 ft, post-capture deviation).
namespace avionics {
namespace {

// Selected Altitude alert flash duration (Fig. 2-32).
constexpr double kAltAlertFlashSeconds = 5.0;

// Altitude Alerting thresholds (Pilot's Guide, Altitude Alerting).
constexpr float kAltAlertArmFt = 1000.0f;
constexpr float kAltAlertCaptureFt = 200.0f;
// A Selected Altitude change beyond this re-arms the alerter.
constexpr float kAltAlertRearmFt = 25.0f;

}  // namespace

void SoftkeyController::updateAltAlert(double dtSeconds,
                                       const FlightData& data) {
  if (altAlertFlashLeft_ > 0.0) {
    altAlertFlashLeft_ = std::max(0.0, altAlertFlashLeft_ - dtSeconds);
  }

  // No usable selection or altitude: keep the alerter quietly armed.
  if (std::fabs(data.selectedAltitudeFt) <= 1.0f || !data.altitudeValid) {
    altAlertPhase_ = AltAlertPhase::Armed;
    altAlertSelectedFt_ = data.selectedAltitudeFt;
    altAlertFlashLeft_ = 0.0;
    return;
  }
  // Whenever the Selected Altitude is changed, the Altitude Alerter is reset.
  if (std::fabs(data.selectedAltitudeFt - altAlertSelectedFt_) >
      kAltAlertRearmFt) {
    altAlertPhase_ = AltAlertPhase::Armed;
    altAlertSelectedFt_ = data.selectedAltitudeFt;
    altAlertFlashLeft_ = 0.0;
  }

  const float diff = std::fabs(data.altitudeFt - data.selectedAltitudeFt);
  switch (altAlertPhase_) {
    case AltAlertPhase::Armed:
      if (diff <= kAltAlertArmFt) {
        altAlertPhase_ = (diff <= kAltAlertCaptureFt) ? AltAlertPhase::Within200
                                                      : AltAlertPhase::Within1000;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Within1000:
      if (diff <= kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Within200;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Within200:
      if (altAlertFlashLeft_ <= 0.0) altAlertPhase_ = AltAlertPhase::Captured;
      break;
    case AltAlertPhase::Captured:
      if (diff > kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Deviation;
        altAlertFlashLeft_ = kAltAlertFlashSeconds;
      }
      break;
    case AltAlertPhase::Deviation:
      if (diff <= kAltAlertCaptureFt) {
        altAlertPhase_ = AltAlertPhase::Captured;
        altAlertFlashLeft_ = 0.0;
      } else if (altAlertFlashLeft_ <= 0.0 && diff > kAltAlertArmFt) {
        altAlertPhase_ = AltAlertPhase::Armed;  // left the capture band
      }
      break;
  }
}

SelectedAltStyle SoftkeyController::selectedAltStyle() const {
  SelectedAltStyle s;
  if (altAlertFlashLeft_ <= 0.0) return s;  // steady between transitions
  // During a five-second flash the alert look alternates with the normal look
  // on the blink clock (Pilot's Guide Fig. 2-32).
  switch (altAlertPhase_) {
    case AltAlertPhase::Within1000:
      s.cyanBackground = blinkOn_;  // black text on a cyan plate
      break;
    case AltAlertPhase::Within200:
      s.hideText = !blinkOn_;  // cyan-on-black readout blinks
      break;
    case AltAlertPhase::Deviation:
      s.amberText = true;
      s.hideText = !blinkOn_;
      break;
    default:
      break;
  }
  return s;
}

}  // namespace avionics
