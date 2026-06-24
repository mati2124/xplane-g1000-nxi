#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>

#include "avionics/render/BezelKeys.h"

// NAV/COM radio tuning (Pilot's Guide, Audio Panel / Fig. 4-3, 4-8): the
// dedicated COM and NAV knobs (and the standalone FMS-knob bezel) select a
// unit, step its standby frequency, flip-flop active/standby with the cyan
// transfer animation, and adjust audio volume / NAV ident. The radios are owned
// by the sim feed, so every change is queued as a commit the shell consumes.
namespace avionics {
namespace {

float activeMhzFor(RadioUnit unit, const FlightData& d) {
  switch (unit) {
    case RadioUnit::Nav1:
      return d.nav1ActiveMhz;
    case RadioUnit::Nav2:
      return d.nav2ActiveMhz;
    case RadioUnit::Com1:
      return d.com1ActiveMhz;
    case RadioUnit::Com2:
      return d.com2ActiveMhz;
  }
  return d.nav1ActiveMhz;
}

int radioDecimals(RadioUnit unit) {
  return (unit == RadioUnit::Com1 || unit == RadioUnit::Com2) ? 3 : 2;
}

// Step a radio's audio level one knob click and show it as a 0..100 percent.
float stepRadioVolume(float current, int direction) {
  return std::clamp(current + static_cast<float>(direction) * kRadioVolumeStep,
                    kRadioVolumeMin, kRadioVolumeMax);
}

}  // namespace

bool SoftkeyController::consumeRadioTune(RadioUnit& unit, float& standbyMhz) {
  if (!radioTunePending_) return false;
  radioTunePending_ = false;
  unit = radioTuneUnit_;
  standbyMhz = radioTuneMhz_;
  return true;
}

bool SoftkeyController::consumeRadioTransfer(RadioUnit& unit) {
  if (!radioTransferPending_) return false;
  radioTransferPending_ = false;
  unit = radioTransferUnit_;
  return true;
}

float SoftkeyController::standbyMhzFor(RadioUnit unit,
                                       const FlightData& d) const {
  switch (unit) {
    case RadioUnit::Nav1:
      return d.nav1StandbyMhz;
    case RadioUnit::Nav2:
      return d.nav2StandbyMhz;
    case RadioUnit::Com1:
      return d.com1StandbyMhz;
    case RadioUnit::Com2:
      return d.com2StandbyMhz;
  }
  return d.nav1StandbyMhz;
}

float SoftkeyController::radioTransferAnim(RadioUnit unit) const {
  if (!radioXferAnimActive_ || radioXferAnim_.unit != unit) return 0.0f;
  return radioXferAnim_.progress;
}

float SoftkeyController::radioTransferFromActive(RadioUnit unit) const {
  if (radioXferAnim_.unit != unit) return 0.0f;
  return radioXferAnim_.fromActiveMhz;
}

float SoftkeyController::radioTransferFromStandby(RadioUnit unit) const {
  if (radioXferAnim_.unit != unit) return 0.0f;
  return radioXferAnim_.fromStandbyMhz;
}

int SoftkeyController::radioTransferDecimals(RadioUnit unit) const {
  if (radioXferAnim_.unit != unit) return 2;
  return radioXferAnim_.decimals;
}

void SoftkeyController::setStandbyMhzFor(RadioUnit unit, float mhz) {
  radioTuneUnit_ = unit;
  radioTuneMhz_ = mhz;
  radioTunePending_ = true;
}

void SoftkeyController::queueRadioTune(RadioUnit unit, float standbyMhz) {
  setStandbyMhzFor(unit, standbyMhz);
  armRadioBand(radioBandOf(unit));
}

void SoftkeyController::queueRadioTransfer(RadioUnit unit,
                                            const FlightData& d) {
  radioTransferUnit_ = unit;
  radioTransferPending_ = true;
  armRadioBand(radioBandOf(unit));
  radioXferAnim_.unit = unit;
  radioXferAnim_.progress = 0.0f;
  radioXferAnim_.fromActiveMhz = activeMhzFor(unit, d);
  radioXferAnim_.fromStandbyMhz = standbyMhzFor(unit, d);
  radioXferAnim_.decimals = radioDecimals(unit);
  radioXferAnimActive_ = true;
}

void SoftkeyController::armRadioBand(RadioBand band) {
  radioArmedBand_ = band;
  radioArmedSeconds_ = kRadioArmedSeconds;
}

void SoftkeyController::cycleRadioSelect() {
  switch (radioSelected_) {
    case RadioUnit::Nav1:
      radioSelected_ = RadioUnit::Nav2;
      break;
    case RadioUnit::Nav2:
      radioSelected_ = RadioUnit::Com1;
      break;
    case RadioUnit::Com1:
      radioSelected_ = RadioUnit::Com2;
      break;
    case RadioUnit::Com2:
      radioSelected_ = RadioUnit::Nav1;
      break;
  }
  // Mirror the unified focus into the per-side cursor so the bar's COM and NAV
  // boxes track the FMS knob, and flash the side it landed on.
  if (radioBandOf(radioSelected_) == RadioBand::Com) {
    comSelected_ = radioSelected_;
  } else {
    navSelected_ = radioSelected_;
  }
  armRadioBand(radioBandOf(radioSelected_));
}

void SoftkeyController::selectCom() {
  comSelected_ =
      comSelected_ == RadioUnit::Com1 ? RadioUnit::Com2 : RadioUnit::Com1;
  radioSelected_ = comSelected_;
  armRadioBand(RadioBand::Com);
}

void SoftkeyController::selectNav() {
  navSelected_ =
      navSelected_ == RadioUnit::Nav1 ? RadioUnit::Nav2 : RadioUnit::Nav1;
  radioSelected_ = navSelected_;
  armRadioBand(RadioBand::Nav);
}

void SoftkeyController::tuneCom(int direction, bool coarse, const FlightData& d) {
  const float cur = standbyMhzFor(comSelected_, d);
  const float next = coarse ? stepComStandbyMhzCoarse(cur, direction)
                            : stepComStandbyMhz(cur, direction);
  queueRadioTune(comSelected_, next);
}

void SoftkeyController::tuneNav(int direction, bool coarse, const FlightData& d) {
  const float cur = standbyMhzFor(navSelected_, d);
  const float next = coarse ? stepNavStandbyMhzCoarse(cur, direction)
                            : stepNavStandbyMhz(cur, direction);
  queueRadioTune(navSelected_, next);
}

void SoftkeyController::transferCom(const FlightData& d) {
  queueRadioTransfer(comSelected_, d);
}

void SoftkeyController::transferNav(const FlightData& d) {
  queueRadioTransfer(navSelected_, d);
}

void SoftkeyController::adjustRadioVolume(RadioUnit unit, RadioBand band,
                                          int direction, const FlightData& d) {
  const float next = stepRadioVolume(d.*radioVolumeMember(unit), direction);
  radioVolumeCommitUnit_ = unit;
  radioVolumeCommitValue_ = next;
  radioVolumePending_ = true;
  // Show the percentage in place of the selected radio's standby frequency.
  radioVolumeBand_ = band;
  radioVolumeUnit_ = unit;
  radioVolumePct_ = static_cast<int>(std::lround(next * 100.0f));
  radioVolumeShownSeconds_ = kRadioVolumeShownSeconds;
  ++radioVolumeEpoch_;
}

void SoftkeyController::adjustComVolume(int direction, const FlightData& d) {
  adjustRadioVolume(comSelected_, RadioBand::Com, direction, d);
}

void SoftkeyController::adjustNavVolume(int direction, const FlightData& d) {
  adjustRadioVolume(navSelected_, RadioBand::Nav, direction, d);
}

void SoftkeyController::toggleNavIdent(const FlightData& d) {
  // The NAV VOL/ID press toggles Morse ident audio for the selected NAV radio;
  // "ID" annunciates while it is on (Pilot's Guide Fig. 4-8). The state lives in
  // the sim (audio_selection_nav*), so flip the current value and queue the
  // write; the annunciation reads back from FlightData.
  navIdentCommitUnit_ = navSelected_;
  navIdentCommitValue_ = !(d.*navIdentAudioMember(navSelected_));
  navIdentPending_ = true;
}

bool SoftkeyController::consumeRadioVolume(RadioUnit& unit, float& volume) {
  if (!radioVolumePending_) return false;
  radioVolumePending_ = false;
  unit = radioVolumeCommitUnit_;
  volume = radioVolumeCommitValue_;
  return true;
}

bool SoftkeyController::consumeNavIdent(RadioUnit& unit, bool& on) {
  if (!navIdentPending_) return false;
  navIdentPending_ = false;
  unit = navIdentCommitUnit_;
  on = navIdentCommitValue_;
  return true;
}

bool SoftkeyController::radioVolumeShown(RadioBand band) const {
  return radioVolumeShownSeconds_ > 0.0 && radioVolumeBand_ == band;
}

void SoftkeyController::mirrorRadioVolumeAnnunciation(
    const SoftkeyController& src) {
  radioVolumeEpoch_ = src.radioVolumeEpoch_;
  radioVolumeShownSeconds_ = src.radioVolumeShownSeconds_;
  radioVolumeBand_ = src.radioVolumeBand_;
  radioVolumeUnit_ = src.radioVolumeUnit_;
  radioVolumePct_ = src.radioVolumePct_;
}

void syncRadioVolumeAnnunciation(SoftkeyController& a, SoftkeyController& b) {
  if (a.radioVolumeEpoch_ == 0 && b.radioVolumeEpoch_ == 0) return;

  if (a.radioVolumeEpoch_ != b.radioVolumeEpoch_) {
    // A fresh knob turn on one GDU should propagate to the peer, not be cleared
    // by a side that has not seen the adjustment yet.
    const SoftkeyController& src =
        a.radioVolumeEpoch_ > b.radioVolumeEpoch_ ? a : b;
    a.mirrorRadioVolumeAnnunciation(src);
    b.mirrorRadioVolumeAnnunciation(src);
    return;
  }

  // Same adjustment: align on the earliest dismissal so a slower-updating peer
  // cannot refresh the timer and leave VOL stuck on screen.
  const SoftkeyController& src =
      a.radioVolumeSecondsLeft() <= b.radioVolumeSecondsLeft() ? a : b;
  a.mirrorRadioVolumeAnnunciation(src);
  b.mirrorRadioVolumeAnnunciation(src);
}

bool SoftkeyController::radioBezelKey(BezelKey key, const FlightData& d) {
  if (!canUseRadioBezel()) return false;

  switch (key) {
    case BezelKey::FmsPush:
      cycleRadioSelect();
      return true;
    case BezelKey::Ent:
      queueRadioTransfer(radioSelected_, d);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const int dir = key == BezelKey::FmsInnerCw ? +1 : -1;
      const float cur = standbyMhzFor(radioSelected_, d);
      const bool isCom = radioSelected_ == RadioUnit::Com1 ||
                         radioSelected_ == RadioUnit::Com2;
      const float next =
          isCom ? stepComStandbyMhz(cur, dir) : stepNavStandbyMhz(cur, dir);
      queueRadioTune(radioSelected_, next);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace avionics
