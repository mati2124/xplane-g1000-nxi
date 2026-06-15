#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include "avionics/UpdateChecker.h"

// Crew Alerting System: the always-on annunciation window (CAS messages driven
// by the X-Plane annunciator conditions) and the system-message Alerts window,
// plus the dynamic Alerts/Warning/Caution/Messages softkey label and flash.
namespace avionics {
namespace {

// Crew Alerting System messages driven by the X-Plane annunciator conditions in
// FlightData. Each entry maps a condition flag to the text and severity shown
// in the Alerts window. Adding a new CAS message is just another row here (plus
// the dataref that feeds its flag).
struct CasMessageDef {
  bool FlightData::* condition;
  const char* text;
  AlertLevel level;
};

constexpr CasMessageDef kCasMessages[] = {
    {&FlightData::casOilPressureLow, "OIL PRESSURE", AlertLevel::Warning},
    {&FlightData::casLowVoltage, "LOW VOLTS", AlertLevel::Warning},
    {&FlightData::casStallWarning, "STALL", AlertLevel::Warning},
    {&FlightData::casGearUnsafe, "CHECK GEAR", AlertLevel::Warning},
    {&FlightData::casLowVacuum, "LOW VACUUM", AlertLevel::Caution},
    {&FlightData::casFuelLow, "FUEL LOW", AlertLevel::Caution},
    {&FlightData::casOilTempHigh, "OIL TEMP", AlertLevel::Caution},
    {&FlightData::casFuelPressureLow, "FUEL PRESS", AlertLevel::Caution},
    {&FlightData::casPitotHeatOff, "PITOT HEAT", AlertLevel::Caution},
    {&FlightData::casIcing, "ANTI ICE", AlertLevel::Caution},
};

bool isAcknowledged(const std::set<std::string>& acked, const std::string& t) {
  return acked.find(t) != acked.end();
}

// Drops acknowledgments whose underlying alert has cleared so that a later
// re-occurrence of the same message flashes again instead of staying silent.
void pruneAcknowledged(const std::vector<AlertMessage>& present,
                       std::set<std::string>& acked) {
  for (auto it = acked.begin(); it != acked.end();) {
    const bool stillPresent =
        std::any_of(present.begin(), present.end(),
                    [&](const AlertMessage& m) { return m.text == *it; });
    if (stillPresent) {
      ++it;
    } else {
      it = acked.erase(it);
    }
  }
}

}  // namespace

void SoftkeyController::updateAlertSoftkey() {
  // The dynamic Alerts softkey only exists on the root menu (submenus carry a
  // Back key in that cell). While unacknowledged alerts are present, the key
  // relabels to the highest active severity and flashes; when none are present
  // (or all are acknowledged) it reads "Alerts" and is steady.
  if (currentMenu() != SoftkeyMenu::Root) {
    alertSoftkeyActive_ = false;
    return;
  }

  // CAS annunciations live in the always-on annunciation window; only their
  // unacknowledged warnings/cautions drive the flashing Warning/Caution label.
  bool warning = false;
  bool caution = false;
  for (const AlertMessage& m : annunciations_) {
    if (isAcknowledged(acknowledgedAnnunciations_, m.text)) continue;
    if (m.level == AlertLevel::Warning) warning = true;
    else if (m.level == AlertLevel::Caution) caution = true;
  }
  // System message advisories live in the Alerts window (not the annunciation
  // window) and drive a flashing "Messages" label until acknowledged.
  bool messages = false;
  for (const AlertMessage& m : alerts_) {
    if (!isAcknowledged(acknowledgedMessages_, m.text)) {
      messages = true;
      break;
    }
  }

  const char* label = "Alerts";
  alertSoftkeyActive_ = false;
  if (warning) {
    label = "Warning";
    alertSoftkeyLevel_ = AlertLevel::Warning;
    alertSoftkeyActive_ = true;
  } else if (caution) {
    label = "Caution";
    alertSoftkeyLevel_ = AlertLevel::Caution;
    alertSoftkeyActive_ = true;
  } else if (messages) {
    label = "Messages";
    alertSoftkeyLevel_ = AlertLevel::Advisory;
    alertSoftkeyActive_ = true;
  }
  labels_[kAlertsKey] = label;
}

void SoftkeyController::pressAlertsSoftkey() {
  // Pilot's Guide, Appendix A: "By pressing the softkey when flashing an
  // annunciation, the alert is acknowledged. The softkey label then returns to
  // Alerts." So while flashing, the first press acknowledges the displayed
  // level rather than opening the window. Warning/Caution acknowledge their CAS
  // annunciations only; the Messages press also opens the Alerts window.
  if (alertSoftkeyActive_) {
    switch (alertSoftkeyLevel_) {
      case AlertLevel::Warning:
      case AlertLevel::Caution:
        for (const AlertMessage& m : annunciations_) {
          if (m.level == alertSoftkeyLevel_) {
            acknowledgedAnnunciations_.insert(m.text);
          }
        }
        break;
      case AlertLevel::Advisory:
        for (const AlertMessage& m : alerts_) {
          acknowledgedMessages_.insert(m.text);
        }
        toggleWindow(PfdWindow::Alerts);
        break;
    }
    updateAlertSoftkey();
    return;
  }

  // Steady "Alerts": a press toggles the Alerts window (Pilot's Guide: pressing
  // the Alerts Softkey a second time views / removes the alert text messages).
  toggleWindow(PfdWindow::Alerts);
}

void SoftkeyController::rebuildAlerts(const FlightData& data) {
  alerts_.clear();
  annunciations_.clear();
  // Reversionary sensor failures surface here as warnings, alongside the
  // per-instrument red X. AHRS feeds attitude + heading; the ADC feeds the
  // air-data instruments.
  if (!data.attitudeValid) {
    alerts_.push_back({"AHRS1 SERVICE - Attitude data unavailable",
                       AlertLevel::Warning});
  }
  if (!data.headingValid) {
    alerts_.push_back(
        {"HDG - Heading reference unavailable", AlertLevel::Warning});
  }
  if (!data.airspeedValid || !data.altitudeValid || !data.verticalSpeedValid) {
    alerts_.push_back(
        {"ADC1 SERVICE - Air data unavailable", AlertLevel::Warning});
  }
  if (!data.navSignalValid) {
    alerts_.push_back(
        {"NAV - No usable navigation signal", AlertLevel::Caution});
  }

  // CAS messages from the live X-Plane annunciators populate the always-on PFD
  // annunciation window only. The Alerts window (above) is for system message
  // advisories, which on the real G1000 are kept separate from CAS messages.
  for (const CasMessageDef& m : kCasMessages) {
    if (data.*(m.condition)) annunciations_.push_back({m.text, m.level});
  }

  // Built-in demo feed: keep a persistent banner in the always-on CAS window so
  // it is obvious the displays are not driven by the sim, and how to leave it.
  if (demoBanner_) {
    annunciations_.push_back({"DEMO MODE", AlertLevel::Caution});
    annunciations_.push_back({"ESC TO EXIT", AlertLevel::Advisory});
  }

  // A pending software-update notice (set by the shell's launch-time update
  // check) shows here as a system-message advisory, which also flashes the
  // "Messages" softkey until the pilot opens the Alerts window.
  if (std::string update = updateAdvisory(); !update.empty()) {
    alerts_.push_back({std::move(update), AlertLevel::Advisory});
  }

  // Real crew alerting stacks the highest-severity messages at the top
  // (warnings, then cautions, then advisories). stable_sort keeps the relative
  // order within each level the one defined above.
  const auto byLevel = [](const AlertMessage& a, const AlertMessage& b) {
    return static_cast<int>(a.level) < static_cast<int>(b.level);
  };
  std::stable_sort(alerts_.begin(), alerts_.end(), byLevel);
  std::stable_sort(annunciations_.begin(), annunciations_.end(), byLevel);

  // Forget acknowledgments for alerts that have since cleared, so the softkey
  // flashes again if the same condition returns.
  pruneAcknowledged(annunciations_, acknowledgedAnnunciations_);
  pruneAcknowledged(alerts_, acknowledgedMessages_);
}

}  // namespace avionics
