#include "avionics/SoftkeyController.h"

#include <algorithm>

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

}  // namespace

void SoftkeyController::updateAlertSoftkey() {
  // The dynamic Alerts softkey only exists on the root menu (submenus carry a
  // Back key in that cell). While alerts are unacknowledged, the key relabels
  // to the highest active severity and flashes; when none are present it reads
  // "Alerts" and is steady.
  if (currentMenu() != SoftkeyMenu::Root) {
    alertSoftkeyActive_ = false;
    return;
  }

  bool warning = false;
  bool caution = false;
  for (const AlertMessage& m : annunciations_) {
    if (m.level == AlertLevel::Warning) warning = true;
    else if (m.level == AlertLevel::Caution) caution = true;
  }
  // System message advisories live in the Alerts window (not the annunciation
  // window) and drive a flashing "Messages" label.
  const bool messages = !alerts_.empty();

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
  // Once the window is open the pilot is viewing the alerts, so stop flashing.
  if (window_ == PfdWindow::Alerts) alertSoftkeyActive_ = false;
  labels_[kAlertsKey] = label;
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

  // Real crew alerting stacks the highest-severity messages at the top
  // (warnings, then cautions, then advisories). stable_sort keeps the relative
  // order within each level the one defined above.
  const auto byLevel = [](const AlertMessage& a, const AlertMessage& b) {
    return static_cast<int>(a.level) < static_cast<int>(b.level);
  };
  std::stable_sort(alerts_.begin(), alerts_.end(), byLevel);
  std::stable_sort(annunciations_.begin(), annunciations_.end(), byLevel);
}

}  // namespace avionics
