#include "avionics/AvionicsEngine.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "avionics/Color.h"
#include "avionics/ComDecode.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FmsNavigator.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/GlidepathGuidance.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/MapData.h"
#include "avionics/MissedApproachGuidance.h"
#include "avionics/NavMath.h"
#include "avionics/render/BootScreen.h"
#include "avionics/render/MultiFunctionDisplay.h"
#include "avionics/render/PrimaryFlightDisplay.h"

namespace avionics {

namespace {

// CSS ease-in-out, matching StartupLogo.css opacity transition on the Power-up
// Page cross-fade.
float bootEaseInOut(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return t < 0.5f ? 2.0f * t * t
                  : 1.0f - std::pow(-2.0f * t + 2.0f, 2.0f) / 2.0f;
}

// Feet per nautical mile (for converting along-track distance to altitude).
constexpr double kFeetPerNm = 6076.12;
// Default VNAV flight-path angle the NXi uses when none is published (3.0 deg,
// G1000 NXi Pilot's Guide, Section 6 "Vertical Navigation").
constexpr float kDefaultFpaDeg = 3.0f;
// PFD VNAV vertical-deviation full-scale: +/-2 dots at +/-500 ft.
constexpr float kVnavDevFtPerDot = 250.0f;

int activeLegIndex(const std::vector<MapLeg>& plan, const std::string& toWpt) {
  const int idx = legIndexInPlan(plan, toWpt);
  return idx >= 0 ? idx : 0;
}

double alongTrackDistanceNm(const MapData& map, const FlightData& data,
                            const std::vector<MapLeg>& plan, int targetIdx) {
  if (targetIdx < 0 || targetIdx >= static_cast<int>(plan.size())) return 0.0;
  int activeIdx = data.fmaActiveLegIndex;
  if (activeIdx < 0) activeIdx = legIndexInPlan(plan, data.fmaToWpt);
  if (activeIdx < 0) activeIdx = 0;
  double distNm = navDistanceNm(map.ownshipLat, map.ownshipLon, plan[activeIdx].lat,
                                plan[activeIdx].lon);
  for (int i = activeIdx; i < targetIdx; ++i) {
    distNm += navDistanceNm(plan[i].lat, plan[i].lon, plan[i + 1].lat,
                            plan[i + 1].lon);
  }
  return distNm;
}

void applyGlidepath(FlightData& data, const MapData& map) {
  if (suppressGlidepath(map, data)) {
    if (data.vdiKind == VerticalDeviationKind::Glidepath ||
        data.vdiKind == VerticalDeviationKind::Glideslope) {
      data.vdiKind = VerticalDeviationKind::None;
      data.vdiValid = false;
    }
    return;
  }
  const GlidepathSolution gp = computeGlidepath(map, data);
  if (gp.valid) {
    applyGlidepathSolution(data, gp);
  } else if (data.vdiKind == VerticalDeviationKind::Glidepath) {
    data.vdiKind = VerticalDeviationKind::None;
    data.vdiValid = false;
  }
}

// Computes the active VNAV profile: finds the next constrained, lower waypoint
// ahead of the active leg, builds the descent path to it at the default FPA, and
// derives the required vertical speed, top-of-descent, and path deviation.
VnvProfile computeVnvProfile(const MapData& map, const FlightData& data) {
  VnvProfile vnv;
  if (data.missedApproachActive) return vnv;
  if (!map.positionValid || map.flightPlan.size() < 1) return vnv;

  const std::vector<MapLeg>& plan = map.flightPlan;

  const int activeIdx = activeLegIndex(plan, data.fmaToWpt);

  // The VNAV target is the next waypoint at/after the active leg that carries an
  // altitude constraint lower than the current altitude (a descent target).
  int targetIdx = -1;
  for (std::size_t i = static_cast<std::size_t>(activeIdx); i < plan.size();
       ++i) {
    const MapLeg& leg = plan[i];
    if (leg.altitudeConstraint != AltConstraintType::None &&
        leg.altitudeConstraintFt > 0 &&
        static_cast<float>(leg.altitudeConstraintFt) < data.altitudeFt - 50.0f) {
      targetIdx = static_cast<int>(i);
      break;
    }
  }
  if (targetIdx < 0) return vnv;

  const double distNm = alongTrackDistanceNm(map, data, plan, targetIdx);

  constexpr double kPi = 3.14159265358979323846;
  float fpaDeg = kDefaultFpaDeg;
  for (int i = activeIdx; i <= targetIdx; ++i) {
    const float legGpa = plan[static_cast<std::size_t>(i)].glidePathAngleDeg;
    if (legGpa > 0.0f) {
      fpaDeg = legGpa;
      break;
    }
  }
  const double tanFpa = std::tan(static_cast<double>(fpaDeg) * kPi / 180.0);
  const double gsKts = std::max(1.0f, data.groundSpeedKts);

  vnv.active = true;
  vnv.targetWpt = plan[static_cast<std::size_t>(targetIdx)].id;
  vnv.targetAltFt = plan[static_cast<std::size_t>(targetIdx)].altitudeConstraintFt;
  vnv.fpaDeg = fpaDeg;

  // Path descent rate (fpm, negative) at the current ground speed and FPA.
  const double gsFpm = gsKts * kFeetPerNm / 60.0;
  vnv.vsTargetFpm = static_cast<float>(-tanFpa * gsFpm);

  // Top of descent sits `descentDistNm` before the target.
  const double altToLoseFt =
      static_cast<double>(data.altitudeFt) - vnv.targetAltFt;
  const double descentDistNm = (altToLoseFt / tanFpa) / kFeetPerNm;
  vnv.distanceToTodNm = static_cast<float>(distNm - descentDistNm);
  vnv.timeToTodSec =
      vnv.distanceToTodNm > 0.0f
          ? static_cast<int>(std::lround(vnv.distanceToTodNm / gsKts * 3600.0))
          : 0;

  // VS required now to reach the target altitude at the target waypoint.
  const double timeToTargetMin = (distNm / gsKts) * 60.0;
  vnv.vsRequiredFpm =
      timeToTargetMin > 0.01
          ? static_cast<float>((vnv.targetAltFt - data.altitudeFt) /
                               timeToTargetMin)
          : 0.0f;

  if (vnv.distanceToTodNm <= 0.0f) {
    // Past TOD: on the descent. The path altitude at this distance from the
    // target is target + tan(FPA) * remaining distance; deviation is above (+).
    vnv.capturing = true;
    const double pathAltFt =
        vnv.targetAltFt + tanFpa * distNm * kFeetPerNm;
    vnv.verticalDeviationFt =
        static_cast<float>(data.altitudeFt - pathAltFt);
  }
  return vnv;
}

// Computes the VNAV profile and, when capturing the path and no approach
// glideslope/glidepath is being received, drives the PFD vertical-deviation
// pointer ("V") and the required-VS chevron from it.
void applyVnav(FlightData& data, const MapData& map) {
  data.vnv = computeVnvProfile(map, data);
  if (data.vnv.capturing && data.vdiKind == VerticalDeviationKind::None) {
    data.vdiKind = VerticalDeviationKind::Vnav;
    data.vdiValid = true;
    data.vdiDeviationDots = std::max(
        -2.0f, std::min(2.0f, data.vnv.verticalDeviationFt / kVnavDevFtPerDot));
    data.requiredVsValid = true;
    data.requiredVsFpm = data.vnv.vsRequiredFpm;
  }
}

// Marks every sensor-backed instrument invalid so the PFD draws each gauge's
// own red-X failure annunciation. Used when the data link is not delivering
// fresh data: rather than a single full-screen "no data" page, each instrument
// shows its failed state, matching real EFIS reversionary behavior.
FlightData withAllSensorsFailed(FlightData data) {
  data.attitudeValid = false;
  data.headingValid = false;
  data.airspeedValid = false;
  data.altitudeValid = false;
  data.verticalSpeedValid = false;
  // The radios and transponder are also unreachable with the link down, so
  // their boxes red-X / annunciate FAIL alongside the sensor gauges.
  data.nav1Valid = false;
  data.nav2Valid = false;
  data.com1Valid = false;
  data.com2Valid = false;
  data.transponderValid = false;
  data.navSignalValid = false;
  data.windValid = false;
  data.bearing1Valid = false;
  data.bearing2Valid = false;
  // The link is down, so the non-sensor chrome readouts (radios, FMA,
  // transponder, OAT, clock) are unknown too: blank them / dash them out
  // rather than leaving the last-received values on screen.
  data.dataLinkValid = false;
  data.nav1Ident.clear();
  data.nav2Ident.clear();
  return data;
}

}  // namespace

AvionicsEngine::AvionicsEngine(DataSource& dataSource, Renderer& renderer,
                               std::string sourceLabel)
    : dataSource_(&dataSource),
      renderer_(renderer),
      sourceLabel_(std::move(sourceLabel)) {}

void AvionicsEngine::setDataSource(DataSource& dataSource,
                                   std::string sourceLabel) {
  dataSource_ = &dataSource;
  sourceLabel_ = std::move(sourceLabel);
  bootElapsedSeconds_ = 0.0;  // re-run the self-test for the new source
  powerUpAcknowledged_ = false;
}

void AvionicsEngine::update(double dtSeconds) {
  const bool powered = displayPowered();
  if (!powerInitialized_) {
    powerInitialized_ = true;
  } else if (powered && !wasPowered_) {
    // The GDU bus just came alive: replay the power-up self-test so the screen
    // visibly boots, like a real unit does when its display is switched on.
    bootElapsedSeconds_ = 0.0;
    powerUpAcknowledged_ = false;
  }
  wasPowered_ = powered;

  if (!powered) {
    // A secondary engine sharing the source (the MFD window) must not pump it a
    // second time. The pump still runs while the screen is dark so switch
    // datarefs are read and the other GDU sharing the source stays current.
    if (drivesDataSource_) dataSource_->update(dtSeconds);
    return;  // dark screen: no boot timer or UI animation
  }

  bootElapsedSeconds_ += dtSeconds;
  softkeys_.update(dtSeconds, dataSource_->snapshot(),
                   dataSource_->mapSnapshot());
  mfd_.update(dtSeconds, dataSource_->snapshot());

  // Nearby-data queries (land vectors, airspaces, nav features) must center on
  // the panned map view before the source refresh so land and symbols match
  // what MapView draws (pointer geo can lag the view center until edge-scroll).
  mfd_.applyMapPanToDataSource(*dataSource_, dataSource_->snapshot());
  dataSource_->setMapViewHalfExtentNm(mfd_.mapViewHalfExtentNm());
  dataSource_->setChartRangeNm(sharedMapQueryRangeNm());
  if (drivesDataSource_) dataSource_->update(dtSeconds);

  if (drivesDataSource_ &&
      dataSource_->connectionState() == ConnectionState::Connected) {
    applyActivateLegRequests();
    applyActivateMissedRequests();
    const FlightData& snap = dataSource_->snapshot();
    dataSource_->applyGpsNavigation(
        navigator_,
        softkeys_.displayToggle(DisplayToggle::Obs),
        softkeys_.cdiSourceFor(snap.cdiSource), 0.0f);
  }

  // Flight-plan sync uses mapSnapshot(); pump the source first so a route
  // override applied before this frame's update is visible (softkeys_.update
  // above may have seen a stale plan).
  softkeys_.syncFlightPlanFromMap(dataSource_->mapSnapshot(),
                                  navDirectToActive(dataSource_->snapshot()));

  // Keep the MFD's checklist navigation in step with the loaded file (the data
  // is owned by the source; the controller only holds the interactive state).
  mfd_.syncChecklist(dataSource_->checklistSnapshot());
  syncSoftkeyPeerRadioVolume();
  // Peer sync runs before the map-driven FPL adopt so a PFD Active Flight
  // Plan edit is not overwritten by a stale mapSnapshot on the other GDU.
  syncFlightPlanPeer();
  // Keep the FPL page's editable plan in step with the active flight plan
  // (and ownship, for waypoint-entry lookups). The active leg's TO ident
  // seeds the Direct-To window's default waypoint.
  mfd_.syncFlightPlan(dataSource_->mapSnapshot(),
                      dataSource_->snapshot().fmaToWpt,
                      navDirectToActive(dataSource_->snapshot()));
  syncFlightPlanApproachPeer();
}

namespace {

bool flightPlanApproachGroupingEqual(const FlightPlanApproachState& a,
                                     const FlightPlanApproachState& b) {
  return a.legStart == b.legStart && a.legCount == b.legCount &&
         a.loaded.type == b.loaded.type && a.loaded.name == b.loaded.name &&
         a.loaded.transition == b.loaded.transition &&
         a.loaded.runway == b.loaded.runway &&
         a.loaded.approachKind == b.loaded.approachKind &&
         a.loaded.levelOfService == b.loaded.levelOfService;
}

}  // namespace

void AvionicsEngine::applyActivateLegRequests() {
  if (!drivesDataSource_) return;

  auto applyFrom = [this](int legIdx) {
    if (legIdx >= 0) {
      navigator_.setActiveLegIndex(legIdx);
      dataSource_->syncSimulatorActiveLeg(legIdx);
    }
  };

  int legIdx = -1;
  if (softkeys_.consumeActivateLegRequest(legIdx)) {
    applyFrom(legIdx);
    return;
  }
  if (mfd_.consumeActivateLegRequest(legIdx)) {
    applyFrom(legIdx);
    return;
  }
  if (!softkeyPeer_) return;
  if (softkeyPeer_->softkeyController().consumeActivateLegRequest(legIdx)) {
    applyFrom(legIdx);
    return;
  }
  if (softkeyPeer_->mfdController().consumeActivateLegRequest(legIdx)) {
    applyFrom(legIdx);
  }
}

void AvionicsEngine::applyActivateMissedRequests() {
  if (!drivesDataSource_) return;

  auto tryConsume = [this]() -> bool {
    if (softkeys_.consumeActivateMissedRequest()) return true;
    if (!softkeyPeer_) return false;
    return softkeyPeer_->softkeyController().consumeActivateMissedRequest();
  };

  if (!tryConsume()) return;

  if (!navigator_.activateMissedApproach()) return;
  const int legIdx = navigator_.activeLegIndex();
  if (legIdx >= 0) {
    dataSource_->syncSimulatorActiveLeg(legIdx);
  }
}

void AvionicsEngine::syncFlightPlanPeer() {
  if (!softkeyPeer_) return;

  SoftkeyController& pfdSk =
      page_ == DisplayPage::PrimaryFlightDisplay
          ? softkeys_
          : softkeyPeer_->softkeyController();
  MfdController& mfdFpl =
      page_ == DisplayPage::MultiFunctionDisplay
          ? mfd_
          : softkeyPeer_->mfdController();

  const std::vector<MapLeg>& skLegs = pfdSk.flightPlanLegs();
  const std::vector<MapLeg>& mfdLegs = mfdFpl.fplLegs();
  const bool skDest = pfdSk.flightPlanDestinationFilled();
  const bool mfdDest = mfdFpl.fplDestinationFilled();
  if (::avionics::flightPlanLegsEqual(skLegs, mfdLegs) && skDest == mfdDest) {
    return;
  }

  const bool skDraft = pfdSk.flightPlanLocalDraft();
  const bool mfdDraft = mfdFpl.fplLocalDraft();
  const FlightPlanApproachState skApproach = pfdSk.flightPlanApproachState();
  const FlightPlanApproachState mfdApproach = mfdFpl.flightPlanApproachState();

  if (skDraft && !mfdDraft) {
    mfdFpl.adoptFlightPlanFromPeer(skLegs, skDest, skApproach);
  } else if (mfdDraft && !skDraft) {
    pfdSk.adoptFlightPlanFromPeer(mfdLegs, mfdDest, mfdApproach);
  } else {
    // Neither side is exclusively drafting — keep the MFD page aligned with
    // the PFD Active Flight Plan window (the primary route editor on the PFD).
    mfdFpl.adoptFlightPlanFromPeer(skLegs, skDest, skApproach);
  }
}

void AvionicsEngine::syncFlightPlanApproachPeer() {
  if (!softkeyPeer_) return;

  const SoftkeyController& pfdSk =
      page_ == DisplayPage::PrimaryFlightDisplay
          ? softkeys_
          : softkeyPeer_->softkeyController();
  const MfdController& mfdFpl =
      page_ == DisplayPage::MultiFunctionDisplay
          ? mfd_
          : softkeyPeer_->mfdController();
  if (!::avionics::flightPlanLegsEqual(pfdSk.flightPlanLegs(),
                                       mfdFpl.fplLegs())) {
    return;
  }

  const FlightPlanApproachState localSk = softkeys_.flightPlanApproachState();
  const FlightPlanApproachState localMfd = mfd_.flightPlanApproachState();
  const FlightPlanApproachState peerSk =
      softkeyPeer_->softkeyController().flightPlanApproachState();
  const FlightPlanApproachState peerMfd =
      softkeyPeer_->mfdController().flightPlanApproachState();

  FlightPlanApproachState authoritative = localSk;
  if (!authoritative.active()) authoritative = peerSk;
  if (!authoritative.active()) authoritative = localMfd;
  if (!authoritative.active()) authoritative = peerMfd;
  if (!authoritative.active()) return;

  const auto stateForLegs = [](const FlightPlanApproachState& source,
                               const std::vector<MapLeg>& legs) {
    return approachStateFitsPlan(source, legs) ? source
                                               : FlightPlanApproachState{};
  };

  const std::vector<MapLeg>& syncedLegs = pfdSk.flightPlanLegs();
  const FlightPlanApproachState mfdState = stateForLegs(authoritative, syncedLegs);
  if (!flightPlanApproachGroupingEqual(mfd_.flightPlanApproachState(), mfdState) ||
      mfd_.fplApproachHeaderLabel() != mfdState.headerLabel) {
    mfd_.applyFlightPlanApproachState(mfdState);
  }

  const FlightPlanApproachState skState =
      stateForLegs(authoritative, softkeys_.flightPlanLegs());
  if (!flightPlanApproachGroupingEqual(softkeys_.flightPlanApproachState(),
                                       skState)) {
    softkeys_.applyFlightPlanApproachState(skState);
  }

  const FlightPlanApproachState peerMfdState =
      stateForLegs(authoritative, syncedLegs);
  if (!flightPlanApproachGroupingEqual(
          softkeyPeer_->mfdController().flightPlanApproachState(),
          peerMfdState) ||
      softkeyPeer_->mfdController().fplApproachHeaderLabel() !=
          peerMfdState.headerLabel) {
    softkeyPeer_->mfdController().applyFlightPlanApproachState(peerMfdState);
  }

  const FlightPlanApproachState peerSkState = stateForLegs(
      authoritative, softkeyPeer_->softkeyController().flightPlanLegs());
  if (!flightPlanApproachGroupingEqual(
          softkeyPeer_->softkeyController().flightPlanApproachState(),
          peerSkState)) {
    softkeyPeer_->softkeyController().applyFlightPlanApproachState(peerSkState);
  }
}

void AvionicsEngine::syncSoftkeyPeerRadioVolume() {
  if (!softkeyPeer_) return;
  syncRadioVolumeAnnunciation(softkeys_, softkeyPeer_->softkeys_);
}

double AvionicsEngine::bootGateSeconds() const {
  return page_ == DisplayPage::MultiFunctionDisplay ? kBootPowerUpFadeSeconds
                                                    : kBootDurationSeconds;
}

bool AvionicsEngine::bootComplete() const {
  // The animated power-up must have run. The PFD goes live on its own once the
  // init cross-fade finishes; only the MFD waits for ENT on the database page.
  if (bootElapsedSeconds_ < bootGateSeconds()) return false;
  if (page_ == DisplayPage::PrimaryFlightDisplay) return true;
  return powerUpAcknowledged_ || !dataSource_->requiresPowerUpAcknowledge();
}

bool AvionicsEngine::awaitingPowerUpAck() const {
  if (page_ == DisplayPage::PrimaryFlightDisplay) return false;
  return bootElapsedSeconds_ >= bootGateSeconds() && !powerUpAcknowledged_ &&
         dataSource_->requiresPowerUpAcknowledge();
}

void AvionicsEngine::acknowledgePowerUp() {
  if (powerUpAcknowledged_) return;
  if (!dataSource_->requiresPowerUpAcknowledge()) {
    powerUpAcknowledged_ = true;
    return;
  }
  if (bootElapsedSeconds_ >= bootGateSeconds()) powerUpAcknowledged_ = true;
}

bool AvionicsEngine::displayPowered() const {
  const FlightData& d = dataSource_->snapshot();
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      return d.masterPowerOn;
    case DisplayPage::MultiFunctionDisplay:
      // The avionics bus is downstream of the battery, so the MFD needs both.
      return d.masterPowerOn && d.avionicsPowerOn;
  }
  return true;
}

bool AvionicsEngine::pfdReversionary() const {
  const FlightData& d = dataSource_->snapshot();
  return page_ == DisplayPage::PrimaryFlightDisplay && d.masterPowerOn &&
         (d.displayBackupActive || !d.avionicsPowerOn);
}

bool AvionicsEngine::mfdReversionary() const {
  const FlightData& d = dataSource_->snapshot();
  return page_ == DisplayPage::MultiFunctionDisplay && d.masterPowerOn &&
         d.avionicsPowerOn && d.displayBackupActive;
}

bool AvionicsEngine::isLivePageUp() const {
  return displayPowered() && bootComplete() &&
         dataSource_->connectionState() == ConnectionState::Connected;
}

float AvionicsEngine::sharedMapQueryRangeNm() const {
  float rangeNm = mfd_.rangeNm();
  if (page_ == DisplayPage::PrimaryFlightDisplay) {
    rangeNm = std::max(rangeNm, softkeys_.insetRangeNm());
  }
  if (softkeyPeer_ != nullptr) {
    rangeNm = std::max(rangeNm, softkeyPeer_->mfdController().rangeNm());
    if (softkeyPeer_->page() == DisplayPage::PrimaryFlightDisplay) {
      rangeNm =
          std::max(rangeNm, softkeyPeer_->softkeyController().insetRangeNm());
    }
  }
  return rangeNm;
}

void AvionicsEngine::pressSoftkey(int index) {
  if (awaitingPowerUpAck()) {
    if (index == kSoftkeyCount - 1) acknowledgePowerUp();
    return;
  }
  if (!isLivePageUp()) return;
  if (page_ == DisplayPage::PrimaryFlightDisplay && index == 4 &&
      !softkeys_.displayToggle(DisplayToggle::Obs) &&
      (navigator_.missedApproachSuspended() || navigator_.inHold())) {
    navigator_.resumeFromAutoSuspend();
    return;
  }
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      softkeys_.pressKey(index);
      break;
    case DisplayPage::MultiFunctionDisplay:
      mfd_.pressKey(index);
      break;
  }
}

const float* AvionicsEngine::softkeyPressLevels() const {
  return page_ == DisplayPage::MultiFunctionDisplay ? mfd_.pressLevels()
                                                    : softkeys_.pressLevels();
}

bool AvionicsEngine::toggleDisplayBackup() {
  if (awaitingPowerUpAck()) return false;
  if (!displayPowered() || !bootComplete()) return false;
  dataSource_->toggleDisplayBackup();
  softkeys_.flashBezelKey(BezelKey::DisplayBackup);
  mfd_.flashBezelKey(BezelKey::DisplayBackup);
  return true;
}

void AvionicsEngine::pressBezelKey(BezelKey key) {
  // While the power-up page waits for acknowledgement, ENT brings up the live
  // pages; other keys are inert except map range/pan on the MFD.
  if (awaitingPowerUpAck()) {
    if (key == BezelKey::Ent) {
      acknowledgePowerUp();
      return;
    }
    if (page_ == DisplayPage::MultiFunctionDisplay &&
        isMapRangePanBezelKey(key)) {
      mfd_.pressBezelKey(key);
      dataSource_->setChartRangeNm(sharedMapQueryRangeNm());
      return;
    }
    return;
  }
  // The audio panel DISPLAY BACKUP key is system-wide (both GDUs reversionary).
  if (key == BezelKey::DisplayBackup) {
    toggleDisplayBackup();
    return;
  }
  // Map range / pan bypass bootComplete and the live-link gate; only require
  // display power (same as NAV/COM knobs once the unit is energized).
  if (isMapRangePanBezelKey(key)) {
    if (!displayPowered()) return;
    switch (page_) {
      case DisplayPage::PrimaryFlightDisplay:
        softkeys_.pressBezelKey(key);
        break;
      case DisplayPage::MultiFunctionDisplay:
        mfd_.pressBezelKey(key);
        dataSource_->setChartRangeNm(sharedMapQueryRangeNm());
        break;
    }
    return;
  }
  if (!displayPowered() || !bootComplete()) return;
  // NAV/COM/CRS/BARO/HDG/VOL knobs work as soon as the display is booted; FMS
  // keys and softkeys still need a live data link.
  if (handleBezelKnob(key)) {
    syncSoftkeyPeerRadioVolume();
    return;
  }
  if (!isLivePageUp()) return;
  // NAV/COM tuning uses the FMS knob on the PFD bezel only (the MFD uses the
  // same knob for page navigation, map pointer, and FPL editing).
  if (page_ == DisplayPage::PrimaryFlightDisplay &&
      softkeys_.canUseRadioBezel() &&
      softkeys_.radioBezelKey(key, dataSource_->snapshot())) {
    return;
  }
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      softkeys_.pressBezelKey(key);
      break;
    case DisplayPage::MultiFunctionDisplay:
      mfd_.pressBezelKey(key);
      break;
  }
}

bool AvionicsEngine::applyGcuEntryKey(char ch) {
  if (!isLivePageUp()) return false;
  if (page_ == DisplayPage::PrimaryFlightDisplay) {
    return softkeys_.applyGcuEntryKey(ch);
  }
  return mfd_.applyGcuEntryKey(ch);
}

bool AvionicsEngine::handleBezelKnob(BezelKey key) {
  const FlightData& d = dataSource_->snapshot();
  switch (key) {
    case BezelKey::ComOuterCw:
      softkeys_.tuneCom(+1, /*coarse=*/true, d);
      break;
    case BezelKey::ComOuterCcw:
      softkeys_.tuneCom(-1, /*coarse=*/true, d);
      break;
    case BezelKey::ComInnerCw:
      softkeys_.tuneCom(+1, /*coarse=*/false, d);
      break;
    case BezelKey::ComInnerCcw:
      softkeys_.tuneCom(-1, /*coarse=*/false, d);
      break;
    case BezelKey::ComPush:
      softkeys_.selectCom();
      break;
    case BezelKey::ComTransfer:
      softkeys_.transferCom(d);
      break;
    case BezelKey::NavOuterCw:
      softkeys_.tuneNav(+1, /*coarse=*/true, d);
      break;
    case BezelKey::NavOuterCcw:
      softkeys_.tuneNav(-1, /*coarse=*/true, d);
      break;
    case BezelKey::NavInnerCw:
      softkeys_.tuneNav(+1, /*coarse=*/false, d);
      break;
    case BezelKey::NavInnerCcw:
      softkeys_.tuneNav(-1, /*coarse=*/false, d);
      break;
    case BezelKey::NavPush:
      softkeys_.selectNav();
      break;
    case BezelKey::NavTransfer:
      softkeys_.transferNav(d);
      break;
    case BezelKey::BaroCw:
      softkeys_.adjustBaro(+1, d);
      break;
    case BezelKey::BaroCcw:
      softkeys_.adjustBaro(-1, d);
      break;
    case BezelKey::CrsCw:
      softkeys_.adjustCourse(+1, d);
      break;
    case BezelKey::CrsCcw:
      softkeys_.adjustCourse(-1, d);
      break;
    case BezelKey::CrsPush:
      softkeys_.setBaroStandard();
      break;
    case BezelKey::HdgCw:
      softkeys_.adjustHeadingBug(+1, d);
      break;
    case BezelKey::HdgCcw:
      softkeys_.adjustHeadingBug(-1, d);
      break;
    case BezelKey::HdgPush:
      softkeys_.syncHeadingBug(d);
      break;
  // Audio VOL/SQ/ID knobs: turn sets the selected radio's volume (shown as a
  // percentage in the NavCom box). The NAV VOL/ID push toggles Morse ident
  // audio; the COM VOL/SQ push is inert (X-Plane has no squelch dataref).
    case BezelKey::ComVolCw:
      softkeys_.adjustComVolume(+1, d);
      break;
    case BezelKey::ComVolCcw:
      softkeys_.adjustComVolume(-1, d);
      break;
    case BezelKey::ComVolPush:
      break;
    case BezelKey::NavVolCw:
      softkeys_.adjustNavVolume(+1, d);
      break;
    case BezelKey::NavVolCcw:
      softkeys_.adjustNavVolume(-1, d);
      break;
    case BezelKey::NavVolPush:
      softkeys_.toggleNavIdent(d);
      break;
    default:
      return false;
  }
  softkeys_.flashBezelKey(key);
  return true;
}

void AvionicsEngine::holdBezelKey(BezelKey key) {
  if (!isLivePageUp()) return;
  // CLR (DFLT MAP) is an MFD-only function (Pilot's Guide).
  if (key == BezelKey::Clr && page_ == DisplayPage::MultiFunctionDisplay) {
    mfd_.clrDefaultMap();
  }
}

void AvionicsEngine::selectComRadio() {
  if (!isLivePageUp()) return;
  softkeys_.selectCom();
}

void AvionicsEngine::selectNavRadio() {
  if (!isLivePageUp()) return;
  softkeys_.selectNav();
}

void AvionicsEngine::tuneComRadio(int direction, bool coarse) {
  if (!isLivePageUp()) return;
  softkeys_.tuneCom(direction, coarse, dataSource_->snapshot());
}

void AvionicsEngine::tuneNavRadio(int direction, bool coarse) {
  if (!isLivePageUp()) return;
  softkeys_.tuneNav(direction, coarse, dataSource_->snapshot());
}

void AvionicsEngine::transferComRadio() {
  if (!isLivePageUp()) return;
  softkeys_.transferCom(dataSource_->snapshot());
}

void AvionicsEngine::transferNavRadio() {
  if (!isLivePageUp()) return;
  softkeys_.transferNav(dataSource_->snapshot());
}

const float* AvionicsEngine::bezelPressLevels() const {
  return page_ == DisplayPage::MultiFunctionDisplay ? mfd_.bezelPressLevels()
                                                    : softkeys_.bezelPressLevels();
}

void AvionicsEngine::renderFrame(int widthPx, int heightPx, float pixelRatio) {
  renderer_.beginFrame(widthPx, heightPx, pixelRatio);

  // No bus power: the GDU screen is simply black, as on the real unit. Fill
  // black explicitly (matching BootScreen) rather than relying on the shell's
  // framebuffer clear.
  if (!displayPowered()) {
    renderer_.fillRect(0.0f, 0.0f, static_cast<float>(widthPx),
                       static_cast<float>(heightPx), colors::kBlack);
    renderer_.endFrame();
    return;
  }

  if (!bootComplete()) {
    // PFD: Garmin logo fades up, then the initialization view cross-fades in.
    // MFD: the power-up page (combined logo + hero bitmap) fades in directly;
    // once the fade completes, live sources show the ENT acknowledgement prompt.
    BootScreen::Phase phase = BootScreen::Phase::PowerUp;
    float phaseAlpha = 0.0f;
    if (page_ == DisplayPage::MultiFunctionDisplay) {
      if (bootElapsedSeconds_ < kBootPowerUpFadeSeconds) {
        const float t = static_cast<float>(
            bootElapsedSeconds_ / kBootPowerUpFadeSeconds);
        phaseAlpha = bootEaseInOut(t);
      } else {
        phaseAlpha = 1.0f;
      }
    } else {
      phase = bootElapsedSeconds_ < kBootLogoSeconds ? BootScreen::Phase::Logo
                                                    : BootScreen::Phase::PowerUp;
      if (phase == BootScreen::Phase::Logo) {
        const float t = static_cast<float>(bootElapsedSeconds_ /
                                            kBootLogoFadeSeconds);
        phaseAlpha = bootEaseInOut(t);
      } else {
        const double fadeStart = kBootLogoSeconds;
        const double fadeEnd = kBootLogoSeconds + kBootPowerUpFadeSeconds;
        if (bootElapsedSeconds_ < fadeEnd) {
          const float t = static_cast<float>(
              (bootElapsedSeconds_ - fadeStart) / kBootPowerUpFadeSeconds);
          phaseAlpha = bootEaseInOut(t);
        } else {
          phaseAlpha = 1.0f;
        }
      }
    }
    const BootScreen::Target target =
        page_ == DisplayPage::MultiFunctionDisplay ? BootScreen::Target::Mfd
                                                   : BootScreen::Target::Pfd;
    BootScreen::render(renderer_, target, phase, dataSource_->snapshot(),
                       dataSource_->mapSnapshot(), softkeys_, mfd_,
                       dataSource_->mapSnapshot().navDatabase,
                       awaitingPowerUpAck(), dataSource_->aircraftIcaoType(),
                       dataSource_->aircraftAcfRelativePath(),
                       dataSource_->checklistSourcePath(), phaseAlpha,
                       widthPx, heightPx);
    renderer_.endFrame();
    return;
  }

  // When the link is not delivering fresh data, draw the live page but with all
  // sensors marked failed, so each instrument shows its own red-X failure
  // annunciation rather than (potentially misleading) stale data.
  const bool connected =
      dataSource_->connectionState() == ConnectionState::Connected;
  FlightData data = connected ? dataSource_->snapshot()
                              : withAllSensorsFailed(dataSource_->snapshot());
  // The root CDI softkey can override which nav source the HSI displays
  // (GPS/VOR1/VOR2); resolve the effective source here so both the HSI and the
  // CDI annunciation draw consistently.
  data.cdiSource = softkeys_.cdiSourceFor(data.cdiSource);
  applyComDecodedIdents(data, dataSource_->mapSnapshot(),
                        softkeys_.navFeatureSource());
  // VNAV profile (FPL "Active VNV Profile" box + PFD vertical deviation) is
  // computed from the live plan and state; skip it when the link is down.
  if (connected) {
    const MapData& map = dataSource_->mapSnapshot();
    applyGlidepath(data, map);
    if (data.missedApproachActive) {
      applyMissedClimbProfile(data, computeMissedClimbProfile(map, data));
    } else {
      applyVnav(data, map);
    }
  }
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      PrimaryFlightDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                   softkeys_, dataSource_->eisLayoutSnapshot(),
                                   pfdReversionary(), widthPx, heightPx);
      break;
    case DisplayPage::MultiFunctionDisplay:
      if (mfdReversionary()) {
        PrimaryFlightDisplay::render(
            renderer_, data, dataSource_->mapSnapshot(), softkeys_,
            dataSource_->eisLayoutSnapshot(), /*reversionary=*/false, widthPx,
            heightPx);
      } else {
        MultiFunctionDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                     dataSource_->checklistSnapshot(),
                                     dataSource_->eisLayoutSnapshot(), mfd_,
                                     softkeys_, widthPx, heightPx);
      }
      break;
  }

  renderer_.endFrame();
}

}  // namespace avionics
