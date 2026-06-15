#include "avionics/AvionicsEngine.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "avionics/Color.h"
#include "avionics/ComDecode.h"
#include "avionics/MapData.h"
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

// Computes the active VNAV profile: finds the next constrained, lower waypoint
// ahead of the active leg, builds the descent path to it at the default FPA, and
// derives the required vertical speed, top-of-descent, and path deviation.
VnvProfile computeVnvProfile(const MapData& map, const FlightData& data) {
  VnvProfile vnv;
  if (!map.positionValid || map.flightPlan.size() < 1) return vnv;

  const std::vector<MapLeg>& plan = map.flightPlan;

  // Active leg = the leg whose TO ident matches the FMA active waypoint.
  int activeIdx = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (!data.fmaToWpt.empty() && plan[i].id == data.fmaToWpt) {
      activeIdx = static_cast<int>(i);
      break;
    }
  }

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

  // Along-track distance from ownship to the target (ownship -> active leg, then
  // leg to leg up to the target).
  double distNm = navDistanceNm(map.ownshipLat, map.ownshipLon,
                                plan[activeIdx].lat, plan[activeIdx].lon);
  for (int i = activeIdx; i < targetIdx; ++i) {
    distNm += navDistanceNm(plan[i].lat, plan[i].lon, plan[i + 1].lat,
                            plan[i + 1].lon);
  }

  constexpr double kPi = 3.14159265358979323846;
  const float fpaDeg = kDefaultFpaDeg;
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
  // A secondary engine sharing the source (the MFD window) must not pump it a
  // second time. The pump runs regardless of this GDU's power so the switch
  // datarefs are still read while the screen is dark (and so the other GDU
  // sharing the source keeps getting fresh data when only one is powered).
  if (drivesDataSource_) dataSource_->update(dtSeconds);

  const bool powered = displayPowered();
  if (!powerInitialized_) {
    powerInitialized_ = true;
  } else if (powered && !wasPowered_) {
    // The GDU bus just came alive: replay the Garmin power-up self-test so the
    // screen visibly boots, like a real unit does when its display is switched
    // on. The power-up page is auto-acknowledged (no in-sim ENT prompt): once
    // the timed animation finishes the live page comes up on its own.
    bootElapsedSeconds_ = 0.0;
    powerUpAcknowledged_ = true;
  }
  wasPowered_ = powered;
  if (!powered) return;  // dark screen: no boot timer or UI animation

  bootElapsedSeconds_ += dtSeconds;
  softkeys_.update(dtSeconds, dataSource_->snapshot(),
                   dataSource_->mapSnapshot());
  mfd_.update(dtSeconds, dataSource_->snapshot());
  // Keep the MFD's checklist navigation in step with the loaded file (the data
  // is owned by the source; the controller only holds the interactive state).
  mfd_.syncChecklist(dataSource_->checklistSnapshot());
  // Keep the FPL page's editable plan in step with the active flight plan
  // (and ownship, for waypoint-entry lookups). The active leg's TO ident
  // seeds the Direct-To window's default waypoint.
  mfd_.syncFlightPlan(dataSource_->mapSnapshot(),
                      dataSource_->snapshot().fmaToWpt);
}

bool AvionicsEngine::bootComplete() const {
  // The animated power-up must have run, and any source that requires an ENT
  // acknowledgement of the power-up page must have received it.
  if (bootElapsedSeconds_ < kBootDurationSeconds) return false;
  return powerUpAcknowledged_ || !dataSource_->requiresPowerUpAcknowledge();
}

bool AvionicsEngine::awaitingPowerUpAck() const {
  return bootElapsedSeconds_ >= kBootDurationSeconds && !powerUpAcknowledged_ &&
         dataSource_->requiresPowerUpAcknowledge();
}

void AvionicsEngine::acknowledgePowerUp() {
  if (awaitingPowerUpAck()) powerUpAcknowledged_ = true;
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
         !d.avionicsPowerOn;
}

bool AvionicsEngine::isLivePageUp() const {
  return displayPowered() && bootComplete() &&
         dataSource_->connectionState() == ConnectionState::Connected;
}

void AvionicsEngine::pressSoftkey(int index) {
  if (!isLivePageUp()) return;
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

void AvionicsEngine::pressBezelKey(BezelKey key) {
  // While the power-up page waits for acknowledgement, ENT brings up the live
  // pages; every other bezel key is inert (as on the real unit).
  if (awaitingPowerUpAck()) {
    if (key == BezelKey::Ent) acknowledgePowerUp();
    return;
  }
  if (!isLivePageUp()) return;
  // The dedicated NAV/COM/CRS/BARO/HDG knobs work on either page (both GDUs
  // carry them), so handle them before the page-specific routing.
  if (handleBezelKnob(key)) return;
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
    // Phase 1: the Garmin logo fades up from black over kBootLogoFadeSeconds,
    // then holds. Phase 2: the MFD Power-up Page or PFD initialization view
    // fades in over kBootPowerUpFadeSeconds; once the timer elapses, live
    // sources show the ENT acknowledgement prompt on the MFD.
    const BootScreen::Phase phase = bootElapsedSeconds_ < kBootLogoSeconds
                                        ? BootScreen::Phase::Logo
                                        : BootScreen::Phase::PowerUp;
    float phaseAlpha = 0.0f;
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
    const BootScreen::Target target =
        page_ == DisplayPage::MultiFunctionDisplay ? BootScreen::Target::Mfd
                                                   : BootScreen::Target::Pfd;
    BootScreen::render(renderer_, target, phase, dataSource_->snapshot(),
                       dataSource_->mapSnapshot(), softkeys_, mfd_,
                       dataSource_->mapSnapshot().navDatabase,
                       awaitingPowerUpAck(), phaseAlpha, widthPx, heightPx);
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
  if (connected) applyVnav(data, dataSource_->mapSnapshot());
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      PrimaryFlightDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                   softkeys_, dataSource_->eisLayoutSnapshot(),
                                   pfdReversionary(), widthPx, heightPx);
      break;
    case DisplayPage::MultiFunctionDisplay:
      MultiFunctionDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                   dataSource_->checklistSnapshot(),
                                   dataSource_->eisLayoutSnapshot(), mfd_,
                                   softkeys_, widthPx, heightPx);
      break;
  }

  renderer_.endFrame();
}

}  // namespace avionics
