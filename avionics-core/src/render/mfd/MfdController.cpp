#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>

#include "avionics/FplRouteEdit.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/render/BezelKeys.h"
#include "render/map/MapViewInternal.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

// Softkey cell assignments for the MFD, matching the NXi trainer / WT
// MFDNavMapRootMenu. Page groups (MAP/WPT/AUX/NRST) and map range are on the
// FMS knob and RNG rocker, not the softkey bar.
constexpr int kKeyEngine = 0;
constexpr int kKeyMapOpt = 2;
constexpr int kKeyDetail = 9;
constexpr int kKeyCharts = 10;
constexpr int kKeyChecklist = 11;

// WPT - Airport Information page softkey bar (NXi trainer apt_054..058). The
// page's sub-views are selected here: Info (Airport), DP, STAR, APR, and WX
// (Weather). The Chart cell reuses the chart-view shortcut (selectChartsPage)
// at its trainer position. Map Opt (kKeyMapOpt) is shown only on the Airport
// and Weather views; the procedure views blank it, like the trainer.
constexpr int kKeyWptChart = 3;
constexpr int kKeyWptInfo = 4;
constexpr int kKeyWptDp = 5;
constexpr int kKeyWptStar = 6;
constexpr int kKeyWptApr = 7;
constexpr int kKeyWptWx = 8;

// Engine submenu (Engine softkey, WT EngineMenu).
constexpr int kKeyEngEngine = 0;
constexpr int kKeyEngLean = 1;
constexpr int kKeyEngSystem = 2;
constexpr int kKeyEngBack = 10;

// Map Opt submenu (WT MapOptMenu).
constexpr int kKeyOptTraffic = 0;
constexpr int kKeyOptTer = 3;
constexpr int kKeyOptAwy = 4;
constexpr int kKeyOptNexrad = 6;
constexpr int kKeyOptLegend = 9;
constexpr int kKeyOptBack = 10;

// MAP - Weather Radar page root bar (WT MFDWeatherRadarRootMenu).
constexpr int kKeyRdrMode = 3;
constexpr int kKeyRdrHorizon = 5;
constexpr int kKeyRdrVertical = 6;
constexpr int kKeyRdrGain = 8;
constexpr int kKeyRdrBrg = 10;
// Mode submenu (WT MFDWeatherRadarModeMenu).
constexpr int kKeyRdrStandby = 2;
constexpr int kKeyRdrWeather = 4;
constexpr int kKeyRdrGround = 5;
constexpr int kKeyRdrModeBack = 10;

// MAP - Traffic Map page root bar (WT MFDTrafficMapRootMenu).
constexpr int kKeyTfcAdsb = 2;
constexpr int kKeyTfcStby = 4;
constexpr int kKeyTfcOper = 5;
constexpr int kKeyTfcTest = 6;
constexpr int kKeyTfcMotion = 9;
constexpr int kKeyTfcAltMode = 10;

// NRST - Nearest Airports page extras (WT MFDNearestAirportRootMenu).
constexpr int kKeyNrstApt = 4;
constexpr int kKeyNrstRnwy = 5;
constexpr int kKeyNrstFreq = 6;
constexpr int kKeyNrstApr = 7;
// NRST - Nearest VOR page extras (WT MFDNearestVorRootMenu).
constexpr int kKeyNrstVor = 4;
constexpr int kKeyNrstVorFreq = 5;

// FPL - Flight Plan Catalog page softkeys (NXi trainer screenshot 060, in
// on-unit order). Edit / Import / Export operate on the SD-card Stored Flight
// Plan page, which this suite does not model, so they are shown greyed.
constexpr int kKeyCatNew = 3;
constexpr int kKeyCatActivate = 4;
constexpr int kKeyCatInvert = 5;
constexpr int kKeyCatEdit = 6;
constexpr int kKeyCatCopy = 7;
constexpr int kKeyCatDelete = 8;
constexpr int kKeyCatImport = 9;
constexpr int kKeyCatExport = 10;

// AUX - System Setup page (WT MFDSystemSetupRootMenu).
constexpr int kKeySetup1 = 5;
constexpr int kKeySetup2 = 6;
constexpr int kKeyDefaults = 9;

// AUX - SIMBRIEF page extras on the root bar (free cells beside Checklist):
// the Navigraph sign-in toggle and the OFP fetch trigger.
constexpr int kKeyNavigraphLogin = 8;
constexpr int kKeySimbriefFetch = 9;

// AUX - Charts page softkeys (Pilot's Guide §8.3 chart selection / CHRT Opt).
constexpr int kKeyChartsChrtOpt = 3;
constexpr int kKeyChartsChart = 4;
constexpr int kKeyChartsInfo = 5;
constexpr int kKeyChartsDp = 6;
constexpr int kKeyChartsStar = 7;
constexpr int kKeyChartsApr = 8;
constexpr int kKeyChartsAux = 9;     // Full SCN (selection) / Fit WDTH (opt)
constexpr int kKeyChartsGoBack = 10; // Go Back (selection) / Back (opt)

// PROC - Approach/Arrival/Departure Loading page softkey bar (trainer
// proc_048 / proc_050): category switches + Go Back, shown while a PROC
// selection window is open. DP/STAR/APR sit at the trainer cell positions.
constexpr int kKeyProcLoadDp = 3;
constexpr int kKeyProcLoadStar = 4;
constexpr int kKeyProcLoadApr = 5;
constexpr int kKeyProcLoadGoBack = 10;

// The root-bar "Charts" softkey label (kKeyCharts). Shared so the press handler
// can tell the Charts shortcut apart from pages that reuse that cell (the
// Weather Radar page's BRG key sits on the same index).
constexpr const char* kChartsSoftkeyLabel = "Charts";

// State-carrying softkey labels, verbatim from the NXi Pilot's Guide ("Select
// the TER Softkey until 'Topo' is shown...", "AWY Off/On/LO/HI", "The Detail
// Softkey label advances to Detail All, Detail 3, Detail 2 and Detail 1").
const char* terLabel(TerrainDisplay t) {
  switch (t) {
    case TerrainDisplay::Topo:
      return "TER Topo";
    case TerrainDisplay::Rel:
      return "TER REL";
    case TerrainDisplay::Off:
      break;
  }
  return "TER Off";
}

const char* awyLabel(AirwayDisplay a) {
  switch (a) {
    case AirwayDisplay::All:
      return "AWY On";
    case AirwayDisplay::Low:
      return "AWY LO";
    case AirwayDisplay::High:
      return "AWY HI";
    case AirwayDisplay::Off:
      break;
  }
  return "AWY Off";
}

// Shared root-bar cells from MFDNavMapRootMenu + MFDRootMenu (trainer default).
void applyNavMapRootLabels(std::array<std::string, 12>& labels,
                           MapDetail detail) {
  labels[kKeyEngine] = "Engine";
  labels[kKeyMapOpt] = "Map Opt";
  labels[kKeyDetail] = mapDetailLabel(detail);
  labels[kKeyCharts] = kChartsSoftkeyLabel;
  labels[kKeyChecklist] = "Checklist";
}

// First page of each group, in MfdPage enum order. Keep in sync with the page
// counts below.
constexpr MfdPage kGroupFirstPage[] = {
    MfdPage::NavigationMap,        // Map
    MfdPage::AirportInformation,   // Waypoint
    MfdPage::TripPlanning,         // Aux
    MfdPage::NearestAirports,      // Nearest
    MfdPage::ActiveFlightPlan,     // FlightPlan
};

constexpr int kGroupPageCount[] = {
    3,  // Map: Navigation Map / Traffic Map / Weather Radar
    4,  // Waypoint: Airport / Intersection / NDB / VOR Information
    6,  // Aux: Trip Planning / Utility / GPS Status / System Setup / System
        //      Status / SimBrief (charts live on WPT - Airport Information)
    6,  // Nearest: Airports / Intersections / NDB / VOR / Frequencies /
        //          Airspaces
    2,  // FlightPlan: Active Flight Plan / Flight Plan Catalog
};

}  // namespace

MfdController::MfdController() {
  // Map Settings defaults. The Map group matches Pilot's Guide Fig. 5-7; the
  // other groups default to showing their symbols (the WT NXi defaults), out to
  // a sensible declutter range. Shared controls (Orientation, Terrain, NEXRAD,
  // Traffic) read the existing members, so they are not seeded here.
  auto setRange = [&](MapSetting id, float nm) {
    int best = 0;
    for (int i = 0; i < kMapRangeLadderCount; ++i) {
      if (kMapRangeLadderNm[i] <= nm + 0.01f) best = i;
    }
    msRange_[static_cast<std::size_t>(id)] = best;
  };
  auto setToggle = [&](MapSetting id, bool on) {
    msToggle_[static_cast<std::size_t>(id)] = on;
  };
  // Map group (Fig. 5-7).
  setToggle(MapSetting::ObstacleOn, true);
  setToggle(MapSetting::WindVectorOn, true);
  setRange(MapSetting::NorthUpAboveRange, 1000.0f);
  setRange(MapSetting::TerrainRange, 1000.0f);
  setRange(MapSetting::ObstacleRange, 10.0f);
  // Weather group.
  setRange(MapSetting::NexradRange, kNexradMapRangeDefaultNm);
  // Traffic group.
  setToggle(MapSetting::TrafficLabelsOn, true);
  setRange(MapSetting::TrafficSymbolsRange, kTrafficMapRangeDefaultNm);
  setRange(MapSetting::TrafficLabelsRange, kTrafficMapRangeDefaultNm);
  // Aviation group.
  setToggle(MapSetting::LargeAirportOn, true);
  setToggle(MapSetting::MediumAirportOn, true);
  setToggle(MapSetting::SmallAirportOn, true);
  setToggle(MapSetting::IntOn, true);
  setToggle(MapSetting::NdbOn, true);
  setToggle(MapSetting::VorOn, true);
  setRange(MapSetting::LargeAirportRange, kAirportMaxRangeNm);
  setRange(MapSetting::MediumAirportRange, kMediumAirportMaxRangeNm);
  setRange(MapSetting::SmallAirportRange, kSmallAirportMaxRangeNm);
  setRange(MapSetting::IntRange, 25.0f);
  setRange(MapSetting::NdbRange, 25.0f);
  setRange(MapSetting::VorRange, 150.0f);
  // Airspace group.
  setToggle(MapSetting::ClassBOn, true);
  setToggle(MapSetting::ClassCOn, true);
  setToggle(MapSetting::ClassDOn, true);
  setToggle(MapSetting::RestrictedOn, true);
  setToggle(MapSetting::MoaOn, true);
  setToggle(MapSetting::OtherOn, true);
  setRange(MapSetting::ClassBRange, 100.0f);
  setRange(MapSetting::ClassCRange, 100.0f);
  setRange(MapSetting::ClassDRange, 50.0f);
  setRange(MapSetting::RestrictedRange, 100.0f);
  setRange(MapSetting::MoaRange, 250.0f);
  setRange(MapSetting::OtherRange, 250.0f);
  // Land group.
  setToggle(MapSetting::UserWaypointOn, true);
  setRange(MapSetting::UserWaypointRange, 25.0f);

  // The FPL insert window can load a published airway by name (the Direct-To
  // and Waypoint pages cannot), so only its entry resolves airway idents.
  fplEntry_.allowAirways = true;

  rebuildLabels();
}

void MfdController::rebuildLabels() {
  for (std::string& l : labels_) l.clear();
  // The Map Settings window pushes an empty softkey menu on the real unit, so
  // the bar is blank while it is open.
  if (mapSettingsOpen_) return;
  // A PROC selection window (Approach / Arrival / Departure Loading) shows the
  // category-switch bar with Go Back, regardless of the page underneath it.
  if (procMenuOpen_ && procSelectMode()) {
    labels_[kKeyEngine] = "Engine";
    labels_[kKeyProcLoadDp] = "DP";
    labels_[kKeyProcLoadStar] = "STAR";
    labels_[kKeyProcLoadApr] = "APR";
    labels_[kKeyProcLoadGoBack] = "Go Back";
    if (checklistCount() > 0) labels_[kKeyChecklist] = "Checklist";
    return;
  }
  if (menu_ == Menu::Engine) {
    labels_[kKeyEngEngine] = "Engine";
    labels_[kKeyEngLean] = "Lean";
    labels_[kKeyEngSystem] = "System";
    labels_[kKeyEngBack] = "Back";
    return;
  }
  if (menu_ == Menu::MapOpt) {
    labels_[kKeyOptTraffic] = "Traffic";
    labels_[kKeyOptTer] = terLabel(terrain_);
    labels_[kKeyOptAwy] = awyLabel(airways_);
    labels_[kKeyOptNexrad] = "NEXRAD";
    labels_[kKeyOptLegend] = "Legend";
    labels_[kKeyOptBack] = "Back";
    return;
  }
  if (menu_ == Menu::RadarMode) {
    labels_[kKeyRdrStandby] = "Standby";
    labels_[kKeyRdrWeather] = "Weather";
    labels_[kKeyRdrGround] = "Ground";
    labels_[kKeyRdrModeBack] = "Back";
    return;
  }
  if (page() == MfdPage::WeatherRadar) {
    labels_[kKeyEngine] = "Engine";
    labels_[kKeyRdrMode] = "Mode";
    labels_[kKeyRdrHorizon] = "Horizon";
    labels_[kKeyRdrVertical] = "Vertical";
    labels_[kKeyRdrGain] = "Gain";
    labels_[kKeyRdrBrg] =
        radarScan_ == RadarScan::Vertical ? "Tilt" : "BRG";
    return;
  }
  if (page() == MfdPage::TrafficMap) {
    labels_[kKeyEngine] = "Engine";
    labels_[kKeyTfcAdsb] = "ADS-B";
    labels_[kKeyTfcStby] = "TAS STBY";
    labels_[kKeyTfcOper] = "TAS OPER";
    labels_[kKeyTfcTest] = "Test";
    labels_[kKeyTfcMotion] = "Motion";
    labels_[kKeyTfcAltMode] = "ALT Mode";
    return;
  }
  if (page() == MfdPage::SystemSetup) {
    labels_[kKeyEngine] = "Engine";
    labels_[kKeySetup1] = "Setup 1";
    labels_[kKeySetup2] = "Setup 2";
    labels_[kKeyDefaults] = "Defaults";
    labels_[kKeyChecklist] = "Checklist";
    return;
  }
  if (page() == MfdPage::NearestAirports) {
    applyNavMapRootLabels(labels_, detail_);
    labels_[kKeyNrstApt] = "APT";
    labels_[kKeyNrstRnwy] = "RNWY";
    labels_[kKeyNrstFreq] = "FREQ";
    labels_[kKeyNrstApr] = "APR";
    labels_[kKeyDetail] = "LD APR";
    return;
  }
  if (page() == MfdPage::NearestVor) {
    applyNavMapRootLabels(labels_, detail_);
    labels_[kKeyNrstVor] = "VOR";
    labels_[kKeyNrstVorFreq] = "FREQ";
    labels_[kKeyDetail].clear();
    return;
  }
  if (page() == MfdPage::SimBrief) {
    applyNavMapRootLabels(labels_, detail_);
    labels_[kKeyDetail].clear();
    labels_[kKeyCharts].clear();
    // Sign-in is automatic (the page shows a QR + code when signed out), so the
    // only manual control is Logout once a session exists.
    labels_[kKeyNavigraphLogin] =
        simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn ? "Logout"
                                                                   : "";
    labels_[kKeySimbriefFetch] = "FETCH";
    return;
  }
  if (chartViewActive_) {
    applyNavMapRootLabels(labels_, detail_);
    labels_[kKeyDetail].clear();
    labels_[kKeyCharts].clear();
    if (chartsMenu_ == ChartsMenu::ChartOpt) {
      labels_[kKeyChartsChrtOpt] = "All";
      labels_[kKeyChartsChart] = "Header";
      labels_[kKeyChartsInfo] = "Plan";
      labels_[kKeyChartsDp] = "Profile";
      labels_[kKeyChartsStar] = "Minimums";
      labels_[kKeyChartsApr] = "Fit WDTH";
      labels_[kKeyChartsAux] = "Full SCN";
      labels_[kKeyChartsGoBack] = "Back";
    } else {
      labels_[kKeyChartsChrtOpt] = "CHRT Opt";
      labels_[kKeyChartsChart] = "Show Map";
      labels_[kKeyChartsInfo] = "Info";
      labels_[kKeyChartsDp] = "DP";
      labels_[kKeyChartsStar] = "STAR";
      labels_[kKeyChartsApr] = "APR";
      labels_[kKeyChartsAux] = "Full SCN";
      labels_[kKeyChartsGoBack] = "Go Back";
    }
    return;
  }
  if (pageGroup_ == MfdPageGroup::Waypoint &&
      page() == MfdPage::AirportInformation) {
    // WPT - Airport Information sub-view bar (trainer apt_054..058): Engine,
    // Map Opt, Chart, Info, DP, STAR, APR, WX, Checklist. Map Opt is hidden on
    // the procedure sub-views (Departure/Arrival/Approach), present on the
    // Airport and Weather views.
    labels_[kKeyEngine] = "Engine";
    if (wptInfoView_ == WptInfoView::Airport ||
        wptInfoView_ == WptInfoView::Weather) {
      labels_[kKeyMapOpt] = "Map Opt";
    }
    labels_[kKeyWptChart] = "Chart";
    labels_[kKeyWptInfo] = "Info";
    labels_[kKeyWptDp] = "DP";
    labels_[kKeyWptStar] = "STAR";
    labels_[kKeyWptApr] = "APR";
    labels_[kKeyWptWx] = "WX";
    labels_[kKeyChecklist] = "Checklist";
    return;
  }
  if (pageGroup_ == MfdPageGroup::FlightPlan) {
    if (page() == MfdPage::FlightPlanCatalog) {
      // Flight Plan Catalog softkey bar (NXi trainer screenshot 060).
      labels_[kKeyEngine] = "Engine";
      labels_[kKeyMapOpt] = "Map Opt";
      labels_[kKeyCatNew] = "New";
      labels_[kKeyCatActivate] = "Activate";
      labels_[kKeyCatInvert] = "Invert";
      labels_[kKeyCatEdit] = "Edit";
      labels_[kKeyCatCopy] = "Copy";
      labels_[kKeyCatDelete] = "Delete";
      labels_[kKeyCatImport] = "Import";
      labels_[kKeyCatExport] = "Export";
      labels_[kKeyChecklist] = "Checklist";
      return;
    }
    labels_[kKeyEngine] = "Engine";
    labels_[kKeyMapOpt] = "Map Opt";
    labels_[kKeyCharts] = kChartsSoftkeyLabel;
    labels_[kKeyChecklist] = "Checklist";
    return;
  }
  applyNavMapRootLabels(labels_, detail_);
}

float MfdController::rangeNm() const { return mapRangeNmAt(rangeIndex_); }

void MfdController::setRangeFromNm(float rangeNm) {
  rangeIndex_ = mapRangeIndexForNm(rangeNm);
}

void MfdController::setProcPreviewFitRange(int ladderIndex) {
  if (procPreviewRangeManual_) return;
  rangeIndex_ = std::max(0, std::min(kMapRangeLadderCount - 1, ladderIndex));
  displayRangeNm_ = mapRangeNmAt(rangeIndex_);
}

void MfdController::setFplPreviewFitRange(int ladderIndex) {
  if (fplPreviewRangeManual_) return;
  rangeIndex_ = std::max(0, std::min(kMapRangeLadderCount - 1, ladderIndex));
  displayRangeNm_ = mapRangeNmAt(rangeIndex_);
}

bool MfdController::stepMapRange(int direction) {
  const int before = rangeIndex_;
  if (direction > 0) {
    rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
  } else if (direction < 0) {
    rangeIndex_ = std::max(0, rangeIndex_ - 1);
  }
  return rangeIndex_ != before;
}

void MfdController::setMapDetail(MapDetail detail) {
  detail_ = detail;
  rebuildLabels();
}

float MfdController::mapViewHalfExtentNm() const {
  if (!mapViewportValid_ || mapViewportW_ <= 0.0f || mapViewportH_ <= 0.0f) {
    return 0.0f;
  }
  MapViewConfig cfg{};
  cfg.w = mapViewportW_;
  cfg.h = mapViewportH_;
  cfg.orientation = mapOrientation_;
  const float mapRadiusPx = mapview::mapRangeSpanPx(cfg);
  const float scaleRangeNm =
      std::max(kMapRangeMinNm, displayRangeNm_ > 0.0f ? displayRangeNm_ : rangeNm());
  if (mapRadiusPx <= 0.0f || scaleRangeNm <= 0.0f) return 0.0f;
  const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
  const float halfW = mapViewportW_ * 0.5f;
  const float halfH = mapViewportH_ * 0.5f;
  return std::sqrt(halfW * halfW + halfH * halfH) / pixelsPerNm;
}

int MfdController::pageCount(MfdPageGroup group) const {
  int count = kGroupPageCount[static_cast<int>(group)];
  // The Weather Radar page is the last page in the MAP group; drop it when the
  // airframe has no radar so it never enters the rotation.
  if (group == MfdPageGroup::Map && !weatherRadarAvailable_) --count;
  return count;
}

void MfdController::setWeatherRadarAvailable(bool available) {
  if (weatherRadarAvailable_ == available) return;
  weatherRadarAvailable_ = available;
  // If the Weather Radar page is no longer offered while it (or a now-invalid
  // index) is selected, fall back to the Navigation Map and close any
  // radar-specific submenu so the bar rebuilds for the visible page.
  if (!available && pageGroup_ == MfdPageGroup::Map) {
    int& index = pageIndex_[static_cast<int>(MfdPageGroup::Map)];
    if (index >= pageCount(MfdPageGroup::Map)) {
      index = 0;
      if (menu_ == Menu::RadarMode) menu_ = Menu::Root;
      mapResetPointer();
    }
  }
  rebuildLabels();
}

int MfdController::pageIndex() const {
  if (pageGroup_ == MfdPageGroup::Checklist) return checklistIndex_;
  return pageIndex_[static_cast<int>(pageGroup_)];
}

MfdPage MfdController::page() const {
  // Checklist is its own page group (not in kGroupFirstPage); avoid indexing
  // past the table and misrouting softkey/bezel handlers.
  if (pageGroup_ == MfdPageGroup::Checklist) {
    return MfdPage::NavigationMap;
  }
  return static_cast<MfdPage>(
      static_cast<int>(kGroupFirstPage[static_cast<int>(pageGroup_)]) +
      pageIndex());
}

void MfdController::stepPage(int direction) {
  pageSelectSec_ = kPageSelectSeconds;
  menu_ = Menu::Root;  // close any page-specific submenu when the page changes
  pageMenuOpen_ = false;
  mapSettingsOpen_ = false;
  chartViewActive_ = false;  // leaving the page closes its chart view
  wptInfoView_ = WptInfoView::Airport;  // and its WPT sub-view selection
  if (pageGroup_ == MfdPageGroup::Checklist) {
    stepChecklist(direction);
    return;
  }
  const int count = pageCount(pageGroup_);
  int& index = pageIndex_[static_cast<int>(pageGroup_)];
  index = ((index + direction) % count + count) % count;
  if (pageGroup_ == MfdPageGroup::Nearest) {
    nrstSelected_ = 0;
  }
  if (pageGroup_ == MfdPageGroup::Map && page() != MfdPage::NavigationMap) {
    mapResetPointer();
  }
}

void MfdController::selectGroup(MfdPageGroup group) {
  // Pressing the already-selected group's key steps to its next page (the
  // softkey doubles as the small FMS knob); selecting a new group restores
  // that group's last-viewed page.
  if (pageGroup_ == group) {
    stepPage(1);
  } else {
    syncFplPreviewRange(group);
    if (pageGroup_ == MfdPageGroup::Map) mapResetPointer();
    if (pageGroup_ == MfdPageGroup::Waypoint) wptResetInteraction();
    if (pageGroup_ == MfdPageGroup::Nearest) nrstResetInteraction();
    chartViewActive_ = false;
    pageGroup_ = group;
    menu_ = Menu::Root;
    pageSelectSec_ = kPageSelectSeconds;
  }
}

void MfdController::update(double dtSeconds, const FlightData& data) {
  const float pressStep = static_cast<float>(dtSeconds) / kPressFlashSeconds;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    press_[i] = std::max(0.0f, press_[i] - pressStep);
  }
  for (int i = 0; i < kBezelKeyCount; ++i) {
    bezelPress_[i] = std::max(0.0f, bezelPress_[i] - pressStep);
  }
  pageSelectSec_ =
      std::max(0.0f, pageSelectSec_ - static_cast<float>(dtSeconds));
  displayRangeNm_ =
      animateMapRange(displayRangeNm_, mapRangeNmAt(rangeIndex_), dtSeconds);

  // Pop-up windows ease toward their open/closed target at a constant rate, so
  // they slide+fade in when opened and out when closed -- the same logic the
  // PFD pop-ups use (SoftkeyController::windowAnim_).
  const float animStep = static_cast<float>(dtSeconds) / kWindowAnimSeconds;
  auto approachAnim = [animStep](float current, bool open) {
    const float target = open ? 1.0f : 0.0f;
    if (current < target) return std::min(target, current + animStep);
    return std::max(target, current - animStep);
  };
  dtoAnim_ = approachAnim(dtoAnim_, dtoOpen_);
  pageMenuAnim_ = approachAnim(pageMenuAnim_, pageMenuOpen_);
  mapSettingsAnim_ = approachAnim(mapSettingsAnim_, mapSettingsOpen_);
  loadAirway_.anim = approachAnim(loadAirway_.anim, loadAirway_.open);

  // ~1 Hz blink for highlight-select cursor fields: on for the first half of
  // each second (matches SoftkeyController::blinkOn_ and WT pulse).
  blinkSeconds_ += dtSeconds;
  blinkOn_ = std::fmod(blinkSeconds_, 1.0) < 0.5;

  // Antenna sweep position for the Weather Radar page scan line (one look per
  // kRadarSweepSeconds; the page maps this to a left/right ping-pong angle).
  radarSweepPhase_ += dtSeconds / kRadarSweepSeconds;
  radarSweepPhase_ = std::fmod(radarSweepPhase_, 1.0);

  // AUX Utility timers / trip statistics: accumulate only while live data is
  // coming in (a dead link freezes the timers rather than running on stale
  // values).
  if (data.dataLinkValid) {
    FlightSessionStats& s = flightStats_;
    s.genericTimerSec += dtSeconds;
    const float gs = data.groundSpeedKts;
    // In-air detection for the flight timer / departure time: airspeed alive
    // above a rotation-ish threshold (the unit's "In-Air" criterion).
    if (!s.airborneSeen && data.airspeedValid && data.airspeedKts >= 40.0f) {
      s.airborneSeen = true;
      s.departureHour = data.utcHour;
      s.departureMinute = data.utcMinute;
    }
    if (s.airborneSeen) s.flightTimerSec += dtSeconds;
    s.odometerNm += static_cast<double>(gs) * dtSeconds / 3600.0;
    if (gs >= 5.0f) s.movingTimeSec += dtSeconds;
    s.maxGroundSpeedKts = std::max(s.maxGroundSpeedKts, gs);
  }

  updateCatalogAutoRefresh();
}

bool MfdController::keyEnabled(int i) const {
  if (labels_[i].empty()) return false;
  if (procMenuOpen_ && procSelectMode()) {
    return i == kKeyEngine || i == kKeyProcLoadDp || i == kKeyProcLoadStar ||
           i == kKeyProcLoadApr || i == kKeyProcLoadGoBack ||
           (i == kKeyChecklist && checklistCount() > 0);
  }
  if (menu_ == Menu::Engine) {
    return i == kKeyEngEngine || i == kKeyEngBack;
  }
  if (menu_ == Menu::MapOpt) {
    return true;
  }
  if (menu_ == Menu::RadarMode && i == kKeyRdrGround) {
    return false;
  }
  if (menu_ == Menu::Root) {
    if (chartViewActive_) {
      if (chartsMenu_ == ChartsMenu::ChartOpt) {
        // The view-slice keys and Fit WDTH need a chart shown (not the map).
        if (chartsShowMap_) return i == kKeyChartsGoBack;
        return true;
      }
      // DP and STAR stay selectable so the pilot can switch to those sections
      // regardless of whether the current index has a chart of that type; the
      // remaining category softkeys grey when no chart of that type exists
      // (Pilot's Guide §8.3 "if available"). Show Map / CHRT Opt / Full SCN /
      // Go Back are always available.
      if (i == kKeyChartsInfo)
        return chartsHasCategory(ChartsCategoryFilter::Airport);
      if (i == kKeyChartsDp) return true;
      if (i == kKeyChartsStar) return true;
      if (i == kKeyChartsApr)
        return chartsHasCategory(ChartsCategoryFilter::Approach);
    }
    // The Charts softkey is a shortcut to the chart view (it stays available
    // so charts are reachable to sign in / browse).
    if (i == kKeyCharts) return true;
    if (i == kKeyChecklist) return checklistCount() > 0;
    if (page() == MfdPage::TrafficMap) {
      return false;  // TAS / ADS-B controls not modeled yet.
    }
    if (page() == MfdPage::SystemSetup &&
        (i == kKeySetup1 || i == kKeySetup2)) {
      return false;
    }
    if (page() == MfdPage::NearestAirports && i == kKeyDetail) {
      return false;  // LD APR (replaces Detail on this page).
    }
    if (page() == MfdPage::FlightPlanCatalog) {
      // Edit / Import / Export operate on the SD-card Stored Flight Plan page,
      // which this suite does not model (greyed, like the real unit's unused
      // SD slot). The plan-specific actions need a stored plan to act on.
      if (i == kKeyCatEdit || i == kKeyCatImport || i == kKeyCatExport) {
        return false;
      }
      if (i == kKeyCatActivate || i == kKeyCatInvert || i == kKeyCatCopy ||
          i == kKeyCatDelete) {
        return !catalog_.empty();
      }
    }
    if (page() == MfdPage::SimBrief && i == kKeyNavigraphLogin) {
      // Only Logout is a manual action (sign-in is automatic via the QR code);
      // it is local-only (clears the saved token) so it stays available offline.
      return simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn;
    }
    if (page() == MfdPage::SimBrief && i == kKeySimbriefFetch) {
      return simbriefState_.commAllowed &&
             simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn &&
             simbriefState_.status != SimBriefStatus::Fetching;
    }
  }
  return true;
}

bool MfdController::keyActive(int i) const {
  if (procMenuOpen_ && procSelectMode()) {
    switch (i) {
      case kKeyProcLoadDp:
        return procCategory() == ProcedureType::Departure;
      case kKeyProcLoadStar:
        return procCategory() == ProcedureType::Arrival;
      case kKeyProcLoadApr:
        return procCategory() == ProcedureType::Approach;
      default:
        return false;
    }
  }
  if (menu_ == Menu::Engine) {
    return i == kKeyEngEngine;
  }
  if (menu_ == Menu::MapOpt) {
    switch (i) {
      case kKeyOptTraffic:
        return showTraffic_;
      case kKeyOptTer:
        return terrain_ != TerrainDisplay::Off;
      case kKeyOptAwy:
        return airways_ != AirwayDisplay::Off;
      case kKeyOptNexrad:
        return showWeather_;
      case kKeyOptLegend:
        return mapSettingOn(MapSetting::TopoScaleOn);
      default:
        return false;
    }
  }
  if (menu_ == Menu::RadarMode) {
    switch (i) {
      case kKeyRdrStandby:
        return radarMode_ == RadarMode::Standby;
      case kKeyRdrWeather:
        return radarMode_ == RadarMode::Weather;
      case kKeyRdrGround:
        return radarMode_ == RadarMode::Ground;
      default:
        return false;
    }
  }
  if (page() == MfdPage::WeatherRadar) {
    switch (i) {
      case kKeyRdrHorizon:
        return radarScan_ == RadarScan::Horizontal;
      case kKeyRdrVertical:
        return radarScan_ == RadarScan::Vertical;
      case kKeyRdrGain:
        return !radarGainCalibrated_;  // lit while in manual gain
      case kKeyRdrBrg:
        return radarScan_ != RadarScan::Vertical && radarBearingLineOn_;
      default:
        return false;
    }
  }
  if (page() == MfdPage::TrafficMap && i == kKeyTfcAdsb) {
    return showTraffic_;
  }
  if (chartViewActive_) {
    if (chartsMenu_ == ChartsMenu::ChartOpt) {
      switch (i) {
        case kKeyChartsChrtOpt:
          return chartsViewMode_ == ChartsViewMode::All;
        case kKeyChartsChart:
          return chartsViewMode_ == ChartsViewMode::Header;
        case kKeyChartsInfo:
          return chartsViewMode_ == ChartsViewMode::Plan;
        case kKeyChartsDp:
          return chartsViewMode_ == ChartsViewMode::Profile;
        case kKeyChartsStar:
          return chartsViewMode_ == ChartsViewMode::Minimums;
        case kKeyChartsApr:
          return chartsFitWidth_;
        case kKeyChartsAux:
          return chartsFullScreen_;
        default:
          return false;
      }
    }
    if (i == kKeyChartsChart) return chartsShowMap_;
    if (i == kKeyChartsAux) return chartsFullScreen_;
    if (i == kKeyChartsInfo)
      return chartsCategoryFilter_ == ChartsCategoryFilter::Airport;
    if (i == kKeyChartsDp)
      return chartsCategoryFilter_ == ChartsCategoryFilter::Departure;
    if (i == kKeyChartsStar)
      return chartsCategoryFilter_ == ChartsCategoryFilter::Arrival;
    if (i == kKeyChartsApr)
      return chartsCategoryFilter_ == ChartsCategoryFilter::Approach;
  }
  if (pageGroup_ == MfdPageGroup::Waypoint &&
      page() == MfdPage::AirportInformation && !chartViewActive_) {
    switch (i) {
      case kKeyWptInfo:
        return wptInfoView_ == WptInfoView::Airport;
      case kKeyWptDp:
        return wptInfoView_ == WptInfoView::Departure;
      case kKeyWptStar:
        return wptInfoView_ == WptInfoView::Arrival;
      case kKeyWptApr:
        return wptInfoView_ == WptInfoView::Approach;
      case kKeyWptWx:
        return wptInfoView_ == WptInfoView::Weather;
      default:
        break;
    }
  }
  switch (i) {
    case kKeyChecklist:
      return pageGroup_ == MfdPageGroup::Checklist;
    default:
      return false;
  }
}

void MfdController::flashBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i >= 0 && i < kBezelKeyCount) bezelPress_[i] = 1.0f;
}

void MfdController::pressBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i < 0 || i >= kBezelKeyCount) return;
  bezelPress_[i] = 1.0f;  // trigger the press-flash animation

  // The "Fly Course Reversal?" prompt is modal: it owns the FMS knob / ENT / CLR
  // until answered, overlaying whatever page is up.
  if (courseReversalPromptBezelKey(key)) {
    rebuildLabels();
    return;
  }

  if (holdActivatePromptBezelKey(key)) {
    rebuildLabels();
    return;
  }

  // The Direct-To window owns the FMS knob / ENT / CLR while it is open; FPL,
  // PROC, and MENU dismiss it and navigate like the real unit.
  if (directToBezelKey(key)) {
    rebuildLabels();
    return;
  }

  // The Map Settings window is modal over the navigation map: it owns the FMS
  // knob / ENT / CLR until the FMS knob push or CLR closes it. The RANGE rocker
  // still zooms the map underneath (Pilot's Guide).
  if (mapSettingsOpen_ && !isMapRangePanBezelKey(key)) {
    mapSettingsBezelKey(key);
    rebuildLabels();
    return;
  }

  // The Page Menu (MENU key) is modal over the base page while it is up: it
  // owns the FMS knob / ENT / CLR until an option is run or it is backed out.
  // RANGE zoom still applies to the base page map.
  if (pageMenuOpen_ && !isMapRangePanBezelKey(key)) {
    pageMenuBezelKey(key);
    rebuildLabels();
    return;
  }

  // The Select Airway window is modal over the FPL page: it owns the FMS knob /
  // ENT / CLR until Load? runs or it is backed out. RANGE still zooms the map.
  if (loadAirway_.open && !isMapRangePanBezelKey(key)) {
    loadAirwayBezelKey(key);
    rebuildLabels();
    return;
  }

  // The Procedures window owns the FMS knob / ENT / CLR while it is open
  // (Pilot's Guide 5.8). It must run before the FPL page handler: PROC is
  // opened as an overlay on the FPL page, and fplBezelKey would otherwise
  // consume knob turns for list scrolling even when the proc menu is up.
  if (procMenuOpen_) {
    if (procBezelKey(key)) {
      rebuildLabels();
      return;
    }
    // Modal overlay: knob / ENT / CLR must not fall through to the FPL page
    // or page-group routing when the proc handler does not recognize the key.
    if (!isMapRangePanBezelKey(key) && !isPageNavigationBezelKey(key)) {
      switch (key) {
        case BezelKey::FmsInnerCw:
        case BezelKey::FmsInnerCcw:
        case BezelKey::FmsOuterCw:
        case BezelKey::FmsOuterCcw:
        case BezelKey::FmsPush:
        case BezelKey::Ent:
        case BezelKey::Clr:
          rebuildLabels();
          return;
        default:
          break;
      }
    }
  }

  // The FPL page owns the FMS knob / ENT / CLR / MENU while it is up (cursor,
  // waypoint entry, remove confirmation); unconsumed keys fall through to the
  // common handling below (FPL toggle, range rocker).
  if (pageGroup_ == MfdPageGroup::FlightPlan && fplBezelKey(key)) {
    rebuildLabels();
    return;
  }

  if (pageGroup_ == MfdPageGroup::Map && mapBezelKey(key)) {
    rebuildLabels();
    return;
  }
  if (pageGroup_ == MfdPageGroup::Map && page() == MfdPage::WeatherRadar &&
      radarBezelKey(key)) {
    rebuildLabels();
    return;
  }
  // Chart view owns the FMS knob / RANGE joystick on the Airport Information
  // page; it must take precedence over the normal waypoint-page handler.
  if (chartViewActive_ && chartsBezelKey(key)) {
    rebuildLabels();
    return;
  }
  if (pageGroup_ == MfdPageGroup::Waypoint && !chartViewActive_ &&
      wptBezelKey(key)) {
    rebuildLabels();
    return;
  }
  if (pageGroup_ == MfdPageGroup::Nearest && nrstBezelKey(key)) {
    rebuildLabels();
    return;
  }

  switch (key) {
    case BezelKey::RangeUp:
      rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
      if (pageGroup_ == MfdPageGroup::FlightPlan) fplPreviewRangeManual_ = true;
      if (procMenuOpen_ && procSelectMode()) procPreviewRangeManual_ = true;
      break;
    case BezelKey::RangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      if (pageGroup_ == MfdPageGroup::FlightPlan) fplPreviewRangeManual_ = true;
      if (procMenuOpen_ && procSelectMode()) procPreviewRangeManual_ = true;
      break;
    case BezelKey::Fpl:
      // FPL toggles the Active Flight Plan page; pressing it again returns to
      // the page that was displayed before.
      if (pageGroup_ == MfdPageGroup::FlightPlan) {
        syncFplPreviewRange(groupBeforeFpl_);
        pageGroup_ = groupBeforeFpl_;
        fplResetInteraction();
      } else {
        syncFplPreviewRange(MfdPageGroup::FlightPlan);
        groupBeforeFpl_ = pageGroup_;
        pageGroup_ = MfdPageGroup::FlightPlan;
        // Open the page with the FMS cursor inactive, like the real unit: no fix
        // is selected until the FMS knob is pushed, and until then the large /
        // small knobs navigate page groups / pages rather than scrolling the
        // list. fplResetInteraction() also clears fplPreviewRangeManual_.
        fplResetInteraction();
      }
      chartViewActive_ = false;
      pageSelectSec_ = kPageSelectSeconds;
      break;
    case BezelKey::Proc:
      // PROC opens the Procedures window over the current MFD page (Pilot's
      // Guide 5.8); pressing it again closes it. Opening rebuilds the top-level
      // menu so the activate items reflect the current plan.
      if (procMenuOpen_) {
        procMenuOpen_ = false;
      } else {
        procMenuOpen_ = true;
        buildProcMenu();
      }
      procPreviewRangeManual_ = false;
      break;
    case BezelKey::FmsInnerCw:
      // Small FMS knob: move the item cursor down the checklist, else step
      // pages within the active group (Pilot's Guide, "Page Selection").
      if (pageGroup_ == MfdPageGroup::Checklist) {
        cursorItem_ =
            std::min(checklistItemCount(checklistIndex_), cursorItem_ + 1);
      } else {
        stepPage(1);
      }
      break;
    case BezelKey::FmsInnerCcw:
      if (pageGroup_ == MfdPageGroup::Checklist) {
        cursorItem_ = std::max(0, cursorItem_ - 1);
      } else {
        stepPage(-1);
      }
      break;
    case BezelKey::FmsOuterCw:
      // Large FMS knob: select the page group.
      stepPageGroup(1);
      break;
    case BezelKey::FmsOuterCcw:
      stepPageGroup(-1);
      break;
    case BezelKey::Menu:
      // MENU opens the current page's Page Menu (no-op on pages without one).
      openPageMenu();
      break;
    case BezelKey::Ent:
      if (pageGroup_ == MfdPageGroup::Checklist) checklistEnter();
      break;
    case BezelKey::Clr:
      if (pageGroup_ == MfdPageGroup::Checklist) checklistClear();
      break;
    default:
      break;
  }
  rebuildLabels();  // the page (and so the ID/FETCH keys) may have changed
}

void MfdController::stepPageGroup(int direction) {
  // The large knob cycles every page group, including FPL and Checklist, in the
  // on-unit order shown on the page-select popup tabs (drawPageIndicator):
  // MAP -> WPT -> AUX -> FPL -> NRST -> CHK and wrap. The FPL and Checklist keys
  // remain shortcuts to those groups, but the knob no longer skips them.
  static constexpr MfdPageGroup kCycle[] = {
      MfdPageGroup::Map,        MfdPageGroup::Waypoint, MfdPageGroup::Aux,
      MfdPageGroup::FlightPlan, MfdPageGroup::Nearest,  MfdPageGroup::Checklist};
  constexpr int kCycleCount =
      static_cast<int>(sizeof(kCycle) / sizeof(kCycle[0]));
  pageSelectSec_ = kPageSelectSeconds;
  menu_ = Menu::Root;
  pageMenuOpen_ = false;
  mapSettingsOpen_ = false;
  wptInfoView_ = WptInfoView::Airport;
  MfdPageGroup target = MfdPageGroup::Map;
  for (int i = 0; i < kCycleCount; ++i) {
    if (kCycle[i] == pageGroup_) {
      target = kCycle[((i + direction) % kCycleCount + kCycleCount) %
                      kCycleCount];
      break;
    }
  }
  if (target == pageGroup_) return;
  syncFplPreviewRange(target);
  // Drop any per-group interaction state of the group we are leaving, mirroring
  // selectGroup() so the knob and the group keys behave identically.
  switch (pageGroup_) {
    case MfdPageGroup::Map:
      mapResetPointer();
      break;
    case MfdPageGroup::Waypoint:
      wptResetInteraction();
      break;
    case MfdPageGroup::Nearest:
      nrstResetInteraction();
      break;
    case MfdPageGroup::FlightPlan:
      fplResetInteraction();
      break;
    default:
      break;
  }
  chartViewActive_ = false;
  // Entering FPL by the knob behaves like the FPL key: the page opens with the
  // cursor off, and the FPL key still toggles back to the prior group.
  if (target == MfdPageGroup::FlightPlan) {
    groupBeforeFpl_ = pageGroup_;
    fplResetInteraction();
  }
  pageGroup_ = target;
}

void MfdController::clrDefaultMap() {
  // Cancel whatever is in progress, exactly like backing all the way out, then
  // bring up the MAP - NAVIGATION MAP page.
  fplResetInteraction();
  wptResetInteraction();
  nrstResetInteraction();
  mapResetPointer();
  dtoOpen_ = false;
  dtoArmed_ = false;
  dtoEntry_.reset();
  menu_ = Menu::Root;
  pageMenuOpen_ = false;
  mapSettingsOpen_ = false;
  procMenuOpen_ = false;
  chartViewActive_ = false;
  chartsAirportEntry_.reset();
  chartsAirportOverride_.clear();
  syncFplPreviewRange(MfdPageGroup::Map);
  pageGroup_ = MfdPageGroup::Map;
  pageIndex_[static_cast<int>(MfdPageGroup::Map)] = 0;
  rebuildLabels();
}

bool MfdController::pressKey(int key) {
  if (key < 0 || key >= kSoftkeyCount || labels_[key].empty()) return false;
  if (!keyEnabled(key)) return false;

  press_[key] = 1.0f;  // trigger the press-flash animation

  if (menu_ == Menu::Engine) {
    if (key == kKeyEngBack) menu_ = Menu::Root;
    rebuildLabels();
    return true;
  }

  if (menu_ == Menu::MapOpt) {
    switch (key) {
      case kKeyOptTraffic:
        showTraffic_ = !showTraffic_;
        break;
      case kKeyOptTer:
        terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Topo
                   : terrain_ == TerrainDisplay::Topo ? TerrainDisplay::Rel
                                                       : TerrainDisplay::Off;
        break;
      case kKeyOptAwy:
        airways_ = airways_ == AirwayDisplay::Off   ? AirwayDisplay::All
                   : airways_ == AirwayDisplay::All ? AirwayDisplay::Low
                   : airways_ == AirwayDisplay::Low ? AirwayDisplay::High
                                                    : AirwayDisplay::Off;
        break;
      case kKeyOptNexrad:
        showWeather_ = !showWeather_;
        break;
      case kKeyOptLegend:
        msToggle_[static_cast<std::size_t>(MapSetting::TopoScaleOn)] =
            !msToggle_[static_cast<std::size_t>(MapSetting::TopoScaleOn)];
        break;
      case kKeyOptBack:
        menu_ = Menu::Root;
        break;
      default:
        return false;
    }
    rebuildLabels();
    return true;
  }

  if (menu_ == Menu::RadarMode) {
    switch (key) {
      case kKeyRdrStandby:
        radarMode_ = RadarMode::Standby;
        menu_ = Menu::Root;
        break;
      case kKeyRdrWeather:
        radarMode_ = RadarMode::Weather;
        menu_ = Menu::Root;
        break;
      case kKeyRdrGround:
        radarMode_ = RadarMode::Ground;
        menu_ = Menu::Root;
        break;
      case kKeyRdrModeBack:
        menu_ = Menu::Root;
        break;
      default:
        return false;
    }
    rebuildLabels();
    return true;
  }

  if (key == kKeyEngine) {
    menu_ = Menu::Engine;
    rebuildLabels();
    return true;
  }
  if (key == kKeyMapOpt) {
    menu_ = Menu::MapOpt;
    rebuildLabels();
    return true;
  }
  // PROC selection window: DP / STAR / APR switch the loaded category in place;
  // Go Back returns to the Procedures menu (Pilot's Guide 5.8).
  if (procMenuOpen_ && procSelectMode()) {
    ProcedureMenuHost host = procedureMenuHost();
    switch (key) {
      case kKeyProcLoadDp:
        procedureMenuOpenArrDepSelect(host, ProcedureType::Departure);
        rebuildLabels();
        return true;
      case kKeyProcLoadStar:
        procedureMenuOpenArrDepSelect(host, ProcedureType::Arrival);
        rebuildLabels();
        return true;
      case kKeyProcLoadApr:
        host.state.category = ProcedureType::Approach;
        procedureMenuOpenApproachSelect(host);
        rebuildLabels();
        return true;
      case kKeyProcLoadGoBack:
        buildProcMenu();
        rebuildLabels();
        return true;
      default:
        break;
    }
  }
  // WPT - Airport Information sub-view bar (trainer apt_054..058): Chart enters
  // the chart view; Info / DP / STAR / APR / WX switch the information panel.
  if (pageGroup_ == MfdPageGroup::Waypoint &&
      page() == MfdPage::AirportInformation && !chartViewActive_) {
    switch (key) {
      case kKeyWptChart:
        selectChartsPage();
        rebuildLabels();
        return true;
      case kKeyWptInfo:
        wptInfoView_ = WptInfoView::Airport;
        rebuildLabels();
        return true;
      case kKeyWptDp:
        wptInfoView_ = WptInfoView::Departure;
        rebuildLabels();
        return true;
      case kKeyWptStar:
        wptInfoView_ = WptInfoView::Arrival;
        rebuildLabels();
        return true;
      case kKeyWptApr:
        wptInfoView_ = WptInfoView::Approach;
        rebuildLabels();
        return true;
      case kKeyWptWx:
        wptInfoView_ = WptInfoView::Weather;
        rebuildLabels();
        return true;
      default:
        break;
    }
  }
  // WPT - Airport Information chart view keys (Pilot's Guide §8.3).
  if (chartViewActive_) {
    if (chartsMenu_ == ChartsMenu::ChartOpt) {
      switch (key) {
        case kKeyChartsChrtOpt:
          chartsViewMode_ = ChartsViewMode::All;
          break;
        case kKeyChartsChart:
          chartsViewMode_ = ChartsViewMode::Header;
          break;
        case kKeyChartsInfo:
          chartsViewMode_ = ChartsViewMode::Plan;
          break;
        case kKeyChartsDp:
          chartsViewMode_ = ChartsViewMode::Profile;
          break;
        case kKeyChartsStar:
          chartsViewMode_ = ChartsViewMode::Minimums;
          break;
        case kKeyChartsApr:  // Fit WDTH: base-fit to width, reset zoom/pan.
          chartsFitWidth_ = !chartsFitWidth_;
          chartsZoom_ = 1.0f;
          chartsPanXFrac_ = 0.0f;
          chartsPanYFrac_ = 0.0f;
          break;
        case kKeyChartsAux:
          chartsFullScreen_ = !chartsFullScreen_;
          break;
        case kKeyChartsGoBack:
          chartsMenu_ = ChartsMenu::Selection;
          break;
        default:
          break;
      }
    } else {
      switch (key) {
        case kKeyChartsChrtOpt:
          chartsMenu_ = ChartsMenu::ChartOpt;
          break;
        case kKeyChartsChart:  // "Show Map": chart image <-> nav map
          chartsShowMap_ = !chartsShowMap_;
          break;
        case kKeyChartsInfo:  // airport diagram / airport-info chart
          chartsSelectCategory(ChartsCategoryFilter::Airport);
          break;
        case kKeyChartsDp:
          chartsSelectCategory(ChartsCategoryFilter::Departure);
          break;
        case kKeyChartsStar:
          chartsSelectCategory(ChartsCategoryFilter::Arrival);
          break;
        case kKeyChartsApr:
          chartsSelectCategory(ChartsCategoryFilter::Approach);
          break;
        case kKeyChartsAux:
          chartsFullScreen_ = !chartsFullScreen_;
          break;
        case kKeyChartsGoBack:
          // Leave the chart view and return to the page we came from (Pilot's
          // Guide §8.3): clearing chart view restores the normal softkey bar.
          chartViewActive_ = false;
          pageGroup_ = groupBeforeCharts_;
          break;
        default:
          break;
      }
    }
    rebuildLabels();
    return true;
  }
  // FPL - Flight Plan Catalog softkeys (New / Activate / Invert / Copy /
  // Delete). The route-changing and destructive actions open the same
  // confirmation window as the bezel ENT path; New and Copy act immediately.
  if (pageGroup_ == MfdPageGroup::FlightPlan &&
      page() == MfdPage::FlightPlanCatalog) {
    switch (key) {
      case kKeyCatNew:
        catalogCreateNew();
        rebuildLabels();
        return true;
      case kKeyCatActivate:
        catalogConfirm_ = CatalogConfirm::Activate;
        catalogConfirmOk_ = true;
        rebuildLabels();
        return true;
      case kKeyCatInvert:
        catalogConfirm_ = CatalogConfirm::InvertActivate;
        catalogConfirmOk_ = true;
        rebuildLabels();
        return true;
      case kKeyCatCopy:
        catalogCopySelected();
        rebuildLabels();
        return true;
      case kKeyCatDelete:
        catalogConfirm_ = CatalogConfirm::Delete;
        catalogConfirmOk_ = true;
        rebuildLabels();
        return true;
      default:
        break;
    }
  }
  // Charts softkey: jump to the AUX - Charts page. Guarded on the label so it
  // does not fire on pages that reuse this cell for another function (BRG on
  // the Weather Radar page).
  if (key == kKeyCharts && labels_[kKeyCharts] == kChartsSoftkeyLabel) {
    selectChartsPage();
    rebuildLabels();
    return true;
  }
  if (key == kKeyChecklist) {
    selectGroup(MfdPageGroup::Checklist);
    rebuildLabels();
    return true;
  }
  if (key == kKeyDetail) {
    detail_ = nextMapDetail(detail_);
    rebuildLabels();
    return true;
  }
  if (key == kKeyNavigraphLogin) {
    // Sign-in is automatic (QR code); only Logout is a manual action here.
    if (simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn) {
      navigraphLogoutRequested_ = true;  // local-only; allowed offline
    }
    rebuildLabels();
    return true;
  }
  if (key == kKeySimbriefFetch) {
    if (simbriefState_.commAllowed &&
        simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn &&
        simbriefState_.status != SimBriefStatus::Fetching) {
      simbriefFetchRequested_ = true;
    }
    rebuildLabels();
    return true;
  }

  if (page() == MfdPage::WeatherRadar) {
    switch (key) {
      case kKeyRdrMode:
        menu_ = Menu::RadarMode;
        break;
      case kKeyRdrHorizon:
        radarScan_ = RadarScan::Horizontal;
        break;
      case kKeyRdrVertical:
        radarScan_ = RadarScan::Vertical;
        break;
      case kKeyRdrGain:
        // Toggle calibrated <-> manual gain (Pilot's Guide: the Gain Softkey
        // activates manual gain; selecting it again restores Calibrated).
        radarGainCalibrated_ = !radarGainCalibrated_;
        radarGainManual_ = radarGainCalibrated_ ? 0.0f : 0.4f;
        break;
      case kKeyRdrBrg:
        if (radarScan_ != RadarScan::Vertical) {
          radarBearingLineOn_ = !radarBearingLineOn_;
          if (!radarBearingLineOn_) radarBearingDeg_ = 0.0f;
        }
        break;
      default:
        return false;
    }
    rebuildLabels();
    return true;
  }

  return false;
}

void MfdController::updateNavigraphAutoLogin() {
  // The pilot dislikes pressing Login: when the SimBrief page is open and we
  // aren't signed in, kick off the Navigraph device-authorization flow
  // automatically so the page can show the QR + code to scan. Only when the sim
  // session permits Navigraph traffic (commAllowed). Treat Error the same as
  // signed-out so an expired code is refreshed on its own.
  const bool needsCode =
      simbriefState_.loginPhase == NavigraphLoginPhase::LoggedOut ||
      simbriefState_.loginPhase == NavigraphLoginPhase::Error;
  const bool wantAuto =
      page() == MfdPage::SimBrief && simbriefState_.commAllowed && needsCode;
  if (!wantAuto) {
    // Disarm whenever the condition clears (signed in, awaiting user, off page,
    // or offline) so the next signed-out visit issues a fresh code.
    navigraphAutoLoginArmed_ = false;
    return;
  }
  if (!navigraphAutoLoginArmed_) {
    navigraphAutoLoginArmed_ = true;
    navigraphLoginRequested_ = true;  // drained by the shell -> requestLogin()
  }
}

bool MfdController::consumeNavigraphLoginRequest() {
  const bool requested = navigraphLoginRequested_;
  navigraphLoginRequested_ = false;
  return requested;
}

bool MfdController::consumeNavigraphLogoutRequest() {
  const bool requested = navigraphLogoutRequested_;
  navigraphLogoutRequested_ = false;
  return requested;
}

bool MfdController::consumeSimbriefFetchRequest() {
  const bool requested = simbriefFetchRequested_;
  simbriefFetchRequested_ = false;
  return requested;
}

bool MfdController::consumeCatalogRefreshRequest() {
  const bool requested = catalogRefreshRequested_;
  catalogRefreshRequested_ = false;
  return requested;
}

void MfdController::updateCatalogAutoRefresh() {
  // Fire a one-shot refresh request the moment the pilot navigates into the
  // Flight Plan Catalog page (not while merely sitting on it). Mirrors the FETCH
  // softkey guard: only when signed in, the sim link is up, and no fetch is in
  // flight. Re-fetching the same route is harmless -- the catalog dedupes by
  // leg sequence and just updates the existing slot.
  const MfdPage current = page();
  if (current == MfdPage::FlightPlanCatalog &&
      lastPageForCatalogRefresh_ != MfdPage::FlightPlanCatalog) {
    if (simbriefState_.commAllowed &&
        simbriefState_.loginPhase == NavigraphLoginPhase::LoggedIn &&
        simbriefState_.status != SimBriefStatus::Fetching) {
      catalogRefreshRequested_ = true;
    }
  }
  lastPageForCatalogRefresh_ = current;
}

namespace {

bool chartMatchesFilter(const ChartListItem& chart,
                        ChartsCategoryFilter filter) {
  switch (filter) {
    case ChartsCategoryFilter::All:
      return true;
    case ChartsCategoryFilter::Departure:
      return chart.category == ChartCategory::Departure;
    case ChartsCategoryFilter::Arrival:
      return chart.category == ChartCategory::Arrival;
    case ChartsCategoryFilter::Approach:
      return chart.category == ChartCategory::Approach;
    case ChartsCategoryFilter::Airport:
      return chart.category == ChartCategory::Airport;
  }
  return true;
}

// Index of the first chart matching `filter`, or -1 when none match.
int firstChartInCategory(const std::vector<ChartListItem>& charts,
                         ChartsCategoryFilter filter) {
  for (int i = 0; i < static_cast<int>(charts.size()); ++i) {
    if (chartMatchesFilter(charts[static_cast<std::size_t>(i)], filter)) {
      return i;
    }
  }
  return -1;
}

}  // namespace

void MfdController::setChartsState(const ChartsState& state) {
  // Reset the list cursor when the airport (and therefore the chart list)
  // changes; otherwise keep it, clamped to the new list length.
  const bool airportChanged = state.airportIcao != chartsState_.airportIcao;
  chartsState_ = state;
  const int count = static_cast<int>(chartsState_.charts.size());
  if (airportChanged) {
    // Open on the airport diagram (Info / Navigraph "APT") with the selection
    // list filtered to that category, matching the real NXi default, rather
    // than the unfiltered "All" mix. -1 leaves nothing selected when the
    // airport has no airport chart (the pilot can pick DP/STAR/APR instead).
    chartsCategoryFilter_ = ChartsCategoryFilter::Airport;
    chartsSelected_ =
        firstChartInCategory(chartsState_.charts, chartsCategoryFilter_);
    chartsResetView();
  } else {
    if (chartsSelected_ < 0) chartsSelected_ = 0;
    if (count > 0 && chartsSelected_ >= count) chartsSelected_ = count - 1;
  }
  // Keep the popup highlight valid and, when not actively browsing, parked on
  // the committed chart.
  if (airportChanged) chartsPending_ = chartsSelected_;
  if (chartsPending_ < 0) chartsPending_ = 0;
  if (count > 0 && chartsPending_ >= count) chartsPending_ = count - 1;
}

bool MfdController::fplDestinationFilledForLayout() const {
  if (fplApproachLegCount_ > 0) return true;
  if (!fplDestinationFilled_ || fplLegs_.empty()) return false;
  // Confirmed airports always count; a digit-bearing airport code (e.g. K1H2)
  // counts even when the nav database has not loaded that field, so the
  // destination never falls through to the Enroute section.
  const std::string& last = fplLegs_.back().id;
  return isKnownAirportIdent(last, mapData_, navSource_) ||
         isAirportCodeWithDigit(last);
}

std::string MfdController::chartsDestinationAirport() const {
  int approachStart = fplApproachLegStart_;
  int approachCount = fplApproachLegCount_;
  const int arrivalEnd =
      fplArrivalLegCount_ > 0 ? fplArrivalLegStart_ + fplArrivalLegCount_ : 0;
  if (!fplHasLoadedApproach() || approachCount <= 0 ||
      approachStart < arrivalEnd) {
    const InferredProcedureBlock block =
        inferProcedureBlockInPlan(fplLegs_, arrivalEnd);
    if (block.valid()) {
      approachStart = block.start;
      approachCount = block.count;
    } else {
      approachStart = 0;
      approachCount = 0;
    }
  }
  const bool approachLoaded = approachCount > 0;
  std::string approachAirport;
  if (approachLoaded) {
    approachAirport = fplApproachAirportIcao();
    if (!isKnownAirportIdent(approachAirport, mapData_, navSource_) &&
        approachStart > 0 &&
        approachStart <= static_cast<int>(fplLegs_.size())) {
      const std::string candidate =
          fplLegs_[static_cast<std::size_t>(approachStart - 1)].id;
      if (!(approachStart == 1 && !fplLegs_.empty() &&
            candidate == fplLegs_.front().id && isAirportIdent(candidate)) &&
          isKnownAirportIdent(candidate, mapData_, navSource_)) {
        approachAirport = candidate;
      } else {
        approachAirport.clear();
      }
    }
  }
  std::string icao = pfd::fplHeaderDestinationIdent(
      fplLegs_, fplDestinationFilled_, approachStart, approachLoaded,
      approachAirport);
  if (!isKnownAirportIdent(icao, mapData_, navSource_)) {
    icao = lastKnownAirportInPlan(fplLegs_, mapData_, navSource_);
  }
  if (icao.empty() && mapData_ != nullptr) {
    icao = lastKnownAirportInPlan(mapData_->flightPlan, mapData_, navSource_);
  }
  if (icao.empty() &&
      isKnownAirportIdent(simbriefState_.destinationIcao, mapData_,
                          navSource_)) {
    icao = simbriefState_.destinationIcao;
  }
  return icao;
}

std::string MfdController::chartsOriginAirport() const {
  std::string icao = firstKnownAirportInPlan(fplLegs_, mapData_, navSource_);
  if (icao.empty() && mapData_ != nullptr) {
    icao = firstKnownAirportInPlan(mapData_->flightPlan, mapData_, navSource_);
  }
  if (icao.empty() &&
      isKnownAirportIdent(simbriefState_.originIcao, mapData_, navSource_)) {
    icao = simbriefState_.originIcao;
  }
  return icao;
}

std::string MfdController::chartsDesiredAirport() const {
  // An explicitly entered airport (Airport-box ICAO entry) wins, so charts for
  // any field can be pulled up, not just the flight-plan ends.
  if (!chartsAirportOverride_.empty()) return chartsAirportOverride_;
  // Charts are per-airport: resolve a 4-letter ICAO airport ident (fixes like
  // BOSTN are not valid). Honor the Airport-box origin/dest choice, but fall
  // back to the other end when the selected one has no airport.
  const std::string preferred =
      chartsUseDestination_ ? chartsDestinationAirport() : chartsOriginAirport();
  if (!preferred.empty()) return preferred;
  return chartsUseDestination_ ? chartsOriginAirport() : chartsDestinationAirport();
}

bool MfdController::chartsHasCategory(ChartsCategoryFilter filter) const {
  for (const ChartListItem& c : chartsState_.charts) {
    if (chartMatchesFilter(c, filter)) return true;
  }
  return false;
}

void MfdController::chartsSelectCategory(ChartsCategoryFilter filter) {
  // A category softkey (DP / STAR / APR / Info) shows that chart immediately
  // (Pilot's Guide §8.3), so it commits the selection rather than opening the
  // browse popup.
  chartsCategoryFilter_ = filter;
  const int n = static_cast<int>(chartsState_.charts.size());
  for (int i = 0; i < n; ++i) {
    if (chartMatchesFilter(chartsState_.charts[static_cast<std::size_t>(i)],
                           filter)) {
      if (i != chartsSelected_) chartsResetView();
      chartsSelected_ = i;
      chartsPending_ = i;
      return;
    }
  }
}

void MfdController::chartsCommitSelection() {
  // ENT applies the highlighted popup row as the shown chart (Fig 8-20 step 9).
  if (chartsPending_ == chartsSelected_) return;
  chartsSelected_ = chartsPending_;
  chartsResetView();
}

void MfdController::chartsCommitAirportEntry() {
  // ENT applies the typed Airport-box ident as the charts airport. Prefer a
  // resolved airport match; otherwise take the typed ident verbatim (the shell
  // validates the 4-letter ICAO and reports NO CHARTS if it has none).
  FmsWaypointEntry& e = chartsAirportEntry_;
  std::string icao =
      (e.hasMatch && e.match.type == MapFeatureType::Airport && !e.match.id.empty())
          ? e.match.id
          : e.ident();
  e.reset();
  chartsField_ = ChartsField::Airport;
  if (icao.empty()) return;
  chartsAirportOverride_ = icao;
}

void MfdController::chartsStepSelection(int delta) {
  // Scrolling the open popup moves the highlight only; the displayed chart does
  // not change until ENT commits it (chartsCommitSelection).
  const int n = static_cast<int>(chartsState_.charts.size());
  if (n == 0) return;
  int i = chartsPending_;
  for (int step = 0; step < n; ++step) {
    i = (i + delta + n) % n;
    if (chartMatchesFilter(chartsState_.charts[static_cast<std::size_t>(i)],
                           chartsCategoryFilter_)) {
      chartsPending_ = i;
      return;
    }
  }
}

bool MfdController::chartsAnyAirportAvailable() const {
  return !chartsOriginAirport().empty() || !chartsDestinationAirport().empty();
}

std::string MfdController::chartsSelectedChartId() const {
  if (chartsSelected_ < 0 ||
      chartsSelected_ >= static_cast<int>(chartsState_.charts.size())) {
    return {};
  }
  return chartsState_.charts[chartsSelected_].id;
}

void MfdController::chartsResetView() {
  chartsZoom_ = 1.0f;
  chartsPanXFrac_ = 0.0f;
  chartsPanYFrac_ = 0.0f;
  chartsFitWidth_ = false;
}

bool MfdController::chartsBezelKey(BezelKey key) {
  if (!chartViewActive_) return false;

  // Free ICAO entry on the Airport box: the FMS knob spells an airport ident and
  // ENT applies it (Pilot's Guide, WPT - Airport Information ident search). This
  // runs before the cursor-off / field-toggle actions so it owns the knob while
  // editing.
  if (chartsAirportEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        chartsCommitAirportEntry();
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        chartsAirportEntry_.reset();
        return true;
      case BezelKey::FmsInnerCw:
        chartsAirportEntry_.turnChar(navSource_, mapData_, +1);
        return true;
      case BezelKey::FmsInnerCcw:
        chartsAirportEntry_.turnChar(navSource_, mapData_, -1);
        return true;
      case BezelKey::FmsOuterCw:
        chartsAirportEntry_.moveCursor(navSource_, mapData_, +1);
        return true;
      case BezelKey::FmsOuterCcw:
        chartsAirportEntry_.moveCursor(navSource_, mapData_, -1);
        return true;
      default:
        return true;  // swallow other keys while editing the ident
    }
  }

  // FMS knob push leaves chart view and returns to the airport-info display
  // (the cursor-off action on the real unit's Airport Information page).
  if (key == BezelKey::FmsPush) {
    chartViewActive_ = false;
    return true;
  }

  // RANGE joystick: zoom and pan the chart image. When the Chart softkey is
  // showing the nav map instead, fall through so RANGE drives the map range.
  if (!chartsShowMap_ && isMapRangePanBezelKey(key)) {
    constexpr float kZoomStep = 1.25f;
    constexpr float kMaxZoom = 8.0f;
    constexpr float kPanStep = 0.08f;
    constexpr float kPanLimit = 0.5f;
    auto clampPan = [kPanLimit](float v) {
      return std::max(-kPanLimit, std::min(kPanLimit, v));
    };
    switch (key) {
      case BezelKey::RangeDown:  // joystick CCW: zoom in
        chartsZoom_ = std::min(kMaxZoom, chartsZoom_ * kZoomStep);
        return true;
      case BezelKey::RangeUp:  // joystick CW: zoom out
        chartsZoom_ = std::max(1.0f, chartsZoom_ / kZoomStep);
        if (chartsZoom_ <= 1.001f) {
          chartsPanXFrac_ = 0.0f;
          chartsPanYFrac_ = 0.0f;
        }
        return true;
      case BezelKey::PanUp:
        chartsPanYFrac_ = clampPan(chartsPanYFrac_ + kPanStep);
        return true;
      case BezelKey::PanDown:
        chartsPanYFrac_ = clampPan(chartsPanYFrac_ - kPanStep);
        return true;
      case BezelKey::PanLeft:
        chartsPanXFrac_ = clampPan(chartsPanXFrac_ + kPanStep);
        return true;
      case BezelKey::PanRight:
        chartsPanXFrac_ = clampPan(chartsPanXFrac_ - kPanStep);
        return true;
      case BezelKey::PanPush:  // press recenters (Pilot's Guide: centers chart)
        chartsResetView();
        return true;
      default:
        return false;
    }
  }

  switch (key) {
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsOuterCcw:
      chartsField_ = chartsField_ == ChartsField::Airport ? ChartsField::Approach
                                                        : ChartsField::Airport;
      // Moving the field cursor discards any uncommitted browse: the popup
      // highlight returns to the chart currently shown.
      chartsPending_ = chartsSelected_;
      return true;
    case BezelKey::Ent:
      // ENT commits the highlighted popup row as the shown chart and closes the
      // browse popup (the dropdown is only drawn while the chart field is the
      // active cursor field, so move the cursor off it).
      chartsCommitSelection();
      chartsField_ = ChartsField::Airport;
      return true;
    case BezelKey::FmsInnerCw:
      if (chartsField_ == ChartsField::Airport) {
        // Start free ICAO entry, seeded with the airport currently shown so the
        // flight-plan origin/dest is the starting point.
        chartsAirportEntry_.allowAirways = false;
        chartsAirportEntry_.open(navSource_, mapData_, chartsDesiredAirport());
      } else {
        chartsStepSelection(1);
      }
      return true;
    case BezelKey::FmsInnerCcw:
      if (chartsField_ == ChartsField::Airport) {
        chartsAirportEntry_.allowAirways = false;
        chartsAirportEntry_.open(navSource_, mapData_, chartsDesiredAirport());
      } else {
        chartsStepSelection(-1);
      }
      return true;
    default:
      return false;
  }
}

void MfdController::selectChartsPage() {
  // Charts live on the WPT - Airport Information page (Pilot's Guide §8.3): jump
  // there and turn on chart view. Remember where we came from for "Go Back".
  if (pageGroup_ == MfdPageGroup::Map) mapResetPointer();
  if (pageGroup_ == MfdPageGroup::Nearest) nrstResetInteraction();
  if (!chartViewActive_) groupBeforeCharts_ = pageGroup_;
  pageGroup_ = MfdPageGroup::Waypoint;
  pageIndex_[static_cast<int>(MfdPageGroup::Waypoint)] =
      static_cast<int>(MfdPage::AirportInformation) -
      static_cast<int>(kGroupFirstPage[static_cast<int>(MfdPageGroup::Waypoint)]);
  chartsMenu_ = ChartsMenu::Selection;
  chartViewActive_ = true;
  // Start on the Airport field with the chart-selection popup closed and no
  // ident edit in progress; the popup only opens once the cursor is moved onto
  // the chart field.
  chartsField_ = ChartsField::Airport;
  chartsAirportEntry_.reset();
  menu_ = Menu::Root;
  // No page-select popup: the Charts softkey jumps straight to the chart on the
  // WPT - Airport Information page (Pilot's Guide §8.3), unlike an FMS-knob page
  // change which flashes the page-group navigation overlay.
  pageSelectSec_ = 0.0f;
}

bool MfdController::blocksRadioBezel() const {
  // Map panning is driven by the RANGE joystick, not the FMS knob, so the
  // active Map Pointer does not claim the knob here.
  return dtoOpen_ || dtoEntry_.active || fplEntry_.active ||
         fplAltEntry_.active || fplConfirm_ != FplConfirm::None ||
         wptEntry_.active || procMenuOpen_ || mapSettingsOpen_ ||
         loadAirway_.open;
}

}  // namespace avionics
