#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>

#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// Softkey cell assignments for the MFD root bar. The four page groups form a
// radio group on the left (standing in for the large FMS knob); range control
// sits on the right two cells, mirroring the G1000 MFD softkey layout. Map Opt
// opens the navigation-map options submenu and Detail cycles the declutter
// level, per the NXi MFD softkey map.
constexpr int kKeyMap = 0;
constexpr int kKeyWaypoint = 1;
constexpr int kKeyAux = 2;
constexpr int kKeyNearest = 3;
constexpr int kKeyOrient = 4;
constexpr int kKeyMapOpt = 5;
constexpr int kKeyDetail = 6;
constexpr int kKeyChecklist = 7;  // Checklist: selects the Checklist page group
constexpr int kKeyRangeDown = 10;
constexpr int kKeyRangeUp = 11;

// Map Opt submenu cells (Pilot's Guide: Traffic, TER, AWY, ..., Back).
constexpr int kKeyOptTraffic = 1;
constexpr int kKeyOptTer = 2;
constexpr int kKeyOptAwy = 3;
constexpr int kKeyOptNexrad = 4;
constexpr int kKeyOptBack = 11;

// MAP - Weather Radar page root bar (Pilot's Guide, Hazard Avoidance -
// Airborne Color Weather Radar): Mode opens the Standby/Weather/Ground submenu;
// Horizon/Vertical pick the scan; the fifth cell is BRG (horizontal scan, the
// bearing line) or Tilt (vertical scan); range stays on the rocker, like the
// other MAP-group pages.
constexpr int kKeyRdrMode = 0;
constexpr int kKeyRdrHorizon = 1;
constexpr int kKeyRdrVertical = 2;
constexpr int kKeyRdrGain = 3;
constexpr int kKeyRdrBrg = 4;
constexpr int kKeyRdrFeatures = 5;
// Mode submenu cells.
constexpr int kKeyRdrStandby = 1;
constexpr int kKeyRdrWeather = 2;
constexpr int kKeyRdrGround = 3;
constexpr int kKeyRdrModeBack = 11;

// AUX - SIMBRIEF page extras on the root bar (free cells beside Checklist), and
// the Pilot ID digit-entry bar (0-9 / BKSP / Back, XPDR-code style).
constexpr int kKeySimbriefId = 8;
constexpr int kKeySimbriefFetch = 9;
constexpr int kKeyEntryBksp = 10;
constexpr int kKeyEntryBack = 11;

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
        //      Status / SimBrief
    6,  // Nearest: Airports / Intersections / NDB / VOR / Frequencies /
        //          Airspaces
    1,  // FlightPlan: Active Flight Plan
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
  setRange(MapSetting::LargeAirportRange, 1000.0f);
  setRange(MapSetting::MediumAirportRange, 100.0f);
  setRange(MapSetting::SmallAirportRange, 25.0f);
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
  if (simbriefIdEntry_) {
    // Pilot ID digit entry replaces the whole bar, like the XPDR Code menu on
    // the PFD: 0-9 with BKSP and Back on the right.
    for (int d = 0; d <= 9; ++d) labels_[d] = static_cast<char>('0' + d);
    labels_[kKeyEntryBksp] = "BKSP";
    labels_[kKeyEntryBack] = "Back";
    return;
  }
  if (menu_ == Menu::MapOpt) {
    labels_[kKeyOptTraffic] = "Traffic";
    labels_[kKeyOptTer] = terLabel(terrain_);
    labels_[kKeyOptAwy] = awyLabel(airways_);
    labels_[kKeyOptNexrad] = "NEXRAD";
    labels_[kKeyOptBack] = "Back";
    return;
  }
  if (menu_ == Menu::RadarMode) {
    // Mode submenu, verbatim from the Pilot's Guide (Mode -> Standby / Weather
    // / Ground).
    labels_[kKeyRdrStandby] = "Standby";
    labels_[kKeyRdrWeather] = "Weather";
    labels_[kKeyRdrGround] = "Ground";
    labels_[kKeyRdrModeBack] = "Back";
    return;
  }
  if (page() == MfdPage::WeatherRadar) {
    labels_[kKeyRdrMode] = "Mode";
    labels_[kKeyRdrHorizon] = "Horizon";
    labels_[kKeyRdrVertical] = "Vertical";
    labels_[kKeyRdrGain] = "Gain";
    // The fifth cell is the bearing line on the horizontal scan and the tilt
    // line on the vertical scan (Figures 6-72 / 6-74).
    labels_[kKeyRdrBrg] =
        radarScan_ == RadarScan::Vertical ? "Tilt" : "BRG";
    labels_[kKeyRdrFeatures] = "Features";
    labels_[kKeyRangeDown] = "RNG-";
    labels_[kKeyRangeUp] = "RNG+";
    return;
  }
  labels_[kKeyMap] = "Map";
  labels_[kKeyWaypoint] = "WPT";
  labels_[kKeyAux] = "AUX";
  labels_[kKeyNearest] = "NRST";
  labels_[kKeyOrient] = "TRK";
  labels_[kKeyMapOpt] = "Map Opt";
  labels_[kKeyDetail] = mapDetailLabel(detail_);
  labels_[kKeyChecklist] = "Checklist";
  labels_[kKeyRangeDown] = "RNG-";
  labels_[kKeyRangeUp] = "RNG+";
  if (page() == MfdPage::SimBrief) {
    labels_[kKeySimbriefId] = "ID";
    labels_[kKeySimbriefFetch] = "FETCH";
  }
}

float MfdController::rangeNm() const { return mapRangeNmAt(rangeIndex_); }

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
  return static_cast<MfdPage>(
      static_cast<int>(kGroupFirstPage[static_cast<int>(pageGroup_)]) +
      pageIndex());
}

void MfdController::stepPage(int direction) {
  pageSelectSec_ = kPageSelectSeconds;
  menu_ = Menu::Root;  // close any page-specific submenu when the page changes
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
    if (pageGroup_ == MfdPageGroup::Map) mapResetPointer();
    if (pageGroup_ == MfdPageGroup::Waypoint) wptResetInteraction();
    if (pageGroup_ == MfdPageGroup::Nearest) nrstResetInteraction();
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
}

bool MfdController::keyActive(int i) const {
  // Digit-entry cells are momentary; no radio/toggle highlight applies.
  if (simbriefIdEntry_) return false;
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
      case kKeyRdrFeatures:
        return radarAct_;
      default:
        return false;
    }
  }
  switch (i) {
    case kKeyMap:
      return pageGroup_ == MfdPageGroup::Map;
    case kKeyWaypoint:
      return pageGroup_ == MfdPageGroup::Waypoint;
    case kKeyAux:
      return pageGroup_ == MfdPageGroup::Aux;
    case kKeyNearest:
      return pageGroup_ == MfdPageGroup::Nearest;
    case kKeyChecklist:
      return pageGroup_ == MfdPageGroup::Checklist;
    case kKeyOrient:
      return mapOrientation_ == MapOrientation::TrackUp;
    default:
      return false;
  }
}

void MfdController::pressBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i < 0 || i >= kBezelKeyCount) return;
  bezelPress_[i] = 1.0f;  // trigger the press-flash animation

  // Pilot ID digit entry is modal, like a cursor field on the real unit: ENT
  // commits the pending digits, CLR erases (cancelling once empty), and the
  // page-navigation keys are inert until the entry is closed.
  if (simbriefIdEntry_) {
    switch (key) {
      case BezelKey::Ent:
        if (!simbriefPendingId_.empty()) {
          simbriefPilotId_ = simbriefPendingId_;
        }
        simbriefPendingId_.clear();
        simbriefIdEntry_ = false;
        break;
      case BezelKey::Clr:
        if (simbriefPendingId_.empty()) {
          simbriefIdEntry_ = false;
        } else {
          simbriefPendingId_.pop_back();
        }
        break;
      default:
        break;
    }
    rebuildLabels();
    return;
  }

  // The Direct-To window is modal over any page: opened by the Direct-To key,
  // it owns the FMS knob / ENT / CLR until it is closed or activated.
  if (directToBezelKey(key)) {
    rebuildLabels();
    return;
  }

  // The Map Settings window is modal over the navigation map: it owns the FMS
  // knob / ENT / CLR until the FMS knob push or CLR closes it.
  if (mapSettingsOpen_) {
    mapSettingsBezelKey(key);
    rebuildLabels();
    return;
  }

  // The Page Menu (MENU key) is modal over the base page while it is up: it
  // owns the FMS knob / ENT / CLR until an option is run or it is backed out.
  if (pageMenuOpen_) {
    pageMenuBezelKey(key);
    rebuildLabels();
    return;
  }

  // The FPL page owns the FMS knob / ENT / CLR / MENU while it is up (cursor,
  // waypoint entry, remove confirmation); unconsumed keys fall through to the
  // common handling below (FPL toggle, range rocker).
  if (pageGroup_ == MfdPageGroup::FlightPlan && fplBezelKey(key)) {
    rebuildLabels();
    return;
  }

  if (procMenuOpen_ && procBezelKey(key)) {
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
  if (pageGroup_ == MfdPageGroup::Waypoint && wptBezelKey(key)) {
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
      break;
    case BezelKey::RangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      break;
    case BezelKey::Fpl:
      // FPL toggles the Active Flight Plan page; pressing it again returns to
      // the page that was displayed before.
      if (pageGroup_ == MfdPageGroup::FlightPlan) {
        pageGroup_ = groupBeforeFpl_;
        fplResetInteraction();
      } else {
        groupBeforeFpl_ = pageGroup_;
        pageGroup_ = MfdPageGroup::FlightPlan;
      }
      pageSelectSec_ = kPageSelectSeconds;
      break;
    case BezelKey::Proc:
      if (pageGroup_ == MfdPageGroup::FlightPlan) {
        procMenuOpen_ = !procMenuOpen_;
        procSelected_ = 0;
        procCategory_ = ProcedureType::Approach;
        procStep_ = ProcMenuStep::ProcedureList;
        procSelectedName_.clear();
      }
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
  // The large knob cycles the softkey-selectable groups; the FPL and
  // Checklist groups are entered with their own keys on the real unit, so
  // turning the knob inside them steps back out to the MAP group.
  static constexpr MfdPageGroup kCycle[] = {
      MfdPageGroup::Map, MfdPageGroup::Waypoint, MfdPageGroup::Aux,
      MfdPageGroup::Nearest};
  constexpr int kCycleCount = 4;
  pageSelectSec_ = kPageSelectSeconds;
  menu_ = Menu::Root;
  for (int i = 0; i < kCycleCount; ++i) {
    if (kCycle[i] == pageGroup_) {
      pageGroup_ = kCycle[((i + direction) % kCycleCount + kCycleCount) %
                          kCycleCount];
      return;
    }
  }
  pageGroup_ = MfdPageGroup::Map;
}

void MfdController::clrDefaultMap() {
  // Cancel whatever is in progress, exactly like backing all the way out, then
  // bring up the MAP - NAVIGATION MAP page.
  simbriefPendingId_.clear();
  simbriefIdEntry_ = false;
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
  pageGroup_ = MfdPageGroup::Map;
  pageIndex_[static_cast<int>(MfdPageGroup::Map)] = 0;
  rebuildLabels();
}

bool MfdController::pressKey(int key) {
  if (key < 0 || key >= kSoftkeyCount || labels_[key].empty()) return false;

  press_[key] = 1.0f;  // trigger the press-flash animation

  if (simbriefIdEntry_) {
    simbriefEntryKey(key);
    rebuildLabels();
    return true;
  }

  if (menu_ == Menu::MapOpt) {
    switch (key) {
      case kKeyOptTraffic:
        showTraffic_ = !showTraffic_;
        break;
      case kKeyOptTer:
        // TER cycles Off -> Topo -> REL -> Off (Pilot's Guide).
        terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Topo
                   : terrain_ == TerrainDisplay::Topo ? TerrainDisplay::Rel
                                                       : TerrainDisplay::Off;
        break;
      case kKeyOptAwy:
        // AWY cycles Off -> On (all) -> LO -> HI -> Off (Pilot's Guide).
        airways_ = airways_ == AirwayDisplay::Off   ? AirwayDisplay::All
                   : airways_ == AirwayDisplay::All ? AirwayDisplay::Low
                   : airways_ == AirwayDisplay::Low ? AirwayDisplay::High
                                                    : AirwayDisplay::Off;
        break;
      case kKeyOptNexrad:
        showWeather_ = !showWeather_;
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
        // Horizontal scan: toggle the bearing line. Vertical scan: the cell is
        // Tilt; tilt is trimmed with the FMS knob (radarBezelKey).
        if (radarScan_ != RadarScan::Vertical) {
          radarBearingLineOn_ = !radarBearingLineOn_;
          if (!radarBearingLineOn_) radarBearingDeg_ = 0.0f;
        }
        break;
      case kKeyRdrFeatures:
        radarAct_ = !radarAct_;  // Altitude Compensated Tilt on/off
        break;
      case kKeyRangeDown:
        rangeIndex_ = std::max(0, rangeIndex_ - 1);
        break;
      case kKeyRangeUp:
        rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
        break;
      default:
        return false;
    }
    rebuildLabels();
    return true;
  }

  switch (key) {
    case kKeyMap:
      selectGroup(MfdPageGroup::Map);
      break;
    case kKeyWaypoint:
      selectGroup(MfdPageGroup::Waypoint);
      break;
    case kKeyAux:
      selectGroup(MfdPageGroup::Aux);
      break;
    case kKeyNearest:
      selectGroup(MfdPageGroup::Nearest);
      break;
    case kKeyChecklist:
      // Selecting the group enters it; pressing again steps to the next
      // checklist (selectGroup -> stepPage -> stepChecklist).
      selectGroup(MfdPageGroup::Checklist);
      break;
    case kKeyOrient:
      mapOrientation_ = (mapOrientation_ == MapOrientation::NorthUp)
                            ? MapOrientation::TrackUp
                            : MapOrientation::NorthUp;
      break;
    case kKeyMapOpt:
      menu_ = Menu::MapOpt;
      rebuildLabels();
      break;
    case kKeyDetail:
      detail_ = nextMapDetail(detail_);
      rebuildLabels();
      break;
    case kKeySimbriefId:
      // Only labeled on the AUX - SIMBRIEF page. A fresh entry starts empty
      // (dashes), like the XPDR Code entry.
      simbriefPendingId_.clear();
      simbriefIdEntry_ = true;
      break;
    case kKeySimbriefFetch:
      if (!simbriefPilotId_.empty() &&
          simbriefState_.status != SimBriefStatus::Fetching) {
        simbriefFetchRequested_ = true;
      }
      break;
    case kKeyRangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      break;
    case kKeyRangeUp:
      rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
      break;
    default:
      return false;
  }
  rebuildLabels();  // the page (and so the ID/FETCH keys) may have changed
  return true;
}

void MfdController::simbriefEntryKey(int key) {
  if (key >= 0 && key <= 9) {
    if (static_cast<int>(simbriefPendingId_.size()) <
        kSimBriefPilotIdMaxDigits) {
      simbriefPendingId_ += static_cast<char>('0' + key);
    }
  } else if (key == kKeyEntryBksp) {
    if (!simbriefPendingId_.empty()) simbriefPendingId_.pop_back();
  } else if (key == kKeyEntryBack) {
    // Abandon the in-progress entry; the committed ID is untouched.
    simbriefPendingId_.clear();
    simbriefIdEntry_ = false;
  }
}

bool MfdController::consumeSimbriefFetchRequest() {
  const bool requested = simbriefFetchRequested_;
  simbriefFetchRequested_ = false;
  return requested;
}

bool MfdController::blocksRadioBezel() const {
  // Map panning is driven by the RANGE joystick, not the FMS knob, so the
  // active Map Pointer does not claim the knob here.
  return dtoOpen_ || dtoEntry_.active || fplEntry_.active ||
         fplAltEntry_.active || fplConfirm_ != FplConfirm::None ||
         wptEntry_.active || simbriefIdEntry_ || procMenuOpen_ ||
         mapSettingsOpen_;
}

}  // namespace avionics
