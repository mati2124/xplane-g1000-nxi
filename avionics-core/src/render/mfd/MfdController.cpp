#include "avionics/MfdController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"
#include "render/mfd/MfdMapSettings.h"

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

// Declutter level as the Navigation Map Page Menu shows it: "Declutter
// (Current Detail All)" / "...3)" / "...2)" / "...1)" (Pilot's Guide Fig. 5-6).
const char* declutterLevelText(MapDetail d) {
  switch (d) {
    case MapDetail::Detail3:
      return "3";
    case MapDetail::Detail2:
      return "2";
    case MapDetail::Detail1:
      return "1";
    case MapDetail::All:
      break;
  }
  return "All";
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

bool legsEqual(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id || a[i].lat != b[i].lat || a[i].lon != b[i].lon) {
      return false;
    }
  }
  return true;
}

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
  setRange(MapSetting::NexradRange, 1000.0f);
  // Traffic group.
  setToggle(MapSetting::TrafficLabelsOn, true);
  setRange(MapSetting::TrafficSymbolsRange, 15.0f);
  setRange(MapSetting::TrafficLabelsRange, 15.0f);
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

int MfdController::checklistCount() const {
  return checklist_ != nullptr ? checklist_->totalChecklists() : 0;
}

int MfdController::checklistItemCount(int checklistIndex) const {
  if (checklist_ == nullptr) return 0;
  const Checklist* c = checklist_->at(checklistIndex);
  return c != nullptr ? static_cast<int>(c->items.size()) : 0;
}

bool MfdController::checklistItemChecked(int checklistIndex,
                                        int itemIndex) const {
  if (checklistIndex < 0 ||
      checklistIndex >= static_cast<int>(checked_.size())) {
    return false;
  }
  const std::vector<std::uint8_t>& items = checked_[checklistIndex];
  if (itemIndex < 0 || itemIndex >= static_cast<int>(items.size())) return false;
  return items[itemIndex] != 0;
}

void MfdController::stepChecklist(int direction) {
  const int count = checklistCount();
  if (count <= 0) {
    checklistIndex_ = 0;
    cursorItem_ = 0;
    return;
  }
  checklistIndex_ = ((checklistIndex_ + direction) % count + count) % count;
  cursorItem_ = 0;
}

void MfdController::syncChecklist(const ChecklistData& data) {
  checklist_ = &data;
  const int total = data.totalChecklists();

  // Steady-state fast path: when the file's shape is unchanged (the common
  // case, since this runs every frame) keep the existing checked flags and skip
  // the reallocation.
  bool sameShape = static_cast<int>(checked_.size()) == total;
  for (int ci = 0; sameShape && ci < total; ++ci) {
    if (static_cast<int>(checked_[ci].size()) != checklistItemCount(ci)) {
      sameShape = false;
    }
  }
  if (!sameShape) {
    // Rebuild to match the file, preserving flags whose checklist/item still
    // exists so a live reload does not wipe progress.
    std::vector<std::vector<std::uint8_t>> resized(
        static_cast<std::size_t>(total));
    for (int ci = 0; ci < total; ++ci) {
      const int items = checklistItemCount(ci);
      std::vector<std::uint8_t> row(static_cast<std::size_t>(items), 0);
      if (ci < static_cast<int>(checked_.size())) {
        const std::vector<std::uint8_t>& old = checked_[ci];
        const std::size_t n = std::min(row.size(), old.size());
        for (std::size_t i = 0; i < n; ++i) row[i] = old[i];
      }
      resized[static_cast<std::size_t>(ci)] = std::move(row);
    }
    checked_ = std::move(resized);
  }

  // Clamp the selection in case the file shrank.
  if (checklistIndex_ >= total) checklistIndex_ = total > 0 ? total - 1 : 0;
  const int itemCount = checklistItemCount(checklistIndex_);
  if (cursorItem_ > itemCount) cursorItem_ = itemCount;
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

// ---- Page menu (MENU bezel key) ----

std::vector<MfdController::PageMenuItem> MfdController::buildPageMenu() const {
  // Only the Navigation Map page defines a Page Menu in this suite for now
  // (Pilot's Guide Fig. 5-6). Other pages return an empty list, so MENU is
  // inert there, matching the real unit's pages that have no page menu.
  if (pageGroup_ != MfdPageGroup::Map || page() != MfdPage::NavigationMap) {
    return {};
  }
  // Verbatim from the figure, in on-unit order. Map Settings opens the Map
  // Settings window (Fig. 5-7); Declutter cycles the map Detail level. Measure
  // Bearing/Distance and Show VSD need tools this suite has not yet modeled,
  // so they are listed disabled; Charts is greyed on the real unit too.
  std::string declutter = "Declutter (Current Detail ";
  declutter += declutterLevelText(detail_);
  declutter += ")";
  return {
      {"Map Settings", PageMenuAction::OpenMapSettings},
      {declutter, PageMenuAction::MapDeclutter},
      {"Measure Bearing/Distance", PageMenuAction::Disabled},
      {"Charts", PageMenuAction::Disabled},
      {"Show VSD", PageMenuAction::Disabled},
  };
}

void MfdController::openPageMenu() {
  pageMenuItems_ = buildPageMenu();
  if (pageMenuItems_.empty()) return;  // no page menu on this page
  pageMenuOpen_ = true;
  // Highlight the first enabled option (the cursor never parks on a disabled
  // row, which the real unit skips).
  pageMenuSel_ = 0;
  for (int i = 0; i < static_cast<int>(pageMenuItems_.size()); ++i) {
    if (pageMenuItems_[i].action != PageMenuAction::Disabled) {
      pageMenuSel_ = i;
      break;
    }
  }
}

const std::string& MfdController::pageMenuItemText(int i) const {
  static const std::string kEmpty;
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return kEmpty;
  return pageMenuItems_[static_cast<std::size_t>(i)].text;
}

bool MfdController::pageMenuItemEnabled(int i) const {
  if (i < 0 || i >= static_cast<int>(pageMenuItems_.size())) return false;
  return pageMenuItems_[static_cast<std::size_t>(i)].action !=
         PageMenuAction::Disabled;
}

void MfdController::pageMenuStep(int direction) {
  const int n = static_cast<int>(pageMenuItems_.size());
  if (n == 0) return;
  const int step = direction >= 0 ? 1 : -1;
  // Walk in the requested direction to the next enabled option, wrapping.
  for (int i = 0; i < n; ++i) {
    pageMenuSel_ = (pageMenuSel_ + step + n) % n;
    if (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action !=
        PageMenuAction::Disabled) {
      return;
    }
  }
}

void MfdController::pageMenuActivate() {
  if (pageMenuSel_ < 0 ||
      pageMenuSel_ >= static_cast<int>(pageMenuItems_.size())) {
    return;
  }
  switch (pageMenuItems_[static_cast<std::size_t>(pageMenuSel_)].action) {
    case PageMenuAction::MapDeclutter:
      detail_ = nextMapDetail(detail_);
      pageMenuOpen_ = false;  // the option runs and closes the menu
      break;
    case PageMenuAction::OpenMapSettings:
      pageMenuOpen_ = false;  // the page menu closes as the window opens
      openMapSettings();
      break;
    case PageMenuAction::Disabled:
      break;  // inert: a disabled row is never highlighted, so this is a no-op
  }
}

bool MfdController::pageMenuBezelKey(BezelKey key) {
  switch (key) {
    case BezelKey::Ent:
      pageMenuActivate();
      break;
    case BezelKey::Clr:
    case BezelKey::Menu:
    case BezelKey::FmsPush:
      pageMenuOpen_ = false;  // back out to the base page
      break;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsOuterCw:
      pageMenuStep(1);
      break;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsOuterCcw:
      pageMenuStep(-1);
      break;
    default:
      break;
  }
  return true;
}

// ---- Map Settings window ----

void MfdController::openMapSettings() {
  mapSettingsOpen_ = true;
  // The window opens with the cursor on the Group selector (Pilot's Guide:
  // "Map Group Selection"), keeping the last-viewed group.
  mapSettingsCursor_ = 0;
}

int MfdController::mapSettingsFieldCount() const {
  MapSetting fields[static_cast<std::size_t>(MapSetting::Count)];
  return mfd::msEditableFields(
      mapSettingsGroup_, fields,
      static_cast<int>(MapSetting::Count));
}

MapSetting MfdController::mapSettingAtCursor(int cursor) const {
  if (cursor <= 0) return MapSetting::Count;
  MapSetting fields[static_cast<std::size_t>(MapSetting::Count)];
  const int n = mfd::msEditableFields(mapSettingsGroup_, fields,
                                      static_cast<int>(MapSetting::Count));
  if (cursor - 1 >= n) return MapSetting::Count;
  return fields[cursor - 1];
}

void MfdController::mapSettingsStepCursor(int dir) {
  const int total = 1 + mapSettingsFieldCount();  // Group selector + controls
  const int step = dir >= 0 ? 1 : -1;
  mapSettingsCursor_ = ((mapSettingsCursor_ + step) % total + total) % total;
}

void MfdController::mapSettingsEdit(int dir) {
  // The Group selector: cycle the active group (Pilot's Guide: small FMS knob
  // selects the group), and keep the cursor on the selector.
  if (mapSettingsCursor_ == 0) {
    constexpr int kGroupCount =
        static_cast<int>(MapSettingsGroup::Land) + 1;
    const int step = dir >= 0 ? 1 : -1;
    int g = (static_cast<int>(mapSettingsGroup_) + step) % kGroupCount;
    if (g < 0) g += kGroupCount;
    mapSettingsGroup_ = static_cast<MapSettingsGroup>(g);
    return;
  }

  const MapSetting id = mapSettingAtCursor(mapSettingsCursor_);
  if (id == MapSetting::Count) return;

  // Shared enums step the existing members, so the window and softkeys agree.
  if (id == MapSetting::Orientation) {
    static constexpr MapOrientation kCycle[] = {
        MapOrientation::NorthUp, MapOrientation::TrackUp,
        MapOrientation::HeadingUp};
    const int step = dir >= 0 ? 1 : -1;
    for (int i = 0; i < 3; ++i) {
      if (kCycle[i] == mapOrientation_) {
        mapOrientation_ = kCycle[((i + step) % 3 + 3) % 3];
        return;
      }
    }
    mapOrientation_ = MapOrientation::NorthUp;
    return;
  }
  if (id == MapSetting::TerrainMode) {
    // Off -> Topo -> REL, matching the TER softkey cycle.
    if (dir >= 0) {
      terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Topo
                 : terrain_ == TerrainDisplay::Topo ? TerrainDisplay::Rel
                                                    : TerrainDisplay::Off;
    } else {
      terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Rel
                 : terrain_ == TerrainDisplay::Rel ? TerrainDisplay::Topo
                                                   : TerrainDisplay::Off;
    }
    return;
  }
  if (id == MapSetting::TrafficMode) {
    const int step = dir >= 0 ? 1 : -1;
    msTrafficMode_ = ((msTrafficMode_ + step) % 3 + 3) % 3;
    return;
  }

  switch (mfd::msControlKind(id)) {
    case mfd::MsKind::Toggle:
      if (id == MapSetting::NexradOn) {
        showWeather_ = !showWeather_;
      } else if (id == MapSetting::TrafficOn) {
        showTraffic_ = !showTraffic_;
      } else {
        bool& v = msToggle_[static_cast<std::size_t>(id)];
        v = !v;
      }
      break;
    case mfd::MsKind::Range: {
      int& idx = msRange_[static_cast<std::size_t>(id)];
      idx = std::max(0, std::min(kMapRangeLadderCount - 1,
                                 idx + (dir >= 0 ? 1 : -1)));
      break;
    }
    default:
      break;
  }
}

bool MfdController::mapSettingsBezelKey(BezelKey key) {
  switch (key) {
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      // Pilot's Guide: "Press FMS Knob To Return"; CLR also backs out.
      mapSettingsOpen_ = false;
      break;
    case BezelKey::FmsOuterCw:
      mapSettingsStepCursor(1);
      break;
    case BezelKey::FmsOuterCcw:
      mapSettingsStepCursor(-1);
      break;
    case BezelKey::FmsInnerCw:
    case BezelKey::Ent:
      mapSettingsEdit(1);
      break;
    case BezelKey::FmsInnerCcw:
      mapSettingsEdit(-1);
      break;
    default:
      break;
  }
  return true;
}

std::string MfdController::mapSettingText(MapSetting id) const {
  switch (id) {
    case MapSetting::Orientation:
      switch (mapOrientation_) {
        case MapOrientation::TrackUp:
          return "Track up";
        case MapOrientation::HeadingUp:
          return "HDG up";
        case MapOrientation::NorthUp:
          break;
      }
      return "North up";
    case MapSetting::TerrainMode:
      switch (terrain_) {
        case TerrainDisplay::Topo:
          return "Topo";
        case TerrainDisplay::Rel:
          return "REL";
        case TerrainDisplay::Off:
          break;
      }
      return "Off";
    case MapSetting::TrafficMode:
      return msTrafficMode_ == 1   ? "TA/PA"
             : msTrafficMode_ == 2 ? "TA Only"
                                   : "All Traffic";
    // Dependent read-outs shown without carets on the real unit (Fig. 5-7).
    case MapSetting::AutoZoomMax:
      return "All";
    case MapSetting::MaxLookFwd:
      return "30min";
    case MapSetting::MinLookFwd:
      return "5min";
    case MapSetting::TimeOut:
      return "0min";
    case MapSetting::TrackVectorTime:
      return "60 sec";
    case MapSetting::FuelRangeRsv:
      return "0+45";
    default:
      break;
  }
  switch (mfd::msControlKind(id)) {
    case mfd::MsKind::Toggle:
      return mapSettingOn(id) ? "On" : "Off";
    case mfd::MsKind::Range: {
      char buf[16];
      formatMapRange(buf, sizeof(buf),
                     mapRangeNmAt(msRange_[static_cast<std::size_t>(id)]));
      return buf;
    }
    default:
      break;
  }
  return "";
}

bool MfdController::mapSettingOn(MapSetting id) const {
  switch (id) {
    case MapSetting::NexradOn:
      return showWeather_;
    case MapSetting::TrafficOn:
      return showTraffic_;
    default:
      break;
  }
  return msToggle_[static_cast<std::size_t>(id)];
}

float MfdController::mapSettingRangeNm(MapSetting id) const {
  return mapRangeNmAt(msRange_[static_cast<std::size_t>(id)]);
}

void MfdController::checklistEnter() {
  const int itemCount = checklistItemCount(checklistIndex_);
  // On the "go to next checklist?" prompt below the last item: advance.
  if (cursorItem_ >= itemCount) {
    stepChecklist(1);
    return;
  }
  if (checklistIndex_ < static_cast<int>(checked_.size()) &&
      cursorItem_ < static_cast<int>(checked_[checklistIndex_].size())) {
    checked_[checklistIndex_][cursorItem_] = 1;
  }
  cursorItem_ = std::min(itemCount, cursorItem_ + 1);  // auto-advance
}

void MfdController::checklistClear() {
  const int itemCount = checklistItemCount(checklistIndex_);
  if (cursorItem_ < itemCount &&
      checklistIndex_ < static_cast<int>(checked_.size()) &&
      cursorItem_ < static_cast<int>(checked_[checklistIndex_].size())) {
    checked_[checklistIndex_][cursorItem_] = 0;
  }
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

std::vector<MapApproach> MfdController::approachesForAirport(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->approachesForAirport(icao);
}

std::vector<MapProcedure> MfdController::proceduresForAirport(
    const std::string& icao, ProcedureType type) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->proceduresForAirport(icao, type);
}

std::vector<MapProcedure> MfdController::proceduresFor(ProcedureType type) const {
  return proceduresForAirport(procAirportIcao(), type);
}

std::string MfdController::procAirportIcao() const {
  if (procCategory_ == ProcedureType::Departure && !fplLegs_.empty() &&
      fplLegs_.front().id.size() == 4) {
    return fplLegs_.front().id;
  }
  for (int i = static_cast<int>(fplLegs_.size()) - 1; i >= 0; --i) {
    if (fplLegs_[static_cast<std::size_t>(i)].id.size() == 4) {
      return fplLegs_[static_cast<std::size_t>(i)].id;
    }
  }
  return activeWaypoint_;
}

bool MfdController::consumeProcLoadRequest(MapProcedure& out) {
  if (!procLoadPending_) return false;
  procLoadPending_ = false;
  out = procLoadTarget_;
  return true;
}

std::vector<MapAirportFrequency> MfdController::airportFrequencies(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  return navSource_->airportFrequencies(icao);
}

std::vector<AirportRunwayInfo> MfdController::airportRunways(
    const std::string& icao) const {
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) return {};
  std::vector<AirportRunwayInfo> runways = navSource_->airportRunways(icao);
  // Primary (longest) runway first, the order the unit pages through them.
  std::stable_sort(runways.begin(), runways.end(),
                   [](const AirportRunwayInfo& a, const AirportRunwayInfo& b) {
                     return a.lengthFt > b.lengthFt;
                   });
  return runways;
}

std::vector<std::string> MfdController::procProcedureNames(
    ProcedureType type) const {
  std::vector<std::string> names;
  for (const MapProcedure& proc : proceduresFor(type)) {
    if (std::find(names.begin(), names.end(), proc.name) == names.end()) {
      names.push_back(proc.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> MfdController::procTransitions(
    ProcedureType type, const std::string& name) const {
  std::vector<std::string> transitions;
  for (const MapProcedure& proc : proceduresFor(type)) {
    if (proc.name != name) continue;
    if (std::find(transitions.begin(), transitions.end(), proc.transition) ==
        transitions.end()) {
      transitions.push_back(proc.transition);
    }
  }
  std::sort(transitions.begin(), transitions.end());
  return transitions;
}

std::vector<MapLeg> MfdController::procPreviewLegs() const {
  if (!procMenuOpen_ || navSource_ == nullptr || !navSource_->ready()) {
    return {};
  }
  const std::string icao = procAirportIcao();
  if (icao.empty()) return {};

  std::string name;
  std::string transition;
  if (procStep_ == ProcMenuStep::ProcedureList) {
    const std::vector<std::string> names = procProcedureNames(procCategory_);
    if (procSelected_ < 0 ||
        procSelected_ >= static_cast<int>(names.size())) {
      return {};
    }
    name = names[static_cast<std::size_t>(procSelected_)];
    const std::vector<std::string> transitions =
        procTransitions(procCategory_, name);
    if (transitions.empty()) return {};
    transition = transitions.front();
  } else {
    name = procSelectedName_;
    const std::vector<std::string> transitions =
        procTransitions(procCategory_, name);
    if (procSelected_ < 0 ||
        procSelected_ >= static_cast<int>(transitions.size())) {
      return {};
    }
    transition = transitions[static_cast<std::size_t>(procSelected_)];
  }
  return navSource_->expandProcedure(icao, procCategory_, name, transition);
}

// ---- Active Flight Plan page ----

void MfdController::syncFlightPlan(const MapData& map,
                                   const std::string& activeWaypoint) {
  mapData_ = &map;
  activeWaypoint_ = activeWaypoint;

  if (!legsEqual(map.flightPlan, fplLastMapPlan_)) {
    fplLastMapPlan_ = map.flightPlan;
    // Adopt the change unless it is just the data source catching up with our
    // own pending/published edit.
    if (!fplEditPending_ && !legsEqual(map.flightPlan, fplLastPublished_)) {
      fplLegs_ = map.flightPlan;
      // The rows the interaction state referenced are gone; close the entry
      // and confirmation windows rather than acting on the wrong waypoint.
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplAltEntry_.active = false;
      fplConfirm_ = FplConfirm::None;
      fplMenuOpen_ = false;
    }
  }

  fplCursorRow_ =
      std::max(0, std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_));
  if (fplCursorRow_ >= static_cast<int>(fplLegs_.size())) {
    fplCursorCol_ = FplCursorCol::Ident;  // the append slot has no ALT field
  }
}

bool MfdController::consumeFlightPlanEdit(std::vector<MapLeg>& out) {
  if (!fplEditPending_) return false;
  fplEditPending_ = false;
  out = fplLegs_;
  fplLastPublished_ = fplLegs_;
  return true;
}

void MfdController::fplPublishEdit() { fplEditPending_ = true; }

void MfdController::fplResetInteraction() {
  fplCursorOn_ = false;
  fplCursorRow_ = std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_);
  fplCursorCol_ = FplCursorCol::Ident;
  fplEntry_.reset();
  fplAltEntry_ = FplAltEntry{};
  fplConfirm_ = FplConfirm::None;
  fplMenuOpen_ = false;
  procMenuOpen_ = false;
  procStep_ = ProcMenuStep::ProcedureList;
  procSelectedName_.clear();
}

namespace {

void insertProcedureLegs(ProcedureType type, std::vector<MapLeg>& fplLegs,
                         const std::vector<MapLeg>& legs) {
  if (legs.empty()) return;
  int row = 0;
  if (type == ProcedureType::Departure) {
    row = fplLegs.empty() ? 0 : 1;
  } else if (type == ProcedureType::Arrival) {
    row = std::max(0, static_cast<int>(fplLegs.size()) - 1);
  } else {
    row = static_cast<int>(fplLegs.size());
  }
  fplLegs.insert(fplLegs.begin() + row, legs.begin(), legs.end());
}

MapProcedure findProcedure(ProcedureType type, const std::string& name,
                         const std::string& transition,
                         const std::vector<MapProcedure>& catalog) {
  for (const MapProcedure& proc : catalog) {
    if (proc.type == type && proc.name == name &&
        proc.transition == transition) {
      return proc;
    }
  }
  MapProcedure fallback;
  fallback.type = type;
  fallback.name = name;
  fallback.transition = transition;
  return fallback;
}

}  // namespace

bool MfdController::procBezelKey(BezelKey key) {
  const std::vector<std::string> names = procProcedureNames(procCategory_);
  const std::vector<std::string> transitions =
      procStep_ == ProcMenuStep::TransitionList
          ? procTransitions(procCategory_, procSelectedName_)
          : std::vector<std::string>{};

  auto loadProcedure = [&](const std::string& name,
                           const std::string& transition) {
    if (navSource_ == nullptr || !navSource_->ready()) return;
    const std::string icao = procAirportIcao();
    std::vector<MapLeg> legs =
        navSource_->expandProcedure(icao, procCategory_, name, transition);
    if (legs.empty()) return;
    insertProcedureLegs(procCategory_, fplLegs_, legs);
    fplCursorRow_ = static_cast<int>(fplLegs_.size());
    fplPublishEdit();
    procLoadTarget_ = findProcedure(procCategory_, name, transition,
                                    proceduresFor(procCategory_));
    procLoadPending_ = true;
    procMenuOpen_ = false;
    procStep_ = ProcMenuStep::ProcedureList;
    procSelectedName_.clear();
  };

  switch (key) {
    case BezelKey::Ent:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        if (procSelected_ >= 0 &&
            procSelected_ < static_cast<int>(names.size())) {
          const std::string& name =
              names[static_cast<std::size_t>(procSelected_)];
          const std::vector<std::string> trans =
              procTransitions(procCategory_, name);
          if (trans.size() == 1) {
            loadProcedure(name, trans.front());
          } else if (trans.size() > 1) {
            procSelectedName_ = name;
            procStep_ = ProcMenuStep::TransitionList;
            procSelected_ = 0;
          }
        }
      } else if (procSelected_ >= 0 &&
                 procSelected_ < static_cast<int>(transitions.size())) {
        loadProcedure(procSelectedName_,
                      transitions[static_cast<std::size_t>(procSelected_)]);
      }
      break;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      if (procStep_ == ProcMenuStep::TransitionList) {
        procStep_ = ProcMenuStep::ProcedureList;
        procSelectedName_.clear();
        procSelected_ = 0;
      } else {
        procMenuOpen_ = false;
      }
      break;
    case BezelKey::FmsInnerCw:
      if (procStep_ == ProcMenuStep::ProcedureList && !names.empty()) {
        procSelected_ = (procSelected_ + 1) % static_cast<int>(names.size());
      } else if (procStep_ == ProcMenuStep::TransitionList &&
                 !transitions.empty()) {
        procSelected_ =
            (procSelected_ + 1) % static_cast<int>(transitions.size());
      }
      break;
    case BezelKey::FmsInnerCcw:
      if (procStep_ == ProcMenuStep::ProcedureList && !names.empty()) {
        procSelected_ = (procSelected_ - 1 + static_cast<int>(names.size())) %
                        static_cast<int>(names.size());
      } else if (procStep_ == ProcMenuStep::TransitionList &&
                 !transitions.empty()) {
        procSelected_ = (procSelected_ - 1 +
                         static_cast<int>(transitions.size())) %
                        static_cast<int>(transitions.size());
      }
      break;
    case BezelKey::FmsOuterCw:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        procCategory_ = static_cast<ProcedureType>(
            (static_cast<int>(procCategory_) + 1) % 3);
        procSelected_ = 0;
        procSelectedName_.clear();
      }
      break;
    case BezelKey::FmsOuterCcw:
      if (procStep_ == ProcMenuStep::ProcedureList) {
        procCategory_ = static_cast<ProcedureType>(
            (static_cast<int>(procCategory_) + 2) % 3);
        procSelected_ = 0;
        procSelectedName_.clear();
      }
      break;
    default:
      break;
  }
  return true;
}

void MfdController::fplAltEntryOpen(int row) {
  if (row < 0 || row >= static_cast<int>(fplLegs_.size())) return;
  fplAltEntry_.active = true;
  fplAltEntry_.row = row;
  fplAltEntry_.pos = 0;
  // Seed the five digit cells with the existing constraint (right-aligned), or
  // zeros for a fresh entry.
  const int ft = fplLegs_[static_cast<std::size_t>(row)].altitudeConstraintFt;
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%05d", std::max(0, std::min(99999, ft)));
  fplAltEntry_.digits.assign(buf, 5);
}

void MfdController::fplAltEntryCommit() {
  const int row = fplAltEntry_.row;
  fplAltEntry_.active = false;
  if (row < 0 || row >= static_cast<int>(fplLegs_.size())) return;
  const int ft = std::atoi(fplAltEntry_.digits.c_str());
  MapLeg& leg = fplLegs_[static_cast<std::size_t>(row)];
  if (ft > 0) {
    leg.altitudeConstraintFt = ft;
    leg.altitudeConstraint = AltConstraintType::At;
    leg.altitudeDesignated = true;  // manually entered -> drawn cyan
  } else {
    leg.altitudeConstraintFt = 0;
    leg.altitudeConstraint = AltConstraintType::None;
    leg.altitudeDesignated = false;
  }
  fplPublishEdit();
}

void MfdController::fplCommitEntry() {
  if (fplEntry_.chars.empty()) {
    // Nothing spelled: close the window, like backing out.
    fplEntry_.active = false;
    return;
  }
  if (!fplEntry_.hasMatch) {
    fplEntry_.notFound = true;  // stay open so the ident can be corrected
    return;
  }

  // Insert before the selected row (Pilot's Guide: "The new waypoint is
  // placed directly in front of the highlighted waypoint"); the blank slot
  // after the last waypoint appends.
  const int row =
      std::max(0, std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_));

  const std::string ident = fplEntry_.autofill.empty() ? fplEntry_.chars
                                                         : fplEntry_.autofill;
  if (navSource_ != nullptr && navSource_->isAirwayName(ident) && row > 0 &&
      row < static_cast<int>(fplLegs_.size())) {
    const std::vector<MapLeg> expanded = navSource_->expandAirway(
        ident, fplLegs_[static_cast<std::size_t>(row - 1)].id,
        fplLegs_[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      fplLegs_.insert(fplLegs_.begin() + row, expanded.begin(), expanded.end());
      fplCursorRow_ = row + static_cast<int>(expanded.size());
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplPublishEdit();
      return;
    }
  }

  MapLeg leg;
  leg.lat = fplEntry_.match.lat;
  leg.lon = fplEntry_.match.lon;
  leg.id = fplEntry_.match.id;
  fplLegs_.insert(fplLegs_.begin() + row, leg);
  fplCursorRow_ = row + 1;  // follow the insertion, ready for the next entry
  fplEntry_.active = false;
  fplEntry_.notFound = false;
  fplPublishEdit();
}

bool MfdController::fplBezelKey(BezelKey key) {
  // The confirmation window is modal: ENT executes the highlighted choice,
  // CLR (or pushing the FMS knob) cancels, the knob toggles OK/CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          if (fplConfirm_ == FplConfirm::RemoveWaypoint) {
            if (fplCursorRow_ < static_cast<int>(fplLegs_.size())) {
              fplLegs_.erase(fplLegs_.begin() + fplCursorRow_);
              fplPublishEdit();
            }
          } else {  // DeleteFlightPlan
            fplLegs_.clear();
            fplCursorRow_ = 0;
            fplPublishEdit();
          }
        }
        fplConfirm_ = FplConfirm::None;
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        fplConfirm_ = FplConfirm::None;
        break;
      case BezelKey::FmsOuterCw:
      case BezelKey::FmsOuterCcw:
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsInnerCcw:
        fplConfirmOk_ = !fplConfirmOk_;
        break;
      default:
        break;
    }
    return true;
  }

  // Page menu: a single option (Delete Flight Plan), ENT selects it.
  if (fplMenuOpen_) {
    switch (key) {
      case BezelKey::Ent:
        fplMenuOpen_ = false;
        fplConfirm_ = FplConfirm::DeleteFlightPlan;
        fplConfirmOk_ = true;
        break;
      case BezelKey::Clr:
      case BezelKey::Menu:
      case BezelKey::FmsPush:
        fplMenuOpen_ = false;
        break;
      default:
        break;
    }
    return true;
  }

  // Waypoint Information entry window: small knob spells, large knob moves
  // the character cursor, ENT accepts, CLR / knob push cancels (the field
  // reverts, Pilot's Guide data-entry procedure).
  if (fplEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        fplCommitEntry();
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        fplEntry_.active = false;
        fplEntry_.notFound = false;
        break;
      case BezelKey::FmsInnerCw:
        fplEntry_.turnChar(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsInnerCcw:
        fplEntry_.turnChar(navSource_, mapData_, -1);
        break;
      case BezelKey::FmsOuterCw:
        fplEntry_.moveCursor(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsOuterCcw:
        fplEntry_.moveCursor(navSource_, mapData_, -1);
        break;
      default:
        break;
    }
    return true;
  }

  // VNAV altitude-constraint entry window: small knob spins the digit under the
  // cursor, large knob moves the cursor, ENT commits (0 clears the constraint),
  // CLR / knob push cancels.
  if (fplAltEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        fplAltEntryCommit();
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        fplAltEntry_.active = false;
        break;
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsInnerCcw: {
        const int step = key == BezelKey::FmsInnerCw ? +1 : -1;
        char& c = fplAltEntry_.digits[static_cast<std::size_t>(fplAltEntry_.pos)];
        c = static_cast<char>('0' + ((c - '0' + step + 10) % 10));
        break;
      }
      case BezelKey::FmsOuterCw:
        fplAltEntry_.pos = std::min(4, fplAltEntry_.pos + 1);
        break;
      case BezelKey::FmsOuterCcw:
        fplAltEntry_.pos = std::max(0, fplAltEntry_.pos - 1);
        break;
      default:
        break;
    }
    return true;
  }

  // MENU opens the page menu whether or not the cursor is on.
  if (key == BezelKey::Menu) {
    fplMenuOpen_ = true;
    return true;
  }

  // Pushing the knob turns the selection cursor on/off.
  if (key == BezelKey::FmsPush) {
    fplCursorOn_ = !fplCursorOn_;
    fplCursorRow_ =
        std::max(0, std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_));
    fplCursorCol_ = FplCursorCol::Ident;
    return true;
  }

  if (!fplCursorOn_) return false;  // knob turns step pages as usual

  const int legCount = static_cast<int>(fplLegs_.size());
  // The blank append slot (row == legCount) has no ALT field.
  const bool onWaypointRow = fplCursorRow_ < legCount;
  const bool onAltCol =
      onWaypointRow && fplCursorCol_ == FplCursorCol::Altitude;

  switch (key) {
    case BezelKey::FmsOuterCw:
      // Step through fields: a waypoint row's IDENT then its ALT, then the next
      // row's IDENT (Pilot's Guide: the large knob moves the field highlight).
      if (onWaypointRow && fplCursorCol_ == FplCursorCol::Ident) {
        fplCursorCol_ = FplCursorCol::Altitude;
      } else {
        fplCursorRow_ = std::min(legCount, fplCursorRow_ + 1);
        fplCursorCol_ = FplCursorCol::Ident;
      }
      return true;
    case BezelKey::FmsOuterCcw:
      if (onAltCol) {
        fplCursorCol_ = FplCursorCol::Ident;
      } else if (fplCursorRow_ > 0) {
        fplCursorRow_ -= 1;
        // Land on the previous waypoint row's ALT field when it has one.
        fplCursorCol_ = fplCursorRow_ < legCount ? FplCursorCol::Altitude
                                                 : FplCursorCol::Ident;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      if (onAltCol) {
        // Small knob on the ALT column opens the altitude-constraint entry.
        fplAltEntryOpen(fplCursorRow_);
      } else {
        // Small knob on the IDENT column opens the Waypoint Information window
        // for an insertion before that row.
        fplEntry_.open(navSource_, mapData_);
      }
      return true;
    case BezelKey::Clr:
      if (onAltCol) {
        // CLR on the ALT column removes an existing constraint.
        MapLeg& leg = fplLegs_[static_cast<std::size_t>(fplCursorRow_)];
        if (leg.altitudeConstraint != AltConstraintType::None) {
          leg.altitudeConstraintFt = 0;
          leg.altitudeConstraint = AltConstraintType::None;
          leg.altitudeDesignated = false;
          fplPublishEdit();
        }
      } else if (onWaypointRow) {
        // CLR on a waypoint row asks "Remove <wpt>?"; the blank append slot has
        // nothing to remove.
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[fplCursorRow_].id;
      }
      return true;
    case BezelKey::Ent:
      return true;  // no function on a bare row, but the cursor owns the key
    default:
      return false;
  }
}

// ---- Direct-To ----

void MfdController::directToOpen() {
  dtoOpen_ = true;
  dtoArmed_ = false;
  // Map Pointer: Direct-To opens on the waypoint under the pointer (Pilot's
  // Guide, Map Panning).
  if (mapPointerActive_) {
    const MapFeature* sel = mapPointerFeature();
    if (sel != nullptr) {
      dtoEntry_.open(navSource_, mapData_, sel->id);
      dtoEntry_.match = *sel;
      dtoEntry_.hasMatch = true;
      dtoEntry_.autofill = sel->id;
      mapResetPointer();
      return;
    }
  }
  // Default destination (Pilot's Guide: the field defaults to the active
  // waypoint, or the highlighted flight-plan waypoint when one is selected).
  std::string initial;
  if (fplCursorOn_ && fplCursorRow_ < static_cast<int>(fplLegs_.size())) {
    initial = fplLegs_[fplCursorRow_].id;
  } else if (!activeWaypoint_.empty()) {
    initial = activeWaypoint_;
  }
  dtoEntry_.open(navSource_, mapData_, initial);
}

bool MfdController::directToBezelKey(BezelKey key) {
  if (!dtoOpen_) {
    if (key != BezelKey::DirectTo) return false;
    directToOpen();
    return true;
  }

  // Pressing Direct-To again, CLR, or pushing the knob closes the window.
  if (key == BezelKey::Clr || key == BezelKey::FmsPush ||
      key == BezelKey::DirectTo) {
    dtoOpen_ = false;
    dtoArmed_ = false;
    dtoEntry_.reset();
    return true;
  }

  // Armed: the ACTIVATE? prompt is highlighted; ENT engages the direct course.
  if (dtoArmed_) {
    if (key == BezelKey::Ent) {
      dtoRequestTarget_.lat = dtoEntry_.match.lat;
      dtoRequestTarget_.lon = dtoEntry_.match.lon;
      dtoRequestTarget_.id = dtoEntry_.match.id;
      dtoRequestPending_ = true;
      dtoOpen_ = false;
      dtoArmed_ = false;
      dtoEntry_.reset();
    }
    return true;
  }

  // Entering the destination identifier.
  switch (key) {
    case BezelKey::Ent:
      // First ENT confirms the waypoint and arms ACTIVATE? (an unknown ident
      // keeps the window open so it can be corrected).
      if (dtoEntry_.chars.empty()) {
        break;
      } else if (dtoEntry_.hasMatch) {
        dtoEntry_.active = false;
        dtoArmed_ = true;
      } else {
        dtoEntry_.notFound = true;
      }
      break;
    case BezelKey::FmsInnerCw:
      dtoEntry_.turnChar(navSource_, mapData_, +1);
      break;
    case BezelKey::FmsInnerCcw:
      dtoEntry_.turnChar(navSource_, mapData_, -1);
      break;
    case BezelKey::FmsOuterCw:
      dtoEntry_.moveCursor(navSource_, mapData_, +1);
      break;
    case BezelKey::FmsOuterCcw:
      dtoEntry_.moveCursor(navSource_, mapData_, -1);
      break;
    default:
      break;
  }
  return true;
}

bool MfdController::consumeDirectToRequest(MapLeg& out) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  return true;
}

// ---- MAP pointer / pan ----

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kNmPerDeg = 60.0;

// Each FMS-knob click pans this fraction of the current map range, so panning
// covers the same on-screen distance per click at every zoom (matching the
// real unit's joystick feel, where one nudge moves a fixed part of the view
// rather than a fixed ground distance).
constexpr double kPanStepFrac = 0.10;
// Smallest pan step (NM), so the tightest range still pans noticeably.
constexpr double kPanStepMinNm = 0.05;

// How close (as a fraction of the current range) the pointer must be to a map
// feature for it to be "selected" -- highlighted and shown in the Map Pointer
// information box. Keeps the snap radius a few pixels of the crosshair at any
// zoom.
constexpr double kMapPointerSnapFrac = 0.04;
constexpr double kMapPointerSnapMinNm = 0.1;

void offsetNm(double lat, double lon, double bearingDeg, double distNm,
              double& outLat, double& outLon) {
  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double brg = bearingDeg * kDegToRad;
  outLat = lat + (distNm * std::cos(brg)) / kNmPerDeg;
  outLon = lon + (distNm * std::sin(brg)) / kNmPerDeg / cosLat;
}

// WPT group page index for a map feature (Airport / Intersection / NDB / VOR).
int wptPageIndexFor(MapFeatureType type) {
  switch (type) {
    case MapFeatureType::Airport:
      return 0;
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      return 1;
    case MapFeatureType::Ndb:
      return 2;
    case MapFeatureType::Vor:
      return 3;
  }
  return 0;
}

}  // namespace

void MfdController::mapResetPointer() { mapPointerActive_ = false; }

const MapFeature* MfdController::mapPointerFeature() const {
  if (!mapPointerActive_ || mapData_ == nullptr) return nullptr;
  const double snapNm =
      std::max(kMapPointerSnapMinNm, rangeNm() * kMapPointerSnapFrac);
  const MapFeature* best = nullptr;
  double bestNm = snapNm;
  for (const MapFeature& f : mapData_->features) {
    const double dNm =
        navDistanceNm(mapPointerLat_, mapPointerLon_, f.lat, f.lon);
    if (dNm < bestNm) {
      bestNm = dNm;
      best = &f;
    }
  }
  return best;
}

bool MfdController::mapBezelKey(BezelKey key) {
  if (page() != MfdPage::NavigationMap) return false;

  // The RANGE joystick drives panning (Pilot's Guide: push the Joystick to
  // bring up the Map Pointer, move it to pan). The FMS knob is not involved.
  if (key == BezelKey::PanPush) {
    if (!mapPointerActive_ && mapData_ != nullptr && mapData_->positionValid) {
      mapPointerLat_ = mapData_->ownshipLat;
      mapPointerLon_ = mapData_->ownshipLon;
    }
    mapPointerActive_ = !mapPointerActive_;
    return true;
  }

  if (!mapPointerActive_) return false;

  // ENT on a highlighted waypoint opens its Waypoint Information page (Pilot's
  // Guide, Map Panning).
  if (key == BezelKey::Ent) {
    const MapFeature* sel = mapPointerFeature();
    if (sel != nullptr) {
      wptEntry_.reset();
      wptFeature_ = *sel;
      wptHasSelection_ = true;
      pageIndex_[static_cast<int>(MfdPageGroup::Waypoint)] =
          wptPageIndexFor(sel->type);
      mapResetPointer();
      pageGroup_ = MfdPageGroup::Waypoint;
      pageSelectSec_ = kPageSelectSeconds;
    }
    return true;
  }

  const double stepNm = std::max(kPanStepMinNm, rangeNm() * kPanStepFrac);
  double bearing = 0.0;
  switch (key) {
    case BezelKey::PanRight:
      bearing = 90.0;
      break;
    case BezelKey::PanLeft:
      bearing = 270.0;
      break;
    case BezelKey::PanUp:
      bearing = 0.0;
      break;
    case BezelKey::PanDown:
      bearing = 180.0;
      break;
  default:
    return false;
  }
  offsetNm(mapPointerLat_, mapPointerLon_, bearing, stepNm,
           mapPointerLat_, mapPointerLon_);
  return true;
}

// ---- Weather Radar page ----

bool MfdController::radarBezelKey(BezelKey key) {
  // The small FMS knob trims the bearing line while it is displayed, otherwise
  // the antenna tilt (Pilot's Guide, Radar Controls). The large knob is left
  // alone so it still steps the page group out of the radar page.
  const bool onBearing =
      radarBearingLineOn_ && radarScan_ != RadarScan::Vertical;
  switch (key) {
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const float dir = key == BezelKey::FmsInnerCw ? 1.0f : -1.0f;
      if (onBearing) {
        radarBearingDeg_ = std::max(
            -kRadarBearingLimitDeg,
            std::min(kRadarBearingLimitDeg,
                     radarBearingDeg_ + dir * kRadarBearingStepDeg));
      } else {
        radarTiltDeg_ = std::max(
            -kRadarTiltLimitDeg,
            std::min(kRadarTiltLimitDeg,
                     radarTiltDeg_ + dir * kRadarTiltStepDeg));
      }
      return true;
    }
    default:
      return false;
  }
}

// ---- WPT ident search ----

void MfdController::wptResetInteraction() {
  wptEntry_.reset();
  wptHasSelection_ = false;
  wptFeature_ = MapFeature{};
}

void MfdController::wptCommitEntry() {
  if (wptEntry_.chars.empty()) {
    wptEntry_.active = false;
    return;
  }
  if (!wptEntry_.hasMatch) {
    wptEntry_.notFound = true;
    return;
  }
  wptFeature_ = wptEntry_.match;
  wptHasSelection_ = true;
  wptEntry_.active = false;
  wptEntry_.notFound = false;
}

bool MfdController::wptBezelKey(BezelKey key) {
  if (wptEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        wptCommitEntry();
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        wptEntry_.active = false;
        wptEntry_.notFound = false;
        break;
      case BezelKey::FmsInnerCw:
        wptEntry_.turnChar(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsInnerCcw:
        wptEntry_.turnChar(navSource_, mapData_, -1);
        break;
      case BezelKey::FmsOuterCw:
        wptEntry_.moveCursor(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsOuterCcw:
        wptEntry_.moveCursor(navSource_, mapData_, -1);
        break;
      default:
        break;
    }
    return true;
  }

  switch (key) {
    case BezelKey::FmsPush:
      wptEntry_.open(navSource_, mapData_, wptHasSelection_ ? wptFeature_.id : "");
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      wptEntry_.open(navSource_, mapData_);
      return true;
    default:
      return false;
  }
}

// ---- NRST list cursor ----

void MfdController::nrstResetInteraction() {
  nrstCursorOn_ = false;
  nrstSelected_ = 0;
}

bool MfdController::nrstBezelKey(BezelKey key) {
  const MfdPage p = page();
  if (p == MfdPage::NearestFrequencies) return false;

  if (key == BezelKey::FmsPush) {
    nrstCursorOn_ = !nrstCursorOn_;
    return true;
  }

  if (nrstCursorOn_) {
    switch (key) {
      case BezelKey::FmsOuterCw:
        ++nrstSelected_;
        return true;
      case BezelKey::FmsOuterCcw:
        nrstSelected_ = std::max(0, nrstSelected_ - 1);
        return true;
      case BezelKey::FmsInnerCw:
        stepPage(1);
        return true;
      case BezelKey::FmsInnerCcw:
        stepPage(-1);
        return true;
      default:
        return false;
    }
  }

  return false;
}

}  // namespace avionics
