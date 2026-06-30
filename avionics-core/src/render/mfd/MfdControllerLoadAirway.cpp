#include <algorithm>
#include <cmath>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MfdController.h"
#include "avionics/NavMath.h"

// FPL - Select Airway window (Pilot's Guide, Flight Planning - Load Airway).
// Opened from the Active Flight Plan page menu with the list cursor on an
// enroute fix lying on a published airway. The entry fix is fixed when the
// window opens; the Airway field picks which airway, the Exit field scrolls the
// fix chain to the exit waypoint, and Load? inserts the expanded segment after
// the entry fix (each inserted leg tagged with viaAirway so the FPL list groups
// it under an "Airway - <name>.<exit>" header).
namespace avionics {

std::string MfdController::loadAirwayName() const {
  if (loadAirway_.airwaySel < 0 ||
      loadAirway_.airwaySel >= static_cast<int>(loadAirway_.airways.size())) {
    return {};
  }
  return loadAirway_.airways[static_cast<std::size_t>(loadAirway_.airwaySel)];
}

std::string MfdController::loadAirwayExitIdent() const {
  if (loadAirway_.exitSel < 0 ||
      loadAirway_.exitSel >= static_cast<int>(loadAirway_.fixes.size())) {
    return {};
  }
  return loadAirway_.fixes[static_cast<std::size_t>(loadAirway_.exitSel)].id;
}

bool MfdController::loadAirwayCanLoad() const {
  return loadAirway_.open && !loadAirwayName().empty() &&
         loadAirway_.exitSel >= 1 &&
         loadAirway_.exitSel < static_cast<int>(loadAirway_.fixes.size());
}

void MfdController::loadAirwayRefreshFixes() {
  loadAirway_.fixes.clear();
  loadAirway_.exitSel = 1;
  const std::string airway = loadAirwayName();
  if (navSource_ != nullptr && !airway.empty() &&
      !loadAirway_.entryIdent.empty()) {
    loadAirway_.fixes =
        navSource_->airwayFixes(airway, loadAirway_.entryIdent);
  }
  // The exit defaults to the first fix after the entry; clamp into range.
  if (loadAirway_.fixes.size() < 2) {
    loadAirway_.exitSel = static_cast<int>(loadAirway_.fixes.size()) - 1;
  }
  loadAirwayRefreshCourse();
}

void MfdController::loadAirwayRefreshCourse() {
  loadAirway_.hasCourse = false;
  loadAirway_.dtkDeg = 0.0f;
  loadAirway_.disNm = 0.0f;
  if (loadAirway_.exitSel < 1 ||
      loadAirway_.exitSel >= static_cast<int>(loadAirway_.fixes.size())) {
    return;
  }
  // Cumulative distance along the chain from the entry fix to the exit, and the
  // course of the final segment into the exit (matching the FPL DTK column).
  double dis = 0.0;
  for (int i = 1; i <= loadAirway_.exitSel; ++i) {
    const MapLeg& a = loadAirway_.fixes[static_cast<std::size_t>(i - 1)];
    const MapLeg& b = loadAirway_.fixes[static_cast<std::size_t>(i)];
    dis += navDistanceNm(a.lat, a.lon, b.lat, b.lon);
  }
  const MapLeg& prev =
      loadAirway_.fixes[static_cast<std::size_t>(loadAirway_.exitSel - 1)];
  const MapLeg& exit =
      loadAirway_.fixes[static_cast<std::size_t>(loadAirway_.exitSel)];
  loadAirway_.dtkDeg =
      static_cast<float>(navBearingDeg(prev.lat, prev.lon, exit.lat, exit.lon));
  loadAirway_.disNm = static_cast<float>(dis);
  loadAirway_.hasCourse = true;
}

void MfdController::openLoadAirwayWindow(const std::string& entryIdent) {
  loadAirway_ = LoadAirwayState{};
  if (entryIdent.empty()) return;
  loadAirway_.entryIdent = entryIdent;
  // Locate the entry fix in the active plan so the segment is inserted right
  // after it; fall back to append when it is not found (defensive).
  loadAirway_.entryLegIndex = -1;
  for (int i = 0; i < static_cast<int>(fplLegs_.size()); ++i) {
    if (fplLegs_[static_cast<std::size_t>(i)].id == entryIdent) {
      loadAirway_.entryLegIndex = i;
      break;
    }
  }
  loadAirway_.airways = airwaysThroughFix(entryIdent);
  if (loadAirway_.airways.empty()) return;  // nothing to load; window stays inert
  loadAirway_.open = true;
  loadAirway_.airwaySel = 0;
  loadAirway_.field = LoadAirwayField::Airway;
  loadAirwayRefreshFixes();
}

void MfdController::closeLoadAirwayWindow() {
  loadAirway_.open = false;
}

void MfdController::loadAirwayCommit() {
  if (!loadAirwayCanLoad() || navSource_ == nullptr) {
    closeLoadAirwayWindow();
    return;
  }
  const std::string airway = loadAirwayName();
  const std::string exit = loadAirwayExitIdent();
  std::vector<MapLeg> segment =
      navSource_->expandAirway(airway, loadAirway_.entryIdent, exit);
  if (segment.empty()) {
    closeLoadAirwayWindow();
    return;
  }
  for (MapLeg& leg : segment) leg.viaAirway = airway;

  // Insert after the entry fix (append when the entry fix is gone).
  int at = loadAirway_.entryLegIndex;
  if (at < 0 || at >= static_cast<int>(fplLegs_.size())) {
    at = static_cast<int>(fplLegs_.size()) - 1;
  }
  const std::size_t insertPos = static_cast<std::size_t>(at + 1);
  fplLegs_.insert(fplLegs_.begin() + static_cast<std::ptrdiff_t>(insertPos),
                  segment.begin(), segment.end());

  // When the airway exit fix coincides with the next existing waypoint in the
  // plan (commonly the destination), merge them: the airway-tagged exit fix is
  // canonical and the duplicate that followed is removed. Otherwise the exit
  // fix is suppressed as a duplicate ident and never shows under its
  // "Airway - <name>.<exit>" header.
  const std::size_t exitPos = insertPos + segment.size() - 1;
  const std::size_t afterPos = exitPos + 1;
  if (afterPos < fplLegs_.size() &&
      fplLegIdentsEqual(fplLegs_[exitPos].id, fplLegs_[afterPos].id)) {
    MapLeg& exitLeg = fplLegs_[exitPos];
    const MapLeg& dup = fplLegs_[afterPos];
    // Keep any constraint the following waypoint carried (e.g. destination).
    if (exitLeg.altitudeConstraint == AltConstraintType::None &&
        dup.altitudeConstraint != AltConstraintType::None) {
      exitLeg.altitudeConstraintFt = dup.altitudeConstraintFt;
      exitLeg.altitudeConstraint = dup.altitudeConstraint;
      exitLeg.altitudeDesignated = dup.altitudeDesignated;
    }
    fplLegs_.erase(fplLegs_.begin() + static_cast<std::ptrdiff_t>(afterPos));
  }

  closeLoadAirwayWindow();
  fplClampCursorRow();
  fplPublishEdit();
}

bool MfdController::loadAirwayBezelKey(BezelKey key) {
  if (!loadAirway_.open) return false;
  switch (key) {
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      closeLoadAirwayWindow();
      return true;
    case BezelKey::Ent:
      if (loadAirway_.field == LoadAirwayField::Load) {
        loadAirwayCommit();
      } else {
        loadAirway_.field =
            loadAirway_.field == LoadAirwayField::Airway
                ? LoadAirwayField::Exit
                : LoadAirwayField::Load;
      }
      return true;
    case BezelKey::FmsOuterCw:
      // Large knob steps the field: Airway -> Exit -> Load (clamped at Load).
      if (loadAirway_.field == LoadAirwayField::Airway) {
        loadAirway_.field = LoadAirwayField::Exit;
      } else if (loadAirway_.field == LoadAirwayField::Exit) {
        loadAirway_.field = LoadAirwayField::Load;
      }
      return true;
    case BezelKey::FmsOuterCcw:
      if (loadAirway_.field == LoadAirwayField::Load) {
        loadAirway_.field = LoadAirwayField::Exit;
      } else if (loadAirway_.field == LoadAirwayField::Exit) {
        loadAirway_.field = LoadAirwayField::Airway;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const int step = key == BezelKey::FmsInnerCw ? +1 : -1;
      if (loadAirway_.field == LoadAirwayField::Airway) {
        const int n = static_cast<int>(loadAirway_.airways.size());
        if (n > 0) {
          loadAirway_.airwaySel = (loadAirway_.airwaySel + step + n) % n;
          loadAirwayRefreshFixes();
        }
      } else if (loadAirway_.field == LoadAirwayField::Exit) {
        const int last = static_cast<int>(loadAirway_.fixes.size()) - 1;
        // The exit never selects the entry fix (index 0).
        loadAirway_.exitSel =
            std::max(1, std::min(last, loadAirway_.exitSel + step));
        loadAirwayRefreshCourse();
      }
      return true;
    }
    default:
      return true;  // the window is modal: swallow every other key
  }
}

}  // namespace avionics
