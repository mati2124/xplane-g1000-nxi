#include <algorithm>

#include "avionics/FplRouteEdit.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// FPL - Select Airway window (Pilot's Guide, Flight Planning - Load Airway), the
// compact PFD variant of the MFD Load Airway flow. Opened from the Active Flight
// Plan window's page menu with the list cursor on an enroute fix lying on a
// published airway. The entry fix is fixed when the window opens; the Airway
// field picks which airway, the Exit field scrolls the fix chain to the exit
// waypoint, and Load? inserts the expanded segment after the entry fix (each
// inserted leg tagged with viaAirway so the FPL list groups it under an
// "Airway - <name>.<exit>" header). Unlike the MFD window the PFD popout shows
// no fix list or DTK/DIS readouts, only Entry / Airway / Exit / Load?.
namespace avionics {

std::vector<std::string> SoftkeyController::airwaysThroughFix(
    const std::string& ident) const {
  if (navSource_ == nullptr || ident.empty()) return {};
  return navSource_->airwaysThrough(ident);
}

bool SoftkeyController::flightPlanHasAirwayLegs() const {
  for (const MapLeg& leg : fplLegs_) {
    if (!leg.viaAirway.empty()) return true;
  }
  return false;
}

std::string SoftkeyController::loadAirwayName() const {
  if (fplLoadAirway_.airwaySel < 0 ||
      fplLoadAirway_.airwaySel >=
          static_cast<int>(fplLoadAirway_.airways.size())) {
    return {};
  }
  return fplLoadAirway_
      .airways[static_cast<std::size_t>(fplLoadAirway_.airwaySel)];
}

std::string SoftkeyController::loadAirwayExitIdent() const {
  if (fplLoadAirway_.exitSel < 0 ||
      fplLoadAirway_.exitSel >= static_cast<int>(fplLoadAirway_.fixes.size())) {
    return {};
  }
  return fplLoadAirway_.fixes[static_cast<std::size_t>(fplLoadAirway_.exitSel)]
      .id;
}

bool SoftkeyController::loadAirwayCanLoad() const {
  return fplLoadAirway_.open && !loadAirwayName().empty() &&
         fplLoadAirway_.exitSel >= 1 &&
         fplLoadAirway_.exitSel < static_cast<int>(fplLoadAirway_.fixes.size());
}

void SoftkeyController::loadAirwayRefreshFixes() {
  fplLoadAirway_.fixes.clear();
  fplLoadAirway_.exitSel = 1;
  const std::string airway = loadAirwayName();
  if (navSource_ != nullptr && !airway.empty() &&
      !fplLoadAirway_.entryIdent.empty()) {
    fplLoadAirway_.fixes =
        navSource_->airwayFixes(airway, fplLoadAirway_.entryIdent);
  }
  // The exit defaults to the first fix after the entry; clamp into range.
  if (fplLoadAirway_.fixes.size() < 2) {
    fplLoadAirway_.exitSel = static_cast<int>(fplLoadAirway_.fixes.size()) - 1;
  }
}

void SoftkeyController::openLoadAirwayWindow(const std::string& entryIdent) {
  fplLoadAirway_ = LoadAirwayState{};
  if (entryIdent.empty()) return;
  fplLoadAirway_.entryIdent = entryIdent;
  // Locate the entry fix in the active plan so the segment is inserted right
  // after it; fall back to append when it is not found (defensive).
  fplLoadAirway_.entryLegIndex = -1;
  for (int i = 0; i < static_cast<int>(fplLegs_.size()); ++i) {
    if (fplLegs_[static_cast<std::size_t>(i)].id == entryIdent) {
      fplLoadAirway_.entryLegIndex = i;
      break;
    }
  }
  fplLoadAirway_.airways = airwaysThroughFix(entryIdent);
  if (fplLoadAirway_.airways.empty()) return;  // nothing to load; window inert
  fplLoadAirway_.open = true;
  fplLoadAirway_.airwaySel = 0;
  fplLoadAirway_.field = LoadAirwayField::Airway;
  loadAirwayRefreshFixes();
}

void SoftkeyController::closeLoadAirwayWindow() { fplLoadAirway_.open = false; }

void SoftkeyController::loadAirwayCommit() {
  if (!loadAirwayCanLoad() || navSource_ == nullptr) {
    closeLoadAirwayWindow();
    return;
  }
  const std::string airway = loadAirwayName();
  const std::string exit = loadAirwayExitIdent();
  std::vector<MapLeg> segment =
      navSource_->expandAirway(airway, fplLoadAirway_.entryIdent, exit);
  if (segment.empty()) {
    closeLoadAirwayWindow();
    return;
  }
  for (MapLeg& leg : segment) leg.viaAirway = airway;

  // Insert after the entry fix (append when the entry fix is gone).
  int at = fplLoadAirway_.entryLegIndex;
  if (at < 0 || at >= static_cast<int>(fplLegs_.size())) {
    at = static_cast<int>(fplLegs_.size()) - 1;
  }
  const std::size_t insertPos = static_cast<std::size_t>(at + 1);
  fplLegs_.insert(fplLegs_.begin() + static_cast<std::ptrdiff_t>(insertPos),
                  segment.begin(), segment.end());

  closeLoadAirwayWindow();
  FplRouteEdit edit = flightPlanRouteEditState();
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
  flightPlanPublishEdit();
}

bool SoftkeyController::loadAirwayBezelKey(BezelKey key) {
  if (!fplLoadAirway_.open) return false;
  switch (key) {
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      closeLoadAirwayWindow();
      return true;
    case BezelKey::Ent:
      if (fplLoadAirway_.field == LoadAirwayField::Load) {
        loadAirwayCommit();
      } else {
        fplLoadAirway_.field = fplLoadAirway_.field == LoadAirwayField::Airway
                                   ? LoadAirwayField::Exit
                                   : LoadAirwayField::Load;
      }
      return true;
    case BezelKey::FmsOuterCw:
      // Large knob steps the field: Airway -> Exit -> Load (clamped at Load).
      if (fplLoadAirway_.field == LoadAirwayField::Airway) {
        fplLoadAirway_.field = LoadAirwayField::Exit;
      } else if (fplLoadAirway_.field == LoadAirwayField::Exit) {
        fplLoadAirway_.field = LoadAirwayField::Load;
      }
      return true;
    case BezelKey::FmsOuterCcw:
      if (fplLoadAirway_.field == LoadAirwayField::Load) {
        fplLoadAirway_.field = LoadAirwayField::Exit;
      } else if (fplLoadAirway_.field == LoadAirwayField::Exit) {
        fplLoadAirway_.field = LoadAirwayField::Airway;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      const int step = key == BezelKey::FmsInnerCw ? +1 : -1;
      if (fplLoadAirway_.field == LoadAirwayField::Airway) {
        const int n = static_cast<int>(fplLoadAirway_.airways.size());
        if (n > 0) {
          fplLoadAirway_.airwaySel = (fplLoadAirway_.airwaySel + step + n) % n;
          loadAirwayRefreshFixes();
        }
      } else if (fplLoadAirway_.field == LoadAirwayField::Exit) {
        const int last = static_cast<int>(fplLoadAirway_.fixes.size()) - 1;
        // The exit never selects the entry fix (index 0).
        fplLoadAirway_.exitSel =
            std::max(1, std::min(last, fplLoadAirway_.exitSel + step));
      }
      return true;
    }
    default:
      return true;  // the window is modal: swallow every other key
  }
}

}  // namespace avionics
