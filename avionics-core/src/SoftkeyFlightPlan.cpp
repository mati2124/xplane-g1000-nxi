#include <algorithm>

#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

// Active Flight Plan window editing (FPL bezel key, Pilot's Guide Fig. 5-48
// "Active Flight Plan Window on PFD"). Mirrors the MFD FPL page's insert/remove
// behavior so the PFD can build and change the active route (the origin and
// destination are simply the first and last rows of the editable leg list),
// minus the VNAV ALT column the PFD window does not show.
namespace avionics {
namespace {

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

void SoftkeyController::syncFlightPlanLegs(const MapData& map) {
  if (!legsEqual(map.flightPlan, fplLastMapPlan_)) {
    fplLastMapPlan_ = map.flightPlan;
    // Adopt the external change unless it is just the data source echoing our
    // own pending/published edit back to us.
    if (!fplEditPending_ && !legsEqual(map.flightPlan, fplLastPublished_)) {
      fplLegs_ = map.flightPlan;
      // The rows our interaction referenced may be gone; back out of any entry
      // or confirmation rather than act on the wrong waypoint.
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplConfirm_ = FplConfirm::None;
    }
  }
  fplCursorRow_ =
      std::max(0, std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_));
}

bool SoftkeyController::consumeFlightPlanEdit(std::vector<MapLeg>& out) {
  if (!fplEditPending_) return false;
  fplEditPending_ = false;
  out = fplLegs_;
  fplLastPublished_ = fplLegs_;
  return true;
}

void SoftkeyController::flightPlanPublishEdit() { fplEditPending_ = true; }

void SoftkeyController::flightPlanCommitEntry() {
  if (fplEntry_.chars.empty()) {
    fplEntry_.active = false;  // nothing spelled: back out
    return;
  }
  if (!fplEntry_.hasMatch) {
    fplEntry_.notFound = true;  // stay open so the ident can be corrected
    return;
  }

  // Insert before the cursor row (Pilot's Guide: "placed directly in front of
  // the highlighted waypoint"); the blank append slot adds to the end.
  const int row =
      std::max(0, std::min(static_cast<int>(fplLegs_.size()), fplCursorRow_));

  const std::string ident =
      fplEntry_.autofill.empty() ? fplEntry_.chars : fplEntry_.autofill;
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
      flightPlanPublishEdit();
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
  flightPlanPublishEdit();
}

bool SoftkeyController::flightPlanBezelKey(BezelKey key) {
  const int legCount = static_cast<int>(fplLegs_.size());

  // Modal confirmation (Remove <wpt>? / Delete Flight Plan?). ENT executes the
  // highlighted choice, CLR / knob push cancels, the knob toggles OK / CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          if (fplConfirm_ == FplConfirm::RemoveWaypoint) {
            if (fplCursorRow_ < legCount) {
              fplLegs_.erase(fplLegs_.begin() + fplCursorRow_);
              flightPlanPublishEdit();
            }
          } else {  // DeleteFlightPlan
            fplLegs_.clear();
            fplCursorRow_ = 0;
            flightPlanPublishEdit();
          }
        }
        fplConfirm_ = FplConfirm::None;
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        fplConfirm_ = FplConfirm::None;
        return true;
      case BezelKey::FmsOuterCw:
      case BezelKey::FmsOuterCcw:
      case BezelKey::FmsInnerCw:
      case BezelKey::FmsInnerCcw:
        fplConfirmOk_ = !fplConfirmOk_;
        return true;
      default:
        return true;  // modal: swallow everything else
    }
  }

  // Waypoint-ident entry: small knob spells, large knob moves the character
  // cursor, ENT accepts, CLR / knob push cancels.
  if (fplEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        flightPlanCommitEntry();
        return true;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        fplEntry_.active = false;
        fplEntry_.notFound = false;
        return true;
      case BezelKey::FmsInnerCw:
        fplEntry_.turnChar(navSource_, mapData_, +1);
        return true;
      case BezelKey::FmsInnerCcw:
        fplEntry_.turnChar(navSource_, mapData_, -1);
        return true;
      case BezelKey::FmsOuterCw:
        fplEntry_.moveCursor(navSource_, mapData_, +1);
        return true;
      case BezelKey::FmsOuterCcw:
        fplEntry_.moveCursor(navSource_, mapData_, -1);
        return true;
      default:
        return true;
    }
  }

  // MENU deletes the whole plan (single-option page menu collapsed into a
  // confirmation), matching the MFD's Delete Flight Plan.
  if (key == BezelKey::Menu) {
    if (legCount > 0) {
      fplConfirm_ = FplConfirm::DeleteFlightPlan;
      fplConfirmOk_ = true;
    }
    return true;
  }

  // Pushing the knob turns the selection cursor on / off.
  if (key == BezelKey::FmsPush) {
    fplCursorOn_ = !fplCursorOn_;
    fplCursorRow_ = std::max(0, std::min(legCount, fplCursorRow_));
    return true;
  }

  if (!fplCursorOn_) {
    // Cursor off: the knob scrolls the leg list; other keys (CLR to close the
    // window, the range rocker, the FPL toggle) fall through to the caller.
    const int last = std::max(0, legCount - 1);
    if (key == BezelKey::FmsOuterCw || key == BezelKey::FmsInnerCw) {
      fplCursorRow_ = std::min(last, fplCursorRow_ + 1);
      return true;
    }
    if (key == BezelKey::FmsOuterCcw || key == BezelKey::FmsInnerCcw) {
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    }
    return false;
  }

  // Cursor on: the large knob moves between rows (including the blank append
  // slot), the small knob opens the insert entry, CLR removes the highlighted
  // waypoint.
  switch (key) {
    case BezelKey::FmsOuterCw:
      fplCursorRow_ = std::min(legCount, fplCursorRow_ + 1);
      return true;
    case BezelKey::FmsOuterCcw:
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      fplEntry_.open(navSource_, mapData_);
      return true;
    case BezelKey::Clr:
      if (fplCursorRow_ < legCount) {
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(fplCursorRow_)].id;
      }
      return true;
    case BezelKey::Ent:
      return true;  // no function on a bare row, but the cursor owns the key
    default:
      return false;
  }
}

}  // namespace avionics
