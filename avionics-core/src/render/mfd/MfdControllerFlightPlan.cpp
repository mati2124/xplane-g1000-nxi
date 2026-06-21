#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "avionics/MfdController.h"

// Active Flight Plan page (FPL group, Pilot's Guide 5.6): caches the map plan
// plus local edits, the leg-row cursor, the Waypoint Information insert window,
// the VNAV altitude-constraint entry, and the remove/delete confirmations.
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

void MfdController::syncFlightPlan(const MapData& map,
                                   const std::string& activeWaypoint) {
  mapData_ = &map;
  activeWaypoint_ = activeWaypoint;

  if (!legsEqual(map.flightPlan, fplLastMapPlan_)) {
    fplLastMapPlan_ = map.flightPlan;
    // Adopt the change unless it is just the data source catching up with our
    // own pending/published edit.
    if (!fplEditPending_ && !legsEqual(map.flightPlan, fplLastPublished_) &&
        !legsEqual(fplLegs_, fplLastPublished_)) {
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

FmsWaypointEntry* MfdController::activeWaypointEntry() {
  if (dtoOpen_ && dtoEntry_.active && !dtoArmed_) return &dtoEntry_;
  if (fplEntry_.active) return &fplEntry_;
  if (wptEntry_.active) return &wptEntry_;
  return nullptr;
}

bool MfdController::applyGcuEntryKey(char ch) {
  FmsWaypointEntry* entry = activeWaypointEntry();
  if (entry == nullptr) return false;
  if (ch == '\b') {
    entry->backspaceChar(navSource_, mapData_);
  } else {
    entry->typeChar(navSource_, mapData_, ch);
  }
  return true;
}

}  // namespace avionics
