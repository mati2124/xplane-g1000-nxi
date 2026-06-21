#include <algorithm>

#include "avionics/SoftkeyController.h"
#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"
#include "render/pfd/PfdFlightPlanSections.h"

// Active Flight Plan window editing (FPL bezel key, Pilot's Guide Fig. 5-48
// "Active Flight Plan Window on PFD"). Mirrors the MFD FPL page's insert/remove
// behavior so the PFD can build and change the active route (the origin and
// destination are simply the first and last rows of the editable leg list),
// minus the VNAV ALT column the PFD window does not show.
namespace avionics {
namespace {

using pfd::fplInsertIndexForSectionRow;
using pfd::fplLegIndexForSectionRow;
using pfd::fplSectionSelectableCount;

bool legsEqual(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id || a[i].lat != b[i].lat || a[i].lon != b[i].lon ||
        a[i].procedureRole != b[i].procedureRole) {
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
    if (!fplEditPending_ && !legsEqual(map.flightPlan, fplLastPublished_) &&
        !legsEqual(fplLegs_, fplLastPublished_)) {
      fplLegs_ = map.flightPlan;
      const int n = static_cast<int>(fplLegs_.size());
      fplDestinationFilled_ = n >= 3 || n == 2;
      fplApproachLegStart_ = 0;
      fplApproachLegCount_ = 0;
      fplLoadedApproach_ = {};
      // The rows our interaction referenced may be gone; back out of any entry
      // or confirmation rather than act on the wrong waypoint.
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplConfirm_ = FplConfirm::None;
    }
  }
  if (fplApproachLegCount_ > 0) {
    const int last = fplApproachLegStart_ + fplApproachLegCount_ - 1;
    fplCursorRow_ = std::max(fplApproachLegStart_,
                             std::min(last, fplCursorRow_));
  } else {
    fplCursorRow_ = std::max(
        0, std::min(fplSectionSelectableCount(static_cast<int>(fplLegs_.size())) - 1,
                    fplCursorRow_));
  }
}

bool SoftkeyController::consumeFlightPlanEdit(std::vector<MapLeg>& out) {
  if (!fplEditPending_) return false;
  fplEditPending_ = false;
  out = fplLegs_;
  fplLastPublished_ = fplLegs_;
  return true;
}

void SoftkeyController::flightPlanPublishEdit() { fplEditPending_ = true; }

std::string SoftkeyController::flightPlanSelectedLegIdent() const {
  if (window_ != PfdWindow::FlightPlan || !fplCursorOn_) return {};

  const int legCount = static_cast<int>(fplLegs_.size());
  if (fplApproachLegCount_ > 0) {
    if (fplCursorRow_ >= fplApproachLegStart_ &&
        fplCursorRow_ < fplApproachLegStart_ + fplApproachLegCount_) {
      return fplLegs_[static_cast<std::size_t>(fplCursorRow_)].id;
    }
    return {};
  }

  const int legIndex =
      fplLegIndexForSectionRow(fplCursorRow_, legCount, fplDestinationFilled_);
  if (legIndex < 0 || legIndex >= legCount) return {};
  return fplLegs_[static_cast<std::size_t>(legIndex)].id;
}

int SoftkeyController::flightPlanSelectedLegIndex() const {
  if (window_ != PfdWindow::FlightPlan || !fplCursorOn_) return -1;

  const int legCount = static_cast<int>(fplLegs_.size());
  if (fplApproachLegCount_ > 0) {
    if (fplCursorRow_ >= fplApproachLegStart_ &&
        fplCursorRow_ < fplApproachLegStart_ + fplApproachLegCount_) {
      return fplCursorRow_;
    }
    return -1;
  }

  return fplLegIndexForSectionRow(fplCursorRow_, legCount, fplDestinationFilled_);
}

void SoftkeyController::flightPlanApplyDirectTo(const MapLeg& target) {
  fplLegs_ = {target};
  fplDestinationFilled_ = false;
  fplCursorRow_ = 0;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
  flightPlanPublishEdit();
}

void SoftkeyController::flightPlanCommitEntry() {
  if (fplEntry_.chars.empty()) {
    fplEntry_.active = false;  // nothing spelled: back out
    return;
  }
  if (!fplEntry_.hasMatch) {
    fplEntry_.notFound = true;  // stay open so the ident can be corrected
    return;
  }

  // Insert at the section row the cursor highlights (Origin / Enroute / Dest).
  const int legCount = static_cast<int>(fplLegs_.size());
  const int lastSection =
      std::max(0, fplSectionSelectableCount(legCount) - 1);
  const int legIndex = fplLegIndexForSectionRow(
      fplCursorRow_, legCount, fplDestinationFilled_);
  const int row = fplInsertIndexForSectionRow(
      fplCursorRow_, legCount, fplDestinationFilled_);

  if (fplCursorRow_ == lastSection) {
    fplDestinationFilled_ = true;
  } else if (fplCursorRow_ == 1 && legCount < 2) {
    fplDestinationFilled_ = false;
  }

  const std::string ident =
      fplEntry_.autofill.empty() ? fplEntry_.chars : fplEntry_.autofill;
  if (navSource_ != nullptr && navSource_->isAirwayName(ident) && row > 0 &&
      row < legCount) {
    const std::vector<MapLeg> expanded = navSource_->expandAirway(
        ident, fplLegs_[static_cast<std::size_t>(row - 1)].id,
        fplLegs_[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      fplLegs_.insert(fplLegs_.begin() + row, expanded.begin(), expanded.end());
      if (fplLegs_.size() >= 3) fplDestinationFilled_ = true;
      fplCursorRow_ = std::min(
          lastSection, fplCursorRow_ + static_cast<int>(expanded.size()));
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
  if (legIndex >= 0 && legIndex < legCount) {
    fplLegs_[static_cast<std::size_t>(legIndex)] = leg;
  } else {
    fplLegs_.insert(fplLegs_.begin() + row, leg);
  }
  if (fplLegs_.size() >= 3) fplDestinationFilled_ = true;
  fplEntry_.active = false;
  fplEntry_.notFound = false;
  flightPlanPublishEdit();
}

bool SoftkeyController::flightPlanEntryHasGeo() const {
  return fplEntry_.hasMatch && mapData_ != nullptr && mapData_->positionValid;
}

float SoftkeyController::flightPlanEntryBearingDeg() const {
  if (!flightPlanEntryHasGeo()) return 0.0f;
  return static_cast<float>(navBearingDeg(mapData_->ownshipLat,
                                          mapData_->ownshipLon,
                                          fplEntry_.match.lat,
                                          fplEntry_.match.lon));
}

float SoftkeyController::flightPlanEntryDistanceNm() const {
  if (!flightPlanEntryHasGeo()) return 0.0f;
  return static_cast<float>(navDistanceNm(mapData_->ownshipLat,
                                          mapData_->ownshipLon,
                                          fplEntry_.match.lat,
                                          fplEntry_.match.lon));
}

bool SoftkeyController::flightPlanBezelKey(BezelKey key) {
  const int legCount = static_cast<int>(fplLegs_.size());
  const bool approachView = fplApproachLegCount_ > 0;
  const int approachLast =
      approachView ? fplApproachLegStart_ + fplApproachLegCount_ - 1 : 0;

  // Modal confirmation (Remove <wpt>? / Delete Flight Plan?). ENT executes the
  // highlighted choice, CLR / knob push cancels, the knob toggles OK / CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          if (fplConfirm_ == FplConfirm::RemoveWaypoint) {
            int legIndex = -1;
            if (fplApproachLegCount_ > 0 &&
                fplCursorRow_ >= fplApproachLegStart_ &&
                fplCursorRow_ < fplApproachLegStart_ + fplApproachLegCount_) {
              legIndex = fplCursorRow_;
            } else {
              legIndex = fplLegIndexForSectionRow(
                  fplCursorRow_, static_cast<int>(fplLegs_.size()),
                  fplDestinationFilled_);
            }
            if (legIndex >= 0 &&
                legIndex < static_cast<int>(fplLegs_.size())) {
              fplLegs_.erase(fplLegs_.begin() + legIndex);
              const int n = static_cast<int>(fplLegs_.size());
              if (n <= 1) {
                fplDestinationFilled_ = false;
              } else if (n == 2 && !fplDestinationFilled_) {
                // still origin + enroute
              } else if (n < 3) {
                fplDestinationFilled_ = n == 2;
              }
              if (fplApproachLegCount_ > 0) {
                if (legIndex >= fplApproachLegStart_ &&
                    legIndex < fplApproachLegStart_ + fplApproachLegCount_) {
                  fplApproachLegCount_--;
                  if (fplApproachLegCount_ <= 0) {
                    fplApproachLegStart_ = 0;
                    fplLoadedApproach_ = {};
                  }
                } else if (legIndex < fplApproachLegStart_) {
                  fplApproachLegStart_--;
                }
              }
              flightPlanPublishEdit();
            }
          } else {  // DeleteFlightPlan
            fplLegs_.clear();
            fplCursorRow_ = 0;
            fplDestinationFilled_ = false;
            fplApproachLegStart_ = 0;
            fplApproachLegCount_ = 0;
            fplLoadedApproach_ = {};
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
    if (approachView) {
      fplCursorRow_ = std::max(fplApproachLegStart_,
                               std::min(approachLast, fplCursorRow_));
    } else {
      fplCursorRow_ = std::max(
          0, std::min(fplSectionSelectableCount(legCount) - 1, fplCursorRow_));
    }
    return true;
  }

  if (!fplCursorOn_) {
    // Cursor off: the knob scrolls the section list; other keys fall through.
    const int last =
        approachView ? approachLast
                     : std::max(0, fplSectionSelectableCount(legCount) - 1);
    if (key == BezelKey::FmsOuterCw || key == BezelKey::FmsInnerCw) {
      fplCursorRow_ = std::min(last, fplCursorRow_ + 1);
      return true;
    }
    if (key == BezelKey::FmsOuterCcw || key == BezelKey::FmsInnerCcw) {
      fplCursorRow_ = std::max(approachView ? fplApproachLegStart_ : 0,
                               fplCursorRow_ - 1);
      return true;
    }
    return false;
  }

  // Cursor on: the large knob moves between section rows, the small knob opens
  // the insert entry, CLR removes the highlighted waypoint.
  const int lastSection =
      approachView ? approachLast
                   : std::max(0, fplSectionSelectableCount(legCount) - 1);
  switch (key) {
    case BezelKey::FmsOuterCw:
      fplCursorRow_ = std::min(lastSection, fplCursorRow_ + 1);
      return true;
    case BezelKey::FmsOuterCcw:
      fplCursorRow_ = std::max(approachView ? fplApproachLegStart_ : 0,
                               fplCursorRow_ - 1);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      if (!approachView) {
        fplEntry_.open(navSource_, mapData_);
      }
      return true;
    case BezelKey::Clr:
      if (approachView) {
        if (fplCursorRow_ >= fplApproachLegStart_ &&
            fplCursorRow_ < fplApproachLegStart_ + fplApproachLegCount_) {
          fplConfirm_ = FplConfirm::RemoveWaypoint;
          fplConfirmOk_ = true;
          fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(fplCursorRow_)].id;
        }
      } else if (fplLegIndexForSectionRow(fplCursorRow_, legCount,
                                          fplDestinationFilled_) >= 0) {
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ =
            fplLegs_[static_cast<std::size_t>(fplLegIndexForSectionRow(
                fplCursorRow_, legCount, fplDestinationFilled_))]
                .id;
      }
      return true;
    case BezelKey::Ent:
      return true;  // no function on a bare row, but the cursor owns the key
    default:
      return false;
  }
}

FmsWaypointEntry* SoftkeyController::activeWaypointEntry() {
  if (dtoOpen_ && dtoEntry_.active && !dtoArmed_) return &dtoEntry_;
  if (fplEntry_.active) return &fplEntry_;
  return nullptr;
}

bool SoftkeyController::applyGcuEntryKey(char ch) {
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
