#include "avionics/MfdController.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide 5.5): opens over any MFD
// page pre-filled with the active (or FPL-selected) waypoint; the first ENT
// confirms the waypoint and arms ACTIVATE?, the second engages the direct course.
namespace avionics {

void MfdController::directToOpen() {
  dtoOpen_ = true;
  dtoArmed_ = false;
  dtoPreservePlan_ = false;
  dtoPreserveLegIndex_ = -1;
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
    dtoPreserveLegIndex_ = fplCursorRow_;
    dtoPreservePlan_ = true;
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
    dtoPreservePlan_ = false;
    dtoPreserveLegIndex_ = -1;
    return true;
  }

  // FPL / PROC / MENU dismiss Direct-To and navigate (real unit behavior).
  if (isPageNavigationBezelKey(key)) {
    dtoOpen_ = false;
    dtoArmed_ = false;
    dtoEntry_.reset();
    dtoPreservePlan_ = false;
    dtoPreserveLegIndex_ = -1;
    return false;
  }

  // Armed: the ACTIVATE? prompt is highlighted; ENT engages the direct course.
  if (dtoArmed_) {
    if (key == BezelKey::Ent) {
      dtoRequestTarget_.lat = dtoEntry_.match.lat;
      dtoRequestTarget_.lon = dtoEntry_.match.lon;
      dtoRequestTarget_.id = dtoEntry_.match.id;
      dtoRequestPending_ = true;
      if (dtoPreservePlan_) {
        if (dtoPreserveLegIndex_ >= 0) {
          fplCursorRow_ = dtoPreserveLegIndex_;
        }
      } else {
        fplLegs_ = {dtoRequestTarget_};
        fplCursorRow_ = 0;
        fplPublishEdit();
      }
      dtoPreservePlan_ = false;
      dtoPreserveLegIndex_ = -1;
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
        return true;
      }
      if (dtoEntry_.hasMatch) {
        dtoEntry_.active = false;
        dtoArmed_ = true;
      } else {
        dtoEntry_.notFound = true;
      }
      return true;
    case BezelKey::FmsInnerCw:
      dtoEntry_.turnChar(navSource_, mapData_, +1);
      return true;
    case BezelKey::FmsInnerCcw:
      dtoEntry_.turnChar(navSource_, mapData_, -1);
      return true;
    case BezelKey::FmsOuterCw:
      dtoEntry_.moveCursor(navSource_, mapData_, +1);
      return true;
    case BezelKey::FmsOuterCcw:
      dtoEntry_.moveCursor(navSource_, mapData_, -1);
      return true;
    default:
      return false;
  }
}

bool MfdController::consumeDirectToRequest(MapLeg& out) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  return true;
}

}  // namespace avionics
