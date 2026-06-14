#include "avionics/MfdController.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide 5.5): opens over any MFD
// page pre-filled with the active (or FPL-selected) waypoint; the first ENT
// confirms the waypoint and arms ACTIVATE?, the second engages the direct course.
namespace avionics {

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

}  // namespace avionics
