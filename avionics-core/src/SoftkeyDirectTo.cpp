#include "avionics/SoftkeyController.h"

#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide Fig. 5-45): opens over
// the PFD blank for ident entry; the first ENT confirms the waypoint and arms
// Activate?, the second engages the direct course.
namespace avionics {

void SoftkeyController::openDirectToWindow(const std::string& initial) {
  directToOpen(initial);
}

void SoftkeyController::directToOpen(const std::string& initial) {
  dtoOpen_ = true;
  dtoArmed_ = false;
  pageMenuOpen_ = false;
  // Close any open softkey pop-up so the Direct-To window does not overlap it.
  window_ = PfdWindow::None;
  // Open blank so the pilot enters a destination (trainer PFD Direct To.bmp).
  // Callers that need a pre-filled ident (e.g. screenshot harness) pass one in.
  dtoEntry_.open(navSource_, mapData_, initial);
}

bool SoftkeyController::directToBezelKey(BezelKey key) {
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

  // Armed: the Activate? prompt is highlighted; ENT engages the direct course.
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
      // First ENT confirms the waypoint and arms Activate? (an unknown ident
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

bool SoftkeyController::directToHasGeo() const {
  return dtoEntry_.hasMatch && mapData_ != nullptr && mapData_->positionValid;
}

float SoftkeyController::directToBearingDeg() const {
  if (!directToHasGeo()) return 0.0f;
  return static_cast<float>(navBearingDeg(mapData_->ownshipLat,
                                          mapData_->ownshipLon,
                                          dtoEntry_.match.lat,
                                          dtoEntry_.match.lon));
}

float SoftkeyController::directToDistanceNm() const {
  if (!directToHasGeo()) return 0.0f;
  return static_cast<float>(navDistanceNm(mapData_->ownshipLat,
                                          mapData_->ownshipLon,
                                          dtoEntry_.match.lat,
                                          dtoEntry_.match.lon));
}

bool SoftkeyController::consumeDirectToRequest(MapLeg& out) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  return true;
}

}  // namespace avionics
