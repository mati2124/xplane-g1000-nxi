#include "avionics/SoftkeyController.h"

#include "avionics/FplRouteEdit.h"
#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide Fig. 5-45): opens over
// the PFD blank for ident entry; the first ENT confirms the waypoint and arms
// Activate?, the second engages the direct course. Pressing Direct-To on a
// published HOLD row in the flight plan opens the compact "Activate hold"
// confirmation instead (trainer FPL Direct-To hold).
namespace avionics {

void SoftkeyController::openHoldActivatePrompt(int legIndex) {
  if (legIndex < 0 || legIndex >= static_cast<int>(fplLegs_.size())) return;
  const MapLeg& leg = fplLegs_[static_cast<std::size_t>(legIndex)];
  if (!leg.hold.active) return;
  holdActivatePromptLeg_ = leg;
  holdActivatePromptActivate_ = true;
  holdActivatePromptActive_ = true;
  dtoOpen_ = false;
  dtoArmed_ = false;
  dtoEntry_.reset();
}

void SoftkeyController::closeHoldActivatePrompt() {
  holdActivatePromptActive_ = false;
  holdActivatePromptLeg_ = {};
  holdActivatePromptActivate_ = true;
}

bool SoftkeyController::holdActivatePromptBezelKey(BezelKey key) {
  if (!holdActivatePromptActive_) return false;
  switch (key) {
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      holdActivatePromptActivate_ = !holdActivatePromptActivate_;
      return true;
    case BezelKey::Ent:
      if (holdActivatePromptActivate_) {
        dtoRequestTarget_ = holdActivatePromptLeg_;
        dtoRequestHold_ = true;
        dtoRequestPending_ = true;
      }
      closeHoldActivatePrompt();
      return true;
    case BezelKey::Clr:
    case BezelKey::FmsPush:
    case BezelKey::DirectTo:
      closeHoldActivatePrompt();
      return true;
    default:
      return false;
  }
}

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
    if (window_ == PfdWindow::FlightPlan) {
      FplRouteEdit edit{
          fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
          fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
          nullptr};
      edit.directToActive = mapDirectToActive();
      edit.localDraft = fplLocalDraft_;
      const std::string approachAirport = flightPlanApproachAirportIcao();
      if (fplCursorOnHoldRow(edit, approachAirport, FplCursorLayout::SectionRows)) {
        const int legIndex =
            fplCursorLegIndex(edit, approachAirport, FplCursorLayout::SectionRows);
        openHoldActivatePrompt(legIndex);
        return true;
      }
    }
    dtoPreservePlan_ = false;
    dtoPreserveLegIndex_ = -1;
    std::string initial;
    if (window_ == PfdWindow::FlightPlan) {
      dtoPreserveLegIndex_ = flightPlanSelectedLegIndex();
      dtoPreservePlan_ = dtoPreserveLegIndex_ >= 0;
      initial = flightPlanSelectedLegIdent();
    }
    if (initial.empty() && window_ == PfdWindow::Nearest && !nearest_.empty()) {
      const int idx = std::max(
          0, std::min(nearestCursor_, static_cast<int>(nearest_.size()) - 1));
      initial = nearest_[static_cast<std::size_t>(idx)].id;
    }
    if (initial.empty() && !activeWaypoint_.empty()) {
      initial = activeWaypoint_;
    }
    directToOpen(initial);
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

  // Armed: ACTIVATE? is highlighted; ENT engages the direct course.
  if (dtoArmed_) {
    if (key == BezelKey::Ent) {
      dtoRequestTarget_ = resolveDirectToTargetLeg(
          dtoEntry_, dtoPreservePlan_, dtoPreserveLegIndex_, fplLegs_);
      dtoRequestHold_ = false;
      dtoRequestPending_ = true;
      if (dtoPreservePlan_) {
        if (dtoPreserveLegIndex_ >= 0) {
          fplCursorRow_ = dtoPreserveLegIndex_;
        }
      } else {
        flightPlanApplyDirectTo(dtoRequestTarget_);
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
      // First ENT confirms the waypoint and arms Activate? (an unknown ident
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

bool SoftkeyController::consumeDirectToRequest(MapLeg& out, bool* flyHold) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  if (flyHold != nullptr) *flyHold = dtoRequestHold_;
  dtoRequestHold_ = false;
  return true;
}

}  // namespace avionics
