#include "avionics/MfdController.h"

#include "avionics/DataSource.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/ProcedureSupport.h"
#include "render/mfd/MfdPageSupport.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide 5.5): opens over any MFD
// page pre-filled with the active (or FPL-selected) waypoint; the first ENT
// confirms the waypoint and arms ACTIVATE?, the second engages the direct course.
// Pressing Direct-To on a published HOLD row in the flight plan opens the compact
// "Activate hold" confirmation instead (trainer FPL Direct-To hold).
namespace avionics {

void MfdController::openHoldActivatePrompt(int legIndex) {
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

void MfdController::closeHoldActivatePrompt() {
  holdActivatePromptActive_ = false;
  holdActivatePromptLeg_ = {};
  holdActivatePromptActivate_ = true;
}

bool MfdController::holdActivatePromptBezelKey(BezelKey key) {
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

void MfdController::openDirectToWindow(const std::string& initial) {
  dtoOpen_ = true;
  dtoArmed_ = false;
  dtoPreservePlan_ = false;
  dtoPreserveLegIndex_ = -1;
  dtoPreserveFplCursorRow_ = -1;
  dtoEntry_.open(navSource_, mapData_, initial);
}

void MfdController::directToOpen() {
  dtoOpen_ = true;
  dtoArmed_ = false;
  dtoPreservePlan_ = false;
  dtoPreserveLegIndex_ = -1;
  dtoPreserveFplCursorRow_ = -1;
  // Approach Loading Sequence box: Direct-To targets the highlighted leg fix
  // (Pilot's Guide 5.8) when the FMS cursor has descended into the sequence.
  if (procMenuOpen_ && procMenu_.sequenceFocused) {
    const std::vector<MapLeg> legs = procPreviewLegs();
    const int sel = procMenu_.sequenceSelected;
    if (sel >= 0 && sel < static_cast<int>(legs.size())) {
      const MapLeg& leg = legs[static_cast<std::size_t>(sel)];
      dtoEntry_.open(navSource_, mapData_, leg.id);
      dtoEntry_.match.id = leg.id;
      if (leg.lat != 0.0 || leg.lon != 0.0) {
        dtoEntry_.match.lat = leg.lat;
        dtoEntry_.match.lon = leg.lon;
      }
      dtoEntry_.hasMatch = true;
      dtoEntry_.autofill = leg.id;
      return;
    }
  }
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
  // NRST list: Direct-To pre-fills with the highlighted facility.
  if (const MapFeature* sel = nrstSelectedFeature()) {
    dtoEntry_.open(navSource_, mapData_, sel->id);
    dtoEntry_.match = *sel;
    dtoEntry_.hasMatch = true;
    dtoEntry_.autofill = sel->id;
    return;
  }
  // Default destination (Pilot's Guide: the field defaults to the active
  // waypoint, or the highlighted flight-plan waypoint when one is selected).
  std::string initial;
  if (pageGroup_ == MfdPageGroup::FlightPlan) {
    FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                      fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                      &fplApproachHeaderLabel_};
    edit.directToActive = fplNavDirectToActive_;
    edit.localDraft = fplLocalDraft_;
    const std::string approachAirport = fplApproachAirportIcao();
    if (fplCursorOnHoldRow(edit, approachAirport, FplCursorLayout::SectionRows)) {
      openHoldActivatePrompt(fplCursorLegIndex());
      dtoOpen_ = false;
      return;
    }
    // Direct-To cannot target a synthetic departure row (RWxx/<alt>FT/MANSEQ);
    // resolve to the next real fix in the plan so the field pre-fills with it.
    const int legIdx = nextNavigableFixLegIndex(fplLegs_, fplCursorLegIndex());
    if (legIdx >= 0 && legIdx < static_cast<int>(fplLegs_.size())) {
      dtoPreserveLegIndex_ = legIdx;
      dtoPreserveFplCursorRow_ = fplCursorRow_;
      dtoPreservePlan_ = true;
      initial = fplLegs_[static_cast<std::size_t>(legIdx)].id;
    }
  }
  if (initial.empty() && !fplListCursorFollowsActive_ && !fplLegs_.empty()) {
    const int legIdx = nextNavigableFixLegIndex(fplLegs_, fplCursorLegIndex());
    if (legIdx >= 0 && legIdx < static_cast<int>(fplLegs_.size())) {
      dtoPreserveLegIndex_ = legIdx;
      dtoPreserveFplCursorRow_ = fplCursorRow_;
      dtoPreservePlan_ = true;
      initial = fplLegs_[static_cast<std::size_t>(legIdx)].id;
    }
  }
  if (initial.empty() && !activeWaypoint_.empty() &&
      fplListCursorFollowsActive_) {
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

  // Armed: ACTIVATE? is highlighted; ENT engages the direct course.
  if (dtoArmed_) {
    if (isMapRangePanBezelKey(key)) return false;
    if (key == BezelKey::Ent) {
      dtoRequestTarget_ = resolveDirectToTargetLeg(
          dtoEntry_, dtoPreservePlan_, dtoPreserveLegIndex_, fplLegs_);
      dtoRequestHold_ = false;
      dtoRequestPending_ = true;
      if (!dtoPreservePlan_) {
        fplLegs_.clear();
        fplDestinationFilled_ = false;
        fplLocalDraft_ = false;
        fplCursorRow_ = 0;
        fplApproachLegStart_ = 0;
        fplApproachLegCount_ = 0;
        fplLoadedApproach_ = {};
        fplApproachHeaderLabel_.clear();
      } else if (dtoPreserveFplCursorRow_ >= 0) {
        fplCursorRow_ = dtoPreserveFplCursorRow_;
      }
      dtoPreservePlan_ = false;
      dtoPreserveLegIndex_ = -1;
      dtoPreserveFplCursorRow_ = -1;
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

bool MfdController::consumeDirectToRequest(MapLeg& out, bool* flyHold) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  if (flyHold != nullptr) *flyHold = dtoRequestHold_;
  dtoRequestHold_ = false;
  return true;
}

void MfdController::applyDirectToInsetToDataSource(DataSource& source,
                                                   const MapData& map) {
  // Direct-To and FPL waypoint-entry popups both draw an inset map centered on
  // the matched target using insetLandLines from the data source.
  const MapFeature* wpt = nullptr;
  if (dtoOpen_ && dtoEntry_.hasMatch) {
    wpt = &dtoEntry_.match;
  } else if (fplEntry_.active && fplEntry_.hasMatch) {
    wpt = &fplEntry_.match;
  }
  MapFeature geo;
  if (wpt != nullptr) geo = mfd::resolveWaypointGeo(map, *wpt);
  if (wpt == nullptr || !mfd::mapFeatureHasGeo(geo)) {
    // Only relinquish the shared inset query if this controller currently owns
    // it. In a multi-display shell the PFD and MFD run separate engines off one
    // data source, and the PFD engine's dormant controller would otherwise
    // clear the inset the MFD's Direct-To/FPL popup just requested -- leaving
    // the popup map as blank ocean.
    if (insetQueryOwned_) {
      source.setInsetMapQuery(false, 0.0, 0.0, 0.0f, 0.0f);
      insetQueryOwned_ = false;
    }
    return;
  }
  const mfd::DirectToInsetView dv = mfd::directToInsetView(map, geo);
  const float halfExtent = mfd::directToInsetViewHalfExtentNm(dv.rangeNm);
  source.setInsetMapQuery(true, dv.centerLat, dv.centerLon, dv.rangeNm,
                          halfExtent, geo.id);
  insetQueryOwned_ = true;
}

}  // namespace avionics
