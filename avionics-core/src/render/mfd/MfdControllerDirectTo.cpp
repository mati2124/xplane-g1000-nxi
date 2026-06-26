#include "avionics/MfdController.h"

#include "avionics/DataSource.h"
#include "render/mfd/MfdPageSupport.h"

// Direct-To window (Direct-To bezel key, Pilot's Guide 5.5): opens over any MFD
// page pre-filled with the active (or FPL-selected) waypoint; the first ENT
// confirms the waypoint and arms ACTIVATE?, the second engages the direct course.
namespace avionics {

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
  const int legIdx = fplCursorLegIndex();
  if (pageGroup_ == MfdPageGroup::FlightPlan && legIdx >= 0 &&
      legIdx < static_cast<int>(fplLegs_.size())) {
    dtoPreserveLegIndex_ = legIdx;
    dtoPreserveFplCursorRow_ = fplCursorRow_;
    dtoPreservePlan_ = true;
    initial = fplLegs_[static_cast<std::size_t>(legIdx)].id;
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
    if (isMapRangePanBezelKey(key)) return false;
    if (key == BezelKey::Ent) {
      if (dtoPreservePlan_ && dtoPreserveLegIndex_ >= 0 &&
          dtoPreserveLegIndex_ < static_cast<int>(fplLegs_.size())) {
        dtoRequestTarget_ =
            fplLegs_[static_cast<std::size_t>(dtoPreserveLegIndex_)];
      } else {
        dtoRequestTarget_.lat = dtoEntry_.match.lat;
        dtoRequestTarget_.lon = dtoEntry_.match.lon;
        dtoRequestTarget_.id = dtoEntry_.match.id;
      }
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

bool MfdController::consumeDirectToRequest(MapLeg& out) {
  if (!dtoRequestPending_) return false;
  dtoRequestPending_ = false;
  out = dtoRequestTarget_;
  return true;
}

void MfdController::applyDirectToInsetToDataSource(DataSource& source,
                                                   const MapData& map) {
  if (!dtoOpen_ || !dtoEntry_.hasMatch) {
    source.setInsetMapQuery(false, 0.0, 0.0, 0.0f, 0.0f);
    return;
  }
  const MapFeature& wpt = dtoEntry_.match;
  if (wpt.lat == 0.0 && wpt.lon == 0.0) {
    source.setInsetMapQuery(false, 0.0, 0.0, 0.0f, 0.0f);
    return;
  }
  const float rangeNm = mfd::directToInsetRangeNm(map, wpt);
  const float halfExtent = mfd::directToInsetViewHalfExtentNm(rangeNm);
  source.setInsetMapQuery(true, wpt.lat, wpt.lon, rangeNm, halfExtent, wpt.id);
}

}  // namespace avionics
