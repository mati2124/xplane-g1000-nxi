#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/SoftkeyController.h"
#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"

// Active Flight Plan window editing (FPL bezel key, Pilot's Guide Fig. 5-48
// "Active Flight Plan Window on PFD"). Mirrors the MFD FPL page's insert/remove
// behavior so the PFD can build and change the active route (the origin and
// destination are simply the first and last rows of the editable leg list),
// minus the VNAV ALT column the PFD window does not show.
namespace avionics {

void SoftkeyController::syncFlightPlanLegs(const MapData& map) {
  const bool mapChanged = !flightPlanLegsEqual(map.flightPlan, fplLastMapPlan_);
  if (mapChanged) {
    fplLastMapPlan_ = map.flightPlan;
  }

  // GPS Direct-To keeps the FPL editor template blank (matches the MFD).
  if (map.directToActive && !fplEditPending_ && !fplLocalDraft_) {
    if (!fplLegs_.empty() || fplDestinationFilled_ || fplApproachLegCount_ > 0) {
      fplLegs_.clear();
      fplDestinationFilled_ = false;
      fplApproachLegStart_ = 0;
      fplApproachLegCount_ = 0;
      fplLoadedApproach_ = {};
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplConfirm_ = FplConfirm::None;
    }
  }

  // Keep the working copy aligned with the shared map plan unless a local edit
  // is waiting to publish. Adopt even when the map echoes our own publication
  // so a stale pre-pump sync cannot leave fplLegs_ behind the other GDU.
  if (!fplEditPending_ && !fplLocalDraft_ && !map.directToActive &&
      !flightPlanLegsEqual(fplLegs_, map.flightPlan)) {
    const MapProcedure savedApproach = fplLoadedApproach_;
    std::vector<MapLeg> adopted = map.flightPlan;
    preserveFlightPlanIdents(adopted, fplLastPublished_);
    fplLegs_ = std::move(adopted);
    fplDestinationFilled_ = fplDestinationFilledForDisplay(
        static_cast<int>(fplLegs_.size()), map.directToActive);
    fplApproachLegStart_ = 0;
    fplApproachLegCount_ = 0;
    fplLoadedApproach_ = {};
    tryRestorePersistedApproach();
    if (fplApproachLegCount_ <= 0) {
      reinferApproachFromProcedureLegs();
    }
    if (fplLoadedApproach_.name.empty() && !savedApproach.name.empty() &&
        fplApproachLegCount_ > 0) {
      fplLoadedApproach_ = savedApproach;
    }
    // The rows our interaction referenced may be gone; back out of any entry
    // or confirmation rather than act on the wrong waypoint.
    fplEntry_.active = false;
    fplEntry_.notFound = false;
    fplConfirm_ = FplConfirm::None;
  }
  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    nullptr};
  edit.directToActive = mapDirectToActive();
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
}

bool SoftkeyController::consumeFlightPlanEdit(std::vector<MapLeg>& out) {
  if (!fplEditPending_) return false;
  fplEditPending_ = false;
  out = fplLegs_;
  fplLastPublished_ = fplLegs_;
  return true;
}

void SoftkeyController::flightPlanPublishEdit() {
  fplEditPending_ = true;
  fplLocalDraft_ = !fplLegs_.empty();
}

std::string SoftkeyController::flightPlanSelectedLegIdent() const {
  if (window_ != PfdWindow::FlightPlan || !fplCursorOn_) return {};
  FplRouteEdit edit{
      const_cast<std::vector<MapLeg>&>(fplLegs_),
      const_cast<bool&>(fplDestinationFilled_),
      const_cast<int&>(fplApproachLegStart_),
      const_cast<int&>(fplApproachLegCount_),
      const_cast<int&>(fplCursorRow_),
      nullptr,
      nullptr};
  edit.directToActive = mapDirectToActive();
  const int legIndex = fplCursorLegIndex(edit, flightPlanApproachAirportIcao(),
                                         FplCursorLayout::SectionRows);
  if (legIndex < 0 || legIndex >= static_cast<int>(fplLegs_.size())) return {};
  return fplLegs_[static_cast<std::size_t>(legIndex)].id;
}

int SoftkeyController::flightPlanSelectedLegIndex() const {
  if (window_ != PfdWindow::FlightPlan || !fplCursorOn_) return -1;
  FplRouteEdit edit{
      const_cast<std::vector<MapLeg>&>(fplLegs_),
      const_cast<bool&>(fplDestinationFilled_),
      const_cast<int&>(fplApproachLegStart_),
      const_cast<int&>(fplApproachLegCount_),
      const_cast<int&>(fplCursorRow_),
      nullptr,
      nullptr};
  edit.directToActive = mapDirectToActive();
  return fplCursorLegIndex(edit, flightPlanApproachAirportIcao(),
                           FplCursorLayout::SectionRows);
}

void SoftkeyController::flightPlanApplyDirectTo(const MapLeg& target) {
  (void)target;
  // Active Direct-To is navigation only; the FPL editor template stays blank.
  fplLegs_.clear();
  fplDestinationFilled_ = false;
  fplLocalDraft_ = false;
  fplCursorRow_ = 0;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
}

void SoftkeyController::flightPlanCommitEntry() {
  if (fplEntry_.chars.empty()) {
    fplEntry_.active = false;
    return;
  }
  if (!fplEntry_.hasMatch) {
    fplEntry_.notFound = true;
    return;
  }

  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    nullptr};
  edit.directToActive = mapDirectToActive();
  const std::string ident =
      fplEntry_.autofill.empty() ? fplEntry_.chars : fplEntry_.autofill;
  if (!fplCommitWaypointIdent(edit, navSource_, fplEntry_.match, ident, fplCursorRow_,
                              flightPlanApproachAirportIcao(),
                              FplCursorLayout::SectionRows)) {
    return;
  }
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
  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    nullptr};
  edit.directToActive = mapDirectToActive();
  const std::string approachAirport = flightPlanApproachAirportIcao();
  const int selectableLast =
      fplCursorSelectableLast(edit, approachAirport, FplCursorLayout::SectionRows);

  // Modal confirmation (Remove <wpt>? / Delete Flight Plan?). ENT executes the
  // highlighted choice, CLR / knob push cancels, the knob toggles OK / CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          if (fplConfirm_ == FplConfirm::RemoveWaypoint) {
            const int legIndex = fplCursorLegIndex(
                edit, approachAirport, FplCursorLayout::SectionRows);
            if (fplRemoveLegAtIndex(edit, legIndex)) {
              flightPlanPublishEdit();
            }
          } else {
            fplClearFlightPlan(edit);
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
    fplClampCursorRow(edit, approachAirport, FplCursorLayout::SectionRows);
    return true;
  }

  if (!fplCursorOn_) {
    // Cursor off: the knob scrolls the section list; other keys fall through.
    if (key == BezelKey::FmsOuterCw || key == BezelKey::FmsInnerCw) {
      fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
      return true;
    }
    if (key == BezelKey::FmsOuterCcw || key == BezelKey::FmsInnerCcw) {
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    }
    return false;
  }

  // Cursor on: the large knob moves between section rows, the small knob opens
  // the insert entry, CLR removes the highlighted waypoint.
  switch (key) {
    case BezelKey::FmsOuterCw:
      fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
      return true;
    case BezelKey::FmsOuterCcw:
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      if (!fplEntry_.active) {
        std::string initial = flightPlanSelectedLegIdent();
        if (!initial.empty() && isFmsLatLonIdent(initial)) initial.clear();
        fplEntry_.open(navSource_, mapData_, initial);
        fplEntry_.selectAll = false;
      }
      fplEntry_.turnChar(navSource_, mapData_,
                         key == BezelKey::FmsInnerCw ? +1 : -1);
      return true;
    }
    case BezelKey::Clr: {
      const int legIndex =
          fplCursorLegIndex(edit, approachAirport, FplCursorLayout::SectionRows);
      if (legIndex >= 0 && legIndex < legCount) {
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(legIndex)].id;
      }
      return true;
    }
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

void SoftkeyController::setPersistedLoadedApproach(
    const PersistedLoadedApproach& saved) {
  persistedApproachRestore_ = saved;
  tryRestorePersistedApproach();
}

PersistedLoadedApproach SoftkeyController::persistedLoadedApproachSnapshot()
    const {
  if (fplApproachLegCount_ <= 0 || fplLoadedApproach_.name.empty()) {
    return PersistedLoadedApproach{};
  }
  return persistedFromMapProcedure(fplLoadedApproach_,
                                   flightPlanApproachAirportIcao());
}

void SoftkeyController::tryRestorePersistedApproach() {
  if (!persistedApproachRestore_.active ||
      persistedApproachRestore_.name.empty()) {
    return;
  }
  if (navSource_ == nullptr || !navSource_->ready() || fplLegs_.empty()) {
    return;
  }
  const std::string icao = persistedApproachRestore_.airportIcao;
  if (icao.empty()) return;

  const std::vector<MapLeg> expanded =
      navSource_->expandProcedure(icao, persistedApproachRestore_.type,
                                  persistedApproachRestore_.name,
                                  persistedApproachRestore_.transition);
  if (expanded.empty()) return;

  int start = 0;
  if (!findLegSequenceInPlan(fplLegs_, expanded, start)) return;

  fplApproachLegStart_ = start;
  fplApproachLegCount_ = static_cast<int>(expanded.size());
  fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
  mergeProcedureLegMetadata(fplLegs_, start, expanded);
}

void SoftkeyController::reinferApproachFromProcedureLegs() {
  const InferredProcedureBlock block = inferProcedureBlockInPlan(fplLegs_);
  if (!block.valid()) return;
  fplApproachLegStart_ = block.start;
  fplApproachLegCount_ = block.count;
  if (fplLoadedApproach_.name.empty() && persistedApproachRestore_.active) {
    fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
  }
}

FlightPlanApproachState SoftkeyController::flightPlanApproachState() const {
  FlightPlanApproachState out;
  out.legStart = fplApproachLegStart_;
  out.legCount = fplApproachLegCount_;
  out.loaded = fplLoadedApproach_;
  out.headerLabel = flightPlanApproachHeaderLabel();
  return out;
}

void SoftkeyController::applyFlightPlanApproachState(
    const FlightPlanApproachState& state) {
  if (state.legCount <= 0) {
    fplApproachLegStart_ = 0;
    fplApproachLegCount_ = 0;
    fplLoadedApproach_ = {};
    return;
  }
  fplApproachLegStart_ = state.legStart;
  fplApproachLegCount_ = state.legCount;
  fplLoadedApproach_ = state.loaded;
}

PersistedFlightPlan SoftkeyController::persistedFlightPlanSnapshot() const {
  PersistedFlightPlan out;
  if (fplLegs_.empty()) return out;
  out.active = true;
  out.destinationFilled = fplDestinationFilled_;
  out.legs = fplLegs_;
  return out;
}

void SoftkeyController::restorePersistedFlightPlan(
    const PersistedFlightPlan& saved) {
  if (!saved.active || saved.legs.empty()) return;
  fplLegs_ = saved.legs;
  fplDestinationFilled_ = saved.destinationFilled;
  fplLocalDraft_ = true;
  fplEditPending_ = false;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
  fplCursorRow_ = 0;
}

void SoftkeyController::replaceFlightPlanFromExternal(
    const std::vector<MapLeg>& plan) {
  fplLegs_ = plan;
  const int n = static_cast<int>(fplLegs_.size());
  fplDestinationFilled_ = n >= 3 || n == 2;
  fplLocalDraft_ = false;
  fplEditPending_ = false;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
  fplLastPublished_ = plan;
  fplLastMapPlan_ = plan;
}

}  // namespace avionics
