#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/MfdController.h"

// Active Flight Plan page (FPL group, Pilot's Guide 5.6): caches the map plan
// plus local edits, the leg-row cursor, the Waypoint Information insert window,
// the VNAV altitude-constraint entry, and the remove/delete confirmations.
namespace avionics {

void MfdController::fplEnsureApproachInferred() {
  if (fplApproachLegCount_ > 0) {
    FlightPlanApproachState state;
    state.legStart = fplApproachLegStart_;
    state.legCount = fplApproachLegCount_;
    if (approachStateFitsPlan(state, fplLegs_)) return;
  }
  reinferApproachFromProcedureLegs();
}

MfdController::FplEffectiveApproach MfdController::fplEffectiveApproach() const {
  FplEffectiveApproach out;
  out.start = fplApproachLegStart_;
  out.count = fplApproachLegCount_;

  const auto inferFromLegs = [&]() {
    const InferredProcedureBlock block = inferProcedureBlockInPlan(fplLegs_);
    if (block.valid()) {
      out.start = block.start;
      out.count = block.count;
    } else {
      out.start = 0;
      out.count = 0;
    }
  };

  // Match the PFD Active Flight Plan window: infer the approach tail from
  // procedureRole tags when stored grouping is missing or stale.
  if (out.count <= 0) {
    inferFromLegs();
  } else {
    FlightPlanApproachState stored;
    stored.legStart = out.start;
    stored.legCount = out.count;
    if (!approachStateFitsPlan(stored, fplLegs_)) {
      inferFromLegs();
    }
  }

  if (out.count > 0 && out.start >= 0 &&
      out.start + out.count <
          static_cast<int>(fplLegs_.size())) {
    out.count = fplNormalizedApproachCount(
        out.start, out.count, static_cast<int>(fplLegs_.size()));
  }

  FlightPlanApproachState state;
  state.legStart = out.start;
  state.legCount = out.count;
  if (!approachStateFitsPlan(state, fplLegs_)) {
    out.start = 0;
    out.count = 0;
  }
  return out;
}

int MfdController::fplCursorLegIndex() const {
  FplRouteEdit edit{
      const_cast<std::vector<MapLeg>&>(fplLegs_),
      const_cast<bool&>(fplDestinationFilled_),
      const_cast<int&>(fplApproachLegStart_),
      const_cast<int&>(fplApproachLegCount_),
      const_cast<int&>(fplCursorRow_),
      const_cast<MapProcedure*>(&fplLoadedApproach_),
      const_cast<std::string*>(&fplApproachHeaderLabel_)};
  edit.directToActive = fplNavDirectToActive_;
  edit.localDraft = fplLocalDraft_;
  const FplCursorLayout layout = FplCursorLayout::SectionRows;
  return ::avionics::fplCursorLegIndex(edit, fplApproachAirportIcao(), layout);
}

void MfdController::fplClampCursorRow() {
  fplEnsureApproachInferred();
  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    &fplApproachHeaderLabel_};
  edit.directToActive = fplNavDirectToActive_;
  edit.localDraft = fplLocalDraft_;
  ::avionics::fplClampCursorRow(edit, fplApproachAirportIcao(),
                                FplCursorLayout::SectionRows);
  fplCursorCol_ = FplCursorCol::Ident;
}

void MfdController::syncFlightPlan(const MapData& map,
                                   const std::string& activeWaypoint,
                                   bool navDirectTo) {
  if (fplApproachRestorePending_ && navSource_ != nullptr &&
      navSource_->ready()) {
    if (persistedApproachRestore_.airportIcao.empty() &&
        fplApproachLegCount_ > 0) {
      const std::string icao = inferApproachAirportFromProcedureLegs(
          navSource_, fplLegs_, fplApproachLegStart_, fplApproachLegCount_);
      if (!icao.empty()) {
        persistedApproachRestore_.airportIcao = icao;
        persistedApproachRestore_.active = true;
      }
    }
    tryRestorePersistedApproach();
  }

  mapData_ = &map;
  activeWaypoint_ = activeWaypoint;
  fplNavDirectToActive_ = navDirectTo;

  // GPS Direct-To keeps the FPL editor template blank unless a procedure is loaded
  // on the map — then mirror the map plan so approach legs stay visible.
  if ((navDirectTo || map.directToActive) && !fplLocalDraft_ &&
      !fplEditPending_) {
    const MapProcedure savedApproach = fplLoadedApproach_;
    const std::string savedHeader = fplApproachHeaderLabel_;
    FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                      fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                      &fplApproachHeaderLabel_};
    if (fplAdoptMapPlanDuringDirectTo(edit, map.flightPlan, fplLastPublished_)) {
      tryRestorePersistedApproach();
      if (fplApproachLegCount_ <= 0) {
        reinferApproachFromProcedureLegs();
      }
      if (fplLoadedApproach_.name.empty() && !savedApproach.name.empty() &&
          fplApproachLegCount_ > 0) {
        fplLoadedApproach_ = savedApproach;
        if (fplApproachHeaderLabel_.empty() && !savedHeader.empty()) {
          fplApproachHeaderLabel_ = savedHeader;
        }
      }
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplAltEntry_.active = false;
      fplConfirm_ = FplConfirm::None;
      fplMenuOpen_ = false;
    } else {
      const bool keepProcedure =
          fplApproachLegCount_ > 0 ||
          inferProcedureBlockInPlan(fplLegs_).valid() ||
          !fplLoadedApproach_.name.empty();
      if (keepProcedure) {
        if (fplApproachLegCount_ <= 0) {
          reinferApproachFromProcedureLegs();
        }
        if (fplLoadedApproach_.name.empty() && !savedApproach.name.empty() &&
            fplApproachLegCount_ > 0) {
          fplLoadedApproach_ = savedApproach;
          if (fplApproachHeaderLabel_.empty() && !savedHeader.empty()) {
            fplApproachHeaderLabel_ = savedHeader;
          }
        }
      } else if (!fplLegs_.empty() || fplDestinationFilled_ ||
                 fplApproachLegCount_ > 0) {
        fplLegs_.clear();
        fplDestinationFilled_ = false;
        fplApproachLegStart_ = 0;
        fplApproachLegCount_ = 0;
        fplLoadedApproach_ = {};
        fplApproachHeaderLabel_.clear();
        fplEntry_.active = false;
        fplEntry_.notFound = false;
        fplAltEntry_.active = false;
        fplConfirm_ = FplConfirm::None;
        fplMenuOpen_ = false;
      }
    }
  }

  const bool mapChanged = !flightPlanLegsEqual(map.flightPlan, fplLastMapPlan_);
  if (mapChanged) {
    fplLastMapPlan_ = map.flightPlan;
  }
  // Keep the FPL page aligned with the shared map plan unless a local edit is
  // waiting to publish. Adopt even when the map echoes our own publication so
  // the PFD and MFD stay in step after either side edits the route.
  if (!fplEditPending_ && !fplLocalDraft_ && !navDirectTo &&
      !map.directToActive &&
      !flightPlanLegsEqual(fplLegs_, map.flightPlan)) {
    const MapProcedure savedApproach = fplLoadedApproach_;
    const std::string savedHeader = fplApproachHeaderLabel_;
    std::vector<MapLeg> adopted = map.flightPlan;
    preserveFlightPlanIdents(adopted, fplLastPublished_);
    fplLegs_ = std::move(adopted);
    fplDestinationFilled_ =
        fplDestinationFilledForDisplay(static_cast<int>(fplLegs_.size()),
                                       navDirectTo);
    fplApproachLegStart_ = 0;
    fplApproachLegCount_ = 0;
    fplLoadedApproach_ = {};
    fplApproachHeaderLabel_.clear();
    tryRestorePersistedApproach();
    if (fplApproachLegCount_ <= 0) {
      reinferApproachFromProcedureLegs();
    }
    if (fplLoadedApproach_.name.empty() && !savedApproach.name.empty() &&
        fplApproachLegCount_ > 0) {
      fplLoadedApproach_ = savedApproach;
      if (fplApproachHeaderLabel_.empty() && !savedHeader.empty()) {
        fplApproachHeaderLabel_ = savedHeader;
      }
    }
    // The rows the interaction state referenced are gone; close the entry
    // and confirmation windows rather than acting on the wrong waypoint.
    fplEntry_.active = false;
    fplEntry_.notFound = false;
    fplAltEntry_.active = false;
    fplConfirm_ = FplConfirm::None;
    fplMenuOpen_ = false;
  }

  fplEnsureApproachInferred();
  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    &fplApproachHeaderLabel_};
  edit.directToActive = fplNavDirectToActive_;
  edit.localDraft = fplLocalDraft_;
  fplSyncListCursorToActiveLeg(edit, fplApproachAirportIcao(), activeWaypoint_,
                               fplListCursorFollowsActive_);
  fplClampCursorRow();
}

bool MfdController::consumeFlightPlanEdit(std::vector<MapLeg>& out) {
  if (!fplEditPending_) return false;
  fplEditPending_ = false;
  out = fplLegs_;
  fplLastPublished_ = fplLegs_;
  return true;
}

void MfdController::fplPublishEdit() {
  fplEditPending_ = true;
  // Any pending edit (including a deliberate delete to empty) owns the plan
  // until it is consumed so sync cannot re-adopt a stale sim route first.
  fplLocalDraft_ = true;
}

void MfdController::fplResetInteraction() {
  fplCursorOn_ = false;
  fplListCursorFollowsActive_ = true;
  fplClampCursorRow();
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
  fplEnsureApproachInferred();
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
                    &fplApproachHeaderLabel_};
  edit.directToActive = fplNavDirectToActive_;
  edit.localDraft = fplLocalDraft_;
  const FplCursorLayout layout = FplCursorLayout::SectionRows;
  const std::string ident =
      fplEntry_.autofill.empty() ? fplEntry_.chars : fplEntry_.autofill;
  if (!::avionics::fplCommitWaypointIdent(edit, navSource_, fplEntry_.match, ident,
                                          fplCursorRow_, fplApproachAirportIcao(),
                                          layout)) {
    return;
  }
  fplEntry_.active = false;
  fplEntry_.notFound = false;
  fplPublishEdit();
}

bool MfdController::fplBezelKey(BezelKey key) {
  fplEnsureApproachInferred();
  const int legCount = static_cast<int>(fplLegs_.size());
  FplRouteEdit edit{fplLegs_,           fplDestinationFilled_, fplApproachLegStart_,
                    fplApproachLegCount_, fplCursorRow_,         &fplLoadedApproach_,
                    &fplApproachHeaderLabel_};
  edit.directToActive = fplNavDirectToActive_;
  edit.localDraft = fplLocalDraft_;
  const std::string approachAirport = fplApproachAirportIcao();
  const FplCursorLayout layout = FplCursorLayout::SectionRows;
  const bool approachView = fplApproachLegCount_ > 0;
  const int selectableLast =
      ::avionics::fplCursorSelectableLast(edit, approachAirport, layout);

  // GCU / GDU range still zooms the map (Pilot's Guide); fall through to the
  // common range handling in pressBezelKey.
  if (isMapRangePanBezelKey(key)) return false;

  // The confirmation window is modal: ENT executes the highlighted choice,
  // CLR (or pushing the FMS knob) cancels, the knob toggles OK/CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          if (fplConfirm_ == FplConfirm::RemoveWaypoint) {
            const int legIndex =
                ::avionics::fplCursorLegIndex(edit, approachAirport, layout);
            if (::avionics::fplRemoveLegAtIndex(edit, legIndex)) {
              fplPublishEdit();
            }
          } else {
            ::avionics::fplClearFlightPlan(edit);
            persistedApproachRestore_ = {};
            dtoRequestTarget_ = {};
            dtoRequestPending_ = true;
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
    fplClampCursorRow();
    return true;
  }

  if (!fplCursorOn_) {
    // Cursor off: the knob scrolls the section list (PFD FPL window behavior).
    if (key == BezelKey::FmsOuterCw || key == BezelKey::FmsInnerCw) {
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
      return true;
    }
    if (key == BezelKey::FmsOuterCcw || key == BezelKey::FmsInnerCcw) {
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    }
    return false;  // other keys fall through to page stepping
  }

  const int legIndex = fplCursorLegIndex();
  const bool onLegRow = legIndex >= 0 && legIndex < legCount;
  const bool onAltCol =
      onLegRow && fplCursorCol_ == FplCursorCol::Altitude;

  switch (key) {
    case BezelKey::FmsOuterCw:
      // Blank section template: step rows like the PFD FPL window. Filled legs
      // also expose the VNAV ALT column on the large knob.
      if (approachView) {
        if (onLegRow && fplCursorCol_ == FplCursorCol::Ident) {
          fplCursorCol_ = FplCursorCol::Altitude;
        } else {
          fplListCursorFollowsActive_ = false;
          fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
          fplCursorCol_ = FplCursorCol::Ident;
        }
      } else if (onLegRow && fplCursorCol_ == FplCursorCol::Ident) {
        fplCursorCol_ = FplCursorCol::Altitude;
      } else {
        fplListCursorFollowsActive_ = false;
        fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
        fplCursorCol_ = FplCursorCol::Ident;
      }
      return true;
    case BezelKey::FmsOuterCcw:
      if (onAltCol) {
        fplCursorCol_ = FplCursorCol::Ident;
      } else if (fplCursorRow_ > 0) {
        fplListCursorFollowsActive_ = false;
        fplCursorRow_ -= 1;
        const int prevLeg = fplCursorLegIndex();
        fplCursorCol_ =
            (prevLeg >= 0 && prevLeg < legCount) ? FplCursorCol::Altitude
                                                 : FplCursorCol::Ident;
      }
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      if (onAltCol) {
        fplAltEntryOpen(legIndex);
      } else {
        if (!fplEntry_.active) {
          std::string initial;
          if (onLegRow) {
            initial = fplLegs_[static_cast<std::size_t>(legIndex)].id;
            if (!initial.empty() && isFmsLatLonIdent(initial)) initial.clear();
          }
          fplEntry_.open(navSource_, mapData_, initial);
          fplEntry_.selectAll = false;
        }
        fplEntry_.turnChar(navSource_, mapData_,
                           key == BezelKey::FmsInnerCw ? +1 : -1);
      }
      return true;
    case BezelKey::Clr:
      if (onAltCol) {
        MapLeg& leg = fplLegs_[static_cast<std::size_t>(legIndex)];
        if (leg.altitudeConstraint != AltConstraintType::None) {
          leg.altitudeConstraintFt = 0;
          leg.altitudeConstraint = AltConstraintType::None;
          leg.altitudeDesignated = false;
          fplPublishEdit();
        }
      } else if (onLegRow) {
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(legIndex)].id;
      }
      return true;
    case BezelKey::Ent:
      if (onLegRow) {
        requestActivateFlightPlanLeg(legIndex);
      }
      return true;
    default:
      return false;
  }
}

void MfdController::requestActivateFlightPlanLeg(int toLegIndex) {
  if (toLegIndex < 0 || toLegIndex >= static_cast<int>(fplLegs_.size())) return;
  dtoRequestPending_ = false;
  fplActivateLegIndex_ = toLegIndex;
  fplActivateLegPending_ = true;
}

bool MfdController::consumeActivateLegRequest(int& toLegIndex) {
  if (!fplActivateLegPending_) return false;
  if (fplEditPending_) return false;
  fplActivateLegPending_ = false;
  toLegIndex = fplActivateLegIndex_;
  return toLegIndex >= 0;
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

void MfdController::setPersistedLoadedApproach(
    const PersistedLoadedApproach& saved) {
  persistedApproachRestore_ = saved;
  if (saved.active && !saved.name.empty()) {
    fplApproachRestorePending_ = true;
  }
  tryRestorePersistedApproach();
}

FlightPlanApproachState MfdController::flightPlanApproachState() const {
  FlightPlanApproachState out;
  out.legStart = fplApproachLegStart_;
  out.legCount = fplApproachLegCount_;
  out.loaded = fplLoadedApproach_;
  out.headerLabel = fplApproachHeaderLabel_;
  return out;
}

void MfdController::applyFlightPlanApproachState(
    const FlightPlanApproachState& state) {
  if (state.legCount <= 0) {
    fplApproachLegStart_ = 0;
    fplApproachLegCount_ = 0;
    fplLoadedApproach_ = {};
    fplApproachHeaderLabel_.clear();
    return;
  }
  fplApproachLegStart_ = state.legStart;
  fplApproachLegCount_ = state.legCount;
  fplLoadedApproach_ = state.loaded;
  fplApproachHeaderLabel_ = state.headerLabel;
}

void MfdController::adoptFlightPlanFromPeer(
    const std::vector<MapLeg>& legs, bool destinationFilled,
    const FlightPlanApproachState& approach) {
  if (fplEntry_.active || fplAltEntry_.active ||
      fplConfirm_ != FplConfirm::None || fplMenuOpen_) {
    return;
  }
  if (flightPlanLegsEqual(fplLegs_, legs) &&
      fplDestinationFilled_ == destinationFilled &&
      flightPlanApproachState() == approach) {
    return;
  }
  fplLegs_ = legs;
  fplDestinationFilled_ = destinationFilled;
  // Mirror the PFD Active Flight Plan window: a non-empty peer route is a local
  // draft on this page too, including during GPS Direct-To (otherwise the MFD
  // keeps drawing the blank Direct-To template even though legs were copied).
  fplLocalDraft_ = !fplLegs_.empty();
  applyFlightPlanApproachState(approach);
  fplEntry_.active = false;
  fplEntry_.notFound = false;
  fplAltEntry_.active = false;
  fplConfirm_ = FplConfirm::None;
  fplMenuOpen_ = false;
  fplClampCursorRow();
}

std::string MfdController::fplApproachAirportIcao() const {
  if (fplApproachLegCount_ <= 0) return {};
  const std::string loadedIcao =
      persistedApproachRestore_.active ? persistedApproachRestore_.airportIcao
                                       : std::string();
  return ::avionics::fplApproachAirportIcao(fplLegs_, fplApproachLegStart_, mapData_,
                                            loadedIcao);
}

void MfdController::tryRestorePersistedApproach() {
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
  if (!findLegSequenceInPlan(fplLegs_, expanded, start)) {
    // Saved legs don't contain this approach contiguously; stop retrying.
    fplApproachRestorePending_ = false;
    return;
  }

  const bool wasRestorePending = fplApproachRestorePending_;
  fplApproachLegStart_ = start;
  fplApproachLegCount_ = static_cast<int>(expanded.size());
  fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
  fplApproachHeaderLabel_ = formatApproachFplHeaderLabel(fplLoadedApproach_);
  mergeProcedureLegMetadata(fplLegs_, start, expanded);
  // Holds, altitude constraints, and glidepath are now re-attached.
  fplApproachRestorePending_ = false;
  // The restored route was pushed to the drawn map before these procedure
  // details existed (holds are not persisted per-leg); re-publish so the route
  // override and peer GDU pick up the re-attached holds.
  if (wasRestorePending) {
    fplPublishEdit();
  }
}

void MfdController::reinferApproachFromProcedureLegs() {
  const InferredProcedureBlock block = inferProcedureBlockInPlan(fplLegs_);
  if (!block.valid()) return;
  fplApproachLegStart_ = block.start;
  fplApproachLegCount_ = block.count;
  if (fplLoadedApproach_.name.empty() && persistedApproachRestore_.active) {
    fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
    fplApproachHeaderLabel_ = formatApproachFplHeaderLabel(fplLoadedApproach_);
  }
}

void MfdController::replaceFlightPlanFromExternal(
    const std::vector<MapLeg>& plan) {
  fplLegs_ = plan;
  const int n = static_cast<int>(fplLegs_.size());
  fplDestinationFilled_ = fplDestinationFilledFromLegCount(n);
  fplLocalDraft_ = false;
  fplEditPending_ = false;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
  fplApproachHeaderLabel_.clear();
  fplLastPublished_ = plan;
  fplLastMapPlan_ = plan;
}

PersistedFlightPlan MfdController::persistedFlightPlanSnapshot() const {
  PersistedFlightPlan out;
  if (fplLegs_.empty()) return out;
  out.active = true;
  out.destinationFilled = fplDestinationFilled_;
  out.legs = fplLegs_;
  if (fplApproachLegCount_ > 0) {
    out.approachLegStart = fplApproachLegStart_;
    out.approachLegCount = fplApproachLegCount_;
    out.approachAirportIcao = fplApproachAirportIcao();
    if (!fplLoadedApproach_.name.empty()) {
      out.approachMeta =
          persistedFromMapProcedure(fplLoadedApproach_, out.approachAirportIcao);
    } else if (persistedApproachRestore_.active) {
      out.approachMeta = persistedApproachRestore_;
    }
  }
  return out;
}

PersistedDirectTo MfdController::persistedDirectToSnapshot() const {
  if (mapData_ == nullptr) return {};
  return persistedDirectToFromMap(*mapData_);
}

void MfdController::restorePersistedFlightPlan(
    const PersistedFlightPlan& saved) {
  if (!saved.active || saved.legs.empty()) return;
  fplLegs_ = saved.legs;
  fplDestinationFilled_ = saved.destinationFilled;
  fplLocalDraft_ = true;
  fplEditPending_ = false;
  fplApproachLegStart_ = 0;
  fplApproachLegCount_ = 0;
  fplLoadedApproach_ = {};
  fplApproachHeaderLabel_.clear();
  fplCursorRow_ = 0;
  fplLastPublished_ = saved.legs;
  fplLastMapPlan_ = saved.legs;
  fplApproachRestorePending_ = false;
  if (saved.approachLegCount > 0) {
    fplApproachLegStart_ = saved.approachLegStart;
    fplApproachLegCount_ = saved.approachLegCount;
    persistedApproachRestore_ = saved.approachMeta;
    if (!saved.approachAirportIcao.empty()) {
      persistedApproachRestore_.airportIcao = saved.approachAirportIcao;
      persistedApproachRestore_.active = true;
    }
    if (!fplDestinationFilled_ && fplApproachLegCount_ > 0) {
      fplDestinationFilled_ = true;
    }
  }
  if (fplApproachLegCount_ <= 0) {
    reinferApproachFromProcedureLegs();
  }
  if (fplApproachLegCount_ > 0 && persistedApproachRestore_.name.empty()) {
    inferApproachMetadataFromLegs(fplLegs_, fplApproachLegStart_,
                                persistedApproachRestore_);
  }
  if (fplApproachLegCount_ > 0 && !fplLoadedApproach_.name.empty()) {
    fplApproachHeaderLabel_ = formatApproachFplHeaderLabel(fplLoadedApproach_);
  } else if (fplApproachLegCount_ > 0 && persistedApproachRestore_.active &&
             !persistedApproachRestore_.name.empty()) {
    fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
    fplApproachHeaderLabel_ = formatApproachFplHeaderLabel(fplLoadedApproach_);
  }
  // Re-expand the CIFP approach (holds, altitude constraints, and glidepath are
  // not persisted per-leg) so they are re-attached after a restart. Arm from the
  // now-final metadata; syncFlightPlan retries until nav data is ready.
  fplApproachRestorePending_ = fplApproachLegCount_ > 0 &&
                               persistedApproachRestore_.active &&
                               !persistedApproachRestore_.name.empty();
  tryRestorePersistedApproach();
}

}  // namespace avionics
