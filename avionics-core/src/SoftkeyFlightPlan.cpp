#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/ProcedureSupport.h"
#include "avionics/SoftkeyController.h"
#include "avionics/SimBriefOfpSupport.h"
#include "avionics/NavMath.h"
#include "avionics/render/BezelKeys.h"

// Active Flight Plan window editing (FPL bezel key, Pilot's Guide Fig. 5-48
// "Active Flight Plan Window on PFD"). Mirrors the MFD FPL page's insert/remove
// behavior so the PFD can build and change the active route (the origin and
// destination are simply the first and last rows of the editable leg list),
// minus the VNAV ALT column the PFD window does not show.
namespace avionics {
namespace {

void applyTerminalProcedureMeta(const PersistedLoadedApproach& meta, int legStart,
                                int legCount, MapProcedure& loaded,
                                std::string& headerLabel, int& outStart,
                                int& outCount) {
  if (!meta.active || meta.name.empty()) {
    loaded = {};
    headerLabel.clear();
    outStart = 0;
    outCount = 0;
    return;
  }
  outStart = legStart;
  outCount = legCount;
  loaded = mapProcedureFromPersisted(meta);
  headerLabel = formatTerminalProcedureFplHeaderLabel(
      loaded.runway, loaded.name, loaded.transition);
}

void clearTerminalProcedureState(MapProcedure& loaded, int& legStart,
                                 int& legCount, std::string& headerLabel,
                                 PersistedLoadedApproach& persisted) {
  loaded = {};
  legStart = 0;
  legCount = 0;
  headerLabel.clear();
  persisted = {};
}

}  // namespace

void SoftkeyController::syncFlightPlanLegs(const MapData& map, bool navDirectTo) {
  if (fplApproachRestorePending_ && navSource_ != nullptr &&
      navSource_->ready()) {
    // The destination airport may need inferring before the CIFP approach can be
    // re-expanded (older saved plans, or coordinate idents from the sim FMS).
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
  if ((fplDepartureRestorePending_ || fplArrivalRestorePending_) &&
      navSource_ != nullptr && navSource_->ready()) {
    tryRestorePersistedTerminalProcedures();
  }

  const bool mapChanged = !flightPlanLegsEqual(map.flightPlan, fplLastMapPlan_);
  if (mapChanged) {
    fplLastMapPlan_ = map.flightPlan;
  }

  if ((navDirectTo || map.directToActive) && !fplEditPending_ && !fplLocalDraft_) {
    const MapProcedure savedApproach = fplLoadedApproach_;
    FplRouteEdit edit = flightPlanRouteEditState();
    if (fplAdoptMapPlanDuringDirectTo(edit, map.flightPlan, fplLastPublished_)) {
      tryRestorePersistedApproach();
      if (fplApproachLegCount_ <= 0) {
        reinferApproachFromProcedureLegs();
      }
      if (fplLoadedApproach_.name.empty() && !savedApproach.name.empty() &&
          fplApproachLegCount_ > 0) {
        fplLoadedApproach_ = savedApproach;
      }
      fplEntry_.active = false;
      fplEntry_.notFound = false;
      fplConfirm_ = FplConfirm::None;
    } else {
      // map.flightPlan is cleared during display-only Direct-To; keep a loaded
      // approach in the FPL editor instead of wiping it.
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
        }
      } else if (!fplLegs_.empty() || fplDestinationFilled_) {
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
  }

  // Keep the working copy aligned with the shared map plan unless a local edit
  // is waiting to publish. Adopt even when the map echoes our own publication
  // so a stale pre-pump sync cannot leave fplLegs_ behind the other GDU.
  if (!fplEditPending_ && !fplLocalDraft_ && !navDirectTo &&
      !map.directToActive &&
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
  FplRouteEdit edit = flightPlanRouteEditState();
  fplSyncListCursorToActiveLeg(edit, flightPlanApproachAirportIcao(), activeWaypoint_,
                               fplListCursorFollowsActive_);
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
  // Any pending edit (including a deliberate delete to empty) owns the plan
  // until it is consumed so sync cannot re-adopt a stale sim route first.
  fplLocalDraft_ = true;
}

void SoftkeyController::stripCourseReversalHoldAtFix(const std::string& fixId) {
  if (fixId.empty()) return;
  for (MapLeg& leg : fplLegs_) {
    if (leg.id == fixId && leg.hold.courseReversal) {
      leg.hold = MapHoldPattern{};
    }
  }
}

FplRouteEdit SoftkeyController::flightPlanRouteEditState() const {
  auto* self = const_cast<SoftkeyController*>(this);
  FplRouteEdit edit{self->fplLegs_,           self->fplDestinationFilled_,
                    self->fplApproachLegStart_, self->fplApproachLegCount_,
                    self->fplCursorRow_,         &self->fplLoadedApproach_,
                    nullptr};
  edit.directToActive = mapDirectToActive();
  edit.localDraft = fplLocalDraft_;
  // The PFD FPL window groups loaded-airway legs under "Airway -" headers and
  // honors the collapse/expand toggle, so the cursor math runs the airway-aware
  // path (mirrors the MFD FPL page).
  edit.groupAirways = true;
  edit.airwaysCollapsed = fplAirwaysCollapsed_;
  fplRouteEditWireTerminalProcedures(
      edit, self->fplDepartureLegStart_, self->fplDepartureLegCount_,
      self->fplDepartureHeaderLabel_, self->fplArrivalLegStart_,
      self->fplArrivalLegCount_, self->fplArrivalHeaderLabel_,
      &self->fplLoadedDeparture_, &self->fplLoadedArrival_);
  return edit;
}

void SoftkeyController::fplOpenProcedureRemoveConfirm(FplConfirm which) {
  fplConfirm_ = which;
  fplConfirmOk_ = true;
  // The prompt subject matches the FPL list header text exactly (trainer /
  // Pilot's Guide: "Remove KATL-BBABE.CHPPR1.RW08B from flight plan?"): the
  // airport ICAO, a dash, then the procedure header label. Fall back to the
  // loaded procedure name, then the bare procedure word, when not known.
  const auto subject = [](const std::string& icao, const std::string& label,
                          const std::string& name, const char* word) {
    const std::string body = !label.empty() ? label : name;
    if (body.empty()) return std::string(word);
    return icao.empty() ? body : icao + "-" + body;
  };
  switch (which) {
    case FplConfirm::RemoveDeparture:
      fplRemoveIdent_ =
          subject(flightPlanDepartureAirportIcao(), flightPlanDepartureHeaderLabel(),
                  fplLoadedDeparture_.name, "departure");
      break;
    case FplConfirm::RemoveArrival:
      fplRemoveIdent_ =
          subject(flightPlanArrivalAirportIcao(), flightPlanArrivalHeaderLabel(),
                  fplLoadedArrival_.name, "arrival");
      break;
    case FplConfirm::RemoveApproach:
      fplRemoveIdent_ =
          subject(flightPlanApproachAirportIcao(), flightPlanApproachHeaderLabel(),
                  fplLoadedApproach_.name, "approach");
      break;
    default:
      break;
  }
}

void SoftkeyController::fplRemoveLoadedDeparture() {
  FplRouteEdit edit = flightPlanRouteEditState();
  if (!fplRemoveDeparture(edit)) return;
  persistedDepartureRestore_ = {};
  fplDepartureRestorePending_ = false;
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
  flightPlanPublishEdit();
}

void SoftkeyController::fplRemoveLoadedArrival() {
  FplRouteEdit edit = flightPlanRouteEditState();
  if (!fplRemoveArrival(edit)) return;
  persistedArrivalRestore_ = {};
  fplArrivalRestorePending_ = false;
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
  flightPlanPublishEdit();
}

void SoftkeyController::fplRemoveLoadedApproach() {
  if (fplApproachLegCount_ <= 0) reinferApproachFromProcedureLegs();
  FplRouteEdit edit = flightPlanRouteEditState();
  if (!fplRemoveApproach(edit)) return;
  persistedApproachRestore_ = {};
  fplApproachRestorePending_ = false;
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
  flightPlanPublishEdit();
}

std::string SoftkeyController::flightPlanCursorLegIdent() const {
  const int legIndex = flightPlanSelectedLegIndex();
  if (legIndex < 0 || legIndex >= static_cast<int>(fplLegs_.size())) return {};
  return fplLegs_[static_cast<std::size_t>(legIndex)].id;
}

std::string SoftkeyController::flightPlanSelectedLegIdent() const {
  if (window_ != PfdWindow::FlightPlan) return {};
  return flightPlanCursorLegIdent();
}

int SoftkeyController::flightPlanSelectedLegIndex() const {
  if (window_ != PfdWindow::FlightPlan) return -1;
  const FplRouteEdit edit = flightPlanRouteEditState();
  const int legIndex = fplCursorLegIndex(edit, flightPlanApproachAirportIcao(),
                                         FplCursorLayout::SectionRows);
  // Direct-To cannot target a synthetic departure row (RWxx/<alt>FT/MANSEQ);
  // resolve to the next real fix in the plan so the field pre-fills with it.
  return nextNavigableFixLegIndex(fplLegs_, legIndex);
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

  FplRouteEdit edit = flightPlanRouteEditState();
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
  // The Select Airway window (page menu -> Load Airway) is a modal sub-mode of
  // the FPL window: it owns the FMS knob / ENT / CLR while open.
  if (fplLoadAirway_.open) return loadAirwayBezelKey(key);
  // While the FPL window's page menu is up, let the shared page-menu handler
  // run instead of editing the list underneath it.
  if (pageMenuOpen_) return false;

  const int legCount = static_cast<int>(fplLegs_.size());
  FplRouteEdit edit = flightPlanRouteEditState();
  const std::string approachAirport = flightPlanApproachAirportIcao();
  const int selectableLast =
      fplCursorSelectableLast(edit, approachAirport, FplCursorLayout::SectionRows);

  // Modal confirmation (Remove <wpt>? / Delete Flight Plan?). ENT executes the
  // highlighted choice, CLR / knob push cancels, the knob toggles OK / CANCEL.
  if (fplConfirm_ != FplConfirm::None) {
    switch (key) {
      case BezelKey::Ent:
        if (fplConfirmOk_) {
          switch (fplConfirm_) {
            case FplConfirm::RemoveWaypoint: {
              const int legIndex = fplCursorLegIndex(
                  edit, approachAirport, FplCursorLayout::SectionRows);
              if (fplRemoveLegAtIndex(edit, legIndex)) {
                flightPlanPublishEdit();
              }
              break;
            }
            case FplConfirm::RemoveDeparture:
              fplRemoveLoadedDeparture();
              break;
            case FplConfirm::RemoveArrival:
              fplRemoveLoadedArrival();
              break;
            case FplConfirm::RemoveApproach:
              fplRemoveLoadedApproach();
              break;
            case FplConfirm::RemoveAirway: {
              const int exitLeg = ::avionics::fplCursorAirwayHeaderExitLeg(
                  edit, approachAirport, FplCursorLayout::SectionRows);
              if (::avionics::fplRemoveAirwaySegment(edit, exitLeg)) {
                fplClampCursorRow(edit, approachAirport,
                                  FplCursorLayout::SectionRows);
                flightPlanPublishEdit();
              }
              break;
            }
            case FplConfirm::DeleteFlightPlan:
              fplClearFlightPlan(edit);
              persistedApproachRestore_ = {};
              persistedDepartureRestore_ = {};
              persistedArrivalRestore_ = {};
              dtoRequestTarget_ = {};
              dtoRequestPending_ = true;
              flightPlanPublishEdit();
              break;
            case FplConfirm::None:
              break;
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

  // MENU opens the FPL window's page menu (Activate Leg / Load Airway /
  // Collapse Airways / Delete Flight Plan); handled by the global MENU path so
  // it falls through here.

  // Pushing the knob turns the selection cursor on / off.
  if (key == BezelKey::FmsPush) {
    fplCursorOn_ = !fplCursorOn_;
    fplClampCursorRow(edit, approachAirport, FplCursorLayout::SectionRows);
    return true;
  }

  // The loaded SID/STAR/approach header rows are selectable cursor stops; CLR on
  // one removes the whole procedure, and the inner knob is a no-op there (not a
  // text-entry field). (Pilot's Guide 5.6, trainer.)
  const ::avionics::FplCursorProcedureBlock cursorProcHeader =
      ::avionics::fplCursorProcedureHeader(edit, approachAirport,
                                           FplCursorLayout::SectionRows);
  const bool onProcHeader =
      cursorProcHeader != ::avionics::FplCursorProcedureBlock::None;
  // The "Airway - <name>.<exit>" header is a selectable cursor stop too; CLR on
  // it removes the whole loaded-airway segment, and the inner knob is a no-op
  // there (not a text-entry field). (Pilot's Guide, Load Airway.)
  const int cursorAirwayExitLeg = ::avionics::fplCursorAirwayHeaderExitLeg(
      edit, approachAirport, FplCursorLayout::SectionRows);
  const bool onAirwayHeader = cursorAirwayExitLeg >= 0;
  const auto openRemoveAirwayConfirm = [&]() {
    fplConfirm_ = FplConfirm::RemoveAirway;
    fplConfirmOk_ = true;
    fplRemoveIdent_ =
        "Airway " +
        fplLegs_[static_cast<std::size_t>(cursorAirwayExitLeg)].viaAirway;
  };
  const auto confirmForCursorProcedure =
      [](::avionics::FplCursorProcedureBlock block) {
        switch (block) {
          case ::avionics::FplCursorProcedureBlock::Departure:
            return FplConfirm::RemoveDeparture;
          case ::avionics::FplCursorProcedureBlock::Arrival:
            return FplConfirm::RemoveArrival;
          case ::avionics::FplCursorProcedureBlock::Approach:
            return FplConfirm::RemoveApproach;
          default:
            return FplConfirm::None;
        }
      };

  if (!fplCursorOn_) {
    // Cursor off: outer knob scrolls the section list. Inner knob on the
    // highlighted ident row opens waypoint entry (trainer / Pilot's Guide).
    if (key == BezelKey::FmsOuterCw) {
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
      return true;
    }
    if (key == BezelKey::FmsOuterCcw) {
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    }
    if (key == BezelKey::FmsInnerCw || key == BezelKey::FmsInnerCcw) {
      if (onProcHeader || onAirwayHeader)
        return true;  // header is a removal stop, not entry
      if (!fplEntry_.active) {
        fplEntry_.open(navSource_, mapData_,
                       fplIdentEntrySeedAtCursor(edit, approachAirport,
                                                 FplCursorLayout::SectionRows));
        fplEntry_.selectAll = false;
      }
      fplEntry_.turnChar(navSource_, mapData_,
                         key == BezelKey::FmsInnerCw ? +1 : -1);
      return true;
    }
    // A fix highlighted by scrolling the list (the cyan selection plate shows
    // even with the cursor off) removes on CLR, instead of falling through to
    // close the window. CLR still closes when the cursor is just following the
    // active leg (no explicit selection made yet).
    if (key == BezelKey::Clr && !fplListCursorFollowsActive_) {
      if (onProcHeader) {
        // CLR on a SID/STAR/approach header removes the whole procedure.
        fplOpenProcedureRemoveConfirm(confirmForCursorProcedure(cursorProcHeader));
        return true;
      }
      if (onAirwayHeader) {
        // CLR on an "Airway -" header removes the whole loaded-airway segment.
        openRemoveAirwayConfirm();
        return true;
      }
      const int legIndex =
          fplCursorLegIndex(edit, approachAirport, FplCursorLayout::SectionRows);
      if (legIndex >= 0 && legIndex < legCount) {
        // CLR on an individual leg removes just that fix, whether it is a plain
        // enroute waypoint or a single leg of a loaded SID/STAR/approach. CLR on
        // the procedure header (handled above) removes the whole procedure.
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(legIndex)].id;
        return true;
      }
    }
    return false;
  }

  // Cursor on: the large knob moves between section rows, the small knob opens
  // the insert entry, CLR removes the highlighted waypoint.
  switch (key) {
    case BezelKey::FmsOuterCw:
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::min(selectableLast, fplCursorRow_ + 1);
      return true;
    case BezelKey::FmsOuterCcw:
      fplListCursorFollowsActive_ = false;
      fplCursorRow_ = std::max(0, fplCursorRow_ - 1);
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw: {
      if (onProcHeader || onAirwayHeader)
        return true;  // header is a removal stop, not entry
      if (!fplEntry_.active) {
        fplEntry_.open(navSource_, mapData_,
                       fplIdentEntrySeedAtCursor(edit, approachAirport,
                                                 FplCursorLayout::SectionRows));
        fplEntry_.selectAll = false;
      }
      fplEntry_.turnChar(navSource_, mapData_,
                         key == BezelKey::FmsInnerCw ? +1 : -1);
      return true;
    }
    case BezelKey::Clr: {
      if (onProcHeader) {
        // CLR on a SID/STAR/approach header removes the whole procedure.
        fplOpenProcedureRemoveConfirm(confirmForCursorProcedure(cursorProcHeader));
        return true;
      }
      if (onAirwayHeader) {
        // CLR on an "Airway -" header removes the whole loaded-airway segment.
        openRemoveAirwayConfirm();
        return true;
      }
      const int legIndex =
          fplCursorLegIndex(edit, approachAirport, FplCursorLayout::SectionRows);
      if (legIndex >= 0 && legIndex < legCount) {
        // CLR on an individual leg removes just that fix, whether it is a plain
        // enroute waypoint or a single leg of a loaded SID/STAR/approach. CLR on
        // the procedure header (handled above) removes the whole procedure.
        fplConfirm_ = FplConfirm::RemoveWaypoint;
        fplConfirmOk_ = true;
        fplRemoveIdent_ = fplLegs_[static_cast<std::size_t>(legIndex)].id;
      }
      return true;
    }
    case BezelKey::Ent: {
      const int legIndex = fplCursorLegIndex(
          edit, approachAirport, FplCursorLayout::SectionRows);
      if (legIndex >= 0 && legIndex < legCount) {
        requestActivateFlightPlanLeg(legIndex);
      }
      return true;
    }
    default:
      return false;
  }
}

void SoftkeyController::requestActivateFlightPlanLeg(int toLegIndex) {
  if (toLegIndex < 0 || toLegIndex >= static_cast<int>(fplLegs_.size())) return;
  dtoRequestPending_ = false;
  fplActivateLegIndex_ = toLegIndex;
  fplActivateLegPending_ = true;
}

void SoftkeyController::requestDirectToFlightPlanLeg(int legIndex) {
  if (legIndex < 0 || legIndex >= static_cast<int>(fplLegs_.size())) return;
  dtoRequestTarget_ = fplLegs_[static_cast<std::size_t>(legIndex)];
  dtoRequestPending_ = true;
}

bool SoftkeyController::consumeActivateLegRequest(int& toLegIndex) {
  if (!fplActivateLegPending_) return false;
  // Route edits are consumed in the shell before engine update; defer activate
  // until the new legs are published so leg indices match the navigator plan.
  if (fplEditPending_) return false;
  fplActivateLegPending_ = false;
  toLegIndex = fplActivateLegIndex_;
  return toLegIndex >= 0;
}

void SoftkeyController::requestActivateMissedApproach() {
  if (!hasMissedApproachLegs(fplLegs_)) return;
  missedActivatePending_ = true;
}

bool SoftkeyController::consumeActivateMissedRequest() {
  if (!missedActivatePending_) return false;
  if (fplEditPending_) return false;
  missedActivatePending_ = false;
  return true;
}

FmsWaypointEntry* SoftkeyController::activeWaypointEntry() {
  if (dtoOpen_ && dtoEntry_.active && !dtoArmed_) return &dtoEntry_;
  if (fplEntry_.active) return &fplEntry_;
  if (procMenu_.airportEntry.active) return &procMenu_.airportEntry;
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
  if (saved.active && !saved.name.empty()) {
    fplApproachRestorePending_ = true;
  }
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
  if (!findLegSequenceInPlan(fplLegs_, expanded, start)) {
    // The saved legs do not contain this approach as a contiguous block; the
    // CIFP holds/altitudes cannot be re-attached, so stop retrying every frame.
    fplApproachRestorePending_ = false;
    return;
  }

  const bool wasRestorePending = fplApproachRestorePending_;
  fplApproachLegStart_ = start;
  fplApproachLegCount_ = static_cast<int>(expanded.size());
  fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
  mergeProcedureLegMetadata(fplLegs_, start, expanded);
  // Holds, altitude constraints, and glidepath are now re-attached.
  fplApproachRestorePending_ = false;
  // The restored route was pushed to the drawn map before these procedure
  // details existed (holds are not persisted per-leg); re-publish so the route
  // override and peer GDU pick up the re-attached holds.
  if (wasRestorePending) {
    flightPlanPublishEdit();
  }
}

void SoftkeyController::tryRestorePersistedTerminalProcedures() {
  bool publishNeeded = false;

  if (fplDepartureRestorePending_) {
    const bool wasPending = fplDepartureRestorePending_;
    const TerminalProcedureMetadataRestore result =
        restoreTerminalProcedureMetadata(navSource_, persistedDepartureRestore_,
                                         fplLegs_, fplDepartureLegStart_,
                                         fplDepartureLegCount_);
    if (result.spanCorrected()) {
      fplDepartureLegStart_ = result.correctedStart;
      fplDepartureLegCount_ = result.correctedCount;
    }
    if (result.merged || result.stopRetrying) {
      fplDepartureRestorePending_ = false;
    }
    if (result.merged && wasPending) {
      publishNeeded = true;
    }
  }

  if (fplArrivalRestorePending_) {
    const bool wasPending = fplArrivalRestorePending_;
    const TerminalProcedureMetadataRestore result =
        restoreTerminalProcedureMetadata(navSource_, persistedArrivalRestore_,
                                         fplLegs_, fplArrivalLegStart_,
                                         fplArrivalLegCount_);
    if (result.spanCorrected()) {
      fplArrivalLegStart_ = result.correctedStart;
      fplArrivalLegCount_ = result.correctedCount;
    }
    if (result.merged || result.stopRetrying) {
      fplArrivalRestorePending_ = false;
    }
    if (result.merged && wasPending) {
      publishNeeded = true;
    }
  }

  if (publishNeeded) {
    flightPlanPublishEdit();
  }
}

void SoftkeyController::reinferApproachFromProcedureLegs() {
  const int arrivalEnd =
      fplArrivalLegCount_ > 0 ? fplArrivalLegStart_ + fplArrivalLegCount_ : 0;
  // Never infer an approach that starts at or before the destination airport, so
  // a STAR's role-tagged fixes are not swallowed when the arrival is untracked.
  const int arrivalFloor = fplApproachInferenceFloor(fplLegs_, arrivalEnd);
  const InferredProcedureBlock block =
      inferProcedureBlockInPlan(fplLegs_, arrivalFloor);
  if (!block.valid()) return;
  int start = block.start;
  std::string transition = fplLoadedApproach_.transition;
  if (transition.empty() && persistedApproachRestore_.active) {
    transition = persistedApproachRestore_.transition;
  }
  start = approachBlockStartFromTransition(fplLegs_, transition, start);
  if (start < arrivalFloor) return;
  fplApproachLegStart_ = start;
  fplApproachLegCount_ = static_cast<int>(fplLegs_.size()) - start;
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

FlightPlanTerminalProcedureState SoftkeyController::flightPlanDepartureState()
    const {
  FlightPlanTerminalProcedureState out;
  out.legStart = fplDepartureLegStart_;
  out.legCount = fplDepartureLegCount_;
  out.loaded = fplLoadedDeparture_;
  out.headerLabel = flightPlanDepartureHeaderLabel();
  return out;
}

void SoftkeyController::applyFlightPlanDepartureState(
    const FlightPlanTerminalProcedureState& state) {
  if (!state.active()) {
    clearTerminalProcedureState(fplLoadedDeparture_, fplDepartureLegStart_,
                                fplDepartureLegCount_, fplDepartureHeaderLabel_,
                                persistedDepartureRestore_);
    return;
  }
  fplDepartureLegStart_ = state.legStart;
  fplDepartureLegCount_ = state.legCount;
  fplLoadedDeparture_ = state.loaded;
  fplDepartureHeaderLabel_ = state.headerLabel;
}

FlightPlanTerminalProcedureState SoftkeyController::flightPlanArrivalState()
    const {
  FlightPlanTerminalProcedureState out;
  out.legStart = fplArrivalLegStart_;
  out.legCount = fplArrivalLegCount_;
  out.loaded = fplLoadedArrival_;
  out.headerLabel = flightPlanArrivalHeaderLabel();
  return out;
}

void SoftkeyController::applyFlightPlanArrivalState(
    const FlightPlanTerminalProcedureState& state) {
  if (!state.active()) {
    clearTerminalProcedureState(fplLoadedArrival_, fplArrivalLegStart_,
                                fplArrivalLegCount_, fplArrivalHeaderLabel_,
                                persistedArrivalRestore_);
    return;
  }
  fplArrivalLegStart_ = state.legStart;
  fplArrivalLegCount_ = state.legCount;
  fplLoadedArrival_ = state.loaded;
  fplArrivalHeaderLabel_ = state.headerLabel;
}

void SoftkeyController::adoptFlightPlanFromPeer(
    const std::vector<MapLeg>& legs, bool destinationFilled,
    const FlightPlanApproachState& approach,
    const FlightPlanTerminalProcedureState& departure,
    const FlightPlanTerminalProcedureState& arrival,
    bool peerLocalDraft) {
  if (fplEntry_.active || fplConfirm_ != FplConfirm::None) return;
  const bool peerDraft = !legs.empty() || peerLocalDraft;
  if (flightPlanLegsEqual(fplLegs_, legs) &&
      fplDestinationFilled_ == destinationFilled &&
      flightPlanApproachState() == approach &&
      flightPlanDepartureState() == departure &&
      flightPlanArrivalState() == arrival &&
      fplLocalDraft_ == peerDraft) {
    return;
  }
  fplLegs_ = legs;
  fplDestinationFilled_ = destinationFilled;
  fplLocalDraft_ = peerDraft;
  applyFlightPlanApproachState(approach);
  applyFlightPlanDepartureState(departure);
  applyFlightPlanArrivalState(arrival);
  fplEntry_.active = false;
  fplEntry_.notFound = false;
  fplConfirm_ = FplConfirm::None;
  FplRouteEdit edit = flightPlanRouteEditState();
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
}

void SoftkeyController::adoptFlightPlanCursorFromPeer(int cursorRow,
                                                      bool followsActive) {
  if (fplEntry_.active || fplConfirm_ != FplConfirm::None) return;
  if (fplCursorRow_ == cursorRow &&
      fplListCursorFollowsActive_ == followsActive) {
    return;
  }
  fplListCursorFollowsActive_ = followsActive;
  fplCursorRow_ = cursorRow;
  FplRouteEdit edit = flightPlanRouteEditState();
  fplClampCursorRow(edit, flightPlanApproachAirportIcao(),
                    FplCursorLayout::SectionRows);
}

PersistedFlightPlan SoftkeyController::persistedFlightPlanSnapshot() const {
  PersistedFlightPlan out;
  if (fplLegs_.empty()) return out;
  out.active = true;
  out.destinationFilled = fplDestinationFilled_;
  out.legs = fplLegs_;
  const int arrivalEnd =
      fplArrivalLegCount_ > 0 ? fplArrivalLegStart_ + fplArrivalLegCount_ : 0;
  // Never persist an approach block that overlaps the arrival/STAR block or
  // starts before the destination airport: those leading legs are STAR legs, not
  // an approach (avoids resurrecting a phantom approach that swallows the STAR).
  const int arrivalFloor = fplApproachInferenceFloor(fplLegs_, arrivalEnd);
  if (fplApproachLegCount_ > 0 && fplApproachLegStart_ >= arrivalFloor) {
    out.approachLegStart = fplApproachLegStart_;
    out.approachLegCount = fplApproachLegCount_;
    out.approachAirportIcao = flightPlanApproachAirportIcao();
    if (!fplLoadedApproach_.name.empty()) {
      out.approachMeta =
          persistedFromMapProcedure(fplLoadedApproach_, out.approachAirportIcao);
    } else if (persistedApproachRestore_.active) {
      out.approachMeta = persistedApproachRestore_;
    }
  }
  if (fplDepartureLegCount_ > 0 || persistedDepartureRestore_.active) {
    out.departureLegStart = fplDepartureLegStart_;
    out.departureLegCount = fplDepartureLegCount_;
    if (!fplLoadedDeparture_.name.empty()) {
      out.departureMeta = persistedFromMapProcedure(fplLoadedDeparture_,
                                                    flightPlanDepartureAirportIcao());
    } else if (persistedDepartureRestore_.active) {
      out.departureMeta = persistedDepartureRestore_;
    }
  }
  if (fplArrivalLegCount_ > 0 || persistedArrivalRestore_.active) {
    out.arrivalLegStart = fplArrivalLegStart_;
    out.arrivalLegCount = fplArrivalLegCount_;
    if (!fplLoadedArrival_.name.empty()) {
      out.arrivalMeta =
          persistedFromMapProcedure(fplLoadedArrival_, flightPlanArrivalAirportIcao());
    } else if (persistedArrivalRestore_.active) {
      out.arrivalMeta = persistedArrivalRestore_;
    }
  }
  return out;
}

PersistedDirectTo SoftkeyController::persistedDirectToSnapshot() const {
  if (mapData_ == nullptr) return {};
  return persistedDirectToFromMap(*mapData_);
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
  clearTerminalProcedureState(fplLoadedDeparture_, fplDepartureLegStart_,
                              fplDepartureLegCount_, fplDepartureHeaderLabel_,
                              persistedDepartureRestore_);
  clearTerminalProcedureState(fplLoadedArrival_, fplArrivalLegStart_,
                              fplArrivalLegCount_, fplArrivalHeaderLabel_,
                              persistedArrivalRestore_);
  fplCursorRow_ = 0;
  fplLastPublished_ = saved.legs;
  fplLastMapPlan_ = saved.legs;
  fplApproachRestorePending_ = false;
  fplDepartureRestorePending_ = false;
  fplArrivalRestorePending_ = false;
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
  if (saved.departureMeta.active) {
    persistedDepartureRestore_ = saved.departureMeta;
    applyTerminalProcedureMeta(saved.departureMeta, saved.departureLegStart,
                               saved.departureLegCount, fplLoadedDeparture_,
                               fplDepartureHeaderLabel_, fplDepartureLegStart_,
                               fplDepartureLegCount_);
  }
  if (saved.arrivalMeta.active) {
    persistedArrivalRestore_ = saved.arrivalMeta;
    applyTerminalProcedureMeta(saved.arrivalMeta, saved.arrivalLegStart,
                               saved.arrivalLegCount, fplLoadedArrival_,
                               fplArrivalHeaderLabel_, fplArrivalLegStart_,
                               fplArrivalLegCount_);
  }
  if (fplApproachLegCount_ <= 0) {
    reinferApproachFromProcedureLegs();
  }
  if (fplApproachLegCount_ > 0 && persistedApproachRestore_.name.empty()) {
    inferApproachMetadataFromLegs(fplLegs_, fplApproachLegStart_,
                                persistedApproachRestore_);
  }
  if (fplApproachLegCount_ > 0 && fplLoadedApproach_.name.empty() &&
      persistedApproachRestore_.active &&
      !persistedApproachRestore_.name.empty()) {
    fplLoadedApproach_ = mapProcedureFromPersisted(persistedApproachRestore_);
  }
  // Re-expand the CIFP approach (holds, altitude constraints, and glidepath are
  // not persisted per-leg) so they are re-attached after a restart. Arm it from
  // the now-final metadata and attempt immediately in case nav data is already
  // loaded; syncFlightPlanLegs retries until nav data is ready.
  fplApproachRestorePending_ = fplApproachLegCount_ > 0 &&
                               persistedApproachRestore_.active &&
                               !persistedApproachRestore_.name.empty();
  fplDepartureRestorePending_ = saved.departureMeta.active &&
                                !saved.departureMeta.name.empty() &&
                                saved.departureLegCount > 0;
  fplArrivalRestorePending_ = saved.arrivalMeta.active &&
                              !saved.arrivalMeta.name.empty() &&
                              saved.arrivalLegCount > 0;
  tryRestorePersistedApproach();
  tryRestorePersistedTerminalProcedures();
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
