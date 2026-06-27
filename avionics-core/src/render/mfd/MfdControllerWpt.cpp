#include "avionics/MfdController.h"

#include "avionics/ProcedureSupport.h"

// WPT facility ident entry (Pilot's Guide, Waypoint Pages): the small knob
// spells an identifier, ENT selects the resolved waypoint for the page.
namespace avionics {

void MfdController::wptResetInteraction() {
  wptEntry_.reset();
  wptHasSelection_ = false;
  wptFeature_ = MapFeature{};
  wptInfoView_ = WptInfoView::Airport;
}

void MfdController::wptCommitEntry() {
  if (wptEntry_.chars.empty()) {
    wptEntry_.active = false;
    return;
  }
  if (!wptEntry_.hasMatch) {
    wptEntry_.notFound = true;
    return;
  }
  wptFeature_ = wptEntry_.match;
  wptHasSelection_ = true;
  wptEntry_.active = false;
  wptEntry_.notFound = false;
  // A new airport invalidates the previewed procedure; revert to the airport
  // information panel like the real unit (mirrors the chart-view reset).
  wptInfoView_ = WptInfoView::Airport;
}

WptProcedureInfo MfdController::wptProcedureInfo(const MapFeature& airport,
                                                 ProcedureType type) const {
  WptProcedureInfo info;
  const std::string icao = airport.id;
  if (navSource_ == nullptr || !navSource_->ready() || icao.empty()) {
    return info;
  }

  const std::vector<MapProcedure> catalog = proceduresForAirport(icao, type);
  if (catalog.empty()) return info;

  if (type == ProcedureType::Approach) {
    // First published approach; transition defaults to VECTORS when offered
    // (defaultProcedureTransition), matching the trainer's initial selection.
    const MapProcedure& proc = catalog.front();
    info.name = formatApproachProcedureLabel(proc);
    const std::vector<std::string> transitions =
        procedureTransitionIds(navSource_, icao, type, proc.name);
    info.transition = defaultProcedureTransition(transitions);
    info.legs =
        navSource_->expandProcedure(icao, type, proc.name, info.transition);
    const ProcPrimaryNav primary =
        resolveProcPrimaryNav(navSource_, mapData_, airport, proc);
    info.primaryFreqMhz = primary.frequency;
    info.primaryIdent = primary.ident;
    info.primaryIsNdb = primary.isNdb;
    info.available = true;
    return info;
  }

  // Arrival / Departure: first procedure name, with the default enroute
  // transition and the first runway option (kProcRunwayAll when none).
  const std::vector<std::string> names = uniqueProcedureNames(catalog);
  if (names.empty()) return info;
  const std::string name = names.front();
  const std::vector<std::string> enroute =
      procedureEnrouteTransitions(navSource_, icao, type, name);
  const std::vector<std::string> runways =
      procedureRunwayOptions(navSource_, icao, type, name);
  info.name = name;
  info.transition = defaultProcedureTransition(enroute);
  info.runway = runways.empty() ? std::string(kProcRunwayAll) : runways.front();
  info.legs = expandArrivalDepartureProcedure(navSource_, icao, type, name,
                                              info.transition, info.runway);
  info.available = true;
  return info;
}

bool MfdController::wptBezelKey(BezelKey key) {
  if (isMapRangePanBezelKey(key)) return false;

  if (wptEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        wptCommitEntry();
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        wptEntry_.active = false;
        wptEntry_.notFound = false;
        break;
      case BezelKey::FmsInnerCw:
        wptEntry_.turnChar(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsInnerCcw:
        wptEntry_.turnChar(navSource_, mapData_, -1);
        break;
      case BezelKey::FmsOuterCw:
        wptEntry_.moveCursor(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsOuterCcw:
        wptEntry_.moveCursor(navSource_, mapData_, -1);
        break;
      default:
        break;
    }
    return true;
  }

  switch (key) {
    case BezelKey::FmsPush:
      wptEntry_.open(navSource_, mapData_, wptHasSelection_ ? wptFeature_.id : "");
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      wptEntry_.open(navSource_, mapData_);
      return true;
    default:
      return false;
  }
}

}  // namespace avionics
