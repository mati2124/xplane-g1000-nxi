#include "avionics/FplRouteEdit.h"

#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FmsWaypointEntry.h"
#include "avionics/GpsLegCourse.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/NavMath.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

using pfd::FplDisplayRow;
using pfd::FplDisplayRowKind;
using pfd::fplApproachInsertIndexForSelectable;
using pfd::fplApproachLegIndexForSelectable;
using pfd::fplApproachSelectableCount;
using pfd::fplApproachSelectableRow;
using pfd::buildFplSectionRows;
using pfd::fplApproachSelectableRowForLegIndex;
using pfd::fplFilteredSectionRows;
using pfd::fplFilteredSectionSelectableCount;
using pfd::fplFilterDuplicateLegSectionRows;
using pfd::fplProcedureInsertIndexForSelectable;
using pfd::fplProcedureLegIndexForSelectable;
using pfd::fplProcedureSelectableCount;
using pfd::fplProcedureSelectableRow;
using pfd::fplProcedureSelectableRowForLegIndex;
using pfd::fplSectionSelectableRowForLegIndex;
using pfd::fplSectionLegIndexForSelectable;
using pfd::fplSectionInsertIndexForSelectable;
using pfd::fplSectionSelectableRow;
using pfd::fplUsesProcedureDisplayRows;

bool approachLayoutDestFilled(const FplRouteEdit& edit) {
  return fplEditLayoutDestinationFilled(edit);
}

int fplEditDepartureLegStart(const FplRouteEdit& edit) {
  return edit.departureLegStart != nullptr ? *edit.departureLegStart : 0;
}

int fplEditDepartureLegCount(const FplRouteEdit& edit) {
  return edit.departureLegCount != nullptr ? *edit.departureLegCount : 0;
}

std::string fplEditDepartureHeader(const FplRouteEdit& edit) {
  return edit.departureHeaderLabel != nullptr ? *edit.departureHeaderLabel
                                              : std::string();
}

int fplEditArrivalLegStart(const FplRouteEdit& edit) {
  return edit.arrivalLegStart != nullptr ? *edit.arrivalLegStart : 0;
}

int fplEditArrivalLegCount(const FplRouteEdit& edit) {
  return edit.arrivalLegCount != nullptr ? *edit.arrivalLegCount : 0;
}

std::string fplEditArrivalHeader(const FplRouteEdit& edit) {
  return edit.arrivalHeaderLabel != nullptr ? *edit.arrivalHeaderLabel
                                            : std::string();
}

bool fplEditUsesProcedureDisplay(const FplRouteEdit& edit) {
  if (fplUsesProcedureDisplayRows(
          fplEditDepartureHeader(edit), fplEditDepartureLegCount(edit),
          fplEditArrivalHeader(edit), fplEditArrivalLegCount(edit),
          edit.approachLegCount)) {
    return true;
  }
  // MFD Load Airway: a plan with loaded-airway legs also uses the procedure
  // display rows so the cursor math matches the grouped/collapsed list.
  return edit.groupAirways && pfd::fplPlanHasAirwayLegs(edit.legs);
}

bool fplEditProcedureBlankOriginSection(const FplRouteEdit& edit,
                                        const std::string& approachAirport) {
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const bool destOnlyPlan =
      edit.destinationFilled && edit.legs.size() == 1;
  const bool hasDeparture = !fplEditDepartureHeader(edit).empty() ||
                            fplEditDepartureLegCount(edit) > 0;
  const bool approachLoaded = edit.approachLegCount > 0;
  return directToPlanBody || destOnlyPlan || hasDeparture ||
         (approachLoaded && edit.approachLegStart <= 1 && !edit.legs.empty() &&
          !approachAirport.empty() && edit.legs.front().id == approachAirport);
}

bool fplEditProcedureDestinationFilled(const FplRouteEdit& edit) {
  return fplEditLayoutDestinationFilled(edit);
}

bool fplCommitApproachIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                            const MapFeature& match, const std::string& ident,
                            int selectableCursorRow,
                            const std::string& approachAirport) {
  const int legCount = static_cast<int>(edit.legs.size());
  const bool blankOrigin = fplApproachBlankOriginSection(
      edit.legs, edit.approachLegStart, approachAirport);
  const bool layoutDestFilled = approachLayoutDestFilled(edit);
  const int selectableLast = fplApproachSelectableCount(
                                 edit.legs, edit.approachLegStart,
                                 edit.approachLegCount, blankOrigin,
                                 layoutDestFilled) -
                             1;
  const int legIndex = fplApproachLegIndexForSelectable(
      selectableCursorRow, edit.legs, edit.approachLegStart,
      edit.approachLegCount, blankOrigin, layoutDestFilled);
  const int row = fplApproachInsertIndexForSelectable(
      selectableCursorRow, edit.legs, edit.approachLegStart,
      edit.approachLegCount, legCount, blankOrigin, layoutDestFilled);
  const pfd::FplDisplayRow* dr = fplApproachSelectableRow(
      selectableCursorRow, edit.legs, edit.approachLegStart,
      edit.approachLegCount, blankOrigin, layoutDestFilled);
  if (dr != nullptr && dr->kind == FplDisplayRowKind::Destination &&
      dr->legIndex < 0) {
    edit.destinationFilled = true;
  }

  if (navSource != nullptr && navSource->isAirwayName(ident) && row > 0 &&
      row < legCount) {
    const std::vector<MapLeg> expanded = navSource->expandAirway(
        ident, edit.legs[static_cast<std::size_t>(row - 1)].id,
        edit.legs[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      edit.legs.insert(edit.legs.begin() + row, expanded.begin(), expanded.end());
      if (row <= edit.approachLegStart) {
        edit.approachLegStart += static_cast<int>(expanded.size());
      }
      if (edit.legs.size() >= 3) edit.destinationFilled = true;
      edit.cursorRow = std::min(
          selectableLast,
          selectableCursorRow + static_cast<int>(expanded.size()));
      return true;
    }
    return false;
  }

  MapLeg leg;
  leg.lat = match.lat;
  leg.lon = match.lon;
  leg.id = ident;
  if (legIndex >= 0 && legIndex < legCount) {
    edit.legs[static_cast<std::size_t>(legIndex)] = leg;
  } else {
    edit.legs.insert(edit.legs.begin() + row, leg);
    if (row <= edit.approachLegStart) {
      ++edit.approachLegStart;
    }
    if (edit.legs.size() >= 3) edit.destinationFilled = true;
    edit.cursorRow = std::min(selectableLast, selectableCursorRow + 1);
  }
  return true;
}

bool fplCommitSectionIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                           const MapFeature& match, const std::string& ident,
                           int selectableCursorRow) {
  const int legCount = fplEditSectionLegCount(edit);
  const bool layoutDestFilled = fplEditLayoutDestinationFilled(edit);
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const std::vector<pfd::FplSectionRow> sectionRows = fplFilteredSectionRows(
      legCount, layoutDestFilled, directToPlanBody, edit.legs);
  const int lastSection =
      std::max(0, fplFilteredSectionSelectableCount(
                       sectionRows, legCount, layoutDestFilled) -
                       1);
  const int legIndex = fplSectionLegIndexForSelectable(
      selectableCursorRow, sectionRows, legCount, layoutDestFilled);
  const int row = fplSectionInsertIndexForSelectable(
      selectableCursorRow, sectionRows, legCount, layoutDestFilled);

  // Decide destination-filled by the kind of row the cursor sits on, not its
  // positional index, since the Origin/Destination dashed rows are now their
  // own cursor stops and shift the index ordering.
  const pfd::FplSectionRow* selRow = pfd::fplSectionSelectableRow(
      selectableCursorRow, sectionRows, legCount, layoutDestFilled);
  const pfd::FplSectionRow::Kind selKind =
      selRow != nullptr ? selRow->kind : pfd::FplSectionRow::Kind::EnrouteBlank;
  if (selKind == pfd::FplSectionRow::Kind::Destination ||
      selKind == pfd::FplSectionRow::Kind::DestinationBlank) {
    edit.destinationFilled = true;
  } else if ((selKind == pfd::FplSectionRow::Kind::Origin ||
              selKind == pfd::FplSectionRow::Kind::OriginBlank ||
              selKind == pfd::FplSectionRow::Kind::EnrouteBlank) &&
             legCount < 2) {
    edit.destinationFilled = false;
  }

  if (navSource != nullptr && navSource->isAirwayName(ident) && row > 0 &&
      row < legCount) {
    const std::vector<MapLeg> expanded = navSource->expandAirway(
        ident, edit.legs[static_cast<std::size_t>(row - 1)].id,
        edit.legs[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      edit.legs.insert(edit.legs.begin() + row, expanded.begin(), expanded.end());
      if (edit.legs.size() >= 3) edit.destinationFilled = true;
      edit.cursorRow = std::min(
          lastSection, selectableCursorRow + static_cast<int>(expanded.size()));
      return true;
    }
    return false;
  }

  MapLeg leg;
  leg.lat = match.lat;
  leg.lon = match.lon;
  leg.id = ident;
  int committedLegIndex = -1;
  if (legIndex >= 0 && legIndex < legCount) {
    edit.legs[static_cast<std::size_t>(legIndex)] = leg;
    committedLegIndex = legIndex;
  } else {
    edit.legs.insert(edit.legs.begin() + row, leg);
    committedLegIndex = row;
  }
  if (edit.legs.size() >= 3) edit.destinationFilled = true;
  // Land the list cursor on the leg just entered so it is immediately
  // selectable (CLR removes it, ENT activates it). Without this the cursor is
  // stranded on a now-blank slot — e.g. entering the first fix on an empty plan
  // shifts it up into the Origin row while the cursor stays below it, leaving
  // the new waypoint unreachable. The other commit paths do the same.
  const int newLegCount = fplEditSectionLegCount(edit);
  const bool newDestFilled = fplEditLayoutDestinationFilled(edit);
  const std::vector<pfd::FplSectionRow> newRows = fplFilteredSectionRows(
      newLegCount, newDestFilled, directToPlanBody, edit.legs);
  const int committedRow = fplSectionSelectableRowForLegIndex(
      committedLegIndex, newRows, newLegCount, newDestFilled);
  if (committedRow >= 0) {
    edit.cursorRow = committedRow;
  } else {
    edit.cursorRow = std::min(lastSection, selectableCursorRow);
  }
  return true;
}

bool fplCommitProcedureIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                             const MapFeature& match, const std::string& ident,
                             int selectableCursorRow,
                             const std::string& approachAirport) {
  const int legCount = static_cast<int>(edit.legs.size());
  const bool blankOrigin =
      fplEditProcedureBlankOriginSection(edit, approachAirport);
  const bool layoutDestFilled = fplEditLayoutDestinationFilled(edit);
  const int depStart = fplEditDepartureLegStart(edit);
  const int depCount = fplEditDepartureLegCount(edit);
  const int arrStart = fplEditArrivalLegStart(edit);
  const int arrCount = fplEditArrivalLegCount(edit);
  const int selectableLast = fplProcedureSelectableCount(
                                 edit.legs, depStart, depCount,
                                 fplEditDepartureHeader(edit), arrStart,
                                 arrCount, fplEditArrivalHeader(edit),
                                 edit.approachLegStart, edit.approachLegCount,
                                 blankOrigin, layoutDestFilled,
                                 edit.airwaysCollapsed) -
                             1;
  const int legIndex = fplProcedureLegIndexForSelectable(
      selectableCursorRow, edit.legs, depStart, depCount,
      fplEditDepartureHeader(edit), arrStart, arrCount,
      fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
      blankOrigin, layoutDestFilled, edit.airwaysCollapsed);
  const int row = fplProcedureInsertIndexForSelectable(
      selectableCursorRow, edit.legs, depStart, depCount,
      fplEditDepartureHeader(edit), arrStart, arrCount,
      fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
      legCount, blankOrigin, layoutDestFilled, edit.airwaysCollapsed);
  const pfd::FplDisplayRow* dr = fplProcedureSelectableRow(
      selectableCursorRow, edit.legs, depStart, depCount,
      fplEditDepartureHeader(edit), arrStart, arrCount,
      fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
      blankOrigin, layoutDestFilled, edit.airwaysCollapsed);
  if (dr != nullptr && dr->kind == FplDisplayRowKind::Destination &&
      dr->legIndex < 0) {
    edit.destinationFilled = true;
  }

  if (navSource != nullptr && navSource->isAirwayName(ident) && row > 0 &&
      row < legCount) {
    const std::vector<MapLeg> expanded = navSource->expandAirway(
        ident, edit.legs[static_cast<std::size_t>(row - 1)].id,
        edit.legs[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      edit.legs.insert(edit.legs.begin() + row, expanded.begin(), expanded.end());
      if (row <= edit.approachLegStart) {
        edit.approachLegStart += static_cast<int>(expanded.size());
      }
      if (edit.arrivalLegStart != nullptr && row <= *edit.arrivalLegStart) {
        *edit.arrivalLegStart += static_cast<int>(expanded.size());
      }
      if (edit.legs.size() >= 3) edit.destinationFilled = true;
      edit.cursorRow = std::min(
          selectableLast,
          selectableCursorRow + static_cast<int>(expanded.size()));
      return true;
    }
    return false;
  }

  MapLeg leg;
  leg.lat = match.lat;
  leg.lon = match.lon;
  leg.id = ident;
  if (legIndex >= 0 && legIndex < legCount) {
    edit.legs[static_cast<std::size_t>(legIndex)] = leg;
  } else {
    edit.legs.insert(edit.legs.begin() + row, leg);
    if (row <= edit.approachLegStart) {
      ++edit.approachLegStart;
    }
    if (edit.arrivalLegStart != nullptr && row <= *edit.arrivalLegStart) {
      ++(*edit.arrivalLegStart);
    }
    if (edit.legs.size() >= 3) edit.destinationFilled = true;
    edit.cursorRow = std::min(selectableLast, selectableCursorRow + 1);
  }
  return true;
}

bool fplCommitFlatIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                        const MapFeature& match, const std::string& ident,
                        int selectableCursorRow) {
  const int legCount = static_cast<int>(edit.legs.size());
  const int row = std::max(0, std::min(legCount, selectableCursorRow));

  if (navSource != nullptr && navSource->isAirwayName(ident) && row > 0 &&
      row < legCount) {
    const std::vector<MapLeg> expanded = navSource->expandAirway(
        ident, edit.legs[static_cast<std::size_t>(row - 1)].id,
        edit.legs[static_cast<std::size_t>(row)].id);
    if (!expanded.empty()) {
      edit.legs.insert(edit.legs.begin() + row, expanded.begin(), expanded.end());
      edit.cursorRow = row + static_cast<int>(expanded.size());
      return true;
    }
    return false;
  }

  MapLeg leg;
  leg.lat = match.lat;
  leg.lon = match.lon;
  leg.id = ident;
  edit.legs.insert(edit.legs.begin() + row, leg);
  edit.cursorRow = row + 1;
  return true;
}

}  // namespace

bool isAirportIdent(const std::string& id) {
  if (id.size() != 4) return false;
  for (char c : id) {
    if (c < 'A' || c > 'Z') return false;
  }
  return true;
}

bool isKnownAirportIdent(const std::string& id, const MapData* map,
                         const NavFeatureSource* navSource) {
  if (!isAirportIdent(id)) return false;
  if (navSource != nullptr && navSource->ready()) {
    const std::vector<MapFeature> hits = navSource->lookupIdent(id, 8);
    for (const MapFeature& f : hits) {
      if (f.id == id && f.type == MapFeatureType::Airport) return true;
    }
  }
  if (map != nullptr) {
    for (const MapFeature& f : map->features) {
      if (f.type == MapFeatureType::Airport && f.id == id) return true;
    }
  }
  // Nav data still loading: allow well-formed ICAOs through to the Charts API.
  return navSource == nullptr || !navSource->ready();
}

std::string firstKnownAirportInPlan(const std::vector<MapLeg>& legs,
                                    const MapData* map,
                                    const NavFeatureSource* navSource) {
  for (const MapLeg& leg : legs) {
    if (isKnownAirportIdent(leg.id, map, navSource)) return leg.id;
  }
  return {};
}

std::string lastKnownAirportInPlan(const std::vector<MapLeg>& legs,
                                   const MapData* map,
                                   const NavFeatureSource* navSource) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (isKnownAirportIdent(legs[static_cast<std::size_t>(i)].id, map,
                            navSource)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

std::string airportIcaoBeforeIndex(const std::vector<MapLeg>& legs, int before) {
  for (int i = std::min(before, static_cast<int>(legs.size())) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

std::string firstAirportInPlan(const std::vector<MapLeg>& legs) {
  for (const MapLeg& leg : legs) {
    if (isAirportIdent(leg.id)) return leg.id;
  }
  return {};
}

std::string lastAirportInPlan(const std::vector<MapLeg>& legs) {
  for (int i = static_cast<int>(legs.size()) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
  }
  return {};
}

std::string directToAirportIcao(const MapData* map) {
  if (map == nullptr || !map->directToActive) return {};
  return isAirportIdent(map->directTo.id) ? map->directTo.id : std::string();
}

namespace {

constexpr double kNearestAirportMaxRangeNm = 200.0;

}  // namespace

std::vector<std::string> nearestAirportIds(const MapData* map, int maxCount) {
  std::vector<std::string> ids;
  if (map == nullptr || !map->positionValid || maxCount <= 0) return ids;

  struct Row {
    std::string id;
    double distNm = 0.0;
  };
  std::vector<Row> rows;
  for (const MapFeature& f : map->features) {
    if (f.type != MapFeatureType::Airport) continue;
    const double dist =
        navDistanceNm(map->ownshipLat, map->ownshipLon, f.lat, f.lon);
    if (dist > kNearestAirportMaxRangeNm) continue;
    rows.push_back({f.id, dist});
  }
  std::sort(rows.begin(), rows.end(),
            [](const Row& a, const Row& b) { return a.distNm < b.distNm; });
  if (static_cast<int>(rows.size()) > maxCount) {
    rows.resize(static_cast<std::size_t>(maxCount));
  }
  ids.reserve(rows.size());
  for (const Row& row : rows) ids.push_back(row.id);
  return ids;
}

std::string nearestAirportIcao(const MapData* map) {
  const std::vector<std::string> ids = nearestAirportIds(map, 1);
  return ids.empty() ? std::string() : ids.front();
}

bool flightPlanLegsEqual(const std::vector<MapLeg>& a,
                         const std::vector<MapLeg>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id || a[i].lat != b[i].lat || a[i].lon != b[i].lon ||
        a[i].procedureRole != b[i].procedureRole ||
        a[i].viaAirway != b[i].viaAirway) {
      return false;
    }
  }
  return true;
}

bool fplAdoptMapPlanDuringDirectTo(FplRouteEdit& edit,
                                   const std::vector<MapLeg>& mapPlan,
                                   const std::vector<MapLeg>& lastPublished) {
  const InferredProcedureBlock proc = inferProcedureBlockInPlan(mapPlan);
  if (!proc.valid()) return false;
  const InferredProcedureBlock existing = inferProcedureBlockInPlan(edit.legs);
  if (existing.valid() && edit.legs.size() > mapPlan.size()) return false;
  edit.legs = mapPlan;
  preserveFlightPlanIdents(edit.legs, lastPublished);
  edit.destinationFilled = false;
  edit.approachLegStart = 0;
  edit.approachLegCount = 0;
  if (edit.loadedApproach != nullptr) *edit.loadedApproach = {};
  if (edit.approachHeaderLabel != nullptr) edit.approachHeaderLabel->clear();
  return true;
}

bool fplDestinationFilledFromLegCount(int legCount) {
  return legCount >= 3 || legCount == 2;
}

bool fplDestinationFilledForDisplay(int legCount, bool directToActive) {
  if (directToActive) return false;
  return fplDestinationFilledFromLegCount(legCount);
}

std::string fplApproachAirportIcao(const std::vector<MapLeg>& legs,
                                   int approachStart, const MapData* map,
                                   const std::string& loadedApproachAirportIcao) {
  // An explicitly loaded/known approach airport is authoritative. Prefer it over
  // the "airport waypoint before the approach" heuristic, which returns the
  // origin when the plan has no destination airport waypoint ahead of the
  // approach (e.g. KJAX -> [RNAV approach into KFMY] with no KFMY leg).
  if (isAirportIdent(loadedApproachAirportIcao)) return loadedApproachAirportIcao;
  if (approachStart <= 0 && legs.empty()) return {};
  std::string icao = airportIcaoBeforeIndex(legs, approachStart);
  if (!icao.empty()) return icao;
  icao = directToAirportIcao(map);
  if (!icao.empty()) return icao;
  if (map != nullptr) {
    icao = lastAirportInPlan(map->flightPlan);
    if (!icao.empty()) return icao;
  }
  return {};
}

int fplCursorLegIndex(const FplRouteEdit& edit,
                      const std::string& approachAirport,
                      FplCursorLayout layout) {
  const int legCount = static_cast<int>(edit.legs.size());
  const int sectionLegCount = fplEditSectionLegCount(edit);
  if (fplEditUsesProcedureDisplay(edit)) {
    return fplProcedureLegIndexForSelectable(
        edit.cursorRow, edit.legs, fplEditDepartureLegStart(edit),
        fplEditDepartureLegCount(edit), fplEditDepartureHeader(edit),
        fplEditArrivalLegStart(edit), fplEditArrivalLegCount(edit),
        fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
        fplEditProcedureBlankOriginSection(edit, approachAirport),
        fplEditProcedureDestinationFilled(edit), edit.airwaysCollapsed);
  }
  if (layout == FplCursorLayout::FlatLegList) {
    if (edit.cursorRow >= 0 && edit.cursorRow < legCount) return edit.cursorRow;
    return -1;
  }
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const bool layoutDestFilled = fplEditLayoutDestinationFilled(edit);
  const std::vector<pfd::FplSectionRow> sectionRows = fplFilteredSectionRows(
      sectionLegCount, layoutDestFilled, directToPlanBody, edit.legs);
  return fplSectionLegIndexForSelectable(edit.cursorRow, sectionRows,
                                         sectionLegCount, layoutDestFilled);
}

bool fplCursorOnHoldRow(const FplRouteEdit& edit,
                        const std::string& approachAirport,
                        FplCursorLayout layout) {
  if (edit.approachLegCount <= 0) return false;
  if (layout != FplCursorLayout::SectionRows) return false;
  const FplDisplayRow* dr = fplApproachSelectableRow(
      edit.cursorRow, edit.legs, edit.approachLegStart, edit.approachLegCount,
      fplApproachBlankOriginSection(edit.legs, edit.approachLegStart,
                                    approachAirport),
      approachLayoutDestFilled(edit));
  return dr != nullptr && dr->kind == FplDisplayRowKind::Hold;
}

FplCursorProcedureBlock fplCursorProcedureHeader(
    const FplRouteEdit& edit, const std::string& approachAirport,
    FplCursorLayout layout) {
  if (layout != FplCursorLayout::SectionRows) return FplCursorProcedureBlock::None;
  if (!fplEditUsesProcedureDisplay(edit)) return FplCursorProcedureBlock::None;
  const FplDisplayRow* dr = fplProcedureSelectableRow(
      edit.cursorRow, edit.legs, fplEditDepartureLegStart(edit),
      fplEditDepartureLegCount(edit), fplEditDepartureHeader(edit),
      fplEditArrivalLegStart(edit), fplEditArrivalLegCount(edit),
      fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
      fplEditProcedureBlankOriginSection(edit, approachAirport),
      fplEditProcedureDestinationFilled(edit), edit.airwaysCollapsed);
  if (dr == nullptr) return FplCursorProcedureBlock::None;
  switch (dr->kind) {
    case FplDisplayRowKind::DepartureHeader:
      return FplCursorProcedureBlock::Departure;
    case FplDisplayRowKind::ArrivalHeader:
      return FplCursorProcedureBlock::Arrival;
    case FplDisplayRowKind::ApproachHeader:
      return FplCursorProcedureBlock::Approach;
    default:
      return FplCursorProcedureBlock::None;
  }
}

int fplCursorAirwayHeaderExitLeg(const FplRouteEdit& edit,
                                 const std::string& approachAirport,
                                 FplCursorLayout layout) {
  if (layout != FplCursorLayout::SectionRows) return -1;
  if (!fplEditUsesProcedureDisplay(edit)) return -1;
  const FplDisplayRow* dr = fplProcedureSelectableRow(
      edit.cursorRow, edit.legs, fplEditDepartureLegStart(edit),
      fplEditDepartureLegCount(edit), fplEditDepartureHeader(edit),
      fplEditArrivalLegStart(edit), fplEditArrivalLegCount(edit),
      fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
      fplEditProcedureBlankOriginSection(edit, approachAirport),
      fplEditProcedureDestinationFilled(edit), edit.airwaysCollapsed);
  if (dr == nullptr || dr->kind != FplDisplayRowKind::AirwayHeader) return -1;
  return dr->legIndex;
}

int fplCursorSelectableLast(const FplRouteEdit& edit,
                            const std::string& approachAirport,
                            FplCursorLayout layout) {
  if (fplEditUsesProcedureDisplay(edit)) {
    return std::max(
        0, fplProcedureSelectableCount(
               edit.legs, fplEditDepartureLegStart(edit),
               fplEditDepartureLegCount(edit), fplEditDepartureHeader(edit),
               fplEditArrivalLegStart(edit), fplEditArrivalLegCount(edit),
               fplEditArrivalHeader(edit), edit.approachLegStart,
               edit.approachLegCount,
               fplEditProcedureBlankOriginSection(edit, approachAirport),
               fplEditProcedureDestinationFilled(edit),
               edit.airwaysCollapsed) -
               1);
  }
  if (layout == FplCursorLayout::FlatLegList) {
    return static_cast<int>(edit.legs.size());
  }
  const int sectionLegCount = fplEditSectionLegCount(edit);
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const bool layoutDestFilled = fplEditLayoutDestinationFilled(edit);
  const std::vector<pfd::FplSectionRow> sectionRows = fplFilteredSectionRows(
      sectionLegCount, layoutDestFilled, directToPlanBody, edit.legs);
  return std::max(0, fplFilteredSectionSelectableCount(
                           sectionRows, sectionLegCount, layoutDestFilled) -
                       1);
}

void fplClampCursorRow(FplRouteEdit& edit, const std::string& approachAirport,
                       FplCursorLayout layout) {
  const int last = fplCursorSelectableLast(edit, approachAirport, layout);
  edit.cursorRow = std::max(0, std::min(last, edit.cursorRow));
}

int fplActiveSelectableRow(const FplRouteEdit& edit,
                             const std::string& approachAirport,
                             const std::string& activeToIdent,
                             FplCursorLayout layout) {
  const int activeLegIdx =
      fplActiveLegIndexInPlan(edit.legs, activeToIdent);
  if (fplEditUsesProcedureDisplay(edit)) {
    return fplProcedureSelectableRowForLegIndex(
        activeLegIdx, edit.legs, fplEditDepartureLegStart(edit),
        fplEditDepartureLegCount(edit), fplEditDepartureHeader(edit),
        fplEditArrivalLegStart(edit), fplEditArrivalLegCount(edit),
        fplEditArrivalHeader(edit), edit.approachLegStart, edit.approachLegCount,
        fplEditProcedureBlankOriginSection(edit, approachAirport),
        fplEditProcedureDestinationFilled(edit), edit.airwaysCollapsed);
  }
  if (layout == FplCursorLayout::FlatLegList) {
    return activeLegIdx;
  }
  const int legCount = fplEditSectionLegCount(edit);
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const bool layoutDestFilled = fplEditLayoutDestinationFilled(edit);
  const std::vector<pfd::FplSectionRow> sectionRows =
      fplFilterDuplicateLegSectionRows(
          buildFplSectionRows(legCount, layoutDestFilled, directToPlanBody),
          edit.legs);
  return fplSectionSelectableRowForLegIndex(
      activeLegIdx, sectionRows, legCount, layoutDestFilled,
      directToPlanBody);
}

void fplSyncListCursorToActiveLeg(FplRouteEdit& edit,
                                  const std::string& approachAirport,
                                  const std::string& activeToIdent,
                                  bool& listCursorFollowsActive,
                                  FplCursorLayout layout) {
  if (!listCursorFollowsActive) return;
  const int activeRow =
      fplActiveSelectableRow(edit, approachAirport, activeToIdent, layout);
  if (activeRow >= 0) {
    edit.cursorRow = activeRow;
    fplClampCursorRow(edit, approachAirport, layout);
  }
}

void fplRefreshDestinationFilledAfterRemove(FplRouteEdit& edit,
                                            int legCountAfter) {
  if (legCountAfter <= 1) {
    edit.destinationFilled = false;
  } else if (legCountAfter == 2 && !edit.destinationFilled) {
    // still origin + enroute
  } else if (legCountAfter < 3) {
    edit.destinationFilled = legCountAfter == 2;
  }
}

void fplAdjustApproachGroupingAfterRemove(FplRouteEdit& edit,
                                           int removedLegIndex) {
  if (edit.approachLegCount <= 0) return;
  if (removedLegIndex >= edit.approachLegStart &&
      removedLegIndex < edit.approachLegStart + edit.approachLegCount) {
    --edit.approachLegCount;
    if (edit.approachLegCount <= 0) {
      edit.approachLegStart = 0;
      if (edit.loadedApproach != nullptr) *edit.loadedApproach = {};
      if (edit.approachHeaderLabel != nullptr) edit.approachHeaderLabel->clear();
    }
  } else if (removedLegIndex < edit.approachLegStart) {
    --edit.approachLegStart;
  }
}

void fplAdjustTerminalProcedureGroupingAfterRemove(FplRouteEdit& edit,
                                                   int removedLegIndex) {
  // Departure block: when its last leg is removed, drop the whole procedure
  // (header label + loaded procedure) so the FPL list no longer shows the
  // empty "RWxx.SIDx" departure heading.
  if (edit.departureLegCount != nullptr && *edit.departureLegCount > 0) {
    const int depStart =
        edit.departureLegStart != nullptr ? *edit.departureLegStart : 0;
    if (removedLegIndex >= depStart &&
        removedLegIndex < depStart + *edit.departureLegCount) {
      --(*edit.departureLegCount);
      if (*edit.departureLegCount <= 0) {
        if (edit.departureLegStart != nullptr) *edit.departureLegStart = 0;
        if (edit.departureHeaderLabel != nullptr) {
          edit.departureHeaderLabel->clear();
        }
        if (edit.loadedDeparture != nullptr) *edit.loadedDeparture = {};
      }
    } else if (removedLegIndex < depStart && edit.departureLegStart != nullptr) {
      --(*edit.departureLegStart);
    }
  }

  // Arrival block: same treatment as the departure.
  if (edit.arrivalLegCount != nullptr && *edit.arrivalLegCount > 0) {
    const int arrStart =
        edit.arrivalLegStart != nullptr ? *edit.arrivalLegStart : 0;
    if (removedLegIndex >= arrStart &&
        removedLegIndex < arrStart + *edit.arrivalLegCount) {
      --(*edit.arrivalLegCount);
      if (*edit.arrivalLegCount <= 0) {
        if (edit.arrivalLegStart != nullptr) *edit.arrivalLegStart = 0;
        if (edit.arrivalHeaderLabel != nullptr) edit.arrivalHeaderLabel->clear();
        if (edit.loadedArrival != nullptr) *edit.loadedArrival = {};
      }
    } else if (removedLegIndex < arrStart && edit.arrivalLegStart != nullptr) {
      --(*edit.arrivalLegStart);
    }
  }
}

bool fplRemoveLegAtIndex(FplRouteEdit& edit, int legIndex) {
  if (legIndex < 0 || legIndex >= static_cast<int>(edit.legs.size())) {
    return false;
  }
  edit.legs.erase(edit.legs.begin() + legIndex);
  fplRefreshDestinationFilledAfterRemove(
      edit, static_cast<int>(edit.legs.size()));
  fplAdjustApproachGroupingAfterRemove(edit, legIndex);
  fplAdjustTerminalProcedureGroupingAfterRemove(edit, legIndex);
  return true;
}

namespace {

// After erasing a contiguous [eraseStart, eraseStart+eraseCount) leg block,
// slide the start index of every other procedure block that sat after it so
// the surviving blocks still point at their legs in the shorter list. The
// removed block's own grouping is cleared separately by the caller.
void fplShiftProcedureBlocksAfterErase(FplRouteEdit& edit, int eraseStart,
                                       int eraseCount) {
  if (eraseCount <= 0) return;
  if (edit.approachLegCount > 0 && edit.approachLegStart >= eraseStart) {
    edit.approachLegStart -= eraseCount;
  }
  if (edit.departureLegStart != nullptr && edit.departureLegCount != nullptr &&
      *edit.departureLegCount > 0 && *edit.departureLegStart >= eraseStart) {
    *edit.departureLegStart -= eraseCount;
  }
  if (edit.arrivalLegStart != nullptr && edit.arrivalLegCount != nullptr &&
      *edit.arrivalLegCount > 0 && *edit.arrivalLegStart >= eraseStart) {
    *edit.arrivalLegStart -= eraseCount;
  }
}

// Erase the leg block [start, start+count) and refresh destination-filled.
// Returns false (without touching the legs) when the block is empty or stale
// (out of range for the current plan), so the caller still clears its grouping.
bool fplEraseProcedureLegBlock(FplRouteEdit& edit, int start, int count) {
  if (count <= 0) return false;
  if (start < 0 || start + count > static_cast<int>(edit.legs.size())) {
    return false;
  }
  edit.legs.erase(edit.legs.begin() + start,
                  edit.legs.begin() + start + count);
  fplShiftProcedureBlocksAfterErase(edit, start, count);
  fplRefreshDestinationFilledAfterRemove(edit,
                                         static_cast<int>(edit.legs.size()));
  return true;
}

}  // namespace

bool fplRemoveDeparture(FplRouteEdit& edit) {
  if (edit.departureLegStart == nullptr || edit.departureLegCount == nullptr ||
      *edit.departureLegCount <= 0) {
    return false;
  }
  const bool erased = fplEraseProcedureLegBlock(edit, *edit.departureLegStart,
                                                *edit.departureLegCount);
  *edit.departureLegStart = 0;
  *edit.departureLegCount = 0;
  if (edit.departureHeaderLabel != nullptr) edit.departureHeaderLabel->clear();
  if (edit.loadedDeparture != nullptr) *edit.loadedDeparture = {};
  return erased;
}

bool fplRemoveArrival(FplRouteEdit& edit) {
  if (edit.arrivalLegStart == nullptr || edit.arrivalLegCount == nullptr ||
      *edit.arrivalLegCount <= 0) {
    return false;
  }
  const bool erased = fplEraseProcedureLegBlock(edit, *edit.arrivalLegStart,
                                                *edit.arrivalLegCount);
  *edit.arrivalLegStart = 0;
  *edit.arrivalLegCount = 0;
  if (edit.arrivalHeaderLabel != nullptr) edit.arrivalHeaderLabel->clear();
  if (edit.loadedArrival != nullptr) *edit.loadedArrival = {};
  return erased;
}

bool fplRemoveApproach(FplRouteEdit& edit) {
  if (edit.approachLegCount <= 0) return false;
  const bool erased = fplEraseProcedureLegBlock(edit, edit.approachLegStart,
                                                edit.approachLegCount);
  edit.approachLegStart = 0;
  edit.approachLegCount = 0;
  if (edit.loadedApproach != nullptr) *edit.loadedApproach = {};
  if (edit.approachHeaderLabel != nullptr) edit.approachHeaderLabel->clear();
  return erased;
}

bool fplRemoveAirwaySegment(FplRouteEdit& edit, int anyLegIndex) {
  const int n = static_cast<int>(edit.legs.size());
  if (anyLegIndex < 0 || anyLegIndex >= n) return false;
  const std::string airway =
      edit.legs[static_cast<std::size_t>(anyLegIndex)].viaAirway;
  if (airway.empty()) return false;
  // Loaded-airway fixes are inserted as one contiguous block, so the segment is
  // the run of neighbouring legs that share this viaAirway tag.
  int start = anyLegIndex;
  while (start > 0 &&
         edit.legs[static_cast<std::size_t>(start - 1)].viaAirway == airway) {
    --start;
  }
  int end = anyLegIndex;
  while (end + 1 < n &&
         edit.legs[static_cast<std::size_t>(end + 1)].viaAirway == airway) {
    ++end;
  }
  const int count = end - start + 1;
  edit.legs.erase(edit.legs.begin() + start,
                  edit.legs.begin() + start + count);
  fplShiftProcedureBlocksAfterErase(edit, start, count);
  fplRefreshDestinationFilledAfterRemove(edit,
                                         static_cast<int>(edit.legs.size()));
  return true;
}

void fplClearFlightPlan(FplRouteEdit& edit) {
  edit.legs.clear();
  edit.cursorRow = 0;
  edit.destinationFilled = false;
  edit.approachLegStart = 0;
  edit.approachLegCount = 0;
  if (edit.loadedApproach != nullptr) *edit.loadedApproach = {};
  if (edit.approachHeaderLabel != nullptr) edit.approachHeaderLabel->clear();
  // Drop any loaded departure / arrival terminal procedure as well, so a
  // deleted flight plan does not leave the SID/STAR heading behind.
  if (edit.departureLegStart != nullptr) *edit.departureLegStart = 0;
  if (edit.departureLegCount != nullptr) *edit.departureLegCount = 0;
  if (edit.departureHeaderLabel != nullptr) edit.departureHeaderLabel->clear();
  if (edit.loadedDeparture != nullptr) *edit.loadedDeparture = {};
  if (edit.arrivalLegStart != nullptr) *edit.arrivalLegStart = 0;
  if (edit.arrivalLegCount != nullptr) *edit.arrivalLegCount = 0;
  if (edit.arrivalHeaderLabel != nullptr) edit.arrivalHeaderLabel->clear();
  if (edit.loadedArrival != nullptr) *edit.loadedArrival = {};
}

bool fplCommitWaypointIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                            const MapFeature& match, const std::string& ident,
                            int selectableCursorRow,
                            const std::string& approachAirport,
                            FplCursorLayout layout) {
  if (edit.approachLegCount > 0) {
    return fplCommitApproachIdent(edit, navSource, match, ident,
                                  selectableCursorRow, approachAirport);
  }
  if (fplEditUsesProcedureDisplay(edit)) {
    return fplCommitProcedureIdent(edit, navSource, match, ident,
                                   selectableCursorRow, approachAirport);
  }
  if (layout == FplCursorLayout::FlatLegList) {
    return fplCommitFlatIdent(edit, navSource, match, ident, selectableCursorRow);
  }
  return fplCommitSectionIdent(edit, navSource, match, ident, selectableCursorRow);
}

std::string fplIdentEntrySeedAtCursor(const FplRouteEdit& edit,
                                      const std::string& approachAirport,
                                      FplCursorLayout layout) {
  const int legIndex =
      fplCursorLegIndex(edit, approachAirport, layout);
  if (legIndex < 0 || legIndex >= static_cast<int>(edit.legs.size())) {
    return {};
  }
  const std::string initial = edit.legs[static_cast<std::size_t>(legIndex)].id;
  if (!initial.empty() && isFmsLatLonIdent(initial)) return {};
  return initial;
}

std::string fplLegDisplayRole(const MapLeg& leg) {
  if (!leg.procedureRole.empty()) return leg.procedureRole;
  if (leg.hold.active) return "hold";
  return {};
}

MapLeg resolveDirectToTargetLeg(const FmsWaypointEntry& entry, bool preservePlan,
                                int preserveLegIndex,
                                const std::vector<MapLeg>& planLegs) {
  if (preservePlan && preserveLegIndex >= 0 &&
      preserveLegIndex < static_cast<int>(planLegs.size())) {
    return planLegs[static_cast<std::size_t>(preserveLegIndex)];
  }
  if (entry.hasMatch && !entry.match.id.empty()) {
    const int idx = legIndexInPlan(planLegs, entry.match.id);
    if (idx >= 0) return planLegs[static_cast<std::size_t>(idx)];
  }
  MapLeg bare;
  bare.id = entry.match.id;
  bare.lat = entry.match.lat;
  bare.lon = entry.match.lon;
  return bare;
}

}  // namespace avionics
