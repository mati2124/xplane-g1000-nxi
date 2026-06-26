#include "avionics/FplRouteEdit.h"

#include <algorithm>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/NavMath.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

using pfd::FplDisplayRowKind;
using pfd::fplApproachInsertIndexForSelectable;
using pfd::fplApproachLegIndexForSelectable;
using pfd::fplApproachSelectableCount;
using pfd::fplApproachSelectableRow;
using pfd::fplEnrouteSelectableBase;
using pfd::buildFplSectionRows;
using pfd::fplApproachSelectableRowForLegIndex;
using pfd::fplFilterDuplicateLegSectionRows;
using pfd::fplSectionSelectableRowForLegIndex;
using pfd::fplInsertIndexForSectionRow;
using pfd::fplLegIndexForSectionRow;
using pfd::fplSectionSelectableCount;
using pfd::fplShowsDestinationBlankRow;

bool approachLayoutDestFilled(const FplRouteEdit& edit) {
  return fplApproachLayoutDestFilled(edit.destinationFilled, edit.approachLegStart);
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
  const int lastSection =
      std::max(0, fplSectionSelectableCount(legCount, edit.destinationFilled,
                                            edit.directToActive) - 1);
  const int legIndex = fplLegIndexForSectionRow(
      selectableCursorRow, legCount, edit.destinationFilled, edit.directToActive);
  const int row = fplInsertIndexForSectionRow(
      selectableCursorRow, legCount, edit.destinationFilled, edit.directToActive);

  if (selectableCursorRow == lastSection ||
      (fplShowsDestinationBlankRow(legCount, edit.destinationFilled) &&
       selectableCursorRow == lastSection - 1)) {
    edit.destinationFilled = true;
  } else if (selectableCursorRow == fplEnrouteSelectableBase(legCount) &&
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
  if (legIndex >= 0 && legIndex < legCount) {
    edit.legs[static_cast<std::size_t>(legIndex)] = leg;
  } else {
    edit.legs.insert(edit.legs.begin() + row, leg);
  }
  if (edit.legs.size() >= 3) edit.destinationFilled = true;
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

std::string airportIcaoBeforeIndex(const std::vector<MapLeg>& legs, int before) {
  for (int i = std::min(before, static_cast<int>(legs.size())) - 1; i >= 0; --i) {
    if (isAirportIdent(legs[static_cast<std::size_t>(i)].id)) {
      return legs[static_cast<std::size_t>(i)].id;
    }
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
        a[i].procedureRole != b[i].procedureRole) {
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
  if (approachStart <= 0 && legs.empty()) {
    return isAirportIdent(loadedApproachAirportIcao)
               ? loadedApproachAirportIcao
               : std::string();
  }
  std::string icao = airportIcaoBeforeIndex(legs, approachStart);
  if (!icao.empty()) return icao;
  icao = directToAirportIcao(map);
  if (!icao.empty()) return icao;
  if (map != nullptr) {
    icao = lastAirportInPlan(map->flightPlan);
    if (!icao.empty()) return icao;
  }
  if (isAirportIdent(loadedApproachAirportIcao)) return loadedApproachAirportIcao;
  return {};
}

int fplCursorLegIndex(const FplRouteEdit& edit,
                      const std::string& approachAirport,
                      FplCursorLayout layout) {
  const int legCount = static_cast<int>(edit.legs.size());
  const int sectionLegCount = fplEditSectionLegCount(edit);
  if (edit.approachLegCount > 0) {
    return fplApproachLegIndexForSelectable(
        edit.cursorRow, edit.legs, edit.approachLegStart, edit.approachLegCount,
        fplApproachBlankOriginSection(edit.legs, edit.approachLegStart,
                                      approachAirport),
        approachLayoutDestFilled(edit));
  }
  if (layout == FplCursorLayout::FlatLegList) {
    if (edit.cursorRow >= 0 && edit.cursorRow < legCount) return edit.cursorRow;
    return -1;
  }
  return fplLegIndexForSectionRow(edit.cursorRow, sectionLegCount,
                                  edit.destinationFilled, edit.directToActive);
}

int fplCursorSelectableLast(const FplRouteEdit& edit,
                            const std::string& approachAirport,
                            FplCursorLayout layout) {
  if (edit.approachLegCount > 0) {
    return fplApproachSelectableCount(
               edit.legs, edit.approachLegStart, edit.approachLegCount,
               fplApproachBlankOriginSection(edit.legs, edit.approachLegStart,
                                             approachAirport),
               approachLayoutDestFilled(edit)) -
           1;
  }
  if (layout == FplCursorLayout::FlatLegList) {
    return static_cast<int>(edit.legs.size());
  }
  return std::max(0, fplSectionSelectableCount(fplEditSectionLegCount(edit),
                                               edit.destinationFilled,
                                               edit.directToActive) -
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
  if (edit.approachLegCount > 0) {
    return fplApproachSelectableRowForLegIndex(
        activeLegIdx, edit.legs, edit.approachLegStart, edit.approachLegCount,
        fplApproachBlankOriginSection(edit.legs, edit.approachLegStart,
                                      approachAirport),
        approachLayoutDestFilled(edit));
  }
  if (layout == FplCursorLayout::FlatLegList) {
    return activeLegIdx;
  }
  const int legCount = fplEditSectionLegCount(edit);
  const bool directToPlanBody =
      edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0;
  const std::vector<pfd::FplSectionRow> sectionRows =
      fplFilterDuplicateLegSectionRows(
          buildFplSectionRows(legCount, edit.destinationFilled, directToPlanBody),
          edit.legs);
  return fplSectionSelectableRowForLegIndex(
      activeLegIdx, sectionRows, legCount, edit.destinationFilled,
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

bool fplRemoveLegAtIndex(FplRouteEdit& edit, int legIndex) {
  if (legIndex < 0 || legIndex >= static_cast<int>(edit.legs.size())) {
    return false;
  }
  edit.legs.erase(edit.legs.begin() + legIndex);
  fplRefreshDestinationFilledAfterRemove(
      edit, static_cast<int>(edit.legs.size()));
  fplAdjustApproachGroupingAfterRemove(edit, legIndex);
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

}  // namespace avionics
