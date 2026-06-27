#pragma once



#include <algorithm>

#include <vector>

#include "avionics/FlightPlanPersistence.h"



namespace avionics::pfd {



// PFD Active Flight Plan window (Pilot's Guide Fig. 5-48): the leg list is

// grouped under Origin, Enroute, and Destination headings rather than a flat

// list. The Enroute label row is display-only; the knob cursor steps the other

// rows.

struct FplSectionRow {

  enum class Kind { Origin, OriginBlank, EnrouteLabel, EnrouteLeg, EnrouteBlank,
                    DestinationLabel, Destination, DestinationBlank };

  Kind kind = Kind::Origin;

  int legIndex = -1;  // index into the legs vector, -1 for a blank slot

};



struct FplSectionLayout {

  int destLegIndex = -1;

  int enrouteFirst = 1;

  int enrouteCount = 0;

};



inline FplSectionLayout fplSectionLayout(int legCount, bool destinationFilled) {

  FplSectionLayout layout;

  if (legCount <= 0) return layout;

  if (legCount == 1) {
    if (destinationFilled) {
      layout.destLegIndex = 0;
    }
    return layout;
  }

  if (legCount == 2) {

    if (destinationFilled) {

      layout.destLegIndex = 1;

    } else {

      layout.enrouteCount = 1;

    }

    return layout;

  }

  if (!destinationFilled) {
    layout.enrouteCount = legCount - 1;
    return layout;
  }

  layout.destLegIndex = legCount - 1;

  layout.enrouteCount = legCount - 2;

  return layout;

}



// GPS Direct-To: navigation shows in the header; the FPL editor keeps the full
// blank template including an empty Destination section.
inline bool fplHidesDestinationSection(bool /*directToActive*/, int /*legCount*/) {
  return false;
}



inline std::vector<FplSectionRow> buildFplSectionRows(int legCount,

                                                      bool destinationFilled,

                                                      bool directToActive = false) {

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  const bool hideDestination = fplHidesDestinationSection(directToActive, legCount);

  std::vector<FplSectionRow> rows;

  rows.reserve(static_cast<std::size_t>(legCount) + 4);

  const bool destOnly = legCount == 1 && destinationFilled;
  rows.push_back({FplSectionRow::Kind::Origin,
                  destOnly ? -1 : (legCount > 0 ? 0 : -1)});
  if (legCount == 0) {
    rows.push_back({FplSectionRow::Kind::OriginBlank, -1});
  }

  rows.push_back({FplSectionRow::Kind::EnrouteLabel, -1});

  if (layout.enrouteCount > 0) {

    for (int i = 0; i < layout.enrouteCount; ++i) {

      rows.push_back({FplSectionRow::Kind::EnrouteLeg, layout.enrouteFirst + i});

    }

  }

  rows.push_back({FplSectionRow::Kind::EnrouteBlank, -1});

  if (layout.destLegIndex >= 0 && destinationFilled && legCount >= 2) {
    rows.push_back({FplSectionRow::Kind::DestinationLabel, -1});
    rows.push_back({FplSectionRow::Kind::Destination, layout.destLegIndex});
    rows.push_back({FplSectionRow::Kind::DestinationBlank, -1});
  } else if (!hideDestination) {
    rows.push_back({FplSectionRow::Kind::Destination, layout.destLegIndex});
    if (layout.destLegIndex < 0) {
      rows.push_back({FplSectionRow::Kind::DestinationBlank, -1});
    }
  }

  return rows;

}



inline bool fplShowsOriginBlankRow(int legCount) { return legCount == 0; }

inline bool fplShowsDestinationBlankRow(int legCount, bool destinationFilled) {
  return fplSectionLayout(legCount, destinationFilled).destLegIndex < 0;
}

// Knob cursor steps the editable slots, including the Origin and Destination
// fields (filled idents or the blank "Origin - ____" / "Destination - ____"
// entry rows) so the cursor can land on them to enter/edit the airports, like
// the real unit. The Enroute / Destination-RW__ section labels stay display-
// only. The blank Origin/Destination rows take the cursor stop that the
// separate dash rows used to carry, so the selectable count/order is preserved.
inline bool fplSectionRowIsSelectable(const FplSectionRow& sr, int legCount,
                                      bool destinationFilled) {
  switch (sr.kind) {
    case FplSectionRow::Kind::EnrouteLabel:
    case FplSectionRow::Kind::DestinationLabel:
      return false;
    case FplSectionRow::Kind::Origin:
      return sr.legIndex >= 0 || fplShowsOriginBlankRow(legCount);
    case FplSectionRow::Kind::Destination:
      return sr.legIndex >= 0 ||
             fplShowsDestinationBlankRow(legCount, destinationFilled);
    // The dashed "add a fix" rows under Origin and Destination are cursor stops
    // too: Origin blank inserts at the start of Enroute, Destination blank
    // appends after the destination (mirrors the trainer's blank entry lines).
    case FplSectionRow::Kind::OriginBlank:
    case FplSectionRow::Kind::DestinationBlank:
    case FplSectionRow::Kind::EnrouteBlank:
    case FplSectionRow::Kind::EnrouteLeg:
      return true;
    default:
      return false;
  }
}

// Filled destination: Destination - RW__ label, then airport ident (no trailing blank).
inline bool fplShowsDestinationLabelRow(int legCount, bool destinationFilled) {
  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);
  return layout.destLegIndex >= 0 && destinationFilled && legCount >= 2;
}

inline int fplEnrouteSelectableBase(int /*legCount*/) { return 1; }

inline int fplEnrouteSelectableSlots(int legCount, bool destinationFilled) {
  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);
  return layout.enrouteCount > 0 ? layout.enrouteCount + 1 : 1;
}

inline int fplSectionSelectableCount(int legCount, bool destinationFilled,
                                     bool directToActive = false) {
  int count = 0;
  for (const FplSectionRow& sr :
       buildFplSectionRows(legCount, destinationFilled, directToActive)) {
    if (fplSectionRowIsSelectable(sr, legCount, destinationFilled)) ++count;
  }
  return count;
}



inline int fplInsertIndexForSectionRow(int sectionRow, int legCount,

                                       bool destinationFilled,

                                       bool directToActive = false) {

  const int last =
      std::max(0, fplSectionSelectableCount(legCount, destinationFilled,
                                            directToActive) - 1);

  sectionRow = std::max(0, std::min(sectionRow, last));

  if (sectionRow == 0) return 0;

  if (sectionRow == last) {
    if (fplHidesDestinationSection(directToActive, legCount)) {
      return legCount;
    }
    const FplSectionLayout layout =
        fplSectionLayout(legCount, destinationFilled);
    if (fplShowsDestinationLabelRow(legCount, destinationFilled) &&
        layout.destLegIndex >= 0) {
      return layout.destLegIndex;
    }
    if (layout.destLegIndex >= 0 && destinationFilled) {
      return layout.destLegIndex;
    }
    return legCount;
  }

  if (fplShowsDestinationBlankRow(legCount, destinationFilled) &&
      sectionRow == last - 1) {
    return legCount;
  }

  if (fplShowsOriginBlankRow(legCount) && sectionRow == 0) {
    return 0;
  }

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  const int enrouteSlot = sectionRow - fplEnrouteSelectableBase(legCount);

  if (layout.enrouteCount > 0 && enrouteSlot == layout.enrouteCount) {
    if (destinationFilled && layout.destLegIndex >= 0) {
      return layout.destLegIndex;
    }
    return legCount;
  }

  if (layout.enrouteCount > 0 && enrouteSlot >= 0 &&
      enrouteSlot < layout.enrouteCount) {

    return layout.enrouteFirst + enrouteSlot;

  }

  if (legCount <= 0) return 0;

  if (legCount == 1) return 1;

  return layout.enrouteFirst;

}



inline int fplLegIndexForSectionRow(int sectionRow, int legCount,

                                    bool destinationFilled,

                                    bool directToActive = false) {

  const int last =
      std::max(0, fplSectionSelectableCount(legCount, destinationFilled,
                                            directToActive) - 1);

  sectionRow = std::max(0, std::min(sectionRow, last));

  if (sectionRow == 0) return legCount > 0 ? 0 : -1;

  if (sectionRow == last) {
    return fplSectionLayout(legCount, destinationFilled).destLegIndex;
  }

  if (fplShowsOriginBlankRow(legCount) && sectionRow == 1) return -1;

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  const int enrouteSlot = sectionRow - fplEnrouteSelectableBase(legCount);

  if (layout.enrouteCount > 0 && enrouteSlot == layout.enrouteCount) {
    return -1;
  }

  if (layout.enrouteCount > 0 && enrouteSlot >= 0 &&
      enrouteSlot < layout.enrouteCount) {

    return layout.enrouteFirst + enrouteSlot;

  }

  return -1;
}

// Keep the magenta active-leg marker visible while scrolling/editing a loaded
// approach whenever the active navigation leg is on the approach segment, or
// during GPS Direct-To to a fix on that approach.
inline bool fplPinActiveApproachLeg(bool approachLoaded, int activeLegIdx,
                                    int approachStart, int approachCount,
                                    bool localDraft, bool directToActive = false) {
  if (localDraft || activeLegIdx < 0) return false;
  if (directToActive) return true;
  if (!approachLoaded || approachCount <= 0) return false;
  return activeLegIdx >= approachStart &&
         activeLegIdx < approachStart + approachCount;
}

// Magenta active-leg flash on the active navigation row. Pinned during
// Direct-To on a loaded approach; otherwise follows the list cursor.
inline bool fplShowActiveLegHighlight(int activeLegIdx, int cursorLegIdx,
                                      int activeSelectableRow, int listCursorRow,
                                      bool pinActiveLeg = false) {
  if (pinActiveLeg) return true;
  if (activeLegIdx >= 0 && cursorLegIdx >= 0 &&
      activeLegIdx == cursorLegIdx) {
    return true;
  }
  return activeSelectableRow >= 0 &&
         listCursorRow == activeSelectableRow;
}

// List scroll offset: center on focusRow when possible; when pinnedRow is set
// expand or shift the window so both rows stay visible when they fit.
inline int fplListScrollFirst(int focusRow, int pinnedRow, int totalRows,
                              int visibleRows) {
  if (totalRows <= visibleRows || visibleRows <= 0) return 0;
  const int maxFirst = totalRows - visibleRows;
  int first = focusRow - visibleRows / 2;
  first = std::max(0, std::min(first, maxFirst));
  if (pinnedRow < 0 || pinnedRow == focusRow) return first;

  const int lo = std::min(focusRow, pinnedRow);
  const int hi = std::max(focusRow, pinnedRow);
  if (hi - lo + 1 > visibleRows) {
    if (pinnedRow < first) first = pinnedRow;
    else if (pinnedRow >= first + visibleRows) {
      first = pinnedRow - visibleRows + 1;
    }
    return std::max(0, std::min(first, maxFirst));
  }

  const int want = lo - (visibleRows - (hi - lo + 1)) / 2;
  return std::max(0, std::min(want, maxFirst));
}

inline bool fplShowListRowSelection(int selectableIdx, int listCursorRow,
                                    int activeSelectableRow, bool cursorOn) {
  if (selectableIdx != listCursorRow) return false;
  if (cursorOn) return true;
  return activeSelectableRow < 0 || listCursorRow != activeSelectableRow;
}

// Which section row carries the magenta active-leg highlight (Pilot's Guide
// Fig. 5-48). The vertical connector is drawn separately from origin to this
// row when the origin is filled.
inline bool fplSectionRowIsActiveDisplay(const FplSectionRow& sr, int legCount,

                                        bool destinationFilled,

                                        int activeLegIndex) {

  if (activeLegIndex < 0 || legCount <= 0) return false;

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  if (sr.kind == FplSectionRow::Kind::Origin && sr.legIndex >= 0 &&
      sr.legIndex == activeLegIndex) {

    return true;

  }

  if (sr.kind == FplSectionRow::Kind::EnrouteLeg &&

      sr.legIndex == activeLegIndex) {

    return true;

  }

  if (sr.kind == FplSectionRow::Kind::Destination &&

      activeLegIndex == layout.destLegIndex) {

    return true;

  }

  return false;

}



// Header origin ident: when flying a loaded approach, the header tracks the
// active navigation fix (e.g. IAF/FAF). When the enroute template hides a
// duplicated airport (blank origin section), fall back to the first approach fix.
inline std::string fplHeaderOriginIdent(const std::vector<MapLeg>& legs,
                                        int approachLegStart,
                                        int approachLegCount,
                                        bool approachLoaded,
                                        bool blankOriginSection,
                                        bool destOnlyPlan,
                                        const std::string& activeToIdent) {
  if (destOnlyPlan) return {};
  if (approachLoaded && approachLegCount > 0 && !activeToIdent.empty()) {
    const int activeIdx = fplActiveLegIndexInPlan(legs, activeToIdent);
    const int approachEnd = approachLegStart + approachLegCount;
    if (activeIdx >= approachLegStart && activeIdx < approachEnd) {
      return activeToIdent;
    }
  }
  if (blankOriginSection && approachLegStart >= 0 &&
      approachLegStart < static_cast<int>(legs.size())) {
    return legs[static_cast<std::size_t>(approachLegStart)].id;
  }
  if (legs.empty()) return {};
  return legs.front().id;
}

// Header destination ident: the route destination airport, not the active leg
// or an enroute fix. When an approach is loaded, use its airport; otherwise
// use the last leg of the enroute plan segment (before approach legs).
inline std::string fplHeaderDestinationIdent(const std::vector<MapLeg>& legs,

                                             bool destinationFilled,

                                             int approachLegStart,

                                             bool approachLoaded,

                                             const std::string& approachAirport) {

  if (approachLoaded && !approachAirport.empty()) {
    return approachAirport;
  }

  const int sectionLegCount =

      approachLoaded ? approachLegStart : static_cast<int>(legs.size());

  if (sectionLegCount < 1 ||

      sectionLegCount > static_cast<int>(legs.size())) {

    return {};

  }

  if (sectionLegCount == 1) {
    if (!destinationFilled) return {};
    return legs[0].id;
  }

  if (sectionLegCount == 2) {

    if (!destinationFilled) return {};

    return legs[1].id;

  }

  return legs[static_cast<std::size_t>(sectionLegCount - 1)].id;

}



// PFD FPL body rows when a loaded approach follows the enroute template
// (trainer: Origin, Enroute, destination, sep dash, approach header, approach legs).
enum class FplDisplayRowKind {
  Origin,
  OriginBlank,
  EnrouteLabel,
  EnrouteBlank,
  EnrouteLeg,
  Destination,
  DestinationLabel,
  DestinationBlank,
  DepartureHeader,
  DepartureLeg,
  ArrivalHeader,
  ArrivalLeg,
  SepDash,
  ApproachHeader,
  ApproachLeg,
  // Published hold (HILPT/hold-in-lieu) shown on its own line below its fix,
  // matching the trainer FPL list ("HOLD" with the inbound course / leg length).
  // legIndex points at the parent fix leg that carries the hold.
  Hold,
};



struct FplDisplayRow {

  FplDisplayRowKind kind = FplDisplayRowKind::Origin;

  int legIndex = -1;

};



inline std::vector<FplDisplayRow> buildFplApproachDisplayRows(

    const std::vector<MapLeg>& legs, int approachStart, int approachCount,

    bool blankOriginSection, bool destinationFilled) {

  std::vector<FplDisplayRow> rows;

  // A published hold renders as its own "HOLD" line directly below its fix.
  const auto pushHoldRowIfPresent = [&](int legIdx) {
    if (legIdx < 0 || legIdx >= static_cast<int>(legs.size())) return;
    if (legs[static_cast<std::size_t>(legIdx)].hold.active) {
      rows.push_back({FplDisplayRowKind::Hold, legIdx});
    }
  };

  const int sectionLegCount =
      blankOriginSection ? 0 : fplEnrouteDisplayLegCount(legs, approachStart);

  const std::vector<FplSectionRow> sectionRows =

      buildFplSectionRows(sectionLegCount, destinationFilled);

  bool enrouteBlock = false;

  for (const FplSectionRow& sr : sectionRows) {

    switch (sr.kind) {

      case FplSectionRow::Kind::Origin:

        if (sr.legIndex >= 0 &&
            fplHideLegForDuplicateIdent(legs, sr.legIndex)) {
          break;
        }
        rows.push_back({FplDisplayRowKind::Origin, sr.legIndex});

        break;

      case FplSectionRow::Kind::OriginBlank:

        rows.push_back({FplDisplayRowKind::OriginBlank, -1});

        break;

      case FplSectionRow::Kind::EnrouteLabel:

        rows.push_back({FplDisplayRowKind::EnrouteLabel, -1});

        enrouteBlock = true;

        break;

      case FplSectionRow::Kind::EnrouteBlank:

        // The enroute add-fix slot is represented by the SepDash before the
        // approach header; do not show a separate template dash row here.
        enrouteBlock = true;

        break;

      case FplSectionRow::Kind::EnrouteLeg:

        if (sr.legIndex >= 0 &&
            fplHideLegForDuplicateIdent(legs, sr.legIndex)) {
          break;
        }
        rows.push_back({FplDisplayRowKind::EnrouteLeg, sr.legIndex});
        pushHoldRowIfPresent(sr.legIndex);

        enrouteBlock = true;

        break;

      case FplSectionRow::Kind::DestinationLabel:
      case FplSectionRow::Kind::Destination:
      case FplSectionRow::Kind::DestinationBlank:

        // Destination is shown in the approach header when a procedure is loaded.
        break;

      default:

        break;

    }

  }

  if (enrouteBlock) {

    rows.push_back({FplDisplayRowKind::SepDash, -1});

  }

  rows.push_back({FplDisplayRowKind::ApproachHeader, -1});

  for (int i = 0; i < approachCount; ++i) {

    const int legIdx = approachStart + i;
    if (fplHideLegForDuplicateIdent(legs, legIdx)) continue;
    rows.push_back({FplDisplayRowKind::ApproachLeg, legIdx});
    pushHoldRowIfPresent(legIdx);

  }

  return rows;

}



inline bool fplSectionRowLegIsHidden(const FplSectionRow& sr,
                                     const std::vector<MapLeg>& legs) {
  if (sr.legIndex < 0) return false;
  switch (sr.kind) {
    case FplSectionRow::Kind::Origin:
    case FplSectionRow::Kind::EnrouteLeg:
    case FplSectionRow::Kind::Destination:
      return fplHideLegForDuplicateIdent(legs, sr.legIndex);
    default:
      return false;
  }
}



inline std::vector<FplSectionRow> fplFilterDuplicateLegSectionRows(
    const std::vector<FplSectionRow>& rows,
    const std::vector<MapLeg>& legs) {
  std::vector<FplSectionRow> out;
  out.reserve(rows.size());
  for (const FplSectionRow& sr : rows) {
    if (fplSectionRowLegIsHidden(sr, legs)) continue;
    out.push_back(sr);
  }
  return out;
}

inline std::vector<FplSectionRow> fplFilteredSectionRows(
    int legCount, bool destinationFilled, bool directToActive,
    const std::vector<MapLeg>& legs) {
  return fplFilterDuplicateLegSectionRows(
      buildFplSectionRows(legCount, destinationFilled, directToActive), legs);
}

inline int fplFilteredSectionSelectableCount(
    const std::vector<FplSectionRow>& sectionRows, int legCount,
    bool destinationFilled) {
  int count = 0;
  for (const FplSectionRow& sr : sectionRows) {
    if (fplSectionRowIsSelectable(sr, legCount, destinationFilled)) ++count;
  }
  return count;
}

inline const FplSectionRow* fplSectionSelectableRow(
    int selectableRow, const std::vector<FplSectionRow>& sectionRows,
    int legCount, bool destinationFilled) {
  int sel = 0;
  for (const FplSectionRow& sr : sectionRows) {
    if (!fplSectionRowIsSelectable(sr, legCount, destinationFilled)) continue;
    if (sel == selectableRow) return &sr;
    ++sel;
  }
  return nullptr;
}

inline bool fplSectionRowCarriesLegIndex(FplSectionRow::Kind kind) {
  switch (kind) {
    case FplSectionRow::Kind::OriginBlank:
    case FplSectionRow::Kind::EnrouteBlank:
    case FplSectionRow::Kind::DestinationBlank:
      return false;
    default:
      return true;
  }
}

inline int fplSectionLegIndexForSelectable(
    int selectableRow, const std::vector<FplSectionRow>& sectionRows,
    int legCount, bool destinationFilled) {
  const FplSectionRow* sr = fplSectionSelectableRow(
      selectableRow, sectionRows, legCount, destinationFilled);
  if (sr == nullptr || !fplSectionRowCarriesLegIndex(sr->kind)) return -1;
  return sr->legIndex;
}

inline int fplSectionInsertIndexForSelectable(
    int selectableRow, const std::vector<FplSectionRow>& sectionRows,
    int legCount, bool destinationFilled) {
  const FplSectionRow* sr = fplSectionSelectableRow(
      selectableRow, sectionRows, legCount, destinationFilled);
  if (sr == nullptr) return legCount;
  switch (sr->kind) {
    case FplSectionRow::Kind::Origin:
      return sr->legIndex >= 0 ? sr->legIndex : 0;
    case FplSectionRow::Kind::OriginBlank:
      return 0;
    case FplSectionRow::Kind::EnrouteLeg:
      return sr->legIndex >= 0 ? sr->legIndex : legCount;
    case FplSectionRow::Kind::EnrouteBlank:
      return legCount;
    case FplSectionRow::Kind::Destination:
      if (sr->legIndex >= 0) return sr->legIndex;
      return fplSectionLayout(legCount, destinationFilled).destLegIndex >= 0
                 ? fplSectionLayout(legCount, destinationFilled).destLegIndex
                 : legCount;
    case FplSectionRow::Kind::DestinationBlank:
      return legCount;
    default:
      return legCount;
  }
}

inline int fplSectionSelectableRowForLegIndex(
    int legIndex, const std::vector<FplSectionRow>& sectionRows, int legCount,
    bool destinationFilled, bool /*directToActive*/ = false) {
  if (legIndex < 0) return -1;
  int sel = 0;
  for (const FplSectionRow& sr : sectionRows) {
    if (!fplSectionRowIsSelectable(sr, legCount, destinationFilled)) continue;
    if (fplSectionRowCarriesLegIndex(sr.kind) && sr.legIndex == legIndex) {
      return sel;
    }
    ++sel;
  }
  return -1;
}



inline bool fplApproachDisplayRowSelectable(FplDisplayRowKind kind) {

  switch (kind) {

    case FplDisplayRowKind::Origin:

    case FplDisplayRowKind::OriginBlank:

    case FplDisplayRowKind::EnrouteLeg:

    case FplDisplayRowKind::Destination:

    case FplDisplayRowKind::DestinationBlank:

    case FplDisplayRowKind::EnrouteBlank:

    case FplDisplayRowKind::SepDash:

    case FplDisplayRowKind::ApproachLeg:

    case FplDisplayRowKind::Hold:

    case FplDisplayRowKind::DepartureLeg:

    case FplDisplayRowKind::ArrivalLeg:

      return true;

    default:

      return false;

  }

}



inline std::vector<FplDisplayRow> fplApproachDisplayRowList(

    const std::vector<MapLeg>& legs, int approachStart, int approachCount,

    bool blankOriginSection, bool destinationFilled) {

  return buildFplApproachDisplayRows(legs, approachStart, approachCount,

                                     blankOriginSection, destinationFilled);

}



inline int fplApproachSelectableCount(const std::vector<MapLeg>& legs,

                                      int approachStart, int approachCount,

                                      bool blankOriginSection,

                                      bool destinationFilled) {

  int count = 0;

  for (const FplDisplayRow& dr :

       fplApproachDisplayRowList(legs, approachStart, approachCount,

                                 blankOriginSection, destinationFilled)) {

    if (fplApproachDisplayRowSelectable(dr.kind)) ++count;

  }

  return count;

}



inline const FplDisplayRow* fplApproachSelectableRow(

    int selectableRow, const std::vector<MapLeg>& legs, int approachStart,

    int approachCount, bool blankOriginSection, bool destinationFilled) {

  static FplDisplayRow scratch;

  int sel = 0;

  for (const FplDisplayRow& dr :

       fplApproachDisplayRowList(legs, approachStart, approachCount,

                                 blankOriginSection, destinationFilled)) {

    if (!fplApproachDisplayRowSelectable(dr.kind)) continue;

    if (sel == selectableRow) return &dr;

    ++sel;

  }

  scratch = {};

  return &scratch;

}



inline int fplApproachDisplayRowIndexForSelectable(

    int selectableRow, const std::vector<MapLeg>& legs, int approachStart,

    int approachCount, bool blankOriginSection, bool destinationFilled) {

  int sel = 0;

  const auto rows = fplApproachDisplayRowList(legs, approachStart, approachCount,

                                              blankOriginSection,

                                              destinationFilled);

  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {

    if (!fplApproachDisplayRowSelectable(rows[static_cast<std::size_t>(i)].kind)) {

      continue;

    }

    if (sel == selectableRow) return i;

    ++sel;

  }

  return 0;

}

inline int fplApproachDisplayRowIndexForLegIndex(
    int legIndex, const std::vector<MapLeg>& legs, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  if (legIndex < 0) return -1;
  const auto rows =
      fplApproachDisplayRowList(legs, approachStart, approachCount,
                                blankOriginSection, destinationFilled);
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    if (rows[static_cast<std::size_t>(i)].legIndex == legIndex) return i;
  }
  return -1;
}

inline int fplApproachDisplayRowIndexForHoldLegIndex(
    int legIndex, const std::vector<MapLeg>& legs, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  if (legIndex < 0) return -1;
  const auto rows =
      fplApproachDisplayRowList(legs, approachStart, approachCount,
                                blankOriginSection, destinationFilled);
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    const FplDisplayRow& dr = rows[static_cast<std::size_t>(i)];
    if (dr.kind == FplDisplayRowKind::Hold && dr.legIndex == legIndex) {
      return i;
    }
  }
  return -1;
}



inline int fplApproachLegIndexForSelectable(int selectableRow,

                                            const std::vector<MapLeg>& legs,

                                            int approachStart,

                                            int approachCount,

                                            bool blankOriginSection,

                                            bool destinationFilled) {

  const FplDisplayRow* dr = fplApproachSelectableRow(

      selectableRow, legs, approachStart, approachCount, blankOriginSection,

      destinationFilled);

  if (dr == nullptr || !fplApproachDisplayRowSelectable(dr->kind)) return -1;

  if (dr->kind == FplDisplayRowKind::OriginBlank ||
      dr->kind == FplDisplayRowKind::EnrouteBlank ||
      dr->kind == FplDisplayRowKind::SepDash) {
    return -1;
  }

  return dr->legIndex;

}

inline int fplApproachSelectableRowForLegIndex(
    int legIndex, const std::vector<MapLeg>& legs, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  if (legIndex < 0) return -1;
  const int last = fplApproachSelectableCount(
                       legs, approachStart, approachCount, blankOriginSection,
                       destinationFilled) -
                   1;
  for (int sel = 0; sel <= last; ++sel) {
    if (fplApproachLegIndexForSelectable(sel, legs, approachStart, approachCount,
                                         blankOriginSection,
                                         destinationFilled) == legIndex) {
      return sel;
    }
  }
  return -1;
}

// Selectable row index for the HOLD line beneath a fix (not the fix row itself).
inline int fplApproachSelectableRowForHoldLegIndex(
    int legIndex, const std::vector<MapLeg>& legs, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  if (legIndex < 0) return -1;
  const int last = fplApproachSelectableCount(
                       legs, approachStart, approachCount, blankOriginSection,
                       destinationFilled) -
                   1;
  for (int sel = 0; sel <= last; ++sel) {
    const FplDisplayRow* dr = fplApproachSelectableRow(
        sel, legs, approachStart, approachCount, blankOriginSection,
        destinationFilled);
    if (dr != nullptr && dr->kind == FplDisplayRowKind::Hold &&
        dr->legIndex == legIndex) {
      return sel;
    }
  }
  return -1;
}

inline int fplApproachInsertIndexForSelectable(int selectableRow,

                                               const std::vector<MapLeg>& legs,

                                               int approachStart,

                                               int approachCount,

                                               int legCount,

                                               bool blankOriginSection,

                                               bool destinationFilled) {

  const FplDisplayRow* dr = fplApproachSelectableRow(

      selectableRow, legs, approachStart, approachCount, blankOriginSection,

      destinationFilled);

  if (dr == nullptr) return approachStart;

  switch (dr->kind) {

    case FplDisplayRowKind::Origin:

    case FplDisplayRowKind::OriginBlank:

      return 0;

    case FplDisplayRowKind::EnrouteBlank:

      return approachStart;

    case FplDisplayRowKind::SepDash:

      return approachStart;

    case FplDisplayRowKind::Destination:

      return dr->legIndex >= 0 ? dr->legIndex : legCount;

    case FplDisplayRowKind::EnrouteLeg:

      return dr->legIndex >= 0 ? dr->legIndex : approachStart;

    case FplDisplayRowKind::ApproachLeg:

      return dr->legIndex >= 0 ? dr->legIndex : approachStart;

    case FplDisplayRowKind::Hold:

      return dr->legIndex >= 0 ? dr->legIndex : approachStart;

    default:

      return approachStart;

  }

}



inline bool fplLegInProcedureBlock(int legIndex, int blockStart, int blockCount) {
  return blockCount > 0 && legIndex >= blockStart &&
         legIndex < blockStart + blockCount;
}



inline std::vector<int> fplProcedureEnrouteLegIndices(
    int legCount, int depStart, int depCount, int arrStart, int arrCount,
    int approachStart, int approachCount, bool destinationFilled) {
  std::vector<int> out;
  if (legCount < 2) return out;
  const int lastEnroute = destinationFilled ? legCount - 2 : legCount - 1;
  for (int i = 1; i <= lastEnroute; ++i) {
    if (fplLegInProcedureBlock(i, depStart, depCount)) continue;
    if (fplLegInProcedureBlock(i, arrStart, arrCount)) continue;
    if (fplLegInProcedureBlock(i, approachStart, approachCount)) continue;
    out.push_back(i);
  }
  return out;
}



inline bool fplUsesProcedureDisplayRows(const std::string& departureHeader,
                                         int departureCount,
                                         const std::string& arrivalHeader,
                                         int arrivalCount, int approachCount) {
  return !departureHeader.empty() || departureCount > 0 ||
         !arrivalHeader.empty() || arrivalCount > 0 || approachCount > 0;
}



inline std::vector<FplDisplayRow> buildFplProcedureDisplayRows(
    const std::vector<MapLeg>& legs, int depStart, int depCount,
    const std::string& departureHeader, int arrStart, int arrCount,
    const std::string& arrivalHeader, int approachStart, int approachCount,
    bool blankOriginSection, bool destinationFilled) {
  if (departureHeader.empty() && depCount <= 0 && arrivalHeader.empty() &&
      arrCount <= 0 && approachCount > 0) {
    return buildFplApproachDisplayRows(legs, approachStart, approachCount,
                                      blankOriginSection, destinationFilled);
  }

  if (departureHeader.empty() && depCount <= 0 && arrivalHeader.empty() &&
      arrCount <= 0 && approachCount <= 0) {
    return {};
  }

  std::vector<FplDisplayRow> rows;
  const auto pushHoldRowIfPresent = [&](int legIdx) {
    if (legIdx < 0 || legIdx >= static_cast<int>(legs.size())) return;
    if (legs[static_cast<std::size_t>(legIdx)].hold.active) {
      rows.push_back({FplDisplayRowKind::Hold, legIdx});
    }
  };

  const bool hasDeparture = !departureHeader.empty() || depCount > 0;
  const bool hasArrival = !arrivalHeader.empty() || arrCount > 0;

  if (hasDeparture) {
    if (!departureHeader.empty()) {
      rows.push_back({FplDisplayRowKind::DepartureHeader, -1});
    }
    for (int i = 0; i < depCount; ++i) {
      const int legIdx = depStart + i;
      if (fplHideLegForDuplicateIdent(legs, legIdx)) continue;
      rows.push_back({FplDisplayRowKind::DepartureLeg, legIdx});
      pushHoldRowIfPresent(legIdx);
    }
  } else if (!blankOriginSection) {
    rows.push_back({FplDisplayRowKind::Origin, legs.empty() ? -1 : 0});
    if (legs.empty()) {
      rows.push_back({FplDisplayRowKind::OriginBlank, -1});
    }
  }

  rows.push_back({FplDisplayRowKind::EnrouteLabel, -1});
  const std::vector<int> enrouteLegs = fplProcedureEnrouteLegIndices(
      static_cast<int>(legs.size()), depStart, depCount, arrStart, arrCount,
      approachStart, approachCount, destinationFilled);
  for (const int legIdx : enrouteLegs) {
    if (fplHideLegForDuplicateIdent(legs, legIdx)) continue;
    rows.push_back({FplDisplayRowKind::EnrouteLeg, legIdx});
    pushHoldRowIfPresent(legIdx);
  }
  // The enroute add-fix slot: a standalone blank dash row only when no
  // arrival/approach follows (otherwise the SepDash separator is that slot).
  if (!hasArrival && approachCount <= 0) {
    rows.push_back({FplDisplayRowKind::EnrouteBlank, -1});
  }

  if (hasArrival) {
    rows.push_back({FplDisplayRowKind::SepDash, -1});
    if (!arrivalHeader.empty()) {
      rows.push_back({FplDisplayRowKind::ArrivalHeader, -1});
    }
    for (int i = 0; i < arrCount; ++i) {
      const int legIdx = arrStart + i;
      if (fplHideLegForDuplicateIdent(legs, legIdx)) continue;
      rows.push_back({FplDisplayRowKind::ArrivalLeg, legIdx});
      pushHoldRowIfPresent(legIdx);
    }
  } else if (approachCount <= 0) {
    if (destinationFilled && legs.size() >= 2) {
      const FplSectionLayout layout =
          fplSectionLayout(static_cast<int>(legs.size()), destinationFilled);
      if (layout.destLegIndex >= 0) {
        rows.push_back({FplDisplayRowKind::DestinationLabel, -1});
        rows.push_back({FplDisplayRowKind::Destination, layout.destLegIndex});
        rows.push_back({FplDisplayRowKind::DestinationBlank, -1});
      }
    } else {
      rows.push_back({FplDisplayRowKind::Destination, -1});
      rows.push_back({FplDisplayRowKind::DestinationBlank, -1});
    }
  }

  if (approachCount > 0) {
    rows.push_back({FplDisplayRowKind::SepDash, -1});
    rows.push_back({FplDisplayRowKind::ApproachHeader, -1});
    for (int i = 0; i < approachCount; ++i) {
      const int legIdx = approachStart + i;
      if (fplHideLegForDuplicateIdent(legs, legIdx)) continue;
      rows.push_back({FplDisplayRowKind::ApproachLeg, legIdx});
      pushHoldRowIfPresent(legIdx);
    }
  }

  return rows;
}



inline bool fplProcedureDisplayRowSelectable(FplDisplayRowKind kind) {
  return fplApproachDisplayRowSelectable(kind);
}



inline std::vector<FplDisplayRow> fplProcedureDisplayRowList(
    const std::vector<MapLeg>& legs, int depStart, int depCount,
    const std::string& departureHeader, int arrStart, int arrCount,
    const std::string& arrivalHeader, int approachStart, int approachCount,
    bool blankOriginSection, bool destinationFilled) {
  return buildFplProcedureDisplayRows(
      legs, depStart, depCount, departureHeader, arrStart, arrCount,
      arrivalHeader, approachStart, approachCount, blankOriginSection,
      destinationFilled);
}



inline int fplProcedureSelectableCount(const std::vector<MapLeg>& legs,
                                       int depStart, int depCount,
                                       const std::string& departureHeader,
                                       int arrStart, int arrCount,
                                       const std::string& arrivalHeader,
                                       int approachStart, int approachCount,
                                       bool blankOriginSection,
                                       bool destinationFilled) {
  int count = 0;
  for (const FplDisplayRow& dr : fplProcedureDisplayRowList(
           legs, depStart, depCount, departureHeader, arrStart, arrCount,
           arrivalHeader, approachStart, approachCount, blankOriginSection,
           destinationFilled)) {
    if (fplProcedureDisplayRowSelectable(dr.kind)) ++count;
  }
  return count;
}



inline const FplDisplayRow* fplProcedureSelectableRow(
    int selectableRow, const std::vector<MapLeg>& legs, int depStart,
    int depCount, const std::string& departureHeader, int arrStart,
    int arrCount, const std::string& arrivalHeader, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  static FplDisplayRow scratch;
  int sel = 0;
  for (const FplDisplayRow& dr : fplProcedureDisplayRowList(
           legs, depStart, depCount, departureHeader, arrStart, arrCount,
           arrivalHeader, approachStart, approachCount, blankOriginSection,
           destinationFilled)) {
    if (!fplProcedureDisplayRowSelectable(dr.kind)) continue;
    if (sel == selectableRow) return &dr;
    ++sel;
  }
  scratch = {};
  return &scratch;
}



inline int fplProcedureLegIndexForSelectable(
    int selectableRow, const std::vector<MapLeg>& legs, int depStart,
    int depCount, const std::string& departureHeader, int arrStart,
    int arrCount, const std::string& arrivalHeader, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  const FplDisplayRow* dr = fplProcedureSelectableRow(
      selectableRow, legs, depStart, depCount, departureHeader, arrStart,
      arrCount, arrivalHeader, approachStart, approachCount, blankOriginSection,
      destinationFilled);
  if (dr == nullptr || !fplProcedureDisplayRowSelectable(dr->kind)) return -1;
  if (dr->kind == FplDisplayRowKind::OriginBlank ||
      dr->kind == FplDisplayRowKind::EnrouteBlank ||
      dr->kind == FplDisplayRowKind::SepDash) {
    return -1;
  }
  return dr->legIndex;
}



inline int fplProcedureSelectableRowForLegIndex(
    int legIndex, const std::vector<MapLeg>& legs, int depStart, int depCount,
    const std::string& departureHeader, int arrStart, int arrCount,
    const std::string& arrivalHeader, int approachStart, int approachCount,
    bool blankOriginSection, bool destinationFilled) {
  if (legIndex < 0) return -1;
  const int last = fplProcedureSelectableCount(
                       legs, depStart, depCount, departureHeader, arrStart,
                       arrCount, arrivalHeader, approachStart, approachCount,
                       blankOriginSection, destinationFilled) -
                   1;
  for (int sel = 0; sel <= last; ++sel) {
    if (fplProcedureLegIndexForSelectable(
            sel, legs, depStart, depCount, departureHeader, arrStart, arrCount,
            arrivalHeader, approachStart, approachCount, blankOriginSection,
            destinationFilled) == legIndex) {
      return sel;
    }
  }
  return -1;
}



inline int fplProcedureDisplayRowIndexForSelectable(
    int selectableRow, const std::vector<MapLeg>& legs, int depStart,
    int depCount, const std::string& departureHeader, int arrStart,
    int arrCount, const std::string& arrivalHeader, int approachStart,
    int approachCount, bool blankOriginSection, bool destinationFilled) {
  int sel = 0;
  const auto rows = fplProcedureDisplayRowList(
      legs, depStart, depCount, departureHeader, arrStart, arrCount,
      arrivalHeader, approachStart, approachCount, blankOriginSection,
      destinationFilled);
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    if (!fplProcedureDisplayRowSelectable(rows[static_cast<std::size_t>(i)].kind)) {
      continue;
    }
    if (sel == selectableRow) return i;
    ++sel;
  }
  return 0;
}



inline int fplProcedureInsertIndexForSelectable(
    int selectableRow, const std::vector<MapLeg>& legs, int depStart,
    int depCount, const std::string& departureHeader, int arrStart,
    int arrCount, const std::string& arrivalHeader, int approachStart,
    int approachCount, int legCount, bool blankOriginSection,
    bool destinationFilled) {
  const FplDisplayRow* dr = fplProcedureSelectableRow(
      selectableRow, legs, depStart, depCount, departureHeader, arrStart,
      arrCount, arrivalHeader, approachStart, approachCount, blankOriginSection,
      destinationFilled);
  if (dr == nullptr) return legCount;
  switch (dr->kind) {
    case FplDisplayRowKind::Origin:
    case FplDisplayRowKind::OriginBlank:
      return 0;
    case FplDisplayRowKind::DepartureLeg:
      return dr->legIndex >= 0 ? dr->legIndex : depStart;
    case FplDisplayRowKind::EnrouteLeg:
      return dr->legIndex >= 0 ? dr->legIndex : legCount;
    case FplDisplayRowKind::EnrouteBlank: {
      const std::vector<int> enrouteLegs = fplProcedureEnrouteLegIndices(
          legCount, depStart, depCount, arrStart, arrCount, approachStart,
          approachCount, destinationFilled);
      if (!enrouteLegs.empty()) return enrouteLegs.back() + 1;
      return depCount > 0 ? depStart + depCount : 1;
    }
    case FplDisplayRowKind::Destination:
      if (dr->legIndex >= 0) return dr->legIndex;
      return legCount;
    case FplDisplayRowKind::ArrivalLeg:
      return dr->legIndex >= 0 ? dr->legIndex : legCount;
    case FplDisplayRowKind::ApproachLeg:
      return dr->legIndex >= 0 ? dr->legIndex : approachStart;
    case FplDisplayRowKind::Hold:
      return dr->legIndex >= 0 ? dr->legIndex : legCount;
    case FplDisplayRowKind::SepDash: {
      const std::vector<int> enrouteLegs = fplProcedureEnrouteLegIndices(
          legCount, depStart, depCount, arrStart, arrCount, approachStart,
          approachCount, destinationFilled);
      if (!enrouteLegs.empty()) return enrouteLegs.back() + 1;
      return approachStart > 0 ? approachStart : legCount;
    }
    default:
      return legCount;
  }
}



}  // namespace avionics::pfd

