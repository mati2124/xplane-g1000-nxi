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

    if (destinationFilled && layout.destLegIndex >= 0) {
      rows.push_back({FplSectionRow::Kind::EnrouteBlank, -1});
    }

  } else {

    rows.push_back({FplSectionRow::Kind::EnrouteBlank, -1});

  }

  if (layout.destLegIndex >= 0 && destinationFilled && legCount >= 2) {
    rows.push_back({FplSectionRow::Kind::DestinationLabel, -1});
    rows.push_back({FplSectionRow::Kind::Destination, layout.destLegIndex});
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
    case FplSectionRow::Kind::OriginBlank:
    case FplSectionRow::Kind::DestinationBlank:
      return false;
    case FplSectionRow::Kind::Origin:
      return sr.legIndex >= 0 || fplShowsOriginBlankRow(legCount);
    case FplSectionRow::Kind::Destination:
      return sr.legIndex >= 0 ||
             fplShowsDestinationBlankRow(legCount, destinationFilled);
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
  int slots = layout.enrouteCount > 0 ? layout.enrouteCount : 1;
  if (layout.enrouteCount > 0 && destinationFilled && layout.destLegIndex >= 0) {
    ++slots;
  }
  return slots;
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

  if (layout.enrouteCount > 0 && destinationFilled && layout.destLegIndex >= 0 &&
      enrouteSlot == layout.enrouteCount) {
    return layout.destLegIndex;
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

  if (layout.enrouteCount > 0 && destinationFilled && layout.destLegIndex >= 0 &&
      enrouteSlot == layout.enrouteCount) {
    return -1;
  }

  if (layout.enrouteCount > 0 && enrouteSlot >= 0 &&
      enrouteSlot < layout.enrouteCount) {

    return layout.enrouteFirst + enrouteSlot;

  }

  return -1;
}

inline int fplSectionSelectableRowForLegIndex(
    int legIndex, const std::vector<FplSectionRow>& sectionRows, int legCount,
    bool destinationFilled, bool directToActive = false) {
  if (legIndex < 0) return -1;
  int sel = 0;
  for (const FplSectionRow& sr : sectionRows) {
    if (!fplSectionRowIsSelectable(sr, legCount, destinationFilled)) continue;
    if (fplLegIndexForSectionRow(sel, legCount, destinationFilled,
                                 directToActive) == legIndex) {
      return sel;
    }
    ++sel;
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
  SepDash,
  ApproachHeader,
  ApproachLeg,
};



struct FplDisplayRow {

  FplDisplayRowKind kind = FplDisplayRowKind::Origin;

  int legIndex = -1;

};



inline std::vector<FplDisplayRow> buildFplApproachDisplayRows(

    const std::vector<MapLeg>& legs, int approachStart, int approachCount,

    bool blankOriginSection, bool destinationFilled) {

  std::vector<FplDisplayRow> rows;

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

        // Enroute blank slot is represented by the SepDash before the approach
        // header; do not show the separate template dash row here.
        enrouteBlock = true;

        break;

      case FplSectionRow::Kind::EnrouteLeg:

        if (sr.legIndex >= 0 &&
            fplHideLegForDuplicateIdent(legs, sr.legIndex)) {
          break;
        }
        rows.push_back({FplDisplayRowKind::EnrouteLeg, sr.legIndex});

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



inline bool fplApproachDisplayRowSelectable(FplDisplayRowKind kind) {

  switch (kind) {

    case FplDisplayRowKind::Origin:

    case FplDisplayRowKind::OriginBlank:

    case FplDisplayRowKind::EnrouteLeg:

    case FplDisplayRowKind::Destination:

    case FplDisplayRowKind::EnrouteBlank:

    case FplDisplayRowKind::SepDash:

    case FplDisplayRowKind::ApproachLeg:

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

    default:

      return approachStart;

  }

}



}  // namespace avionics::pfd

