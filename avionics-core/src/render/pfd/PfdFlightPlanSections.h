#pragma once



#include <algorithm>

#include <vector>



namespace avionics::pfd {



// PFD Active Flight Plan window (Pilot's Guide Fig. 5-48): the leg list is

// grouped under Origin, Enroute, and Destination headings rather than a flat

// list. The Enroute label row is display-only; the knob cursor steps the other

// rows.

struct FplSectionRow {

  enum class Kind { Origin, EnrouteLabel, EnrouteLeg, EnrouteBlank, Destination };

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

  if (legCount <= 1) return layout;

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



inline std::vector<FplSectionRow> buildFplSectionRows(int legCount,

                                                      bool destinationFilled) {

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  std::vector<FplSectionRow> rows;

  rows.reserve(static_cast<std::size_t>(legCount) + 4);

  rows.push_back({FplSectionRow::Kind::Origin, legCount > 0 ? 0 : -1});

  rows.push_back({FplSectionRow::Kind::EnrouteLabel, -1});

  if (layout.enrouteCount > 0) {

    for (int i = 0; i < layout.enrouteCount; ++i) {

      rows.push_back({FplSectionRow::Kind::EnrouteLeg, layout.enrouteFirst + i});

    }

  } else {

    rows.push_back({FplSectionRow::Kind::EnrouteBlank, -1});

  }

  rows.push_back({FplSectionRow::Kind::Destination, layout.destLegIndex});

  return rows;

}



inline int fplSectionSelectableCount(int legCount) {

  const int middle = legCount >= 3 ? legCount - 2 : (legCount == 2 ? 1 : 0);

  const int enrouteSlots = middle > 0 ? middle : 1;

  return 2 + enrouteSlots;

}



inline int fplInsertIndexForSectionRow(int sectionRow, int legCount,

                                       bool destinationFilled) {

  const int last = std::max(0, fplSectionSelectableCount(legCount) - 1);

  sectionRow = std::max(0, std::min(sectionRow, last));

  if (sectionRow == 0) return 0;

  if (sectionRow == last) return legCount;

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  const int enrouteSlot = sectionRow - 1;

  if (layout.enrouteCount > 0 && enrouteSlot < layout.enrouteCount) {

    return layout.enrouteFirst + enrouteSlot;

  }

  return legCount <= 0 ? 0 : 1;

}



inline int fplLegIndexForSectionRow(int sectionRow, int legCount,

                                    bool destinationFilled) {

  const int last = std::max(0, fplSectionSelectableCount(legCount) - 1);

  sectionRow = std::max(0, std::min(sectionRow, last));

  if (sectionRow == 0) return legCount > 0 ? 0 : -1;

  if (sectionRow == last) {

    return fplSectionLayout(legCount, destinationFilled).destLegIndex;

  }

  const FplSectionLayout layout = fplSectionLayout(legCount, destinationFilled);

  const int enrouteSlot = sectionRow - 1;

  if (layout.enrouteCount > 0 && enrouteSlot < layout.enrouteCount) {

    return layout.enrouteFirst + enrouteSlot;

  }

  return -1;

}



// PFD FPL body rows when a loaded approach follows the enroute template
// (trainer: Origin, dash, Enroute, dash, approach header, approach legs).
enum class FplDisplayRowKind {
  Origin,
  EnrouteLabel,
  EnrouteBlank,
  EnrouteLeg,
  SepDash,
  ApproachHeader,
  ApproachLeg,
};



struct FplDisplayRow {

  FplDisplayRowKind kind = FplDisplayRowKind::Origin;

  int legIndex = -1;

};



inline std::vector<FplDisplayRow> buildFplApproachDisplayRows(

    int approachStart, int approachCount, bool blankOriginSection) {

  std::vector<FplDisplayRow> rows;

  const int sectionLegCount = blankOriginSection ? 0 : approachStart;

  const std::vector<FplSectionRow> sectionRows =

      buildFplSectionRows(sectionLegCount, false);

  bool enrouteBlock = false;

  for (const FplSectionRow& sr : sectionRows) {

    if (sr.kind == FplSectionRow::Kind::Destination) continue;

    if (sr.kind == FplSectionRow::Kind::EnrouteBlank) continue;

    switch (sr.kind) {

      case FplSectionRow::Kind::Origin:

        rows.push_back({FplDisplayRowKind::Origin, sr.legIndex});

        rows.push_back({FplDisplayRowKind::SepDash, -1});

        break;

      case FplSectionRow::Kind::EnrouteLabel:

        rows.push_back({FplDisplayRowKind::EnrouteLabel, -1});

        enrouteBlock = true;

        break;

      case FplSectionRow::Kind::EnrouteLeg:

        rows.push_back({FplDisplayRowKind::EnrouteLeg, sr.legIndex});

        enrouteBlock = true;

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

    rows.push_back({FplDisplayRowKind::ApproachLeg, approachStart + i});

  }

  return rows;

}



}  // namespace avionics::pfd

