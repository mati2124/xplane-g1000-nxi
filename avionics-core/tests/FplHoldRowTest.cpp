#include <gtest/gtest.h>

#include "avionics/FlightData.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/FlightPlanPersistence.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics::pfd {
namespace {

TEST(FplHoldRowTest, ApproachDisplayRowsInsertHoldAfterFixWithPublishedHold) {
  MapLeg cmi;
  cmi.id = "CMI";
  cmi.lat = 40.0;
  cmi.lon = -88.27;
  MapLeg bostn;
  bostn.id = "BOSTN";
  bostn.lat = 40.10;
  bostn.lon = -88.20;
  bostn.procedureRole = "iaf";
  bostn.hold.active = true;
  bostn.hold.inboundCourseDeg = 44.0f;
  bostn.hold.legLengthNm = 4.0f;
  MapLeg faf;
  faf.id = "AFTOR";
  faf.lat = 40.05;
  faf.lon = -88.15;
  faf.procedureRole = "faf";
  const std::vector<MapLeg> legs = {cmi, bostn, faf};

  const auto rows = buildFplApproachDisplayRows(legs, 1, 2, false, true);
  bool sawBostn = false;
  bool sawHoldAfterBostn = false;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const FplDisplayRow& dr = rows[i];
    if (dr.kind == FplDisplayRowKind::ApproachLeg && dr.legIndex == 1) {
      sawBostn = true;
      if (i + 1 < rows.size() &&
          rows[i + 1].kind == FplDisplayRowKind::Hold &&
          rows[i + 1].legIndex == 1) {
        sawHoldAfterBostn = true;
      }
    }
  }
  EXPECT_TRUE(sawBostn);
  EXPECT_TRUE(sawHoldAfterBostn);
  EXPECT_TRUE(fplApproachDisplayRowSelectable(FplDisplayRowKind::Hold));
}

TEST(FplHoldRowTest, ApproachDisplayKeepsDestinationAirportOutOfEnroute) {
  // KFMY -> ... -> KJAX, then RNAV 08 (WADOR iaf / GRRDN faf). With the
  // destination section filled, the airport (KJAX) belongs in the approach
  // header, never as an Enroute leg.
  const auto leg = [](const char* id) {
    MapLeg l;
    l.id = id;
    return l;
  };
  std::vector<MapLeg> legs = {
      leg("KFMY"), leg("LAL"), leg("JINOS"), leg("TEBOW"), leg("KJAX"),
      leg("WADOR"), leg("AMXUQ"),
  };
  legs.back().procedureRole = "faf";
  // approachStart at WADOR (index 5), two approach legs.
  const auto rows = buildFplApproachDisplayRows(legs, 5, 2,
                                                /*blankOriginSection=*/false,
                                                /*destinationFilled=*/true);
  for (const FplDisplayRow& dr : rows) {
    if (dr.kind == FplDisplayRowKind::EnrouteLeg && dr.legIndex >= 0) {
      EXPECT_FALSE(isAirportIdent(legs[static_cast<std::size_t>(dr.legIndex)].id))
          << "airport leg leaked into Enroute: "
          << legs[static_cast<std::size_t>(dr.legIndex)].id;
    }
  }
  // KJAX (index 4) must not appear as an Enroute leg at all.
  bool kjaxEnroute = false;
  for (const FplDisplayRow& dr : rows) {
    if (dr.kind == FplDisplayRowKind::EnrouteLeg && dr.legIndex == 4) {
      kjaxEnroute = true;
    }
  }
  EXPECT_FALSE(kjaxEnroute);
}

TEST(FplHoldRowTest, ApproachDisplayRowsOmitHoldWhenHoldInactive) {
  MapLeg bostn;
  bostn.id = "BOSTN";
  bostn.lat = 40.10;
  bostn.lon = -88.20;
  bostn.procedureRole = "iaf";
  MapLeg faf;
  faf.id = "AFTOR";
  faf.lat = 40.05;
  faf.lon = -88.15;
  faf.procedureRole = "faf";
  const std::vector<MapLeg> legs = {bostn, faf};

  const auto rows = buildFplApproachDisplayRows(legs, 0, 2, true, false);
  for (const FplDisplayRow& dr : rows) {
    EXPECT_NE(dr.kind, FplDisplayRowKind::Hold);
  }
}

TEST(FplHoldRowTest, CursorOnHoldRowDetectsHoldDisplayRow) {
  MapLeg bostn;
  bostn.id = "BOSTN";
  bostn.lat = 40.10;
  bostn.lon = -88.20;
  bostn.procedureRole = "iaf";
  bostn.hold.active = true;
  bostn.hold.inboundCourseDeg = 44.0f;
  bostn.hold.legLengthNm = 4.0f;
  MapLeg faf;
  faf.id = "AFTOR";
  faf.lat = 40.05;
  faf.lon = -88.15;
  faf.procedureRole = "faf";
  const std::vector<MapLeg> legs = {bostn, faf};

  bool destinationFilled = true;
  int approachStart = 0;
  int approachCount = 2;
  const bool blankOrigin =
      ::avionics::fplApproachBlankOriginSection(legs, approachStart, "KCMI");
  int holdCursor = -1;
  int selectable = 0;
  for (const pfd::FplDisplayRow& dr :
       pfd::fplApproachDisplayRowList(legs, approachStart, approachCount,
                                      blankOrigin, destinationFilled)) {
    if (!pfd::fplApproachDisplayRowSelectable(dr.kind)) continue;
    if (dr.kind == pfd::FplDisplayRowKind::Hold) holdCursor = selectable;
    ++selectable;
  }
  ASSERT_GE(holdCursor, 0);

  int cursorRow = holdCursor;
  FplRouteEdit edit{const_cast<std::vector<MapLeg>&>(legs), destinationFilled,
                    approachStart, approachCount, cursorRow, nullptr, nullptr};
  EXPECT_TRUE(::avionics::fplCursorOnHoldRow(
      edit, "KCMI", ::avionics::FplCursorLayout::SectionRows));
  cursorRow = 0;
  EXPECT_FALSE(::avionics::fplCursorOnHoldRow(
      edit, "KCMI", ::avionics::FplCursorLayout::SectionRows));
}

TEST(FplHoldRowTest, OriginLegIsSelectableWithSingleEnrouteSectionLeg) {
  // Origin airport followed straight by a loaded approach (no enroute legs and
  // no separate destination airport leg): KATL -> [R20L approach into KBNA].
  // The single enroute-section leg (KATL) is the Origin, not a destination-only
  // plan, so its display row must carry leg index 0 — otherwise the cursor sits
  // on a row that maps to no leg and CLR cannot delete it.
  const auto leg = [](const char* id, const char* role = "") {
    MapLeg l;
    l.id = id;
    l.procedureRole = role;
    return l;
  };
  const std::vector<MapLeg> legs = {
      leg("KATL"), leg("WAYLN", "iaf"), leg("CRAMR"),
      leg("JUUDD", "faf"), leg("XIYRI"), leg("RW20L", "mapt"),
  };
  const int approachStart = 1;
  const int approachCount = 5;

  const auto rows = buildFplApproachDisplayRows(legs, approachStart,
                                                approachCount,
                                                /*blankOriginSection=*/false,
                                                /*destinationFilled=*/true);
  bool sawOriginKatl = false;
  for (const FplDisplayRow& dr : rows) {
    if (dr.kind == FplDisplayRowKind::Origin) {
      EXPECT_EQ(dr.legIndex, 0) << "Origin row must reference KATL (leg 0)";
      sawOriginKatl = true;
    }
  }
  EXPECT_TRUE(sawOriginKatl);

  // The first selectable cursor row (the Origin) resolves to leg 0, so CLR on
  // KATL removes it.
  EXPECT_EQ(fplApproachLegIndexForSelectable(0, legs, approachStart,
                                             approachCount,
                                             /*blankOriginSection=*/false,
                                             /*destinationFilled=*/true),
            0);

  std::vector<MapLeg> mutableLegs = legs;
  bool destinationFilled = true;
  int editApproachStart = approachStart;
  int editApproachCount = approachCount;
  int cursorRow = 0;
  FplRouteEdit edit{mutableLegs,     destinationFilled, editApproachStart,
                    editApproachCount, cursorRow,       nullptr,
                    nullptr};
  EXPECT_EQ(::avionics::fplCursorLegIndex(
                edit, "KBNA", ::avionics::FplCursorLayout::SectionRows),
            0);
}

TEST(FplHoldRowTest, HoldNavActiveUsesHoldSelectableRow) {
  MapLeg bostn;
  bostn.id = "BOSTN";
  bostn.lat = 40.10;
  bostn.lon = -88.20;
  bostn.procedureRole = "iaf";
  bostn.hold.active = true;
  MapLeg faf;
  faf.id = "AFTOR";
  faf.lat = 40.05;
  faf.lon = -88.15;
  faf.procedureRole = "faf";
  const std::vector<MapLeg> legs = {bostn, faf};

  FlightData d;
  d.fmaLegIsHold = true;
  d.fmaActiveLegIndex = 0;
  EXPECT_TRUE(::avionics::fplHoldNavActiveOnLeg(d, false, false, false, 0, 0,
                                                bostn));

  const int fixRow = fplApproachSelectableRowForLegIndex(
      0, legs, 0, 2, true, false);
  const int holdRow = fplApproachSelectableRowForHoldLegIndex(
      0, legs, 0, 2, true, false);
  EXPECT_GE(fixRow, 0);
  EXPECT_GE(holdRow, 0);
  EXPECT_NE(fixRow, holdRow);
}

}  // namespace
}  // namespace avionics::pfd
