#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/MapData.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics::test {
namespace {

MapLeg Leg(const std::string& id, const std::string& role = std::string()) {
  MapLeg leg;
  leg.id = id;
  leg.procedureRole = role;
  return leg;
}

// Realistic SimBrief-style plan: KFMY origin, one enroute fix, a STAR into KJAX
// (INPIN/DEEDS/SHFTY, role-tagged), the destination airport KJAX, then an RNAV
// approach into KJAX (HOMER iaf / GIPNO / RW08 mapt / MISSD mahp).
std::vector<MapLeg> MakeStarPlusApproachPlan() {
  return {
      Leg("KFMY"),            // 0 origin
      Leg("DURTE"),           // 1 enroute
      Leg("INPIN", "trans"),  // 2 STAR
      Leg("DEEDS", "trans"),  // 3 STAR
      Leg("SHFTY", "trans"),  // 4 STAR
      Leg("KJAX"),            // 5 destination airport
      Leg("HOMER", "iaf"),    // 6 approach
      Leg("GIPNO", ""),       // 7 approach (untagged intermediate)
      Leg("RW08", "mapt"),    // 8 approach
      Leg("MISSD", "mahp"),   // 9 missed
  };
}

// Mirror of the MFD/PFD page approach-block resolution (MfdFlightPlanPage.cpp
// ~2218-2234, ChromeFlightPlanWindow.cpp ~592-599).
struct ResolvedBlocks {
  int approachStart = 0;
  int approachCount = 0;
};
ResolvedBlocks ResolveLikePage(const std::vector<MapLeg>& plan, int arrStart,
                               int arrCount, int storedApproachStart,
                               int storedApproachCount,
                               const std::string& transition) {
  const int arrivalEnd = arrCount > 0 ? arrStart + arrCount : 0;
  InferredProcedureBlock block = resolveApproachBlockInPlan(
      plan, storedApproachStart, storedApproachCount, transition, arrivalEnd);
  int start = block.start;
  int count = block.count;
  if (count > 0 && start >= 0 &&
      start + count < static_cast<int>(plan.size())) {
    count = fplNormalizedApproachCount(start, count, static_cast<int>(plan.size()));
  }
  return {start, count};
}

bool HasArrivalHeader(const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows)
    if (dr.kind == pfd::FplDisplayRowKind::ArrivalHeader) return true;
  return false;
}
bool HasApproachHeader(const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows)
    if (dr.kind == pfd::FplDisplayRowKind::ApproachHeader) return true;
  return false;
}
// True when a STAR fix renders under the Approach header (wrong grouping).
bool StarLegUnderApproach(const std::vector<MapLeg>& plan,
                          const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::ApproachLeg && dr.legIndex >= 0) {
      const std::string& id = plan[static_cast<std::size_t>(dr.legIndex)].id;
      if (id == "INPIN" || id == "DEEDS" || id == "SHFTY") return true;
    }
  }
  return false;
}

// Baseline: when both the arrival and approach blocks are correctly tracked the
// page shows both an Arrival header and an Approach header.
TEST(StarApproachParentTest, BothHeadersWhenArrivalAndApproachTracked) {
  const auto plan = MakeStarPlusApproachPlan();
  const ResolvedBlocks rb = ResolveLikePage(plan, /*arrStart=*/2, /*arrCount=*/3,
                                            /*storedApproachStart=*/6,
                                            /*storedApproachCount=*/4, "HOMER");
  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, /*depStart=*/0, /*depCount=*/0, /*departureHeader=*/std::string(),
      /*arrStart=*/2, /*arrCount=*/3, /*arrivalHeader=*/"SHFTY6.INPIN",
      rb.approachStart, rb.approachCount, /*blankOriginSection=*/false,
      /*destinationFilled=*/true);
  EXPECT_TRUE(HasArrivalHeader(rows));
  EXPECT_TRUE(HasApproachHeader(rows));
  EXPECT_FALSE(StarLegUnderApproach(plan, rows));
}

// The reported bug: a STAR is loaded but its arrival block is NOT tracked (e.g.
// the plan arrived from the sim / external FMS where only the approach is
// re-inferred). The approach must still render as its own parent and the STAR
// fixes must not be swallowed into the approach block.
TEST(StarApproachParentTest, ApproachIsParentEvenWhenArrivalUntracked) {
  const auto plan = MakeStarPlusApproachPlan();
  const ResolvedBlocks rb = ResolveLikePage(plan, /*arrStart=*/0, /*arrCount=*/0,
                                            /*storedApproachStart=*/0,
                                            /*storedApproachCount=*/0,
                                            std::string());
  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, /*depStart=*/0, /*depCount=*/0, /*departureHeader=*/std::string(),
      /*arrStart=*/0, /*arrCount=*/0, /*arrivalHeader=*/std::string(),
      rb.approachStart, rb.approachCount, /*blankOriginSection=*/false,
      /*destinationFilled=*/true);
  EXPECT_TRUE(HasApproachHeader(rows));
  EXPECT_FALSE(StarLegUnderApproach(plan, rows))
      << "approachStart=" << rb.approachStart
      << " approachCount=" << rb.approachCount;
}

bool HasDepartureHeader(const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows)
    if (dr.kind == pfd::FplDisplayRowKind::DepartureHeader) return true;
  return false;
}
// Any leg row that points past the end of the leg list (the out-of-range read
// that produced "blank lines" on screen).
bool HasOutOfRangeLegRow(const std::vector<MapLeg>& plan,
                         const std::vector<pfd::FplDisplayRow>& rows) {
  const int n = static_cast<int>(plan.size());
  for (const auto& dr : rows) {
    const bool isLeg = dr.kind == pfd::FplDisplayRowKind::DepartureLeg ||
                       dr.kind == pfd::FplDisplayRowKind::ArrivalLeg ||
                       dr.kind == pfd::FplDisplayRowKind::ApproachLeg ||
                       dr.kind == pfd::FplDisplayRowKind::EnrouteLeg;
    if (isLeg && (dr.legIndex < 0 || dr.legIndex >= n)) return true;
  }
  return false;
}

// Regression for the "bunch of blank lines" report: loading an approach can
// replace a longer route with a short KJAX + approach plan, but the previous
// departure/arrival block indices (here legs 1-7 and 8-18) are left behind and
// no longer fit the 6-leg list. Those stale blocks must not emit parent headers
// or rows for out-of-range legs.
TEST(StarApproachParentTest, StaleTerminalBlocksDoNotEmitBlankRows) {
  const std::vector<MapLeg> plan = {
      Leg("KJAX"),           // 0 origin
      Leg("APASY", "iaf"),   // 1 approach
      Leg("GLUMM", "faf"),   // 2 approach
      Leg("EBAXE", ""),      // 3 approach
      Leg("RW23", "mapt"),   // 4 approach
      Leg("CALOO", "mahp"),  // 5 missed
  };
  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, /*depStart=*/1, /*depCount=*/7, /*departureHeader=*/"JETIN2.JAYJA",
      /*arrStart=*/8, /*arrCount=*/11, /*arrivalHeader=*/"SHFTY6.INPIN",
      /*approachStart=*/1, /*approachCount=*/5, /*blankOriginSection=*/false,
      /*destinationFilled=*/true);
  EXPECT_FALSE(HasDepartureHeader(rows));
  EXPECT_FALSE(HasArrivalHeader(rows));
  EXPECT_TRUE(HasApproachHeader(rows));
  EXPECT_FALSE(HasOutOfRangeLegRow(plan, rows));
}

bool HasAnyLegRow(const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows) {
    const bool isLeg = dr.kind == pfd::FplDisplayRowKind::DepartureLeg ||
                       dr.kind == pfd::FplDisplayRowKind::ArrivalLeg ||
                       dr.kind == pfd::FplDisplayRowKind::ApproachLeg ||
                       dr.kind == pfd::FplDisplayRowKind::EnrouteLeg;
    if (isLeg && dr.legIndex >= 0) return true;
  }
  return false;
}

// Regression for "NO ACTIVE FLIGHT PLAN": the page chooses the procedure display
// path from stale departure/arrival headers, but the blocks no longer fit the
// (shorter) plan. After sanitizing the stale blocks the page must still surface
// the real legs (here a role-tagged approach inferred from the leg roles), not
// an empty row list. Mirrors the MFD/PFD block-fit guard.
TEST(StarApproachParentTest, StaleBlocksStillSurfaceRealLegs) {
  const std::vector<MapLeg> plan = {
      Leg("KJAX"),           // 0 origin
      Leg("APASY", "iaf"),   // 1 approach
      Leg("GLUMM", "faf"),   // 2 approach
      Leg("EBAXE", ""),      // 3 approach
      Leg("RW23", "mapt"),   // 4 approach
      Leg("CALOO", "mahp"),  // 5 missed
  };
  const int planSize = static_cast<int>(plan.size());

  // Stale departure/arrival blocks left over from a longer route.
  int depStart = 1, depCount = 7;
  int arrStart = 8, arrCount = 11;
  std::string depHeader = "JETIN2.JAYJA";
  std::string arrHeader = "SHFTY6.INPIN";
  if (!pfd::fplBlockFitsPlan(depStart, depCount, planSize)) {
    depStart = 0;
    depCount = 0;
    depHeader.clear();
  }
  if (!pfd::fplBlockFitsPlan(arrStart, arrCount, planSize)) {
    arrStart = 0;
    arrCount = 0;
    arrHeader.clear();
  }
  const int arrivalEnd = arrCount > 0 ? arrStart + arrCount : 0;
  // No stored approach (approachActive was cleared); infer from leg roles.
  InferredProcedureBlock block =
      resolveApproachBlockInPlan(plan, 0, 0, std::string(), arrivalEnd);
  int approachStart = block.start;
  int approachCount = block.count;
  if (approachCount > 0 && approachStart >= 0 &&
      approachStart + approachCount < planSize) {
    approachCount =
        fplNormalizedApproachCount(approachStart, approachCount, planSize);
  }

  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, depStart, depCount, depHeader, arrStart, arrCount, arrHeader,
      approachStart, approachCount, /*blankOriginSection=*/false,
      /*destinationFilled=*/true);
  EXPECT_FALSE(HasDepartureHeader(rows));
  EXPECT_FALSE(HasArrivalHeader(rows));
  EXPECT_TRUE(HasAnyLegRow(rows));
  EXPECT_FALSE(HasOutOfRangeLegRow(plan, rows));
}

// A KJAX -> RNAV-into-KFMY plan with NO destination airport waypoint before the
// approach (the only airport ahead of the approach is the origin KJAX).
std::vector<MapLeg> MakeOriginPlusApproachPlan() {
  return {
      Leg("KJAX"),           // 0 origin
      Leg("APASY", "iaf"),   // 1 approach
      Leg("GLUMM", "faf"),   // 2 approach
      Leg("RW23", "mapt"),   // 3 approach
      Leg("CALOO", "mahp"),  // 4 missed
  };
}

// Fix B: an explicitly known approach airport must win over the "airport before
// the approach" heuristic, which otherwise returns the origin (KJAX) when no
// destination waypoint precedes the approach.
TEST(StarApproachParentTest, LoadedApproachAirportWinsOverAirportBeforeApproach) {
  const auto plan = MakeOriginPlusApproachPlan();
  EXPECT_EQ(fplApproachAirportIcao(plan, /*approachStart=*/1, /*map=*/nullptr,
                                   /*loadedApproachAirportIcao=*/"KFMY"),
            "KFMY");
  // With no known approach airport the origin is not the destination when the
  // plan continues past the IAF; fall back to the only airport in the plan
  // (same-field IFR into the departure airport).
  EXPECT_EQ(fplApproachAirportIcao(plan, /*approachStart=*/1, /*map=*/nullptr,
                                   /*loadedApproachAirportIcao=*/std::string()),
            "KJAX");
}

// The FPL header destination must use the approach airport when it is known,
// rather than the pre-approach origin.
TEST(StarApproachParentTest, DestinationUsesApproachAirportWhenKnown) {
  const auto plan = MakeOriginPlusApproachPlan();
  EXPECT_EQ(pfd::fplHeaderDestinationIdent(plan, /*destinationFilled=*/true,
                                           /*approachLegStart=*/1,
                                           /*approachLoaded=*/true,
                                           /*approachAirport=*/"KFMY"),
            "KFMY");
  EXPECT_EQ(pfd::fplHeaderDestinationIdent(plan, /*destinationFilled=*/true,
                                           /*approachLegStart=*/1,
                                           /*approachLoaded=*/true,
                                           /*approachAirport=*/std::string()),
            "KJAX");
}

}  // namespace
}  // namespace avionics::test
