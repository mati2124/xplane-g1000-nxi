#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
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

// KFMY origin, a SID (RW05/420FT/MANSEQ/CSHEL/JUNLO) loaded via PROC, an enroute
// fix (LAL), then an RNAV approach appended at the tail. The destination airport
// waypoint (KJAX) is NOT in the leg list - X-Plane's FMS drops the separate
// airport waypoint once an approach is loaded, so the plan that echoes back has
// the approach fixes directly after the last enroute fix.
std::vector<MapLeg> MakeSidPlusApproachNoDestLeg() {
  std::vector<MapLeg> plan = {
      Leg("KFMY"),          // 0 origin
      Leg("RW05"),          // 1 SID
      Leg("420FT"),         // 2 SID (climb)
      Leg("MANSEQ"),        // 3 SID (vector)
      Leg("CSHEL"),         // 4 SID
      Leg("JUNLO"),         // 5 SID
      Leg("LAL"),           // 6 enroute
      Leg("HOMER", "iaf"),  // 7 approach
      Leg("GIPNO", ""),     // 8 approach
      Leg("RW08", "mapt"),  // 9 approach
      Leg("MISSD", "mahp"), // 10 missed
  };
  plan[2].pathTerminator = "CA";
  plan[3].pathTerminator = "VM";
  return plan;
}

bool SidLegUnderApproach(const std::vector<MapLeg>& plan,
                         const std::vector<pfd::FplDisplayRow>& rows) {
  for (const auto& dr : rows) {
    if (dr.kind == pfd::FplDisplayRowKind::ApproachLeg && dr.legIndex >= 0) {
      const std::string& id = plan[static_cast<std::size_t>(dr.legIndex)].id;
      if (id == "RW05" || id == "CSHEL" || id == "JUNLO") return true;
    }
  }
  return false;
}

// The approach inference must not walk back over the SID/enroute fixes when no
// destination-airport waypoint separates them from the approach.
TEST(SidApproachInferenceTest, ApproachDoesNotSwallowSid) {
  const auto plan = MakeSidPlusApproachNoDestLeg();
  // No stored approach block (sim resync cleared it); a SID departure block is
  // tracked (loaded via PROC): RW05..JUNLO = legs 1-5.
  const int depStart = 1;
  const int depCount = 5;
  const int depEnd = depStart + depCount;

  const InferredProcedureBlock block =
      resolveApproachBlockInPlan(plan, /*storedStart=*/0, /*storedCount=*/0,
                                 /*transition=*/"HOMER", /*arrivalEnd=*/0,
                                 /*departureEnd=*/depEnd);
  ASSERT_TRUE(block.valid());
  // The approach must start at the IAF (HOMER, index 7), not back at the SID.
  EXPECT_GE(block.start, 6) << "approach swallowed the SID/enroute fixes";

  const auto rows = pfd::buildFplProcedureDisplayRows(
      plan, depStart, depCount, /*departureHeader=*/"RW05.CSHEL6.LAL",
      /*arrStart=*/0, /*arrCount=*/0, /*arrivalHeader=*/std::string(),
      block.start, block.count, /*blankOriginSection=*/false,
      /*destinationFilled=*/true);
  EXPECT_FALSE(SidLegUnderApproach(plan, rows))
      << "SID rendered under the Approach (destination) header; start="
      << block.start << " count=" << block.count;
}

}  // namespace
}  // namespace avionics::test
