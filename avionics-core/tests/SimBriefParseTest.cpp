#include "avionics/SimBriefOfpSupport.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avionics/MapData.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics {
namespace {

MapLeg makeLeg(const char* id) {
  MapLeg leg;
  leg.id = id;
  return leg;
}

TEST(SimBriefParseTest, InferDepartureBlockFromViaAirway) {
  // KFMY origin, CSHEL on CSHEL8, enroute fixes, KJAX destination.
  const std::vector<std::string> via = {
      "", "CSHEL8", "DIRECT", "DIRECT", "DIRECT", ""};
  const SimBriefProcedureBlocks blocks =
      inferSimBriefProcedureBlocks(static_cast<int>(via.size()), via, "CSHEL8",
                                   "");
  EXPECT_EQ(blocks.departureLegStart, 1);
  EXPECT_EQ(blocks.departureLegCount, 1);
  EXPECT_EQ(blocks.arrivalLegStart, -1);
  EXPECT_EQ(blocks.arrivalLegCount, 0);
}

TEST(SimBriefParseTest, InferArrivalBlockFromViaAirway) {
  const std::vector<std::string> via = {"", "DIRECT", "STAR1", "STAR1", ""};
  const SimBriefProcedureBlocks blocks =
      inferSimBriefProcedureBlocks(static_cast<int>(via.size()), via, "",
                                   "STAR1");
  EXPECT_EQ(blocks.departureLegStart, -1);
  EXPECT_EQ(blocks.departureLegCount, 0);
  EXPECT_EQ(blocks.arrivalLegStart, 2);
  EXPECT_EQ(blocks.arrivalLegCount, 2);
}

TEST(SimBriefParseTest, FormatTerminalProcedureHeaderLabel) {
  EXPECT_EQ(formatTerminalProcedureFplHeaderLabel("05", "CSHEL8", ""),
            "RW05.CSHEL8");
  EXPECT_EQ(formatTerminalProcedureFplHeaderLabel("05", "CSHEL6", "LAL"),
            "RW05.CSHEL6.LAL");
  EXPECT_EQ(formatTerminalProcedureFplHeaderLabel("RW05", "CSHEL6", "LAL"),
            "RW05.CSHEL6.LAL");
}

TEST(SimBriefParseTest, BuildProcedureDisplayRowsDepartureBeforeLeg) {
  std::vector<MapLeg> legs = {makeLeg("KFMY"), makeLeg("CSHEL"),
                              makeLeg("SEALZ"), makeLeg("KJAX")};
  const std::string depHeader = "RW05.CSHEL8";
  const auto rows = pfd::buildFplProcedureDisplayRows(
      legs, 1, 1, depHeader, -1, 0, std::string(), -1, 0, true, true);
  ASSERT_GE(rows.size(), 2u);
  EXPECT_EQ(rows[0].kind, pfd::FplDisplayRowKind::DepartureHeader);
  EXPECT_EQ(rows[1].kind, pfd::FplDisplayRowKind::DepartureLeg);
  EXPECT_EQ(rows[1].legIndex, 1);
}

TEST(SimBriefParseTest, PersistedPlanFromSimBriefImportCarriesDepartureMeta) {
  SimBriefOfpImport imp;
  imp.legs = {makeLeg("KFMY"), makeLeg("CSHEL"), makeLeg("KJAX")};
  imp.originIcao = "KFMY";
  imp.destinationIcao = "KJAX";
  imp.sidIdent = "CSHEL8";
  imp.originRunway = "05";
  imp.departureLegStart = 1;
  imp.departureLegCount = 1;

  const PersistedFlightPlan plan = persistedFlightPlanFromSimBriefImport(imp);
  EXPECT_TRUE(plan.departureMeta.active);
  EXPECT_EQ(plan.departureMeta.type, ProcedureType::Departure);
  EXPECT_EQ(plan.departureMeta.airportIcao, "KFMY");
  EXPECT_EQ(plan.departureMeta.name, "CSHEL8");
  EXPECT_EQ(plan.departureMeta.runway, "05");
  EXPECT_EQ(plan.departureLegStart, 1);
  EXPECT_EQ(plan.departureLegCount, 1);
}

}  // namespace
}  // namespace avionics
