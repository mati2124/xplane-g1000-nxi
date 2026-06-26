#include "avionics/FlightPlanBridgeProtocol.h"

#include <gtest/gtest.h>

namespace avionics::fpbridge {
namespace {

MapLeg makeLeg(const char* id, double lat, double lon) {
  MapLeg leg;
  leg.id = id;
  leg.lat = lat;
  leg.lon = lon;
  return leg;
}

TEST(FlightPlanBridgeProtocolTest, SetDirectToRoundTripWithOrigin) {
  const MapLeg target = makeLeg("AZOMY", 26.522381, -82.132675);
  const std::vector<unsigned char> encoded =
      encodeSetDirectTo(true, target, true, 26.60, -82.20, true);

  bool active = false;
  MapLeg decoded;
  bool originValid = false;
  double originLat = 0.0;
  double originLon = 0.0;
  bool programFms = false;
  ASSERT_TRUE(decodeSetDirectTo(encoded.data(), encoded.size(), active, decoded,
                                originValid, originLat, originLon, programFms));
  EXPECT_TRUE(active);
  EXPECT_EQ(decoded.id, target.id);
  EXPECT_DOUBLE_EQ(decoded.lat, target.lat);
  EXPECT_DOUBLE_EQ(decoded.lon, target.lon);
  EXPECT_TRUE(originValid);
  EXPECT_DOUBLE_EQ(originLat, 26.60);
  EXPECT_DOUBLE_EQ(originLon, -82.20);
  EXPECT_TRUE(programFms);
}

TEST(FlightPlanBridgeProtocolTest, SetDirectToDisplayOnlyRoundTrip) {
  const MapLeg target = makeLeg("KFMY", 26.586, -81.863);
  const std::vector<unsigned char> encoded =
      encodeSetDirectTo(true, target, true, 26.50, -81.90, false);

  bool active = false;
  MapLeg decoded;
  bool originValid = false;
  double originLat = 0.0;
  double originLon = 0.0;
  bool programFms = true;
  ASSERT_TRUE(decodeSetDirectTo(encoded.data(), encoded.size(), active, decoded,
                                originValid, originLat, originLon, programFms));
  EXPECT_TRUE(active);
  EXPECT_FALSE(programFms);
}

TEST(FlightPlanBridgeProtocolTest, ClearDirectToRoundTrip) {
  const std::vector<unsigned char> encoded =
      encodeSetDirectTo(false, {}, false, 0.0, 0.0, true);

  bool active = true;
  MapLeg decoded;
  bool originValid = true;
  double originLat = 1.0;
  double originLon = 2.0;
  bool programFms = false;
  ASSERT_TRUE(decodeSetDirectTo(encoded.data(), encoded.size(), active, decoded,
                                originValid, originLat, originLon, programFms));
  EXPECT_FALSE(active);
  EXPECT_TRUE(programFms);
}

TEST(FlightPlanBridgeProtocolTest, ReplyRoundTripWithDirectToTrailer) {
  const std::vector<MapLeg> legs = {
      makeLeg("AZOMY", 26.522381, -82.132675),
      makeLeg("UZAWO", 26.436994, -82.046517),
  };
  DirectToState dto;
  dto.active = true;
  dto.target = legs.front();
  dto.originValid = true;
  dto.originLat = 26.61;
  dto.originLon = -82.19;

  const std::vector<unsigned char> encoded = encodeReply(legs, dto);

  std::vector<MapLeg> decodedLegs;
  DirectToState decodedDto;
  ASSERT_TRUE(decodeReply(encoded.data(), encoded.size(), decodedLegs,
                          &decodedDto));
  ASSERT_EQ(decodedLegs.size(), 2u);
  EXPECT_EQ(decodedLegs[0].id, "AZOMY");
  EXPECT_TRUE(decodedDto.active);
  EXPECT_EQ(decodedDto.target.id, "AZOMY");
  EXPECT_TRUE(decodedDto.originValid);
  EXPECT_DOUBLE_EQ(decodedDto.originLat, 26.61);
  EXPECT_DOUBLE_EQ(decodedDto.originLon, -82.19);
}

TEST(FlightPlanBridgeProtocolTest, ReplyWithoutDirectToTrailer) {
  const std::vector<MapLeg> legs = {makeLeg("ABC", 1.0, 2.0)};
  const std::vector<unsigned char> encoded = encodeReply(legs);

  std::vector<MapLeg> decodedLegs;
  DirectToState decodedDto;
  decodedDto.active = true;
  ASSERT_TRUE(decodeReply(encoded.data(), encoded.size(), decodedLegs,
                          &decodedDto));
  EXPECT_EQ(decodedLegs.size(), 1u);
  EXPECT_FALSE(decodedDto.active);
}

TEST(FlightPlanBridgeProtocolTest, ClearDirectToDisplayMessage) {
  const std::vector<unsigned char> encoded = encodeClearDirectToDisplay();
  EXPECT_TRUE(isClearDirectToDisplay(encoded.data(), encoded.size()));
}

}  // namespace
}  // namespace avionics::fpbridge
