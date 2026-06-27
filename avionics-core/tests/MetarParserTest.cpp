#include <gtest/gtest.h>

#include "avionics/MetarParser.h"

// METAR decoder for the WPT - Weather Information page. The page decodes the
// full field list (time, wind, visibility, clouds, temperature, dew point,
// altimeter) and then shows the verbatim report (Garmin NXi trainer apt_059,
// KFMY).
namespace avionics {
namespace {

// The exact report shown on the trainer's WPT - Weather page for KFMY.
constexpr const char* kTrainerKfmyMetar =
    "SA KFMY 161453Z 08005KT 10SM FEW014 24/21 A3009 RMK AO2 SLP189 "
    "T02440206 51015";

TEST(MetarParserTest, DecodesTrainerKfmyReport) {
  const MetarDecoded d = parseMetar(kTrainerKfmyMetar);

  ASSERT_TRUE(d.observationTime.has_value());
  EXPECT_EQ(*d.observationTime, "161453Z");

  EXPECT_FALSE(d.windCalm);
  EXPECT_FALSE(d.windVariable);
  ASSERT_TRUE(d.windDirectionDeg.has_value());
  EXPECT_EQ(*d.windDirectionDeg, 80);
  ASSERT_TRUE(d.windSpeedKt.has_value());
  EXPECT_EQ(*d.windSpeedKt, 5);
  EXPECT_FALSE(d.windGustKt.has_value());

  ASSERT_TRUE(d.visibility.has_value());
  EXPECT_EQ(*d.visibility, "10SM");

  ASSERT_TRUE(d.clouds.has_value());
  EXPECT_EQ(*d.clouds, "FEW014");

  ASSERT_TRUE(d.temperatureC.has_value());
  EXPECT_FLOAT_EQ(*d.temperatureC, 24.0f);
  ASSERT_TRUE(d.dewPointC.has_value());
  EXPECT_FLOAT_EQ(*d.dewPointC, 21.0f);

  ASSERT_TRUE(d.altimeterInHg.has_value());
  EXPECT_FLOAT_EQ(*d.altimeterInHg, 30.09f);
}

TEST(MetarParserTest, IgnoresLeadingDesignatorAndRemarks) {
  // Without the "SA " designator the body still parses, and the high-precision
  // T-group inside RMK must not override the body's "24/21" dew point.
  const MetarDecoded d =
      parseMetar("KFMY 161453Z 08005KT 10SM FEW014 24/21 A3009 RMK T02440206");
  ASSERT_TRUE(d.temperatureC.has_value());
  EXPECT_FLOAT_EQ(*d.temperatureC, 24.0f);
  ASSERT_TRUE(d.dewPointC.has_value());
  EXPECT_FLOAT_EQ(*d.dewPointC, 21.0f);
  ASSERT_TRUE(d.altimeterInHg.has_value());
  EXPECT_FLOAT_EQ(*d.altimeterInHg, 30.09f);
}

TEST(MetarParserTest, DecodesGustsAndMultipleCloudLayers) {
  const MetarDecoded d = parseMetar(
      "KORD 121651Z 34015G25KT 10SM FEW014 SCT025 BKN040 M03/M08 A2998");
  ASSERT_TRUE(d.windDirectionDeg.has_value());
  EXPECT_EQ(*d.windDirectionDeg, 340);
  ASSERT_TRUE(d.windSpeedKt.has_value());
  EXPECT_EQ(*d.windSpeedKt, 15);
  ASSERT_TRUE(d.windGustKt.has_value());
  EXPECT_EQ(*d.windGustKt, 25);
  ASSERT_TRUE(d.clouds.has_value());
  EXPECT_EQ(*d.clouds, "FEW014 SCT025 BKN040");
  ASSERT_TRUE(d.temperatureC.has_value());
  EXPECT_FLOAT_EQ(*d.temperatureC, -3.0f);
  ASSERT_TRUE(d.dewPointC.has_value());
  EXPECT_FLOAT_EQ(*d.dewPointC, -8.0f);
  ASSERT_TRUE(d.altimeterInHg.has_value());
  EXPECT_FLOAT_EQ(*d.altimeterInHg, 29.98f);
}

TEST(MetarParserTest, DecodesVariableAndCalmWinds) {
  const MetarDecoded vrb = parseMetar("KAPA 121653Z VRB03KT 10SM CLR 20/M01 A3001");
  EXPECT_TRUE(vrb.windVariable);
  EXPECT_FALSE(vrb.windDirectionDeg.has_value());
  ASSERT_TRUE(vrb.windSpeedKt.has_value());
  EXPECT_EQ(*vrb.windSpeedKt, 3);
  ASSERT_TRUE(vrb.clouds.has_value());
  EXPECT_EQ(*vrb.clouds, "CLR");

  const MetarDecoded calm = parseMetar("KAPA 121653Z 00000KT 10SM CLR 20/M01 A3001");
  EXPECT_TRUE(calm.windCalm);
  EXPECT_FALSE(calm.windDirectionDeg.has_value());
  EXPECT_FALSE(calm.windSpeedKt.has_value());
}

TEST(MetarParserTest, DoesNotMisreadVisibilityOrRvrAsTempDewPoint) {
  // "1/2SM" (fractional visibility) and "R06/2000" (RVR) both contain a slash
  // but are not temperature/dew-point groups, so temp/dew stay unknown.
  const MetarDecoded d =
      parseMetar("KSEA 121653Z 18004KT 1/2SM R06/2000 FG VV002 Q1015");
  EXPECT_FALSE(d.temperatureC.has_value());
  EXPECT_FALSE(d.dewPointC.has_value());
  EXPECT_FALSE(d.altimeterInHg.has_value());  // Q-group is hPa, not decoded
  ASSERT_TRUE(d.visibility.has_value());
  EXPECT_EQ(*d.visibility, "1/2SM");
  ASSERT_TRUE(d.clouds.has_value());
  EXPECT_EQ(*d.clouds, "VV002");
}

TEST(MetarParserTest, EmptyReportYieldsNothing) {
  const MetarDecoded d = parseMetar("");
  EXPECT_FALSE(d.observationTime.has_value());
  EXPECT_FALSE(d.windSpeedKt.has_value());
  EXPECT_FALSE(d.temperatureC.has_value());
  EXPECT_FALSE(d.dewPointC.has_value());
  EXPECT_FALSE(d.altimeterInHg.has_value());
}

}  // namespace
}  // namespace avionics
