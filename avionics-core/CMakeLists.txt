# Platform-agnostic avionics engine: all logic + drawing, zero platform deps.
# Builds cleanly on its own so the shared code can be developed and tested
# without the X-Plane SDK or a windowing/GPU stack present.
add_library(avionics-core STATIC
  src/AssetPaths.cpp
  src/UpdateChecker.cpp
  src/AvionicsEngine.cpp
  src/ChecklistParser.cpp
  src/EisParser.cpp
  src/EisLegacy.cpp
  src/EisStore.cpp
  src/MockDataSource.cpp
  src/PersistentState.cpp
  src/Radio.cpp
  src/NavDatabase.cpp
  src/SoftkeyController.cpp
  src/render/BezelKeys.cpp
  src/render/BezelStyle.cpp
  src/render/BootScreen.cpp
  src/render/SoftkeyBezel.cpp
  src/render/PrimaryFlightDisplay.cpp
  src/render/mfd/EisStrip.cpp
  src/render/mfd/MfdController.cpp
  src/render/mfd/MfdPageSupport.cpp
  src/render/mfd/MfdMapPage.cpp
  src/render/mfd/MfdMapSettingsPage.cpp
  src/render/mfd/MfdWeatherRadarPage.cpp
  src/render/mfd/MfdWaypointPages.cpp
  src/render/mfd/MfdAuxPages.cpp
  src/render/mfd/MfdNearestPages.cpp
  src/render/mfd/MfdFlightPlanPage.cpp
  src/render/mfd/MfdChecklistPage.cpp
  src/render/mfd/MultiFunctionDisplay.cpp
  src/nav/AptDatParser.cpp
  src/nav/AptDatGeometryCache.cpp
  src/nav/CifpParser.cpp
  src/nav/OpenAirParser.cpp
  src/nav/Terrain.cpp
  src/nav/WeatherRadar.cpp
  src/render/map/MapProjection.cpp
  src/render/map/MapSymbols.cpp
  src/render/map/MapView.cpp
  src/render/map/TerrainRaster.cpp
  src/render/map/WeatherRaster.cpp
  src/render/pfd/InsetMap.cpp
  src/render/pfd/PfdShared.cpp
  src/render/pfd/AttitudeIndicator.cpp
  src/render/pfd/AirspeedTape.cpp
  src/render/pfd/Altimeter.cpp
  src/render/pfd/VerticalDeviation.cpp
  src/render/pfd/VerticalSpeedIndicator.cpp
  src/render/pfd/Hsi.cpp
  src/render/pfd/Chrome.cpp
)

# Generate avionics/Version.h from the release identity cache variables so the
# version is baked into the binaries (used by the update checker and UI).
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/avionics/Version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/avionics/Version.h
  @ONLY)

target_include_directories(avionics-core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_include_directories(avionics-core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(avionics-core PUBLIC cxx_std_17)
target_compile_definitions(avionics-core PRIVATE
  AVIONICS_DEFAULT_EIS="${CMAKE_CURRENT_SOURCE_DIR}/assets/eis/c172s.eis")
