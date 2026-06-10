# Platform-agnostic avionics engine: all logic + drawing, zero platform deps.
# Builds cleanly on its own so the shared code can be developed and tested
# without the X-Plane SDK or a windowing/GPU stack present.
add_library(avionics-core STATIC
  src/AvionicsEngine.cpp
  src/ChecklistParser.cpp
  src/MockDataSource.cpp
  src/SoftkeyController.cpp
  src/render/BezelKeys.cpp
  src/render/BezelStyle.cpp
  src/render/BootScreen.cpp
  src/render/SoftkeyBezel.cpp
  src/render/PrimaryFlightDisplay.cpp
  src/render/mfd/EisStrip.cpp
  src/render/mfd/MfdController.cpp
  src/render/mfd/MfdPages.cpp
  src/render/mfd/MultiFunctionDisplay.cpp
  src/nav/OpenAirParser.cpp
  src/nav/Terrain.cpp
  src/render/map/MapProjection.cpp
  src/render/map/MapView.cpp
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

target_include_directories(avionics-core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(avionics-core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(avionics-core PUBLIC cxx_std_17)
