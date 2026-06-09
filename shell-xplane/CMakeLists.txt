# X-Plane plugin shell. Produces a .xpl module that links the shared core.
#
# Requires the X-Plane SDK: download from https://developer.x-plane.com/sdk/
# and configure with -DXPLANE_SDK_DIR=/path/to/SDK (the folder containing
# CHeaders/ and Libraries/).

set(XPLANE_SDK_DIR "" CACHE PATH "Path to the X-Plane SDK root")
if(NOT XPLANE_SDK_DIR)
  message(FATAL_ERROR
    "BUILD_XPLANE_SHELL=ON requires -DXPLANE_SDK_DIR=/path/to/X-Plane-SDK")
endif()

add_library(xplane-avionics MODULE
  src/plugin_main.cpp
  src/DatarefDataSource.cpp
)

target_include_directories(xplane-avionics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${XPLANE_SDK_DIR}/CHeaders/XPLM
  ${XPLANE_SDK_DIR}/CHeaders/Widgets
)

# SDK version + platform macros expected by the XPLM headers.
target_compile_definitions(xplane-avionics PRIVATE XPLM200 XPLM210 XPLM300 XPLM301 XPLM303 XPLM400)
if(APPLE)
  target_compile_definitions(xplane-avionics PRIVATE APL=1 IBM=0 LIN=0)
elseif(WIN32)
  target_compile_definitions(xplane-avionics PRIVATE APL=0 IBM=1 LIN=0)
else()
  target_compile_definitions(xplane-avionics PRIVATE APL=0 IBM=0 LIN=1)
endif()

target_link_libraries(xplane-avionics PRIVATE avionics-core)

# X-Plane loads plugins as <name>/<platform>/<name>.xpl with no lib prefix.
set_target_properties(xplane-avionics PROPERTIES
  PREFIX ""
  OUTPUT_NAME "xplane-avionics"
  SUFFIX ".xpl"
)

if(APPLE)
  target_link_libraries(xplane-avionics PRIVATE
    "-F${XPLANE_SDK_DIR}/Libraries/Mac"
    "-framework XPLM"
  )
elseif(WIN32)
  target_link_libraries(xplane-avionics PRIVATE
    ${XPLANE_SDK_DIR}/Libraries/Win/XPLM_64.lib
  )
endif()
