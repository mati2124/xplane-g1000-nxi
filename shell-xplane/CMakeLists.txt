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

# DsfTerrainStore (and its XPlaneInstall dependency) are shared with the
# standalone shell: they read X-Plane's Global Scenery DSF DEM tiles for the
# topographic map background. The plugin resolves the scenery path via the SDK
# rather than the install-list files, so only the explicit-directory path is
# exercised here.
add_library(xplane-avionics MODULE
  src/plugin_main.cpp
  src/UpdateNotify.cpp
  src/DatarefDataSource.cpp
  src/DatarefWeatherRadar.cpp
  src/FlightPlanBridge.cpp
  src/CommandBridge.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../shell-standalone/src/DsfTerrainStore.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../shell-standalone/src/XPlaneInstall.cpp
  # Shared portable SHA-256 (also used by the standalone updater) verifies the
  # downloaded plugin archive against the release's SHA256SUMS.
  ${CMAKE_CURRENT_SOURCE_DIR}/../shell-standalone/src/Sha256.cpp
)

target_include_directories(xplane-avionics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../shell-standalone/src
  ${XPLANE_SDK_DIR}/CHeaders/XPLM
  ${XPLANE_SDK_DIR}/CHeaders/Widgets
)

# SDK version + platform macros expected by the XPLM headers. XPLM410 unlocks
# the FMS flight-plan write APIs used by the flight-plan bridge (true
# present-position Direct-To); requires the X-Plane 12.1+ SDK and runtime.
target_compile_definitions(xplane-avionics PRIVATE XPLM200 XPLM210 XPLM300 XPLM301 XPLM303 XPLM400 XPLM410 XPLM420)
if(APPLE)
  target_compile_definitions(xplane-avionics PRIVATE APL=1 IBM=0 LIN=0)
elseif(WIN32)
  target_compile_definitions(xplane-avionics PRIVATE APL=0 IBM=1 LIN=0)
else()
  target_compile_definitions(xplane-avionics PRIVATE APL=0 IBM=0 LIN=1)
endif()

target_compile_definitions(xplane-avionics PRIVATE
  AVIONICS_DEFAULT_EIS="${CMAKE_CURRENT_SOURCE_DIR}/../avionics-core/assets/eis/c172s.eis")

# Airspace parsing runs on a background std::thread.
find_package(Threads REQUIRED)

# HTTPS for the in-sim update checker / self-updater (GitHub Releases API +
# plugin archive download). Mirrors the standalone shell's curl usage.
find_package(CURL REQUIRED)
target_compile_definitions(xplane-avionics PRIVATE AVIONICS_HAS_CURL)

# render-nanovg brings the shared NanoVG Renderer backend (and, transitively,
# avionics-core, NanoVG, and the platform OpenGL framework) so the plugin can
# draw the PFD/MFD into the sim's avionics device screens.
target_link_libraries(xplane-avionics PRIVATE
  render-nanovg Threads::Threads CURL::libcurl)

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
else()
  target_link_libraries(xplane-avionics PRIVATE
    ${XPLANE_SDK_DIR}/Libraries/Lin/XPLM_64.so
  )
endif()

if(WIN32)
  # Every translation unit must see winsock2.h before windows.h (XPLM headers
  # include the latter). WinSockFirst.h is force-included for the whole target.
  target_compile_options(xplane-avionics PRIVATE /FI"WinSockFirst.h")
endif()

# --- Install / packaging ----------------------------------------------------
# X-Plane plugin folder layout, ready to drop into <X-Plane>/Resources/plugins:
#   xplane-avionics/<platform>/xplane-avionics.xpl
#   xplane-avionics/assets/...   (shared fonts + EIS sample, resolved at runtime)
# CI installs the "plugin" component into a staging dir; the installer copies the
# xplane-avionics/ tree into the user's selected X-Plane install.
install(TARGETS xplane-avionics
  LIBRARY DESTINATION "xplane-avionics/${AVIONICS_XPL_PLATFORM_DIR}" COMPONENT plugin)
avionics_install_shared_assets("xplane-avionics/assets" plugin)
