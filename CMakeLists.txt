cmake_minimum_required(VERSION 3.20)
# C is needed in addition to C++ because the NanoVG renderer and GLFW compile
# C sources fetched at configure time.
project(XPlaneAvionics VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MSVC does not define M_PI unless this is set before <cmath>. NOMINMAX keeps the
# Windows min/max macros from breaking std::min/std::max.
if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Release identity. AVIONICS_VERSION defaults to the project() version above and
# is overridden by CI from the pushed git tag (release.yml passes
# -DAVIONICS_VERSION=<tag without leading v>). The owner/repo feed the in-app
# update checker that queries the GitHub Releases API.
set(AVIONICS_VERSION "${PROJECT_VERSION}" CACHE STRING "Release version string")
set(AVIONICS_RELEASES_OWNER "andywmm9-pixel" CACHE STRING "GitHub releases owner")
set(AVIONICS_RELEASES_REPO "xplane-g1000-nxi" CACHE STRING "GitHub releases repo")

# One engine, two shells. The core always builds; each shell is opt-in because
# it pulls in heavier platform dependencies (X-Plane SDK / windowing + GPU).
option(BUILD_XPLANE_SHELL "Build the X-Plane plugin shell (requires XPLM SDK)" OFF)
option(BUILD_STANDALONE_SHELL "Build the standalone desktop shell" ON)

# Linux X-Plane plugins are shared objects; static libs they link must be -fPIC.
if(BUILD_XPLANE_SHELL AND UNIX AND NOT APPLE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# --- Packaging: bundled runtime assets -------------------------------------
# The binaries resolve these at runtime relative to the install location (see
# avionics/AssetPaths.h). The install rules below stage them next to each shell
# so a release archive is self-contained; CI installs per-component into a
# staging dir and the installers wrap that up.
# Always-present (tracked) fonts shared by both shells.
set(AVIONICS_FONT_FILES
  ${CMAKE_SOURCE_DIR}/render-nanovg/assets/fonts/Roboto-Regular.ttf
  ${CMAKE_SOURCE_DIR}/render-nanovg/assets/fonts/Roboto-Bold.ttf
  ${CMAKE_SOURCE_DIR}/render-nanovg/assets/fonts/LICENSE_OFL.txt)
# Optional fonts (the DejaVu secondary display face + its license). Installed
# when present but not required, so a checkout without them still packages.
set(AVIONICS_FONT_FILES_OPTIONAL
  ${CMAKE_SOURCE_DIR}/render-nanovg/assets/fonts/DejaVuSans-SemiBold.ttf
  ${CMAKE_SOURCE_DIR}/render-nanovg/assets/fonts/LICENSE_DejaVu.txt)
set(AVIONICS_EIS_FILE ${CMAKE_SOURCE_DIR}/avionics-core/assets/eis/c172s.eis)
set(AVIONICS_CHECKLIST_FILE ${CMAKE_SOURCE_DIR}/shell-standalone/assets/checklists.txt)
# Generated map overlay (large, often gitignored); optional so the build/package
# never fails when it has not been generated/committed.
set(AVIONICS_LAND_FILE ${CMAKE_SOURCE_DIR}/shell-standalone/assets/land_data.bin)

# X-Plane plugin platform subfolder (<plugin>/<platform>/<plugin>.xpl).
if(APPLE)
  set(AVIONICS_XPL_PLATFORM_DIR "mac_x64")
elseif(WIN32)
  set(AVIONICS_XPL_PLATFORM_DIR "win_x64")
else()
  set(AVIONICS_XPL_PLATFORM_DIR "lin_x64")
endif()

# Install the fonts + EIS sample shared by both shells into <dest>, under the
# given install COMPONENT, mirroring the runtime "assets/" layout.
function(avionics_install_shared_assets dest component)
  install(FILES ${AVIONICS_FONT_FILES} DESTINATION ${dest}/fonts COMPONENT ${component})
  install(FILES ${AVIONICS_FONT_FILES_OPTIONAL} DESTINATION ${dest}/fonts COMPONENT ${component} OPTIONAL)
  install(FILES ${AVIONICS_EIS_FILE} DESTINATION ${dest}/eis COMPONENT ${component})
endfunction()

add_subdirectory(avionics-core)

# HTTPS fetch for the notify-only update checker (GitHub Releases API).
if(BUILD_XPLANE_SHELL OR BUILD_STANDALONE_SHELL)
  find_package(CURL REQUIRED)
  target_compile_definitions(avionics-core PRIVATE AVIONICS_HAS_CURL)
  target_link_libraries(avionics-core PRIVATE CURL::libcurl)
endif()

# The NanoVG renderer is shared by both shells; build it whenever a shell that
# needs to put pixels on screen is enabled.
if(BUILD_XPLANE_SHELL OR BUILD_STANDALONE_SHELL)
  add_subdirectory(render-nanovg)
endif()

if(BUILD_XPLANE_SHELL)
  add_subdirectory(shell-xplane)
  add_subdirectory(tools/reload-plugins)
endif()

if(BUILD_STANDALONE_SHELL)
  add_subdirectory(shell-standalone)
endif()
