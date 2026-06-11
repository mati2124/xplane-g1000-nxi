# Standalone shell. Produces a desktop executable that links the shared core,
# the shared NanoVG renderer, and a GLFW window owning a real 60 fps loop.
#
# GLFW is fetched and built at configure time, so no system package is needed.
# The displays read a live X-Plane connection (UDP RREF / Web API).

include(FetchContent)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
# Linux CI/dev boxes often lack wayland-scanner; the standalone shell only needs
# X11 on Linux (same as a typical desktop session).
if(UNIX AND NOT APPLE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
endif()
FetchContent_MakeAvailable(glfw)

# JSON parser for the SimBrief OFP fetch (header-only, fetched like GLFW).
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(nlohmann_json)

set(STANDALONE_SOURCES
  src/main.cpp
  src/AppSettings.cpp
  src/XPlaneConnection.cpp
  src/XPlaneWebApi.cpp
  src/FlightPlanBridgeClient.cpp
  src/NavData.cpp
  src/FmsPlanStore.cpp
  src/AirspaceStore.cpp
  src/AirwayStore.cpp
  src/AptDatStore.cpp
  src/ProcedureStore.cpp
  src/LandDataStore.cpp
  src/ObstacleStore.cpp
  src/ChecklistStore.cpp
  src/XPlaneInstall.cpp
  src/DsfTerrainStore.cpp
  src/SimBriefClient.cpp
  src/SimBriefStore.cpp
  src/UpdateNotify.cpp
)

# The Web API client polls X-Plane on a background thread.
find_package(Threads REQUIRED)

# HTTPS client for the SimBrief OFP fetch. libcurl ships with macOS and most
# Linux distributions; on Windows install it via vcpkg or similar.
find_package(CURL REQUIRED)

# Native macOS update-available alert (Objective-C++, Cocoa).
if(APPLE)
  list(APPEND STANDALONE_SOURCES src/MacMenu.mm)
  set_source_files_properties(src/MacMenu.mm PROPERTIES COMPILE_FLAGS "-fobjc-arc")
endif()

add_executable(avionics-standalone ${STANDALONE_SOURCES})

# avionics-core arrives transitively (PUBLIC) through render-nanovg.
target_link_libraries(avionics-standalone PRIVATE
  render-nanovg
  glfw
  Threads::Threads
  CURL::libcurl
  nlohmann_json::nlohmann_json
)
target_include_directories(avionics-standalone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(avionics-standalone PRIVATE cxx_std_17)

# Bundled sample checklist file, used by the MFD Checklist page group when no
# --checklist is given (so the page is populated during development).
target_compile_definitions(avionics-standalone PRIVATE
  AVIONICS_DEFAULT_CHECKLIST="${CMAKE_CURRENT_SOURCE_DIR}/assets/checklists.txt")

target_compile_definitions(avionics-standalone PRIVATE
  AVIONICS_DEFAULT_EIS="${CMAKE_CURRENT_SOURCE_DIR}/../avionics-core/assets/eis/c172s.eis")

# Bundled Natural Earth land-data asset for the map's rivers/roads/cities/
# borders overlay (built by tools/convert_natural_earth.py; loading fails
# gracefully when the file is absent).
target_compile_definitions(avionics-standalone PRIVATE
  AVIONICS_LAND_DATA="${CMAKE_CURRENT_SOURCE_DIR}/assets/land_data.bin")

if(APPLE)
  target_compile_definitions(avionics-standalone PRIVATE GL_SILENCE_DEPRECATION)
  target_link_libraries(avionics-standalone PRIVATE "-framework OpenGL" "-framework Cocoa")
endif()

# X-Plane UDP connection needs Winsock on Windows.
if(WIN32)
  target_link_libraries(avionics-standalone PRIVATE ws2_32)
endif()

# --- Install / packaging ----------------------------------------------------
# Portable layout: the executable with an assets/ folder beside it (resolved at
# runtime by avionics/AssetPaths.h). CI installs the "standalone" component into
# a staging dir and the per-platform installer wraps it (.app/.dmg, Inno Setup,
# AppImage). On macOS the installer relocates the binary into a .app bundle;
# AssetPaths also searches ../Resources/assets to cover that case.
install(TARGETS avionics-standalone
  RUNTIME DESTINATION . COMPONENT standalone
  BUNDLE DESTINATION . COMPONENT standalone)
avionics_install_shared_assets("assets" standalone)
install(FILES ${AVIONICS_CHECKLIST_FILE} DESTINATION assets COMPONENT standalone)
install(FILES ${AVIONICS_LAND_FILE} DESTINATION assets COMPONENT standalone OPTIONAL)
