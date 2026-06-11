# Standalone shell. Produces a desktop executable that links the shared core,
# the shared NanoVG renderer, and a GLFW window owning a real 60 fps loop.
#
# GLFW is fetched and built at configure time, so no system package is needed.
# Data still comes from MockDataSource until the WebSocket bridge lands.

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
)

# The Web API client polls X-Plane on a background thread.
find_package(Threads REQUIRED)

# HTTPS client for the SimBrief OFP fetch. libcurl ships with macOS and most
# Linux distributions; on Windows install it via vcpkg or similar.
find_package(CURL REQUIRED)

# Native macOS menu-bar integration (Objective-C++, Cocoa).
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
