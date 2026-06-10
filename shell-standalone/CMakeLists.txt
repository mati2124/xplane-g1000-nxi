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

set(STANDALONE_SOURCES
  src/main.cpp
  src/XPlaneConnection.cpp
  src/XPlaneWebApi.cpp
  src/NavData.cpp
  src/FmsPlanStore.cpp
  src/AirspaceStore.cpp
  src/ChecklistStore.cpp
  src/XPlaneInstall.cpp
  src/DsfTerrainStore.cpp
)

# The Web API client polls X-Plane on a background thread.
find_package(Threads REQUIRED)

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
)
target_include_directories(avionics-standalone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(avionics-standalone PRIVATE cxx_std_17)

# Bundled sample checklist file, used by the MFD Checklist page group when no
# --checklist is given (so the page is populated during development).
target_compile_definitions(avionics-standalone PRIVATE
  AVIONICS_DEFAULT_CHECKLIST="${CMAKE_CURRENT_SOURCE_DIR}/assets/checklists.txt")

if(APPLE)
  target_compile_definitions(avionics-standalone PRIVATE GL_SILENCE_DEPRECATION)
  target_link_libraries(avionics-standalone PRIVATE "-framework OpenGL" "-framework Cocoa")
endif()

# X-Plane UDP connection needs Winsock on Windows.
if(WIN32)
  target_link_libraries(avionics-standalone PRIVATE ws2_32)
endif()
