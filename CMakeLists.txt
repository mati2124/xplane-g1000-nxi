cmake_minimum_required(VERSION 3.20)
# C is needed in addition to C++ because the NanoVG renderer and GLFW compile
# C sources fetched at configure time.
project(XPlaneAvionics VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# One engine, two shells. The core always builds; each shell is opt-in because
# it pulls in heavier platform dependencies (X-Plane SDK / windowing + GPU).
option(BUILD_XPLANE_SHELL "Build the X-Plane plugin shell (requires XPLM SDK)" OFF)
option(BUILD_STANDALONE_SHELL "Build the standalone desktop shell" ON)

add_subdirectory(avionics-core)

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
