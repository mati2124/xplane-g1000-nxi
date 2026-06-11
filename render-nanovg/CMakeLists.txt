# Shared NanoVG renderer backend implementing avionics::Renderer.
#
# Both shells link this; it expects a GPU (OpenGL) context to be current when a
# NanoVgRenderer is constructed. NanoVG itself is fetched at configure time and
# compiled here (upstream ships no CMake), so no system package is required.

include(FetchContent)

FetchContent_Declare(
  nanovg
  GIT_REPOSITORY https://github.com/memononen/nanovg.git
  # Upstream has no tagged releases; pin a commit for reproducible builds.
  GIT_TAG f93799c078fa11ed61c078c65a53914c8782c00b
)
FetchContent_MakeAvailable(nanovg)

# NanoVG core (C, GL-agnostic) + our GL3 and GL2 backend translation units. The
# standalone shell drives the GL3 backend (its own 3.2 core context); the
# X-Plane plugin drives the GL2 backend (the sim's OpenGL 2.1 bridge).
add_library(nanovg STATIC
  ${nanovg_SOURCE_DIR}/src/nanovg.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/nanovg_gl_impl.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/nanovg_gl2_impl.c
)
target_include_directories(nanovg PUBLIC ${nanovg_SOURCE_DIR}/src)

if(APPLE)
  target_compile_definitions(nanovg PRIVATE GL_SILENCE_DEPRECATION)
  target_link_libraries(nanovg PUBLIC "-framework OpenGL")
endif()

# C++ renderer that drives NanoVG from the gauge code's vector commands.
add_library(render-nanovg STATIC
  src/NanoVgRenderer.cpp
)
target_include_directories(render-nanovg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(render-nanovg PUBLIC cxx_std_17)
target_link_libraries(render-nanovg PUBLIC avionics-core nanovg)

# Bundled display fonts (Roboto, OFL). The renderer loads these first so the
# look is consistent across machines instead of depending on a system font.
target_compile_definitions(render-nanovg PRIVATE
  AVIONICS_FONT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts")
