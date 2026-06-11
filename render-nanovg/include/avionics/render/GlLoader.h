#pragma once

namespace avionics {
namespace render {

// Initialize the OpenGL function loader for the currently-current GL context.
//
// NanoVG's GL3 (standalone) and GL2 (X-Plane plugin) backends call OpenGL
// entry points beyond the 1.1 baseline that Windows' opengl32 and Linux' libGL
// export directly, so on those platforms the pointers must be resolved at
// runtime via a loader (GLEW). Call this once after a GL context is made current
// and before constructing a NanoVgRenderer in that context.
//
// No-op on macOS, where the system OpenGL framework exposes every entry point at
// link time. Returns false if the loader failed (the renderer should then be
// treated as unavailable).
bool ensureGlLoaded();

}  // namespace render
}  // namespace avionics
