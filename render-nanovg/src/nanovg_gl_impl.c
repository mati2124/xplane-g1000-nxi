// Single translation unit that compiles NanoVG's OpenGL 3 backend.
//
// NanoVG ships its GL backend as a header that emits its implementation only
// when NANOVG_GL3_IMPLEMENTATION is defined, so it must be compiled in exactly
// one .c file. We bind it to the platform's GL headers here and nowhere else.

#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#else
// Windows/Linux: GLEW supplies the GL3 entry points NanoVG calls (opengl32 /
// libGL don't export them directly). avionics::render::ensureGlLoaded() must run
// after the context is current and before nvgCreateGL3. glew.h must precede any
// other GL header.
#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/glew.h>
#endif

#include "nanovg.h"

#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg_gl.h"
