// Single translation unit that compiles NanoVG's OpenGL 3 backend.
//
// NanoVG ships its GL backend as a header that emits its implementation only
// when NANOVG_GL3_IMPLEMENTATION is defined, so it must be compiled in exactly
// one .c file. We bind it to the platform's GL headers here and nowhere else.

#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
// On Windows/Linux a loader (glad/GLEW) must provide GL3 entry points; wire one
// in alongside this file when those platforms are brought up.
#else
#include <GL/gl.h>
#endif

#include "nanovg.h"

#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg_gl.h"
