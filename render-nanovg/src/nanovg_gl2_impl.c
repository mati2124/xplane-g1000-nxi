// Single translation unit that compiles NanoVG's OpenGL 2 backend.
//
// X-Plane 12's OpenGL bridge (used by avionics drawing callbacks) exposes only
// an OpenGL 2.1 context, so the in-sim plugin needs the GL2 backend rather than
// the GL3 one the standalone shell uses. NanoVG emits its GL backend only when
// the matching implementation macro is defined, so it must be compiled in
// exactly one .c file; this is that file for GL2 (nanovg_gl_impl.c is GL3).
//
// The GL2 and GL3 implementations expose distinctly-named entry points
// (nvgCreateGL2 vs nvgCreateGL3); all their other symbols are file-local
// (static), so both backends can coexist in the same library.

#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#else
// Windows/Linux: GLEW supplies the GL2 + extension entry points NanoVG calls.
// glew.h must precede any other GL header.
#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/glew.h>
#endif

#include "nanovg.h"

#define NANOVG_GL2_IMPLEMENTATION
#include "nanovg_gl.h"
