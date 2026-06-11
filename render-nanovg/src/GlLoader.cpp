#include "avionics/render/GlLoader.h"

#if !defined(__APPLE__)
#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/glew.h>
#endif

namespace avionics {
namespace render {

bool ensureGlLoaded() {
#if defined(__APPLE__)
  // macOS links the system OpenGL framework, which provides all entry points.
  return true;
#else
  // Resolve entry points for the current context. glewExperimental ensures
  // pointers are fetched even under a core profile. Safe to call again per
  // context (the standalone has separate PFD/MFD contexts); the cost is trivial
  // and only paid at startup.
  glewExperimental = GL_TRUE;
  return glewInit() == GLEW_OK;
#endif
}

}  // namespace render
}  // namespace avionics
