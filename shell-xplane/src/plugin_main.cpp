// X-Plane plugin shell.
//
// Hosts the shared avionics-core inside the sim. X-Plane drives us: we register
// a drawing callback and render into the sim's active graphics context during
// its frame. We do NOT own the loop and must stay within the frame budget.
//
// Build: requires the X-Plane SDK (set XPLANE_SDK_DIR) and a NanoVG renderer
// bound to the sim's GL/Vulkan/Metal context (see TODO below).

#include <cstring>
#include <memory>

#include "DatarefDataSource.h"
#include "XPLMDisplay.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "avionics/AvionicsEngine.h"

// TODO: implement against the sim's graphics context.
//   - X-Plane 12 default backend is Vulkan (Win/Linux) / Metal (macOS); OpenGL
//     is legacy/compat. NanoVG has GL2/GL3 backends; for Vulkan/Metal you bridge
//     through the appropriate context or use OpenGL compatibility mode.
//   - beginFrame()/endFrame() should set up and flush the NanoVG frame.
#include "avionics/Renderer.h"
namespace avionics {
class NanoVgRenderer;  // forward decl; concrete impl lives in render-nanovg/.
}

namespace {

std::unique_ptr<avionics::DatarefDataSource> g_dataSource;
std::unique_ptr<avionics::Renderer> g_renderer;  // NanoVgRenderer once wired.
std::unique_ptr<avionics::AvionicsEngine> g_engine;

// The PFD display geometry (panel pixels). Real builds map this to the 3D
// panel texture region or a popup window.
constexpr int kDisplayWidth = 1024;
constexpr int kDisplayHeight = 768;

int DrawCallback(XPLMDrawingPhase /*phase*/, int /*isBefore*/, void* /*ref*/) {
  if (g_engine) {
    g_engine->update(0.0 /* dt: derive from XPLMGetElapsedTime deltas */);
    g_engine->renderFrame(kDisplayWidth, kDisplayHeight, 1.0f);
  }
  return 1;  // allow X-Plane to continue its own drawing
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
  std::strcpy(outName, "PoolFlow Avionics");
  std::strcpy(outSig, "com.andrewmiller.xplaneavionics");
  std::strcpy(outDesc, "Shared-core glass cockpit (PFD/MFD) for X-Plane.");

  g_dataSource = std::make_unique<avionics::DatarefDataSource>();
  // g_renderer = std::make_unique<avionics::NanoVgRenderer>(); // TODO
  if (g_renderer) {
    g_engine = std::make_unique<avionics::AvionicsEngine>(*g_dataSource,
                                                          *g_renderer);
  }

  XPLMRegisterDrawCallback(DrawCallback, xplm_Phase_Gauges, 0 /*after*/,
                           nullptr);
  return 1;
}

PLUGIN_API void XPluginStop(void) {
  XPLMUnregisterDrawCallback(DrawCallback, xplm_Phase_Gauges, 0, nullptr);
  g_engine.reset();
  g_renderer.reset();
  g_dataSource.reset();
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
