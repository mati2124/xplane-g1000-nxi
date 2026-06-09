// Standalone shell.
//
// Owns its own window and frame loop (unlike the plugin, which is driven by the
// sim). Puts the shared PFD on screen via a GLFW window + the shared
// NanoVgRenderer, and can be fed by either the built-in mock data or a live
// X-Plane connection (UDP RREF). Press M at runtime to toggle between them.
//
// On startup (when no --source is given) it starts on the mock feed but keeps
// the X-Plane link warm and switches to it automatically the moment X-Plane
// starts delivering data. Passing --source pins the feed and disables this, as
// does manually toggling with M or the Data Source menu.
//
//   --source mock|xplane     initial data feed (default: auto-detect)
//   --xplane-host HOST        X-Plane host (default: 127.0.0.1)
//   --xplane-port PORT        X-Plane UDP port (default: 49000)

#if defined(__APPLE__)
#include <OpenGL/gl3.h>  // GL_SILENCE_DEPRECATION is set by the build.
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "XPlaneConnection.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/ConnectionState.h"
#include "avionics/MockDataSource.h"
#include "avionics/render/BootScreen.h"
#include "avionics/render/NanoVgRenderer.h"

#if defined(__APPLE__)
#include "MacMenu.h"
#endif

namespace {

constexpr int kWindowWidth = 1024;
constexpr int kWindowHeight = 768;
constexpr const char* kWindowTitle = "XPlane Avionics - PFD";

constexpr const char* kSourceMock = "mock";
constexpr const char* kSourceXPlane = "xplane";
constexpr const char* kLabelMock = "MOCK DATA";
constexpr const char* kDefaultXPlaneHost = "127.0.0.1";
constexpr std::uint16_t kDefaultXPlanePort = 49000;

const char* FlagValue(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
  }
  return nullptr;
}

// Live data sources plus which one the engine is currently driven by, shared
// with the key callback so M can toggle between them.
struct AppState {
  avionics::AvionicsEngine* engine = nullptr;
  avionics::MockDataSource* mock = nullptr;
  avionics::SimulatorConnection* xplane = nullptr;
  bool usingXPlane = false;
  // While true, the main loop keeps the X-Plane link pumped and switches to it
  // as soon as it connects. Cleared once the user takes manual control (M key /
  // menu) or when a --source was given explicitly.
  bool autoDetectXPlane = false;
};

void SwitchSource(AppState& app, bool useXPlane) {
  if (useXPlane == app.usingXPlane) return;
  app.usingXPlane = useXPlane;
  if (useXPlane) {
    app.engine->setDataSource(*app.xplane, app.xplane->simulatorName());
  } else {
    app.engine->setDataSource(*app.mock, kLabelMock);
  }
#if defined(__APPLE__)
  avionics::SetDataSourceMenuSelection(useXPlane);  // keep the menu in sync
#endif
}

// Menu-bar action target: switches the feed when the user picks from the menu.
void OnMenuSelectSource(void* context, bool useXPlane) {
  auto* app = static_cast<AppState*>(context);
  app->autoDetectXPlane = false;  // explicit user choice wins from here on
  SwitchSource(*app, useXPlane);
}

// Renders a single deterministic frame offscreen and writes it to a binary PPM
// (P6). PPM keeps this dependency-free; convert to PNG with `sips` afterwards.
// Returns 0 on success. The mock state is advanced by `seconds` so we can pick a
// clean, representative attitude (seconds = 0 is wings-level, no turbulence).
int RunScreenshot(const char* path, double seconds, const char* state) {
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                        kWindowTitle, nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "Failed to create window\n");
    return 1;
  }
  glfwMakeContextCurrent(window);

  avionics::NanoVgRenderer renderer;
  if (!renderer.valid()) {
    std::fprintf(stderr, "Failed to create NanoVG renderer\n");
    return 1;
  }

  avionics::MockDataSource dataSource;
  avionics::AvionicsEngine engine(dataSource, renderer, kLabelMock);

  int fbWidth = 0;
  int fbHeight = 0;
  glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

  glViewport(0, 0, fbWidth, fbHeight);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // --state selects which screen to capture (default: the live PFD):
  //   pfd  - skip the boot animation and show the live page
  //   boot - the power-on initialization screen
  if (state != nullptr && std::strcmp(state, "boot") == 0) {
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(renderer, kLabelMock, 0.6f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "alerts") == 0) {
    // Drive the real interaction path: bring up the live page, "click" the
    // Alerts softkey, then run the open animation to completion before capture.
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(fbWidth, fbHeight, 1.0f);  // establishes the click space
    const double barH = fbHeight * (35.0 / 768.0);  // softkey row height
    const double cellW = fbWidth / 12.0;
    engine.onPointerDown(11.5 * cellW, fbHeight - barH * 0.5);  // Alerts cell
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.renderFrame(fbWidth, fbHeight, 1.0f);
  } else if (state != nullptr && std::strcmp(state, "menu") == 0) {
    // Exercise the softkey menu state machine: open the PFD Options submenu and
    // turn on a couple of display-option toggles so the captured frame shows the
    // submenu (with its Back key) and the highlighted active cells.
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(fbWidth, fbHeight, 1.0f);  // establishes the click space
    const double barH = fbHeight * (35.0 / 768.0);
    const double cellW = fbWidth / 12.0;
    const double rowY = fbHeight - barH * 0.5;
    engine.onPointerDown(3.5 * cellW, rowY);  // "PFD Opt" -> open submenu
    engine.onPointerDown(1.5 * cellW, rowY);  // "SVT" toggle on
    engine.onPointerDown(2.5 * cellW, rowY);  // "Wind" toggle on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.renderFrame(fbWidth, fbHeight, 1.0f);
  } else {
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(fbWidth, fbHeight, 1.0f);
  }
  glFinish();

  std::vector<unsigned char> pixels(
      static_cast<size_t>(fbWidth) * fbHeight * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, fbWidth, fbHeight, GL_RGB, GL_UNSIGNED_BYTE,
               pixels.data());

  FILE* f = std::fopen(path, "wb");
  if (!f) {
    std::fprintf(stderr, "Failed to open %s for writing\n", path);
    return 1;
  }
  std::fprintf(f, "P6\n%d %d\n255\n", fbWidth, fbHeight);
  // GL framebuffer origin is bottom-left; flip rows to top-left for the image.
  const size_t rowBytes = static_cast<size_t>(fbWidth) * 3;
  for (int y = fbHeight - 1; y >= 0; --y) {
    std::fwrite(pixels.data() + static_cast<size_t>(y) * rowBytes, 1, rowBytes,
                f);
  }
  std::fclose(f);
  std::fprintf(stderr, "Wrote screenshot %s (%dx%d)\n", path, fbWidth,
               fbHeight);

  glfwDestroyWindow(window);
  return 0;
}

void OnKey(GLFWwindow* window, int key, int /*scancode*/, int action,
           int /*mods*/) {
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }
  // M toggles between the mock feed and the live X-Plane connection.
  if (key == GLFW_KEY_M) {
    auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (app != nullptr) {
      app->autoDetectXPlane = false;  // explicit user choice wins from here on
      SwitchSource(*app, !app->usingXPlane);
    }
  }
}

// Forward left-clicks to the engine. The engine works in framebuffer pixels
// (what renderFrame is given), while GLFW reports the cursor in window points,
// so scale by the framebuffer/window ratio to stay correct on HiDPI displays.
void OnMouseButton(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app == nullptr || app->engine == nullptr) return;

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);

  int winW = 0, winH = 0, fbW = 0, fbH = 0;
  glfwGetWindowSize(window, &winW, &winH);
  glfwGetFramebufferSize(window, &fbW, &fbH);
  const double sx = winW > 0 ? static_cast<double>(fbW) / winW : 1.0;
  const double sy = winH > 0 ? static_cast<double>(fbH) / winH : 1.0;
  app->engine->onPointerDown(cursorX * sx, cursorY * sy);
}

// Keep the window floating above other windows when requested via the
// --always-on-top flag or the AVIONICS_ALWAYS_ON_TOP env var. Handy for
// iterating in the editor while watching the PFD.
bool WantsAlwaysOnTop(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--always-on-top") == 0) return true;
  }
  const char* env = std::getenv("AVIONICS_ALWAYS_ON_TOP");
  return env != nullptr && env[0] != '\0' && std::strcmp(env, "0") != 0;
}

}  // namespace

int main(int argc, char** argv) {
  const bool alwaysOnTop = WantsAlwaysOnTop(argc, argv);

  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  // Offscreen single-frame capture mode for development/iteration:
  //   avionics-standalone --screenshot out.ppm [--time SECONDS]
  if (const char* shot = FlagValue(argc, argv, "--screenshot")) {
    const char* timeStr = FlagValue(argc, argv, "--time");
    const double seconds = timeStr ? std::atof(timeStr) : 0.0;
    const char* state = FlagValue(argc, argv, "--state");
    const int rc = RunScreenshot(shot, seconds, state);
    glfwTerminate();
    return rc;
  }

  // NanoVG's GL3 backend needs a 3.2+ core profile context.
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // GLFW_FLOATING keeps the window above normal windows ("always on top").
  if (alwaysOnTop) glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);

  GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                        kWindowTitle, nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "Failed to create window\n");
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // vsync: paces the loop to the display refresh.
  glfwSetKeyCallback(window, OnKey);
  glfwSetMouseButtonCallback(window, OnMouseButton);

  // The renderer creates its NanoVG GL context now, so it must be constructed
  // after a context is current.
  avionics::NanoVgRenderer renderer;
  if (!renderer.valid()) {
    std::fprintf(stderr, "Failed to create NanoVG renderer\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  // Both feeds exist for the whole session; the engine is pointed at one at a
  // time and M swaps between them. Opening the X-Plane UDP socket up front is
  // cheap and lets the connection start acquiring data immediately.
  const char* host = FlagValue(argc, argv, "--xplane-host");
  const char* portStr = FlagValue(argc, argv, "--xplane-port");
  const std::uint16_t port = portStr
                                 ? static_cast<std::uint16_t>(std::atoi(portStr))
                                 : kDefaultXPlanePort;

  avionics::MockDataSource mock;
  avionics::XPlaneConnection xplane(host ? host : kDefaultXPlaneHost, port);

  const char* sourceArg = FlagValue(argc, argv, "--source");
  const bool startWithXPlane =
      sourceArg != nullptr && std::strcmp(sourceArg, kSourceXPlane) == 0;

  AppState app;
  app.mock = &mock;
  app.xplane = &xplane;
  app.usingXPlane = startWithXPlane;
  // With no explicit --source, start on mock and auto-switch to X-Plane once it
  // is detected. An explicit --source pins the feed instead.
  app.autoDetectXPlane = sourceArg == nullptr;

  avionics::AvionicsEngine engine(
      startWithXPlane ? static_cast<avionics::DataSource&>(xplane)
                      : static_cast<avionics::DataSource&>(mock),
      renderer, startWithXPlane ? xplane.simulatorName() : kLabelMock);
  app.engine = &engine;
  glfwSetWindowUserPointer(window, &app);

#if defined(__APPLE__)
  // Add the "Data Source" menu (Mock Data / X-Plane) to the macOS menu bar.
  avionics::InstallDataSourceMenu(startWithXPlane, &OnMenuSelectSource, &app);
#endif

  using clock = std::chrono::steady_clock;
  auto previous = clock::now();

  while (!glfwWindowShouldClose(window)) {
    const auto now = clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;

    int fbWidth = 0;
    int fbHeight = 0;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Auto-detect: while showing mock, keep the X-Plane link pumped (the engine
    // only updates the active source) and switch over the instant it connects.
    if (app.autoDetectXPlane && !app.usingXPlane) {
      xplane.update(dt);
      if (xplane.connectionState() == avionics::ConnectionState::Connected) {
        SwitchSource(app, true);
      }
    }

    engine.update(dt);
    // The gauge code draws directly in framebuffer pixels, so the NanoVG
    // device-pixel-ratio is 1.0 (the coordinate space is already device px).
    engine.renderFrame(fbWidth, fbHeight, 1.0f);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
