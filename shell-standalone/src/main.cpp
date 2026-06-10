// Standalone shell.
//
// Owns its own windows and frame loop (unlike the plugin, which is driven by the
// sim). Puts the shared PFD and MFD on screen as two GLFW windows (each with its
// own GL context + NanoVgRenderer + AvionicsEngine, both reading one shared data
// source), fed by either the built-in mock data or a live X-Plane connection
// (UDP RREF). Press M at runtime to toggle the feed. Pass --no-mfd to run the
// PFD window only.
//
// On startup (when no --source is given) it starts on the mock feed but keeps
// the X-Plane link warm and switches to it automatically the moment X-Plane
// starts delivering data. Passing --source pins the feed and disables this, as
// does manually toggling with M or the Data Source menu.
//
//   --source mock|xplane     initial data feed (default: auto-detect)
//   --no-mfd                  open only the PFD window (no MFD)
//   --xplane-host HOST        X-Plane host (default: 127.0.0.1)
//   --xplane-port PORT        X-Plane UDP port (default: 49000)
//   --fms-plan NAME           .fms flight plan for the inset map route
//                             (name under Output/FMS plans/, or a full path;
//                             default: most recently modified .fms in that dir)

#if defined(__APPLE__)
#include <OpenGL/gl3.h>  // GL_SILENCE_DEPRECATION is set by the build.
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>

#include "FmsPlanStore.h"
#include "NavData.h"
#include "XPlaneConnection.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/ConnectionState.h"
#include "avionics/MockDataSource.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/BootScreen.h"
#include "avionics/render/NanoVgRenderer.h"

#if defined(__APPLE__)
#include "MacMenu.h"
#endif

namespace {

constexpr int kWindowWidth = 1024;   // avionics "screen" region (4:3)
constexpr int kWindowHeight = 768;
// The hardware bezel strip is part of the window, to the right of the screen,
// the way a GDU's physical keys frame the display. The window is screen + strip.
constexpr int kBezelStripWidth = 112;
constexpr int kSuiteWidth = kWindowWidth + kBezelStripWidth;
constexpr const char* kWindowTitle = "XPlane Avionics - PFD";
constexpr const char* kMfdWindowTitle = "XPlane Avionics - MFD";

// Horizontal gap between the PFD and MFD windows when both open.
constexpr int kWindowGap = 24;

// Width of the bezel strip in framebuffer pixels for a given window framebuffer
// width, scaled so the screen:strip ratio is fixed across HiDPI back buffers.
inline int BezelStripPx(int fbWidth) {
  return static_cast<int>(std::lround(static_cast<double>(fbWidth) *
                                      kBezelStripWidth / kSuiteWidth));
}

// Renders one engine into the left "screen" region of its window and draws the
// hardware bezel strip on the right. The gauge code draws in screen pixels
// (device-pixel-ratio 1.0); the bezel is drawn in full-window pixels. update()
// is the caller's responsibility.
inline void RenderSuite(avionics::NanoVgRenderer& renderer,
                        avionics::AvionicsEngine& eng, int fbWidth,
                        int fbHeight) {
  const int bezelPx = BezelStripPx(fbWidth);
  const int screenW = std::max(1, fbWidth - bezelPx);

  glViewport(0, 0, screenW, fbHeight);
  eng.renderFrame(screenW, fbHeight, 1.0f);

  glViewport(0, 0, fbWidth, fbHeight);
  renderer.beginFrame(fbWidth, fbHeight, 1.0f);
  avionics::BezelKeyPanel::render(
      renderer, static_cast<float>(screenW), 0.0f,
      static_cast<float>(fbWidth - screenW), static_cast<float>(fbHeight),
      static_cast<float>(fbHeight), eng.bezelPressLevels());
  renderer.endFrame();
}

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

// Serves a fixed snapshot while reporting the link as down. Used by
// --state failed to capture the reversionary display the engine draws when the
// simulator stops sending packets (every instrument red-X'd, chrome dashed).
class StaleSource : public avionics::DataSource {
 public:
  explicit StaleSource(avionics::FlightData data) : data_(std::move(data)) {}
  void update(double /*dtSeconds*/) override {}
  const avionics::FlightData& snapshot() const override { return data_; }
  avionics::ConnectionState connectionState() const override {
    return avionics::ConnectionState::Disconnected;
  }

 private:
  avionics::FlightData data_;
};

// Live data sources plus which one the engines are currently driven by, shared
// with the input callbacks. The PFD and MFD run as two windows, each with its
// own engine/renderer, but both read the same data source (the PFD engine pumps
// it; the MFD engine does not, to avoid double-stepping it).
struct AppState {
  avionics::AvionicsEngine* pfdEngine = nullptr;
  avionics::AvionicsEngine* mfdEngine = nullptr;  // null when --no-mfd
  GLFWwindow* pfdWindow = nullptr;
  GLFWwindow* mfdWindow = nullptr;
  avionics::MockDataSource* mock = nullptr;
  avionics::SimulatorConnection* xplane = nullptr;
  bool usingXPlane = false;
  // While true, the main loop keeps the X-Plane link pumped and switches to it
  // as soon as it connects. Cleared once the user takes manual control (M key /
  // menu) or when a --source was given explicitly.
  bool autoDetectXPlane = false;
};

// Point both displays at the same feed so the MFD and PFD never diverge.
void SwitchSource(AppState& app, bool useXPlane) {
  if (useXPlane == app.usingXPlane) return;
  app.usingXPlane = useXPlane;
  avionics::DataSource* source =
      useXPlane ? static_cast<avionics::DataSource*>(app.xplane)
                : static_cast<avionics::DataSource*>(app.mock);
  const std::string label = useXPlane ? app.xplane->simulatorName() : kLabelMock;
  if (app.pfdEngine != nullptr) app.pfdEngine->setDataSource(*source, label);
  if (app.mfdEngine != nullptr) app.mfdEngine->setDataSource(*source, label);
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

  GLFWwindow* window = glfwCreateWindow(kSuiteWidth, kWindowHeight, kWindowTitle,
                                        nullptr, nullptr);
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

  // Use real X-Plane nav features when an install is present (falls back to the
  // built-in demo features otherwise); give the background loader a moment so
  // the captured frame shows the actual nearby navaids/fixes.
  avionics::NavDataStore navData;
  dataSource.setNavFeatureSource(&navData);
  for (int i = 0; i < 400 && !navData.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  avionics::AvionicsEngine engine(dataSource, renderer, kLabelMock);

  int fbWidth = 0;
  int fbHeight = 0;
  glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

  // The avionics screen is the left region; the bezel strip is on the right.
  // Softkey hit-testing happens in screen-pixel coordinates.
  const int screenW = fbWidth - BezelStripPx(fbWidth);

  glViewport(0, 0, fbWidth, fbHeight);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // --state selects which screen to capture (default: the live PFD):
  //   pfd    - skip the boot animation and show the live page
  //   boot   - the power-on initialization screen
  //   failed - the connection-lost display (link down: instruments red-X'd,
  //            chrome readouts dashed)
  if (state != nullptr && std::strcmp(state, "boot") == 0) {
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(renderer, kLabelMock, 0.6f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "alerts") == 0) {
    // Drive the real interaction path: bring up the live page, "click" the
    // Alerts softkey, then run the open animation to completion before capture.
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(screenW, fbHeight, 1.0f);  // establishes the click space
    const double barH = fbHeight * (35.0 / 768.0);  // softkey row height
    const double cellW = screenW / 12.0;
    engine.onPointerDown(11.5 * cellW, fbHeight - barH * 0.5);  // Alerts cell
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuite(renderer, engine, fbWidth, fbHeight);
  } else if (state != nullptr && std::strcmp(state, "menu") == 0) {
    // Exercise the softkey menu state machine: open the PFD Options submenu and
    // turn on a couple of display-option toggles so the captured frame shows the
    // submenu (with its Back key) and the highlighted active cells.
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(screenW, fbHeight, 1.0f);  // establishes the click space
    const double barH = fbHeight * (35.0 / 768.0);
    const double cellW = screenW / 12.0;
    const double rowY = fbHeight - barH * 0.5;
    engine.onPointerDown(3.5 * cellW, rowY);  // "PFD Opt" -> open submenu
    engine.onPointerDown(1.5 * cellW, rowY);  // "SVT" toggle on
    engine.onPointerDown(2.5 * cellW, rowY);  // "Wind" toggle on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuite(renderer, engine, fbWidth, fbHeight);
  } else if (state != nullptr && std::strcmp(state, "map") == 0) {
    // Turn on the PFD inset map: open the Map/HSI submenu, then toggle "Inset".
    engine.skipBoot();
    engine.update(seconds);
    engine.renderFrame(screenW, fbHeight, 1.0f);  // establishes the click space
    const double barH = fbHeight * (35.0 / 768.0);
    const double cellW = screenW / 12.0;
    const double rowY = fbHeight - barH * 0.5;
    engine.onPointerDown(1.5 * cellW, rowY);  // "Map/HSI" -> open submenu
    engine.onPointerDown(2.5 * cellW, rowY);  // "Inset" toggle on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuite(renderer, engine, fbWidth, fbHeight);
  } else if (state != nullptr && std::strcmp(state, "mfd") == 0) {
    // The MFD full-screen MAP page (its own window in normal operation).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    RenderSuite(renderer, engine, fbWidth, fbHeight);
  } else if (state != nullptr && std::strcmp(state, "failed") == 0) {
    // Capture the connection-lost display: populate believable live values from
    // the mock feed, then serve them through a source that reports the link as
    // down so the engine draws every instrument failed and the chrome dashed.
    engine.skipBoot();
    engine.update(seconds);
    StaleSource stale(dataSource.snapshot());
    avionics::AvionicsEngine failedEngine(stale, renderer, kSourceXPlane);
    failedEngine.skipBoot();
    failedEngine.update(1.0 / 60.0);
    RenderSuite(renderer, failedEngine, fbWidth, fbHeight);
  } else {
    engine.skipBoot();
    engine.update(seconds);
    RenderSuite(renderer, engine, fbWidth, fbHeight);
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

// Forward left-clicks to the engine that owns the clicked window. The engine
// works in framebuffer pixels (what renderFrame is given), while GLFW reports
// the cursor in window points, so scale by the framebuffer/window ratio to stay
// correct on HiDPI displays.
void OnMouseButton(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app == nullptr) return;

  avionics::AvionicsEngine* engine =
      (window == app->mfdWindow) ? app->mfdEngine : app->pfdEngine;
  if (engine == nullptr) return;

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);

  int winW = 0, winH = 0, fbW = 0, fbH = 0;
  glfwGetWindowSize(window, &winW, &winH);
  glfwGetFramebufferSize(window, &fbW, &fbH);
  const double sx = winW > 0 ? static_cast<double>(fbW) / winW : 1.0;
  const double sy = winH > 0 ? static_cast<double>(fbH) / winH : 1.0;
  const double fx = cursorX * sx;
  const double fy = cursorY * sy;

  // Clicks on the right-hand bezel strip drive the hardware keys; clicks on the
  // screen go to the softkey bar / pages (in screen-pixel coordinates).
  const int bezelPx = BezelStripPx(fbW);
  const int screenW = fbW - bezelPx;
  if (fx >= screenW) {
    const avionics::BezelKey key = avionics::BezelKeyPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(screenW), 0.0f, static_cast<float>(bezelPx),
        static_cast<float>(fbH));
    if (key != avionics::BezelKey::Count) engine->pressBezelKey(key);
  } else {
    engine->onPointerDown(fx, fy);
  }
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

bool HasFlag(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], flag) == 0) return true;
  }
  return false;
}

// Creates a GLFW window with the GL 3.2 core profile NanoVG needs. Returns null
// on failure. `share` lets a second window share the first's GL object space
// (unused here -- each renderer owns its own resources -- but kept for clarity).
GLFWwindow* CreateAvionicsWindow(const char* title, bool alwaysOnTop,
                                 GLFWwindow* share) {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_FLOATING, alwaysOnTop ? GLFW_TRUE : GLFW_FALSE);
  return glfwCreateWindow(kSuiteWidth, kWindowHeight, title, nullptr, share);
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

  const bool wantMfd = !HasFlag(argc, argv, "--no-mfd");

  // The PFD window owns vsync (paces the whole loop). Its context is created
  // first; the renderer is constructed while that context is current.
  GLFWwindow* pfdWindow = CreateAvionicsWindow(kWindowTitle, alwaysOnTop,
                                               nullptr);
  if (!pfdWindow) {
    std::fprintf(stderr, "Failed to create window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(pfdWindow);
  glfwSwapInterval(1);  // vsync on the PFD paces the loop to the display refresh
  glfwSetKeyCallback(pfdWindow, OnKey);
  glfwSetMouseButtonCallback(pfdWindow, OnMouseButton);

  avionics::NanoVgRenderer pfdRenderer;
  if (!pfdRenderer.valid()) {
    std::fprintf(stderr, "Failed to create NanoVG renderer\n");
    glfwDestroyWindow(pfdWindow);
    glfwTerminate();
    return 1;
  }

  // The MFD is a second window with its own GL context and renderer. Its swap
  // interval is 0 so its buffer swap does not block on a second vsync (which
  // would otherwise halve the frame rate); the PFD's vsync still paces things.
  GLFWwindow* mfdWindow = nullptr;
  avionics::NanoVgRenderer* mfdRenderer = nullptr;
  if (wantMfd) {
    mfdWindow = CreateAvionicsWindow(kMfdWindowTitle, alwaysOnTop, nullptr);
    if (mfdWindow) {
      glfwMakeContextCurrent(mfdWindow);
      glfwSwapInterval(0);
      glfwSetKeyCallback(mfdWindow, OnKey);
      glfwSetMouseButtonCallback(mfdWindow, OnMouseButton);
      mfdRenderer = new avionics::NanoVgRenderer();
      if (!mfdRenderer->valid()) {
        std::fprintf(stderr, "Failed to create MFD renderer; running PFD only\n");
        delete mfdRenderer;
        mfdRenderer = nullptr;
        glfwDestroyWindow(mfdWindow);
        mfdWindow = nullptr;
      }
    }
  }

  // Side-by-side placement: put the MFD just to the right of the PFD.
  if (mfdWindow != nullptr) {
    int px = 0, py = 0;
    glfwGetWindowPos(pfdWindow, &px, &py);
    glfwSetWindowPos(mfdWindow, px + kSuiteWidth + kWindowGap, py);
  }

  // Both feeds exist for the whole session; the engines are pointed at one at a
  // time and M swaps between them. Opening the X-Plane UDP socket up front is
  // cheap and lets the connection start acquiring data immediately.
  const char* host = FlagValue(argc, argv, "--xplane-host");
  const char* portStr = FlagValue(argc, argv, "--xplane-port");
  const std::uint16_t port = portStr
                                 ? static_cast<std::uint16_t>(std::atoi(portStr))
                                 : kDefaultXPlanePort;

  const char* fmsPlanArg = FlagValue(argc, argv, "--fms-plan");

  // Nav database + flight plan are loaded once and shared by both feeds: the
  // live X-Plane connection and the mock (so the mock can fly a real route over
  // real navaids when an X-Plane install is present).
  avionics::NavDataStore navData;
  avionics::FmsPlanStore fmsPlan(fmsPlanArg ? fmsPlanArg : "");

  avionics::MockDataSource mock;
  mock.setNavFeatureSource(&navData);
  bool mockRouteSet = false;  // set once the .fms flight plan has loaded

  avionics::XPlaneConnection xplane(host ? host : kDefaultXPlaneHost, port,
                                    navData, fmsPlan);

  const char* sourceArg = FlagValue(argc, argv, "--source");
  const bool startWithXPlane =
      sourceArg != nullptr && std::strcmp(sourceArg, kSourceXPlane) == 0;

  avionics::DataSource& initialSource =
      startWithXPlane ? static_cast<avionics::DataSource&>(xplane)
                      : static_cast<avionics::DataSource&>(mock);
  const std::string initialLabel =
      startWithXPlane ? xplane.simulatorName() : kLabelMock;

  // Both displays read the same source. The PFD engine pumps it each frame; the
  // MFD engine renders the MFD page from the same data without pumping again.
  avionics::AvionicsEngine pfdEngine(initialSource, pfdRenderer, initialLabel);
  pfdEngine.setPage(avionics::DisplayPage::PrimaryFlightDisplay);

  avionics::AvionicsEngine* mfdEngine = nullptr;
  if (mfdRenderer != nullptr) {
    mfdEngine =
        new avionics::AvionicsEngine(initialSource, *mfdRenderer, initialLabel);
    mfdEngine->setPage(avionics::DisplayPage::MultiFunctionDisplay);
    mfdEngine->setDrivesDataSource(false);
  }

  AppState app;
  app.mock = &mock;
  app.xplane = &xplane;
  app.usingXPlane = startWithXPlane;
  // With no explicit --source, start on mock and auto-switch to X-Plane once it
  // is detected. An explicit --source pins the feed instead.
  app.autoDetectXPlane = sourceArg == nullptr;
  app.pfdEngine = &pfdEngine;
  app.mfdEngine = mfdEngine;
  app.pfdWindow = pfdWindow;
  app.mfdWindow = mfdWindow;
  glfwSetWindowUserPointer(pfdWindow, &app);
  if (mfdWindow != nullptr) glfwSetWindowUserPointer(mfdWindow, &app);

#if defined(__APPLE__)
  // Add the "Data Source" menu (Mock Data / X-Plane) to the macOS menu bar.
  avionics::InstallDataSourceMenu(startWithXPlane, &OnMenuSelectSource, &app);
#endif

  // Renders one engine into its window: the avionics screen on the left and the
  // hardware bezel strip on the right. The gauge code draws directly in
  // framebuffer pixels, so the NanoVG device-pixel-ratio is 1.0.
  const auto renderWindow = [](GLFWwindow* win, avionics::AvionicsEngine& eng,
                               avionics::NanoVgRenderer& renderer, double dt) {
    glfwMakeContextCurrent(win);
    int fbWidth = 0, fbHeight = 0;
    glfwGetFramebufferSize(win, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    eng.update(dt);
    RenderSuite(renderer, eng, fbWidth, fbHeight);
    glfwSwapBuffers(win);
  };

  using clock = std::chrono::steady_clock;
  auto previous = clock::now();

  // Closing either window exits: the PFD and MFD are one avionics suite.
  while (!glfwWindowShouldClose(pfdWindow) &&
         (mfdWindow == nullptr || !glfwWindowShouldClose(mfdWindow))) {
    const auto now = clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;

    // Once the .fms flight plan has loaded, fly it on the mock feed too (until
    // then the mock flies its built-in demo route).
    if (!mockRouteSet && fmsPlan.loaded() && fmsPlan.flightPlan().size() >= 2) {
      mock.setRoute(fmsPlan.flightPlan());
      mockRouteSet = true;
    }

    // Auto-detect: while showing mock, keep the X-Plane link pumped (the engines
    // only update the active source) and switch over the instant it connects.
    if (app.autoDetectXPlane && !app.usingXPlane) {
      xplane.update(dt);
      if (xplane.connectionState() == avionics::ConnectionState::Connected) {
        SwitchSource(app, true);
      }
    }

    renderWindow(pfdWindow, pfdEngine, pfdRenderer, dt);
    if (mfdWindow != nullptr && mfdEngine != nullptr) {
      renderWindow(mfdWindow, *mfdEngine, *mfdRenderer, dt);
    }

    glfwPollEvents();
  }

  // Tear down GL objects while their contexts are still current.
  if (mfdWindow != nullptr) {
    glfwMakeContextCurrent(mfdWindow);
    delete mfdEngine;
    delete mfdRenderer;
    glfwDestroyWindow(mfdWindow);
  }
  glfwDestroyWindow(pfdWindow);
  glfwTerminate();
  return 0;
}
