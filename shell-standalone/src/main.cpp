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
//   --fms-bridge-port PORT    UDP port of the in-sim flight-plan bridge
//                             (shell-xplane plugin) that serves the live FMS
//                             route over the network (default: 49100)
//   --no-fms-write            don't program FPL/SimBrief/Direct-To edits back
//                             into X-Plane's FMS (display-only; the live route
//                             is still read from the bridge)
//   --fms-plan NAME           .fms flight plan for the inset map route, used
//                             when the live FMS bridge is unavailable
//                             (name under Output/FMS plans/, or a full path;
//                             default: most recently modified .fms in that dir)
//   --checklist PATH          checklist file for the MFD Checklist page group
//                             (default: the bundled sample)
//   --eis PATH                engine display layout file for the MFD EIS strip
//                             (default: bundled C172S sample, or g1000_eis.txt
//                             beside the loaded aircraft in the plugin)
//   --simbrief-id ID          SimBrief Pilot ID for the AUX - SIMBRIEF page's
//                             OFP fetch (default: the persisted setting,
//                             entered on the page itself)
//   --obstacles PATH          FAA Digital Obstacle File in CSV format
//                             (the "DDOF CSV" download) for the map's
//                             obstacle overlay; US-only, off when omitted

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

#include "AppSettings.h"
#include "ChecklistStore.h"
#include "avionics/EisStore.h"
#include "DsfTerrainStore.h"
#include "FmsPlanStore.h"
#include "NavData.h"
#include "ObstacleStore.h"
#include "ProcedureStore.h"
#include "ShellNavMapData.h"
#include "SimBriefStore.h"
#include "XPlaneConnection.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/ConnectionState.h"
#include "avionics/MockDataSource.h"
#include "avionics/SimBrief.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/BootScreen.h"
#include "avionics/render/NanoVgRenderer.h"
#include "avionics/render/SoftkeyBezel.h"

#if defined(__APPLE__)
#include "MacMenu.h"
#endif

namespace {

constexpr int kWindowWidth = 1024;   // avionics "screen" region (4:3)
constexpr int kWindowHeight = 768;
// The hardware bezel strips are part of the window, the way a GDU's physical
// keys frame the display: the key column on the right and the row of twelve
// softkey selection keys below the screen. The window is screen + strips.
constexpr int kBezelStripWidth = 112;
constexpr int kSoftkeyStripHeight = 72;
constexpr int kSuiteWidth = kWindowWidth + kBezelStripWidth;
constexpr int kSuiteHeight = kWindowHeight + kSoftkeyStripHeight;
constexpr const char* kWindowTitle = "XPlane Avionics - PFD";
constexpr const char* kMfdWindowTitle = "XPlane Avionics - MFD";

// Horizontal gap between the PFD and MFD windows when both open.
constexpr int kWindowGap = 24;

// Size of the bezel strips in framebuffer pixels for a given window framebuffer
// size, scaled so the screen:strip ratio is fixed across HiDPI back buffers.
inline int BezelStripPx(int fbWidth) {
  return static_cast<int>(std::lround(static_cast<double>(fbWidth) *
                                      kBezelStripWidth / kSuiteWidth));
}
inline int SoftkeyStripPx(int fbHeight) {
  return static_cast<int>(std::lround(static_cast<double>(fbHeight) *
                                      kSoftkeyStripHeight / kSuiteHeight));
}

// Renders one engine into the top-left "screen" region of its window and draws
// the hardware bezel strips: the key column on the right (full height) and the
// physical softkey row below the screen, aligned with the on-screen softkey
// labels. The gauge code draws in screen pixels (device-pixel-ratio 1.0); the
// bezel is drawn in full-window pixels. update() is the caller's
// responsibility.
inline void RenderSuite(avionics::NanoVgRenderer& renderer,
                        avionics::AvionicsEngine& eng, int fbWidth,
                        int fbHeight, bool showBezel = true) {
  // With the bezel hidden the screen fills the whole window (no physical key
  // strips to frame it), so the engine draws into the full framebuffer.
  if (!showBezel) {
    glViewport(0, 0, fbWidth, fbHeight);
    eng.renderFrame(fbWidth, fbHeight, 1.0f);
    return;
  }

  const int bezelPx = BezelStripPx(fbWidth);
  const int softkeyPx = SoftkeyStripPx(fbHeight);
  const int screenW = std::max(1, fbWidth - bezelPx);
  const int screenH = std::max(1, fbHeight - softkeyPx);

  // GL viewports are bottom-left anchored, so the screen's viewport is lifted
  // by the softkey strip's height to sit at the top of the window.
  glViewport(0, fbHeight - screenH, screenW, screenH);
  eng.renderFrame(screenW, screenH, 1.0f);

  glViewport(0, 0, fbWidth, fbHeight);
  renderer.beginFrame(fbWidth, fbHeight, 1.0f);
  avionics::BezelKeyPanel::render(
      renderer, static_cast<float>(screenW), 0.0f,
      static_cast<float>(fbWidth - screenW), static_cast<float>(fbHeight),
      static_cast<float>(screenH), eng.bezelPressLevels());
  avionics::SoftkeyBezelPanel::render(
      renderer, 0.0f, static_cast<float>(screenH), static_cast<float>(screenW),
      static_cast<float>(fbHeight - screenH), eng.softkeyPressLevels());
  renderer.endFrame();
}

constexpr const char* kSourceMock = "mock";
constexpr const char* kSourceXPlane = "xplane";
constexpr const char* kLabelMock = "MOCK DATA";
constexpr const char* kDefaultXPlaneHost = "127.0.0.1";
constexpr std::uint16_t kDefaultXPlanePort = 49000;

// Bundled Natural Earth land-data asset path, baked in by the build (empty when
// the build provides none; LandDataStore then loads nothing).
#ifndef AVIONICS_LAND_DATA
#define AVIONICS_LAND_DATA ""
#endif
constexpr const char* kLandDataAssetPath = AVIONICS_LAND_DATA;

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
  // Whether the hardware bezel strips are drawn (and the windows sized to
  // include them). Mirrors settings.showBezel; toggled from the View menu.
  bool showBezel = true;
  // Persisted user preferences, written back whenever the user changes the
  // feed or flips one of the View menu toggles.
  avionics::AppSettings settings;
  // In-progress CLR press-and-hold (CLR DFLT MAP): the engine whose CLR bezel
  // key the mouse went down on, and when. Cleared on release or once the hold
  // function fires.
  avionics::AvionicsEngine* clrHoldEngine = nullptr;
  double clrHoldStart = 0.0;
  // Hover cursors for the on-screen bezel: a left-right cursor over the
  // rotatable rings (FMS knob / RANGE joystick zoom), a hand over the other
  // clickable controls. Created at startup, freed at shutdown.
  GLFWcursor* rotateCursor = nullptr;
  GLFWcursor* handCursor = nullptr;
};

// Window dimensions for the two bezel states: the full suite (screen + strips)
// when the bezel is shown, and the bare 4:3 screen when it is hidden.
inline int SuiteWindowWidth(bool showBezel) {
  return showBezel ? kSuiteWidth : kWindowWidth;
}
inline int SuiteWindowHeight(bool showBezel) {
  return showBezel ? kSuiteHeight : kWindowHeight;
}

// Records the current window placement into the settings, for the "Remember
// Window Position" option. Positions are screen coordinates of the content
// area's top-left corner, as reported by GLFW.
void CaptureWindowPositions(AppState& app) {
  if (app.pfdWindow == nullptr) return;
  glfwGetWindowPos(app.pfdWindow, &app.settings.pfdWindowX,
                   &app.settings.pfdWindowY);
  if (app.mfdWindow != nullptr) {
    glfwGetWindowPos(app.mfdWindow, &app.settings.mfdWindowX,
                     &app.settings.mfdWindowY);
  } else {
    // PFD-only run: save the docked position so a later dual-window launch
    // still puts the MFD beside the PFD.
    app.settings.mfdWindowX = app.settings.pfdWindowX +
                              SuiteWindowWidth(app.showBezel) + kWindowGap;
    app.settings.mfdWindowY = app.settings.pfdWindowY;
  }
  app.settings.hasWindowPos = true;
}

// Resizes both windows to match the current bezel state and keeps the MFD
// docked just to the right of the PFD.
void ApplyBezelWindowSize(AppState& app) {
  const int w = SuiteWindowWidth(app.showBezel);
  const int h = SuiteWindowHeight(app.showBezel);
  if (app.pfdWindow != nullptr) glfwSetWindowSize(app.pfdWindow, w, h);
  if (app.mfdWindow != nullptr) {
    glfwSetWindowSize(app.mfdWindow, w, h);
    if (app.pfdWindow != nullptr) {
      int px = 0, py = 0;
      glfwGetWindowPos(app.pfdWindow, &px, &py);
      glfwSetWindowPos(app.mfdWindow, px + w + kWindowGap, py);
    }
  }
}

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

// ENT during power-up acknowledges the database information and brings up the
// live pages on both displays at once. A real unit acknowledges per display,
// but the standalone PFD and MFD windows boot as one suite, so a single ENT
// dismisses both. No-op once the pages are live (mock advances on its own).
void AcknowledgeBoot(AppState& app) {
  if (app.pfdEngine != nullptr) app.pfdEngine->acknowledgePowerUp();
  if (app.mfdEngine != nullptr) app.mfdEngine->acknowledgePowerUp();
}

bool AwaitingBootAck(const AppState& app) {
  return (app.pfdEngine != nullptr && app.pfdEngine->awaitingPowerUpAck()) ||
         (app.mfdEngine != nullptr && app.mfdEngine->awaitingPowerUpAck());
}

// Persists the current data-source choice so the next launch starts on it.
void PersistDataSource(AppState& app) {
  app.settings.useXPlane = app.usingXPlane;
  avionics::SaveAppSettings(app.settings);
}

// Menu-bar action target: switches the feed when the user picks from the menu.
void OnMenuSelectSource(void* context, bool useXPlane) {
  auto* app = static_cast<AppState*>(context);
  app->autoDetectXPlane = false;  // explicit user choice wins from here on
  SwitchSource(*app, useXPlane);
  PersistDataSource(*app);
}

// Menu-bar action target: turns simulated turbulence on the mock feed on/off
// and remembers the choice across runs (no effect on the live X-Plane feed).
void OnMenuToggleTurbulence(void* context, bool enabled) {
  auto* app = static_cast<AppState*>(context);
  if (app->mock != nullptr) app->mock->setTurbulenceEnabled(enabled);
  app->settings.simulateTurbulence = enabled;
  avionics::SaveAppSettings(app->settings);
}

// Menu-bar action target: shows/hides the hardware bezel strips and remembers
// the choice across runs.
void OnMenuToggleBezel(void* context, bool showBezel) {
  auto* app = static_cast<AppState*>(context);
  app->showBezel = showBezel;
  app->settings.showBezel = showBezel;
  avionics::SaveAppSettings(app->settings);
  ApplyBezelWindowSize(*app);
}

// Menu-bar action target: shows/hides the OS window chrome (the title bar with
// its close / minimize / maximize controls) on both windows.
void OnMenuToggleWindowChrome(void* context, bool showChrome) {
  auto* app = static_cast<AppState*>(context);
  const int decorated = showChrome ? GLFW_TRUE : GLFW_FALSE;
  if (app->pfdWindow != nullptr) {
    glfwSetWindowAttrib(app->pfdWindow, GLFW_DECORATED, decorated);
  }
  if (app->mfdWindow != nullptr) {
    glfwSetWindowAttrib(app->mfdWindow, GLFW_DECORATED, decorated);
  }
  app->settings.showWindowChrome = showChrome;
  avionics::SaveAppSettings(app->settings);
}

// Menu-bar action target: floats/unfloats both windows above other windows and
// remembers the choice across runs.
void OnMenuToggleAlwaysOnTop(void* context, bool alwaysOnTop) {
  auto* app = static_cast<AppState*>(context);
  const int floating = alwaysOnTop ? GLFW_TRUE : GLFW_FALSE;
  if (app->pfdWindow != nullptr) {
    glfwSetWindowAttrib(app->pfdWindow, GLFW_FLOATING, floating);
  }
  if (app->mfdWindow != nullptr) {
    glfwSetWindowAttrib(app->mfdWindow, GLFW_FLOATING, floating);
  }
  app->settings.alwaysOnTop = alwaysOnTop;
  avionics::SaveAppSettings(app->settings);
}

// Menu-bar action target: enables/disables restoring the window positions on
// the next launch. Enabling captures the current placement immediately so the
// preference takes effect even if the app later exits abnormally.
void OnMenuToggleRememberWindowPos(void* context, bool remember) {
  auto* app = static_cast<AppState*>(context);
  app->settings.rememberWindowPos = remember;
  if (remember) CaptureWindowPositions(*app);
  avionics::SaveAppSettings(app->settings);
}

// Screenshot-mode render: repeats the suite render for a short settle period
// so progressively-built content converges before capture -- the DSF terrain
// tiles load on a worker thread and the terrain raster rebuilds across frames,
// so the first frame would otherwise show the procedural fallback. The engine
// is NOT updated between repeats, keeping the captured values deterministic.
inline void RenderSuiteSettled(avionics::NanoVgRenderer& renderer,
                               avionics::AvionicsEngine& eng, int fbWidth,
                               int fbHeight, bool showBezel) {
  constexpr int kSettleFrames = 60;
  for (int i = 0; i < kSettleFrames; ++i) {
    RenderSuite(renderer, eng, fbWidth, fbHeight, showBezel);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  RenderSuite(renderer, eng, fbWidth, fbHeight, showBezel);
}

// Renders a deterministic frame offscreen and writes it to a binary PPM
// (P6). PPM keeps this dependency-free; convert to PNG with `sips` afterwards.
// Returns 0 on success. The mock state is advanced by `seconds` so we can pick a
// clean, representative attitude (seconds = 0 is wings-level, no turbulence).
// When `fmsPlan` is non-null the mock flies that .fms route (same selector
// rules as the interactive --fms-plan flag) instead of its built-in demo route.
int RunScreenshot(const char* path, double seconds, const char* state,
                  const char* fmsPlan, bool showBezel) {
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Match the framebuffer to the bezel state: the full suite (screen + key
  // strips) when the bezel is shown, the bare 4:3 screen when it is hidden.
  GLFWwindow* window = glfwCreateWindow(SuiteWindowWidth(showBezel),
                                        SuiteWindowHeight(showBezel),
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

  // The mock simulates only the aircraft motion: all moving-map navigation data
  // comes from the real X-Plane databases (features, airspaces, airways,
  // runways, land vectors, obstacles), so a screenshot reflects the actual
  // installed data, never fabricated data. Give the background loaders a moment
  // so the captured frame shows the actual nearby data.
  avionics::NavDataStore navData;
  avionics::AirspaceStore airspace;
  avionics::AirwayStore airways;
  avionics::AptDatStore aptData;
  avionics::LandDataStore landData(kLandDataAssetPath);
  avionics::ObstacleStore obstacles("");
  avionics::ProcedureStore procedures(navData);
  avionics::ShellNavMapData navMapData(navData, airspace, airways, aptData,
                                       landData, procedures, &obstacles);
  avionics::DsfTerrainStore terrain;
  avionics::ChecklistStore checklists;
  avionics::EisStore eisStore;
  dataSource.setNavFeatureSource(&navMapData);
  dataSource.setTerrainSource(&terrain);
  dataSource.setChecklistSource(&checklists);
  dataSource.setEisSource(&eisStore);
  // Wait for the database loaders so the captured frame reflects the real data
  // (the airspace file in particular is large and parses on its own thread).
  for (int i = 0; i < 2000 && !(navData.ready() && airspace.loaded() &&
                                airways.loaded() && landData.loaded() &&
                                aptData.loaded());
       ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Optional .fms route: once it has loaded, fly it on the mock feed (the
  // aircraft repositions to the route's first waypoint), matching the
  // interactive shell's --fms-plan behavior.
  if (fmsPlan != nullptr) {
    avionics::FmsPlanStore plan(fmsPlan);
    for (int i = 0; i < 400 && !plan.loaded(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (plan.flightPlan().size() >= 2) dataSource.setRoute(plan.flightPlan());
  }

  avionics::AvionicsEngine engine(dataSource, renderer, kLabelMock);
  engine.mfdController().setNavFeatureSource(&navMapData);

  int fbWidth = 0;
  int fbHeight = 0;
  glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

  // The avionics screen is the top-left region; the bezel key column is on the
  // right and the physical softkey row is below the screen.

  glViewport(0, 0, fbWidth, fbHeight);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // --state selects which screen to capture (default: the live PFD):
  //   pfd    - skip the boot animation and show the live page
  //   boot   - the power-on initialization screen
  //   failed - the connection-lost display (link down: instruments red-X'd,
  //            chrome readouts dashed)
  if (state != nullptr && std::strcmp(state, "bootlogo") == 0) {
    // The initial Garmin logo splash (phase 1 of power-up).
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(renderer, avionics::BootScreen::Phase::Logo,
                                 kLabelMock, dataSource.mapSnapshot().navDatabase,
                                 false, 1.0f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "bootfade") == 0) {
    // Mid cross-fade: Power-up Page at ~50% opacity (1s into the 2s fade).
    dataSource.update(0.0);
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(renderer, avionics::BootScreen::Phase::PowerUp,
                                 kLabelMock, dataSource.mapSnapshot().navDatabase,
                                 false, 0.5f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "boot") == 0) {
    // The MFD Power-up Page (phase 2). Pump the source once so the database
    // currency block is populated (from the parsed nav data when available, the
    // demo cycle otherwise), and show the ENT acknowledgement prompt.
    dataSource.update(0.0);
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(renderer, avionics::BootScreen::Phase::PowerUp,
                                 kLabelMock, dataSource.mapSnapshot().navDatabase,
                                 true, 1.0f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "alerts") == 0) {
    // Drive the real interaction path: bring up the live page, press the
    // Alerts softkey, then run the open animation to completion before capture.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(11);  // Alerts key
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "menu") == 0) {
    // Exercise the softkey menu state machine: open the PFD Options submenu and
    // turn on a couple of display-option toggles so the captured frame shows the
    // submenu (with its Back key) and the highlighted active cells.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(3);  // "PFD Opt" -> open submenu
    engine.pressSoftkey(1);  // "SVT" toggle on
    engine.pressSoftkey(2);  // "Wind" toggle on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "tmrref") == 0) {
    // Timer/References window: open it, start the timer, run it for a bit,
    // then set BARO minimums so the BARO MIN box and tape bug are captured.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(9);  // "Tmr/Ref" -> open the References window
    engine.pressBezelKey(avionics::BezelKey::Ent);  // Start? -> timer runs
    for (int i = 0; i < 90; ++i) engine.update(1.0 / 60.0);  // 1.5 s elapses
    // Cursor down to MINS (over the four V-speed rows) with the large FMS
    // knob, select BARO, then step the altitude up with the small knob
    // (100 ft per click).
    for (int i = 0; i < 5; ++i) {
      engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
    }
    engine.pressBezelKey(avionics::BezelKey::Ent);  // MINS Off -> BARO
    for (int i = 0; i < 54; ++i) {
      // 5400 ft: just below the mock's cruise altitude so the BARO MIN box
      // and the tape bug are both captured.
      engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);
    }
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "nrst") == 0) {
    // Nearest Airports window with the FMS cursor stepped to the second entry.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(10);  // "Nearest"
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
    for (int i = 0; i < 10; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "ident") == 0) {
    // IDNT annunciation plus an in-progress squawk entry: press Ident at the
    // root (starts the 18 s annunciation), then type two digits of a new code
    // on the XPDR > Code softkeys.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(8);  // "Ident" -> IDNT annunciation
    engine.pressSoftkey(7);  // "XPDR"
    engine.pressSoftkey(5);  // "Code"
    engine.pressSoftkey(4);  // digit 4
    engine.pressSoftkey(5);  // digit 5
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "map") == 0) {
    // The PFD with its inset map (on by default) and the Map/HSI submenu open,
    // showing the Detail / Traffic / Topo / Rel Ter map option keys.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(1);  // "Map/HSI" -> open submenu
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfd") == 0) {
    // The MFD full-screen MAP page (its own window in normal operation).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdsimbrief") == 0) {
    // The AUX - SIMBRIEF page with a Pilot ID entry in progress: step to the
    // page (fourth AUX page), open the ID digit-entry softkeys, and type two
    // digits so the cyan edit plate and the digit bar are captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 4; ++p) engine.pressSoftkey(2);  // AUX -> SimBrief
    engine.pressSoftkey(8);  // "ID" -> digit entry
    engine.pressSoftkey(8);  // digit 8
    engine.pressSoftkey(4);  // digit 4
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdlayers") == 0) {
    // The MAP page with every optional overlay enabled via the Map Opt
    // submenu (Traffic on, TER Topo, AWY On), left open so the submenu
    // labels/highlights are captured too.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(5);  // "Map Opt" -> open submenu
    engine.pressSoftkey(1);  // "Traffic" on
    engine.pressSoftkey(3);  // "AWY" Off -> On
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdwx") == 0) {
    // The MAP page with the NEXRAD precipitation overlay enabled, zoomed out
    // so the (forward-range) mock weather cells are on screen. Zoom first on
    // the root bar (RNG+ steps the range ladder), then open Map Opt and toggle
    // NEXRAD, leaving the submenu open so the highlighted key is captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 4; ++p) engine.pressSoftkey(11);  // RNG+ -> ~100 NM
    engine.pressSoftkey(5);  // "Map Opt" -> open submenu
    engine.pressSoftkey(2);  // TER Topo -> Rel
    engine.pressSoftkey(2);  // TER Rel -> Off (clean background)
    engine.pressSoftkey(4);  // "NEXRAD" on
    for (int i = 0; i < 120; ++i) engine.update(1.0 / 60.0);  // settle zoom
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdwxr") == 0) {
    // The dedicated MAP - Weather Radar page (airborne GWX radar): step to the
    // third MAP-group page, select Weather mode, trim the antenna tilt down,
    // show the bearing line, and zoom out so the mock storm cells fall in the
    // forward scan wedge.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(0);  // Map -> Traffic Map
    engine.pressSoftkey(0);  // Map -> Weather Radar
    engine.pressSoftkey(0);  // "Mode" -> submenu
    engine.pressSoftkey(2);  // "Weather"
    for (int i = 0; i < 4; ++i)
      engine.pressBezelKey(avionics::BezelKey::FmsInnerCcw);  // tilt DN 1.00
    engine.pressSoftkey(4);                                   // "BRG" line on
    for (int p = 0; p < 5; ++p) engine.pressSoftkey(11);      // RNG+ -> wide
    // Settle past the 3 s page-select popup so it fades clear of the readouts.
    for (int i = 0; i < 260; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdfplent") == 0) {
    // The FPL page mid-edit: cursor on (FMS knob push), Waypoint Information
    // window open with a partially spelled ident (small knob spells, large
    // knob moves the character cursor) so the entry overlay, the spell-ahead
    // fill, and the leg-list cursor are all captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Fpl);
    engine.pressBezelKey(avionics::BezelKey::FmsPush);     // cursor on
    // The large knob steps IDENT -> ALT -> next row's IDENT, so two clicks
    // reach the second waypoint's identifier.
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // first row ALT
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // second row
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // entry window
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // first char: K
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // next cell
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // spell on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfddto") == 0) {
    // The Direct-To window: the Direct-To key opens it pre-filled with the
    // active waypoint (resolved from the nav data), ready to ACTIVATE.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::DirectTo);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdfplrmv") == 0) {
    // The FPL page's "Remove <wpt>?" confirmation: cursor onto the second
    // waypoint, CLR opens the OK/CANCEL window (ENT would delete the leg).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Fpl);
    engine.pressBezelKey(avionics::BezelKey::FmsPush);     // cursor on
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // second row
    engine.pressBezelKey(avionics::BezelKey::Clr);         // Remove <wpt>?
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strncmp(state, "mfd", 3) == 0) {
    // MFD page screenshots. The state encodes a page-group softkey plus an
    // optional repeat count (pressing the active group's key again steps to
    // the group's next page): "mfdwpt" = WPT page 1, "mfdwpt3" = WPT page 3.
    // "mfdfpl" presses the FPL bezel key instead, and "mfdtrk" toggles
    // track-up on the MAP page. "mfdrng<N>" zooms the MAP page out N range
    // steps (RangeUp) to capture wide/continental views.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    const char* suffix = state + 3;
    int cell = -1;
    if (std::strncmp(suffix, "wpt", 3) == 0) cell = 1;
    if (std::strncmp(suffix, "aux", 3) == 0) cell = 2;
    if (std::strncmp(suffix, "nrst", 4) == 0) cell = 3;
    if (std::strncmp(suffix, "trk", 3) == 0) cell = 4;
    if (std::strncmp(suffix, "chklist", 7) == 0) cell = 7;
    if (std::strncmp(suffix, "rng", 3) == 0) {
      const char* digits = suffix + 3;
      const int presses = *digits != '\0' ? std::atoi(digits) : 1;
      for (int p = 0; p < presses; ++p) {
        engine.pressBezelKey(avionics::BezelKey::RangeUp);
        for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
      }
    } else if (std::strncmp(suffix, "fpl", 3) == 0) {
      engine.pressBezelKey(avionics::BezelKey::Fpl);
    } else if (cell >= 0) {
      const char* digits = suffix;
      while (*digits != '\0' && (*digits < '0' || *digits > '9')) ++digits;
      const int presses = *digits != '\0' ? std::atoi(digits) : 1;
      for (int p = 0; p < presses; ++p) engine.pressSoftkey(cell);
    }
    // On the Checklist page, check off the first few items so the captured frame
    // shows the cursor, the green checks, and the auto-advance flow.
    if (std::strncmp(suffix, "chklist", 7) == 0) {
      for (int i = 0; i < 400 && !checklists.ready(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      for (int p = 0; p < 3; ++p) engine.pressBezelKey(avionics::BezelKey::Ent);
    }
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
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
    RenderSuiteSettled(renderer, failedEngine, fbWidth, fbHeight, showBezel);
  } else {
    engine.skipBoot();
    engine.update(seconds);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
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
  // ENT acknowledges the power-up page (live sim link); the keyboard Enter key
  // is a convenience alongside clicking the ENT bezel key.
  if (key == GLFW_KEY_ENTER || key == GLFW_KEY_KP_ENTER) {
    auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (app != nullptr) AcknowledgeBoot(*app);
    return;
  }
  // M toggles between the mock feed and the live X-Plane connection.
  if (key == GLFW_KEY_M) {
    auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (app != nullptr) {
      app->autoDetectXPlane = false;  // explicit user choice wins from here on
      SwitchSource(*app, !app->usingXPlane);
      PersistDataSource(*app);
    }
  }
}

// Forward left-clicks to the engine that owns the clicked window. The engine
// works in framebuffer pixels (what renderFrame is given), while GLFW reports
// the cursor in window points, so scale by the framebuffer/window ratio to stay
// correct on HiDPI displays.
void OnMouseButton(GLFWwindow* window, int button, int action, int /*mods*/) {
  if (button != GLFW_MOUSE_BUTTON_LEFT) return;
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app == nullptr) return;
  if (action == GLFW_RELEASE) {
    app->clrHoldEngine = nullptr;  // released before the hold function fired
    return;
  }
  if (action != GLFW_PRESS) return;
  // With the bezel hidden there are no physical keys to click; the screen
  // itself is just glass, so swallow the click.
  if (!app->showBezel) return;

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

  // Only the physical bezel controls are clickable, like the real unit: the
  // key column on the right and the softkey row below the screen. Clicks on
  // the screen itself do nothing (it is just glass).
  const int bezelPx = BezelStripPx(fbW);
  const int screenW = fbW - bezelPx;
  const int screenH = fbH - SoftkeyStripPx(fbH);
  if (fx >= screenW) {
    const avionics::BezelKey key = avionics::BezelKeyPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(screenW), 0.0f, static_cast<float>(bezelPx),
        static_cast<float>(fbH));
    if (key == avionics::BezelKey::Ent && AwaitingBootAck(*app)) {
      // Acknowledge the power-up page on both displays together.
      AcknowledgeBoot(*app);
    } else if (key != avionics::BezelKey::Count) {
      engine->pressBezelKey(key);
      // Arm the CLR press-and-hold; the main loop fires CLR (DFLT MAP) if the
      // button is still down after kClrDefaultMapHoldSeconds.
      if (key == avionics::BezelKey::Clr) {
        app->clrHoldEngine = engine;
        app->clrHoldStart = glfwGetTime();
      }
    }
  } else if (fy >= screenH) {
    const int key = avionics::SoftkeyBezelPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy), 0.0f,
        static_cast<float>(screenH), static_cast<float>(screenW),
        static_cast<float>(fbH - screenH));
    if (key >= 0) engine->pressSoftkey(key);
  }
}

// Hover feedback over the on-screen bezel: a left-right (rotate) cursor over
// the FMS knob / RANGE joystick rings, a hand over the other clickable keys and
// softkeys, and the default arrow over the glass screen. Uses the same hit-test
// and pixel scaling as OnMouseButton so the cursor matches what a click does.
void OnCursorPos(GLFWwindow* window, double cursorX, double cursorY) {
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app == nullptr) return;
  if (!app->showBezel) {
    glfwSetCursor(window, nullptr);  // glass only; default arrow
    return;
  }

  int winW = 0, winH = 0, fbW = 0, fbH = 0;
  glfwGetWindowSize(window, &winW, &winH);
  glfwGetFramebufferSize(window, &fbW, &fbH);
  const double sx = winW > 0 ? static_cast<double>(fbW) / winW : 1.0;
  const double sy = winH > 0 ? static_cast<double>(fbH) / winH : 1.0;
  const double fx = cursorX * sx;
  const double fy = cursorY * sy;

  const int bezelPx = BezelStripPx(fbW);
  const int screenW = fbW - bezelPx;
  const int screenH = fbH - SoftkeyStripPx(fbH);

  GLFWcursor* cursor = nullptr;  // default arrow over the glass screen
  if (fx >= screenW) {
    const avionics::BezelKey key = avionics::BezelKeyPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(screenW), 0.0f, static_cast<float>(bezelPx),
        static_cast<float>(fbH));
    if (key != avionics::BezelKey::Count) {
      cursor = avionics::isRotatableBezelKey(key) ? app->rotateCursor
                                                  : app->handCursor;
    }
  } else if (fy >= screenH) {
    const int key = avionics::SoftkeyBezelPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy), 0.0f,
        static_cast<float>(screenH), static_cast<float>(screenW),
        static_cast<float>(fbH - screenH));
    if (key >= 0) cursor = app->handCursor;
  }
  glfwSetCursor(window, cursor);
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
// The window starts hidden so it can be positioned (e.g. a restored saved
// placement) before its first appearance; the caller shows it when ready.
GLFWwindow* CreateAvionicsWindow(const char* title, bool alwaysOnTop,
                                 bool decorated, GLFWwindow* share, int width,
                                 int height) {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_FLOATING, alwaysOnTop ? GLFW_TRUE : GLFW_FALSE);
  glfwWindowHint(GLFW_DECORATED, decorated ? GLFW_TRUE : GLFW_FALSE);
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  return glfwCreateWindow(width, height, title, nullptr, share);
}

}  // namespace

int main(int argc, char** argv) {
  const bool cliAlwaysOnTop = WantsAlwaysOnTop(argc, argv);

  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  // Offscreen single-frame capture mode for development/iteration:
  //   avionics-standalone --screenshot out.ppm [--time SECONDS]
  //       [--state STATE] [--fms-plan NAME] [--no-bezel]
  // --no-bezel captures just the 4:3 avionics screen, omitting the hardware
  // bezel key column and softkey strip.
  if (const char* shot = FlagValue(argc, argv, "--screenshot")) {
    const char* timeStr = FlagValue(argc, argv, "--time");
    const double seconds = timeStr ? std::atof(timeStr) : 0.0;
    const char* state = FlagValue(argc, argv, "--state");
    const char* fmsPlan = FlagValue(argc, argv, "--fms-plan");
    const bool showBezel = !HasFlag(argc, argv, "--no-bezel");
    const int rc = RunScreenshot(shot, seconds, state, fmsPlan, showBezel);
    glfwTerminate();
    return rc;
  }

  const bool wantMfd = !HasFlag(argc, argv, "--no-mfd");

  // Restore persisted preferences unless the command line overrides them.
  const avionics::AppSettings savedSettings = avionics::LoadAppSettings();
  const bool showBezel = savedSettings.showBezel;
  const bool showWindowChrome = savedSettings.showWindowChrome;
  // The --always-on-top flag / env var forces floating on; otherwise honor the
  // persisted preference.
  const bool alwaysOnTop = cliAlwaysOnTop || savedSettings.alwaysOnTop;
  const int winW = SuiteWindowWidth(showBezel);
  const int winH = SuiteWindowHeight(showBezel);

  const char* sourceArg = FlagValue(argc, argv, "--source");
  bool startWithXPlane = false;
  bool autoDetectXPlane = false;
  if (sourceArg != nullptr) {
    startWithXPlane = std::strcmp(sourceArg, kSourceXPlane) == 0;
  } else if (savedSettings.loaded) {
    startWithXPlane = savedSettings.useXPlane;
  } else {
    autoDetectXPlane = true;
  }

  // The PFD window owns vsync (paces the whole loop). Its context is created
  // first; the renderer is constructed while that context is current.
  GLFWwindow* pfdWindow = CreateAvionicsWindow(
      kWindowTitle, alwaysOnTop, showWindowChrome, nullptr, winW, winH);
  if (!pfdWindow) {
    std::fprintf(stderr, "Failed to create window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(pfdWindow);
  glfwSwapInterval(1);  // vsync on the PFD paces the loop to the display refresh
  glfwSetKeyCallback(pfdWindow, OnKey);
  glfwSetMouseButtonCallback(pfdWindow, OnMouseButton);
  glfwSetCursorPosCallback(pfdWindow, OnCursorPos);

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
    mfdWindow = CreateAvionicsWindow(kMfdWindowTitle, alwaysOnTop,
                                     showWindowChrome, nullptr, winW, winH);
    if (mfdWindow) {
      glfwMakeContextCurrent(mfdWindow);
      glfwSwapInterval(0);
      glfwSetKeyCallback(mfdWindow, OnKey);
      glfwSetMouseButtonCallback(mfdWindow, OnMouseButton);
      glfwSetCursorPosCallback(mfdWindow, OnCursorPos);
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

  // Window placement, applied while the windows are still hidden so they first
  // appear in their final spots: the saved positions when "Remember Window
  // Position" is on, otherwise the MFD docked just to the right of the PFD.
  if (savedSettings.rememberWindowPos && savedSettings.hasWindowPos) {
    glfwSetWindowPos(pfdWindow, savedSettings.pfdWindowX,
                     savedSettings.pfdWindowY);
    if (mfdWindow != nullptr) {
      glfwSetWindowPos(mfdWindow, savedSettings.mfdWindowX,
                       savedSettings.mfdWindowY);
    }
  } else if (mfdWindow != nullptr) {
    int px = 0, py = 0;
    glfwGetWindowPos(pfdWindow, &px, &py);
    glfwSetWindowPos(mfdWindow, px + winW + kWindowGap, py);
  }
  glfwShowWindow(pfdWindow);
  if (mfdWindow != nullptr) glfwShowWindow(mfdWindow);

  // Both feeds exist for the whole session; the engines are pointed at one at a
  // time and M swaps between them. Opening the X-Plane UDP socket up front is
  // cheap and lets the connection start acquiring data immediately.
  const char* host = FlagValue(argc, argv, "--xplane-host");
  const char* portStr = FlagValue(argc, argv, "--xplane-port");
  const std::uint16_t port = portStr
                                 ? static_cast<std::uint16_t>(std::atoi(portStr))
                                 : kDefaultXPlanePort;

  // UDP port of the in-sim flight-plan bridge (shell-xplane plugin), which feeds
  // the live FMS route the RREF/Web API transports can't carry.
  const char* bridgePortStr = FlagValue(argc, argv, "--fms-bridge-port");
  const std::uint16_t bridgePort =
      bridgePortStr ? static_cast<std::uint16_t>(std::atoi(bridgePortStr))
                    : avionics::fpbridge::kDefaultPort;

  // FPL/SimBrief/Direct-To edits are programmed back into X-Plane's FMS through
  // the bridge by default; --no-fms-write keeps them display-only.
  const bool fmsWriteEnabled = !HasFlag(argc, argv, "--no-fms-write");

  const char* fmsPlanArg = FlagValue(argc, argv, "--fms-plan");

  // X-Plane navigation databases + flight plan, loaded once and shared by both
  // feeds: the live X-Plane connection and the motion-only mock both read the
  // same real data (the mock never fabricates navigation data). The large
  // databases -- notably the Global Airports apt.dat -- are therefore parsed
  // only once.
  avionics::NavDataStore navData;
  avionics::AirspaceStore airspace;
  avionics::AirwayStore airways;
  avionics::AptDatStore aptData;
  avionics::LandDataStore landData(kLandDataAssetPath);
  avionics::FmsPlanStore fmsPlan(fmsPlanArg ? fmsPlanArg : "");
  avionics::DsfTerrainStore terrain;

  // Author-supplied checklists for the MFD Checklist page group. --checklist
  // selects a file; otherwise the build-time sample is used.
  const char* checklistArg = FlagValue(argc, argv, "--checklist");
  avionics::ChecklistStore checklists(checklistArg ? checklistArg : "");
  const char* eisArg = FlagValue(argc, argv, "--eis");
  avionics::EisStore eisStore(eisArg ? eisArg : "");

  // Optional FAA DOF obstacle database (CSV). Loads nothing when no file is
  // given, leaving the map's obstacle layer empty.
  const char* obstaclesArg = FlagValue(argc, argv, "--obstacles");
  avionics::ObstacleStore obstacles(obstaclesArg ? obstaclesArg : "");
  avionics::ProcedureStore procedures(navData);

  // Real X-Plane navigation data behind the core's NavFeatureSource interface,
  // shared by every feed so the map always shows X-Plane data, never mock data.
  avionics::ShellNavMapData navMapData(navData, airspace, airways, aptData,
                                       landData, procedures, &obstacles);

  avionics::MockDataSource mock;
  mock.setNavFeatureSource(&navMapData);
  mock.setTerrainSource(&terrain);
  mock.setChecklistSource(&checklists);
  mock.setEisSource(&eisStore);
  mock.setTurbulenceEnabled(savedSettings.simulateTurbulence);
  bool mockRouteSet = false;  // set once the .fms flight plan has loaded

  avionics::XPlaneConnection xplane(host ? host : kDefaultXPlaneHost, port,
                                    navData, airspace, airways, aptData,
                                    landData, fmsPlan, &terrain, &checklists,
                                    &eisStore, &obstacles, bridgePort,
                                    fmsWriteEnabled);

  // SimBrief OFP fetch (AUX - SIMBRIEF page). The Pilot ID comes from the
  // command line, falling back to the persisted setting; when one is known the
  // latest OFP is fetched once at startup, and the page's FETCH softkey
  // re-fetches on demand.
  avionics::SimBriefStore simbrief;
  const char* simbriefIdArg = FlagValue(argc, argv, "--simbrief-id");
  std::string simbriefPilotId =
      simbriefIdArg != nullptr ? simbriefIdArg : savedSettings.simbriefPilotId;
  avionics::SimBriefState simbriefState;
  if (simbriefPilotId.empty()) {
    simbriefState.status = avionics::SimBriefStatus::NotConfigured;
  } else {
    simbriefState.status = avionics::SimBriefStatus::Fetching;
    simbrief.requestFetch(simbriefPilotId);
  }

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
    mfdEngine->mfdController().setSimbriefPilotId(simbriefPilotId);
    // Ident lookups for FPL waypoint entry come from the parsed nav database.
    mfdEngine->mfdController().setNavFeatureSource(&navMapData);
  }

  // Restore the durable display preferences from the last run (e.g. the PFD
  // inset map on/off, map ranges, declutter) so the avionics come up the way
  // the pilot left them.
  avionics::applyPfdState(pfdEngine.softkeyController(),
                          savedSettings.avionics.pfd);
  if (mfdEngine != nullptr) {
    avionics::applyMfdState(mfdEngine->mfdController(),
                            savedSettings.avionics.mfd);
  }

  AppState app;
  app.mock = &mock;
  app.xplane = &xplane;
  app.usingXPlane = startWithXPlane;
  app.autoDetectXPlane = autoDetectXPlane;
  app.showBezel = showBezel;
  app.settings = savedSettings;
  app.settings.alwaysOnTop = alwaysOnTop;
  if (!app.settings.loaded) {
    app.settings.useXPlane = startWithXPlane;
    app.settings.showBezel = showBezel;
  }
  app.pfdEngine = &pfdEngine;
  app.mfdEngine = mfdEngine;
  app.pfdWindow = pfdWindow;
  app.mfdWindow = mfdWindow;
  // Hover cursors for the bezel: left-right over the rotatable rings, hand over
  // the other clickable controls (a null handle falls back to the arrow).
  app.rotateCursor = glfwCreateStandardCursor(GLFW_RESIZE_EW_CURSOR);
  app.handCursor = glfwCreateStandardCursor(GLFW_POINTING_HAND_CURSOR);
  glfwSetWindowUserPointer(pfdWindow, &app);
  if (mfdWindow != nullptr) glfwSetWindowUserPointer(mfdWindow, &app);

#if defined(__APPLE__)
  // macOS menu bar: data feed plus the View toggles (all persisted). The
  // Data Source menu also carries the mock-only "Simulate Turbulence" toggle.
  avionics::InstallDataSourceMenu(startWithXPlane,
                                  app.settings.simulateTurbulence,
                                  &OnMenuSelectSource, &OnMenuToggleTurbulence,
                                  &app);
  avionics::ViewMenuConfig viewMenu;
  viewMenu.showBezel = showBezel;
  viewMenu.showWindowChrome = showWindowChrome;
  viewMenu.alwaysOnTop = alwaysOnTop;
  viewMenu.rememberWindowPos = app.settings.rememberWindowPos;
  viewMenu.onToggleBezel = &OnMenuToggleBezel;
  viewMenu.onToggleWindowChrome = &OnMenuToggleWindowChrome;
  viewMenu.onToggleAlwaysOnTop = &OnMenuToggleAlwaysOnTop;
  viewMenu.onToggleRememberWindowPos = &OnMenuToggleRememberWindowPos;
  viewMenu.context = &app;
  avionics::InstallViewMenu(viewMenu);
#endif

  // Renders one engine into its window: the avionics screen on the left and the
  // hardware bezel strip on the right. The gauge code draws directly in
  // framebuffer pixels, so the NanoVG device-pixel-ratio is 1.0.
  const auto renderWindow = [&app](GLFWwindow* win, avionics::AvionicsEngine& eng,
                               avionics::NanoVgRenderer& renderer, double dt) {
    glfwMakeContextCurrent(win);
    int fbWidth = 0, fbHeight = 0;
    glfwGetFramebufferSize(win, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    eng.update(dt);
    RenderSuite(renderer, eng, fbWidth, fbHeight, app.showBezel);
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

    // Pick up live edits to the checklist file so authors can iterate without
    // restarting.
    checklists.refreshIfChanged();
    eisStore.refreshIfChanged();

    // SimBrief: react to the AUX - SIMBRIEF page (a newly committed Pilot ID
    // is persisted; FETCH kicks off a download), land completed fetches into
    // both feeds' flight plans, and publish the status back for rendering.
    if (mfdEngine != nullptr) {
      avionics::MfdController& mfdUi = mfdEngine->mfdController();
      if (mfdUi.simbriefPilotId() != simbriefPilotId) {
        simbriefPilotId = mfdUi.simbriefPilotId();
        app.settings.simbriefPilotId = simbriefPilotId;
        avionics::SaveAppSettings(app.settings);
        if (simbriefState.status == avionics::SimBriefStatus::NotConfigured) {
          simbriefState.status = avionics::SimBriefStatus::Idle;
        }
      }
      if (mfdUi.consumeSimbriefFetchRequest() && !simbriefPilotId.empty() &&
          !simbrief.fetching()) {
        simbriefState.status = avionics::SimBriefStatus::Fetching;
        simbrief.requestFetch(simbriefPilotId);
      }
    }
    avionics::SimBriefFetchResult simbriefResult;
    if (simbrief.consumeResult(simbriefResult)) {
      if (simbriefResult.ok) {
        simbriefState.status = avionics::SimBriefStatus::Ok;
        simbriefState.error.clear();
        simbriefState.originIcao = simbriefResult.originIcao;
        simbriefState.destinationIcao = simbriefResult.destinationIcao;
        simbriefState.route = simbriefResult.route;
        simbriefState.generatedUtc = simbriefResult.generatedUtc;
        simbriefState.waypointCount =
            static_cast<int>(simbriefResult.legs.size());
        // The OFP becomes the active flight plan on both feeds (and blocks the
        // later-loading .fms plan from overwriting it on the mock).
        mock.setRoute(simbriefResult.legs);
        mockRouteSet = true;
        xplane.setRouteOverride(simbriefResult.legs);
      } else {
        simbriefState.status = avionics::SimBriefStatus::Error;
        simbriefState.error = simbriefResult.error;
      }
    }
    if (mfdEngine != nullptr) {
      mfdEngine->mfdController().setSimbriefState(simbriefState);
    }

    // FPL page edits become the active flight plan on both feeds: the mock
    // keeps flying (no reposition) and the X-Plane feed's displayed plan is
    // overridden, mirroring the SimBrief flow above.
    if (mfdEngine != nullptr) {
      std::vector<avionics::MapLeg> editedPlan;
      if (mfdEngine->mfdController().consumeFlightPlanEdit(editedPlan)) {
        mock.updateRoute(editedPlan);
        mockRouteSet = true;
        xplane.setRouteOverride(editedPlan);
      }

      // Direct-To activation: the mock flies the direct course; the X-Plane
      // feed shows the magenta direct line (display only over UDP).
      avionics::MapLeg dtoTarget;
      if (mfdEngine->mfdController().consumeDirectToRequest(dtoTarget)) {
        mock.directTo(dtoTarget);
        xplane.setDirectTo(dtoTarget);
      }

      avionics::MapProcedure proc;
      if (mfdEngine->mfdController().consumeProcLoadRequest(proc) &&
          proc.frequencyMhz > 0.0f) {
        mock.tuneRadioStandby(avionics::RadioUnit::Nav1, proc.frequencyMhz);
        xplane.tuneRadioStandby(avionics::RadioUnit::Nav1, proc.frequencyMhz);
      }

      // Map panning: keep both feeds' nearby-data queries centered on the MFD
      // Map Pointer while panning, so the panned-to area loads features /
      // airspaces instead of staying empty around the aircraft.
      const avionics::MfdController& mapUi = mfdEngine->mfdController();
      mock.setMapPanCenter(mapUi.mapPointerActive(), mapUi.mapPointerLat(),
                           mapUi.mapPointerLon());
      xplane.setMapPanCenter(mapUi.mapPointerActive(), mapUi.mapPointerLat(),
                             mapUi.mapPointerLon());
    }

    // PFD radio / transponder commands from the bezel and XPDR softkeys.
    {
      avionics::SoftkeyController& pfdUi = pfdEngine.softkeyController();
      avionics::RadioUnit radioUnit;
      float standbyMhz = 0.0f;
      if (pfdUi.consumeRadioTune(radioUnit, standbyMhz)) {
        mock.tuneRadioStandby(radioUnit, standbyMhz);
        xplane.tuneRadioStandby(radioUnit, standbyMhz);
      }
      if (pfdUi.consumeRadioTransfer(radioUnit)) {
        mock.transferRadio(radioUnit);
        xplane.transferRadio(radioUnit);
      }
      int xpdrCode = 0;
      if (pfdUi.consumeXpdrCodeCommit(xpdrCode)) {
        mock.setTransponderCode(xpdrCode);
        xplane.setTransponderCode(xpdrCode);
      }
      int xpdrMode = 0;
      if (pfdUi.consumeXpdrModeCommit(xpdrMode)) {
        mock.setTransponderMode(xpdrMode);
        xplane.setTransponderMode(xpdrMode);
      }
    }

    // Auto-detect: while showing mock, keep the X-Plane link pumped (the engines
    // only update the active source) and switch over the instant it connects.
    if (app.autoDetectXPlane && !app.usingXPlane) {
      xplane.update(dt);
      if (xplane.connectionState() == avionics::ConnectionState::Connected) {
        SwitchSource(app, true);
      }
    }

    // CLR held past the threshold acts as CLR (DFLT MAP): display the MFD
    // Navigation Map page immediately.
    if (app.clrHoldEngine != nullptr &&
        glfwGetTime() - app.clrHoldStart >=
            avionics::kClrDefaultMapHoldSeconds) {
      app.clrHoldEngine->holdBezelKey(avionics::BezelKey::Clr);
      app.clrHoldEngine = nullptr;
    }

    renderWindow(pfdWindow, pfdEngine, pfdRenderer, dt);
    if (mfdWindow != nullptr && mfdEngine != nullptr) {
      renderWindow(mfdWindow, *mfdEngine, *mfdRenderer, dt);
    }

    // Persist durable display preferences whenever the pilot changes one (e.g.
    // toggling the PFD inset map), so they survive the next launch. Writes only
    // happen on an actual change, so the common no-change frame costs a cheap
    // struct compare.
    {
      avionics::AvionicsPersistentState current = app.settings.avionics;
      avionics::capturePfdState(pfdEngine.softkeyController(), current.pfd);
      if (mfdEngine != nullptr) {
        avionics::captureMfdState(mfdEngine->mfdController(), current.mfd);
      }
      if (current != app.settings.avionics) {
        app.settings.avionics = current;
        avionics::SaveAppSettings(app.settings);
      }
    }

    glfwPollEvents();
  }

  // Capture the final window placement for the next launch before the windows
  // go away.
  if (app.settings.rememberWindowPos) {
    CaptureWindowPositions(app);
    avionics::SaveAppSettings(app.settings);
  }

  // Tear down GL objects while their contexts are still current.
  if (mfdWindow != nullptr) {
    glfwMakeContextCurrent(mfdWindow);
    delete mfdEngine;
    delete mfdRenderer;
    glfwDestroyWindow(mfdWindow);
  }
  glfwDestroyWindow(pfdWindow);
  if (app.rotateCursor != nullptr) glfwDestroyCursor(app.rotateCursor);
  if (app.handCursor != nullptr) glfwDestroyCursor(app.handCursor);
  glfwTerminate();
  return 0;
}
