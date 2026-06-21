// Standalone shell.
//
// Owns its own windows and frame loop (unlike the plugin, which is driven by the
// sim). Puts the shared PFD and MFD on screen as two GLFW windows (each with its
// own GL context + NanoVgRenderer + AvionicsEngine, both reading one shared data
// source), fed by a live X-Plane connection (UDP RREF). Pass --no-mfd to run the
// PFD window only, or --no-pfd to run the MFD window only (so the two displays
// can be launched as separate processes).
//
// The displays always read the live X-Plane link: until the sim starts
// delivering data the engine shows the power-up / waiting screen, then the live
// pages. (The --screenshot dev mode renders a mock feed offscreen for asset
// iteration; it never appears in the interactive app.)
//
//   --no-mfd                  open only the PFD window (no MFD)
//   --no-pfd                  open only the MFD window (no PFD); the MFD then
//                             drives the shared data source and owns vsync
//   --demo                    boot straight into the built-in demo feed (the
//                             same motion-only mock Ctrl+Shift+D toggles), so
//                             the displays show believable motion with no sim
//   --debug-menu              install the dev-only macOS "Debug" menu (switch
//                             the data source between X-Plane and the demo
//                             ground/flying/turbulence states, and flip the
//                             demo Master/Avionics power switches). The choices
//                             are remembered across runs. Passed by the IDE
//                             launch configs; never by the installer
//   --fullscreen              run both displays borderless full screen, each
//                             taking over a monitor (the 4:3 image letterboxed)
//   --no-fullscreen           force windowed, overriding the saved preference
//   --pfd-monitor N           put the PFD full screen on monitor N (implies
//                             --fullscreen for the PFD; see --list-monitors)
//   --mfd-monitor N           put the MFD full screen on monitor N
//   --list-monitors           print the connected monitors and indices, then
//                             exit
//   --identify-monitors       flash each monitor's index large on that screen
//                             for a few seconds (--time SECONDS), then exit;
//                             the visual companion to --list-monitors
//   --xplane-host HOST        X-Plane host (default: 127.0.0.1)
//   --xplane-port PORT        X-Plane UDP port (default: 49000)
//   --fms-bridge-port PORT    UDP port of the in-sim flight-plan bridge
//                             (shell-xplane plugin) that serves the live FMS
//                             route over the network (default: 49100)
//   --command-bridge-port PORT
//                             UDP port the standalone listens on for G1000
//                             bezel/softkey events forwarded by the plugin
//                             (default: 49101)
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
//   --obstacles PATH          override the bundled FAA DDOF CSV for the map's
//                             obstacle overlay (default: assets/obstacles.csv
//                             next to the app / plugin when present)
//   --nav-data-dir PATH       directory holding a copied X-Plane nav-data tree
//                             (Custom Data/, Resources/default data/, Global
//                             Scenery/, Custom Data/CIFP/, Custom Data/
//                             Airspaces/) used instead of a local X-Plane
//                             install, so the standalone can run the moving map
//                             on a PC without X-Plane; live telemetry still
//                             comes over the network (--xplane-host)

#if defined(__APPLE__)
#include <OpenGL/gl3.h>  // GL_SILENCE_DEPRECATION is set by the build.
#include <mach-o/dyld.h>  // _NSGetExecutablePath, to locate bundled assets.
#endif
#if defined(_WIN32)
#include <windows.h>  // GetModuleFileNameW, to locate bundled assets.
#endif
#if !defined(__APPLE__)
#include <GL/glew.h>  // GL3 entry points on Windows/Linux (must precede gl.h).
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "AppSettings.h"
#include "avionics/ChecklistStore.h"
#include "CommandBridgeClient.h"
#include "avionics/EisStore.h"
#include "DsfTerrainStore.h"
#include "FmsPlanStore.h"
#include "MacMenu.h"
#include "NavData.h"
#include "ObstacleStore.h"
#include "ProcedureStore.h"
#include "ShellNavMapData.h"
#include "SimBriefStore.h"
#include "UpdateNotify.h"
#include "XPlaneConnection.h"
#include "XPlaneInstall.h"
#include "avionics/AssetPaths.h"
#include "avionics/CommandBridgeProtocol.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/ConnectionState.h"
#include "avionics/ComDecode.h"
#include "avionics/MockDataSource.h"
#include "avionics/PersistentState.h"
#include "avionics/UpdateChecker.h"
#include "avionics/SimBrief.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/BootScreen.h"
#include "avionics/render/GlLoader.h"
#include "avionics/render/NanoVgRenderer.h"
#include "avionics/render/SoftkeyBezel.h"

namespace {

// Absolute directory containing the running executable, used to find bundled
// assets in a distributed build. Returns empty on failure (the asset resolver
// then falls back to the compile-time development paths).
std::string ExecutableDir() {
  namespace fs = std::filesystem;
  std::error_code ec;
#if defined(__APPLE__)
  std::uint32_t size = 0;
  _NSGetExecutablePath(nullptr, &size);
  std::string buf(size, '\0');
  if (_NSGetExecutablePath(buf.data(), &size) != 0) return std::string();
  const fs::path exe = fs::weakly_canonical(fs::path(buf), ec);
#elif defined(_WIN32)
  std::wstring buf(MAX_PATH, L'\0');
  DWORD len = GetModuleFileNameW(nullptr, buf.data(),
                                 static_cast<DWORD>(buf.size()));
  if (len == 0) return std::string();
  buf.resize(len);
  const fs::path exe = fs::weakly_canonical(fs::path(buf), ec);
#else
  const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
#endif
  if (ec) return std::string();
  return exe.parent_path().string();
}

// Register the locations where bundled runtime assets may live, relative to the
// installed executable, so a distributed build finds its fonts/EIS/map data/
// checklists. Harmless during development (none of these exist when running
// from the build tree, so the resolver falls back to the source paths).
void RegisterAssetSearchDirs() {
  namespace fs = std::filesystem;
  const std::string dir = ExecutableDir();
  if (dir.empty()) return;
  const fs::path exeDir(dir);
  // Layout next to the binary (Windows/Linux portable, and the dev convention).
  avionics::assets::addSearchDir((exeDir / "assets").string());
  // macOS .app bundle: Contents/MacOS/<exe> -> Contents/Resources/assets.
  avionics::assets::addSearchDir(
      (exeDir.parent_path() / "Resources" / "assets").string());
  // Linux FHS install: bin/<exe> -> share/g1000-nxi/assets.
  avionics::assets::addSearchDir(
      (exeDir.parent_path() / "share" / "g1000-nxi" / "assets").string());
}

constexpr int kWindowWidth = 1024;   // avionics "screen" region (4:3)
constexpr int kWindowHeight = 768;
// The hardware bezel strips are part of the window, the way a GDU's physical
// keys frame the display (G1000 NXi Pilot's Guide Figure 1-2): the NAV/HDG knob
// column on the left, the COM/CRS-BARO/RANGE/FMS key column on the right, and
// the row of twelve softkey selection keys below the screen. The window is the
// screen plus those strips.
constexpr int kLeftBezelStripWidth = 112;
constexpr int kBezelStripWidth = 112;
constexpr int kSoftkeyStripHeight = 72;
constexpr int kSuiteWidth = kLeftBezelStripWidth + kWindowWidth + kBezelStripWidth;
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
inline int LeftBezelStripPx(int fbWidth) {
  return static_cast<int>(std::lround(static_cast<double>(fbWidth) *
                                      kLeftBezelStripWidth / kSuiteWidth));
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

// Windows can set a monitor to Landscape (flipped) (180°). The desktop compositor
// rotates unfocused window surfaces, but focused borderless/topmost OpenGL
// windows on many drivers bypass that path — the panel looks upside down. Compensate
// in NanoVG after nvgBeginFrame (a glViewport flip alone is undone by the gauge
// draw path). Mouse hit-testing must apply the same flip so bezel clicks stay
// aligned. Only flip while the window is focused; when unfocused, DWM already
// applies the monitor rotation and a GL flip double-rotates.
inline void FlipContentCoords(double& fx, double& fy, int contentW,
                              int contentH, bool flip180) {
  if (!flip180) return;
  fx = static_cast<double>(contentW) - fx;
  fy = static_cast<double>(contentH) - fy;
}

#if defined(_WIN32)
enum class MonitorOrientation {
  Landscape,
  Portrait,
  LandscapeFlipped,
  PortraitFlipped,
  Unknown,
};

MonitorOrientation OrientationForMonitor(HMONITOR monitor) {
  MONITORINFOEXW info{};
  info.cbSize = sizeof(info);
  if (!GetMonitorInfoW(monitor, reinterpret_cast<MONITORINFO*>(&info))) {
    return MonitorOrientation::Unknown;
  }
  DEVMODEW mode{};
  mode.dmSize = sizeof(mode);
  if (!EnumDisplaySettingsW(info.szDevice, ENUM_CURRENT_SETTINGS, &mode)) {
    return MonitorOrientation::Unknown;
  }
  if ((mode.dmFields & DM_DISPLAYORIENTATION) == 0) {
    return MonitorOrientation::Landscape;
  }
  switch (mode.dmDisplayOrientation) {
    case DMDO_DEFAULT:
      return MonitorOrientation::Landscape;
    case DMDO_90:
      return MonitorOrientation::Portrait;
    case DMDO_180:
      return MonitorOrientation::LandscapeFlipped;
    case DMDO_270:
      return MonitorOrientation::PortraitFlipped;
    default:
      return MonitorOrientation::Unknown;
  }
}

const char* MonitorOrientationLabel(MonitorOrientation orientation) {
  switch (orientation) {
    case MonitorOrientation::Landscape:
      return "landscape";
    case MonitorOrientation::Portrait:
      return "portrait";
    case MonitorOrientation::LandscapeFlipped:
      return "landscape (flipped)";
    case MonitorOrientation::PortraitFlipped:
      return "portrait (flipped)";
    default:
      return "unknown orientation";
  }
}

MonitorOrientation OrientationForGlfwMonitor(GLFWmonitor* monitor) {
  if (monitor == nullptr) return MonitorOrientation::Unknown;
  int mx = 0, my = 0;
  glfwGetMonitorPos(monitor, &mx, &my);
  const POINT pt = {mx + 1, my + 1};
  const HMONITOR hmon = MonitorFromPoint(pt, MONITOR_DEFAULTTONULL);
  if (hmon == nullptr) return MonitorOrientation::Unknown;
  return OrientationForMonitor(hmon);
}
#else
enum class MonitorOrientation { Unknown };
#endif

GLFWmonitor* MonitorForWindow(GLFWwindow* window);

#if defined(_WIN32)
bool NeedsDisplayFlip180(GLFWwindow* window, GLFWmonitor* pinnedMonitor) {
  if (const char* disable = std::getenv("AVIONICS_NO_DISPLAY_FLIP");
      disable != nullptr && disable[0] != '\0') {
    return false;
  }
  if (const char* force = std::getenv("AVIONICS_FORCE_DISPLAY_FLIP");
      force != nullptr && force[0] != '\0') {
    return true;
  }
  GLFWmonitor* monitor = pinnedMonitor;
  if (monitor == nullptr && window != nullptr) {
    monitor = MonitorForWindow(window);
  }
  if (monitor == nullptr) return false;
  if (OrientationForGlfwMonitor(monitor) !=
      MonitorOrientation::LandscapeFlipped) {
    return false;
  }
  // Unfocused surfaces are already rotation-corrected by DWM on DMDO_180
  // displays; flipping there double-rotates (correct when you click away).
  if (window != nullptr && glfwGetWindowAttrib(window, GLFW_FOCUSED) == 0) {
    return false;
  }
  return true;
}
#else
bool NeedsDisplayFlip180(GLFWwindow* /*window*/,
                         GLFWmonitor* /*pinnedMonitor*/) {
  return false;
}
#endif

// Renders the suite into an arbitrary viewport sub-rectangle of the current
// framebuffer (vpX/vpY are GL bottom-left anchored). The full-window path passes
// the whole framebuffer; the full-screen path passes a centered, aspect-
// preserving rectangle so the 4:3 image is letterboxed rather than stretched to
// a wide monitor. All of the internal geometry (bezel strips, screen region) is
// computed relative to the viewport, so it scales with the rectangle.
inline void RenderSuiteViewport(avionics::NanoVgRenderer& renderer,
                                avionics::AvionicsEngine& eng, int vpX, int vpY,
                                int vpW, int vpH, bool showBezel, bool flip180,
                                avionics::NanoVgRenderer::DrawStats*
                                    engineStats = nullptr) {
  // With the bezel hidden the screen fills the whole viewport (no physical key
  // strips to frame it), so the engine draws into the full rectangle.
  if (!showBezel) {
    glViewport(vpX, vpY, vpW, vpH);
    renderer.setDisplayFlip180(flip180);
    eng.renderFrame(vpW, vpH, 1.0f);
    renderer.setDisplayFlip180(false);
    // Capture the engine's draw stats before any further beginFrame resets
    // them (the bezel path below resets on its own beginFrame).
    if (engineStats != nullptr) *engineStats = renderer.drawStats();
    return;
  }

  const int leftPx = LeftBezelStripPx(vpW);
  const int bezelPx = BezelStripPx(vpW);
  const int softkeyPx = SoftkeyStripPx(vpH);
  const int screenW = std::max(1, vpW - leftPx - bezelPx);
  const int screenH = std::max(1, vpH - softkeyPx);

  // GL viewports are bottom-left anchored, so the screen's viewport is lifted
  // by the softkey strip's height to sit at the top of the rectangle, and
  // shifted right by the left key column.
  glViewport(vpX + leftPx, vpY + (vpH - screenH), screenW, screenH);
  renderer.setDisplayFlip180(flip180);
  eng.renderFrame(screenW, screenH, 1.0f);
  renderer.setDisplayFlip180(false);
  // Snapshot the engine's per-frame draw stats now: the bezel beginFrame below
  // zeroes the counter, so reading it after RenderSuite would only show the
  // bezel (a constant), not the map content we're profiling.
  if (engineStats != nullptr) *engineStats = renderer.drawStats();

  glViewport(vpX, vpY, vpW, vpH);
  renderer.setDisplayFlip180(flip180);
  renderer.beginFrame(vpW, vpH, 1.0f);
  renderer.setDisplayFlip180(false);
  avionics::BezelKeyPanel::renderLeft(renderer, 0.0f, 0.0f,
                                      static_cast<float>(leftPx),
                                      static_cast<float>(vpH),
                                      static_cast<float>(screenH),
                                      eng.bezelPressLevels());
  avionics::BezelKeyPanel::render(
      renderer, static_cast<float>(leftPx + screenW), 0.0f,
      static_cast<float>(vpW - leftPx - screenW), static_cast<float>(vpH),
      static_cast<float>(screenH), eng.bezelPressLevels());
  avionics::SoftkeyBezelPanel::render(
      renderer, static_cast<float>(leftPx), static_cast<float>(screenH),
      static_cast<float>(screenW), static_cast<float>(vpH - screenH),
      eng.softkeyPressLevels());
  renderer.endFrame();
}

// Fills the whole framebuffer with the suite (the normal window path, where the
// window is already sized to the suite's aspect ratio).
inline void RenderSuite(avionics::NanoVgRenderer& renderer,
                        avionics::AvionicsEngine& eng, int fbWidth,
                        int fbHeight, bool showBezel = true,
                        bool flip180 = false,
                        avionics::NanoVgRenderer::DrawStats* engineStats =
                            nullptr) {
  RenderSuiteViewport(renderer, eng, 0, 0, fbWidth, fbHeight, showBezel,
                      flip180, engineStats);
}

constexpr const char* kSourceXPlane = "xplane";
constexpr const char* kLabelMock = "MOCK DATA";  // --screenshot dev mode only
constexpr const char* kLabelDemo = "DEMO DATA";  // built-in demo feed toggle
constexpr const char* kDefaultXPlaneHost = "127.0.0.1";
constexpr std::uint16_t kDefaultXPlanePort = 49000;

// Bundled Natural Earth land-data asset path, baked in by the build (empty when
// the build provides none; LandDataStore then loads nothing).
#ifndef AVIONICS_LAND_DATA
#define AVIONICS_LAND_DATA ""
#endif
constexpr const char* kLandDataAssetPath = AVIONICS_LAND_DATA;

#ifndef AVIONICS_OBSTACLES
#define AVIONICS_OBSTACLES ""
#endif
constexpr const char* kObstaclesAssetPath = AVIONICS_OBSTACLES;

std::string resolveObstacleDatabasePath(const char* cliOverride) {
  if (cliOverride != nullptr && cliOverride[0] != '\0') {
    return cliOverride;
  }
  return avionics::assets::resolve("obstacles.csv", kObstaclesAssetPath);
}

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
  avionics::AvionicsEngine* pfdEngine = nullptr;  // null when --no-pfd
  avionics::AvionicsEngine* mfdEngine = nullptr;  // null when --no-mfd
  GLFWwindow* pfdWindow = nullptr;
  GLFWwindow* mfdWindow = nullptr;
  // The standalone normally reads the live X-Plane connection: the engine shows
  // the power-up / waiting screen until the sim starts delivering data, then the
  // live pages. The built-in demo (mock) feed lets the displays show believable
  // motion without a running sim; Ctrl+Shift+D toggles between them (a
  // deliberately awkward chord so it isn't pressed by accident). activeSource
  // points at whichever feed is currently driving the displays.
  avionics::SimulatorConnection* xplane = nullptr;
  avionics::MockDataSource* demoSource = nullptr;
  avionics::DataSource* activeSource = nullptr;
  bool demoMode = false;
  // Whether the dev-only Debug menu is installed (launched with --debug-menu).
  // When set, the data-source / demo-state / power selections are restored from
  // and persisted to the settings file; the installed app leaves this false.
  bool debugMenuEnabled = false;
  // Whether the hardware bezel strips are drawn (and the windows sized to
  // include them). Mirrors settings.showBezel; toggled with the B key.
  bool showBezel = true;
  // Whether each display currently fills a whole monitor (borderless full
  // screen). Mirrors settings.pfd/mfdFullscreen; toggled with the F key.
  bool pfdFullscreen = false;
  bool mfdFullscreen = false;
  // Windowed placement saved when a display enters full screen, restored when
  // it leaves so the windows return to where they were.
  int pfdRestoreX = 0, pfdRestoreY = 0, pfdRestoreW = 0, pfdRestoreH = 0;
  int mfdRestoreX = 0, mfdRestoreY = 0, mfdRestoreW = 0, mfdRestoreH = 0;
  // Persisted user preferences, written back whenever the user changes the
  // feed or flips one of the View menu toggles.
  avionics::AppSettings settings;
  // In-progress CLR press-and-hold (CLR DFLT MAP): the engine whose CLR bezel
  // key the mouse went down on, and when. Cleared on release or once the hold
  // function fires.
  avionics::AvionicsEngine* clrHoldEngine = nullptr;
  double clrHoldStart = 0.0;
  // Hover cursors for the on-screen bezel: a left-right cursor over the
  // rotatable FMS knob rings, circular rotate cursors over the RANGE joystick's
  // zoom ring (a clockwise arrow on the zoom-out side, a counter-clockwise
  // arrow on the zoom-in side), and a hand over the other clickable controls.
  // Created at startup, freed at shutdown.
  GLFWcursor* rotateCursor = nullptr;
  GLFWcursor* handCursor = nullptr;
  GLFWcursor* rangeCwCursor = nullptr;
  GLFWcursor* rangeCcwCursor = nullptr;
  // When borderless full-screen, the monitor each display is pinned to (used to
  // detect landscape-flipped panels before the Win32 window is fully settled).
  GLFWmonitor* pfdPinnedMonitor = nullptr;
  GLFWmonitor* mfdPinnedMonitor = nullptr;
  // Accumulated scroll-wheel delta over the RANGE joystick, so a high-resolution
  // trackpad steps the map range one ladder stop per whole notch instead of
  // racing through the ladder. Reset whenever the wheel turns off the knob.
  double rangeScrollAccum = 0.0;
};

inline GLFWmonitor* PinnedMonitorForWindow(const AppState& app,
                                           GLFWwindow* window) {
  if (window == app.pfdWindow) return app.pfdPinnedMonitor;
  if (window == app.mfdWindow) return app.mfdPinnedMonitor;
  return nullptr;
}

// Window dimensions for the two bezel states: the full suite (screen + strips)
// when the bezel is shown, and the bare 4:3 screen when it is hidden.
inline int SuiteWindowWidth(bool showBezel) {
  return showBezel ? kSuiteWidth : kWindowWidth;
}
inline int SuiteWindowHeight(bool showBezel) {
  return showBezel ? kSuiteHeight : kWindowHeight;
}

// Largest rectangle inside a framebuffer that keeps the suite's aspect ratio
// (the 4:3 screen, plus the bezel strips when shown), centered with black bars.
// For a window sized to the suite this is the whole framebuffer; for a
// full-screen window on a monitor of a different shape it pillar/letterboxes the
// image so the gauges keep their proportions instead of stretching. Returned in
// framebuffer pixels with a top-left origin (so it composes with mouse
// coordinates); the renderer flips y for the GL viewport.
struct ContentRect {
  int x, y, w, h;
};
inline ContentRect ComputeContentRect(int fbWidth, int fbHeight,
                                      bool showBezel) {
  ContentRect rect{0, 0, fbWidth, fbHeight};
  if (fbWidth <= 0 || fbHeight <= 0) return rect;
  const double aspect = static_cast<double>(SuiteWindowWidth(showBezel)) /
                        static_cast<double>(SuiteWindowHeight(showBezel));
  const double fbAspect =
      static_cast<double>(fbWidth) / static_cast<double>(fbHeight);
  if (fbAspect > aspect) {  // framebuffer wider than the suite: pillarbox
    rect.h = fbHeight;
    rect.w = static_cast<int>(std::lround(fbHeight * aspect));
    rect.x = (fbWidth - rect.w) / 2;
    rect.y = 0;
  } else {  // framebuffer taller than the suite: letterbox
    rect.w = fbWidth;
    rect.h = static_cast<int>(std::lround(fbWidth / aspect));
    rect.x = 0;
    rect.y = (fbHeight - rect.h) / 2;
  }
  return rect;
}

// Records the current window placement into the settings (window positions are
// always restored on the next launch). Positions are screen coordinates of the
// content area's top-left corner, as reported by GLFW.
void CaptureWindowPositions(AppState& app) {
  // A full-screen window sits at its monitor's origin; capturing that would
  // clobber the saved windowed layout, so skip it and keep the prior values.
  // Each display is captured independently so a single-display run (--no-mfd or
  // --no-pfd) still remembers its own window placement.
  bool captured = false;
  if (app.pfdWindow != nullptr && !app.pfdFullscreen) {
    glfwGetWindowPos(app.pfdWindow, &app.settings.pfdWindowX,
                     &app.settings.pfdWindowY);
    captured = true;
  }
  if (app.mfdWindow != nullptr && !app.mfdFullscreen) {
    glfwGetWindowPos(app.mfdWindow, &app.settings.mfdWindowX,
                     &app.settings.mfdWindowY);
    captured = true;
  } else if (app.mfdWindow == nullptr && app.pfdWindow != nullptr &&
             !app.pfdFullscreen) {
    // PFD-only run: save the docked position so a later dual-window launch
    // still puts the MFD beside the PFD.
    app.settings.mfdWindowX = app.settings.pfdWindowX +
                              SuiteWindowWidth(app.showBezel) + kWindowGap;
    app.settings.mfdWindowY = app.settings.pfdWindowY;
    captured = true;
  }
  if (captured) app.settings.hasWindowPos = true;
}

// Resizes both windows to match the current bezel state and keeps the MFD
// docked just to the right of the PFD.
void ApplyBezelWindowSize(AppState& app) {
  const int w = SuiteWindowWidth(app.showBezel);
  const int h = SuiteWindowHeight(app.showBezel);
  // Full-screen displays keep filling their monitor; the bezel change only
  // affects how the image is letterboxed within it, not the window size.
  if (app.pfdWindow != nullptr && !app.pfdFullscreen) {
    glfwSetWindowSize(app.pfdWindow, w, h);
  }
  if (app.mfdWindow != nullptr && !app.mfdFullscreen) {
    glfwSetWindowSize(app.mfdWindow, w, h);
    if (app.pfdWindow != nullptr && !app.pfdFullscreen) {
      int px = 0, py = 0;
      glfwGetWindowPos(app.pfdWindow, &px, &py);
      glfwSetWindowPos(app.mfdWindow, px + w + kWindowGap, py);
    }
  }
}

// Which engine owns keyboard Enter during boot: the focused window's GDU only
// (a real unit acknowledges per display; the PFD is already live).
avionics::AvionicsEngine* BootAckEngineForWindow(const AppState& app,
                                                  GLFWwindow* window) {
  if (window == app.mfdWindow) return app.mfdEngine;
  if (window == app.pfdWindow) return app.pfdEngine;
  return nullptr;
}

// Drives both displays with the feed (and, for the demo feed, the flight state)
// named by `sel`. The PFD skips boot on source switches for an immediate view;
// the MFD replays the power-up page and waits for ENT. When `persist` is set the choice is written to the settings file so the next
// launch restores it; the macOS Debug menu checkmarks are kept in sync either
// way. This is the single entry point for every data-source change (the Debug
// menu, the Ctrl+Shift+D toggle, and startup restore all route through it).
void ApplyDebugDataSource(AppState& app, avionics::DebugDataSource sel,
                          bool persist) {
  if (app.xplane == nullptr || app.demoSource == nullptr) return;
  const bool demo = sel != avionics::DebugDataSource::XPlane;
  app.demoMode = demo;
  if (demo) {
    app.demoSource->setGroundMode(sel == avionics::DebugDataSource::DemoGround);
    app.demoSource->setTurbulenceEnabled(
        sel == avionics::DebugDataSource::DemoTurbulence);
  }
  avionics::DataSource* next =
      demo ? static_cast<avionics::DataSource*>(app.demoSource)
           : static_cast<avionics::DataSource*>(app.xplane);
  const std::string label =
      demo ? kLabelDemo : std::string(app.xplane->simulatorName());
  app.activeSource = next;
  if (app.pfdEngine != nullptr) {
    app.pfdEngine->setDataSource(*next, label);
    app.pfdEngine->skipBoot();
    // Persistent CAS banner on the PFD while the demo feed drives the displays.
    app.pfdEngine->softkeyController().setDemoBanner(demo);
  }
  if (app.mfdEngine != nullptr) {
    app.mfdEngine->setDataSource(*next, label);
  }
  app.settings.debugDataSource = sel;
  if (persist) avionics::SaveAppSettings(app.settings);
#if defined(__APPLE__)
  avionics::SyncDebugMenuSource(sel);
#endif
}

// Swaps both displays between the live X-Plane feed and the built-in demo
// (mock) feed. Bound to Ctrl+Shift+D (an awkward chord so it isn't hit by
// accident). Returns to whichever demo flight state was last selected (calm
// flying by default), so the chord pairs with the Debug menu's richer states.
void ToggleDemoSource(AppState& app) {
  if (app.xplane == nullptr || app.demoSource == nullptr) return;
  avionics::DebugDataSource sel;
  if (app.demoMode) {
    sel = avionics::DebugDataSource::XPlane;
  } else {
    sel = app.settings.debugDataSource == avionics::DebugDataSource::XPlane
              ? avionics::DebugDataSource::DemoFlying
              : app.settings.debugDataSource;
  }
  ApplyDebugDataSource(app, sel, /*persist=*/app.debugMenuEnabled);
  std::fprintf(stderr, "Data source: %s\n",
               app.demoMode ? "DEMO (built-in mock feed)" : "X-PLANE (live)");
}

// Applies the demo feed's Master / Avionics power switches (master gates the
// PFD; avionics additionally gates the MFD) and, when `persist` is set, saves
// the choice. The live X-Plane feed takes its power from the sim datarefs, so
// these toggles only affect the demo feed.
void ApplyDebugPower(AppState& app, bool masterOn, bool avionicsOn,
                     bool persist) {
  if (app.demoSource != nullptr) {
    app.demoSource->setMasterPowerOn(masterOn);
    app.demoSource->setAvionicsPowerOn(avionicsOn);
  }
  app.settings.demoMasterPowerOn = masterOn;
  app.settings.demoAvionicsPowerOn = avionicsOn;
  if (persist) avionics::SaveAppSettings(app.settings);
#if defined(__APPLE__)
  avionics::SyncDebugMenuPower(masterOn, avionicsOn);
#endif
}

// Enables or disables the demo feed's cycling CAS annunciations (the periodic
// OIL PRESSURE / LOW VOLTS / FUEL LOW etc. alerts) and, when `persist` is set,
// saves the choice. The live X-Plane feed's CAS comes from the sim datarefs, so
// this only affects the demo feed.
void ApplyDebugCas(AppState& app, bool enabled, bool persist) {
  if (app.demoSource != nullptr) {
    app.demoSource->setCasMessagesEnabled(enabled);
  }
  app.settings.demoCasMessagesOn = enabled;
  if (persist) avionics::SaveAppSettings(app.settings);
#if defined(__APPLE__)
  avionics::SyncDebugMenuCas(enabled);
#endif
}

// Debug-menu callbacks (the menu fires these on the main thread).
void OnDebugSelectSource(void* context, avionics::DebugDataSource sel) {
  ApplyDebugDataSource(*static_cast<AppState*>(context), sel, /*persist=*/true);
}
void OnDebugTogglePower(void* context, bool masterOn, bool avionicsOn) {
  ApplyDebugPower(*static_cast<AppState*>(context), masterOn, avionicsOn,
                  /*persist=*/true);
}
void OnDebugToggleCas(void* context, bool enabled) {
  ApplyDebugCas(*static_cast<AppState*>(context), enabled, /*persist=*/true);
}

void ApplyRadioBridgeAction(avionics::AvionicsEngine& engine,
                            avionics::cmdbridge::RadioAction action) {
  switch (action) {
    case avionics::cmdbridge::RadioAction::ComToggle:
      engine.selectComRadio();
      break;
    case avionics::cmdbridge::RadioAction::ComFlip:
      engine.transferComRadio();
      break;
    case avionics::cmdbridge::RadioAction::ComOuterUp:
      engine.tuneComRadio(+1, /*coarse=*/true);
      break;
    case avionics::cmdbridge::RadioAction::ComOuterDown:
      engine.tuneComRadio(-1, /*coarse=*/true);
      break;
    case avionics::cmdbridge::RadioAction::ComInnerUp:
      engine.tuneComRadio(+1, /*coarse=*/false);
      break;
    case avionics::cmdbridge::RadioAction::ComInnerDown:
      engine.tuneComRadio(-1, /*coarse=*/false);
      break;
    case avionics::cmdbridge::RadioAction::NavToggle:
      engine.selectNavRadio();
      break;
    case avionics::cmdbridge::RadioAction::NavFlip:
      engine.transferNavRadio();
      break;
    case avionics::cmdbridge::RadioAction::NavOuterUp:
      engine.tuneNavRadio(+1, /*coarse=*/true);
      break;
    case avionics::cmdbridge::RadioAction::NavOuterDown:
      engine.tuneNavRadio(-1, /*coarse=*/true);
      break;
    case avionics::cmdbridge::RadioAction::NavInnerUp:
      engine.tuneNavRadio(+1, /*coarse=*/false);
      break;
    case avionics::cmdbridge::RadioAction::NavInnerDown:
      engine.tuneNavRadio(-1, /*coarse=*/false);
      break;
    case avionics::cmdbridge::RadioAction::ComVolUp:
      engine.pressBezelKey(avionics::BezelKey::ComVolCw);
      break;
    case avionics::cmdbridge::RadioAction::ComVolDown:
      engine.pressBezelKey(avionics::BezelKey::ComVolCcw);
      break;
    case avionics::cmdbridge::RadioAction::NavVolUp:
      engine.pressBezelKey(avionics::BezelKey::NavVolCw);
      break;
    case avionics::cmdbridge::RadioAction::NavVolDown:
      engine.pressBezelKey(avionics::BezelKey::NavVolCcw);
      break;
    case avionics::cmdbridge::RadioAction::NavVolPush:
      engine.pressBezelKey(avionics::BezelKey::NavVolPush);
      break;
  }
}

// Toggles manual display-backup mode once and flashes the left-bezel key on
// both GDU windows (shared DataSource; either engine may drive the toggle).
void ToggleDisplayBackup(AppState& app) {
  avionics::AvionicsEngine* engine =
      app.pfdEngine != nullptr ? app.pfdEngine : app.mfdEngine;
  if (engine != nullptr) {
    engine->toggleDisplayBackup();
  } else if (app.activeSource != nullptr) {
    app.activeSource->toggleDisplayBackup();
  }
}

// MFD GDU FMS keys are inert while a PFD pop-up owns FMS input. ENT on the MFD
// still acknowledges the power-up page even when a PFD pop-up is open.
bool BridgeFmsKeyBlockedByPfd(const AppState& app,
                              avionics::cmdbridge::Device device,
                              avionics::BezelKey key) {
  if (device != avionics::cmdbridge::Device::Mfd) return false;
  if (key == avionics::BezelKey::Ent && app.mfdEngine != nullptr &&
      app.mfdEngine->awaitingPowerUpAck()) {
    return false;
  }
  if (app.pfdEngine == nullptr) return false;
  if (!app.pfdEngine->softkeyController().pfdClaimsFmsInput()) return false;
  return avionics::isGduFmsInputKey(key);
}

// Applies bezel / softkey / radio events forwarded from the in-sim plugin.
void ApplyBridgeEvents(AppState& app,
                       const std::vector<avionics::cmdbridge::Event>& events) {
  for (const avionics::cmdbridge::Event& ev : events) {
    if (ev.kind == avionics::cmdbridge::Kind::Radio) {
      if (ev.phase != avionics::cmdbridge::Phase::Begin ||
          app.pfdEngine == nullptr) {
        continue;
      }
      ApplyRadioBridgeAction(
          *app.pfdEngine,
          static_cast<avionics::cmdbridge::RadioAction>(ev.value));
      continue;
    }

    if (ev.kind == avionics::cmdbridge::Kind::GcuEntry) {
      if (ev.phase != avionics::cmdbridge::Phase::Begin) continue;
      avionics::AvionicsEngine* engine =
          ev.device == avionics::cmdbridge::Device::Mfd ? app.mfdEngine
                                                        : app.pfdEngine;
      if (engine == nullptr) continue;
      const char ch =
          static_cast<char>(static_cast<unsigned char>(ev.value & 0xFF));
      engine->applyGcuEntryKey(ch);
      continue;
    }

    avionics::AvionicsEngine* engine =
        ev.device == avionics::cmdbridge::Device::Mfd ? app.mfdEngine
                                                      : app.pfdEngine;
    if (engine == nullptr) continue;

    if (ev.kind == avionics::cmdbridge::Kind::Softkey) {
      if (ev.phase == avionics::cmdbridge::Phase::Begin) {
        engine->pressSoftkey(ev.value);
      }
      continue;
    }

    if (ev.kind == avionics::cmdbridge::Kind::BezelDiagonal) {
      if (ev.phase == avionics::cmdbridge::Phase::Begin) {
        const auto keyA = static_cast<avionics::BezelKey>(ev.value);
        const auto keyB = static_cast<avionics::BezelKey>(ev.value2);
        if (BridgeFmsKeyBlockedByPfd(app, ev.device, keyA) ||
            BridgeFmsKeyBlockedByPfd(app, ev.device, keyB)) {
          continue;
        }
        engine->pressBezelKey(keyA);
        if (ev.value2 >= 0) {
          engine->pressBezelKey(keyB);
        }
      }
      continue;
    }

    if (ev.kind != avionics::cmdbridge::Kind::Bezel) continue;

    const auto key = static_cast<avionics::BezelKey>(ev.value);
    if (BridgeFmsKeyBlockedByPfd(app, ev.device, key)) continue;

    if (ev.phase == avionics::cmdbridge::Phase::Begin) {
      if (key == avionics::BezelKey::Ent && engine->awaitingPowerUpAck()) {
        engine->acknowledgePowerUp();
      } else if (key == avionics::BezelKey::DisplayBackup) {
        ToggleDisplayBackup(app);
      } else if (key != avionics::BezelKey::Count) {
        engine->pressBezelKey(key);
      }
      if (key == avionics::BezelKey::Clr) {
        app.clrHoldEngine = engine;
        app.clrHoldStart = glfwGetTime();
      }
    } else if (ev.phase == avionics::cmdbridge::Phase::Continue &&
               key == avionics::BezelKey::Clr) {
      engine->holdBezelKey(avionics::BezelKey::Clr);
      app.clrHoldEngine = nullptr;
    } else if (ev.phase == avionics::cmdbridge::Phase::End &&
               key == avionics::BezelKey::Clr) {
      app.clrHoldEngine = nullptr;
    }
  }
}

// B-key action: shows/hides the hardware bezel strips and remembers the choice
// across runs.
void OnMenuToggleBezel(void* context, bool showBezel) {
  auto* app = static_cast<AppState*>(context);
  app->showBezel = showBezel;
  app->settings.showBezel = showBezel;
  avionics::SaveAppSettings(app->settings);
  ApplyBezelWindowSize(*app);
}

// T-key action: shows/hides the OS window chrome (the title bar with its
// close / minimize / maximize controls) on both windows.
void OnMenuToggleWindowChrome(void* context, bool showChrome) {
  auto* app = static_cast<AppState*>(context);
  const int decorated = showChrome ? GLFW_TRUE : GLFW_FALSE;
  // Full-screen displays have no chrome regardless; only retitle the windowed
  // ones (the preference is still persisted and applied when they return).
  if (app->pfdWindow != nullptr && !app->pfdFullscreen) {
    glfwSetWindowAttrib(app->pfdWindow, GLFW_DECORATED, decorated);
  }
  if (app->mfdWindow != nullptr && !app->mfdFullscreen) {
    glfwSetWindowAttrib(app->mfdWindow, GLFW_DECORATED, decorated);
  }
  app->settings.showWindowChrome = showChrome;
  avionics::SaveAppSettings(app->settings);
}

// P-key action: floats/unfloats both windows above other windows and remembers
// the choice across runs.
void OnMenuToggleAlwaysOnTop(void* context, bool alwaysOnTop) {
  auto* app = static_cast<AppState*>(context);
  const int floating = alwaysOnTop ? GLFW_TRUE : GLFW_FALSE;
  // Full-screen displays stay floating (always on top) regardless, so the P
  // toggle only affects the windowed ones.
  if (app->pfdWindow != nullptr && !app->pfdFullscreen) {
    glfwSetWindowAttrib(app->pfdWindow, GLFW_FLOATING, floating);
  }
  if (app->mfdWindow != nullptr && !app->mfdFullscreen) {
    glfwSetWindowAttrib(app->mfdWindow, GLFW_FLOATING, floating);
  }
  app->settings.alwaysOnTop = alwaysOnTop;
  avionics::SaveAppSettings(app->settings);
}

// Defined alongside the other window helpers below; declared here because the
// F-key handler uses it.
void SetDisplayFullscreen(GLFWwindow* window, bool fullscreen, bool& isFull,
                          int& restoreX, int& restoreY, int& restoreW,
                          int& restoreH, int monitorIndex, bool showChrome,
                          bool windowedFloating);

// F-key action: toggles borderless full screen for the whole suite. Each
// display takes over its pinned monitor (settings.pfd/mfdMonitor) or, when
// unpinned, whichever monitor it is currently on. If either display is full
// screen, the key returns both to their windowed placement. The choice is
// remembered across runs.
void OnToggleFullscreen(AppState* app) {
  const bool target = !(app->pfdFullscreen || app->mfdFullscreen);
  SetDisplayFullscreen(app->pfdWindow, target, app->pfdFullscreen,
                       app->pfdRestoreX, app->pfdRestoreY, app->pfdRestoreW,
                       app->pfdRestoreH, app->settings.pfdMonitor,
                       app->settings.showWindowChrome, app->settings.alwaysOnTop);
  SetDisplayFullscreen(app->mfdWindow, target, app->mfdFullscreen,
                       app->mfdRestoreX, app->mfdRestoreY, app->mfdRestoreW,
                       app->mfdRestoreH, app->settings.mfdMonitor,
                       app->settings.showWindowChrome, app->settings.alwaysOnTop);
  app->settings.pfdFullscreen = app->pfdFullscreen;
  app->settings.mfdFullscreen = app->mfdFullscreen;
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
    // Update before each render so map pan scroll and land-data queries stay
    // aligned with the projected view center (viewport sync runs during render).
    eng.update(1.0 / 60.0);
    RenderSuite(renderer, eng, fbWidth, fbHeight, showBezel);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  eng.update(1.0 / 60.0);
  RenderSuite(renderer, eng, fbWidth, fbHeight, showBezel);
}

// Block until DSF tiles cover the terrain raster footprint (rangeNm * 2.4) so
// wide-map topo screenshots and the first async rebuild sample real elevation.
void WarmTerrainTilesForRange(avionics::DsfTerrainStore& terrain,
                              double centerLat, double centerLon,
                              float rangeNm) {
  if (!terrain.ready()) return;
  constexpr float kCoverageRangeFactor = 2.4f;
  constexpr float kTerrainCornerRangeFactor = 3.5f;
  constexpr float kFullDetailTerrainMaxNm = 200.0f;
  constexpr double kNmPerDegLat = 60.0;
  const double halfNm =
      std::max(static_cast<double>(rangeNm) * kCoverageRangeFactor,
               static_cast<double>(rangeNm) * kTerrainCornerRangeFactor);
  const double nmLon =
      kNmPerDegLat * std::cos(centerLat * 3.14159265358979323846 / 180.0);
  const bool coarse = rangeNm > kFullDetailTerrainMaxNm;
  terrain.setBulkTerrainSample(true);
  terrain.setCoarseTerrainSample(coarse);
  terrain.setTerrainViewCenter(centerLat, centerLon,
                               coarse ? rangeNm * 0.25f
                                      : static_cast<float>(halfNm));
  terrain.ensureCoverage(centerLat - halfNm / kNmPerDegLat,
                         centerLat + halfNm / kNmPerDegLat,
                         centerLon - halfNm / nmLon,
                         centerLon + halfNm / nmLon,
                         /*waitForTiles=*/true);
  terrain.setBulkTerrainSample(false);
  terrain.setCoarseTerrainSample(false);
}

// Renders a deterministic frame offscreen and writes it to a binary PPM
// (P6). PPM keeps this dependency-free; convert to PNG with `sips` afterwards.
// Returns 0 on success. The mock state is advanced by `seconds` so we can pick a
// clean, representative attitude (seconds = 0 is wings-level, no turbulence).
// When `fmsPlan` is non-null the mock flies that .fms route (same selector
// rules as the interactive --fms-plan flag) instead of its built-in demo route.
int RunScreenshot(const char* path, double seconds, const char* state,
                  const char* fmsPlan, bool showBezel,
                  const char* eisSelector, const char* checklistSelector) {
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
  avionics::render::ensureGlLoaded();

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
  avionics::LandDataStore landData(
      avionics::assets::resolve("land_data.bin", kLandDataAssetPath));
  avionics::ObstacleStore obstacles(resolveObstacleDatabasePath(nullptr));
  avionics::ProcedureStore procedures(navData, &aptData);
  avionics::ShellNavMapData navMapData(navData, airspace, airways, aptData,
                                       landData, procedures, &obstacles);
  avionics::DsfTerrainStore terrain;
  avionics::ChecklistStore checklists(checklistSelector ? checklistSelector
                                                        : "");
  avionics::EisStore eisStore(eisSelector ? eisSelector : "");
  dataSource.setNavFeatureSource(&navMapData);
  dataSource.setTerrainSource(&terrain);
  dataSource.setChecklistSource(&checklists);
  dataSource.setEisSource(&eisStore);
  // Wait for the database loaders so the captured frame reflects the real data
  // (the airspace file in particular is large and parses on its own thread).
  for (int i = 0; i < 2000 && !(navData.ready() && airspace.loaded() &&
                                airways.loaded() && landData.loaded() &&
                                aptData.loaded() && obstacles.loaded());
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
  engine.softkeyController().setNavFeatureSource(&navMapData);

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
  //   boot   - MFD power-up page (database review + ENT prompt)
  //   bootpfd - PFD initialization (red-X instruments + AHRS align)
  //   failed - the connection-lost display (link down: instruments red-X'd,
  //            chrome readouts dashed)
  if (state != nullptr && std::strcmp(state, "bootlogo") == 0) {
    // The initial Garmin logo splash (phase 1 of power-up), fully faded up.
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(
        renderer, avionics::BootScreen::Target::Mfd,
        avionics::BootScreen::Phase::Logo, dataSource.snapshot(),
        dataSource.mapSnapshot(), engine.softkeyController(),
        engine.mfdController(), dataSource.mapSnapshot().navDatabase, false,
        dataSource.aircraftIcaoType(), dataSource.aircraftAcfRelativePath(),
        dataSource.checklistSourcePath(), 1.0f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "bootlogofade") == 0) {
    // Mid fade-up: the Garmin logo at ~40% opacity as it rises from black.
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(
        renderer, avionics::BootScreen::Target::Mfd,
        avionics::BootScreen::Phase::Logo, dataSource.snapshot(),
        dataSource.mapSnapshot(), engine.softkeyController(),
        engine.mfdController(), dataSource.mapSnapshot().navDatabase, false,
        dataSource.aircraftIcaoType(), dataSource.aircraftAcfRelativePath(),
        dataSource.checklistSourcePath(), 0.4f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "bootfade") == 0) {
    // Mid cross-fade: MFD Power-up Page at ~50% opacity (1s into the 2s fade).
    dataSource.update(0.0);
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(
        renderer, avionics::BootScreen::Target::Mfd,
        avionics::BootScreen::Phase::PowerUp, dataSource.snapshot(),
        dataSource.mapSnapshot(), engine.softkeyController(),
        engine.mfdController(), dataSource.mapSnapshot().navDatabase, false,
        dataSource.aircraftIcaoType(), dataSource.aircraftAcfRelativePath(),
        dataSource.checklistSourcePath(), 0.5f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "boot") == 0) {
    // MFD Power-up Page (phase 2). Pump the source once so the Navigation row
    // reflects the loaded nav database, and show the ENT acknowledgement prompt.
    dataSource.update(0.0);
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(
        renderer, avionics::BootScreen::Target::Mfd,
        avionics::BootScreen::Phase::PowerUp, dataSource.snapshot(),
        dataSource.mapSnapshot(), engine.softkeyController(),
        engine.mfdController(), dataSource.mapSnapshot().navDatabase, true,
        dataSource.aircraftIcaoType(), dataSource.aircraftAcfRelativePath(),
        dataSource.checklistSourcePath(), 1.0f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "bootpfd") == 0) {
    // PFD initialization (Figure 1-7): instruments red-X'd, AHRS align message.
    dataSource.update(0.0);
    renderer.beginFrame(fbWidth, fbHeight, 1.0f);
    avionics::BootScreen::render(
        renderer, avionics::BootScreen::Target::Pfd,
        avionics::BootScreen::Phase::PowerUp, dataSource.snapshot(),
        dataSource.mapSnapshot(), engine.softkeyController(),
        engine.mfdController(), dataSource.mapSnapshot().navDatabase, false,
        dataSource.aircraftIcaoType(), dataSource.aircraftAcfRelativePath(),
        dataSource.checklistSourcePath(), 1.0f, fbWidth, fbHeight);
    renderer.endFrame();
  } else if (state != nullptr && std::strcmp(state, "alerts") == 0) {
    // Drive the real interaction path: bring up the live page, press the
    // Alerts softkey, then run the open animation to completion before capture.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(11);  // Alerts key
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "alertsfilled") == 0) {
    // Alerts window with multiple wrapped entries (reversionary failures + an
    // advisory message), matching the trainer's filled PFD Alerts screenshot.
    dataSource.setReversionaryAlertsDemo(true);
    avionics::setUpdateAdvisory(
        "DATABASE UPDATE - A new navigation database is available");
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
    engine.pressSoftkey(3);  // "DME" toggle on
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "tmrref") == 0) {
    // Timer/References window: open it with the timer stopped (00:00:00 /
    // "Start?", as in Pilot's Guide Fig. 2-6), then set BARO minimums so the
    // BARO MIN box and tape bug are captured.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(9);  // "Tmr/Ref" -> open the References window
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
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
  } else if (state != nullptr && std::strcmp(state, "comvol") == 0) {
    // COM VOL/SQ knob turned: the COM audio level replaces the COM standby
    // frequency as a cyan percentage + white "VOL" (Pilot's Guide Fig. 4-3).
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::ComVolCcw);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "navvol") == 0) {
    // NAV VOL/ID knob turned: the NAV audio level replaces the NAV standby
    // frequency as a white "VOL" + cyan percentage (Pilot's Guide Fig. 4-8).
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::NavVolCcw);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "comdecode") == 0) {
    // Decoded COM station identifier: tune COM1 active to a real nearby
    // airport's published comm frequency so the green decode line (e.g.
    // "KFMY TOWER") shows centered beneath the COM box.
    engine.skipBoot();
    engine.update(seconds);
    const avionics::MapData& comMap = dataSource.mapSnapshot();
    for (const avionics::MapFeature& f : comMap.features) {
      if (f.type != avionics::MapFeatureType::Airport) continue;
      bool tuned = false;
      for (const avionics::MapAirportFrequency& fr :
           navMapData.airportFrequencies(f.id)) {
        if (fr.mhz > 0.0f &&
            avionics::airportCommServiceLabel(fr.service) != nullptr) {
          dataSource.tuneRadioStandby(avionics::RadioUnit::Com1, fr.mhz);
          dataSource.transferRadio(avionics::RadioUnit::Com1);
          tuned = true;
          break;
        }
      }
      if (tuned) break;
    }
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "radioann") == 0) {
    // Selected NAV Morse ident audio on: a white "ID" replaces the transfer
    // carets by the active NAV (Pilot's Guide Fig. 4-8). The NAV VOL/ID push
    // queues the audio_selection_nav write, which the mock applies here. (The
    // COM VOL/SQ push is inert -- X-Plane has no squelch dataref.)
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::NavVolPush);  // NAV ident on -> ID
    avionics::RadioUnit identUnit;
    bool identOn = false;
    if (engine.softkeyController().consumeNavIdent(identUnit, identOn)) {
      dataSource.setNavIdent(identUnit, identOn);
    }
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "map") == 0) {
    // The PFD with its inset map (on by default) and the Map/HSI submenu open,
    // showing the Detail / Traffic / Topo / Rel Ter map option keys.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(1);  // "Map/HSI" -> open submenu
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "hsimap") == 0) {
    // The PFD with the HSI Map layout selected (Map/HSI > Layout > HSI Map):
    // the moving map fills the compass rose. Topo is turned off first so the
    // map draws over a black background instead of the topographic shading.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(1);  // "Map/HSI" -> open submenu
    for (int i = 0; i < 5; ++i) engine.update(1.0 / 60.0);
    engine.pressSoftkey(4);  // "Topo" -> off (black map background)
    for (int i = 0; i < 5; ++i) engine.update(1.0 / 60.0);
    engine.pressSoftkey(1);  // "Layout" -> open layout radio
    for (int i = 0; i < 5; ++i) engine.update(1.0 / 60.0);
    engine.pressSoftkey(3);  // "HSI Map"
    for (int i = 0; i < 60; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdmenu") == 0) {
    // The PFD Setup Menu (MENU bezel key, Pilot's Guide Fig. 1-18): MENU opens
    // the backlighting popout; the cursor opens on 'Auto' next to PFD Display.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Menu);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfddto") == 0) {
    // PFD Direct-To empty window (trainer PFD Direct To.bmp): opens blank for
    // ident entry.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::DirectTo);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfddtofilled") == 0) {
    // PFD Direct-To with KRSW matched (trainer PFD Direct To Filled Out.bmp).
    engine.skipBoot();
    engine.update(seconds);
    engine.softkeyController().openDirectToWindow("KRSW");
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfddtoarmed") == 0) {
    // PFD Direct-To with the waypoint confirmed and Activate? armed (first ENT).
    engine.skipBoot();
    engine.update(seconds);
    engine.softkeyController().openDirectToWindow("KRSW");
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::Ent);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdpagemenu") == 0) {
    // PFD Page Menu on Nearest (Pilot's Guide Fig. 1-10): NRST shows "No Options".
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(10);  // Nearest
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::Menu);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdpagemenuref") == 0) {
    // PFD Page Menu on Tmr/Ref: references bulk On/Off/Restore options.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressSoftkey(9);  // Tmr/Ref
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::Menu);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdfpl") == 0) {
    // The PFD Active Flight Plan window (FPL bezel key, Pilot's Guide Fig. 5-48):
    // opens the active flight-plan legs as the lower-right popout.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Fpl);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdfpledit") == 0) {
    // The PFD Active Flight Plan window mid-edit: turn the FMS cursor on, step
    // to a leg row, and open the waypoint-ident entry so the insert/change flow
    // (how the origin and destination are edited) is captured.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Fpl);
    for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::FmsPush);     // cursor on
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // step to second row
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // open ident entry
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // spell a character
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdproc") == 0) {
    // The PFD Procedures window (PROC bezel key, Pilot's Guide 5.8): the
    // top-level Procedures menu.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Proc);
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "pfdprocsel") == 0) {
    // The PFD Procedures window selection sub-window: open PROC, then ENT on
    // "Select Approach" to show the approach list for the plan's airport.
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::Proc);
    for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    engine.pressBezelKey(avionics::BezelKey::Ent);  // "Select Approach"
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfd") == 0) {
    // The MFD full-screen MAP page (its own window in normal operation).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdnrst") == 0) {
    // The MFD NRST - Nearest Airports page (Pilot's Guide Fig. 5-30): select
    // the NRST page group (first page is Nearest Airports), step the cursor to
    // the second airport so the white selection arrow is captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // NRST
    // Let the 3 s page-select popup fade so the Approaches box is captured.
    for (int i = 0; i < 260; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdsimbrief") == 0) {
    // The AUX - SIMBRIEF page with a Pilot ID entry in progress: step to the
    // page (fourth AUX page), open the ID digit-entry softkeys, and type two
    // digits so the cyan edit plate and the digit bar are captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
    engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);  // AUX
    for (int p = 0; p < 5; ++p)
      engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // -> SimBrief
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
    engine.pressSoftkey(2);  // "Map Opt" -> open submenu
    engine.pressSoftkey(0);  // "Traffic" on
    engine.pressSoftkey(4);  // "AWY" Off -> On
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdawy") == 0) {
    // The MAP page zoomed to 50 NM with low/high airways enabled (AWY On),
    // then back on the root bar so the view matches the standard MAP capture
    // with the airway network drawn.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 3; ++p) {  // RNG+ -> 50 NM
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
    engine.pressSoftkey(2);   // "Map Opt" -> open submenu
    engine.pressSoftkey(4);   // "AWY" Off -> On
    engine.pressSoftkey(10);  // "Back" -> root bar
    for (int i = 0; i < 60; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdwx") == 0) {
    // The MAP page with the datalink NEXRAD overlay enabled, zoomed out so the
    // mock precipitation (a 360-degree field centered on the aircraft) fills the
    // map. Zoom first on the root bar (RNG+ steps the range ladder), then open
    // Map Opt and toggle NEXRAD, leaving the submenu open so the highlighted key
    // is captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 4; ++p) {
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
    engine.pressSoftkey(2);  // "Map Opt" -> open submenu
    // TER cycles Off -> Topo -> Rel -> Off; press three times back to Off for a
    // clean background under the weather overlay.
    engine.pressSoftkey(3);  // TER Off -> Topo
    engine.pressSoftkey(3);  // TER Topo -> Rel
    engine.pressSoftkey(3);  // TER Rel -> Off (clean background)
    engine.pressSoftkey(6);  // "NEXRAD" on
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
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // Traffic Map
    engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);  // Weather Radar
    engine.pressSoftkey(3);  // "Mode" -> submenu
    engine.pressSoftkey(4);  // "Weather"
    for (int i = 0; i < 4; ++i)
      engine.pressBezelKey(avionics::BezelKey::FmsInnerCcw);  // tilt DN 1.00
    engine.pressSoftkey(10);  // "BRG" line on
    for (int p = 0; p < 5; ++p) {
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
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
  } else if (state != nullptr && std::strcmp(state, "mfdmenu") == 0) {
    // The Navigation Map Page Menu (MENU bezel key, Pilot's Guide Fig. 5-6):
    // open the page menu on the default MAP page so the option list and the
    // highlighted (live) Declutter option are captured.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int i = 0; i < 200; ++i) engine.update(1.0 / 60.0);  // popup fades out
    engine.pressBezelKey(avionics::BezelKey::Menu);
    // Let the open slide+fade finish (kWindowAnimSeconds) so the capture shows
    // the menu fully in place rather than mid-animation.
    for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdmapset") == 0) {
    // The Map Settings window (Pilot's Guide Fig. 5-7): open the Navigation Map
    // Page Menu, ENT on the highlighted 'Map Settings' option to open the
    // window (cursor on the Group selector), then let the slide+fade finish.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int i = 0; i < 200; ++i) engine.update(1.0 / 60.0);  // popup fades out
    // Fig. 5-7 shows Terrain Display = Topo; cycle once from the suite default
    // (Off) via the Map Opt softkey submenu.
    engine.pressSoftkey(2);  // Map Opt
    engine.pressSoftkey(3);  // Terrain -> Topo
    engine.pressBezelKey(avionics::BezelKey::Menu);
    engine.pressBezelKey(avionics::BezelKey::Ent);  // open Map Settings
    for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr &&
             (std::strncmp(state, "mfdmap1000pann", 14) == 0 ||
              std::strncmp(state, "mfdmap1000pans", 14) == 0 ||
              std::strncmp(state, "mfdmap1000pane", 14) == 0)) {
    // MFD MAP at 1000 NM with the map pointer panned north, south, or east
    // (range locked). Optional trailing digits set pan steps (default 8); e.g.
    // mfdmap1000pann18 pans ~1800 NM north toward Hudson Bay.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 10; ++p) {
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
    engine.pressBezelKey(avionics::BezelKey::PanPush);
    const bool panNorth = std::strncmp(state, "mfdmap1000pann", 14) == 0;
    const bool panEast = std::strncmp(state, "mfdmap1000pane", 14) == 0;
    const avionics::BezelKey panKey =
        panNorth ? avionics::BezelKey::PanUp
        : panEast ? avionics::BezelKey::PanRight
                  : avionics::BezelKey::PanDown;
    const char* panDigits = state + 14;
    const int panSteps =
        *panDigits != '\0' ? std::max(1, std::atoi(panDigits)) : 8;
    for (int p = 0; p < panSteps; ++p) {
      engine.pressBezelKey(panKey);
      for (int i = 0; i < 10; ++i) engine.update(1.0 / 60.0);
    }
    WarmTerrainTilesForRange(terrain, 26.5862, -81.7552, 1000.0f);
    engine.pressSoftkey(2);   // Map Opt
    engine.pressSoftkey(3);   // TER Off -> Topo
    engine.pressSoftkey(10);  // Back
    for (int i = 0; i < 240; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdterrain") == 0) {
    // MFD MAP at 25 NM with TER Topo (trainer MFD Terrain Colors.bmp).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 2; ++p) {
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
    WarmTerrainTilesForRange(terrain, 26.5862, -81.7552, 25.0f);
    engine.pressSoftkey(2);   // Map Opt
    engine.pressSoftkey(3);   // TER Off -> Topo
    engine.pressSoftkey(10);  // Back
    for (int i = 0; i < 180; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdtopo10") == 0) {
    // MFD MAP at 10 NM (default range) with TER Topo.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    WarmTerrainTilesForRange(terrain, 26.5862, -81.7552, 10.0f);
    engine.pressSoftkey(2);   // Map Opt
    engine.pressSoftkey(3);   // TER Off -> Topo
    engine.pressSoftkey(10);  // Back
    for (int i = 0; i < 180; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "mfdtopo50") == 0) {
    // MFD MAP at 50 NM with TER Topo (wide view; open ocean must stay navy).
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    for (int p = 0; p < 3; ++p) {
      engine.pressBezelKey(avionics::BezelKey::RangeUp);
      for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
    }
    WarmTerrainTilesForRange(terrain, 26.5862, -81.7552, 50.0f);
    engine.pressSoftkey(2);   // Map Opt
    engine.pressSoftkey(3);   // TER Off -> Topo
    engine.pressSoftkey(10);  // Back
    for (int i = 0; i < 240; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strncmp(state, "mfd", 3) == 0) {
    // MFD page screenshots. Page groups are selected with the large FMS knob;
    // pages within a group use the small FMS knob. "mfdwpt" = WPT page 1,
    // "mfdwpt3" = WPT page 3. "mfdfpl" presses the FPL bezel key, and
    // "mfdtrk" sets track-up on the MAP page. "mfdrng<N>" zooms the MAP page
    // out N range steps (RangeUp) to capture wide/continental views.
    engine.setPage(avionics::DisplayPage::MultiFunctionDisplay);
    engine.skipBoot();
    engine.update(seconds);
    const char* suffix = state + 3;
    if (std::strncmp(suffix, "wpt", 3) == 0) {
      engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
      const char* digits = suffix + 3;
      const int pageSteps =
          *digits != '\0' ? std::max(0, std::atoi(digits) - 1) : 0;
      for (int p = 0; p < pageSteps; ++p)
        engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);
    } else if (std::strncmp(suffix, "aux", 3) == 0) {
      engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
      engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
      const char* digits = suffix + 3;
      const int pageSteps =
          *digits != '\0' ? std::max(0, std::atoi(digits) - 1) : 0;
      for (int p = 0; p < pageSteps; ++p)
        engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);
    } else if (std::strncmp(suffix, "nrst", 4) == 0) {
      for (int i = 0; i < 3; ++i)
        engine.pressBezelKey(avionics::BezelKey::FmsOuterCw);
      const char* digits = suffix + 4;
      const int pageSteps =
          *digits != '\0' ? std::max(0, std::atoi(digits) - 1) : 0;
      for (int p = 0; p < pageSteps; ++p)
        engine.pressBezelKey(avionics::BezelKey::FmsInnerCw);
    } else if (std::strncmp(suffix, "trk", 3) == 0) {
      avionics::MfdPersistentState mfdState;
      avionics::captureMfdState(engine.mfdController(), mfdState);
      mfdState.orientation = avionics::MapOrientation::TrackUp;
      avionics::applyMfdState(engine.mfdController(), mfdState);
    } else if (std::strncmp(suffix, "chklist", 7) == 0) {
      engine.pressSoftkey(11);  // Checklist page group
    } else if (std::strncmp(suffix, "rngdn", 5) == 0) {
      const char* digits = suffix + 5;
      const int presses = *digits != '\0' ? std::atoi(digits) : 1;
      for (int p = 0; p < presses; ++p) {
        engine.pressBezelKey(avionics::BezelKey::RangeDown);
        for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
      }
    } else if (std::strncmp(suffix, "rng", 3) == 0) {
      const char* digits = suffix + 3;
      const int presses = *digits != '\0' ? std::atoi(digits) : 1;
      for (int p = 0; p < presses; ++p) {
        engine.pressBezelKey(avionics::BezelKey::RangeUp);
        for (int i = 0; i < 20; ++i) engine.update(1.0 / 60.0);
      }
    } else if (std::strncmp(suffix, "pan", 3) == 0) {
      // MAP page with the pan pointer active and nudged north N times
      // (mfdpan3 = three PanUp steps after PanPush).
      const char* digits = suffix + 3;
      const int presses = *digits != '\0' ? std::atoi(digits) : 3;
      engine.pressBezelKey(avionics::BezelKey::PanPush);
      for (int p = 0; p < presses; ++p) {
        engine.pressBezelKey(avionics::BezelKey::PanUp);
        for (int i = 0; i < 10; ++i) engine.update(1.0 / 60.0);
      }
    } else if (std::strncmp(suffix, "fpl", 3) == 0) {
      engine.pressBezelKey(avionics::BezelKey::Fpl);
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
  } else if (state != nullptr && std::strcmp(state, "ground") == 0) {
    // The PFD parked on the ground at KFMY runway 31 with the engine idling
    // (mock on-ground mode): stationary, wings level, at field elevation.
    dataSource.setGroundMode(true);
    engine.skipBoot();
    engine.update(seconds);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
  } else if (state != nullptr && std::strcmp(state, "reversionary") == 0) {
    // Display-backup (reversionary) mode: manual backup active with avionics on
    // so both GDUs stay lit (PFD adds the EIS strip; MFD shows PFD instruments).
    engine.skipBoot();
    engine.update(seconds);
    dataSource.toggleDisplayBackup();
    for (int i = 0; i < 30; ++i) engine.update(1.0 / 60.0);
    RenderSuiteSettled(renderer, engine, fbWidth, fbHeight, showBezel);
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
           int mods) {
  if (action != GLFW_PRESS) return;
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (key == GLFW_KEY_ESCAPE) {
    // While the built-in demo feed is active, Esc returns to the live feed
    // rather than quitting (this is what the demo CAS banner tells the pilot);
    // a second press, now live, quits.
    if (app != nullptr && app->demoMode) {
      ToggleDemoSource(*app);
      return;
    }
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }
  // ENT acknowledges the power-up page on the focused display only (same as the
  // bezel ENT key on that GDU).
  if (key == GLFW_KEY_ENTER || key == GLFW_KEY_KP_ENTER) {
    if (app != nullptr) {
      avionics::AvionicsEngine* bootEngine =
          BootAckEngineForWindow(*app, window);
      if (bootEngine != nullptr && bootEngine->awaitingPowerUpAck()) {
        bootEngine->acknowledgePowerUp();
      } else {
        avionics::AvionicsEngine* owner = app->pfdEngine;
        if (owner != nullptr &&
            owner->softkeyController().pfdClaimsFmsInput()) {
          owner->pressBezelKey(avionics::BezelKey::Ent);
        } else if (app->mfdEngine != nullptr) {
          app->mfdEngine->pressBezelKey(avionics::BezelKey::Ent);
        } else if (owner != nullptr) {
          owner->pressBezelKey(avionics::BezelKey::Ent);
        }
      }
    }
    return;
  }
  if (app == nullptr) return;
  // A-Z / 0-9 / Backspace type into an active waypoint-ident entry (same as
  // the GCU 478 alphanumeric keypad when bridged from the X-Plane plugin).
  {
    char gcuCh = 0;
    if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
      gcuCh = static_cast<char>('A' + (key - GLFW_KEY_A));
    } else if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
      gcuCh = static_cast<char>('0' + (key - GLFW_KEY_0));
    } else if (key >= GLFW_KEY_KP_0 && key <= GLFW_KEY_KP_9) {
      gcuCh = static_cast<char>('0' + (key - GLFW_KEY_KP_0));
    } else if (key == GLFW_KEY_BACKSPACE) {
      gcuCh = '\b';
    }
    if (gcuCh != 0) {
      const auto tryEntry = [&](avionics::AvionicsEngine* eng) {
        return eng != nullptr && eng->applyGcuEntryKey(gcuCh);
      };
      if (app->pfdEngine != nullptr &&
          app->pfdEngine->softkeyController().pfdClaimsFmsInput() &&
          tryEntry(app->pfdEngine)) {
        return;
      }
      if (tryEntry(app->mfdEngine)) return;
      if (tryEntry(app->pfdEngine)) return;
    }
  }
  // Ctrl+Shift+D toggles the built-in demo feed (an awkward chord so it isn't
  // hit by accident). Handled before the single-key view toggles below.
  if (key == GLFW_KEY_D && (mods & GLFW_MOD_CONTROL) != 0 &&
      (mods & GLFW_MOD_SHIFT) != 0) {
    ToggleDemoSource(*app);
    return;
  }
  // Shift+B toggles manual display-backup (reversionary) mode (audio panel key).
  if (key == GLFW_KEY_B && (mods & GLFW_MOD_SHIFT) != 0) {
    ToggleDisplayBackup(*app);
    return;
  }
  // View toggles (persisted). No on-screen menu: these are the standalone's
  // keyboard shortcuts for the bezel strips, the OS window title bar, and
  // keep-on-top. Each affects both windows at once.
  switch (key) {
    case GLFW_KEY_B:
      OnMenuToggleBezel(app, !app->showBezel);
      break;
    case GLFW_KEY_T:
      OnMenuToggleWindowChrome(app, !app->settings.showWindowChrome);
      break;
    case GLFW_KEY_P:
      OnMenuToggleAlwaysOnTop(app, !app->settings.alwaysOnTop);
      break;
    case GLFW_KEY_F:
      OnToggleFullscreen(app);
      break;
    default:
      break;
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
  // Hit-test in the letterboxed content rectangle, not the raw framebuffer, so
  // clicks line up with what is drawn when the display is full screen (the
  // image is centered with black bars). In a windowed display the rectangle is
  // the whole framebuffer, so this is a no-op there.
  const ContentRect cr = ComputeContentRect(fbW, fbH, app->showBezel);
  double fx = cursorX * sx - cr.x;
  double fy = cursorY * sy - cr.y;
  FlipContentCoords(fx, fy, cr.w, cr.h,
                    NeedsDisplayFlip180(window, PinnedMonitorForWindow(*app, window)));
  if (fx < 0.0 || fy < 0.0 || fx >= cr.w || fy >= cr.h) return;  // on the bars

  // Only the physical bezel controls are clickable, like the real unit: the
  // NAV/HDG key column on the left, the COM/RANGE/FMS key column on the right
  // and the softkey row below the screen. Clicks on the screen itself do
  // nothing (it is just glass).
  const int leftPx = LeftBezelStripPx(cr.w);
  const int bezelPx = BezelStripPx(cr.w);
  const int screenW = cr.w - leftPx - bezelPx;
  const int screenRight = leftPx + screenW;
  const int screenH = cr.h - SoftkeyStripPx(cr.h);
  if (fx >= screenRight || fx < leftPx) {
    const avionics::BezelKey key =
        fx >= screenRight
            ? avionics::BezelKeyPanel::hitTest(
                  static_cast<float>(fx), static_cast<float>(fy),
                  static_cast<float>(screenRight), 0.0f,
                  static_cast<float>(cr.w - screenRight),
                  static_cast<float>(cr.h))
            : avionics::BezelKeyPanel::hitTestLeft(
                  static_cast<float>(fx), static_cast<float>(fy), 0.0f, 0.0f,
                  static_cast<float>(leftPx), static_cast<float>(cr.h));
    if (key == avionics::BezelKey::Ent && engine->awaitingPowerUpAck()) {
      engine->acknowledgePowerUp();
    } else if (key != avionics::BezelKey::Count) {
      // A click on a knob ring steps it once: the hit-test resolves the left or
      // right half of the ring to its counter-clockwise / clockwise key, so a
      // click is a clean single detent. The scroll wheel still spins the same
      // knobs continuously (handy for big changes); the center push caps, the
      // RANGE ring, and the molded keys click normally too.
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
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(leftPx), static_cast<float>(screenH),
        static_cast<float>(screenW), static_cast<float>(cr.h - screenH));
    if (key >= 0) engine->pressSoftkey(key);
  }
}

// Builds a circular "rotate" cursor: a near-complete ring broken by a small
// wedge, with a single arrowhead on one open end pointing tangentially in the
// `clockwise` direction of travel (the other end is blunt), so it reads as
// "turn this clockwise" or, mirrored, "turn this counter-clockwise". Shown
// while the pointer is over the RANGE joystick's zoom ring -- the clockwise
// side zooms the map out, the counter-clockwise side zooms it in -- matching
// the rotation affordance X-Plane uses for its turnable knobs. Drawn white with
// a dark outline so it reads on both the dark bezel and lighter backgrounds.
// Returns null on failure; callers then fall back to the left-right cursor.
GLFWcursor* MakeRotateCursor(bool clockwise) {
  constexpr int kSize = 32;
  constexpr float kCx = 15.5f;        // ring center
  constexpr float kCy = 15.5f;
  constexpr float kR = 8.5f;          // ring centerline radius
  constexpr float kRingHalf = 1.6f;   // half the ring thickness
  constexpr float kGapHalf = 0.62f;   // half-angle of the open wedge (rad)
  constexpr float kArrow = 4.6f;      // arrowhead reach past the ring end
  constexpr float kArrowHalfW = 3.6f;  // arrowhead half-width

  const auto clamp01 = [](float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
  };
  const auto segDist = [](float px, float py, float ax, float ay, float bx,
                          float by) {
    const float vx = bx - ax, vy = by - ay;
    const float wx = px - ax, wy = py - ay;
    const float len2 = vx * vx + vy * vy;
    float t = len2 > 0.0f ? (wx * vx + wy * vy) / len2 : 0.0f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    const float dx = px - (ax + t * vx);
    const float dy = py - (ay + t * vy);
    return std::sqrt(dx * dx + dy * dy);
  };
  const auto cross = [](float ax, float ay, float bx, float by) {
    return ax * by - ay * bx;
  };
  // Signed distance to a filled triangle: negative inside, positive outside.
  const auto triDist = [&](float px, float py, float ax, float ay, float bx,
                           float by, float cxv, float cyv) {
    const float d = std::min({segDist(px, py, ax, ay, bx, by),
                              segDist(px, py, bx, by, cxv, cyv),
                              segDist(px, py, cxv, cyv, ax, ay)});
    const float e1 = cross(bx - ax, by - ay, px - ax, py - ay);
    const float e2 = cross(cxv - bx, cyv - by, px - bx, py - by);
    const float e3 = cross(ax - cxv, ay - cyv, px - cxv, py - cyv);
    const bool hasNeg = e1 < 0.0f || e2 < 0.0f || e3 < 0.0f;
    const bool hasPos = e1 > 0.0f || e2 > 0.0f || e3 > 0.0f;
    return (hasNeg && hasPos) ? d : -d;  // mixed signs -> outside
  };

  // The ring ends sit at +/-kGapHalf (the open wedge faces +x). The single
  // arrowhead caps the end that leads in the travel direction (+theta is
  // clockwise on screen, since y points down): the upper-right end for CW, the
  // lower-right end for CCW, pointing into the gap. The two cursors are thus
  // vertical mirrors of each other.
  struct Arrow {
    float tx, ty, b1x, b1y, b2x, b2y;
  };
  const auto makeArrow = [&](float ang, float dirSign) {
    const float ex = kCx + std::cos(ang) * kR;
    const float ey = kCy + std::sin(ang) * kR;
    // Unit tangent at this angle, oriented along the travel direction.
    const float ax = dirSign * -std::sin(ang);
    const float ay = dirSign * std::cos(ang);
    const float px = -ay, py = ax;  // perpendicular (unit)
    const float tipx = ex + ax * kArrow;
    const float tipy = ey + ay * kArrow;
    const float basex = ex - ax * (kArrow * 0.5f);
    const float basey = ey - ay * (kArrow * 0.5f);
    return Arrow{tipx,
                 tipy,
                 basex + px * kArrowHalfW,
                 basey + py * kArrowHalfW,
                 basex - px * kArrowHalfW,
                 basey - py * kArrowHalfW};
  };
  const Arrow arrow = clockwise ? makeArrow(-kGapHalf, 1.0f)
                                : makeArrow(kGapHalf, -1.0f);

  std::vector<unsigned char> pixels(static_cast<size_t>(kSize) * kSize * 4, 0);
  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x) {
      const float px = static_cast<float>(x) + 0.5f;
      const float py = static_cast<float>(y) + 0.5f;

      const float dx = px - kCx;
      const float dy = py - kCy;
      float d = std::fabs(std::sqrt(dx * dx + dy * dy) - kR) - kRingHalf;
      if (std::fabs(std::atan2(dy, dx)) < kGapHalf) d = 1.0e9f;  // open wedge
      d = std::min(d, triDist(px, py, arrow.tx, arrow.ty, arrow.b1x, arrow.b1y,
                              arrow.b2x, arrow.b2y));

      const float inkA = clamp01(0.5f - d);            // white shape
      const float outA = clamp01(0.5f - (d - 1.3f));   // dark outline, expanded
      const float whiteA = inkA;
      const float blackA = outA * (1.0f - inkA);
      const float a = whiteA + blackA;
      const unsigned char lum = static_cast<unsigned char>(
          (a > 0.0f ? whiteA / a : 0.0f) * 255.0f + 0.5f);
      const size_t i = (static_cast<size_t>(y) * kSize + x) * 4;
      pixels[i + 0] = lum;
      pixels[i + 1] = lum;
      pixels[i + 2] = lum;
      pixels[i + 3] = static_cast<unsigned char>(clamp01(a) * 255.0f + 0.5f);
    }
  }

  GLFWimage image;
  image.width = kSize;
  image.height = kSize;
  image.pixels = pixels.data();
  return glfwCreateCursor(&image, kSize / 2, kSize / 2);
}

// Hover feedback over the on-screen bezel: a left-right cursor over the FMS
// knob rings, a circular rotate cursor over the RANGE joystick's zoom ring
// (which turns to zoom the map in/out), a hand over the other clickable keys
// and softkeys, and the default arrow over the glass screen. Uses the same
// hit-test and pixel scaling as OnMouseButton so the cursor matches what a
// click does.
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
  const ContentRect cr = ComputeContentRect(fbW, fbH, app->showBezel);
  double fx = cursorX * sx - cr.x;
  double fy = cursorY * sy - cr.y;
  FlipContentCoords(fx, fy, cr.w, cr.h,
                    NeedsDisplayFlip180(window, PinnedMonitorForWindow(*app, window)));
  if (fx < 0.0 || fy < 0.0 || fx >= cr.w || fy >= cr.h) {
    glfwSetCursor(window, nullptr);  // over the letterbox bars
    return;
  }

  const int leftPx = LeftBezelStripPx(cr.w);
  const int bezelPx = BezelStripPx(cr.w);
  const int screenW = cr.w - leftPx - bezelPx;
  const int screenRight = leftPx + screenW;
  const int screenH = cr.h - SoftkeyStripPx(cr.h);

  GLFWcursor* cursor = nullptr;  // default arrow over the glass screen
  if (fx >= screenRight || fx < leftPx) {
    const avionics::BezelKey key =
        fx >= screenRight
            ? avionics::BezelKeyPanel::hitTest(
                  static_cast<float>(fx), static_cast<float>(fy),
                  static_cast<float>(screenRight), 0.0f,
                  static_cast<float>(cr.w - screenRight),
                  static_cast<float>(cr.h))
            : avionics::BezelKeyPanel::hitTestLeft(
                  static_cast<float>(fx), static_cast<float>(fy), 0.0f, 0.0f,
                  static_cast<float>(leftPx), static_cast<float>(cr.h));
    if (key == avionics::BezelKey::RangeDown) {
      // Counter-clockwise side of the zoom ring (zoom in).
      cursor = app->rangeCcwCursor ? app->rangeCcwCursor : app->rotateCursor;
    } else if (key == avionics::BezelKey::RangeUp) {
      // Clockwise side of the zoom ring (zoom out).
      cursor = app->rangeCwCursor ? app->rangeCwCursor : app->rotateCursor;
    } else if (avionics::rotatedBezelKey(key, /*clockwise=*/true) !=
               avionics::BezelKey::Count) {
      // A turnable knob ring (FMS / COM / NAV / VOL / CRS-BARO / HDG): show the
      // same circular rotate cursor as the RANGE ring -- the clockwise variant
      // on the right half, the counter-clockwise variant on the left. The knob
      // is turned with the scroll wheel.
      const bool cw = avionics::rotatedBezelKey(key, true) == key;
      cursor = cw
                   ? (app->rangeCwCursor ? app->rangeCwCursor : app->rotateCursor)
                   : (app->rangeCcwCursor ? app->rangeCcwCursor
                                          : app->rotateCursor);
    } else if (key != avionics::BezelKey::Count) {
      cursor = app->handCursor;
    }
  } else if (fy >= screenH) {
    const int key = avionics::SoftkeyBezelPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(leftPx), static_cast<float>(screenH),
        static_cast<float>(screenW), static_cast<float>(cr.h - screenH));
    if (key >= 0) cursor = app->handCursor;
  }
  glfwSetCursor(window, cursor);
}

// Scroll-wheel over the on-screen RANGE joystick zooms the map, the way you'd
// spin the real knob (which is what the rotate hover cursor advertises): wheel
// up zooms in (range down), wheel down zooms out (range up). Reliable where a
// click is fiddly -- it doesn't depend on which window has focus, and you can
// rest the pointer anywhere on the knob. Uses the same hit-test and pixel
// scaling as the click/hover handlers so it triggers exactly where the rotate
// cursor shows. The delta is accumulated so a trackpad's many small ticks step
// the range ladder one stop per whole notch.
void OnScroll(GLFWwindow* window, double /*xoffset*/, double yoffset) {
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app == nullptr || !app->showBezel || yoffset == 0.0) return;

  avionics::AvionicsEngine* engine =
      (window == app->mfdWindow) ? app->mfdEngine : app->pfdEngine;
  if (engine == nullptr) return;

  double cursorX = 0.0, cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);
  int winW = 0, winH = 0, fbW = 0, fbH = 0;
  glfwGetWindowSize(window, &winW, &winH);
  glfwGetFramebufferSize(window, &fbW, &fbH);
  const double sx = winW > 0 ? static_cast<double>(fbW) / winW : 1.0;
  const double sy = winH > 0 ? static_cast<double>(fbH) / winH : 1.0;
  const ContentRect cr = ComputeContentRect(fbW, fbH, app->showBezel);
  double fx = cursorX * sx - cr.x;
  double fy = cursorY * sy - cr.y;
  FlipContentCoords(fx, fy, cr.w, cr.h,
                    NeedsDisplayFlip180(window, PinnedMonitorForWindow(*app, window)));
  if (fx < 0.0 || fy < 0.0 || fx >= cr.w || fy >= cr.h) {
    app->rangeScrollAccum = 0.0;
    return;
  }

  const int leftPx = LeftBezelStripPx(cr.w);
  const int bezelPx = BezelStripPx(cr.w);
  const int screenW = cr.w - leftPx - bezelPx;
  const int screenRight = leftPx + screenW;

  // Which bezel control is under the pointer (right strip carries the COM /
  // CRS-BARO / RANGE / FMS cluster; the left strip the NAV / HDG knobs).
  avionics::BezelKey key = avionics::BezelKey::Count;
  if (fx >= screenRight) {
    key = avionics::BezelKeyPanel::hitTest(
        static_cast<float>(fx), static_cast<float>(fy),
        static_cast<float>(screenRight), 0.0f,
        static_cast<float>(cr.w - screenRight), static_cast<float>(cr.h));
  } else if (fx < leftPx) {
    key = avionics::BezelKeyPanel::hitTestLeft(
        static_cast<float>(fx), static_cast<float>(fy), 0.0f, 0.0f,
        static_cast<float>(leftPx), static_cast<float>(cr.h));
  }

  // Anywhere on the RANGE joystick cluster (zoom ring or the pan center)
  // scrolls the map range, so the gesture is forgiving about exact placement.
  const int ki = static_cast<int>(key);
  const bool overRange =
      ki >= avionics::kRangeJoyFirst && ki < avionics::kFmsKnobFirst;

  if (overRange) {
    app->rangeScrollAccum += yoffset;
    while (app->rangeScrollAccum >= 1.0) {
      engine->pressBezelKey(avionics::BezelKey::RangeDown);  // wheel up: zoom in
      app->rangeScrollAccum -= 1.0;
    }
    while (app->rangeScrollAccum <= -1.0) {
      engine->pressBezelKey(avionics::BezelKey::RangeUp);  // wheel down: zoom out
      app->rangeScrollAccum += 1.0;
    }
    return;
  }

  // Any other rotatable knob (FMS, COM, NAV, VOL, CRS/BARO, HDG): scrolling
  // spins it, which is far easier than repeatedly clicking the thin ring.
  // Wheel up turns clockwise, wheel down counter-clockwise -- the affordance
  // the rotate hover cursor advertises.
  if (!avionics::isRotatableBezelKey(key)) {
    app->rangeScrollAccum = 0.0;
    return;
  }
  app->rangeScrollAccum += yoffset;
  while (app->rangeScrollAccum >= 1.0) {
    engine->pressBezelKey(avionics::rotatedBezelKey(key, /*clockwise=*/true));
    app->rangeScrollAccum -= 1.0;
  }
  while (app->rangeScrollAccum <= -1.0) {
    engine->pressBezelKey(avionics::rotatedBezelKey(key, /*clockwise=*/false));
    app->rangeScrollAccum += 1.0;
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
// The window starts hidden so it can be positioned (e.g. a restored saved
// placement) before its first appearance; the caller shows it when ready.
GLFWwindow* CreateAvionicsWindow(const char* title, bool alwaysOnTop,
                                 bool decorated, GLFWwindow* share, int width,
                                 int height, GLFWmonitor* monitor) {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  if (monitor != nullptr) {
    // Borderless full screen: an undecorated window sized to the monitor and
    // kept floating (always on top) so it covers the whole screen and never
    // hides when the user clicks another app. (Exclusive full screen would
    // minimize on focus loss and live in its own macOS Space.) The caller
    // positions it at the monitor's top-left corner.
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (mode != nullptr) {
      width = mode->width;
      height = mode->height;
    }
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
    return glfwCreateWindow(width, height, title, nullptr, share);
  }
  glfwWindowHint(GLFW_FLOATING, alwaysOnTop ? GLFW_TRUE : GLFW_FALSE);
  glfwWindowHint(GLFW_DECORATED, decorated ? GLFW_TRUE : GLFW_FALSE);
  return glfwCreateWindow(width, height, title, nullptr, share);
}

// Monitor at a given index into GLFW's list. A negative or out-of-range index
// (e.g. an unplugged monitor) falls back to the primary monitor so a saved
// preference never leaves a display with nowhere to go. Null only when no
// monitors are connected at all.
GLFWmonitor* MonitorByIndex(int index) {
  int count = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&count);
  if (monitors == nullptr || count == 0) return nullptr;
  if (index < 0 || index >= count) return glfwGetPrimaryMonitor();
  return monitors[index];
}

// The monitor a window currently sits on (by its center point), used when the
// user toggles full screen with the keyboard and hasn't pinned a specific
// monitor: the display takes over whichever screen it is already on.
GLFWmonitor* MonitorForWindow(GLFWwindow* window) {
  int wx = 0, wy = 0, ww = 0, wh = 0;
  glfwGetWindowPos(window, &wx, &wy);
  glfwGetWindowSize(window, &ww, &wh);
  const int cx = wx + ww / 2;
  const int cy = wy + wh / 2;
  int count = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&count);
  for (int i = 0; i < count; ++i) {
    int mx = 0, my = 0;
    glfwGetMonitorPos(monitors[i], &mx, &my);
    const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
    if (mode == nullptr) continue;
    if (cx >= mx && cx < mx + mode->width && cy >= my &&
        cy < my + mode->height) {
      return monitors[i];
    }
  }
  return glfwGetPrimaryMonitor();
}

// Switches one display between borderless full screen and its windowed
// placement. Entering saves the current windowed geometry (so it can return);
// leaving restores it and re-applies the window chrome preference. `isFull`
// tracks the current state and is flipped on success.
void SetDisplayFullscreen(GLFWwindow* window, bool fullscreen, bool& isFull,
                          int& restoreX, int& restoreY, int& restoreW,
                          int& restoreH, int monitorIndex, bool showChrome,
                          bool windowedFloating) {
  if (window == nullptr || fullscreen == isFull) return;
  if (fullscreen) {
    glfwGetWindowPos(window, &restoreX, &restoreY);
    glfwGetWindowSize(window, &restoreW, &restoreH);
    GLFWmonitor* monitor =
        monitorIndex >= 0 ? MonitorByIndex(monitorIndex) : MonitorForWindow(window);
    if (monitor == nullptr) return;
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (mode == nullptr) return;
    int mx = 0, my = 0;
    glfwGetMonitorPos(monitor, &mx, &my);
    // Borderless + floating: cover the monitor and stay on top, so the display
    // never disappears when another window is clicked.
    glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_FALSE);
    glfwSetWindowAttrib(window, GLFW_AUTO_ICONIFY, GLFW_FALSE);
    glfwSetWindowAttrib(window, GLFW_FLOATING, GLFW_TRUE);
    glfwSetWindowMonitor(window, nullptr, mx, my, mode->width, mode->height,
                         GLFW_DONT_CARE);
    isFull = true;
  } else {
    glfwSetWindowMonitor(window, nullptr, restoreX, restoreY, restoreW,
                         restoreH, GLFW_DONT_CARE);
    glfwSetWindowAttrib(window, GLFW_DECORATED,
                        showChrome ? GLFW_TRUE : GLFW_FALSE);
    glfwSetWindowAttrib(window, GLFW_FLOATING,
                        windowedFloating ? GLFW_TRUE : GLFW_FALSE);
    isFull = false;
  }
}

// Prints the connected monitors so the user can pick an index for
// --pfd-monitor / --mfd-monitor. The primary monitor is flagged.
void ListMonitors() {
  int count = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&count);
  GLFWmonitor* primary = glfwGetPrimaryMonitor();
  std::fprintf(stderr, "Connected monitors (%d):\n", count);
  for (int i = 0; i < count; ++i) {
    int mx = 0, my = 0;
    glfwGetMonitorPos(monitors[i], &mx, &my);
    const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
    const char* name = glfwGetMonitorName(monitors[i]);
#if defined(_WIN32)
    const char* orientation = MonitorOrientationLabel(
        OrientationForGlfwMonitor(monitors[i]));
#else
    const char* orientation = "landscape";
#endif
    std::fprintf(stderr, "  [%d] %s  %dx%d @ (%d,%d)  %s%s\n", i,
                 name ? name : "(unnamed)", mode ? mode->width : 0,
                 mode ? mode->height : 0, mx, my, orientation,
                 monitors[i] == primary ? "  (primary)" : "");
  }
}

// Any key on an identify overlay dismisses the whole flash early.
void OnIdentifyKey(GLFWwindow* window, int /*key*/, int /*scancode*/,
                   int action, int /*mods*/) {
  if (action == GLFW_PRESS) glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Exits a short-lived utility mode (--list-monitors / --identify-monitors)
// immediately after its work is done. We flush output and call _Exit rather
// than returning, because tearing down GLFW and the process's static/global
// state from these modes aborts during teardown on some platforms (a harmless
// but ugly non-zero exit); these modes own no persistent state worth unwinding.
[[noreturn]] void QuitUtilityMode(int code = 0) {
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(code);
}

// Draws one seven-segment digit filling the box (x, y, w, h) with bar
// thickness t. Used by the identify overlay so the big monitor index renders
// with no font dependency -- the installer launches --identify-monitors from a
// temporary copy of the (font-less) executable alone.
void DrawSevenSegDigit(avionics::NanoVgRenderer& r, float x, float y, float w,
                       float h, float t, int digit, const avionics::Color& c) {
  static const bool seg[10][7] = {
      // a, b, c, d, e, f, g
      {1, 1, 1, 1, 1, 1, 0},  // 0
      {0, 1, 1, 0, 0, 0, 0},  // 1
      {1, 1, 0, 1, 1, 0, 1},  // 2
      {1, 1, 1, 1, 0, 0, 1},  // 3
      {0, 1, 1, 0, 0, 1, 1},  // 4
      {1, 0, 1, 1, 0, 1, 1},  // 5
      {1, 0, 1, 1, 1, 1, 1},  // 6
      {1, 1, 1, 0, 0, 0, 0},  // 7
      {1, 1, 1, 1, 1, 1, 1},  // 8
      {1, 1, 1, 1, 0, 1, 1},  // 9
  };
  if (digit < 0 || digit > 9) return;
  const float rad = t * 0.5f;
  const float hLen = w - t * 1.4f;      // horizontal bar length
  const float hx = x + t * 0.7f;        // horizontal bar left
  const float vLen = (h - t * 3.0f) * 0.5f;  // vertical bar length per half
  const float midY = y + (h - t) * 0.5f;
  const auto hbar = [&](float by) {
    r.fillRoundedRect(hx, by, hLen, t, rad, c);
  };
  const auto vbar = [&](float bx, float by) {
    r.fillRoundedRect(bx, by, t, vLen, rad, c);
  };
  if (seg[digit][0]) hbar(y);                            // a (top)
  if (seg[digit][6]) hbar(midY);                         // g (middle)
  if (seg[digit][3]) hbar(y + h - t);                    // d (bottom)
  if (seg[digit][5]) vbar(x, y + t * 0.7f);              // f (top-left)
  if (seg[digit][1]) vbar(x + w - t, y + t * 0.7f);      // b (top-right)
  if (seg[digit][4]) vbar(x, midY + t * 0.7f);           // e (bottom-left)
  if (seg[digit][2]) vbar(x + w - t, midY + t * 0.7f);   // c (bottom-right)
}

// Draws a non-negative integer centered at (cx, cy) using seven-segment digits
// of the given height, returning nothing. Multi-digit values are laid out
// left-to-right and centered as a group.
void DrawBigNumber(avionics::NanoVgRenderer& r, float cx, float cy, float height,
                   int value, const avionics::Color& c) {
  const std::string digits = std::to_string(value < 0 ? 0 : value);
  const float dw = height * 0.6f;
  const float t = height * 0.16f;
  const float gap = height * 0.22f;
  const float totalW =
      digits.size() * dw + (digits.size() - 1) * gap;
  float x = cx - totalW * 0.5f;
  const float y = cy - height * 0.5f;
  for (char ch : digits) {
    DrawSevenSegDigit(r, x, y, dw, height, t, ch - '0', c);
    x += dw + gap;
  }
}

// Flashes each connected monitor's index large on that physical screen for a
// few seconds, so the user can see which number maps to which monitor before
// choosing one for --pfd-monitor / --mfd-monitor (or in the installer's Display
// Setup page, which launches this via its "Identify" button). One borderless
// window is opened per monitor; the timeout, any key, or a click ends it.
void IdentifyMonitors(double seconds) {
  int count = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&count);
  if (monitors == nullptr || count == 0) return;
  if (seconds <= 0.0) seconds = 4.0;
  GLFWmonitor* primary = glfwGetPrimaryMonitor();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
  glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
  glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

  struct Overlay {
    GLFWwindow* window = nullptr;
    avionics::NanoVgRenderer* renderer = nullptr;
    int index = 0;
    bool primary = false;
    int width = 0;
    int height = 0;
  };
  std::vector<Overlay> overlays;
  for (int i = 0; i < count; ++i) {
    const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
    if (mode == nullptr) continue;
    int mx = 0, my = 0;
    glfwGetMonitorPos(monitors[i], &mx, &my);
    GLFWwindow* window = glfwCreateWindow(mode->width, mode->height,
                                          "Identify Monitor", nullptr, nullptr);
    if (window == nullptr) continue;
    glfwSetWindowPos(window, mx, my);
    glfwSetKeyCallback(window, OnIdentifyKey);
    glfwShowWindow(window);
    glfwFocusWindow(window);  // raise above other apps so the flash is visible
    glfwMakeContextCurrent(window);
    avionics::render::ensureGlLoaded();
    glfwSwapInterval(0);
    auto* renderer = new avionics::NanoVgRenderer();
    if (!renderer->valid()) {
      delete renderer;
      glfwDestroyWindow(window);
      continue;
    }
    overlays.push_back(
        {window, renderer, i, monitors[i] == primary, mode->width, mode->height});
  }
  if (overlays.empty()) return;

  const double endTime = glfwGetTime() + seconds;
  bool done = false;
  while (!done && glfwGetTime() < endTime) {
    glfwPollEvents();
    const int secondsLeft =
        static_cast<int>(endTime - glfwGetTime()) + 1;
    for (Overlay& ov : overlays) {
      if (glfwWindowShouldClose(ov.window) ||
          glfwGetMouseButton(ov.window, GLFW_MOUSE_BUTTON_LEFT) ==
              GLFW_PRESS) {
        done = true;
        break;
      }
      glfwMakeContextCurrent(ov.window);
      int fbW = 0, fbH = 0;
      glfwGetFramebufferSize(ov.window, &fbW, &fbH);
      glViewport(0, 0, fbW, fbH);
      // Deep avionics blue so the white index reads clearly on any display.
      glClearColor(0.04f, 0.18f, 0.42f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
      ov.renderer->beginFrame(fbW, fbH, 1.0f);
      const float cx = fbW * 0.5f;
      const float cy = fbH * 0.5f;
      // The big index is drawn with vector segments (no font), so it renders
      // even when launched from a font-less temporary copy by the installer.
      DrawBigNumber(*ov.renderer, cx, cy, fbH * 0.5f, ov.index,
                    avionics::colors::kWhite);
      // Captions are a font-based nicety; harmlessly skipped when no font is
      // available (e.g. the installer's temp-copy launch).
      std::string caption =
          std::to_string(ov.width) + " x " + std::to_string(ov.height);
      if (ov.primary) caption += "   (primary)";
      ov.renderer->fillText(cx, cy + fbH * 0.34f, caption, fbH * 0.05f,
                            avionics::TextAlign::Center,
                            avionics::colors::kWhite);
      ov.renderer->fillText(cx, fbH * 0.92f,
                            "Monitor index for --pfd-monitor / --mfd-monitor"
                            "   (" + std::to_string(secondsLeft) + ")",
                            fbH * 0.035f, avionics::TextAlign::Center,
                            avionics::colors::kWhite);
      ov.renderer->endFrame();
      glfwSwapBuffers(ov.window);
    }
  }

  for (Overlay& ov : overlays) {
    delete ov.renderer;
    glfwDestroyWindow(ov.window);
  }
}

}  // namespace

int main(int argc, char** argv) {
  // Locate bundled assets relative to the installed binary before anything that
  // loads them (renderer fonts, land data, EIS, checklists), including the
  // offscreen --screenshot path below.
  RegisterAssetSearchDirs();

  // Restore persisted preferences up front: the nav-data directory in
  // particular must be applied before any store is constructed (the offscreen
  // --screenshot path below and the interactive ShellNavMapData both spawn
  // loader threads in their constructors).
  const avionics::AppSettings savedSettings = avionics::LoadAppSettings();

  // Point the nav-data loaders at an explicit install-like tree so a display PC
  // without X-Plane can read a copied nav-data tree. The --nav-data-dir flag
  // wins; otherwise use the saved setting (written by the installer's Display
  // Setup page or a prior run).
  if (const char* navDataDir = FlagValue(argc, argv, "--nav-data-dir")) {
    avionics::xplane_install::setNavDataRoot(navDataDir);
  } else if (!savedSettings.navDataDir.empty()) {
    avionics::xplane_install::setNavDataRoot(savedSettings.navDataDir);
  }

  const bool cliAlwaysOnTop = WantsAlwaysOnTop(argc, argv);

  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  // DEM terrain rebuilds sample ~260k elevation points per raster. That work
  // must not run on the render thread (it pinned the MFD at ~100 ms/frame and
  // the whole suite at ~8 fps), so both shells offload it to a worker; only the
  // finished RGBA buffer's GPU upload happens on the render thread.
  avionics::map::setAsyncTerrainBuilds(true);

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
    const char* eisSel = FlagValue(argc, argv, "--eis");
    const char* checklistSel = FlagValue(argc, argv, "--checklist");
    const int rc = RunScreenshot(shot, seconds, state, fmsPlan, showBezel,
                                 eisSel, checklistSel);
    // Skip the global static teardown, which aborts on macOS; the capture is
    // already flushed to disk. Matches the other short-lived utility modes.
    QuitUtilityMode(rc);
  }

  // Lists the connected monitors and their indices, so the user can choose one
  // for --pfd-monitor / --mfd-monitor, then exits.
  if (HasFlag(argc, argv, "--list-monitors")) {
    ListMonitors();
    QuitUtilityMode();
  }

  // Flashes each monitor's index on screen (the visual companion to
  // --list-monitors), then exits. Used by the installer's "Identify" button so
  // the user can see which number is which monitor. --time overrides how long.
  if (HasFlag(argc, argv, "--identify-monitors")) {
    const char* timeStr = FlagValue(argc, argv, "--time");
    IdentifyMonitors(timeStr ? std::atof(timeStr) : 0.0);
    QuitUtilityMode();
  }

  // Check for a newer release in the background (a detached network thread).
  // Deferred to here so the short-lived utility modes above (--screenshot,
  // --list-monitors, --identify-monitors) exit immediately without spawning it.

  // --no-mfd / --no-pfd suppress one display so the PFD and MFD can be launched
  // as separate processes. At least one must remain; if both are suppressed the
  // PFD is kept. With no PFD the MFD becomes the primary display (it drives the
  // shared data source and owns vsync). --demo boots straight into the built-in
  // demo feed (same as the Ctrl+Shift+D toggle) so the displays show motion
  // without a running sim.
  bool wantMfd = !HasFlag(argc, argv, "--no-mfd");
  bool wantPfd = !HasFlag(argc, argv, "--no-pfd");
  if (!wantMfd && !wantPfd) wantPfd = true;
  const bool demoStart = HasFlag(argc, argv, "--demo");
  // The dev-only Debug menu (data-source / demo-state / power switches) is
  // installed only with --debug-menu, which the IDE launch configs/tasks pass
  // but the installer never does -- so it never appears in a shipped build.
  const bool debugMenu = HasFlag(argc, argv, "--debug-menu");

  // Persisted preferences (loaded up front) drive the display unless the
  // command line overrides them.
  const bool showBezel = savedSettings.showBezel;
  const bool showWindowChrome = savedSettings.showWindowChrome;
  // The --always-on-top flag / env var forces floating on; otherwise honor the
  // persisted preference.
  const bool alwaysOnTop = cliAlwaysOnTop || savedSettings.alwaysOnTop;
  const int winW = SuiteWindowWidth(showBezel);
  const int winH = SuiteWindowHeight(showBezel);

  // Borderless full-screen: each display takes over a monitor with no title
  // bar. Resolved from the persisted preference, overridden by the command
  // line: --fullscreen turns both on, --no-fullscreen forces both off, and
  // --pfd-monitor / --mfd-monitor pin a display to a monitor (implying full
  // screen for it). When a monitor index is left unset (-1) the PFD takes the
  // first monitor and the MFD the second one if a second exists.
  bool pfdFullscreen = savedSettings.pfdFullscreen;
  bool mfdFullscreen = savedSettings.mfdFullscreen;
  int pfdMonitorIdx = savedSettings.pfdMonitor;
  int mfdMonitorIdx = savedSettings.mfdMonitor;
  if (HasFlag(argc, argv, "--fullscreen")) {
    pfdFullscreen = true;
    mfdFullscreen = true;
  }
  if (HasFlag(argc, argv, "--no-fullscreen")) {
    pfdFullscreen = false;
    mfdFullscreen = false;
  }
  if (const char* v = FlagValue(argc, argv, "--pfd-monitor")) {
    pfdMonitorIdx = std::atoi(v);
    pfdFullscreen = true;
  }
  if (const char* v = FlagValue(argc, argv, "--mfd-monitor")) {
    mfdMonitorIdx = std::atoi(v);
    mfdFullscreen = true;
  }
  int monitorCount = 0;
  glfwGetMonitors(&monitorCount);
  // Default monitor assignment when unpinned: PFD on monitor 0, MFD on monitor
  // 1 (so two-screen cockpits get one display each without extra flags).
  const int pfdMonitorResolved = pfdMonitorIdx >= 0 ? pfdMonitorIdx : 0;
  const int mfdMonitorResolved =
      mfdMonitorIdx >= 0 ? mfdMonitorIdx : (monitorCount > 1 ? 1 : 0);
  GLFWmonitor* pfdMonitorHandle =
      pfdFullscreen ? MonitorByIndex(pfdMonitorResolved) : nullptr;
  GLFWmonitor* mfdMonitorHandle =
      mfdFullscreen ? MonitorByIndex(mfdMonitorResolved) : nullptr;

  // Each display is its own GLFW window with its own GL context + renderer. The
  // first one created owns vsync (swap interval 1) and paces the whole loop;
  // any second display uses swap interval 0 so its buffer swap does not block on
  // a second vsync (which would otherwise halve the frame rate). With both
  // displays this makes the PFD primary, as before; with --no-pfd the MFD is the
  // sole display and becomes primary.
  GLFWwindow* pfdWindow = nullptr;
  avionics::NanoVgRenderer* pfdRenderer = nullptr;
  GLFWwindow* mfdWindow = nullptr;
  avionics::NanoVgRenderer* mfdRenderer = nullptr;
  bool primaryCreated = false;
  const auto createDisplay = [&](const char* title, bool fullscreen,
                                 GLFWmonitor* monitor, GLFWwindow** outWindow,
                                 avionics::NanoVgRenderer** outRenderer) -> bool {
    GLFWwindow* window = CreateAvionicsWindow(
        title, alwaysOnTop, fullscreen ? false : showWindowChrome, nullptr, winW,
        winH, monitor);
    if (window == nullptr) return false;
    glfwMakeContextCurrent(window);
    avionics::render::ensureGlLoaded();
    glfwSwapInterval(primaryCreated ? 0 : 1);  // first window's vsync paces the loop
    glfwSetKeyCallback(window, OnKey);
    glfwSetMouseButtonCallback(window, OnMouseButton);
    glfwSetCursorPosCallback(window, OnCursorPos);
    glfwSetScrollCallback(window, OnScroll);
    auto* renderer = new avionics::NanoVgRenderer();
    if (!renderer->valid()) {
      delete renderer;
      glfwDestroyWindow(window);
      return false;
    }
    *outWindow = window;
    *outRenderer = renderer;
    primaryCreated = true;
    return true;
  };

  // The PFD is created first when present, keeping it the vsync-owning primary.
  if (wantPfd &&
      !createDisplay(kWindowTitle, pfdFullscreen, pfdMonitorHandle, &pfdWindow,
                     &pfdRenderer)) {
    std::fprintf(stderr, "Failed to create PFD window\n");
    glfwTerminate();
    return 1;
  }
  if (wantMfd &&
      !createDisplay(kMfdWindowTitle, mfdFullscreen, mfdMonitorHandle,
                     &mfdWindow, &mfdRenderer)) {
    // A PFD-less run has no fallback display, so the MFD failing is fatal;
    // otherwise fall back to running the PFD on its own.
    if (!wantPfd) {
      std::fprintf(stderr, "Failed to create MFD window\n");
      glfwTerminate();
      return 1;
    }
    std::fprintf(stderr, "Failed to create MFD renderer; running PFD only\n");
  }

  // Borderless full-screen displays are placed at their monitor's top-left so
  // the undecorated window covers exactly that screen.
  if (pfdWindow != nullptr && pfdFullscreen && pfdMonitorHandle != nullptr) {
    int mx = 0, my = 0;
    glfwGetMonitorPos(pfdMonitorHandle, &mx, &my);
    glfwSetWindowPos(pfdWindow, mx, my);
  }
  if (mfdWindow != nullptr && mfdFullscreen && mfdMonitorHandle != nullptr) {
    int mx = 0, my = 0;
    glfwGetMonitorPos(mfdMonitorHandle, &mx, &my);
    glfwSetWindowPos(mfdWindow, mx, my);
  }

  // Window placement, applied while the windowed displays are still hidden so
  // they first appear in their final spots: the saved positions when a layout
  // was remembered, otherwise the MFD docked just to the right of the PFD.
  // Full-screen displays already own their monitor, so they are left alone.
  if (pfdWindow != nullptr && !pfdFullscreen) {
    if (savedSettings.hasWindowPos) {
      glfwSetWindowPos(pfdWindow, savedSettings.pfdWindowX,
                       savedSettings.pfdWindowY);
    }
  }
  if (mfdWindow != nullptr && !mfdFullscreen) {
    if (savedSettings.hasWindowPos) {
      glfwSetWindowPos(mfdWindow, savedSettings.mfdWindowX,
                       savedSettings.mfdWindowY);
    } else if (pfdWindow != nullptr && !pfdFullscreen) {
      int px = 0, py = 0;
      glfwGetWindowPos(pfdWindow, &px, &py);
      glfwSetWindowPos(mfdWindow, px + winW + kWindowGap, py);
    }
  }
  if (pfdWindow != nullptr) glfwShowWindow(pfdWindow);
  if (mfdWindow != nullptr) glfwShowWindow(mfdWindow);
  // Bring both displays to the front immediately so they appear on their
  // monitors without the user having to click each window (otherwise, launched
  // over a terminal or onto a second monitor, they can open in the background).
  if (mfdWindow != nullptr) glfwFocusWindow(mfdWindow);
  if (pfdWindow != nullptr) glfwFocusWindow(pfdWindow);
  glfwPollEvents();  // sync GLFW_FOCUSED before the first render frame

#if defined(_WIN32)
  avionics::setUpdateDialogOwnerWindows(
      pfdWindow != nullptr ? glfwGetWin32Window(pfdWindow) : nullptr,
      mfdWindow != nullptr ? glfwGetWin32Window(mfdWindow) : nullptr);
#endif
  // Deferred until the display HWNDs exist so the update dialog can parent above
  // borderless full-screen windows (WS_EX_TOPMOST).
  avionics::startUpdateCheckOnLaunch();

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

  const char* cmdBridgePortStr = FlagValue(argc, argv, "--command-bridge-port");
  const std::uint16_t cmdBridgePort =
      cmdBridgePortStr
          ? static_cast<std::uint16_t>(std::atoi(cmdBridgePortStr))
          : avionics::cmdbridge::kDefaultListenPort;

  avionics::CommandBridgeClient commandBridge(host ? host : kDefaultXPlaneHost,
                                              cmdBridgePort);

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
  avionics::LandDataStore landData(
      avionics::assets::resolve("land_data.bin", kLandDataAssetPath));
  avionics::FmsPlanStore fmsPlan(fmsPlanArg ? fmsPlanArg : "");
  avionics::DsfTerrainStore terrain;

  // Author-supplied checklists for the MFD Checklist page group. --checklist
  // selects a file; otherwise the build-time sample is used.
  const char* checklistArg = FlagValue(argc, argv, "--checklist");
  avionics::ChecklistStore checklists(checklistArg ? checklistArg : "");
  const char* eisArg = FlagValue(argc, argv, "--eis");
  avionics::EisStore eisStore(eisArg ? eisArg : "");

  // FAA DDOF obstacle database (CSV). Loads from assets/obstacles.csv when
  // present; --obstacles overrides the path.
  const char* obstaclesArg = FlagValue(argc, argv, "--obstacles");
  avionics::ObstacleStore obstacles(resolveObstacleDatabasePath(obstaclesArg));
  avionics::ProcedureStore procedures(navData, &aptData);

  // Real X-Plane navigation data behind the core's NavFeatureSource interface,
  // so the map always shows X-Plane data.
  avionics::ShellNavMapData navMapData(navData, airspace, airways, aptData,
                                       landData, procedures, &obstacles);

  avionics::XPlaneConnection xplane(host ? host : kDefaultXPlaneHost, port,
                                    navData, airspace, airways, aptData,
                                    landData, fmsPlan, &terrain, &checklists,
                                    &eisStore, &obstacles, bridgePort,
                                    fmsWriteEnabled);

  // Built-in demo feed: the same motion-only mock the screenshot path uses,
  // wired to the same real nav databases (it never fabricates navigation data)
  // so toggling to it (Ctrl+Shift+D) shows believable motion on the real map
  // without a running sim.
  avionics::MockDataSource demoSource;
  demoSource.setNavFeatureSource(&navMapData);
  demoSource.setTerrainSource(&terrain);
  demoSource.setChecklistSource(&checklists);
  demoSource.setEisSource(&eisStore);

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

  // The standalone always reads the live X-Plane connection. Until the sim
  // starts delivering data the engine shows the power-up / waiting screen, then
  // the live pages.
  avionics::DataSource& initialSource = xplane;
  const std::string initialLabel = xplane.simulatorName();

  // Both displays read the same source. Exactly one engine pumps it each frame
  // (the primary display); the other renders the same data without stepping it
  // again. The PFD is that pump when present; with --no-pfd the MFD takes over.
  avionics::AvionicsEngine* pfdEngine = nullptr;
  if (pfdRenderer != nullptr) {
    pfdEngine =
        new avionics::AvionicsEngine(initialSource, *pfdRenderer, initialLabel);
    pfdEngine->setPage(avionics::DisplayPage::PrimaryFlightDisplay);
    pfdEngine->softkeyController().setNavFeatureSource(&navMapData);
  }

  avionics::AvionicsEngine* mfdEngine = nullptr;
  if (mfdRenderer != nullptr) {
    mfdEngine =
        new avionics::AvionicsEngine(initialSource, *mfdRenderer, initialLabel);
    mfdEngine->setPage(avionics::DisplayPage::MultiFunctionDisplay);
    // The MFD only pumps the source when it is the sole display (--no-pfd);
    // otherwise the PFD drives it.
    mfdEngine->setDrivesDataSource(pfdEngine == nullptr);
    mfdEngine->mfdController().setSimbriefPilotId(simbriefPilotId);
    // Ident lookups for FPL waypoint entry come from the parsed nav database.
    mfdEngine->mfdController().setNavFeatureSource(&navMapData);
  }

  // Restore the durable display preferences from the last run (e.g. the PFD
  // inset map on/off, map ranges, declutter) so the avionics come up the way
  // the pilot left them.
  if (pfdEngine != nullptr) {
    // Like the in-sim plugin: the PFD comes up on the live page; only the MFD
    // replays the power-up screen and waits for ENT.
    pfdEngine->skipBoot();
    avionics::applyPfdState(pfdEngine->softkeyController(),
                            savedSettings.avionics.pfd);
  }
  if (mfdEngine != nullptr) {
    avionics::applyMfdState(mfdEngine->mfdController(),
                            savedSettings.avionics.mfd);
  }

  AppState app;
  app.xplane = &xplane;
  app.demoSource = &demoSource;
  app.activeSource = &xplane;
  app.debugMenuEnabled = debugMenu;
  app.showBezel = showBezel;
  app.settings = savedSettings;
  app.settings.alwaysOnTop = alwaysOnTop;
  app.settings.pfdFullscreen = pfdFullscreen;
  app.settings.mfdFullscreen = mfdFullscreen;
  app.settings.pfdMonitor = pfdMonitorIdx;
  app.settings.mfdMonitor = mfdMonitorIdx;
  if (!app.settings.loaded) {
    app.settings.showBezel = showBezel;
  }
  app.pfdFullscreen = pfdFullscreen;
  app.mfdFullscreen = mfdFullscreen;
  // Seed the windowed geometry to restore when the F key leaves full screen
  // (the displays boot straight into full screen with no prior windowed pose),
  // from the saved layout when there is one, otherwise the suite size docked
  // side by side.
  app.pfdRestoreW = winW;
  app.pfdRestoreH = winH;
  app.mfdRestoreW = winW;
  app.mfdRestoreH = winH;
  if (savedSettings.hasWindowPos) {
    app.pfdRestoreX = savedSettings.pfdWindowX;
    app.pfdRestoreY = savedSettings.pfdWindowY;
    app.mfdRestoreX = savedSettings.mfdWindowX;
    app.mfdRestoreY = savedSettings.mfdWindowY;
  } else {
    app.pfdRestoreX = 100;
    app.pfdRestoreY = 100;
    app.mfdRestoreX = 100 + winW + kWindowGap;
    app.mfdRestoreY = 100;
  }
  app.pfdEngine = pfdEngine;
  app.mfdEngine = mfdEngine;
  if (pfdEngine != nullptr && mfdEngine != nullptr) {
    pfdEngine->setSoftkeyPeer(mfdEngine);
    mfdEngine->setSoftkeyPeer(pfdEngine);
  }
  app.pfdWindow = pfdWindow;
  app.mfdWindow = mfdWindow;
  app.pfdPinnedMonitor = pfdMonitorHandle;
  app.mfdPinnedMonitor = mfdMonitorHandle;
  // Hover cursors for the bezel: left-right over the rotatable rings, hand over
  // the other clickable controls (a null handle falls back to the arrow).
  app.rotateCursor = glfwCreateStandardCursor(GLFW_RESIZE_EW_CURSOR);
  app.handCursor = glfwCreateStandardCursor(GLFW_POINTING_HAND_CURSOR);
  app.rangeCwCursor = MakeRotateCursor(true);
  app.rangeCcwCursor = MakeRotateCursor(false);
  if (pfdWindow != nullptr) glfwSetWindowUserPointer(pfdWindow, &app);
  if (mfdWindow != nullptr) glfwSetWindowUserPointer(mfdWindow, &app);

  // View toggles are keyboard shortcuts (persisted): B = bezel strips, T = OS
  // window title bar, P = keep windows on top. Window positions are always
  // restored on the next launch.
  std::fprintf(stderr,
               "Shortcuts: B = bezel, T = title bar, P = always-on-top, "
               "F = full screen, Shift+B = display backup, Ctrl+Shift+D = demo feed, Esc = quit.\n");

  // Bring up the initial data source. With the Debug menu enabled the last-used
  // feed, demo flight state, and demo power switches are restored from settings
  // (--demo still forces the demo feed, defaulting to flying), and the macOS
  // Debug menu is installed so the choices can be changed live and remembered.
  // Without the menu (the installer, or a bare dev run), --demo just boots into
  // the demo feed as before.
  if (debugMenu) {
    ApplyDebugPower(app, app.settings.demoMasterPowerOn,
                    app.settings.demoAvionicsPowerOn, /*persist=*/false);
    ApplyDebugCas(app, app.settings.demoCasMessagesOn, /*persist=*/false);
    avionics::DebugDataSource initialSel = app.settings.debugDataSource;
    if (demoStart && initialSel == avionics::DebugDataSource::XPlane) {
      initialSel = avionics::DebugDataSource::DemoFlying;
    }
    ApplyDebugDataSource(app, initialSel, /*persist=*/false);
#if defined(__APPLE__)
    avionics::DebugMenuConfig menuCfg;
    menuCfg.source = initialSel;
    menuCfg.masterPowerOn = app.settings.demoMasterPowerOn;
    menuCfg.avionicsPowerOn = app.settings.demoAvionicsPowerOn;
    menuCfg.casMessagesOn = app.settings.demoCasMessagesOn;
    menuCfg.onSource = OnDebugSelectSource;
    menuCfg.onPower = OnDebugTogglePower;
    menuCfg.onCas = OnDebugToggleCas;
    menuCfg.context = &app;
    avionics::InstallDebugMenu(menuCfg);
#endif
  } else if (demoStart) {
    ToggleDemoSource(app);
  }

  // Optional per-display render profiler (AVIONICS_PROFILE=1): isolates the GPU
  // cost of each window's draw with a glFinish so we can see whether the PFD or
  // the MFD is what keeps the single-threaded, vsync-paced loop from holding 60
  // fps. Off by default (glFinish serializes the pipeline, so it skews timing).
  const bool profile = std::getenv("AVIONICS_PROFILE") != nullptr;

  // Renders one engine into its window: the avionics screen on the left and the
  // hardware bezel strip on the right. The gauge code draws directly in
  // framebuffer pixels, so the NanoVG device-pixel-ratio is 1.0. Returns the
  // render time in milliseconds when profiling (0 otherwise).
  avionics::NanoVgRenderer::DrawStats mfdLastStats;
  const auto renderWindow = [&app, profile](
                                GLFWwindow* win, avionics::AvionicsEngine& eng,
                                avionics::NanoVgRenderer& renderer, double dt,
                                avionics::NanoVgRenderer::DrawStats* stats =
                                    nullptr) -> double {
    glfwMakeContextCurrent(win);
    int fbWidth = 0, fbHeight = 0;
    glfwGetFramebufferSize(win, &fbWidth, &fbHeight);
    const bool flip180 =
        NeedsDisplayFlip180(win, PinnedMonitorForWindow(app, win));
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    eng.update(dt);
    const auto t0 = std::chrono::steady_clock::now();
    // Draw into the aspect-preserving content rectangle, leaving the cleared
    // black bars around it on a full-screen monitor of a different shape. For a
    // windowed display the rectangle is the whole framebuffer.
    const ContentRect cr = ComputeContentRect(fbWidth, fbHeight, app.showBezel);
    const int glY = fbHeight - (cr.y + cr.h);  // GL viewport origin is bottom-left
    RenderSuiteViewport(renderer, eng, cr.x, glY, cr.w, cr.h, app.showBezel,
                        flip180, stats);
    if (profile) glFinish();
    const double ms =
        profile ? std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count()
                : 0.0;
    glfwSwapBuffers(win);
    return ms;
  };

  using clock = std::chrono::steady_clock;
  auto previous = clock::now();

  // Profiler accumulators (only used when AVIONICS_PROFILE is set). The MFD
  // draw-stat sums are the engine's map content (captured before the bezel
  // resets the counter), so they reflect the actual map cost, not the bezel.
  double profPfdMs = 0.0, profMfdMs = 0.0;
  std::uint64_t profFills = 0, profStrokes = 0, profTexts = 0, profImages = 0,
                profVerts = 0;
  int profFrames = 0;
  auto profPrev = clock::now();

  // Closing either window exits: the PFD and MFD are one avionics suite. A
  // suppressed display is null and simply ignored here.
  while ((pfdWindow == nullptr || !glfwWindowShouldClose(pfdWindow)) &&
         (mfdWindow == nullptr || !glfwWindowShouldClose(mfdWindow))) {
    glfwPollEvents();  // refresh GLFW_FOCUSED before drawing (needed for flipped
                       // monitor compensation on the first frame after launch)

    const auto now = clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;

    // Pick up live edits to the checklist file so authors can iterate without
    // restarting.
    checklists.refreshIfChanged();
    eisStore.refreshIfChanged();

    // SimBrief: react to the AUX - SIMBRIEF page (a newly committed Pilot ID
    // is persisted; FETCH kicks off a download), land completed fetches into
    // the X-Plane feed's flight plan, and publish the status back for rendering.
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
        // The OFP becomes the displayed flight plan on the X-Plane feed.
        xplane.setRouteOverride(simbriefResult.legs);
      } else {
        simbriefState.status = avionics::SimBriefStatus::Error;
        simbriefState.error = simbriefResult.error;
      }
    }
    if (mfdEngine != nullptr) {
      mfdEngine->mfdController().setSimbriefState(simbriefState);
    }

    // PFD Direct-To activation (the PFD has its own Direct-To window): engage
    // the direct course the same way the MFD's window does.
    if (pfdEngine != nullptr) {
      avionics::MapLeg pfdDto;
      if (pfdEngine->softkeyController().consumeDirectToRequest(pfdDto)) {
        xplane.setDirectTo(pfdDto);
        if (app.demoSource != nullptr &&
            app.activeSource == app.demoSource) {
          app.demoSource->directTo(pfdDto);
        }
      }

      // PFD Active Flight Plan window edits (insert / remove / delete), and PROC
      // window procedure loads, override the displayed plan and program the FMS,
      // the same as the MFD FPL page.
      std::vector<avionics::MapLeg> pfdEditedPlan;
      if (pfdEngine->softkeyController().consumeFlightPlanEdit(pfdEditedPlan)) {
        xplane.setRouteOverride(pfdEditedPlan);
      }

      // A loaded PROC approach with an ILS frequency tunes NAV1 standby (same as
      // the MFD's procedure load).
      avionics::MapProcedure pfdProc;
      if (pfdEngine->softkeyController().consumeProcLoadRequest(pfdProc) &&
          pfdProc.frequencyMhz > 0.0f) {
        xplane.tuneRadioStandby(avionics::RadioUnit::Nav1, pfdProc.frequencyMhz);
      }
    }

    // FPL page edits override the X-Plane feed's displayed plan, mirroring the
    // SimBrief flow above.
    if (mfdEngine != nullptr) {
      std::vector<avionics::MapLeg> editedPlan;
      if (mfdEngine->mfdController().consumeFlightPlanEdit(editedPlan)) {
        xplane.setRouteOverride(editedPlan);
      }

      // Direct-To activation: the X-Plane feed shows the magenta direct line
      // (display only over UDP).
      avionics::MapLeg dtoTarget;
      if (mfdEngine->mfdController().consumeDirectToRequest(dtoTarget)) {
        xplane.setDirectTo(dtoTarget);
        if (app.demoSource != nullptr &&
            app.activeSource == app.demoSource) {
          app.demoSource->directTo(dtoTarget);
        }
      }

      avionics::MapProcedure proc;
      if (mfdEngine->mfdController().consumeProcLoadRequest(proc) &&
          proc.frequencyMhz > 0.0f) {
        xplane.tuneRadioStandby(avionics::RadioUnit::Nav1, proc.frequencyMhz);
      }

      // Map panning: keep the feed's nearby-data queries centered on the panned
      // map view (not just the pointer geo) so land and symbols match the screen.
      avionics::MfdController& mapUi = mfdEngine->mfdController();
      avionics::DataSource* mapSource =
          app.activeSource != nullptr
              ? app.activeSource
              : static_cast<avionics::DataSource*>(&xplane);
      mapUi.applyMapPanToDataSource(*mapSource, mapSource->snapshot());
      mapSource->setMapViewHalfExtentNm(mapUi.mapViewHalfExtentNm());
      mapSource->setChartRangeNm(mapUi.rangeNm());
    }

    // Cockpit bezel / softkey / radio events forwarded from the in-sim plugin.
    {
      std::vector<avionics::cmdbridge::Event> bridgeEvents;
      commandBridge.drainEvents(bridgeEvents);
      ApplyBridgeEvents(app, bridgeEvents);
    }

    // Radio / transponder / HDG / CRS / BARO commands from the bezel and XPDR
    // softkeys. The transponder lives on the PFD; the NAV/COM/CRS/BARO/HDG
    // knobs exist on both GDU bezels, so both engines' controllers are drained.
    {
      // In demo mode the displays read the mock feed, so radio swaps/tunes,
      // transponder edits and the HDG/CRS/BARO knobs must be applied there;
      // otherwise they'd update the (hidden) X-Plane feed and the on-glass
      // values would never change even though the press animation plays. (The
      // mock auto-drives course along the active leg, so a manual course set is
      // transient there, but the heading bug and baro setting hold.)
      avionics::MockDataSource* demoRadio =
          app.demoMode ? app.demoSource : nullptr;
      const auto commitKnobs = [&xplane,
                                demoRadio](avionics::SoftkeyController& ui) {
        avionics::RadioUnit unit;
        float mhz = 0.0f;
        if (ui.consumeRadioTune(unit, mhz)) {
          if (demoRadio != nullptr) demoRadio->tuneRadioStandby(unit, mhz);
          else xplane.tuneRadioStandby(unit, mhz);
        }
        if (ui.consumeRadioTransfer(unit)) {
          if (demoRadio != nullptr) demoRadio->transferRadio(unit);
          else xplane.transferRadio(unit);
        }
        float volume = 0.0f;
        if (ui.consumeRadioVolume(unit, volume)) {
          if (demoRadio != nullptr) demoRadio->setRadioVolume(unit, volume);
          else xplane.setRadioVolume(unit, volume);
        }
        bool identOn = false;
        if (ui.consumeNavIdent(unit, identOn)) {
          if (demoRadio != nullptr) demoRadio->setNavIdent(unit, identOn);
          else xplane.setNavIdent(unit, identOn);
        }
        float deg = 0.0f;
        if (ui.consumeHeadingBug(deg)) {
          if (demoRadio != nullptr) demoRadio->setHeadingBug(deg);
          else xplane.setHeadingBug(deg);
        }
        if (ui.consumeCourse(deg)) {
          if (demoRadio != nullptr) demoRadio->setSelectedCourse(deg);
          else xplane.setSelectedCourse(deg);
        }
        float inHg = 0.0f;
        if (ui.consumeBaro(inHg)) {
          if (demoRadio != nullptr) demoRadio->setBaroInHg(inHg);
          else xplane.setBaroInHg(inHg);
        }
      };
      // The transponder lives only on the PFD; drain it there when present.
      if (pfdEngine != nullptr) {
        avionics::SoftkeyController& pfdUi = pfdEngine->softkeyController();
        int xpdrCode = 0;
        if (pfdUi.consumeXpdrCodeCommit(xpdrCode)) {
          if (demoRadio != nullptr) demoRadio->setTransponderCode(xpdrCode);
          else xplane.setTransponderCode(xpdrCode);
        }
        int xpdrMode = 0;
        if (pfdUi.consumeXpdrModeCommit(xpdrMode)) {
          if (demoRadio != nullptr) demoRadio->setTransponderMode(xpdrMode);
          else xplane.setTransponderMode(xpdrMode);
        }
        commitKnobs(pfdUi);
      }
      if (mfdEngine != nullptr) commitKnobs(mfdEngine->softkeyController());
    }

    // CLR held past the threshold acts as CLR (DFLT MAP): display the MFD
    // Navigation Map page immediately.
    if (app.clrHoldEngine != nullptr &&
        glfwGetTime() - app.clrHoldStart >=
            avionics::kClrDefaultMapHoldSeconds) {
      app.clrHoldEngine->holdBezelKey(avionics::BezelKey::Clr);
      app.clrHoldEngine = nullptr;
    }

    // Render the data-driving (primary) display first so the shared source is
    // pumped once before the secondary draws the same frame. The PFD is primary
    // when present; with --no-pfd the MFD is the lone, primary display.
    double pfdMs = 0.0;
    double mfdMs = 0.0;
    if (pfdWindow != nullptr && pfdEngine != nullptr) {
      pfdMs = renderWindow(pfdWindow, *pfdEngine, *pfdRenderer, dt);
    }
    if (mfdWindow != nullptr && mfdEngine != nullptr) {
      mfdMs = renderWindow(mfdWindow, *mfdEngine, *mfdRenderer, dt,
                           profile ? &mfdLastStats : nullptr);
    }
    if (profile) {
      profPfdMs += pfdMs;
      profMfdMs += mfdMs;
      profFills += static_cast<std::uint64_t>(mfdLastStats.fills);
      profStrokes += static_cast<std::uint64_t>(mfdLastStats.strokes);
      profTexts += static_cast<std::uint64_t>(mfdLastStats.texts);
      profImages += static_cast<std::uint64_t>(mfdLastStats.images);
      profVerts += static_cast<std::uint64_t>(mfdLastStats.verts);
      ++profFrames;
      const auto pnow = clock::now();
      const double sinceMs =
          std::chrono::duration<double, std::milli>(pnow - profPrev).count();
      if (sinceMs >= 1000.0 && profFrames > 0) {
        std::fprintf(
            stderr,
            "[profile] %.1f fps | PFD %.1f ms | MFD %.1f ms | MFD draw "
            "fills %llu strokes %llu text %llu img %llu verts %llu "
            "(avg/%d frames)\n",
            profFrames * 1000.0 / sinceMs, profPfdMs / profFrames,
            profMfdMs / profFrames,
            static_cast<unsigned long long>(profFills / profFrames),
            static_cast<unsigned long long>(profStrokes / profFrames),
            static_cast<unsigned long long>(profTexts / profFrames),
            static_cast<unsigned long long>(profImages / profFrames),
            static_cast<unsigned long long>(profVerts / profFrames),
            profFrames);
        profPfdMs = profMfdMs = 0.0;
        profFills = profStrokes = profTexts = profImages = profVerts = 0;
        profFrames = 0;
        profPrev = pnow;
      }
    }

    // Persist durable display preferences whenever the pilot changes one (e.g.
    // toggling the PFD inset map), so they survive the next launch. Writes only
    // happen on an actual change, so the common no-change frame costs a cheap
    // struct compare.
    {
      avionics::AvionicsPersistentState current = app.settings.avionics;
      if (pfdEngine != nullptr) {
        avionics::capturePfdState(pfdEngine->softkeyController(), current.pfd);
      }
      if (mfdEngine != nullptr) {
        avionics::captureMfdState(mfdEngine->mfdController(), current.mfd);
      }
      if (current != app.settings.avionics) {
        app.settings.avionics = current;
        avionics::SaveAppSettings(app.settings);
      }
    }

  }

  // Capture the final window placement for the next launch before the windows
  // go away (always remembered).
  CaptureWindowPositions(app);
  avionics::SaveAppSettings(app.settings);

  // Tear down GL objects while their contexts are still current.
  if (pfdWindow != nullptr) {
    glfwMakeContextCurrent(pfdWindow);
    delete pfdEngine;
    delete pfdRenderer;
    glfwDestroyWindow(pfdWindow);
  }
  if (mfdWindow != nullptr) {
    glfwMakeContextCurrent(mfdWindow);
    delete mfdEngine;
    delete mfdRenderer;
    glfwDestroyWindow(mfdWindow);
  }
  if (app.rotateCursor != nullptr) glfwDestroyCursor(app.rotateCursor);
  if (app.handCursor != nullptr) glfwDestroyCursor(app.handCursor);
  if (app.rangeCwCursor != nullptr) glfwDestroyCursor(app.rangeCwCursor);
  if (app.rangeCcwCursor != nullptr) glfwDestroyCursor(app.rangeCcwCursor);
  avionics::map::setAsyncTerrainBuilds(false);
  glfwTerminate();
  return 0;
}
