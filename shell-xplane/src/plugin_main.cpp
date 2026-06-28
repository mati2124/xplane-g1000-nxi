// X-Plane plugin shell.
//
// Hosts the shared avionics-core inside the sim. X-Plane drives us: we hook the
// built-in G1000 PFD and MFD "avionics devices" (XPLMRegisterAvionicsCallbacksEx)
// and draw our own glass cockpit into their screen framebuffers, suppressing
// X-Plane's stock G1000 rendering. The same engine + NanoVG renderer the
// standalone shell uses run here, fed by datarefs (DatarefDataSource) instead of
// the network feed. We do NOT own the loop and must stay within the frame budget.
//
// Build: requires the X-Plane SDK (set XPLANE_SDK_DIR). The Avionics device API
// used here needs XPLM400 (X-Plane 12.04+), which CMake already defines.

// X-Plane's avionics OpenGL bridge is a 2.1 context, so use the legacy GL header
// (and the GL2 NanoVG backend below) rather than the GL3 path the standalone
// shell uses.
#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>  // EXT_framebuffer_object (render-to-texture cache)
#else
// Windows/Linux: GLEW provides the GL2 + EXT_framebuffer_object entry points the
// render-to-texture cache uses. avionics::render::ensureGlLoaded() initializes
// it once inside X-Plane's GL context. glew.h must precede any other GL header.
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#endif
#include <GL/glew.h>
#endif

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "CommandBridge.h"
#include "DatarefDataSource.h"
#include "ObstacleStore.h"
#include "NavData.h"
#include "PluginNavMapData.h"
#include "ProcedureStore.h"
#include "FlightPlanBridge.h"
#include "FmsDebugOverlay.h"
#include "FmsRouteProgrammer.h"
#include "NavigraphStore.h"
#include "UpdateNotify.h"
#include "avionics/AssetPaths.h"
#include "avionics/Charts.h"
#include "avionics/ChecklistStore.h"
#include "avionics/EisStore.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/CommandBridgeProtocol.h"
#include "avionics/FlightPlanBridgeProtocol.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/PersistentState.h"
#include "avionics/Terrain.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/GlLoader.h"
#include "avionics/render/NanoVgRenderer.h"

namespace {

// Prefix for our Log.txt diagnostics so they're easy to grep.
void Log(const char* msg) { XPLMDebugString(msg); }

// Feed label for the boot / connection-lost screens (matches the standalone
// shell's live-source naming).
constexpr const char* kSourceLabel = "X-PLANE";

#ifndef AVIONICS_OBSTACLES
#define AVIONICS_OBSTACLES ""
#endif
constexpr const char* kObstaclesAssetPath = AVIONICS_OBSTACLES;

// Fallback device-screen size if X-Plane reports an empty viewport (it always
// sets one to the device's native resolution; this just avoids a zero divide).
constexpr int kFallbackScreenW = 1024;
constexpr int kFallbackScreenH = 768;

// The data source is shared by both device engines; the PFD engine pumps it,
// the MFD engine reads the same snapshot without double-stepping it.
std::unique_ptr<avionics::DatarefDataSource> g_dataSource;
std::unique_ptr<avionics::NavDataStore> g_navDataStore;
std::unique_ptr<avionics::ProcedureStore> g_procedureStore;
std::unique_ptr<avionics::ObstacleStore> g_obstacleStore;
std::unique_ptr<avionics::PluginNavMapData> g_navMapData;
std::unique_ptr<avionics::EisStore> g_eisStore;
std::unique_ptr<avionics::ChecklistStore> g_checklistStore;
std::uint32_t g_mapGeometryEpoch = 0;

// Serves the live FMS flight plan to the networked standalone shell over UDP
// (the standalone shell cannot read the FMS itself; see FlightPlanBridge.h).
std::unique_ptr<avionics::FlightPlanBridge> g_flightPlanBridge;
std::unique_ptr<avionics::CommandBridge> g_commandBridge;

// SimBrief OFP import via Navigraph (AUX - SIMBRIEF page). Owns the OAuth
// device-flow sign-in + OFP fetch on a background thread; the page's
// Login/Logout/FETCH softkeys drive it. The refresh token persists to the
// per-user config dir so the session restores on the next sim launch.
std::unique_ptr<avionics::NavigraphStore> g_navigraph;
avionics::SimBriefState g_simbriefState;
std::string g_navigraphRefreshToken;
avionics::ChartsState g_chartsState;
unsigned g_chartImageGeneration = 0;

// Redraw divisors. The heavy work (NanoVG re-tessellates the whole vector scene
// on every draw, on the CPU in the GL2 backend) runs only every Nth time our
// draw callback fires; on the other frames we cheaply re-blit the cached
// texture. This is a frame-count divisor, not a time cap: a time cap stops
// skipping once the sim is slow (every frame's dt already exceeds the cap),
// which is exactly when we most need to skip. A real G1000 doesn't refresh its
// glass at the full sim framerate, so this is also more faithful. The MFD's
// moving map / terrain is the heavy scene, so it re-renders less often.
//
// Net effect: the glass refreshes at (sim fps / N). E.g. at 60 sim fps a PFD
// divisor of 3 updates the PFD ~20x/s, and the other frames cost only a cheap
// blit of the last cached frame.
//
// The divisor pair is chosen by a user-selectable quality preset (Plugins ->
// G1000 NXi menu), trading sim fps against how smoothly the glass updates.
// Higher divisors = less avionics work = more sim fps, but choppier glass.
enum class RatePreset { Full, Smooth, Balanced, Performance, Count };

struct PresetSpec {
  const char* name;  // menu label and config-file token
  int pfdEveryN;
  int mfdEveryN;
  // Resolution the heavy MFD scene renders at, as a fraction of the device's
  // native pixels; the cached texture is then upscaled to fill the screen on
  // the per-frame blit. The MFD is fill-rate bound (it shades the whole map
  // surface), so rendering fewer pixels cuts the per-render cost roughly with
  // the square of this factor while only softening the glass slightly. The PFD
  // is cheap and full of crisp tapes/text, so it always renders at 1:1.
  float mfdRenderScale;
  // Absolute upper bound on full scene re-renders per second, independent of
  // sim framerate. The frame-count divisor alone yields (sim fps / N), so at a
  // high sim framerate (120/144 Hz) the glass would re-render far more often
  // than it needs to, burning main-thread time for no visible benefit. This cap
  // holds the redraw rate steady once the sim is fast; the divisor still does
  // the skipping when the sim is slow. 0 means uncapped (Full preset).
  float pfdMaxHz;
  float mfdMaxHz;
};

// Indexed by RatePreset. Full = redraw every frame at full resolution (best
// smoothness, highest cost); Performance = redraw rarely at low resolution
// (best sim fps, choppiest/softest glass).
// The MFD divisor is set noticeably higher than the PFD's: the moving map /
// terrain is the heavy scene but it changes slowly (ground track, not tapes),
// so refreshing it at roughly half the PFD's cadence is visually fine and is
// where most of the per-frame render saving comes from.
constexpr PresetSpec kPresets[] = {
    //          name                pfdN mfdN  scale  pfdHz  mfdHz
    {"Full (every frame)",           1,   1,  1.00f,  0.0f,  0.0f},
    {"Smooth",                       2,   4,  0.85f, 30.0f, 15.0f},
    {"Balanced",                     3,   6,  0.70f, 20.0f, 10.0f},
    {"Performance",                  5,  10,  0.55f, 12.0f,  6.0f},
};
constexpr int kPresetCount = static_cast<int>(RatePreset::Count);

// Default until the saved config (if any) is loaded.
constexpr RatePreset kDefaultPreset = RatePreset::Balanced;

// One customized built-in G1000 screen (PFD or MFD). Its NanoVG renderer must
// be created with a current GL context, which only exists inside the draw
// callback, so the renderer and engine are built lazily on the first draw.
//
// Performance: rendering the full scene every sim frame is expensive, so we
// render it into our own offscreen texture (an FBO we own) only at the device's
// capped rate, and on every sim frame blit that texture onto the device's
// screen with a single quad. Drawing every frame avoids the flicker that
// skipping draws causes for customized built-in devices (their panel
// framebuffer isn't preserved between calls), while the costly re-tessellation
// runs only at the cap. If the FBO can't be set up, cacheDisabled falls back to
// rendering the scene directly every frame.
struct AvionicsDevice {
  avionics::DisplayPage page;
  bool drivesSource;   // only one engine may pump the shared data source
  int renderEveryN;    // re-render the full scene every Nth draw-callback frame
  float renderScale;   // fraction of native res the cached scene renders at
  float maxRenderHz;   // absolute re-render ceiling (0 = uncapped)
  const char* label;   // for diagnostics ("PFD" / "MFD")
  XPLMAvionicsID handle = nullptr;
  std::unique_ptr<avionics::NanoVgRenderer> renderer;
  std::unique_ptr<avionics::AvionicsEngine> engine;
  float lastRenderElapsed = 0.0f;  // XPLMGetElapsedTime at our last scene render
  int frameCounter = 0;            // draw-callback invocations since last reset

  // Offscreen render-to-texture cache.
  GLuint fbo = 0;
  GLuint tex = 0;
  // Stencil buffer for the FBO. NanoVG fills concave/complex paths (e.g. the
  // HSI ownship symbol) with a stencil-then-cover pass, so without a stencil
  // attachment those fills render nothing while convex shapes still draw. The
  // standalone shell's GLFW window has a stencil buffer by default; our own FBO
  // must attach one explicitly. A packed depth24-stencil8 renderbuffer is the
  // most broadly supported way to get an 8-bit stencil on an FBO.
  GLuint depthStencilRbo = 0;
  int texW = 0;
  int texH = 0;
  bool cacheReady = false;     // tex has at least one rendered frame
  bool cacheDisabled = false;  // FBO unavailable; render directly each frame

  // Lightweight profiling: total time spent in our draw callback, plus how many
  // of those frames did a full scene re-render vs. a cheap blit, logged
  // periodically so we can see the real per-frame cost and whether the cache is
  // active.
  double profAccumMs = 0.0;
  int profSamples = 0;
  int profRenders = 0;
  float profLastLog = 0.0f;
  bool modeLogged = false;
  // Draw-call counts summed across the scene re-renders in the current logging
  // window (blits issue none), so the log shows the per-render submission cost
  // the GL2 backend is bound by.
  long profFills = 0;
  long profStrokes = 0;
  long profTexts = 0;
  long profImages = 0;
  long profVerts = 0;

  bool rendererFailed = false;  // GL context couldn't host NanoVG; let X-Plane draw
};

AvionicsDevice g_pfd{
    avionics::DisplayPage::PrimaryFlightDisplay, /*drivesSource=*/true,
    kPresets[static_cast<int>(kDefaultPreset)].pfdEveryN, /*renderScale=*/1.0f,
    kPresets[static_cast<int>(kDefaultPreset)].pfdMaxHz, "PFD"};
AvionicsDevice g_mfd{
    avionics::DisplayPage::MultiFunctionDisplay, /*drivesSource=*/false,
    kPresets[static_cast<int>(kDefaultPreset)].mfdEveryN,
    kPresets[static_cast<int>(kDefaultPreset)].mfdRenderScale,
    kPresets[static_cast<int>(kDefaultPreset)].mfdMaxHz, "MFD"};

// The active quality preset, and the menu that selects it.
RatePreset g_preset = kDefaultPreset;
XPLMMenuID g_rateMenu = nullptr;
int g_rateMenuParentItem = -1;

// "Install Update" item in the G1000 NXi submenu plus its bindable command. The
// item stays disabled until the launch-time check finds a newer release, at
// which point the update pump enables it and labels it with the version.
XPLMCommandRef g_installUpdateCmd = nullptr;
XPLMCommandRef g_fmsDebugCmd = nullptr;
int g_installUpdateMenuIndex = -1;
bool g_installUpdateMenuShown = false;

// When false the stock G1000 renders untouched and only the FMS bridge runs for
// the networked standalone shell.
bool g_replaceDisplays = true;
bool g_showFmsDebug = false;
avionics::AircraftOverride g_aircraftOverride = avionics::AircraftOverride::Auto;

// Durable PFD/MFD display preferences (the softkey-selectable options that
// survive between flights, e.g. the PFD inset map on/off). Loaded at startup,
// applied to each device engine when it is created, and rewritten whenever a
// GDU key press changes one.
avionics::AvionicsPersistentState g_avionicsState;

// Applies a preset's divisors to both device engines.
void ApplyPreset(RatePreset preset) {
  const int idx = static_cast<int>(preset);
  if (idx < 0 || idx >= kPresetCount) return;
  g_preset = preset;
  g_pfd.renderEveryN = kPresets[idx].pfdEveryN;
  g_mfd.renderEveryN = kPresets[idx].mfdEveryN;
  g_mfd.renderScale = kPresets[idx].mfdRenderScale;
  g_pfd.maxRenderHz = kPresets[idx].pfdMaxHz;
  g_mfd.maxRenderHz = kPresets[idx].mfdMaxHz;
  // Force a fresh render on the next frame so the change is visible at once.
  // The cache is dropped too so a render-scale change resizes the FBO.
  g_pfd.frameCounter = 0;
  g_mfd.frameCounter = 0;
  g_pfd.cacheReady = false;
  g_mfd.cacheReady = false;
}

// ---- persistence ------------------------------------------------------------
// Our config is saved next to X-Plane's preferences so it survives restarts.
// It is a tiny key=value text file: the refresh-rate preset token plus the
// shared core's durable display-preference keys (PFD inset map, map ranges,
// declutter, etc.).
constexpr const char* kConfigFileName = "g1000nxi.prf";
constexpr const char* kKeyRate = "rate";
constexpr const char* kKeyReplaceDisplays = "replace_displays";
constexpr const char* kKeyFmsDebug = "fms_debug";
constexpr const char* kKeyAircraftOverride = "aircraft_override";

std::string ConfigFilePath() {
  char prefs[512] = {0};
  XPLMGetPrefsPath(prefs);  // .../Output/preferences/<something>
  std::string path(prefs);
  // Trim back to the last directory separator to get the preferences dir.
  const std::size_t slash = path.find_last_of("/\\");
  if (slash != std::string::npos) path.resize(slash + 1);
  path += kConfigFileName;
  return path;
}

void SaveConfig() {
  std::FILE* f = std::fopen(ConfigFilePath().c_str(), "w");
  if (f == nullptr) return;
  std::fprintf(f, "%s=%s\n", kKeyRate, kPresets[static_cast<int>(g_preset)].name);
  std::fprintf(f, "%s=%d\n", kKeyReplaceDisplays, g_replaceDisplays ? 1 : 0);
  std::fprintf(f, "%s=%d\n", kKeyFmsDebug, g_showFmsDebug ? 1 : 0);
  std::fprintf(f, "%s=%d\n", kKeyAircraftOverride, static_cast<int>(g_aircraftOverride));
  std::string stateLines;
  avionics::appendStateLines(g_avionicsState, stateLines);
  std::fwrite(stateLines.data(), 1, stateLines.size(), f);
  std::fclose(f);
}

void LoadConfig() {
  std::FILE* f = std::fopen(ConfigFilePath().c_str(), "r");
  if (f == nullptr) return;  // no saved config; keep the defaults
  char line[128] = {0};
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    const char* eq = std::strchr(line, '=');
    if (eq == nullptr) continue;
    const std::string key(line, eq - line);
    std::string value(eq + 1);
    // Drop the trailing newline so token/int parsing sees a clean value.
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) {
      value.pop_back();
    }
    if (key == kKeyRate) {
      for (int i = 0; i < kPresetCount; ++i) {
        if (value == kPresets[i].name) {
          ApplyPreset(static_cast<RatePreset>(i));
          break;
        }
      }
    } else if (key == kKeyReplaceDisplays) {
      g_replaceDisplays = (value == "1");
    } else if (key == kKeyFmsDebug) {
      g_showFmsDebug = (value == "1");
    } else if (key == kKeyAircraftOverride) {
      const int val = std::atoi(value.c_str());
      if (val >= 0 && val < static_cast<int>(avionics::AircraftOverride::Count)) {
        g_aircraftOverride = static_cast<avionics::AircraftOverride>(val);
      }
    } else {
      // Durable display preferences are parsed by the shared core.
      avionics::applyStateLine(key, value, g_avionicsState);
    }
  }
  std::fclose(f);
}

// (Re)creates the cache texture + FBO at the given render size (which may be
// below native when the device downsamples). `smooth` selects linear filtering
// so an upscaled (sub-native) render blits without hard pixel edges. Returns
// false if the FBO is unusable, in which case the caller renders directly.
bool EnsureCache(AvionicsDevice& dev, int width, int height, bool smooth) {
  if (dev.cacheReady && dev.texW == width && dev.texH == height) return true;

  if (dev.fbo == 0) glGenFramebuffersEXT(1, &dev.fbo);
  if (dev.tex == 0) glGenTextures(1, &dev.tex);
  if (dev.depthStencilRbo == 0) glGenRenderbuffersEXT(1, &dev.depthStencilRbo);
  if (dev.fbo == 0 || dev.tex == 0 || dev.depthStencilRbo == 0) return false;

  const GLint filter = smooth ? GL_LINEAR : GL_NEAREST;
  glBindTexture(GL_TEXTURE_2D, dev.tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  // Size the depth-stencil renderbuffer to match the color texture (resized
  // alongside it when the render scale changes).
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, dev.depthStencilRbo);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8, width,
                           height);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

  GLint prevFbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &prevFbo);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, dev.fbo);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                            GL_TEXTURE_2D, dev.tex, 0);
  // A packed depth24-stencil8 renderbuffer attaches to both the depth and
  // stencil attachment points; NanoVG only needs the stencil half.
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                               GL_RENDERBUFFER_EXT, dev.depthStencilRbo);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
                               GL_RENDERBUFFER_EXT, dev.depthStencilRbo);
  const GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, static_cast<GLuint>(prevFbo));
  if (status != GL_FRAMEBUFFER_COMPLETE_EXT) return false;

  dev.texW = width;
  dev.texH = height;
  dev.cacheReady = false;  // not yet rendered at this size
  return true;
}

// Renders the full PFD/MFD scene into the device's cache texture.
void RenderSceneToCache(AvionicsDevice& dev, int width, int height, double dt,
                        const GLint deviceViewport[4]) {
  GLint prevFbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &prevFbo);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, dev.fbo);
  glViewport(0, 0, width, height);

  XPLMSetGraphicsState(/*fog=*/0, /*texUnits=*/1, /*lighting=*/0,
                       /*alphaTest=*/0, /*alphaBlend=*/1, /*depthTest=*/0,
                       /*depthWrite=*/0);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClearStencil(0);
  glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  dev.engine->update(dt);
  dev.engine->renderFrame(width, height, 1.0f);

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, static_cast<GLuint>(prevFbo));
  glViewport(deviceViewport[0], deviceViewport[1], deviceViewport[2],
             deviceViewport[3]);
  dev.cacheReady = true;
}

// Blits the cache texture onto the device screen (the framebuffer X-Plane has
// bound) as a single fullscreen quad in the device's viewport.
void BlitCache(AvionicsDevice& dev, int width, int height) {
  XPLMSetGraphicsState(/*fog=*/0, /*texUnits=*/1, /*lighting=*/0,
                       /*alphaTest=*/0, /*alphaBlend=*/0, /*depthTest=*/0,
                       /*depthWrite=*/0);
  glBindTexture(GL_TEXTURE_2D, dev.tex);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0, width, 0.0, height, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f);
  glVertex2f(0.0f, 0.0f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex2f(static_cast<float>(width), 0.0f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex2f(static_cast<float>(width), static_cast<float>(height));
  glTexCoord2f(0.0f, 1.0f);
  glVertex2f(0.0f, static_cast<float>(height));
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glBindTexture(GL_TEXTURE_2D, 0);
}

// Draws one device's screen. Returns 1 to let X-Plane draw its stock G1000
// (when our renderer is unavailable), or 0 to suppress it and show ours.
void InvalidateAvionicsCacheIfMapGeometryChanged() {
  if (!g_dataSource) return;
  const std::uint32_t epoch = g_dataSource->mapGeometryEpoch();
  if (epoch == g_mapGeometryEpoch) return;
  g_mapGeometryEpoch = epoch;
  g_pfd.cacheReady = false;
  g_mfd.cacheReady = false;
}

// Adds the renderer's most-recent-frame draw-call counts (valid right after a
// renderFrame call) into the device's logging-window totals.
void AccumulateDrawStats(AvionicsDevice& dev) {
  if (!dev.renderer) return;
  const avionics::NanoVgRenderer::DrawStats& s = dev.renderer->drawStats();
  dev.profFills += s.fills;
  dev.profStrokes += s.strokes;
  dev.profTexts += s.texts;
  dev.profImages += s.images;
  dev.profVerts += s.verts;
}

void WireNavMapData(avionics::AvionicsEngine& engine) {
  if (!g_navMapData) return;
  engine.softkeyController().setNavFeatureSource(g_navMapData.get());
  engine.mfdController().setNavFeatureSource(g_navMapData.get());
  // WPT - Weather Information reads X-Plane's downloaded METAR files via the
  // data source's station-weather store.
  if (g_dataSource) {
    engine.mfdController().setStationWeatherSource(
        &g_dataSource->stationWeatherSource());
  }
}

void WireSoftkeyPeers() {
  if (!g_pfd.engine || !g_mfd.engine) return;
  g_pfd.engine->setSoftkeyPeer(g_mfd.engine.get());
  g_mfd.engine->setSoftkeyPeer(g_pfd.engine.get());
}

void ApplyQueuedFlightPlanEdits();

int DrawDevice(AvionicsDevice& dev) {
  InvalidateAvionicsCacheIfMapGeometryChanged();
  if (!g_dataSource || dev.rendererFailed) return 1;

  // This MFD-state sync feeds the shared data source's map/weather queries and
  // is independent of which screen is drawing, so run it once per frame on the
  // source-driving device (PFD) rather than on both draw callbacks. Doing it on
  // both was pure duplicate main-thread work every frame.
  if (dev.drivesSource && g_dataSource && g_mfd.engine) {
    avionics::MfdController& ui = g_mfd.engine->mfdController();
    // Hide the dedicated Weather Radar page on airframes with no radar fit; the
    // NEXRAD map overlay is independent and stays available.
    ui.setWeatherRadarAvailable(g_dataSource->weatherRadarEquipped());
    g_dataSource->syncMapRangeFromSim(ui);
    // Keep the moving-map feature/airspace/land queries centered on the panned
    // map view while panning so the on-screen area loads data, not just around
    // ownship or the pointer geo alone.
    g_dataSource->syncWeatherRadar(ui);
    ui.applyMapPanToDataSource(*g_dataSource, g_dataSource->snapshot());
    ui.applyDirectToInsetToDataSource(*g_dataSource,
                                      g_dataSource->mapSnapshot());
  }

  if (!dev.engine) {
    // NanoVG's GL backend is created here, where X-Plane has made the device's
    // GL context current. The bridge is OpenGL 2.1, so use the GL2 backend.
    // Resolve GL entry points first (no-op on macOS; GLEW on Windows/Linux).
    avionics::render::ensureGlLoaded();
    dev.renderer = std::make_unique<avionics::NanoVgRenderer>(
        avionics::NanoVgRenderer::Backend::GL2);
    if (!dev.renderer->valid()) {
      dev.renderer.reset();
      dev.rendererFailed = true;  // don't retry every frame
      return 1;
    }
    dev.engine = std::make_unique<avionics::AvionicsEngine>(
        *g_dataSource, *dev.renderer, kSourceLabel);
    dev.engine->setPage(dev.page);
    dev.engine->setDrivesDataSource(dev.drivesSource);
    // PFD skips the logo self-test when the plugin loads mid-flight; the MFD
    // always shows the power-up page until the pilot presses ENT.
    if (dev.page == avionics::DisplayPage::PrimaryFlightDisplay) {
      dev.engine->skipBoot();
    }
    // Restore the durable display preferences saved on the last run so the
    // glass comes up the way the pilot left it (e.g. PFD inset map on/off).
    if (dev.page == avionics::DisplayPage::PrimaryFlightDisplay) {
      avionics::applyPfdState(dev.engine->softkeyController(),
                              g_avionicsState.pfd);
    } else {
      avionics::applyMfdState(dev.engine->mfdController(), g_avionicsState.mfd);
    }
    WireNavMapData(*dev.engine);
    WireSoftkeyPeers();
    dev.lastRenderElapsed = XPLMGetElapsedTime();
  }

  // X-Plane has bound the device's screen framebuffer and set the viewport to
  // its native pixel size; render into exactly that region.
  GLint viewport[4] = {0, 0, 0, 0};
  glGetIntegerv(GL_VIEWPORT, viewport);
  const int width = viewport[2] > 0 ? viewport[2] : kFallbackScreenW;
  const int height = viewport[3] > 0 ? viewport[3] : kFallbackScreenH;

  // The heavy scene renders into the cache at a fraction of native resolution
  // (fill-rate is the cost), then the per-frame blit upscales it to the device
  // screen. The PFD keeps renderScale 1.0; the MFD downsamples per preset.
  const float scale = dev.renderScale > 0.05f ? dev.renderScale : 1.0f;
  const bool downsample = scale < 0.999f;
  int renderW = downsample ? static_cast<int>(width * scale + 0.5f) : width;
  int renderH = downsample ? static_cast<int>(height * scale + 0.5f) : height;
  if (renderW < 1) renderW = 1;
  if (renderH < 1) renderH = 1;

  const float now = XPLMGetElapsedTime();

  if (dev.drivesSource) {
    ApplyQueuedFlightPlanEdits();
  }

  // Direct fallback: if the offscreen cache can't be set up, render the scene
  // straight to the device framebuffer every frame (correct, just not capped).
  if (dev.cacheDisabled || !EnsureCache(dev, renderW, renderH, downsample)) {
    dev.cacheDisabled = true;
    double dt = static_cast<double>(now - dev.lastRenderElapsed);
    if (dt < 0.0) dt = 0.0;
    dev.lastRenderElapsed = now;
    XPLMSetGraphicsState(/*fog=*/0, /*texUnits=*/1, /*lighting=*/0,
                         /*alphaTest=*/0, /*alphaBlend=*/1, /*depthTest=*/0,
                         /*depthWrite=*/0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    dev.engine->update(dt);
    dev.engine->renderFrame(width, height, 1.0f);
    AccumulateDrawStats(dev);
    ++dev.profRenders;
    return 0;
  }

  // Frame-count divisor: re-render the full scene into the cache every Nth
  // callback (and always on the first frame after a resize, when the cache has
  // no content yet). On the other frames we fall through to a cheap blit of the
  // last cached frame. Unlike a time cap, this keeps skipping work even when the
  // sim is slow, which is exactly when we need to. Each display keeps a steady,
  // independent cadence (deferring renders to balance per-frame load made the
  // refresh interval irregular, which reads as judder).
  // Two independent throttles combine so the glass redraws at
  // min(sim fps / N, maxRenderHz): the frame-count divisor keeps skipping work
  // when the sim is slow (a time cap can't, since every dt already exceeds it),
  // while the absolute Hz cap stops the divisor from redrawing far too often
  // when the sim is fast. The first frame (no cached content) always renders.
  const bool divisorAllows = (dev.frameCounter % dev.renderEveryN) == 0;
  bool hzCapAllows = true;
  if (dev.maxRenderHz > 0.0f) {
    const float minInterval = 1.0f / dev.maxRenderHz;
    hzCapAllows = (now - dev.lastRenderElapsed) >= minInterval;
  }
  const bool doRender = !dev.cacheReady || (divisorAllows && hzCapAllows);
  ++dev.frameCounter;
  if (doRender) {
    double dt = static_cast<double>(now - dev.lastRenderElapsed);
    if (dt < 0.0) dt = 0.0;
    RenderSceneToCache(dev, renderW, renderH, dt, viewport);
    dev.lastRenderElapsed = now;
    AccumulateDrawStats(dev);
    ++dev.profRenders;
  }

  // Every sim frame: cheap blit of the cached texture onto the device screen.
  BlitCache(dev, width, height);
  return 0;  // we drew the screen; suppress X-Plane's stock G1000
}

// Times DrawDevice and logs the average per-frame cost (and how many of those
// frames did a full scene re-render vs. a cheap cached blit) every few seconds,
// so we can see the real cost and whether the render cache is active.
int RunDevice(AvionicsDevice& dev) {
  const auto t0 = std::chrono::high_resolution_clock::now();
  const int rc = DrawDevice(dev);
  const auto t1 = std::chrono::high_resolution_clock::now();
  dev.profAccumMs +=
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  ++dev.profSamples;

  if (!dev.modeLogged && dev.engine) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "G1000 NXi %s: render path = %s\n", dev.label,
                  dev.cacheDisabled ? "DIRECT (FBO unavailable, every frame)"
                                    : "CACHED (offscreen FBO, capped)");
    Log(buf);
    dev.modeLogged = true;
  }

  const float now = XPLMGetElapsedTime();
  if (dev.profLastLog == 0.0f) dev.profLastLog = now;
  if (now - dev.profLastLog >= 5.0f && dev.profSamples > 0) {
    const int renders = dev.profRenders > 0 ? dev.profRenders : 1;
    char buf[320];
    std::snprintf(
        buf, sizeof(buf),
        "G1000 NXi %s: %.3f ms/frame avg over %d frames, %d scene re-renders; "
        "render %dx%d; per-render: %ld fill, %ld stroke, %ld text, %ld img, "
        "%ld verts\n",
        dev.label, dev.profAccumMs / dev.profSamples, dev.profSamples,
        dev.profRenders, dev.texW, dev.texH, dev.profFills / renders,
        dev.profStrokes / renders, dev.profTexts / renders,
        dev.profImages / renders, dev.profVerts / renders);
    Log(buf);
    dev.profAccumMs = 0.0;
    dev.profSamples = 0;
    dev.profRenders = 0;
    dev.profFills = 0;
    dev.profStrokes = 0;
    dev.profTexts = 0;
    dev.profImages = 0;
    dev.profVerts = 0;
    dev.profLastLog = now;
  }
  return rc;
}

int PfdDrawCallback(XPLMDeviceID /*device*/, int isBefore, void* /*ref*/) {
  if (!isBefore) return 1;
  if (!g_replaceDisplays) return 1;
  return RunDevice(g_pfd);
}

int MfdDrawCallback(XPLMDeviceID /*device*/, int isBefore, void* /*ref*/) {
  if (!isBefore) return 1;
  if (!g_replaceDisplays) return 1;
  return RunDevice(g_mfd);
}

void RegisterDevice(AvionicsDevice& dev, XPLMDeviceID deviceId,
                    XPLMAvionicsCallback_f callback) {
  XPLMCustomizeAvionics_t params;
  std::memset(&params, 0, sizeof(params));
  params.structSize = sizeof(params);
  params.deviceId = deviceId;
  params.drawCallbackBefore = callback;  // returns 0 to replace stock drawing
  params.drawCallbackAfter = nullptr;
  params.refcon = &dev;
  dev.handle = XPLMRegisterAvionicsCallbacksEx(&params);
}

// ---- G1000 bezel / softkey commands ------------------------------------------
//
// Two ways to drive the bezel from hardware/keyboard:
//
//  1. We intercept X-Plane's stock GDU commands (sim/GPS/g1000nN_*) so a cockpit
//     or controller already bound to the stock G1000 keeps working. Since we've
//     taken over the screen, we consume them so the hidden stock G1000 doesn't
//     diverge. n1 is the pilot PFD, n3 is the MFD (n2 is the copilot PFD, which
//     we don't drive).
//  2. We also CREATE our own commands (xplane_avionics/pfd|mfd/*) so every key
//     is bindable from X-Plane's Keyboard/Joystick UI even in aircraft that
//     don't expose the full stock G1000 command set. These appear under
//     "G1000 NXi" and behave identically to the intercepted stock keys.

// Custom command namespace + UI category for the commands we create. X-Plane
// shows the category text beside each binding in its settings list.
constexpr const char* kCmdPrefix = "xplane_avionics";

// Named GDU key -> our BezelKey. `suffix` is the stock command suffix; `label`
// is shown in X-Plane's binding UI for the custom command. Softkeys 1..12 are
// handled separately as a numeric range (softkeyN -> pressSoftkey(N-1)).
struct NamedKey {
  const char* suffix;
  avionics::BezelKey key;
  const char* label;
};
const NamedKey kNamedKeys[] = {
    {"direct", avionics::BezelKey::DirectTo, "Direct-To"},
    {"menu", avionics::BezelKey::Menu, "MENU key"},
    {"fpl", avionics::BezelKey::Fpl, "FPL key"},
    {"proc", avionics::BezelKey::Proc, "PROC key"},
    {"clr", avionics::BezelKey::Clr, "CLR key (hold for Default Map)"},
    {"ent", avionics::BezelKey::Ent, "ENT key"},
    {"cursor", avionics::BezelKey::FmsPush, "FMS knob push (cursor)"},
    {"fms_outer_up", avionics::BezelKey::FmsOuterCw, "FMS outer knob clockwise"},
    {"fms_outer_down", avionics::BezelKey::FmsOuterCcw,
     "FMS outer knob counter-clockwise"},
    {"fms_inner_up", avionics::BezelKey::FmsInnerCw, "FMS inner knob clockwise"},
    {"fms_inner_down", avionics::BezelKey::FmsInnerCcw,
     "FMS inner knob counter-clockwise"},
    {"range_up", avionics::BezelKey::RangeUp, "RANGE knob out (zoom out)"},
    {"range_down", avionics::BezelKey::RangeDown, "RANGE knob in (zoom in)"},
    // RANGE joystick map panning (X-Plane g1000nN_pan_*). The center push
    // activates the Map Pointer; the cardinal moves pan it.
    {"pan_push", avionics::BezelKey::PanPush, "RANGE joystick push (pan)"},
    {"pan_up", avionics::BezelKey::PanUp, "Map pan up"},
    {"pan_down", avionics::BezelKey::PanDown, "Map pan down"},
    {"pan_left", avionics::BezelKey::PanLeft, "Map pan left"},
    {"pan_right", avionics::BezelKey::PanRight, "Map pan right"},
};

// Diagonal joystick pushes (X-Plane g1000nN_pan_up_left, etc.) have no single
// BezelKey; each fires the two cardinal pans it combines.
struct DiagonalKey {
  const char* suffix;
  avionics::BezelKey a;
  avionics::BezelKey b;
  const char* label;
};
const DiagonalKey kDiagonalKeys[] = {
    {"pan_up_left", avionics::BezelKey::PanUp, avionics::BezelKey::PanLeft,
     "Map pan up-left"},
    {"pan_up_right", avionics::BezelKey::PanUp, avionics::BezelKey::PanRight,
     "Map pan up-right"},
    {"pan_down_left", avionics::BezelKey::PanDown, avionics::BezelKey::PanLeft,
     "Map pan down-left"},
    {"pan_down_right", avionics::BezelKey::PanDown, avionics::BezelKey::PanRight,
     "Map pan down-right"},
};

// ---- dedicated NAV/COM knob commands ----
// The cockpit COM/NAV knobs fire their own X-Plane commands (g1000nN_com12 /
// nav12 toggle, com/nav inner/outer tuning rings, com/nav flip-flop) rather
// than the FMS knob set above. We route every GDU's knobs (PFD n1 and MFD n3)
// to the PFD engine, which owns the NAV/COM bar state.
using RadioAction = avionics::cmdbridge::RadioAction;

struct RadioCommand {
  const char* suffix;
  RadioAction action;
  const char* label;
};
const RadioCommand kRadioCommands[] = {
    {"com12", RadioAction::ComToggle, "COM select (COM1/COM2)"},
    {"com_ff", RadioAction::ComFlip, "COM flip-flop"},
    {"com_outer_up", RadioAction::ComOuterUp, "COM outer knob up (MHz)"},
    {"com_outer_down", RadioAction::ComOuterDown, "COM outer knob down (MHz)"},
    {"com_inner_up", RadioAction::ComInnerUp, "COM inner knob up (kHz)"},
    {"com_inner_down", RadioAction::ComInnerDown, "COM inner knob down (kHz)"},
    {"nav12", RadioAction::NavToggle, "NAV select (NAV1/NAV2)"},
    {"nav_ff", RadioAction::NavFlip, "NAV flip-flop"},
    {"nav_outer_up", RadioAction::NavOuterUp, "NAV outer knob up (MHz)"},
    {"nav_outer_down", RadioAction::NavOuterDown, "NAV outer knob down (MHz)"},
    {"nav_inner_up", RadioAction::NavInnerUp, "NAV inner knob up (kHz)"},
    {"nav_inner_down", RadioAction::NavInnerDown, "NAV inner knob down (kHz)"},
    // COM/NAV audio volume (X-Plane g1000nN_cvol_* / nvol_*).
    {"cvol_up", RadioAction::ComVolUp, "COM volume up"},
    {"cvol_dn", RadioAction::ComVolDown, "COM volume down"},
    {"nvol_up", RadioAction::NavVolUp, "NAV volume up"},
    {"nvol_dn", RadioAction::NavVolDown, "NAV volume down"},
    {"nvol", RadioAction::NavVolPush, "NAV ident toggle (VOL/ID push)"},
};

struct RadioCommandBinding {
  RadioAction action;
  XPLMCommandRef cmd;
};
std::vector<RadioCommandBinding> g_radioBindings;

// ---- GCU 478 control unit (sim/GPS/gcu478/*) --------------------------------
//
// The optional Garmin GCU 478 keypad is a separate hardware control unit that
// drives the same avionics. X-Plane exposes a built-in command set for it,
// which we intercept just like the GDU keys so a bound GCU (or a hardware
// replica) works against our glass. The GCU is the FMS / map controller, so
// its map, pan, Direct-To, cursor and FMS-menu keys are routed to the MFD engine; its
// COM/NAV tuning and flip-flop go to the PFD engine's radio bar. The alphanumeric
// keypad (A-Z/0-9/dot/backspace) types into the active waypoint-ident entry.

// GCU keys that map straight onto an existing BezelKey on the MFD engine. These
// reuse the same CommandBinding / G1000CommandHandler path as the GDU keys
// (including the CLR press-and-hold "Default Map" behavior).
const NamedKey kGcuNamedKeys[] = {
    {"range_up", avionics::BezelKey::RangeUp, "GCU RANGE out (zoom out)"},
    {"range_down", avionics::BezelKey::RangeDown, "GCU RANGE in (zoom in)"},
    {"pan_push", avionics::BezelKey::PanPush, "GCU joystick push (pan)"},
    {"pan_up", avionics::BezelKey::PanUp, "GCU map pan up"},
    {"pan_down", avionics::BezelKey::PanDown, "GCU map pan down"},
    {"pan_left", avionics::BezelKey::PanLeft, "GCU map pan left"},
    {"pan_right", avionics::BezelKey::PanRight, "GCU map pan right"},
    {"direct", avionics::BezelKey::DirectTo, "GCU Direct-To"},
    {"menu", avionics::BezelKey::Menu, "GCU MENU"},
    {"fpl", avionics::BezelKey::Fpl, "GCU FPL"},
    {"proc", avionics::BezelKey::Proc, "GCU PROC"},
    {"clr", avionics::BezelKey::Clr, "GCU CLR (hold for Default Map)"},
    {"ent", avionics::BezelKey::Ent, "GCU ENT"},
    {"cursor", avionics::BezelKey::FmsPush, "GCU cursor (FMS push)"},
};

// GCU joystick diagonals (each fires the two cardinal pans it combines).
const DiagonalKey kGcuDiagonalKeys[] = {
    {"pan_up_left", avionics::BezelKey::PanUp, avionics::BezelKey::PanLeft,
     "GCU map pan up-left"},
    {"pan_up_right", avionics::BezelKey::PanUp, avionics::BezelKey::PanRight,
     "GCU map pan up-right"},
    {"pan_down_left", avionics::BezelKey::PanDown, avionics::BezelKey::PanLeft,
     "GCU map pan down-left"},
    {"pan_down_right", avionics::BezelKey::PanDown, avionics::BezelKey::PanRight,
     "GCU map pan down-right"},
};

// The GCU has a single dual-concentric knob whose target is chosen by the FMS /
// COM / NAV / XPDR keys (one knob shared across all four functions). We track
// the selected target and route each inner/outer detent to the matching engine
// action: FMS turns the MFD's FMS knob; COM/NAV tune the PFD radio bar.
enum class GcuKnobMode { Fms, Com, Nav, Xpdr };
GcuKnobMode g_gcuKnobMode = GcuKnobMode::Fms;

enum class GcuKnobAction {
  OuterUp,
  OuterDown,
  InnerUp,
  InnerDown,
  ModeFms,
  ModeCom,
  ModeNav,
  ModeXpdr,
  FlipFlop,
};

struct GcuKnobCommand {
  const char* suffix;
  GcuKnobAction action;
};
const GcuKnobCommand kGcuKnobCommands[] = {
    {"outer_up", GcuKnobAction::OuterUp},
    {"outer_down", GcuKnobAction::OuterDown},
    {"inner_up", GcuKnobAction::InnerUp},
    {"inner_down", GcuKnobAction::InnerDown},
    {"fms", GcuKnobAction::ModeFms},
    {"com", GcuKnobAction::ModeCom},
    {"nav", GcuKnobAction::ModeNav},
    {"xpdr", GcuKnobAction::ModeXpdr},
    {"ff", GcuKnobAction::FlipFlop},
};

struct GcuKnobBinding {
  GcuKnobAction action;
  XPLMCommandRef cmd;
};
std::vector<GcuKnobBinding> g_gcuKnobBindings;

struct GcuBezelBinding {
  avionics::BezelKey key = avionics::BezelKey::Count;
  avionics::BezelKey key2 = avionics::BezelKey::Count;
  XPLMCommandRef cmd = nullptr;
  float holdStart = 0.0f;
  bool holdFired = false;
};
std::vector<GcuBezelBinding> g_gcuBezelBindings;

struct GcuKeypadBinding {
  char ch = '\0';
  XPLMCommandRef cmd = nullptr;
};
std::vector<GcuKeypadBinding> g_gcuKeypadBindings;

// One intercepted command: which device engine it targets and what to press.
struct CommandBinding {
  AvionicsDevice* dev;
  bool isSoftkey;
  int value;   // softkey index when isSoftkey, else BezelKey cast to int
  int value2;  // second BezelKey for diagonal pans, or -1
  XPLMCommandRef cmd;
  // CLR press-and-hold tracking (CLR DFLT MAP): set on the key-down edge, fired
  // once the button has been held past kClrDefaultMapHoldSeconds.
  float holdStart = 0.0f;
  bool holdFired = false;
};

bool GduFmsKeyBlockedByPfd(const CommandBinding& b);

// Stable storage: refcons point into this, so it must not reallocate after the
// handlers are registered (filled once, then left alone until Disable).
std::vector<CommandBinding> g_commandBindings;

// Captures the durable display preferences from whichever device engines exist
// and rewrites the config file if they changed. Called after a GDU key press,
// so any softkey/bezel change to a persisted option (inset map, ranges,
// declutter, etc.) survives the next launch.
void PersistStateIfChanged() {
  avionics::AvionicsPersistentState current = g_avionicsState;
  if (g_pfd.engine) {
    avionics::capturePfdState(g_pfd.engine->softkeyController(), current.pfd);
  }
  if (g_mfd.engine) {
    avionics::captureMfdState(g_mfd.engine->mfdController(), current.mfd);
  }
  if (current != g_avionicsState) {
    g_avionicsState = current;
    SaveConfig();
  }
}

// Drains the NAV/COM tuning and transponder commands the PFD softkey controller
// queues on a bezel/softkey press and writes them back to the sim's radio
// datarefs (the standalone shell does this against its UDP link; in-process we
// write the datarefs directly). Without this the bezel tunes the on-screen
// standby readout but nothing reaches the radios. NAV/COM tuning is a PFD-only
// bezel function, so only the PFD engine is drained.
void ApplyQueuedRadioCommands() {
  if (!g_dataSource || !g_pfd.engine) return;
  avionics::SoftkeyController& sk = g_pfd.engine->softkeyController();

  avionics::RadioUnit unit;
  float standbyMhz = 0.0f;
  if (sk.consumeRadioTune(unit, standbyMhz)) {
    g_dataSource->tuneRadioStandby(unit, standbyMhz);
  }
  if (sk.consumeRadioTransfer(unit)) {
    g_dataSource->transferRadio(unit);
  }
  float volume = 0.0f;
  if (sk.consumeRadioVolume(unit, volume)) {
    g_dataSource->setRadioVolume(unit, volume);
  }
  bool identOn = false;
  if (sk.consumeNavIdent(unit, identOn)) {
    g_dataSource->setNavIdent(unit, identOn);
  }

  int xpdrCode = 0;
  if (sk.consumeXpdrCodeCommit(xpdrCode)) {
    g_dataSource->setTransponderCode(xpdrCode);
  }
  int xpdrMode = 0;
  if (sk.consumeXpdrModeCommit(xpdrMode)) {
    g_dataSource->setTransponderMode(xpdrMode);
  }
}

// Drains flight-plan edits from both GDUs. Partial routes stay on the in-plugin
// display feed only; the sim FMS is programmed once the route is complete.
void ApplyQueuedFlightPlanEdits() {
  if (!g_dataSource) return;

  auto applyPlan = [](const std::vector<avionics::MapLeg>& plan,
                      bool destinationFilled) {
    if (plan.empty()) {
      // Authoritative empty plan (deleted FPL) — do not fall back to .fms.
      g_dataSource->setRouteOverride({});
      return;
    }
    if (avionics::flightPlanReadyForSimulator(plan, destinationFilled)) {
      g_dataSource->setRouteOverride(plan);
    } else {
      g_dataSource->setLocalFlightPlan(plan);
    }
  };

  if (g_pfd.engine) {
    std::vector<avionics::MapLeg> pfdEditedPlan;
    if (g_pfd.engine->softkeyController().consumeFlightPlanEdit(pfdEditedPlan)) {
      applyPlan(pfdEditedPlan,
                g_pfd.engine->softkeyController().flightPlanDestinationFilled());
    }
    avionics::MapLeg pfdDto;
    bool pfdDtoHold = false;
    if (g_pfd.engine->softkeyController().consumeDirectToRequest(pfdDto,
                                                               &pfdDtoHold)) {
      g_dataSource->setDirectTo(pfdDto, pfdDtoHold);
    }
    avionics::MapProcedure pfdProc;
    if (g_pfd.engine->softkeyController().consumeProcLoadRequest(pfdProc) &&
        pfdProc.frequencyMhz > 0.0f) {
      g_dataSource->tuneRadioStandby(avionics::RadioUnit::Nav1,
                                    pfdProc.frequencyMhz);
    }
  }

  if (g_mfd.engine) {
    std::vector<avionics::MapLeg> editedPlan;
    if (g_mfd.engine->mfdController().consumeFlightPlanEdit(editedPlan)) {
      applyPlan(editedPlan,
                g_mfd.engine->mfdController().fplDestinationFilled());
    }
    avionics::MapLeg dtoTarget;
    bool dtoHold = false;
    if (g_mfd.engine->mfdController().consumeDirectToRequest(dtoTarget,
                                                            &dtoHold)) {
      g_dataSource->setDirectTo(dtoTarget, dtoHold);
    }
    avionics::MapProcedure proc;
    if (g_mfd.engine->mfdController().consumeProcLoadRequest(proc) &&
        proc.frequencyMhz > 0.0f) {
      g_dataSource->tuneRadioStandby(avionics::RadioUnit::Nav1,
                                    proc.frequencyMhz);
    }
  }
}

// Drives the Navigraph sign-in + SimBrief OFP import each frame: reacts to the
// AUX - SIMBRIEF softkeys, persists the rotating refresh token, lands completed
// fetches into the displayed/active flight plan, and publishes status for the
// page renderer. Mirrors the standalone shell's pump.
void PumpNavigraph() {
  if (!g_navigraph) return;

  if (g_mfd.engine) {
    avionics::MfdController& ui = g_mfd.engine->mfdController();
    if (ui.consumeNavigraphLoginRequest()) g_navigraph->requestLogin();
    if (ui.consumeNavigraphLogoutRequest()) {
      g_navigraph->requestLogout();
      g_simbriefState.status = avionics::SimBriefStatus::NotConfigured;
    }
    if (ui.consumeSimbriefFetchRequest() && !g_navigraph->fetching()) {
      g_navigraph->requestFetch();
    }
  }

  std::string newToken;
  if (g_navigraph->consumeRefreshToken(newToken) &&
      newToken != g_navigraphRefreshToken) {
    g_navigraphRefreshToken = newToken;
    avionics::SaveNavigraphRefreshToken(newToken);
  }

  const avionics::NavigraphAuthSnapshot snap = g_navigraph->snapshot();
  avionics::NavigraphLoginPhase phase = avionics::NavigraphLoginPhase::LoggedOut;
  switch (snap.phase) {
    case avionics::NavigraphAuthPhase::LoggedOut:
      phase = avionics::NavigraphLoginPhase::LoggedOut;
      break;
    case avionics::NavigraphAuthPhase::AwaitingUser:
      phase = avionics::NavigraphLoginPhase::AwaitingUser;
      break;
    case avionics::NavigraphAuthPhase::LoggedIn:
      phase = avionics::NavigraphLoginPhase::LoggedIn;
      break;
    case avionics::NavigraphAuthPhase::Error:
      phase = avionics::NavigraphLoginPhase::Error;
      break;
  }
  g_simbriefState.loginPhase = phase;
  g_simbriefState.username = snap.username;
  g_simbriefState.userCode = snap.userCode;
  g_simbriefState.verificationUri = snap.verificationUri;
  g_simbriefState.verificationUriComplete = snap.verificationUriComplete;
  g_simbriefState.loginError = snap.error;
  if (g_navigraph->fetching()) {
    g_simbriefState.status = avionics::SimBriefStatus::Fetching;
  } else if (phase != avionics::NavigraphLoginPhase::LoggedIn &&
             g_simbriefState.status != avionics::SimBriefStatus::Ok &&
             g_simbriefState.status != avionics::SimBriefStatus::Error) {
    g_simbriefState.status = avionics::SimBriefStatus::NotConfigured;
  } else if (phase == avionics::NavigraphLoginPhase::LoggedIn &&
             g_simbriefState.status ==
                 avionics::SimBriefStatus::NotConfigured) {
    g_simbriefState.status = avionics::SimBriefStatus::Idle;
  }

  avionics::SimBriefFetchResult result;
  if (g_navigraph->consumeResult(result)) {
    if (result.ok) {
      g_simbriefState.status = avionics::SimBriefStatus::Ok;
      g_simbriefState.error.clear();
      g_simbriefState.originIcao = result.originIcao;
      g_simbriefState.destinationIcao = result.destinationIcao;
      g_simbriefState.route = result.route;
      g_simbriefState.generatedUtc = result.generatedUtc;
      g_simbriefState.waypointCount = static_cast<int>(result.legs.size());
      if (g_dataSource) g_dataSource->setRouteOverride(result.legs);
      if (g_pfd.engine) {
        g_pfd.engine->softkeyController().replaceFlightPlanFromExternal(
            result.legs);
      }
      if (g_mfd.engine) {
        g_mfd.engine->mfdController().replaceFlightPlanFromExternal(
            result.legs);
      }
    } else {
      g_simbriefState.status = avionics::SimBriefStatus::Error;
      g_simbriefState.error = result.error;
    }
  }

  if (g_mfd.engine) {
    g_mfd.engine->mfdController().setSimbriefState(g_simbriefState);
  }

  // Navigraph charts (WPT - Airport Information chart view): in-sim plugin
  // always has a live link.
  if (g_mfd.engine && g_navigraph) {
    avionics::MfdController& mfdUi = g_mfd.engine->mfdController();
    const bool chartsActive =
        mfdUi.chartViewActive() &&
        g_simbriefState.loginPhase == avionics::NavigraphLoginPhase::LoggedIn;

    g_chartsState.commAllowed = true;
    g_chartsState.loginPhase = g_simbriefState.loginPhase;

    if (chartsActive) {
      const std::string apt = mfdUi.chartsDesiredAirport();
      g_navigraph->setChartAirport(apt);
      g_chartsState.airportIcao = apt;

      const avionics::ChartIndexSnapshot idx = g_navigraph->chartIndexSnapshot();
      const std::string chartId = mfdUi.chartsSelectedChartId();
      if (!chartId.empty()) {
        for (const avionics::NavigraphChartMeta& meta : idx.charts) {
          if (meta.id == chartId) {
            const std::string url = mfdUi.chartsNight() ? meta.imageNightUrl
                                                        : meta.imageDayUrl;
            g_navigraph->setChartSelection(chartId, mfdUi.chartsNight(), url);
            break;
          }
        }
      }

      switch (idx.status) {
        case avionics::ChartIndexStatus::Idle:
          g_chartsState.status = avionics::ChartsStatus::Idle;
          break;
        case avionics::ChartIndexStatus::Loading:
          g_chartsState.status = avionics::ChartsStatus::Loading;
          break;
        case avionics::ChartIndexStatus::Ready:
          g_chartsState.status = avionics::ChartsStatus::Ready;
          break;
        case avionics::ChartIndexStatus::Empty:
          g_chartsState.status = avionics::ChartsStatus::Empty;
          break;
        case avionics::ChartIndexStatus::Error:
          g_chartsState.status = avionics::ChartsStatus::Error;
          break;
      }
      g_chartsState.error = idx.error;
      g_chartsState.charts.clear();
      for (const avionics::NavigraphChartMeta& meta : idx.charts) {
        g_chartsState.charts.push_back(avionics::NavigraphChartToListItem(meta));
      }
    } else {
      g_navigraph->setChartAirport("");
      g_chartsState.airportIcao.clear();
      g_chartsState.status = avionics::ChartsStatus::Idle;
      g_chartsState.error.clear();
      g_chartsState.charts.clear();
    }

    avionics::ChartImageResult imgResult;
    if (g_navigraph->consumeChartImage(imgResult)) {
      ++g_chartImageGeneration;
      g_chartsState.image.generation = g_chartImageGeneration;
      if (imgResult.ok && !imgResult.pngBytes.empty()) {
        g_chartsState.image.pngBytes =
            std::make_shared<const std::vector<unsigned char>>(
                std::move(imgResult.pngBytes));
      } else {
        g_chartsState.image.pngBytes.reset();
      }
    }

    mfdUi.setChartsState(g_chartsState);
  }
}

bool ForwardG1000Event(const CommandBinding& b, avionics::cmdbridge::Phase phase) {
  if (!g_commandBridge) return false;
  avionics::cmdbridge::Event ev;
  ev.device = (b.dev == &g_mfd) ? avionics::cmdbridge::Device::Mfd
                                : avionics::cmdbridge::Device::Pfd;
  ev.phase = phase;
  if (b.isSoftkey) {
    ev.kind = avionics::cmdbridge::Kind::Softkey;
    ev.value = b.value;
  } else if (b.value2 >= 0) {
    ev.kind = avionics::cmdbridge::Kind::BezelDiagonal;
    ev.value = b.value;
    ev.value2 = b.value2;
  } else {
    ev.kind = avionics::cmdbridge::Kind::Bezel;
    ev.value = b.value;
  }
  return g_commandBridge->sendEvent(ev);
}

bool ForwardRadioEvent(RadioAction action) {
  if (!g_commandBridge) return false;
  avionics::cmdbridge::Event ev;
  ev.kind = avionics::cmdbridge::Kind::Radio;
  ev.value = static_cast<std::int32_t>(action);
  return g_commandBridge->sendEvent(ev);
}

// Hardware autopilot panel (GFC700 etc.): forward to the networked standalone,
// which owns VNAV path guidance when using external PFD/MFD displays.
XPLMCommandRef g_apVnavCmd = nullptr;

int ApVnavCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                         void* /*ref*/) {
  if (phase != xplm_CommandBegin) return 0;
  if (g_commandBridge == nullptr) return 0;
  avionics::cmdbridge::Event ev;
  ev.phase = avionics::cmdbridge::Phase::Begin;
  ev.kind = avionics::cmdbridge::Kind::Autopilot;
  ev.value =
      static_cast<std::int32_t>(avionics::cmdbridge::AutopilotAction::Vnav);
  return g_commandBridge->sendEvent(ev) ? 1 : 0;
}

void RegisterAutopilotCommands() {
  g_apVnavCmd = XPLMFindCommand("sim/autopilot/vnav");
  if (g_apVnavCmd != nullptr) {
    XPLMRegisterCommandHandler(g_apVnavCmd, &ApVnavCommandHandler, /*before=*/1,
                               nullptr);
  }
}

void UnregisterAutopilotCommands() {
  if (g_apVnavCmd != nullptr) {
    XPLMUnregisterCommandHandler(g_apVnavCmd, &ApVnavCommandHandler,
                                 /*before=*/1, nullptr);
    g_apVnavCmd = nullptr;
  }
}

int G1000CommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                        void* ref) {
  // Route GDU keys to our in-sim engines when active and/or forward them to the
  // networked standalone shell. Consume when we handle or forward the key so
  // the stock G1000 does not diverge; pass through when neither applies.
  auto* b = static_cast<CommandBinding*>(ref);
  if (b == nullptr) return 1;
  const bool isClr =
      !b->isSoftkey && b->value == static_cast<int>(avionics::BezelKey::Clr);
  if (GduFmsKeyBlockedByPfd(*b)) {
    if (phase == xplm_CommandEnd && isClr) b->holdFired = false;
    return 0;
  }
  const bool hasEngine = b->dev->engine != nullptr;
  bool forwarded = false;
  bool applied = false;

  if (phase == xplm_CommandBegin) {
    forwarded = ForwardG1000Event(*b, avionics::cmdbridge::Phase::Begin);
    if (hasEngine) {
      if (b->isSoftkey) {
        b->dev->engine->pressSoftkey(b->value);
      } else {
        b->dev->engine->pressBezelKey(
            static_cast<avionics::BezelKey>(b->value));
        if (b->value2 >= 0) {
          b->dev->engine->pressBezelKey(
              static_cast<avionics::BezelKey>(b->value2));
        }
      }
      if (isClr) {
        b->holdStart = XPLMGetElapsedTime();
        b->holdFired = false;
      }
      ApplyQueuedRadioCommands();
      PersistStateIfChanged();
      applied = true;
    } else if (forwarded) {
      applied = true;
    }
  } else if (phase == xplm_CommandContinue) {
    // CLR held past the threshold acts as CLR (DFLT MAP): jump to the MFD
    // Navigation Map page. Fire once per hold.
    if (isClr && !b->holdFired &&
        XPLMGetElapsedTime() - b->holdStart >=
            static_cast<float>(avionics::kClrDefaultMapHoldSeconds)) {
      forwarded =
          ForwardG1000Event(*b, avionics::cmdbridge::Phase::Continue);
      if (hasEngine) {
        b->dev->engine->holdBezelKey(avionics::BezelKey::Clr);
        b->holdFired = true;
        PersistStateIfChanged();
        applied = true;
      } else if (forwarded) {
        b->holdFired = true;
        applied = true;
      }
    }
  } else if (phase == xplm_CommandEnd) {
    if (isClr) {
      forwarded = ForwardG1000Event(*b, avionics::cmdbridge::Phase::End);
      b->holdFired = false;
      if (forwarded) applied = true;
    }
  }

  return (forwarded || applied) ? 0 : 1;
}

// Applies one NAV/COM knob action to the PFD engine (which owns the bar) and
// drains the resulting tune/transfer into the radio datarefs. The COM/NAV outer
// ring steps whole MHz (coarse); the inner ring steps one channel. No-op when
// the PFD engine is not up.
void ApplyRadioAction(RadioAction action) {
  if (!g_pfd.engine) return;
  avionics::AvionicsEngine& e = *g_pfd.engine;
  switch (action) {
    case RadioAction::ComToggle:
      e.selectComRadio();
      break;
    case RadioAction::ComFlip:
      e.transferComRadio();
      break;
    case RadioAction::ComOuterUp:
      e.tuneComRadio(+1, /*coarse=*/true);
      break;
    case RadioAction::ComOuterDown:
      e.tuneComRadio(-1, /*coarse=*/true);
      break;
    case RadioAction::ComInnerUp:
      e.tuneComRadio(+1, /*coarse=*/false);
      break;
    case RadioAction::ComInnerDown:
      e.tuneComRadio(-1, /*coarse=*/false);
      break;
    case RadioAction::NavToggle:
      e.selectNavRadio();
      break;
    case RadioAction::NavFlip:
      e.transferNavRadio();
      break;
    case RadioAction::NavOuterUp:
      e.tuneNavRadio(+1, /*coarse=*/true);
      break;
    case RadioAction::NavOuterDown:
      e.tuneNavRadio(-1, /*coarse=*/true);
      break;
    case RadioAction::NavInnerUp:
      e.tuneNavRadio(+1, /*coarse=*/false);
      break;
    case RadioAction::NavInnerDown:
      e.tuneNavRadio(-1, /*coarse=*/false);
      break;
    case RadioAction::ComVolUp:
      e.pressBezelKey(avionics::BezelKey::ComVolCw);
      break;
    case RadioAction::ComVolDown:
      e.pressBezelKey(avionics::BezelKey::ComVolCcw);
      break;
    case RadioAction::NavVolUp:
      e.pressBezelKey(avionics::BezelKey::NavVolCw);
      break;
    case RadioAction::NavVolDown:
      e.pressBezelKey(avionics::BezelKey::NavVolCcw);
      break;
    case RadioAction::NavVolPush:
      e.pressBezelKey(avionics::BezelKey::NavVolPush);
      break;
  }
  ApplyQueuedRadioCommands();
}

// Dispatches a NAV/COM knob command (the GDU's own COM/NAV knobs) to the PFD
// engine and forwards it to the networked standalone shell.
int RadioCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                        void* ref) {
  if (phase != xplm_CommandBegin) return 1;
  auto* b = static_cast<RadioCommandBinding*>(ref);
  if (b == nullptr) return 1;

  const bool forwarded = ForwardRadioEvent(b->action);
  bool applied = false;
  if (g_pfd.engine) {
    ApplyRadioAction(b->action);
    applied = true;
  }
  return (forwarded || applied) ? 0 : 1;
}

// Forwards the audio-panel DISPLAY BACKUP press to the networked standalone shell.
bool ForwardDisplayBackupEvent() {
  if (!g_commandBridge) return false;
  avionics::cmdbridge::Event ev;
  ev.device = avionics::cmdbridge::Device::Pfd;
  ev.phase = avionics::cmdbridge::Phase::Begin;
  ev.kind = avionics::cmdbridge::Kind::Bezel;
  ev.value = static_cast<std::int32_t>(avionics::BezelKey::DisplayBackup);
  return g_commandBridge->sendEvent(ev);
}

void ApplyDisplayBackup() {
  if (g_dataSource == nullptr) return;
  // Toggle once on the shared source so both GDUs see it; flash any live engines
  // for bezel feedback. Do not route through pressBezelKey here — that path
  // requires displayPowered() and can no-op when bus datarefs lag.
  g_dataSource->toggleDisplayBackup();
  auto flash = [](avionics::AvionicsEngine* engine) {
    if (engine == nullptr) return;
    engine->softkeyController().flashBezelKey(avionics::BezelKey::DisplayBackup);
    engine->mfdController().flashBezelKey(avionics::BezelKey::DisplayBackup);
  };
  flash(g_pfd.engine.get());
  flash(g_mfd.engine.get());
}

// GMA audio-panel red DISPLAY BACKUP key (sim/GPS/G1000_display_reversion).
int DisplayBackupCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                                void* /*ref*/) {
  if (phase != xplm_CommandBegin) return 1;
  const bool forwarded = ForwardDisplayBackupEvent();
  if (g_dataSource != nullptr) ApplyDisplayBackup();
  XPLMDebugString("G1000 NXi: DISPLAY BACKUP command received\n");
  return (forwarded || g_dataSource != nullptr) ? 0 : 1;
}

std::vector<XPLMCommandRef> g_displayBackupCommands;
XPLMCommandRef g_displayBackupCustomCmd = nullptr;

// Forward; defined with g_customStrings below.
const char* StoreStr(const std::string& s);

void RegisterDisplayBackupCommands() {
  g_displayBackupCommands.clear();
  if (XPLMCommandRef stock =
          XPLMFindCommand("sim/GPS/G1000_display_reversion")) {
    g_displayBackupCommands.push_back(stock);
  } else {
    XPLMDebugString(
        "G1000 NXi: sim/GPS/G1000_display_reversion not found; bind "
        "xplane_avionics/display_backup in Keyboard/Joystick settings\n");
  }
  if (g_displayBackupCustomCmd == nullptr) {
    g_displayBackupCustomCmd = XPLMCreateCommand(
        StoreStr(std::string(kCmdPrefix) + "/display_backup"),
        StoreStr("G1000 NXi: Display Backup (reversionary mode)"));
  }
  if (g_displayBackupCustomCmd != nullptr) {
    g_displayBackupCommands.push_back(g_displayBackupCustomCmd);
  }
  for (XPLMCommandRef cmd : g_displayBackupCommands) {
    XPLMRegisterCommandHandler(cmd, &DisplayBackupCommandHandler, /*before=*/1,
                               nullptr);
  }
}

// The stock command is always present in X-Plane 12, but re-check after an
// aircraft load in case registration was missed on an earlier pass.
void RefreshDisplayBackupStockCommand() {
  if (XPLMCommandRef stock =
          XPLMFindCommand("sim/GPS/G1000_display_reversion")) {
    for (XPLMCommandRef cmd : g_displayBackupCommands) {
      if (cmd == stock) return;
    }
    g_displayBackupCommands.push_back(stock);
    XPLMRegisterCommandHandler(stock, &DisplayBackupCommandHandler,
                               /*before=*/1, nullptr);
  }
}

void UnregisterDisplayBackupCommands() {
  for (XPLMCommandRef cmd : g_displayBackupCommands) {
    XPLMUnregisterCommandHandler(cmd, &DisplayBackupCommandHandler,
                                 /*before=*/1, nullptr);
  }
  g_displayBackupCommands.clear();
}

// Forwards a single bezel-key press for a device to the networked standalone
// shell (the in-sim engines are driven directly by the caller).
bool ForwardBezelEvent(AvionicsDevice& dev, avionics::BezelKey key) {
  if (!g_commandBridge) return false;
  avionics::cmdbridge::Event ev;
  ev.device = (&dev == &g_mfd) ? avionics::cmdbridge::Device::Mfd
                               : avionics::cmdbridge::Device::Pfd;
  ev.phase = avionics::cmdbridge::Phase::Begin;
  ev.kind = avionics::cmdbridge::Kind::Bezel;
  ev.value = static_cast<std::int32_t>(key);
  return g_commandBridge->sendEvent(ev);
}

// True when a PFD pop-up (Direct-To, Flight Plan, etc.) owns GCU FMS input.
bool PfdClaimsGcuFms() {
  if (!g_pfd.engine) return false;
  return g_pfd.engine->softkeyController().pfdClaimsFmsInput();
}

// MFD GDU ENT/CLR/FMS-push are inert while a PFD pop-up owns FMS input. The
// MFD's own FMS knob detents still step page groups/pages. ENT on the MFD still
// acknowledges the power-up page even when a PFD pop-up is open.
bool GduFmsKeyBlockedByPfd(const CommandBinding& b) {
  if (b.isSoftkey) return false;
  if (b.dev == &g_pfd) return false;
  const auto key = static_cast<avionics::BezelKey>(b.value);
  if (key == avionics::BezelKey::Ent && g_mfd.engine != nullptr &&
      g_mfd.engine->awaitingPowerUpAck()) {
    return false;
  }
  if (avionics::isMfdPageSelectionBezelKey(key)) return false;
  if (g_mfd.engine != nullptr &&
      g_mfd.engine->mfdController().ownsLocalFmsInput()) {
    return false;
  }
  if (!PfdClaimsGcuFms()) return false;
  return avionics::isGduFmsInputKey(key);
}

AvionicsDevice* GcuFmsBezelDevice() {
  return PfdClaimsGcuFms() ? &g_pfd : &g_mfd;
}

// Pick PFD vs MFD for a GCU bezel key (map/pan and Direct-To on MFD; FMS / ENT /
// CLR follow whichever display owns FMS input).
AvionicsDevice* ResolveGcuBezelDevice(avionics::BezelKey key) {
  switch (key) {
    case avionics::BezelKey::RangeUp:
    case avionics::BezelKey::RangeDown:
    case avionics::BezelKey::PanPush:
    case avionics::BezelKey::PanUp:
    case avionics::BezelKey::PanDown:
    case avionics::BezelKey::PanLeft:
    case avionics::BezelKey::PanRight:
    case avionics::BezelKey::DirectTo:
      return &g_mfd;
    case avionics::BezelKey::Fpl:
    case avionics::BezelKey::Proc:
    case avionics::BezelKey::Menu:
      return PfdClaimsGcuFms() ? &g_pfd : &g_mfd;
    case avionics::BezelKey::Ent:
      if (g_mfd.engine != nullptr && g_mfd.engine->awaitingPowerUpAck()) {
        return &g_mfd;
      }
      return GcuFmsBezelDevice();
    default:
      return GcuFmsBezelDevice();
  }
}

bool ForwardGcuBezelEvent(const GcuBezelBinding& b,
                          avionics::cmdbridge::Phase phase) {
  if (!g_commandBridge) return false;
  AvionicsDevice* dev = ResolveGcuBezelDevice(b.key);
  if (dev == nullptr) return false;
  avionics::cmdbridge::Event ev;
  ev.device = (dev == &g_mfd) ? avionics::cmdbridge::Device::Mfd
                              : avionics::cmdbridge::Device::Pfd;
  ev.phase = phase;
  if (b.key2 != avionics::BezelKey::Count) {
    ev.kind = avionics::cmdbridge::Kind::BezelDiagonal;
    ev.value = static_cast<std::int32_t>(b.key);
    ev.value2 = static_cast<std::int32_t>(b.key2);
  } else {
    ev.kind = avionics::cmdbridge::Kind::Bezel;
    ev.value = static_cast<std::int32_t>(b.key);
  }
  return g_commandBridge->sendEvent(ev);
}

bool IsMfdMapRangeKey(avionics::BezelKey key) {
  return key == avionics::BezelKey::RangeUp ||
         key == avionics::BezelKey::RangeDown;
}

// Applies a GCU/GDU range detent to the MFD map ladder. Returns true when the
// selected range step changed.
bool ApplyMfdMapRangeKey(avionics::BezelKey key) {
  if (!IsMfdMapRangeKey(key)) return false;
  const int direction = key == avionics::BezelKey::RangeUp ? +1 : -1;
  avionics::MfdController* ui =
      g_mfd.engine ? &g_mfd.engine->mfdController() : nullptr;
  float beforeNm = ui != nullptr ? ui->rangeNm() : -1.0f;

  if (g_mfd.engine != nullptr) {
    g_mfd.engine->pressBezelKey(key);
    if (ui != nullptr && ui->rangeNm() != beforeNm) {
      if (g_dataSource) {
        g_dataSource->setChartRangeNm(ui->rangeNm());
      }
      return true;
    }
    if (ui != nullptr && ui->stepMapRange(direction)) {
      if (g_dataSource) {
        g_dataSource->setChartRangeNm(ui->rangeNm());
      }
      return true;
    }
  }
  return false;
}

int GcuBezelCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                           void* ref) {
  auto* b = static_cast<GcuBezelBinding*>(ref);
  if (b == nullptr) return 1;
  AvionicsDevice* dev = ResolveGcuBezelDevice(b->key);
  const bool isClr = b->key == avionics::BezelKey::Clr;
  const bool hasEngine = dev != nullptr && dev->engine != nullptr;
  bool forwarded = false;
  bool applied = false;

  if (phase == xplm_CommandBegin) {
    if (IsMfdMapRangeKey(b->key) && ResolveGcuBezelDevice(b->key) == &g_mfd) {
      // Standalone window: forward over UDP first; the in-sim engine (when
      // present) is updated in parallel for pilots using both displays.
      forwarded = ForwardGcuBezelEvent(*b, avionics::cmdbridge::Phase::Begin);
      bool localChanged = ApplyMfdMapRangeKey(b->key);
      applied = forwarded || localChanged;
      if (!applied && g_dataSource != nullptr) {
        const int direction =
            b->key == avionics::BezelKey::RangeUp ? +1 : -1;
        avionics::MfdController* ui =
            g_mfd.engine ? &g_mfd.engine->mfdController() : nullptr;
        applied = g_dataSource->stepMapRangeFromSim(direction, ui);
      }
      if (applied) {
        ApplyQueuedRadioCommands();
        PersistStateIfChanged();
      }
    } else {
      forwarded = ForwardGcuBezelEvent(*b, avionics::cmdbridge::Phase::Begin);
      if (hasEngine) {
        dev->engine->pressBezelKey(b->key);
        if (b->key2 != avionics::BezelKey::Count) {
          dev->engine->pressBezelKey(b->key2);
        }
        if (isClr) {
          b->holdStart = XPLMGetElapsedTime();
          b->holdFired = false;
        }
        ApplyQueuedRadioCommands();
        PersistStateIfChanged();
        applied = true;
      } else if (forwarded) {
        applied = true;
      }
    }
  } else if (phase == xplm_CommandContinue) {
    if (isClr && !b->holdFired && dev == &g_mfd &&
        XPLMGetElapsedTime() - b->holdStart >=
            static_cast<float>(avionics::kClrDefaultMapHoldSeconds)) {
      forwarded =
          ForwardGcuBezelEvent(*b, avionics::cmdbridge::Phase::Continue);
      if (hasEngine) {
        dev->engine->holdBezelKey(avionics::BezelKey::Clr);
        b->holdFired = true;
        PersistStateIfChanged();
        applied = true;
      } else if (forwarded) {
        b->holdFired = true;
        applied = true;
      }
    }
  } else if (phase == xplm_CommandEnd) {
    if (isClr) {
      forwarded = ForwardGcuBezelEvent(*b, avionics::cmdbridge::Phase::End);
      b->holdFired = false;
    }
  }

  return (forwarded || applied) ? 0 : 1;
}

bool ForwardGcuEntryEvent(AvionicsDevice& dev, char ch) {
  if (!g_commandBridge) return false;
  avionics::cmdbridge::Event ev;
  ev.device = (&dev == &g_mfd) ? avionics::cmdbridge::Device::Mfd
                               : avionics::cmdbridge::Device::Pfd;
  ev.phase = avionics::cmdbridge::Phase::Begin;
  ev.kind = avionics::cmdbridge::Kind::GcuEntry;
  ev.value = static_cast<std::int32_t>(static_cast<unsigned char>(ch));
  return g_commandBridge->sendEvent(ev);
}

bool ApplyGcuEntryKey(char ch) {
  if (PfdClaimsGcuFms()) {
    const bool forwarded = ForwardGcuEntryEvent(g_pfd, ch);
    if (g_pfd.engine != nullptr && g_pfd.engine->applyGcuEntryKey(ch)) {
      PersistStateIfChanged();
      return true;
    }
    return forwarded;
  }
  const bool forwarded = ForwardGcuEntryEvent(g_mfd, ch);
  if (g_mfd.engine != nullptr && g_mfd.engine->applyGcuEntryKey(ch)) {
    PersistStateIfChanged();
    return true;
  }
  return forwarded;
}

int GcuKeypadCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                              void* ref) {
  if (phase != xplm_CommandBegin) return 1;
  auto* b = static_cast<GcuKeypadBinding*>(ref);
  if (b == nullptr) return 1;
  return ApplyGcuEntryKey(b->ch) ? 0 : 1;
}

// Routes one GCU dual-knob detent to the engine action chosen by the current
// knob mode. Returns true if an engine consumed it (so we suppress the stock
// G1000). The knob is inert in XPDR mode (no transponder tuning knob exists).
bool ApplyGcuKnobTurn(GcuKnobAction turn) {
  const bool up =
      turn == GcuKnobAction::OuterUp || turn == GcuKnobAction::InnerUp;
  const bool outer =
      turn == GcuKnobAction::OuterUp || turn == GcuKnobAction::OuterDown;
  switch (g_gcuKnobMode) {
    case GcuKnobMode::Fms: {
      AvionicsDevice* dev = GcuFmsBezelDevice();
      const avionics::BezelKey key =
          outer ? (up ? avionics::BezelKey::FmsOuterCw
                      : avionics::BezelKey::FmsOuterCcw)
                : (up ? avionics::BezelKey::FmsInnerCw
                      : avionics::BezelKey::FmsInnerCcw);
      const bool forwarded = ForwardBezelEvent(*dev, key);
      if (dev->engine != nullptr) {
        dev->engine->pressBezelKey(key);
        PersistStateIfChanged();
      }
      return forwarded || dev->engine != nullptr;
    }
    case GcuKnobMode::Com:
    case GcuKnobMode::Nav: {
      RadioAction action;
      if (g_gcuKnobMode == GcuKnobMode::Com) {
        action = outer ? (up ? RadioAction::ComOuterUp : RadioAction::ComOuterDown)
                       : (up ? RadioAction::ComInnerUp : RadioAction::ComInnerDown);
      } else {
        action = outer ? (up ? RadioAction::NavOuterUp : RadioAction::NavOuterDown)
                       : (up ? RadioAction::NavInnerUp : RadioAction::NavInnerDown);
      }
      const bool forwarded = ForwardRadioEvent(action);
      if (g_pfd.engine != nullptr) {
        ApplyRadioAction(action);
        return true;
      }
      return forwarded;
    }
    case GcuKnobMode::Xpdr:
      return false;
  }
  return false;
}

// GCU COM/NAV flip-flop key: flips whichever band the knob is currently
// assigned to (the key is labeled "COM/NAV flip flop"); defaults to COM when
// the knob is in FMS or XPDR mode.
bool ApplyGcuFlip() {
  const RadioAction action = g_gcuKnobMode == GcuKnobMode::Nav
                                 ? RadioAction::NavFlip
                                 : RadioAction::ComFlip;
  const bool forwarded = ForwardRadioEvent(action);
  if (g_pfd.engine != nullptr) {
    ApplyRadioAction(action);
    return true;
  }
  return forwarded;
}

// Handles the GCU shared knob, its FMS/COM/NAV/XPDR mode keys, and the
// flip-flop key. Mode keys only re-target the knob (their effect shows on the
// next turn), matching the real unit.
int GcuKnobCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                          void* ref) {
  if (phase != xplm_CommandBegin) return 1;
  auto* b = static_cast<GcuKnobBinding*>(ref);
  if (b == nullptr) return 1;

  switch (b->action) {
    case GcuKnobAction::ModeFms:
      g_gcuKnobMode = GcuKnobMode::Fms;
      return 0;
    case GcuKnobAction::ModeCom:
      g_gcuKnobMode = GcuKnobMode::Com;
      return 0;
    case GcuKnobAction::ModeNav:
      g_gcuKnobMode = GcuKnobMode::Nav;
      return 0;
    case GcuKnobAction::ModeXpdr:
      g_gcuKnobMode = GcuKnobMode::Xpdr;
      return 0;
    case GcuKnobAction::FlipFlop:
      return ApplyGcuFlip() ? 0 : 1;
    case GcuKnobAction::OuterUp:
    case GcuKnobAction::OuterDown:
    case GcuKnobAction::InnerUp:
    case GcuKnobAction::InnerDown:
      return ApplyGcuKnobTurn(b->action) ? 0 : 1;
  }
  return 1;
}

// Stable backing store for the names/descriptions of the commands we create, so
// the pointers handed to XPLMCreateCommand stay valid. A deque never relocates
// its elements, so c_str() stays good (a vector would invalidate on growth).
std::deque<std::string> g_customStrings;
const char* StoreStr(const std::string& s) {
  g_customStrings.push_back(s);
  return g_customStrings.back().c_str();
}

// Binds an existing stock command (intercept) if X-Plane has it.
void BindCommand(AvionicsDevice& dev, const char* name, bool isSoftkey,
                 int value) {
  XPLMCommandRef cmd = XPLMFindCommand(name);
  if (cmd == nullptr) return;
  g_commandBindings.push_back({&dev, isSoftkey, value, -1, cmd});
}

void BindDiagonal(AvionicsDevice& dev, const char* name, avionics::BezelKey a,
                  avionics::BezelKey b) {
  XPLMCommandRef cmd = XPLMFindCommand(name);
  if (cmd == nullptr) return;
  g_commandBindings.push_back(
      {&dev, false, static_cast<int>(a), static_cast<int>(b), cmd});
}

// Creates one of our own commands (always present, so it's bindable from the
// X-Plane Keyboard/Joystick UI) and binds it to the same action.
void BindCreatedCommand(AvionicsDevice& dev, const std::string& name,
                        const std::string& desc, bool isSoftkey, int value,
                        int value2) {
  XPLMCommandRef cmd = XPLMCreateCommand(StoreStr(name), StoreStr(desc));
  if (cmd == nullptr) return;
  g_commandBindings.push_back({&dev, isSoftkey, value, value2, cmd});
}

// Builds the binding list for one device. `stockPrefix` is the X-Plane stock
// command prefix we intercept ("g1000n1" / "g1000n3"); `customId`/`label` name
// the parallel commands we create ("pfd"/"PFD"). Every key gets both, so it
// works whether the user binds the stock cockpit command or our own. The list
// is filled completely before any handler is registered so the refcon pointers
// stay valid.
void CollectDeviceCommands(AvionicsDevice& dev, const char* stockPrefix,
                           const char* customId, const char* label) {
  char name[96];
  char desc[128];

  // 1) Intercept the aircraft's stock G1000 commands when present.
  for (int i = 1; i <= avionics::kSoftkeyCount; ++i) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_softkey%d", stockPrefix, i);
    BindCommand(dev, name, /*isSoftkey=*/true, i - 1);
  }
  for (const NamedKey& nk : kNamedKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", stockPrefix, nk.suffix);
    BindCommand(dev, name, /*isSoftkey=*/false, static_cast<int>(nk.key));
  }
  for (const DiagonalKey& dk : kDiagonalKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", stockPrefix, dk.suffix);
    BindDiagonal(dev, name, dk.a, dk.b);
  }

  // 2) Create our own commands so every key is always bindable.
  for (int i = 1; i <= avionics::kSoftkeyCount; ++i) {
    std::snprintf(name, sizeof(name), "%s/%s/softkey%d", kCmdPrefix, customId, i);
    std::snprintf(desc, sizeof(desc), "G1000 NXi %s: Softkey %d", label, i);
    BindCreatedCommand(dev, name, desc, /*isSoftkey=*/true, i - 1, -1);
  }
  for (const NamedKey& nk : kNamedKeys) {
    std::snprintf(name, sizeof(name), "%s/%s/%s", kCmdPrefix, customId,
                  nk.suffix);
    std::snprintf(desc, sizeof(desc), "G1000 NXi %s: %s", label, nk.label);
    BindCreatedCommand(dev, name, desc, /*isSoftkey=*/false,
                       static_cast<int>(nk.key), -1);
  }
  for (const DiagonalKey& dk : kDiagonalKeys) {
    std::snprintf(name, sizeof(name), "%s/%s/%s", kCmdPrefix, customId,
                  dk.suffix);
    std::snprintf(desc, sizeof(desc), "G1000 NXi %s: %s", label, dk.label);
    BindCreatedCommand(dev, name, desc, /*isSoftkey=*/false,
                       static_cast<int>(dk.a), static_cast<int>(dk.b));
  }
}

// Collects the NAV/COM knob commands for one GDU prefix. Both GDUs' knobs tune
// the same radios, so they all route through RadioCommandHandler to the PFD
// engine; the binding only needs the action.
void CollectRadioCommands(const char* prefix) {
  char name[96];
  for (const RadioCommand& rc : kRadioCommands) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", prefix, rc.suffix);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_radioBindings.push_back({rc.action, cmd});
  }
}

// Creates our own NAV/COM knob commands (one set, since both GDUs tune the same
// radios) so the radios are bindable even without stock G1000 commands.
void CreateRadioCommands() {
  char name[96];
  char desc[128];
  for (const RadioCommand& rc : kRadioCommands) {
    std::snprintf(name, sizeof(name), "%s/radio/%s", kCmdPrefix, rc.suffix);
    std::snprintf(desc, sizeof(desc), "G1000 NXi: %s", rc.label);
    XPLMCommandRef cmd = XPLMCreateCommand(StoreStr(name), StoreStr(desc));
    if (cmd == nullptr) continue;
    g_radioBindings.push_back({rc.action, cmd});
  }
}

// Collects the GCU 478 commands. Intercept stock sim/GPS/gcu478/* when present
// and create parallel xplane_avionics/gcu/* commands (SPAD / joystick bindings).
// Map / pan keys go to the MFD; Direct-To opens the PFD popout; FMS / ENT / CLR
// / cursor follow whichever display currently owns FMS input (PFD pop-ups vs
// MFD).
bool GcuCommandAlreadyBound(XPLMCommandRef cmd) {
  if (cmd == nullptr) return true;
  for (const GcuBezelBinding& b : g_gcuBezelBindings) {
    if (b.cmd == cmd) return true;
  }
  for (const GcuKnobBinding& b : g_gcuKnobBindings) {
    if (b.cmd == cmd) return true;
  }
  for (const GcuKeypadBinding& b : g_gcuKeypadBindings) {
    if (b.cmd == cmd) return true;
  }
  return false;
}

void BindGcuBezelCommand(avionics::BezelKey key, avionics::BezelKey key2,
                         XPLMCommandRef cmd) {
  if (GcuCommandAlreadyBound(cmd)) return;
  g_gcuBezelBindings.push_back({key, key2, cmd});
}

void CollectGcuCommands() {
  char name[96];
  char desc[128];
  for (const NamedKey& nk : kGcuNamedKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%s", nk.suffix);
    BindGcuBezelCommand(nk.key, avionics::BezelKey::Count, XPLMFindCommand(name));
    std::snprintf(name, sizeof(name), "%s/gcu/%s", kCmdPrefix, nk.suffix);
    std::snprintf(desc, sizeof(desc), "G1000 NXi GCU: %s", nk.label);
    BindGcuBezelCommand(nk.key, avionics::BezelKey::Count,
                        XPLMCreateCommand(StoreStr(name), StoreStr(desc)));
  }
  // Alternate suffixes used by some GCU hardware / aircraft integrations.
  struct GcuRangeAlias {
    const char* suffix;
    avionics::BezelKey key;
  };
  static const GcuRangeAlias kGcuRangeAliases[] = {
      {"rng_up", avionics::BezelKey::RangeUp},
      {"rng_dn", avionics::BezelKey::RangeDown},
      {"rng_down", avionics::BezelKey::RangeDown},
      {"range_dn", avionics::BezelKey::RangeDown},
  };
  for (const GcuRangeAlias& alias : kGcuRangeAliases) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%s", alias.suffix);
    BindGcuBezelCommand(alias.key, avionics::BezelKey::Count,
                        XPLMFindCommand(name));
    std::snprintf(name, sizeof(name), "%s/gcu/%s", kCmdPrefix, alias.suffix);
    std::snprintf(desc, sizeof(desc), "G1000 NXi GCU: %s",
                  alias.key == avionics::BezelKey::RangeUp ? "RANGE out (zoom out)"
                                                           : "RANGE in (zoom in)");
    BindGcuBezelCommand(alias.key, avionics::BezelKey::Count,
                        XPLMCreateCommand(StoreStr(name), StoreStr(desc)));
  }
  for (const DiagonalKey& dk : kGcuDiagonalKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%s", dk.suffix);
    BindGcuBezelCommand(dk.a, dk.b, XPLMFindCommand(name));
  }
  for (const GcuKnobCommand& kc : kGcuKnobCommands) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%s", kc.suffix);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_gcuKnobBindings.push_back({kc.action, cmd});
  }
  for (char c = 'A'; c <= 'Z'; ++c) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%c", c);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_gcuKeypadBindings.push_back({c, cmd});
  }
  for (char c = '0'; c <= '9'; ++c) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%c", c);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_gcuKeypadBindings.push_back({c, cmd});
  }
  struct SpecialKey {
    const char* suffix;
    char ch;
  };
  static const SpecialKey kSpecial[] = {{"dot", '.'}, {"bksp", '\b'}};
  for (const SpecialKey& sk : kSpecial) {
    std::snprintf(name, sizeof(name), "sim/GPS/gcu478/%s", sk.suffix);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_gcuKeypadBindings.push_back({sk.ch, cmd});
  }
}

void RegisterG1000Commands() {
  g_commandBindings.clear();
  g_gcuKnobBindings.clear();
  g_gcuBezelBindings.clear();
  g_gcuKeypadBindings.clear();
  g_customStrings.clear();
  // 3 GDU prefixes x (12 softkeys + 18 named + 4 diagonal) stock + the same
  // created, plus the GCU's intercept-only named/diagonal keys. Over-reserve so
  // the vector never reallocates while collecting (refcons point into it).
  g_commandBindings.reserve(448);
  CollectDeviceCommands(g_pfd, "g1000n1", "pfd", "PFD");
  // Copilot-side GDU keys (g1000n2) drive the same pilot PFD engine.
  CollectDeviceCommands(g_pfd, "g1000n2", "pfd_copilot", "PFD (copilot GDU)");
  CollectDeviceCommands(g_mfd, "g1000n3", "mfd", "MFD");
  // Optional GCU 478 control unit (routed dynamically to PFD or MFD).
  CollectGcuCommands();
  for (CommandBinding& b : g_commandBindings) {
    XPLMRegisterCommandHandler(b.cmd, &G1000CommandHandler, /*before=*/1, &b);
  }
  for (GcuBezelBinding& b : g_gcuBezelBindings) {
    XPLMRegisterCommandHandler(b.cmd, &GcuBezelCommandHandler, /*before=*/1, &b);
  }
  for (GcuKeypadBinding& b : g_gcuKeypadBindings) {
    XPLMRegisterCommandHandler(b.cmd, &GcuKeypadCommandHandler, /*before=*/1,
                               &b);
  }
  for (GcuKnobBinding& b : g_gcuKnobBindings) {
    XPLMRegisterCommandHandler(b.cmd, &GcuKnobCommandHandler, /*before=*/1, &b);
  }

  // NAV/COM knobs: intercept both GDUs' stock commands plus our own set, all
  // routed to the PFD engine's bar.
  g_radioBindings.clear();
  g_radioBindings.reserve(64);
  CollectRadioCommands("g1000n1");
  CollectRadioCommands("g1000n2");
  CollectRadioCommands("g1000n3");
  CreateRadioCommands();
  for (RadioCommandBinding& b : g_radioBindings) {
    XPLMRegisterCommandHandler(b.cmd, &RadioCommandHandler, /*before=*/1, &b);
  }
  RegisterDisplayBackupCommands();
  RegisterAutopilotCommands();
}

void UnregisterG1000Commands() {
  for (CommandBinding& b : g_commandBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &G1000CommandHandler, /*before=*/1, &b);
  }
  g_commandBindings.clear();
  for (GcuBezelBinding& b : g_gcuBezelBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &GcuBezelCommandHandler, /*before=*/1,
                                 &b);
  }
  g_gcuBezelBindings.clear();
  for (GcuKeypadBinding& b : g_gcuKeypadBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &GcuKeypadCommandHandler, /*before=*/1,
                                 &b);
  }
  g_gcuKeypadBindings.clear();
  for (GcuKnobBinding& b : g_gcuKnobBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &GcuKnobCommandHandler, /*before=*/1, &b);
  }
  g_gcuKnobBindings.clear();
  for (RadioCommandBinding& b : g_radioBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &RadioCommandHandler, /*before=*/1, &b);
  }
  g_radioBindings.clear();
  UnregisterDisplayBackupCommands();
  UnregisterAutopilotCommands();
}

void EnableGlassTakeover();
void DisableGlassTakeover();

// ---- rate-preset menu --------------------------------------------------------
// itemRef -1 toggles in-sim display replacement; 0..kPresetCount-1 pick a preset.
constexpr int kReplaceDisplaysMenuRef = -1;
constexpr int kAircraftOverrideAutoRef = -2;
constexpr int kAircraftOverrideC172Ref = -3;
constexpr int kAircraftOverrideSF50Ref = -4;
constexpr int kAircraftOverridePA46TRef = -5;

constexpr int kReplaceDisplaysMenuIndex = kPresetCount + 1;
constexpr int kFmsDebugMenuIndex = kPresetCount + 2;

constexpr int kAircraftOverrideAutoIndex = kPresetCount + 4;
constexpr int kAircraftOverrideC172Index = kPresetCount + 5;
constexpr int kAircraftOverrideSF50Index = kPresetCount + 6;
constexpr int kAircraftOverridePA46TIndex = kPresetCount + 7;

// Puts a check mark beside the active preset and clears the others.
void RefreshRateMenuChecks() {
  if (g_rateMenu == nullptr) return;
  for (int i = 0; i < kPresetCount; ++i) {
    XPLMCheckMenuItem(g_rateMenu, i,
                      static_cast<int>(i) == static_cast<int>(g_preset)
                          ? xplm_Menu_Checked
                          : xplm_Menu_Unchecked);
  }
  XPLMCheckMenuItem(g_rateMenu, kReplaceDisplaysMenuIndex,
                    g_replaceDisplays ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  XPLMCheckMenuItem(g_rateMenu, kFmsDebugMenuIndex,
                    g_showFmsDebug ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  XPLMCheckMenuItem(g_rateMenu, kAircraftOverrideAutoIndex,
                    g_aircraftOverride == avionics::AircraftOverride::Auto ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  XPLMCheckMenuItem(g_rateMenu, kAircraftOverrideC172Index,
                    g_aircraftOverride == avionics::AircraftOverride::C172 ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  XPLMCheckMenuItem(g_rateMenu, kAircraftOverrideSF50Index,
                    g_aircraftOverride == avionics::AircraftOverride::SF50 ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  XPLMCheckMenuItem(g_rateMenu, kAircraftOverridePA46TIndex,
                    g_aircraftOverride == avionics::AircraftOverride::PA46T ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

void OnRateMenuItem(void* /*menuRef*/, void* itemRef) {
  const int idx = static_cast<int>(reinterpret_cast<intptr_t>(itemRef));
  if (idx == kReplaceDisplaysMenuRef) {
    g_replaceDisplays = !g_replaceDisplays;
    if (g_replaceDisplays) {
      RegisterG1000Commands();
    } else {
      UnregisterG1000Commands();
    }
    SaveConfig();
    RefreshRateMenuChecks();
    return;
  }
  if (idx == kAircraftOverrideAutoRef) {
    g_aircraftOverride = avionics::AircraftOverride::Auto;
    if (g_dataSource) {
      g_dataSource->setAircraftOverride(avionics::AircraftOverride::Auto);
    }
    SaveConfig();
    RefreshRateMenuChecks();
    return;
  }
  if (idx == kAircraftOverrideC172Ref) {
    g_aircraftOverride = avionics::AircraftOverride::C172;
    if (g_dataSource) {
      g_dataSource->setAircraftOverride(avionics::AircraftOverride::C172);
    }
    SaveConfig();
    RefreshRateMenuChecks();
    return;
  }
  if (idx == kAircraftOverrideSF50Ref) {
    g_aircraftOverride = avionics::AircraftOverride::SF50;
    if (g_dataSource) {
      g_dataSource->setAircraftOverride(avionics::AircraftOverride::SF50);
    }
    SaveConfig();
    RefreshRateMenuChecks();
    return;
  }
  if (idx == kAircraftOverridePA46TRef) {
    g_aircraftOverride = avionics::AircraftOverride::PA46T;
    if (g_dataSource) {
      g_dataSource->setAircraftOverride(avionics::AircraftOverride::PA46T);
    }
    SaveConfig();
    RefreshRateMenuChecks();
    return;
  }
  if (idx < 0 || idx >= kPresetCount) return;
  ApplyPreset(static_cast<RatePreset>(idx));
  SaveConfig();
  RefreshRateMenuChecks();
}

// Bindable command (and the "Install Update" menu item) that kicks off the
// in-sim self-update. Inert until the launch-time check finds a newer release.
int OnInstallUpdateCommand(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                           void* /*ref*/) {
  if (phase == xplm_CommandBegin) avionics::beginPluginSelfUpdate();
  return 0;
}

void ToggleFmsDebugOverlay() {
  g_showFmsDebug = !g_showFmsDebug;
  avionics::FmsDebugOverlay::setEnabled(g_showFmsDebug);
  SaveConfig();
  RefreshRateMenuChecks();
}

int OnToggleFmsDebugCommand(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                            void* /*ref*/) {
  if (phase == xplm_CommandBegin) ToggleFmsDebugOverlay();
  return 0;
}

// Main-thread pump for the self-updater (the XPLM API and plugin reload are
// main-thread only). Applies a verified download and, the first time a newer
// release is found, reveals the "Install Update" menu item with its version.
float UpdatePumpFlightLoop(float /*sinceLast*/, float /*sinceLoop*/,
                           int /*counter*/, void* /*ref*/) {
  avionics::pumpPluginSelfUpdate();
  PumpNavigraph();

  // Remember the pilot's V-speed reference bugs per aircraft type: restore them
  // when the airframe changes, and persist any edits made since the last check.
  if (g_pfd.engine && g_dataSource) {
    static avionics::VspeedAircraftMemory vspeedMemory;
    if (vspeedMemory.sync(g_pfd.engine->softkeyController(),
                          g_dataSource->aircraftIcaoType(),
                          g_avionicsState.vspeedByAircraft)) {
      SaveConfig();
    }
  }
  if (!g_installUpdateMenuShown && avionics::updateAvailable() &&
      g_rateMenu != nullptr && g_installUpdateMenuIndex >= 0) {
    const std::string label =
        "Install Update (v" + avionics::availableUpdateVersion() + ")";
    XPLMSetMenuItemName(g_rateMenu, g_installUpdateMenuIndex, label.c_str(), 0);
    XPLMEnableMenuItem(g_rateMenu, g_installUpdateMenuIndex, 1);
    g_installUpdateMenuShown = true;
  }
  return 1.0f;
}

// Builds the Plugins -> G1000 NXi submenu with one radio-style item per preset.
void BuildRateMenu() {
  g_rateMenuParentItem =
      XPLMAppendMenuItem(XPLMFindPluginsMenu(), "G1000 NXi", nullptr, 1);
  g_rateMenu = XPLMCreateMenu("G1000 NXi", XPLMFindPluginsMenu(),
                              g_rateMenuParentItem, &OnRateMenuItem, nullptr);
  for (int i = 0; i < kPresetCount; ++i) {
    XPLMAppendMenuItem(g_rateMenu, kPresets[i].name,
                       reinterpret_cast<void*>(static_cast<intptr_t>(i)), 1);
  }
  XPLMAppendMenuSeparator(g_rateMenu);
  XPLMAppendMenuItem(
      g_rateMenu, "Replace in-sim G1000 displays",
      reinterpret_cast<void*>(static_cast<intptr_t>(kReplaceDisplaysMenuRef)), 1);
  g_fmsDebugCmd = XPLMCreateCommand(
      "xplaneavionics/toggle_fms_debug",
      "G1000 NXi: Toggle FMS / flight-plan debug overlay");
  XPLMRegisterCommandHandler(g_fmsDebugCmd, &OnToggleFmsDebugCommand,
                             /*before=*/1, nullptr);
  XPLMAppendMenuItemWithCommand(g_rateMenu, "Show FMS debug overlay",
                                g_fmsDebugCmd);
  XPLMAppendMenuSeparator(g_rateMenu);
  XPLMAppendMenuItem(
      g_rateMenu, "Aircraft Layout: Auto",
      reinterpret_cast<void*>(static_cast<intptr_t>(kAircraftOverrideAutoRef)), 1);
  XPLMAppendMenuItem(
      g_rateMenu, "Aircraft Layout: Cessna 172S",
      reinterpret_cast<void*>(static_cast<intptr_t>(kAircraftOverrideC172Ref)), 1);
  XPLMAppendMenuItem(
      g_rateMenu, "Aircraft Layout: Cirrus SF50",
      reinterpret_cast<void*>(static_cast<intptr_t>(kAircraftOverrideSF50Ref)), 1);
  XPLMAppendMenuItem(
      g_rateMenu, "Aircraft Layout: Piper PA-46T",
      reinterpret_cast<void*>(static_cast<intptr_t>(kAircraftOverridePA46TRef)), 1);
  // "Install Update" is bound to its command so it is both clickable and
  // key-bindable. Disabled until the update pump finds a newer release.
  XPLMAppendMenuSeparator(g_rateMenu);
  g_installUpdateMenuIndex = XPLMAppendMenuItemWithCommand(
      g_rateMenu, "Install Update", g_installUpdateCmd);
  XPLMEnableMenuItem(g_rateMenu, g_installUpdateMenuIndex, 0);
  g_installUpdateMenuShown = false;
  RefreshRateMenuChecks();
}

void DestroyRateMenu() {
  if (g_rateMenu != nullptr) {
    XPLMDestroyMenu(g_rateMenu);
    g_rateMenu = nullptr;
  }
  if (g_rateMenuParentItem >= 0) {
    XPLMRemoveMenuItem(XPLMFindPluginsMenu(), g_rateMenuParentItem);
    g_rateMenuParentItem = -1;
  }
  g_installUpdateMenuIndex = -1;
  g_installUpdateMenuShown = false;
}

// Releases an engine and intentionally leaks its NanoVG / FBO GL resources:
// deleting them requires the device's GL context to be current, which is not
// guaranteed during plugin unload, so we drop the engine (no GL) and abandon
// the renderer and cache objects rather than risk GL calls without a context.
// (On XPLMReloadPlugins the DLL is reloaded, so these globals reset to zero and
// the cache is regenerated cleanly against the fresh context.)
void ShutdownDevice(AvionicsDevice& dev) {
  if (dev.handle != nullptr) {
    XPLMUnregisterAvionicsCallbacks(dev.handle);
    dev.handle = nullptr;
  }
  dev.engine.reset();
  if (dev.renderer) dev.renderer.release();
  dev.fbo = 0;
  dev.tex = 0;
  dev.depthStencilRbo = 0;
  dev.texW = 0;
  dev.texH = 0;
  dev.cacheReady = false;
}

// Take over the built-in G1000 PFD (pilot) + MFD and grab the GDU keys.
void EnableGlassTakeover() {
  if (g_pfd.handle == nullptr) {
    RegisterDevice(g_pfd, xplm_device_G1000_PFD_1, &PfdDrawCallback);
  }
  if (g_mfd.handle == nullptr) {
    RegisterDevice(g_mfd, xplm_device_G1000_MFD, &MfdDrawCallback);
  }
  // XPluginEnable already registered handlers; avoid stacking duplicates when
  // the pilot turns display replacement back on from the Plugins menu.
  UnregisterG1000Commands();
  RegisterG1000Commands();
}

// Hand the screens and GDU keys back to the stock G1000.
void DisableGlassTakeover() {
  UnregisterG1000Commands();
  ShutdownDevice(g_pfd);
  ShutdownDevice(g_mfd);
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
  std::strcpy(outName, "G1000 NXi");
  std::strcpy(outSig, "com.andrewmiller.xplaneavionics");
  std::strcpy(outDesc, "Shared-core glass cockpit (PFD/MFD) for X-Plane.");

  // Initialize libcurl once on X-Plane's main thread before the NavigraphStore
  // worker thread can issue a request. libcurl's implicit init is not
  // thread-safe and would otherwise race other curl users inside the sim.
  avionics::EnsureCurlGlobalInit();

  // Opt into POSIX paths. Without this the SDK returns legacy HFS-style paths
  // (e.g. "Macintosh HD:Users:...") from XPLMGetSystemPath/XPLMGetPrefsPath/the
  // plugin path, which our std::ifstream/stat file probes can't open. Must run
  // before any path-returning SDK call (map data discovery, config load).
  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

  // Find assets bundled with the plugin. The plugin loads from
  // <plugins>/xplane-avionics/<platform>/xplane-avionics.xpl; the installer puts
  // the shared fonts/EIS sample at <plugins>/xplane-avionics/assets. Must run
  // after enabling native paths so the SDK returns a POSIX path we can split.
  {
    char pluginPath[512] = {0};
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, pluginPath, nullptr, nullptr);
    if (pluginPath[0] != '\0') {
      // .../xplane-avionics/<platform>/xplane-avionics.xpl -> .../xplane-avionics
      const std::filesystem::path pluginRoot =
          std::filesystem::path(pluginPath).parent_path().parent_path();
      avionics::assets::addSearchDir((pluginRoot / "assets").string());
    }
  }

  g_eisStore = std::make_unique<avionics::EisStore>();
  g_checklistStore = std::make_unique<avionics::ChecklistStore>();
  g_obstacleStore = std::make_unique<avionics::ObstacleStore>(
      avionics::assets::resolve("obstacles.csv", kObstaclesAssetPath));
  g_dataSource = std::make_unique<avionics::DatarefDataSource>(g_eisStore.get());
  g_dataSource->setChecklistSource(g_checklistStore.get());
  g_dataSource->setObstacleStore(g_obstacleStore.get());
  g_navDataStore = std::make_unique<avionics::NavDataStore>();
  g_procedureStore =
      std::make_unique<avionics::ProcedureStore>(*g_navDataStore);
  g_navMapData = std::make_unique<avionics::PluginNavMapData>(
      g_dataSource.get(), g_procedureStore.get());

  // X-Plane persists the pilot FMS across restarts; clear it so the NXi FPL
  // editor starts blank and the standalone bridge does not echo yesterday's route.
  avionics::programFmsRoute({});

  g_flightPlanBridge = std::make_unique<avionics::FlightPlanBridge>(
      avionics::fpbridge::kDefaultPort);
  g_commandBridge = std::make_unique<avionics::CommandBridge>();

  // SimBrief OFP import via Navigraph. Credentials load from the per-user config
  // dir / env vars; a saved refresh token silently restores the prior session
  // (and auto-fetches the latest OFP) without a fresh sign-in.
  g_navigraph = std::make_unique<avionics::NavigraphStore>();
  g_navigraph->setCredentials(avionics::LoadNavigraphCredentials());
  g_navigraphRefreshToken = avionics::LoadNavigraphRefreshToken();
  if (g_navigraph->hasCredentials() && !g_navigraphRefreshToken.empty()) {
    g_navigraph->restoreSession(g_navigraphRefreshToken);
  }

  // Start the UDP bridges and intercept GDU commands as soon as the plugin
  // loads. X-Plane does not always call XPluginEnable after a plugin reload
  // (only "Loaded:" appears in Log.txt), so relying on Enable alone left the
  // command bridge down and cockpit keys never reached the standalone shell.
  g_flightPlanBridge->start();
  g_commandBridge->start();
  RegisterG1000Commands();

  // The "Install Update" command must exist before BuildRateMenu binds the
  // menu item to it.
  g_installUpdateCmd = XPLMCreateCommand(
      "xplaneavionics/install_update", "G1000 NXi: Install available update");
  XPLMRegisterCommandHandler(g_installUpdateCmd, &OnInstallUpdateCommand,
                             /*before=*/1, nullptr);

  Log("G1000 NXi plugin loaded (FMS debug overlay + active-leg sync)\n");

  // Restore the saved config
  // preferences) before building the menu, so the right item starts checked
  // and the engines pick up the saved options when first created.
  LoadConfig();
  if (g_dataSource) {
    g_dataSource->setAircraftOverride(g_aircraftOverride);
  }
  avionics::FmsDebugOverlay::setEnabled(g_showFmsDebug);
  BuildRateMenu();

  // Periodic main-thread pump that drives the self-updater and reveals the
  // "Install Update" menu item once the background check finds a newer release.
  XPLMRegisterFlightLoopCallback(&UpdatePumpFlightLoop, 1.0f, nullptr);

  // Move the MFD terrain raster's DEM sampling + hillshade colorize (~100 ms for
  // a full 512x512 rebuild) off the sim thread onto a worker; only the finished
  // RGBA buffer's GPU upload stays on the sim thread (inside the MFD draw). The
  // worker dereferences the DataSource's TerrainSource, so it must be shut down
  // before g_dataSource is destroyed (see XPluginStop). The worker never touches
  // GL, so it is safe to run alongside X-Plane's render loop.
  avionics::map::setAsyncTerrainBuilds(true);

  // Devices are registered on enable and aircraft load rather than plugin start
  // to ensure X-Plane's built-in G1000 devices are fully initialized first.

  avionics::startUpdateCheckOnLaunch();
  return 1;
}

PLUGIN_API void XPluginStop(void) {
  XPLMUnregisterFlightLoopCallback(&UpdatePumpFlightLoop, nullptr);
  // Joins the Navigraph worker thread; do it before tearing down engines since
  // the pump callback (now unregistered) is the only other thread that uses it.
  g_navigraph.reset();
  if (g_installUpdateCmd != nullptr) {
    XPLMUnregisterCommandHandler(g_installUpdateCmd, &OnInstallUpdateCommand,
                                 /*before=*/1, nullptr);
    g_installUpdateCmd = nullptr;
  }
  if (g_fmsDebugCmd != nullptr) {
    XPLMUnregisterCommandHandler(g_fmsDebugCmd, &OnToggleFmsDebugCommand,
                                 /*before=*/1, nullptr);
    g_fmsDebugCmd = nullptr;
  }
  UnregisterG1000Commands();
  if (g_commandBridge) g_commandBridge->stop();
  if (g_flightPlanBridge) g_flightPlanBridge->stop();
  avionics::FmsDebugOverlay::unregisterDrawCallback();
  DestroyRateMenu();
  ShutdownDevice(g_pfd);
  ShutdownDevice(g_mfd);
  // Join the terrain worker before the DataSource (which owns the TerrainSource
  // the worker samples) is destroyed, so the worker can't dereference freed
  // terrain data mid-build.
  avionics::map::setAsyncTerrainBuilds(false);
  g_commandBridge.reset();
  g_flightPlanBridge.reset();
  g_dataSource.reset();
  g_obstacleStore.reset();
  g_navMapData.reset();
  g_eisStore.reset();
  g_checklistStore.reset();
}

// The flight-plan bridge owns a flight-loop callback and a network thread, so
// it follows X-Plane's enable/disable lifecycle rather than start/stop.
PLUGIN_API int XPluginEnable(void) {
  if (g_flightPlanBridge) g_flightPlanBridge->start();
  if (g_commandBridge) g_commandBridge->start();
  
  // Seamlessly keep callbacks registered during plugin enable
  EnableGlassTakeover();
  if (!g_replaceDisplays) {
    UnregisterG1000Commands();
  }
  return 1;
}
PLUGIN_API void XPluginDisable(void) {
  DisableGlassTakeover();
  if (g_commandBridge) g_commandBridge->stop();
  if (g_flightPlanBridge) g_flightPlanBridge->stop();
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID /*from*/, int msg,
                                      void* /*param*/) {
  if (msg == XPLM_MSG_PLANE_LOADED) {
    RefreshDisplayBackupStockCommand();
    
    // Re-register device callbacks because X-Plane recreated the screens
    if (g_pfd.handle) {
      XPLMUnregisterAvionicsCallbacks(g_pfd.handle);
      g_pfd.handle = nullptr;
    }
    if (g_mfd.handle) {
      XPLMUnregisterAvionicsCallbacks(g_mfd.handle);
      g_mfd.handle = nullptr;
    }
    RegisterDevice(g_pfd, xplm_device_G1000_PFD_1, &PfdDrawCallback);
    RegisterDevice(g_mfd, xplm_device_G1000_MFD, &MfdDrawCallback);
    
    // Command interception depends on g_replaceDisplays
    UnregisterG1000Commands();
    if (g_replaceDisplays) {
      RegisterG1000Commands();
    }
  }
}
