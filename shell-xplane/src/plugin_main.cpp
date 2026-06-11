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
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "DatarefDataSource.h"
#include "FlightPlanBridge.h"
#include "avionics/EisStore.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "avionics/AvionicsEngine.h"
#include "avionics/FlightPlanBridgeProtocol.h"
#include "avionics/PersistentState.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/NanoVgRenderer.h"

namespace {

// Prefix for our Log.txt diagnostics so they're easy to grep.
void Log(const char* msg) { XPLMDebugString(msg); }

// Feed label for the boot / connection-lost screens (matches the standalone
// shell's live-source naming).
constexpr const char* kSourceLabel = "X-PLANE";

// Fallback device-screen size if X-Plane reports an empty viewport (it always
// sets one to the device's native resolution; this just avoids a zero divide).
constexpr int kFallbackScreenW = 1024;
constexpr int kFallbackScreenH = 768;

// The data source is shared by both device engines; the PFD engine pumps it,
// the MFD engine reads the same snapshot without double-stepping it.
std::unique_ptr<avionics::DatarefDataSource> g_dataSource;
std::unique_ptr<avionics::EisStore> g_eisStore;
std::uint32_t g_mapGeometryEpoch = 0;

// Serves the live FMS flight plan to the networked standalone shell over UDP
// (the standalone shell cannot read the FMS itself; see FlightPlanBridge.h).
std::unique_ptr<avionics::FlightPlanBridge> g_flightPlanBridge;

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
};

// Indexed by RatePreset. Full = redraw every frame at full resolution (best
// smoothness, highest cost); Performance = redraw rarely at low resolution
// (best sim fps, choppiest/softest glass).
constexpr PresetSpec kPresets[] = {
    {"Full (every frame)", 1, 1, 1.00f},
    {"Smooth", 2, 3, 0.85f},
    {"Balanced", 3, 4, 0.70f},
    {"Performance", 5, 6, 0.55f},
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
  const char* label;   // for diagnostics ("PFD" / "MFD")
  XPLMAvionicsID handle = nullptr;
  std::unique_ptr<avionics::NanoVgRenderer> renderer;
  std::unique_ptr<avionics::AvionicsEngine> engine;
  float lastRenderElapsed = 0.0f;  // XPLMGetElapsedTime at our last scene render
  int frameCounter = 0;            // draw-callback invocations since last reset

  // Offscreen render-to-texture cache.
  GLuint fbo = 0;
  GLuint tex = 0;
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
    "PFD"};
AvionicsDevice g_mfd{
    avionics::DisplayPage::MultiFunctionDisplay, /*drivesSource=*/false,
    kPresets[static_cast<int>(kDefaultPreset)].mfdEveryN,
    kPresets[static_cast<int>(kDefaultPreset)].mfdRenderScale, "MFD"};

// The active quality preset, and the menu that selects it.
RatePreset g_preset = kDefaultPreset;
XPLMMenuID g_rateMenu = nullptr;
int g_rateMenuParentItem = -1;

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
  if (dev.fbo == 0 || dev.tex == 0) return false;

  const GLint filter = smooth ? GL_LINEAR : GL_NEAREST;
  glBindTexture(GL_TEXTURE_2D, dev.tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  GLint prevFbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &prevFbo);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, dev.fbo);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                            GL_TEXTURE_2D, dev.tex, 0);
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
  glClear(GL_COLOR_BUFFER_BIT);

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

int DrawDevice(AvionicsDevice& dev) {
  InvalidateAvionicsCacheIfMapGeometryChanged();
  if (!g_dataSource || dev.rendererFailed) return 1;

  if (g_dataSource && g_mfd.engine) {
    const avionics::MfdController& ui = g_mfd.engine->mfdController();
    g_dataSource->syncWeatherRadar(ui);
    // Keep the moving-map feature/airspace queries centered on the Map Pointer
    // while panning so the panned-to area loads data, not just around ownship.
    g_dataSource->setMapPanCenter(ui.mapPointerActive(), ui.mapPointerLat(),
                                  ui.mapPointerLon());
  }

  if (!dev.engine) {
    // NanoVG's GL backend is created here, where X-Plane has made the device's
    // GL context current. The bridge is OpenGL 2.1, so use the GL2 backend.
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
    dev.engine->skipBoot();  // the sim is already running; no power-on self test
    // Restore the durable display preferences saved on the last run so the
    // glass comes up the way the pilot left it (e.g. PFD inset map on/off).
    if (dev.page == avionics::DisplayPage::PrimaryFlightDisplay) {
      avionics::applyPfdState(dev.engine->softkeyController(),
                              g_avionicsState.pfd);
    } else {
      avionics::applyMfdState(dev.engine->mfdController(), g_avionicsState.mfd);
    }
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
  const bool doRender =
      !dev.cacheReady || (dev.frameCounter % dev.renderEveryN) == 0;
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
  return RunDevice(g_pfd);
}

int MfdDrawCallback(XPLMDeviceID /*device*/, int isBefore, void* /*ref*/) {
  if (!isBefore) return 1;
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
// The physical GDU keys in the 3D cockpit fire X-Plane commands
// (sim/GPS/g1000nN_*) that drive X-Plane's stock G1000. Since we've taken over
// the screen, we intercept those commands and route them to our own engine
// instead, consuming them so the hidden stock G1000 doesn't diverge. n1 is the
// pilot PFD, n3 is the MFD (n2 is the copilot PFD, which we don't drive).

// Named GDU key -> our BezelKey. Softkeys 1..12 are handled separately as a
// numeric range (softkeyN -> pressSoftkey(N-1)).
struct NamedKey {
  const char* suffix;
  avionics::BezelKey key;
};
const NamedKey kNamedKeys[] = {
    {"direct", avionics::BezelKey::DirectTo},
    {"menu", avionics::BezelKey::Menu},
    {"fpl", avionics::BezelKey::Fpl},
    {"proc", avionics::BezelKey::Proc},
    {"clr", avionics::BezelKey::Clr},
    {"ent", avionics::BezelKey::Ent},
    {"cursor", avionics::BezelKey::FmsPush},
    {"fms_outer_up", avionics::BezelKey::FmsOuterCw},
    {"fms_outer_down", avionics::BezelKey::FmsOuterCcw},
    {"fms_inner_up", avionics::BezelKey::FmsInnerCw},
    {"fms_inner_down", avionics::BezelKey::FmsInnerCcw},
    {"range_up", avionics::BezelKey::RangeUp},
    {"range_down", avionics::BezelKey::RangeDown},
    // RANGE joystick map panning (X-Plane g1000nN_pan_*). The center push
    // activates the Map Pointer; the cardinal moves pan it.
    {"pan_push", avionics::BezelKey::PanPush},
    {"pan_up", avionics::BezelKey::PanUp},
    {"pan_down", avionics::BezelKey::PanDown},
    {"pan_left", avionics::BezelKey::PanLeft},
    {"pan_right", avionics::BezelKey::PanRight},
};

// Diagonal joystick pushes (X-Plane g1000nN_pan_up_left, etc.) have no single
// BezelKey; each fires the two cardinal pans it combines.
struct DiagonalKey {
  const char* suffix;
  avionics::BezelKey a;
  avionics::BezelKey b;
};
const DiagonalKey kDiagonalKeys[] = {
    {"pan_up_left", avionics::BezelKey::PanUp, avionics::BezelKey::PanLeft},
    {"pan_up_right", avionics::BezelKey::PanUp, avionics::BezelKey::PanRight},
    {"pan_down_left", avionics::BezelKey::PanDown, avionics::BezelKey::PanLeft},
    {"pan_down_right", avionics::BezelKey::PanDown,
     avionics::BezelKey::PanRight},
};

// ---- dedicated NAV/COM knob commands ----
// The cockpit COM/NAV knobs fire their own X-Plane commands (g1000nN_com12 /
// nav12 toggle, com/nav inner/outer tuning rings, com/nav flip-flop) rather
// than the FMS knob set above. We route every GDU's knobs (PFD n1 and MFD n3)
// to the PFD engine, which owns the NAV/COM bar state.
enum class RadioAction {
  ComToggle,
  ComFlip,
  ComOuterUp,
  ComOuterDown,
  ComInnerUp,
  ComInnerDown,
  NavToggle,
  NavFlip,
  NavOuterUp,
  NavOuterDown,
  NavInnerUp,
  NavInnerDown,
};

struct RadioCommand {
  const char* suffix;
  RadioAction action;
};
const RadioCommand kRadioCommands[] = {
    {"com12", RadioAction::ComToggle},
    {"com_ff", RadioAction::ComFlip},
    {"com_outer_up", RadioAction::ComOuterUp},
    {"com_outer_down", RadioAction::ComOuterDown},
    {"com_inner_up", RadioAction::ComInnerUp},
    {"com_inner_down", RadioAction::ComInnerDown},
    {"nav12", RadioAction::NavToggle},
    {"nav_ff", RadioAction::NavFlip},
    {"nav_outer_up", RadioAction::NavOuterUp},
    {"nav_outer_down", RadioAction::NavOuterDown},
    {"nav_inner_up", RadioAction::NavInnerUp},
    {"nav_inner_down", RadioAction::NavInnerDown},
};

struct RadioCommandBinding {
  RadioAction action;
  XPLMCommandRef cmd;
};
std::vector<RadioCommandBinding> g_radioBindings;

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

  int xpdrCode = 0;
  if (sk.consumeXpdrCodeCommit(xpdrCode)) {
    g_dataSource->setTransponderCode(xpdrCode);
  }
  int xpdrMode = 0;
  if (sk.consumeXpdrModeCommit(xpdrMode)) {
    g_dataSource->setTransponderMode(xpdrMode);
  }
}

int G1000CommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                        void* ref) {
  // Consume every phase so the stock G1000 never sees these keys. Most keys act
  // on the key-down edge; CLR also has a press-and-hold function (CLR DFLT MAP),
  // which fires from the Continue phase once the button has been held.
  auto* b = static_cast<CommandBinding*>(ref);
  if (b == nullptr || !b->dev->engine) return 0;
  const bool isClr =
      !b->isSoftkey && b->value == static_cast<int>(avionics::BezelKey::Clr);

  if (phase == xplm_CommandBegin) {
    if (b->isSoftkey) {
      b->dev->engine->pressSoftkey(b->value);
    } else {
      b->dev->engine->pressBezelKey(static_cast<avionics::BezelKey>(b->value));
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
  } else if (phase == xplm_CommandContinue) {
    // CLR held past the threshold acts as CLR (DFLT MAP): jump to the MFD
    // Navigation Map page. Fire once per hold.
    if (isClr && !b->holdFired &&
        XPLMGetElapsedTime() - b->holdStart >=
            static_cast<float>(avionics::kClrDefaultMapHoldSeconds)) {
      b->dev->engine->holdBezelKey(avionics::BezelKey::Clr);
      b->holdFired = true;
      PersistStateIfChanged();
    }
  } else if (phase == xplm_CommandEnd) {
    if (isClr) b->holdFired = false;
  }
  return 0;  // consume: these GDU keys drive our display, not the stock G1000
}

// Dispatches a NAV/COM knob command to the PFD engine (which owns the bar) and
// drains the resulting tune/transfer into the radio datarefs. The COM/NAV
// outer ring steps whole MHz (coarse); the inner ring steps one channel.
int RadioCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                        void* ref) {
  if (phase == xplm_CommandBegin) {
    auto* b = static_cast<RadioCommandBinding*>(ref);
    if (b != nullptr && g_pfd.engine) {
      avionics::AvionicsEngine& e = *g_pfd.engine;
      switch (b->action) {
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
      }
      ApplyQueuedRadioCommands();
    }
  }
  return 0;  // consume: the knobs drive our radios, not the stock G1000
}

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

// Builds the binding list for one device (prefix "g1000n1" / "g1000n3"). The
// list is filled completely before any handler is registered so the refcon
// pointers stay valid.
void CollectDeviceCommands(AvionicsDevice& dev, const char* prefix) {
  char name[64];
  for (int i = 1; i <= avionics::kSoftkeyCount; ++i) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_softkey%d", prefix, i);
    BindCommand(dev, name, /*isSoftkey=*/true, i - 1);
  }
  for (const NamedKey& nk : kNamedKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", prefix, nk.suffix);
    BindCommand(dev, name, /*isSoftkey=*/false, static_cast<int>(nk.key));
  }
  for (const DiagonalKey& dk : kDiagonalKeys) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", prefix, dk.suffix);
    BindDiagonal(dev, name, dk.a, dk.b);
  }
}

// Collects the NAV/COM knob commands for one GDU prefix. Both GDUs' knobs tune
// the same radios, so they all route through RadioCommandHandler to the PFD
// engine; the binding only needs the action.
void CollectRadioCommands(const char* prefix) {
  char name[64];
  for (const RadioCommand& rc : kRadioCommands) {
    std::snprintf(name, sizeof(name), "sim/GPS/%s_%s", prefix, rc.suffix);
    XPLMCommandRef cmd = XPLMFindCommand(name);
    if (cmd == nullptr) continue;
    g_radioBindings.push_back({rc.action, cmd});
  }
}

void RegisterG1000Commands() {
  g_commandBindings.clear();
  // 2 devices x (12 softkeys + named keys + diagonal pans).
  g_commandBindings.reserve(96);
  CollectDeviceCommands(g_pfd, "g1000n1");
  CollectDeviceCommands(g_mfd, "g1000n3");
  for (CommandBinding& b : g_commandBindings) {
    XPLMRegisterCommandHandler(b.cmd, &G1000CommandHandler, /*before=*/1, &b);
  }

  // NAV/COM knobs on both GDUs, all routed to the PFD engine's bar.
  g_radioBindings.clear();
  g_radioBindings.reserve(2 * (sizeof(kRadioCommands) / sizeof(kRadioCommands[0])));
  CollectRadioCommands("g1000n1");
  CollectRadioCommands("g1000n3");
  for (RadioCommandBinding& b : g_radioBindings) {
    XPLMRegisterCommandHandler(b.cmd, &RadioCommandHandler, /*before=*/1, &b);
  }
}

void UnregisterG1000Commands() {
  for (CommandBinding& b : g_commandBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &G1000CommandHandler, /*before=*/1, &b);
  }
  g_commandBindings.clear();
  for (RadioCommandBinding& b : g_radioBindings) {
    XPLMUnregisterCommandHandler(b.cmd, &RadioCommandHandler, /*before=*/1, &b);
  }
  g_radioBindings.clear();
}

// ---- rate-preset menu --------------------------------------------------------
// Puts a check mark beside the active preset and clears the others.
void RefreshRateMenuChecks() {
  if (g_rateMenu == nullptr) return;
  for (int i = 0; i < kPresetCount; ++i) {
    XPLMCheckMenuItem(g_rateMenu, i,
                      static_cast<int>(i) == static_cast<int>(g_preset)
                          ? xplm_Menu_Checked
                          : xplm_Menu_Unchecked);
  }
}

void OnRateMenuItem(void* /*menuRef*/, void* itemRef) {
  // itemRef carries the preset index (stuffed into the pointer at append time).
  const int idx = static_cast<int>(reinterpret_cast<intptr_t>(itemRef));
  if (idx < 0 || idx >= kPresetCount) return;
  ApplyPreset(static_cast<RatePreset>(idx));
  SaveConfig();
  RefreshRateMenuChecks();
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
  dev.texW = 0;
  dev.texH = 0;
  dev.cacheReady = false;
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
  std::strcpy(outName, "G1000 NXi");
  std::strcpy(outSig, "com.andrewmiller.xplaneavionics");
  std::strcpy(outDesc, "Shared-core glass cockpit (PFD/MFD) for X-Plane.");

  // Opt into POSIX paths. Without this the SDK returns legacy HFS-style paths
  // (e.g. "Macintosh HD:Users:...") from XPLMGetSystemPath/XPLMGetPrefsPath/the
  // plugin path, which our std::ifstream/stat file probes can't open. Must run
  // before any path-returning SDK call (map data discovery, config load).
  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

  g_eisStore = std::make_unique<avionics::EisStore>();
  g_dataSource = std::make_unique<avionics::DatarefDataSource>(g_eisStore.get());

  g_flightPlanBridge = std::make_unique<avionics::FlightPlanBridge>(
      avionics::fpbridge::kDefaultPort);

  // Restore the saved config (refresh-rate preset + durable display
  // preferences) before building the menu, so the right item starts checked
  // and the engines pick up the saved options when first created.
  LoadConfig();
  BuildRateMenu();

  // Take over the built-in G1000 PFD (pilot side) and MFD: our before-callbacks
  // draw our own glass and return 0 to suppress X-Plane's stock rendering.
  RegisterDevice(g_pfd, xplm_device_G1000_PFD_1, &PfdDrawCallback);
  RegisterDevice(g_mfd, xplm_device_G1000_MFD, &MfdDrawCallback);
  return 1;
}

PLUGIN_API void XPluginStop(void) {
  DestroyRateMenu();
  ShutdownDevice(g_pfd);
  ShutdownDevice(g_mfd);
  g_flightPlanBridge.reset();
  g_dataSource.reset();
  g_eisStore.reset();
}

// The flight-plan bridge owns a flight-loop callback and a network thread, so
// it follows X-Plane's enable/disable lifecycle rather than start/stop.
PLUGIN_API int XPluginEnable(void) {
  if (g_flightPlanBridge) g_flightPlanBridge->start();
  // Take over the GDU bezel/softkey commands so they drive our display. Done on
  // enable (not start) so disabling the plugin cleanly returns the keys to the
  // stock G1000.
  RegisterG1000Commands();
  return 1;
}
PLUGIN_API void XPluginDisable(void) {
  UnregisterG1000Commands();
  if (g_flightPlanBridge) g_flightPlanBridge->stop();
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
