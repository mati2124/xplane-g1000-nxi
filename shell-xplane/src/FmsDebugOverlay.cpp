#include "FmsDebugOverlay.h"

#include <cstdio>
#include <cstring>

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMNavigation.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

namespace avionics {
namespace {

constexpr int kMaxEvents = 8;
constexpr int kFmsIdBufferSize = 256;
constexpr int kLineChars = 88;
constexpr int kLineStep = 26;
// SDK marks these fonts deprecated, but X-Plane still renders them and they
// are much larger than xplmFont_Basic / xplmFont_Proportional.
constexpr XPLMFontID kOverlayFont = static_cast<XPLMFontID>(4);  // LedWide
constexpr XPLMFontID kTitleFont = static_cast<XPLMFontID>(4);

struct WriteEvent {
  float simTime = 0.0f;
  char kind[24] = {};
  char detail[kLineChars] = {};
};

bool g_enabled = false;
bool g_drawRegistered = false;
WriteEvent g_events[kMaxEvents];
int g_eventCount = 0;
FmsDebugNavState g_navState{};

void pushEvent(const char* kind, const char* detail) {
  if (kind == nullptr) kind = "?";
  if (detail == nullptr) detail = "";

  WriteEvent* ev = nullptr;
  if (g_eventCount < kMaxEvents) {
    ev = &g_events[g_eventCount++];
  } else {
    for (int i = 1; i < kMaxEvents; ++i) {
      g_events[i - 1] = g_events[i];
    }
    ev = &g_events[kMaxEvents - 1];
  }

  ev->simTime = XPLMGetElapsedTime();
  std::snprintf(ev->kind, sizeof(ev->kind), "%s", kind);
  std::snprintf(ev->detail, sizeof(ev->detail), "%s", detail);
}

void drawLine(int x, int& y, const float color[3], const char* text,
              XPLMFontID font = kOverlayFont) {
  if (text == nullptr || text[0] == '\0') return;
  XPLMDrawString(const_cast<float*>(color), x, y, const_cast<char*>(text),
                 nullptr, font);
  y -= kLineStep;
}

void appendFmsLegSummary(char* out, std::size_t outSize) {
  if (outSize == 0) return;
  out[0] = '\0';

  const int count = XPLMCountFMSEntries();
  if (count <= 0) return;

  std::size_t used = 0;
  for (int i = 0; i < count && used + 1 < outSize; ++i) {
    XPLMNavType type = xplm_Nav_Unknown;
    char id[kFmsIdBufferSize] = {};
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int altitude = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    XPLMGetFMSEntryInfo(i, &type, id, &ref, &altitude, &lat, &lon);
    id[sizeof(id) - 1] = '\0';

    char piece[32] = {};
    if (id[0] != '\0') {
      std::snprintf(piece, sizeof(piece), "%d:%s", i, id);
    } else if (lat != 0.0f || lon != 0.0f) {
      std::snprintf(piece, sizeof(piece), "%d:%.3f,%.3f", i, lat, lon);
    } else {
      std::snprintf(piece, sizeof(piece), "%d:?", i);
    }

    if (used > 0) {
      const int n = std::snprintf(out + used, outSize - used, " ");
      if (n < 0) break;
      used += static_cast<std::size_t>(n);
    }
    const int n = std::snprintf(out + used, outSize - used, "%s", piece);
    if (n < 0) break;
    used += static_cast<std::size_t>(n);
    if (used + 16 >= outSize) {
      std::snprintf(out + used, outSize - used, " ...");
      break;
    }
  }
}

int DrawCallback(XPLMDrawingPhase /*phase*/, int /*isBefore*/, void* /*refcon*/) {
  if (!g_enabled) return 1;

  const float white[3] = {1.0f, 1.0f, 1.0f};
  const float yellow[3] = {1.0f, 1.0f, 0.4f};
  const float cyan[3] = {0.5f, 1.0f, 1.0f};
  const float green[3] = {0.4f, 1.0f, 0.4f};

  char line[kLineChars] = {};
  const int x = 20;
  int y = 720;

  const int lineCount = 6 + g_eventCount;
  const int left = x - 10;
  const int top = y + 14;
  const int right = x + 780;
  const int bottom = y - (kLineStep * lineCount) - 6;

  XPLMDrawTranslucentDarkBox(left, top, right, bottom);

  drawLine(x, y, yellow, "G1000 NXi  FMS / flight-plan debug", kTitleFont);

  std::snprintf(
      line, sizeof(line),
      "NXi leg %d/%d  FROM %s  TO %s  DTO:%s  OBS:%s",
      g_navState.activeLegIndex, g_navState.planLegCount,
      g_navState.fmaFrom[0] ? g_navState.fmaFrom : "-",
      g_navState.fmaTo[0] ? g_navState.fmaTo : "-",
      g_navState.directTo ? "on" : "off", g_navState.obsMode ? "on" : "off");
  drawLine(x, y, white, line);

  std::snprintf(line, sizeof(line),
                "NXi GPS override:%s  CRS:%.1f  CDI:%+.2f dots",
                g_navState.gpsOverride ? "ON" : "off", g_navState.courseDeg,
                g_navState.cdiDots);
  drawLine(x, y, white, line);

  const int fmsCount = XPLMCountFMSEntries();
  const int fmsDest = fmsCount > 0 ? XPLMGetDestinationFMSEntry() : -1;
  char destId[kFmsIdBufferSize] = {};
  if (fmsDest >= 0 && fmsDest < fmsCount) {
    XPLMNavType type = xplm_Nav_Unknown;
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int altitude = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    XPLMGetFMSEntryInfo(fmsDest, &type, destId, &ref, &altitude, &lat, &lon);
    destId[sizeof(destId) - 1] = '\0';
  }
  std::snprintf(line, sizeof(line),
                "X-Plane FMS  entries:%d  dest:%d  destId:%s",
                fmsCount, fmsDest, destId[0] ? destId : "-");
  drawLine(x, y, cyan, line);

  char legs[kLineChars] = {};
  appendFmsLegSummary(legs, sizeof(legs));
  if (legs[0] != '\0') {
    std::snprintf(line, sizeof(line), "FMS route: %s", legs);
    drawLine(x, y, cyan, line);
  }

  drawLine(x, y, green, "Recent writes to X-Plane:");
  if (g_eventCount == 0) {
    drawLine(x, y, white, "  (none yet)");
  } else {
    const float now = XPLMGetElapsedTime();
    for (int i = g_eventCount - 1; i >= 0; --i) {
      const WriteEvent& ev = g_events[i];
      std::snprintf(line, sizeof(line), "  %.1fs  %-10s  %s", now - ev.simTime,
                    ev.kind, ev.detail);
      drawLine(x, y, white, line);
    }
  }

  return 1;
}

}  // namespace

void FmsDebugOverlay::setEnabled(bool enabled) {
  g_enabled = enabled;
  if (g_enabled && !g_drawRegistered) {
    registerDrawCallback();
  }
}

bool FmsDebugOverlay::enabled() { return g_enabled; }

void FmsDebugOverlay::registerDrawCallback() {
  if (g_drawRegistered) return;
  XPLMRegisterDrawCallback(DrawCallback, xplm_Phase_Window, /*before=*/0,
                           nullptr);
  g_drawRegistered = true;
}

void FmsDebugOverlay::unregisterDrawCallback() {
  if (!g_drawRegistered) return;
  XPLMUnregisterDrawCallback(DrawCallback, xplm_Phase_Window, /*before=*/0,
                             nullptr);
  g_drawRegistered = false;
}

void FmsDebugOverlay::recordWrite(const char* kind, const char* detail) {
  pushEvent(kind, detail);
}

void FmsDebugOverlay::updateNavState(const FmsDebugNavState& state) {
  g_navState = state;
}

}  // namespace avionics
