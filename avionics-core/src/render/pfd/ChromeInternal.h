#pragma once

#include <algorithm>
#include <string>

#include "render/pfd/PfdInternal.h"

// Shared internals for the PFD chrome. drawChrome (Chrome.cpp) is the
// orchestrator; each chrome component is its own translation unit
// (ChromeTopBar.cpp, ChromeInfoPanel.cpp, ChromeSoftkeyBar.cpp, and one file
// per pop-up window). They all share the placeholder strings, the eased
// slide/fade helpers, and the pop-up window frame declared here, mirroring the
// per-instrument split elsewhere under render/pfd.
namespace avionics::pfd {

// Placeholders shown for the chrome data readouts when the data link is down,
// so a dead feed reads as unknown rather than as stale live values.
constexpr const char* kFreqDash2 = "---.--";
constexpr const char* kFreqDash3 = "---.---";
constexpr const char* kOatDashes = "---";
constexpr const char* kXpdrDashes = "----";
constexpr const char* kTimeDashes = "--:--:--";

// Smooth Hermite ease for window slide/fade, matching the Working Title feel.
inline float smoothstep(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return t * t * (3.0f - 2.0f * t);
}

inline Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Shared frame for the PFD pop-up windows (Alerts / References / Nearest
// Airports). Anchored above the softkey bar at the lower right and animated
// with an eased slide-up + fade, driven by the controller's 0..1 progress.
// Returns the eased alpha (0 = nothing drawn) and the panel geometry.
struct WindowFrame {
  float a = 0.0f;
  float x = 0.0f, top = 0.0f, w = 0.0f, h = 0.0f;
  float contentTop = 0.0f;  // first content line below the title bar
};

WindowFrame drawWindowFrame(Renderer& r, float w, float h, const Layout& L,
                            float rawAnim, const char* title, float panelW,
                            float panelH);

// One field of a pop-up window: text with an optional highlight-select cursor
// (pulses cyan plate / black text vs plain cyan text). Returns the x just past
// the field. Shared by the References and PFD Setup windows.
float putField(Renderer& r, float x, float cy, const std::string& text,
               float size, const Color& color, bool highlighted, float alpha,
               bool blinkOn, float trailingGapFrac = 0.6f,
               FontFace face = FontFace::Default);

// --- Per-component entry points (called from drawChrome). ---

void drawTopBar(Renderer& r, float w, float h, const Layout& L,
                const FlightData& d, const SoftkeyController& ui);
void drawBottomInfoPanel(Renderer& r, float w, float h, const Layout& L,
                         const FlightData& d, const SoftkeyController& ui);
void drawSoftkeyBar(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui);
void drawCasAnnunciations(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui);
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui);
void drawReferencesWindow(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui);
void drawNearestWindow(Renderer& r, float w, float h, const Layout& L,
                       const SoftkeyController& ui);
void drawPfdSetupWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui);
void drawDirectToWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui);

}  // namespace avionics::pfd
