#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

void drawChrome(Renderer& r, const Layout& L, const FlightData& d,
                const SoftkeyController& ui, float w, float h, bool powerUp) {
  drawTopBar(r, w, h, L, d, ui);
  drawBottomInfoPanel(r, w, h, L, d, ui, powerUp);
  // CAS annunciation window is always visible (when active) on the main PFD.
  drawCasAnnunciations(r, w, h, L, ui);
  // The pop-up windows (Alerts / References / Nearest Airports) share the
  // lower-right region -- one is active at a time, but each is drawn while its
  // animation is nonzero so a replaced window fades out under the new one.
  // They sit above the info panel but below the softkey bar, so the bar (and
  // its press highlights) always stay on top.
  drawAlertsWindow(r, w, h, L, ui);
  drawReferencesWindow(r, w, h, L, ui);
  drawNearestWindow(r, w, h, L, ui);
  if (ui.flightPlanEntryActive()) {
    drawWaypointInformationWindow(r, w, h, L, ui);
  } else {
    drawFlightPlanWindow(r, w, h, L, d, ui);
  }
  drawProcWindow(r, w, h, L, ui);
  drawPfdSetupWindow(r, w, h, L, ui);
  // The Direct-To window (Direct-To bezel key) shares the lower-right region.
  drawDirectToWindow(r, w, h, L, ui);
  // The Page Menu (MENU on an open popout) overlays the active window.
  drawPageMenuWindow(r, w, h, L, ui);
  drawSoftkeyBar(r, w, h, L, ui);
}

}  // namespace avionics::pfd
