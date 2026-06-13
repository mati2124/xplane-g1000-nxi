#pragma once

#include <string>

#include "avionics/DataSource.h"
#include "avionics/MfdController.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {

enum class DisplayPage {
  PrimaryFlightDisplay,
  MultiFunctionDisplay,
};

// Power-on initialization timing, mirroring the real G1000 NXi power-up.
// Working Title StartupLogo.css: the centered Garmin logo appears at full
// opacity (no fade), then the next screen fades in over 2s ease-in-out when
// ScreenState leaves INIT. The logo is held while LRUs boot; 3s is a
// representative minimum before the Power-up Page cross-fade begins.
inline constexpr double kBootLogoSeconds = 3.0;
// MFD Power-up Page / PFD init opacity ramp (StartupLogo.css: transition opacity
// 2s ease-in-out on .startup-confirm-screen).
inline constexpr double kBootPowerUpFadeSeconds = 2.0;
// Total animated power-up before the unit is either live (mock) or awaiting the
// ENT acknowledgement (live sim link).
inline constexpr double kBootDurationSeconds =
    kBootLogoSeconds + kBootPowerUpFadeSeconds;

// Ties a DataSource and a Renderer together and owns the screen state machine
// (boot -> live page / connection-lost). Both shells construct one of these and
// call update() then renderFrame() once per frame.
class AvionicsEngine {
 public:
  // sourceLabel names the data feed for the boot / connection-lost screens
  // (e.g. "X-PLANE", "MOCK DATA").
  AvionicsEngine(DataSource& dataSource, Renderer& renderer,
                 std::string sourceLabel = "");

  void update(double dtSeconds);
  void renderFrame(int widthPx, int heightPx, float pixelRatio);

  void setPage(DisplayPage page) { page_ = page; }
  DisplayPage page() const { return page_; }

  // When two engines share one DataSource (e.g. a PFD window and an MFD window
  // fed by the same sim link), only one should pump the source each frame.
  // Call setDrivesDataSource(false) on the secondary engine so update() advances
  // its own boot/UI animations without double-stepping the shared source.
  void setDrivesDataSource(bool drives) { drivesDataSource_ = drives; }

  // Swap the live data source (e.g. toggling mock <-> X-Plane at runtime). The
  // boot sequence restarts so the new source's acquisition is shown cleanly.
  void setDataSource(DataSource& dataSource, std::string sourceLabel = "");

  // Skip straight to the live page, bypassing the boot animation and the ENT
  // acknowledgement. Used by the X-Plane plugin (the sim is already running) and
  // the deterministic screenshot path.
  void skipBoot() {
    bootElapsedSeconds_ = kBootDurationSeconds;
    powerUpAcknowledged_ = true;
  }

  // True while the power-up page is up and waiting for the pilot to press ENT to
  // acknowledge the database information (only for sources that require it; the
  // mock advances on its own). The shell uses this to route ENT during boot.
  bool awaitingPowerUpAck() const;
  // Acknowledge the power-up page (the ENT key). No-op unless awaiting; brings
  // up the live pages.
  void acknowledgePowerUp();

  // ---- physical softkeys (the key row the shell draws below the screen) ----
  // Apply a press of softkey `index` (0..kSoftkeyCount-1) to whichever page is
  // active. The on-screen softkey bar is labels only, as on the real unit: all
  // interaction goes through these keys. Ignored unless the live page is up.
  void pressSoftkey(int index);
  // Press-flash levels (0..1, kSoftkeyCount entries) for the active page, so
  // the shell can animate the physical key row.
  const float* softkeyPressLevels() const;

  // ---- window bezel keys (drawn by the standalone shell around the screen) ----
  // Apply a hardware bezel key press to whichever page is active (the range
  // rocker zooms that page's map). Ignored unless the live page is up.
  void pressBezelKey(BezelKey key);
  // Apply a press-and-hold of a bezel key. Currently only CLR has a hold
  // function: on the MFD it acts as CLR (DFLT MAP) and displays the Navigation
  // Map page (the shell fires this after kClrDefaultMapHoldSeconds).
  void holdBezelKey(BezelKey key);

  // ---- dedicated NAV/COM tuning knobs ----
  // The real GDU has separate COM and NAV knobs (each with a 1/2 toggle, an
  // inner/outer tuning ring and a flip-flop key). Shells with those physical
  // controls (the X-Plane plugin maps the sim's g1000nN_com*/nav* commands)
  // call these; the PFD's NAV/COM bar owns the state. selectCom/selectNav
  // toggle which unit each side tunes; tuneCom/tuneNav step the selected
  // standby (coarse = outer ring = whole MHz); transferCom/transferNav swap
  // active and standby. No-ops unless the live page is up.
  void selectComRadio();
  void selectNavRadio();
  void tuneComRadio(int direction, bool coarse);
  void tuneNavRadio(int direction, bool coarse);
  void transferComRadio();
  void transferNavRadio();
  // Press-flash levels (0..1, indexed by BezelKey) for the active page, so the
  // shell can render the key feedback.
  const float* bezelPressLevels() const;

  // MFD softkey/page state, exposed so the shell can wire page-driven
  // integrations that live outside the core (e.g. the SimBrief OFP fetch on
  // the AUX - SIMBRIEF page: seed the Pilot ID, consume FETCH requests, and
  // publish the fetch status back for rendering).
  MfdController& mfdController() { return mfd_; }
  SoftkeyController& softkeyController() { return softkeys_; }

 private:
  // The dedicated NAV/COM/CRS/BARO/HDG knobs on the GDU bezel act on the PFD's
  // radio bar and selected references no matter which page this engine shows
  // (both GDUs carry the same knobs). Returns true when `key` was one of those
  // controls and was handled.
  bool handleBezelKnob(BezelKey key);

  // True once the animated power-up has run and (for sources that require it)
  // the power-up page has been acknowledged with ENT, independent of link
  // health.
  bool bootComplete() const;

  // True once the boot self-test has finished and the source is connected, i.e.
  // the interactive live page is actually on screen.
  bool isLivePageUp() const;

  DataSource* dataSource_;
  Renderer& renderer_;
  DisplayPage page_ = DisplayPage::PrimaryFlightDisplay;
  std::string sourceLabel_;
  double bootElapsedSeconds_ = 0.0;
  // Set once the pilot acknowledges the power-up page with ENT (or immediately
  // for feeds that don't require it / when boot is skipped).
  bool powerUpAcknowledged_ = false;
  bool drivesDataSource_ = true;
  SoftkeyController softkeys_;
  MfdController mfd_;
};

}  // namespace avionics
