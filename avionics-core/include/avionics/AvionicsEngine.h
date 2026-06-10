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

// Power-on initialization duration. The display shows the boot screen for this
// long after power-up (or after the data source is swapped) before it brings up
// the live pages, like the self-test screen on a real EFIS.
inline constexpr double kBootDurationSeconds = 4.0;

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

  // Skip straight to the live page, bypassing the boot animation. Used by the
  // deterministic screenshot path.
  void skipBoot() { bootElapsedSeconds_ = kBootDurationSeconds; }

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
  // Press-flash levels (0..1, indexed by BezelKey) for the active page, so the
  // shell can render the key feedback.
  const float* bezelPressLevels() const;

 private:
  // True once the boot self-test has finished and the source is connected, i.e.
  // the interactive live page is actually on screen.
  bool isLivePageUp() const;

  DataSource* dataSource_;
  Renderer& renderer_;
  DisplayPage page_ = DisplayPage::PrimaryFlightDisplay;
  std::string sourceLabel_;
  double bootElapsedSeconds_ = 0.0;
  bool drivesDataSource_ = true;
  SoftkeyController softkeys_;
  MfdController mfd_;
};

}  // namespace avionics
