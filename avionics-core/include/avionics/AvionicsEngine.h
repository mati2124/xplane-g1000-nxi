#pragma once

#include <string>

#include "avionics/DataSource.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"

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

  // Swap the live data source (e.g. toggling mock <-> X-Plane at runtime). The
  // boot sequence restarts so the new source's acquisition is shown cleanly.
  void setDataSource(DataSource& dataSource, std::string sourceLabel = "");

  // Skip straight to the live page, bypassing the boot animation. Used by the
  // deterministic screenshot path.
  void skipBoot() { bootElapsedSeconds_ = kBootDurationSeconds; }

  // Forward a pointer press from the shell in display-pixel coordinates (same
  // space as renderFrame's width/height). Used to interact with the softkey bar
  // and the windows it opens. Ignored unless the live page is up.
  void onPointerDown(double xPx, double yPx);

 private:
  // True once the boot self-test has finished and the source is connected, i.e.
  // the interactive live page is actually on screen.
  bool isLivePageUp() const;

  DataSource* dataSource_;
  Renderer& renderer_;
  DisplayPage page_ = DisplayPage::PrimaryFlightDisplay;
  std::string sourceLabel_;
  double bootElapsedSeconds_ = 0.0;
  SoftkeyController softkeys_;
  int lastWidthPx_ = 0;
  int lastHeightPx_ = 0;
};

}  // namespace avionics
