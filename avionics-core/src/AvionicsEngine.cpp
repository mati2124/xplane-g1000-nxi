#include "avionics/AvionicsEngine.h"

#include <utility>

#include "avionics/render/BootScreen.h"
#include "avionics/render/PrimaryFlightDisplay.h"

namespace avionics {

namespace {

// Marks every sensor-backed instrument invalid so the PFD draws each gauge's
// own red-X failure annunciation. Used when the data link is not delivering
// fresh data: rather than a single full-screen "no data" page, each instrument
// shows its failed state, matching real EFIS reversionary behavior.
FlightData withAllSensorsFailed(FlightData data) {
  data.attitudeValid = false;
  data.headingValid = false;
  data.airspeedValid = false;
  data.altitudeValid = false;
  data.verticalSpeedValid = false;
  data.navSignalValid = false;
  data.windValid = false;
  data.bearing1Valid = false;
  data.bearing2Valid = false;
  return data;
}

}  // namespace

AvionicsEngine::AvionicsEngine(DataSource& dataSource, Renderer& renderer,
                               std::string sourceLabel)
    : dataSource_(&dataSource),
      renderer_(renderer),
      sourceLabel_(std::move(sourceLabel)) {}

void AvionicsEngine::setDataSource(DataSource& dataSource,
                                   std::string sourceLabel) {
  dataSource_ = &dataSource;
  sourceLabel_ = std::move(sourceLabel);
  bootElapsedSeconds_ = 0.0;  // re-run the self-test for the new source
}

void AvionicsEngine::update(double dtSeconds) {
  bootElapsedSeconds_ += dtSeconds;
  dataSource_->update(dtSeconds);
  softkeys_.update(dtSeconds, dataSource_->snapshot());
}

bool AvionicsEngine::isLivePageUp() const {
  return bootElapsedSeconds_ >= kBootDurationSeconds &&
         dataSource_->connectionState() == ConnectionState::Connected;
}

void AvionicsEngine::onPointerDown(double xPx, double yPx) {
  if (!isLivePageUp() || lastWidthPx_ <= 0 || lastHeightPx_ <= 0) return;
  if (page_ != DisplayPage::PrimaryFlightDisplay) return;
  softkeys_.pointerDown(static_cast<float>(xPx), static_cast<float>(yPx),
                        static_cast<float>(lastWidthPx_),
                        static_cast<float>(lastHeightPx_));
}

void AvionicsEngine::renderFrame(int widthPx, int heightPx, float pixelRatio) {
  lastWidthPx_ = widthPx;
  lastHeightPx_ = heightPx;
  renderer_.beginFrame(widthPx, heightPx, pixelRatio);

  if (bootElapsedSeconds_ < kBootDurationSeconds) {
    const float progress =
        static_cast<float>(bootElapsedSeconds_ / kBootDurationSeconds);
    BootScreen::render(renderer_, sourceLabel_, progress, widthPx, heightPx);
    renderer_.endFrame();
    return;
  }

  // When the link is not delivering fresh data, draw the live page but with all
  // sensors marked failed, so each instrument shows its own red-X failure
  // annunciation rather than (potentially misleading) stale data.
  const bool connected =
      dataSource_->connectionState() == ConnectionState::Connected;
  const FlightData data = connected ? dataSource_->snapshot()
                                    : withAllSensorsFailed(dataSource_->snapshot());
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      PrimaryFlightDisplay::render(renderer_, data, softkeys_, widthPx,
                                   heightPx);
      break;
    case DisplayPage::MultiFunctionDisplay:
      // TODO: MFD page groups (MAP / WPT / AUX / NRST).
      break;
  }

  renderer_.endFrame();
}

}  // namespace avionics
