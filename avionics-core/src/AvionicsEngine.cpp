#include "avionics/AvionicsEngine.h"

#include <utility>

#include "avionics/render/BootScreen.h"
#include "avionics/render/MultiFunctionDisplay.h"
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
  // The link is down, so the non-sensor chrome readouts (radios, FMA,
  // transponder, OAT, clock) are unknown too: blank them / dash them out
  // rather than leaving the last-received values on screen.
  data.dataLinkValid = false;
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
  // A secondary engine sharing the source (the MFD window) must not pump it a
  // second time; it still advances its own boot timer and UI animations.
  if (drivesDataSource_) dataSource_->update(dtSeconds);
  softkeys_.update(dtSeconds, dataSource_->snapshot(),
                   dataSource_->mapSnapshot());
  mfd_.update(dtSeconds);
  // Keep the MFD's checklist navigation in step with the loaded file (the data
  // is owned by the source; the controller only holds the interactive state).
  mfd_.syncChecklist(dataSource_->checklistSnapshot());
}

bool AvionicsEngine::isLivePageUp() const {
  return bootElapsedSeconds_ >= kBootDurationSeconds &&
         dataSource_->connectionState() == ConnectionState::Connected;
}

void AvionicsEngine::pressSoftkey(int index) {
  if (!isLivePageUp()) return;
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      softkeys_.pressKey(index);
      break;
    case DisplayPage::MultiFunctionDisplay:
      mfd_.pressKey(index);
      break;
  }
}

const float* AvionicsEngine::softkeyPressLevels() const {
  return page_ == DisplayPage::MultiFunctionDisplay ? mfd_.pressLevels()
                                                    : softkeys_.pressLevels();
}

void AvionicsEngine::pressBezelKey(BezelKey key) {
  if (!isLivePageUp()) return;
  switch (page_) {
    case DisplayPage::PrimaryFlightDisplay:
      softkeys_.pressBezelKey(key);
      break;
    case DisplayPage::MultiFunctionDisplay:
      mfd_.pressBezelKey(key);
      break;
  }
}

const float* AvionicsEngine::bezelPressLevels() const {
  return page_ == DisplayPage::MultiFunctionDisplay ? mfd_.bezelPressLevels()
                                                    : softkeys_.bezelPressLevels();
}

void AvionicsEngine::renderFrame(int widthPx, int heightPx, float pixelRatio) {
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
      PrimaryFlightDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                   softkeys_, widthPx, heightPx);
      break;
    case DisplayPage::MultiFunctionDisplay:
      MultiFunctionDisplay::render(renderer_, data, dataSource_->mapSnapshot(),
                                   dataSource_->checklistSnapshot(), mfd_,
                                   widthPx, heightPx);
      break;
  }

  renderer_.endFrame();
}

}  // namespace avionics
