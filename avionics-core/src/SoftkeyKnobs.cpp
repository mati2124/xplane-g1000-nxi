#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cmath>

#include "avionics/render/BezelKeys.h"

// Dedicated HDG, CRS, and BARO knobs (left/right bezel): each click steps the
// selected-heading bug, selected course, or altimeter barometric setting by one
// and queues the new value as a sim write (consumed by the shell). The knob
// pushes sync the bug/course to the current heading or set standard pressure.
namespace avionics {
namespace {

// One click of the HDG / CRS knob steps one degree; the BARO knob steps 0.01
// inHg per click. The barometric setting clamps to the GDU's settable range.
constexpr float kBaroStepInHg = 0.01f;
constexpr float kBaroMinInHg = 27.50f;
constexpr float kBaroMaxInHg = 31.50f;

float wrapHeadingDeg(float deg) {
  deg = std::fmod(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

}  // namespace

void SoftkeyController::adjustHeadingBug(int direction, const FlightData& d) {
  headingBugDeg_ = wrapHeadingDeg(std::round(d.selectedHeadingDeg) +
                                  static_cast<float>(direction));
  headingBugPending_ = true;
}

void SoftkeyController::syncHeadingBug(const FlightData& d) {
  headingBugDeg_ = wrapHeadingDeg(std::round(d.headingDeg));
  headingBugPending_ = true;
}

void SoftkeyController::adjustCourse(int direction, const FlightData& d) {
  coursePendingDeg_ =
      wrapHeadingDeg(std::round(d.courseDeg) + static_cast<float>(direction));
  coursePending_ = true;
}

void SoftkeyController::adjustBaro(int direction, const FlightData& d) {
  baroPendingInHg_ = std::clamp(
      d.baroSettingInHg + static_cast<float>(direction) * kBaroStepInHg,
      kBaroMinInHg, kBaroMaxInHg);
  baroPending_ = true;
}

void SoftkeyController::setBaroStandard() {
  baroPendingInHg_ = kBaroStandardInHg;
  baroPending_ = true;
}

void SoftkeyController::flashBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i >= 0 && i < kBezelKeyCount) bezelPress_[i] = 1.0f;
}

bool SoftkeyController::consumeHeadingBug(float& deg) {
  if (!headingBugPending_) return false;
  deg = headingBugDeg_;
  headingBugPending_ = false;
  return true;
}

bool SoftkeyController::consumeCourse(float& deg) {
  if (!coursePending_) return false;
  deg = coursePendingDeg_;
  coursePending_ = false;
  return true;
}

bool SoftkeyController::consumeBaro(float& inHg) {
  if (!baroPending_) return false;
  inHg = baroPendingInHg_;
  baroPending_ = false;
  return true;
}

}  // namespace avionics
