#include "avionics/MfdController.h"

#include <algorithm>

#include "avionics/render/BezelKeys.h"

namespace avionics {
namespace {

// Softkey bar height as a fraction of the display, matching the shared GDU
// canvas (1024x768 with a 35 px softkey row), so the MFD bar hit area lines up
// with the PFD's.
constexpr float kBarHeightFraction = 35.0f / 768.0f;

// Softkey cell assignments for the MFD bar. The four page groups form a radio
// group on the left; range control sits on the right two cells, mirroring the
// G1000 MFD softkey layout.
constexpr int kKeyMap = 0;
constexpr int kKeyWaypoint = 1;
constexpr int kKeyAux = 2;
constexpr int kKeyNearest = 3;
constexpr int kKeyTerrain = 5;
constexpr int kKeyRangeDown = 10;
constexpr int kKeyRangeUp = 11;

}  // namespace

MfdController::MfdController() {
  labels_[kKeyMap] = "Map";
  labels_[kKeyWaypoint] = "WPT";
  labels_[kKeyAux] = "AUX";
  labels_[kKeyNearest] = "NRST";
  labels_[kKeyTerrain] = "TERR";
  labels_[kKeyRangeDown] = "RNG-";
  labels_[kKeyRangeUp] = "RNG+";
}

float MfdController::rangeNm() const { return mapRangeNmAt(rangeIndex_); }

void MfdController::update(double dtSeconds) {
  const float pressStep = static_cast<float>(dtSeconds) / kPressFlashSeconds;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    press_[i] = std::max(0.0f, press_[i] - pressStep);
  }
  for (int i = 0; i < kBezelKeyCount; ++i) {
    bezelPress_[i] = std::max(0.0f, bezelPress_[i] - pressStep);
  }
}

bool MfdController::keyActive(int i) const {
  switch (i) {
    case kKeyMap:
      return pageGroup_ == MfdPageGroup::Map;
    case kKeyWaypoint:
      return pageGroup_ == MfdPageGroup::Waypoint;
    case kKeyAux:
      return pageGroup_ == MfdPageGroup::Aux;
    case kKeyNearest:
      return pageGroup_ == MfdPageGroup::Nearest;
    case kKeyTerrain:
      return showTerrain_;
    default:
      return false;
  }
}

int MfdController::hitTest(float xPx, float yPx, float widthPx, float heightPx) {
  const float barH = heightPx * kBarHeightFraction;
  const float barTop = heightPx - barH;
  if (yPx < barTop || yPx > heightPx || xPx < 0.0f || xPx > widthPx) return -1;
  const float cellW = widthPx / static_cast<float>(kSoftkeyCount);
  int idx = static_cast<int>(xPx / cellW);
  return std::max(0, std::min(kSoftkeyCount - 1, idx));
}

void MfdController::pressBezelKey(BezelKey key) {
  const int i = static_cast<int>(key);
  if (i < 0 || i >= kBezelKeyCount) return;
  bezelPress_[i] = 1.0f;  // trigger the press-flash animation
  if (key == BezelKey::RangeUp) {
    rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
  } else if (key == BezelKey::RangeDown) {
    rangeIndex_ = std::max(0, rangeIndex_ - 1);
  }
}

bool MfdController::pointerDown(float xPx, float yPx, float widthPx,
                               float heightPx) {
  const int key = hitTest(xPx, yPx, widthPx, heightPx);
  if (key < 0 || labels_[key].empty()) return false;

  press_[key] = 1.0f;  // trigger the press-flash animation

  switch (key) {
    case kKeyMap:
      pageGroup_ = MfdPageGroup::Map;
      break;
    case kKeyWaypoint:
      pageGroup_ = MfdPageGroup::Waypoint;
      break;
    case kKeyAux:
      pageGroup_ = MfdPageGroup::Aux;
      break;
    case kKeyNearest:
      pageGroup_ = MfdPageGroup::Nearest;
      break;
    case kKeyTerrain:
      showTerrain_ = !showTerrain_;
      break;
    case kKeyRangeDown:
      rangeIndex_ = std::max(0, rangeIndex_ - 1);
      break;
    case kKeyRangeUp:
      rangeIndex_ = std::min(kMapRangeLadderCount - 1, rangeIndex_ + 1);
      break;
    default:
      return false;
  }
  return true;
}

}  // namespace avionics
