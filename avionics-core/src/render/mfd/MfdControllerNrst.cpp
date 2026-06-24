#include <algorithm>

#include "avionics/MfdController.h"
#include "render/mfd/MfdPageSupport.h"

// NRST nearest-list cursor (Pilot's Guide, Nearest pages): the FMS knob push
// turns the cursor on, the large knob scrolls the list, the small knob steps
// pages within the group.
namespace avionics {

void MfdController::nrstResetInteraction() {
  nrstCursorOn_ = false;
  nrstSelected_ = 0;
}

const MapFeature* MfdController::nrstSelectedFeature() const {
  if (pageGroup_ != MfdPageGroup::Nearest || mapData_ == nullptr) {
    return nullptr;
  }

  MapFeatureType type = MapFeatureType::Airport;
  int maxRows = 5;
  switch (page()) {
    case MfdPage::NearestAirports:
      type = MapFeatureType::Airport;
      maxRows = 5;
      break;
    case MfdPage::NearestIntersections:
      type = MapFeatureType::Fix;
      maxRows = 10;
      break;
    case MfdPage::NearestNdb:
      type = MapFeatureType::Ndb;
      maxRows = 10;
      break;
    case MfdPage::NearestVor:
      type = MapFeatureType::Vor;
      maxRows = 10;
      break;
    default:
      return nullptr;
  }

  const std::vector<mfd::NearRow> rows =
      mfd::collectNearest(*mapData_, type, maxRows);
  if (rows.empty()) return nullptr;
  const int idx = std::max(
      0, std::min(nrstSelected_, static_cast<int>(rows.size()) - 1));
  return rows[static_cast<std::size_t>(idx)].feature;
}

bool MfdController::nrstBezelKey(BezelKey key) {
  const MfdPage p = page();
  if (p == MfdPage::NearestFrequencies) return false;

  if (key == BezelKey::FmsPush) {
    nrstCursorOn_ = !nrstCursorOn_;
    return true;
  }

  if (nrstCursorOn_) {
    switch (key) {
      case BezelKey::FmsOuterCw:
        ++nrstSelected_;
        return true;
      case BezelKey::FmsOuterCcw:
        nrstSelected_ = std::max(0, nrstSelected_ - 1);
        return true;
      case BezelKey::FmsInnerCw:
        stepPage(1);
        return true;
      case BezelKey::FmsInnerCcw:
        stepPage(-1);
        return true;
      default:
        return false;
    }
  }

  return false;
}

}  // namespace avionics
