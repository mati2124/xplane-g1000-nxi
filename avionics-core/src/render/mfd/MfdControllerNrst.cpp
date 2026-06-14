#include <algorithm>

#include "avionics/MfdController.h"

// NRST nearest-list cursor (Pilot's Guide, Nearest pages): the FMS knob push
// turns the cursor on, the large knob scrolls the list, the small knob steps
// pages within the group.
namespace avionics {

void MfdController::nrstResetInteraction() {
  nrstCursorOn_ = false;
  nrstSelected_ = 0;
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
