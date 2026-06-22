#include "avionics/MfdController.h"

// WPT facility ident entry (Pilot's Guide, Waypoint Pages): the small knob
// spells an identifier, ENT selects the resolved waypoint for the page.
namespace avionics {

void MfdController::wptResetInteraction() {
  wptEntry_.reset();
  wptHasSelection_ = false;
  wptFeature_ = MapFeature{};
}

void MfdController::wptCommitEntry() {
  if (wptEntry_.chars.empty()) {
    wptEntry_.active = false;
    return;
  }
  if (!wptEntry_.hasMatch) {
    wptEntry_.notFound = true;
    return;
  }
  wptFeature_ = wptEntry_.match;
  wptHasSelection_ = true;
  wptEntry_.active = false;
  wptEntry_.notFound = false;
}

bool MfdController::wptBezelKey(BezelKey key) {
  if (isMapRangePanBezelKey(key)) return false;

  if (wptEntry_.active) {
    switch (key) {
      case BezelKey::Ent:
        wptCommitEntry();
        break;
      case BezelKey::Clr:
      case BezelKey::FmsPush:
        wptEntry_.active = false;
        wptEntry_.notFound = false;
        break;
      case BezelKey::FmsInnerCw:
        wptEntry_.turnChar(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsInnerCcw:
        wptEntry_.turnChar(navSource_, mapData_, -1);
        break;
      case BezelKey::FmsOuterCw:
        wptEntry_.moveCursor(navSource_, mapData_, +1);
        break;
      case BezelKey::FmsOuterCcw:
        wptEntry_.moveCursor(navSource_, mapData_, -1);
        break;
      default:
        break;
    }
    return true;
  }

  switch (key) {
    case BezelKey::FmsPush:
      wptEntry_.open(navSource_, mapData_, wptHasSelection_ ? wptFeature_.id : "");
      return true;
    case BezelKey::FmsInnerCw:
    case BezelKey::FmsInnerCcw:
      wptEntry_.open(navSource_, mapData_);
      return true;
    default:
      return false;
  }
}

}  // namespace avionics
