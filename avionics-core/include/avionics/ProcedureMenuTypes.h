#pragma once

#include <string>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Shared Procedures window types (PFD PROC + MFD FPL PROC, Pilot's Guide 5.8).
// The top-level menu items, verbatim and in order, from the real unit (Working
// Title PFDProc). Activate Vector-to-Final is not modeled; Activate Approach
// and Activate Missed Approach are enabled when a loaded approach (and missed
// segment) is available.
enum class ProcMenuAction {
  ActivateVtf,
  ActivateApproach,
  ActivateMissed,
  SelectApproach,
  SelectArrival,
  SelectDeparture,
};

enum class ProcMode { Menu, Select };

enum class ProcStep { AirportList, ProcedureList, TransitionList };

// Select Approach detail-form fields the FMS cursor can highlight. MinsAlt is
// reached from Mins via the small knob when minimums are BARO; it is skipped by
// the large-knob field walk. Id is display-only and is also skipped by the walk.
enum class ProcApproachField {
  Airport,
  Apr,
  Trans,
  Mins,
  MinsAlt,
  Id,
  Load,
  Activate,
  Count
};

struct ProcMenuItem {
  std::string text;
  ProcMenuAction action = ProcMenuAction::SelectApproach;
  bool enabled = true;
};

// Mutable Procedures-window state shared between the PFD and MFD controllers.
struct ProcedureMenuState {
  std::vector<ProcMenuItem> menuItems;
  int menuSel = 0;
  ProcMode mode = ProcMode::Menu;
  ProcStep step = ProcStep::ProcedureList;
  ProcedureType category = ProcedureType::Approach;
  int selected = 0;
  std::string selectedName;
  std::string selectedTransition;
  std::string selectedAirportIcao;
  bool subListOpen = false;
  ProcApproachField approachField = ProcApproachField::Airport;
  // The field cursor can descend past the form fields into the Sequence box,
  // where each previewed leg row is individually highlightable. While focused,
  // sequenceSelected indexes the leg under the cursor.
  bool sequenceFocused = false;
  int sequenceSelected = 0;
  bool loadArmed = false;
  bool activateArmed = false;
  bool loadPending = false;
  MapProcedure loadTarget{};
};

}  // namespace avionics
