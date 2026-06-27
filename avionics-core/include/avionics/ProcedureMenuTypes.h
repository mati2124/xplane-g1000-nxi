#pragma once

#include <string>
#include <vector>

#include "avionics/FmsWaypointEntry.h"
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

enum class ProcStep { AirportList, ProcedureList, TransitionList, RunwayList };

// Select Approach detail-form fields the FMS cursor can highlight. MinsAlt is
// reached from Mins via the small knob when minimums are BARO; it is skipped by
// the large-knob field walk. Id is display-only and is also skipped by the walk.
// Runway is used only by the Arrival/Departure loading forms (STAR/SID runway
// transition); it is skipped by the Approach field walk.
enum class ProcApproachField {
  Airport,
  Apr,
  Trans,
  Mins,
  MinsAlt,
  Id,
  Load,
  Activate,
  Runway,
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
  // Selected runway transition for Arrival/Departure loading (STAR/SID). Holds
  // the runway transition label ("RW05") or the kProcRunwayAll sentinel ("ALL")
  // when the procedure has no runway-specific transition.
  std::string selectedRunway;
  std::string selectedAirportIcao;
  bool subListOpen = false;
  ProcApproachField approachField = ProcApproachField::Airport;
  // Inline ICAO entry for the Airport field (small knob spells the identifier,
  // large knob moves the cursor) so the pilot can change which airport's
  // approaches are listed, like the real unit's Select Approach page.
  FmsWaypointEntry airportEntry;
  // The field cursor can descend past the form fields into the Sequence box,
  // where each previewed leg row is individually highlightable. While focused,
  // sequenceSelected indexes the leg under the cursor.
  bool sequenceFocused = false;
  int sequenceSelected = 0;
  bool loadArmed = false;
  bool activateArmed = false;
  bool loadPending = false;
  MapProcedure loadTarget{};

  // "Fly Course Reversal at <fix>?" prompt, raised when an approach is loaded
  // via an IAF that has a HILPT course reversal (ARINC HF). The pilot answers
  // YES (fly the hold) or NO (remove it / straight-in). Default selection is NO,
  // matching the trainer.
  bool courseReversalPromptActive = false;
  std::string courseReversalFix;
  int courseReversalLegIndex = -1;
  bool courseReversalYes = false;
  // Cross-display coordination latches (the engine mirrors the prompt onto the
  // peer GDU). courseReversalYesDirty marks the display the pilot just toggled
  // as authoritative for the YES/NO selection; courseReversalAnswered marks that
  // the prompt was answered so the engine can clear it on both GDUs.
  bool courseReversalYesDirty = false;
  bool courseReversalAnswered = false;
  // Pilot's YES/NO choice, latched until the engine applies it on every GDU.
  bool courseReversalAnswerFlyIt = false;
  // Deferred decision. When the prompt is raised at transition-select time (the
  // approach is not yet committed to the flight plan, so courseReversalLegIndex
  // is -1), the pilot's YES/NO answer is latched here and applied when the
  // approach is finally loaded/activated. courseReversalDecisionFix records the
  // transition it applies to so re-selecting a different transition discards it.
  bool courseReversalDecisionMade = false;
  bool courseReversalDecisionFlyIt = false;
  std::string courseReversalDecisionFix;
};

}  // namespace avionics
