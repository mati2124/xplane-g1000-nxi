#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "avionics/Checklist.h"
#include "avionics/MapRange.h"
#include "avionics/render/BezelKeys.h"
#include "avionics/render/MapView.h"

namespace avionics {

// The MFD page groups (G1000 Pilot's Guide for Cessna Nav III, Section 1.4).
// MAP/WPT/AUX/NRST and Checklist are selected from the bottom softkey bar
// (standing in for the large FMS knob); the FPL group is entered with the FPL
// bezel key, as on the real unit. Each checklist in the loaded file is a page
// within the Checklist group (stepped like any other group's pages).
enum class MfdPageGroup { Map, Waypoint, Aux, Nearest, FlightPlan, Checklist };

// Pages within each group, in real on-unit order. Optional equipment pages
// (Stormscope, weather data link, XM, telephone, video) and pages that need
// data this suite has no source for (user waypoints, ARTCC/FSS frequencies,
// System Setup editing) are omitted.
enum class MfdPage {
  // MAP group.
  NavigationMap,
  // WPT group.
  AirportInformation,
  IntersectionInformation,
  NdbInformation,
  VorInformation,
  // AUX group.
  TripPlanning,
  GpsStatus,
  SystemStatus,
  // NRST group.
  NearestAirports,
  NearestIntersections,
  NearestNdb,
  NearestVor,
  NearestAirspaces,
  // FPL group.
  ActiveFlightPlan,
};

// Owns the interactive state of the MFD's softkey bar: the selected page group
// and the moving-map range. It mirrors SoftkeyController's shape (label /
// pressLevel / keyActive read by the renderer, pressKey / update driven by
// the engine) so the shared softkey-bar drawing style applies to both displays.
//
// The MFD keeps its own range independent of the PFD inset map, so zooming one
// display does not affect the other.
class MfdController {
 public:
  // The MFD reuses the 12-cell softkey bar layout shared with the PFD.
  static constexpr int kSoftkeyCount = 12;

  // Press-flash decay, in seconds (matches the PFD softkey feel).
  static constexpr float kPressFlashSeconds = 0.18f;

  MfdController();

  // Advance the key-press flash animations.
  void update(double dtSeconds);

  // Apply a press of physical softkey `key` (0..kSoftkeyCount-1, the hardware
  // keys below the display; the on-screen bar is labels only, like the real
  // unit). Returns true if the key currently has a function.
  bool pressKey(int key);

  MfdPageGroup pageGroup() const { return pageGroup_; }

  // Number of pages in a group and the index of the page currently selected
  // within the active group (the FMS rocker / repeated group-softkey presses
  // step it, like the small FMS knob).
  static int pageCount(MfdPageGroup group);
  int pageIndex() const;
  // The specific page on screen, resolved from the group + page index.
  MfdPage page() const;

  // Current map range, in NM (a discrete G1000-style range ladder).
  float rangeNm() const;

  // Whether the topographic terrain background is enabled (TERR softkey).
  bool showTerrain() const { return showTerrain_; }

  // MAP page orientation (TRK softkey toggles north-up vs track-up).
  MapOrientation mapOrientation() const { return mapOrientation_; }

  // ---- checklists (Checklist page group) ----
  // Cache the latest loaded checklists each frame so item navigation tracks the
  // file (the data is owned by the DataSource). Resizes/clamps the interactive
  // checked state and selection when the file's shape changes.
  void syncChecklist(const ChecklistData& data);
  // Flat index of the displayed checklist, and the highlighted item within it.
  // The cursor ranges [0, itemCount]; itemCount selects the "go to next
  // checklist" prompt below the last item.
  int checklistIndex() const { return checklistIndex_; }
  int checklistCursor() const { return cursorItem_; }
  int checklistCount() const;
  bool checklistItemChecked(int checklistIndex, int itemIndex) const;

  // ---- read by the renderer ----
  const std::string& label(int i) const { return labels_[i]; }
  float pressLevel(int i) const { return press_[i]; }
  // All kSoftkeyCount press levels, for the shell's physical softkey row.
  const float* pressLevels() const { return press_.data(); }
  // True while the cell's page group is the selected one (radio highlight).
  bool keyActive(int i) const;

  // ---- window bezel key column (drawn by the standalone shell) ----
  // Apply a press of a hardware bezel key: flashes the key and, for the range
  // rocker, steps the MFD map range.
  void pressBezelKey(BezelKey key);
  const float* bezelPressLevels() const { return bezelPress_.data(); }

 private:
  // Step the active group's page index by +/-1, wrapping (small FMS knob).
  void stepPage(int direction);
  void selectGroup(MfdPageGroup group);
  // Step the displayed checklist by +/-1, wrapping, and reset the item cursor.
  void stepChecklist(int direction);
  // Number of items in the checklist at the given flat index (0 if none).
  int checklistItemCount(int checklistIndex) const;
  // ENT on the Checklist page: check the cursor item and auto-advance, or (on
  // the "go to next checklist?" prompt) advance to the next checklist.
  void checklistEnter();
  // CLR on the Checklist page: uncheck the cursor item.
  void checklistClear();

  MfdPageGroup pageGroup_ = MfdPageGroup::Map;
  // Per-group selected page, remembered across group switches like the real
  // unit (indexed by MfdPageGroup). Sized for every group so an out-of-range
  // index is never possible; the Checklist group uses checklistIndex_ instead.
  std::array<int, 6> pageIndex_{};
  // Group shown before the FPL key was pressed, restored when it is pressed
  // again (the FPL page is a toggle overlaid on normal page navigation).
  MfdPageGroup groupBeforeFpl_ = MfdPageGroup::Map;
  int rangeIndex_ = kMapRangeDefaultIndex;  // ladder index (defaults to 10 NM)
  bool showTerrain_ = true;  // topographic background on by default
  MapOrientation mapOrientation_ = MapOrientation::NorthUp;

  // Checklist page group state. checklist_ is the latest data cached by
  // syncChecklist (owned by the DataSource, not this controller). The checked
  // flags are kept here because they are interactive UI state, not file data.
  const ChecklistData* checklist_ = nullptr;
  int checklistIndex_ = 0;
  int cursorItem_ = 0;
  std::vector<std::vector<std::uint8_t>> checked_;

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};
  std::array<float, kBezelKeyCount> bezelPress_{};
};

}  // namespace avionics
