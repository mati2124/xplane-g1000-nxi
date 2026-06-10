#pragma once

#include <array>
#include <string>

#include "avionics/MapRange.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {

// The four top-level MFD page groups, selected from the bottom softkey bar and
// cycled with the FMS knob on a real G1000. Only MAP renders live content for
// now; the others draw a titled placeholder until their pages are built.
enum class MfdPageGroup { Map, Waypoint, Aux, Nearest };

// Owns the interactive state of the MFD's softkey bar: the selected page group
// and the moving-map range. It mirrors SoftkeyController's shape (label /
// pressLevel / keyActive read by the renderer, pointerDown / update driven by
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

  // Forward a pointer press in display-pixel coordinates. Returns true if it
  // landed on an interactive softkey (and was therefore consumed).
  bool pointerDown(float xPx, float yPx, float widthPx, float heightPx);

  MfdPageGroup pageGroup() const { return pageGroup_; }

  // Current map range, in NM (a discrete G1000-style range ladder).
  float rangeNm() const;

  // Whether the topographic terrain background is enabled (TERR softkey).
  bool showTerrain() const { return showTerrain_; }

  // ---- read by the renderer ----
  const std::string& label(int i) const { return labels_[i]; }
  float pressLevel(int i) const { return press_[i]; }
  // True while the cell's page group is the selected one (radio highlight).
  bool keyActive(int i) const;

  // ---- window bezel key column (drawn by the standalone shell) ----
  // Apply a press of a hardware bezel key: flashes the key and, for the range
  // rocker, steps the MFD map range.
  void pressBezelKey(BezelKey key);
  const float* bezelPressLevels() const { return bezelPress_.data(); }

 private:
  static int hitTest(float xPx, float yPx, float widthPx, float heightPx);

  MfdPageGroup pageGroup_ = MfdPageGroup::Map;
  int rangeIndex_ = kMapRangeDefaultIndex;  // ladder index (defaults to 10 NM)
  bool showTerrain_ = true;                 // topographic background on by default

  std::array<std::string, kSoftkeyCount> labels_;
  std::array<float, kSoftkeyCount> press_{};
  std::array<float, kBezelKeyCount> bezelPress_{};
};

}  // namespace avionics
