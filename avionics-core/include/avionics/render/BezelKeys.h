#pragma once

#include "avionics/Renderer.h"

namespace avionics {

// Hardware controls that live on the right bezel of a G1000 GDU, emulated
// on-screen so the unit is usable without a physical bezel. Bottom-to-top the
// cluster matches Figure 1-2: the dual concentric FMS knob, the 2×3 molded key
// grid (D→/MENU, FPL/PROC, CLR/ENT), then the RANGE joystick.
//
// The RANGE joystick (Pilot's Guide / X-Plane G1000 commands) rotates to zoom
// the map (RangeUp/RangeDown), is pushed in the center to activate or cancel
// map panning (PanPush), and is moved to pan the Map Pointer (Pan* = X-Plane
// g1000n*_pan_up/down/left/right). The FMS knob is a separate control: the four
// Fms* turn events are its clicks (large knob selects a page group / moves the
// cursor, small knob selects a page / enters characters) and FmsPush turns the
// selection cursor on and off. Key order is top-to-bottom, which the hit-test
// relies on.
enum class BezelKey {
  DirectTo,
  Menu,
  Fpl,
  Proc,
  Clr,
  Ent,
  // RANGE joystick.
  RangeUp,    // rotate clockwise: zoom out (range up)
  RangeDown,  // rotate counter-clockwise: zoom in (range down)
  PanPush,    // press the joystick: activate / cancel the Map Pointer
  PanUp,      // move the joystick: pan the Map Pointer
  PanDown,
  PanLeft,
  PanRight,
  // Dual concentric FMS knob.
  FmsOuterCcw,  // large FMS knob, one click counter-clockwise
  FmsOuterCw,   // large FMS knob, one click clockwise
  FmsInnerCcw,  // small FMS knob, one click counter-clockwise
  FmsInnerCw,   // small FMS knob, one click clockwise
  FmsPush,      // press the FMS knob (cursor on/off, cancel entry)
  Count,
};
inline constexpr int kBezelKeyCount = static_cast<int>(BezelKey::Count);

// The molded key caps drawn as a vertical column (everything above the RANGE
// joystick widget).
inline constexpr int kBezelButtonCount = static_cast<int>(BezelKey::RangeUp);

// The RANGE joystick widget spans [kRangeJoyFirst, kFmsKnobFirst); the FMS knob
// widget spans [kFmsKnobFirst, Count).
inline constexpr int kRangeJoyFirst = static_cast<int>(BezelKey::RangeUp);
inline constexpr int kFmsKnobFirst = static_cast<int>(BezelKey::FmsOuterCcw);

// True for the bezel controls operated by rotation: the FMS knob's inner/outer
// rings and the RANGE joystick's outer zoom ring. A shell can use this to show
// a rotate-style hover cursor over those regions (the center push caps and the
// pan directions are presses, not rotations).
inline bool isRotatableBezelKey(BezelKey key) {
  switch (key) {
    case BezelKey::RangeUp:
    case BezelKey::RangeDown:
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsOuterCw:
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsInnerCw:
      return true;
    default:
      return false;
  }
}

// How long the CLR key must be held before it acts as "CLR (DFLT MAP)" and
// displays the MFD Navigation Map Page (G1000 NXi Pilot's Guide: "press and
// hold CLR (MFD only)"). The guide gives no duration; this matches the
// roughly-one-second feel of the real unit.
inline constexpr double kClrDefaultMapHoldSeconds = 1.0;

// Draws and hit-tests the bezel key column inside an explicit rectangle (the
// window's bezel strip beside the screen). The keys fill the rect top-to-bottom.
// Render and hit-test share one layout so they always agree.
class BezelKeyPanel {
 public:
  // Draws the bezel face and keys filling [x, x+w] x [y, y+h]. pressLevels
  // points at kBezelKeyCount floats (0..1) for the press-flash; displayH scales
  // the label fonts to the physical display height.
  static void render(Renderer& r, float x, float y, float w, float h,
                     float displayH, const float* pressLevels);

  // Returns the key under the pointer, or BezelKey::Count if outside any key.
  static BezelKey hitTest(float xPx, float yPx, float x, float y, float w,
                          float h);
};

}  // namespace avionics
