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

  // The remaining hardware controls on the GDU bezel (Figure 1-2 "PFD/MFD
  // Controls"). The right bezel carries the COM controls and the CRS/BARO knob
  // (above the RANGE joystick); the left bezel carries the NAV controls and the
  // HDG knob. These are added after the FMS keys so the historical index ranges
  // above (grid / RANGE joystick / FMS knob) stay fixed.

  // COM knob (right bezel): large knob tunes whole MHz, small knob tunes the
  // channel; pressing toggles COM1/COM2.
  ComOuterCcw,
  ComOuterCw,
  ComInnerCcw,
  ComInnerCw,
  ComPush,
  ComTransfer,  // COM frequency transfer key (flip-flop; EMERG on hold)

  // COM VOL/SQ knob (right bezel): turn sets COM audio volume. The press would
  // toggle automatic squelch on the real unit, but X-Plane has no squelch
  // dataref, so the press is inert beyond the press-flash.
  ComVolCcw,
  ComVolCw,
  ComVolPush,

  // CRS/BARO knob (right bezel): large knob sets the altimeter barometric
  // setting, small knob sets the selected course; pressing syncs the course.
  BaroCcw,
  BaroCw,
  CrsCcw,
  CrsCw,
  CrsPush,

  // NAV knob (left bezel): large knob tunes whole MHz, small knob tunes the
  // channel; pressing toggles NAV1/NAV2.
  NavOuterCcw,
  NavOuterCw,
  NavInnerCcw,
  NavInnerCw,
  NavPush,
  NavTransfer,  // NAV frequency transfer key (flip-flop)

  // NAV VOL/ID knob (left bezel): turn sets NAV audio volume, press toggles the
  // selected NAV's Morse identifier audio (audio_selection_nav*, "ID").
  NavVolCcw,
  NavVolCw,
  NavVolPush,

  // HDG knob (left bezel): turn moves the selected-heading bug, press syncs the
  // bug to the current heading.
  HdgCcw,
  HdgCw,
  HdgPush,

  // Red DISPLAY BACKUP key on the GMA audio panel (drawn on the lower left
  // bezel in the standalone shell). Toggles manual reversionary mode.
  DisplayBackup,

  Count,
};
inline constexpr int kBezelKeyCount = static_cast<int>(BezelKey::Count);

// The molded key caps drawn as a vertical column (everything above the RANGE
// joystick widget).
inline constexpr int kBezelButtonCount = static_cast<int>(BezelKey::RangeUp);

// The RANGE joystick widget spans [kRangeJoyFirst, kFmsKnobFirst); the FMS knob
// widget spans [kFmsKnobFirst, kRightExtraFirst).
inline constexpr int kRangeJoyFirst = static_cast<int>(BezelKey::RangeUp);
inline constexpr int kFmsKnobFirst = static_cast<int>(BezelKey::FmsOuterCcw);

// First of the COM / CRS-BARO / NAV / HDG controls added for the full bezel.
inline constexpr int kRightExtraFirst = static_cast<int>(BezelKey::ComOuterCcw);

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
    case BezelKey::ComOuterCcw:
    case BezelKey::ComOuterCw:
    case BezelKey::ComInnerCcw:
    case BezelKey::ComInnerCw:
    case BezelKey::ComVolCcw:
    case BezelKey::ComVolCw:
    case BezelKey::BaroCcw:
    case BezelKey::BaroCw:
    case BezelKey::CrsCcw:
    case BezelKey::CrsCw:
    case BezelKey::NavOuterCcw:
    case BezelKey::NavOuterCw:
    case BezelKey::NavInnerCcw:
    case BezelKey::NavInnerCw:
    case BezelKey::NavVolCcw:
    case BezelKey::NavVolCw:
    case BezelKey::HdgCcw:
    case BezelKey::HdgCw:
      return true;
    default:
      return false;
  }
}

// For a rotatable bezel key (one side of a knob ring), returns the same knob's
// clockwise variant when `clockwise` is true, else its counter-clockwise
// variant. Lets a shell spin the hovered knob with the scroll wheel regardless
// of which side of the ring the pointer rests on. Returns BezelKey::Count for
// non-rotatable keys. (The RANGE joystick keeps its own scroll convention, so
// it is intentionally not handled here.)
inline BezelKey rotatedBezelKey(BezelKey key, bool clockwise) {
  switch (key) {
    case BezelKey::FmsOuterCcw:
    case BezelKey::FmsOuterCw:
      return clockwise ? BezelKey::FmsOuterCw : BezelKey::FmsOuterCcw;
    case BezelKey::FmsInnerCcw:
    case BezelKey::FmsInnerCw:
      return clockwise ? BezelKey::FmsInnerCw : BezelKey::FmsInnerCcw;
    case BezelKey::ComOuterCcw:
    case BezelKey::ComOuterCw:
      return clockwise ? BezelKey::ComOuterCw : BezelKey::ComOuterCcw;
    case BezelKey::ComInnerCcw:
    case BezelKey::ComInnerCw:
      return clockwise ? BezelKey::ComInnerCw : BezelKey::ComInnerCcw;
    case BezelKey::ComVolCcw:
    case BezelKey::ComVolCw:
      return clockwise ? BezelKey::ComVolCw : BezelKey::ComVolCcw;
    case BezelKey::BaroCcw:
    case BezelKey::BaroCw:
      return clockwise ? BezelKey::BaroCw : BezelKey::BaroCcw;
    case BezelKey::CrsCcw:
    case BezelKey::CrsCw:
      return clockwise ? BezelKey::CrsCw : BezelKey::CrsCcw;
    case BezelKey::NavOuterCcw:
    case BezelKey::NavOuterCw:
      return clockwise ? BezelKey::NavOuterCw : BezelKey::NavOuterCcw;
    case BezelKey::NavInnerCcw:
    case BezelKey::NavInnerCw:
      return clockwise ? BezelKey::NavInnerCw : BezelKey::NavInnerCcw;
    case BezelKey::NavVolCcw:
    case BezelKey::NavVolCw:
      return clockwise ? BezelKey::NavVolCw : BezelKey::NavVolCcw;
    case BezelKey::HdgCcw:
    case BezelKey::HdgCw:
      return clockwise ? BezelKey::HdgCw : BezelKey::HdgCcw;
    default:
      return BezelKey::Count;
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
  // Draws the right bezel strip (COM VOL/SQ + COM transfer, the COM knob, the
  // CRS/BARO knob, the RANGE joystick, the 2x3 key grid and the FMS knob)
  // filling [x, x+w] x [y, y+h]. pressLevels points at kBezelKeyCount floats
  // (0..1) for the press-flash; displayH scales the label fonts to the physical
  // display height.
  static void render(Renderer& r, float x, float y, float w, float h,
                     float displayH, const float* pressLevels);

  // Returns the key under the pointer in the right strip, or BezelKey::Count if
  // outside any control.
  static BezelKey hitTest(float xPx, float yPx, float x, float y, float w,
                          float h);

  // Draws the left bezel strip (NAV VOL/ID + NAV transfer, the NAV knob and the
  // HDG knob). Same conventions as render().
  static void renderLeft(Renderer& r, float x, float y, float w, float h,
                         float displayH, const float* pressLevels);

  // Returns the key under the pointer in the left strip, or BezelKey::Count if
  // outside any control.
  static BezelKey hitTestLeft(float xPx, float yPx, float x, float y, float w,
                              float h);
};

}  // namespace avionics
