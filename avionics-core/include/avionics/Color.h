#pragma once

namespace avionics {

struct Color {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  float a = 1.0f;
};

namespace colors {
inline constexpr Color kBlack{0.0f, 0.0f, 0.0f, 1.0f};
inline constexpr Color kWhite{1.0f, 1.0f, 1.0f, 1.0f};

// Attitude indicator sky/ground. These are the exact values from the Working
// Title G1000 NXi ArtificialHorizon.css: the sky is a deep saturated blue
// (#0033e6) blending to a slightly lighter blue (#284be4) only in the last few
// percent above the horizon, the ground is a flat dark earth brown (#3a2400),
// and the horizon line is white.
inline constexpr Color kSkyTop{0.0f, 0.2f, 0.902f, 1.0f};          // #0033e6
inline constexpr Color kSkyHorizon{0.157f, 0.294f, 0.894f, 1.0f};  // #284be4
inline constexpr Color kGroundHorizon{0.227f, 0.141f, 0.0f, 1.0f};  // #3a2400
inline constexpr Color kGroundBottom{0.227f, 0.141f, 0.0f, 1.0f};   // #3a2400
inline constexpr Color kHorizon{1.0f, 1.0f, 1.0f, 1.0f};           // #ffffff

// Moving-tape background: the Working Title NXi tape uses a vertical gradient
// that is translucent black (alpha 0.5) at the top and bottom edges and fully
// clear through the middle, so the scale fades out toward the center where the
// readout box sits. Readout boxes are a near-black vertical gradient; the VSI
// value box is navy.
inline constexpr Color kTapeEdge{0.0f, 0.0f, 0.0f, 0.5f};
inline constexpr Color kTapeClear{0.0f, 0.0f, 0.0f, 0.0f};
inline constexpr Color kAirspeedBox{0.0f, 0.0f, 0.0f, 0.55f};
inline constexpr Color kReadoutBox{0.0f, 0.0f, 0.0f, 1.0f};
// IAS/altitude readout box fill gradient (rgba(0,0,0,1) -> rgb(30,30,30) ->
// rgba(0,0,0,1)) from the NXi digit-box background.
inline constexpr Color kReadoutBoxMid{0.118f, 0.118f, 0.118f, 1.0f};  // rgb(30,30,30)
inline constexpr Color kVsiBox{0.0f, 0.075f, 0.255f, 1.0f};  // rgb(0,19,65)
inline constexpr Color kTapeTopBorder{0.392f, 0.392f, 0.392f, 1.0f};  // #646464
inline constexpr Color kTapeBottomBorder{0.173f, 0.173f, 0.173f, 1.0f};  // #2c2c2c
inline constexpr Color kMagenta{1.0f, 0.0f, 1.0f, 1.0f};
inline constexpr Color kGreen{0.0f, 1.0f, 0.0f, 1.0f};

// Aircraft reference symbol: two-tone yellow (bright top #ffff00, shaded bottom
// rgb(152,140,0)) with a black outline, per the Working Title G1000 NXi.
inline constexpr Color kSymbolYellow{1.0f, 1.0f, 0.0f, 1.0f};
inline constexpr Color kSymbolYellowDark{0.596f, 0.549f, 0.0f, 1.0f};

// Outline drawn behind yellow/white symbology for contrast over sky/ground.
inline constexpr Color kSymbolOutline{0.0f, 0.0f, 0.0f, 0.9f};

// Darkened band behind the roll scale so the white bank ticks read over sky.
inline constexpr Color kRollArcBand{0.0f, 0.0f, 0.0f, 0.30f};

// Translucent dark backing inside the HSI compass rose so the white ticks and
// numbers read clearly while the sky/ground still shows through dimmed. Matches
// the Working Title G1000 NXi rose fill (rgba(0,0,0,.3)).
inline constexpr Color kRoseBackground{0.0f, 0.0f, 0.0f, 0.30f};

// Thin border line on the inner edge of the moving tapes.
inline constexpr Color kTapeBorder{0.85f, 0.85f, 0.85f, 1.0f};

// Cyan: selected/reference values (selected altitude, baro, heading bug).
inline constexpr Color kCyan{0.0f, 0.85f, 1.0f, 1.0f};

// Garmin corporate-logo blue, used for the triangle trademark on the power-up
// logo splash. The real wordmark's triangle is a gradient (#09bcef -> #00467f);
// this is the brighter end (#09bcef), which reads as the recognizable Garmin
// blue against the black startup screen.
inline constexpr Color kGarminLogoBlue{0.035f, 0.737f, 0.937f, 1.0f};  // #09bcef

// Active radio frequency / GPS source annunciation (G1000 uses a bright green).
inline constexpr Color kActiveGreen{0.0f, 0.95f, 0.0f, 1.0f};

// Airspace boundaries on the moving map. Exact values from the Working Title /
// Garmin SDK MapAirspaceRendering: Class B/D and restricted-type airspace draw
// blue (#3080ff, Class D dashed, restricted/prohibited/warning combed), while
// Class C and MOA/Alert draw maroon (#4a0045, MOA/Alert combed).
inline constexpr Color kAirspaceBlue{0.188f, 0.502f, 1.0f, 1.0f};    // #3080ff
inline constexpr Color kAirspaceMaroon{0.290f, 0.0f, 0.271f, 1.0f};  // #4a0045

// Navigation-map point-feature symbology, matching the Garmin G1000/G3000 NXi
// map icon assets (Working Title fspackages, Garmin/Map): towered airports are
// a muted blue, non-towered/heliport/private airports a muted magenta, VORs and
// intersections a muted cyan, and NDBs a muted magenta. Every symbol carries a
// dark-gray outline for contrast over terrain.
inline constexpr Color kAirportTowered{0.251f, 0.439f, 0.690f, 1.0f};     // #4070b0
inline constexpr Color kAirportNonTowered{0.565f, 0.251f, 0.502f, 1.0f};  // #904080
inline constexpr Color kNavaidCyan{0.502f, 0.816f, 0.816f, 1.0f};         // #80d0d0
inline constexpr Color kNdbMagenta{0.627f, 0.188f, 0.502f, 1.0f};         // #a03080
inline constexpr Color kMapSymbolOutline{0.251f, 0.251f, 0.251f, 1.0f};   // #404040

// Top NAV/COM bar and bottom info-panel boxes use a dark blue-grey vertical
// gradient (rgb(4,4,12) -> rgb(24,28,43)), per the NXi NavComBox. The softkey
// bar is near-black.
inline constexpr Color kPanelBackground{0.016f, 0.016f, 0.047f, 1.0f};   // rgb(4,4,12)
inline constexpr Color kPanelBackgroundBottom{0.094f, 0.110f, 0.169f, 1.0f};  // rgb(24,28,43)
inline constexpr Color kInfoBoxTop{0.094f, 0.094f, 0.094f, 1.0f};  // rgb(24,24,24)
inline constexpr Color kSoftkeyBackground{0.0f, 0.0f, 0.0f, 1.0f};
// A selected softkey label changes to black text on a gray background and
// stays that way until turned off (G1000 Pilot's Guide for the Diamond DA40,
// "Softkey Function").
inline constexpr Color kSoftkeySelected{0.62f, 0.62f, 0.62f, 1.0f};

// Panel borders (rgb(133,133,133)) and groove separators; muted grey labels.
inline constexpr Color kPanelBorder{0.522f, 0.522f, 0.522f, 1.0f};  // rgb(133,133,133)
inline constexpr Color kPanelSeparator{0.35f, 0.35f, 0.35f, 1.0f};
inline constexpr Color kLabelText{0.78f, 0.78f, 0.78f, 1.0f};

// MFD page chrome, exact values from the Working Title G1000 NXi CSS: the
// gray data panel behind the group boxes (.mfd-dark-background / WT
// --background-gray #323232), the group-box border (1px solid rgb(120,120,120)),
// the muted field-label gray (--title-gray #b7b7b7), and whitesmoke for
// computed values (editable values are cyan, GPS-derived magenta).
inline constexpr Color kMfdPanelGray{0.196f, 0.196f, 0.196f, 1.0f};   // rgb(50,50,50)
inline constexpr Color kGroupBoxBorder{0.471f, 0.471f, 0.471f, 1.0f};  // rgb(120,120,120)
inline constexpr Color kTitleGray{0.718f, 0.718f, 0.718f, 1.0f};       // #b7b7b7
inline constexpr Color kWhitesmoke{0.961f, 0.961f, 0.961f, 1.0f};      // #f5f5f5
// Faint steel-blue tint at the top edge of a group box body
// (rgba(152,159,174,0.1) fading to black over the first 10px).
inline constexpr Color kGroupBoxSheen{0.596f, 0.624f, 0.682f, 0.10f};
inline constexpr Color kGroupBoxSheenEnd{0.0f, 0.0f, 0.0f, 0.0f};

// Airspeed-tape color ranges, exact NXi values: normal range green (#008000),
// caution amber (yellow), never-exceed / low-speed red, white flap range.
inline constexpr Color kBandGreen{0.0f, 0.502f, 0.0f, 1.0f};  // #008000
inline constexpr Color kBandYellow{1.0f, 1.0f, 0.0f, 1.0f};
inline constexpr Color kBandRed{1.0f, 0.0f, 0.0f, 1.0f};
}  // namespace colors

}  // namespace avionics
