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

// Active radio frequency / GPS source annunciation (G1000 uses a bright green).
inline constexpr Color kActiveGreen{0.0f, 0.95f, 0.0f, 1.0f};

// Top NAV/COM bar and bottom info-panel boxes use a dark blue-grey vertical
// gradient (rgb(4,4,12) -> rgb(24,28,43)), per the NXi NavComBox. The softkey
// bar is near-black.
inline constexpr Color kPanelBackground{0.016f, 0.016f, 0.047f, 1.0f};   // rgb(4,4,12)
inline constexpr Color kPanelBackgroundBottom{0.094f, 0.110f, 0.169f, 1.0f};  // rgb(24,28,43)
inline constexpr Color kInfoBoxTop{0.094f, 0.094f, 0.094f, 1.0f};  // rgb(24,24,24)
inline constexpr Color kSoftkeyBackground{0.0f, 0.0f, 0.0f, 1.0f};

// Panel borders (rgb(133,133,133)) and groove separators; muted grey labels.
inline constexpr Color kPanelBorder{0.522f, 0.522f, 0.522f, 1.0f};  // rgb(133,133,133)
inline constexpr Color kPanelSeparator{0.35f, 0.35f, 0.35f, 1.0f};
inline constexpr Color kLabelText{0.78f, 0.78f, 0.78f, 1.0f};

// Airspeed-tape color ranges, exact NXi values: normal range green (#008000),
// caution amber (yellow), never-exceed / low-speed red, white flap range.
inline constexpr Color kBandGreen{0.0f, 0.502f, 0.0f, 1.0f};  // #008000
inline constexpr Color kBandYellow{1.0f, 1.0f, 0.0f, 1.0f};
inline constexpr Color kBandRed{1.0f, 0.0f, 0.0f, 1.0f};
}  // namespace colors

}  // namespace avionics
