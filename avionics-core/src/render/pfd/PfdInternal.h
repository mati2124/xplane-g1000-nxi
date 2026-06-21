#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"
#include "avionics/Renderer.h"
#include "avionics/SoftkeyController.h"

namespace avionics::pfd {

// The layout reproduces the Working Title G1000 NXi PFD, whose component CSS is
// authored against a fixed 1024x768 GDU canvas. computeLayout maps those exact
// pixel positions onto the actual display: sx/sy are the horizontal/vertical
// scale factors and s is the uniform scale used for radii, tick lengths, and
// other size quantities (sx == sy for the native 4:3 aspect).
constexpr float kWtCanvasWidth = 1024.0f;
constexpr float kWtCanvasHeightPx = 768.0f;

// HSI chrome geometry (#HSI in WT HSI.css, HSIMap.css, HSIMapCourseDeviation.css).
// Positions are WT canvas pixels; callers scale with Layout sx/sy.
namespace hsi {
constexpr float kOriginX = 277.0f;
constexpr float kOriginY = 387.0f;
constexpr float kMapContainerLeft = 7.0f;
constexpr float kCourseBandH = 23.0f;
constexpr float kCourseSrcLeft = 30.0f;
constexpr float kCourseSrcW = 53.0f;
constexpr float kCourseDevLeft = 85.0f;
constexpr float kCourseDevW = 183.0f;
constexpr float kCoursePhaseLeft = 270.0f;
constexpr float kCoursePhaseW = 84.0f;
constexpr float kCourseGap = 2.0f;
constexpr float kMapHeadingBoxLeft = 134.0f;
constexpr float kMapHeadingBoxTop = 27.0f;
constexpr float kMapHeadingBoxW = 84.0f;
constexpr float kMapHeadingBoxH = 34.0f;
constexpr float kRefBoxTop = 26.0f;
constexpr float kRefBoxHdgLeft = 6.0f;
constexpr float kRefBoxDtkLeft = 276.0f;
constexpr float kRefBoxW = 84.0f;
constexpr float kRefBoxH = 26.0f;
constexpr float kRefBoxRadius = 5.0f;
}  // namespace hsi

// Lower-right PFD popout shell (WT .popout-dialog on the 1024x768 GDU canvas).
// Every PFD pop-up (Direct-To, Alerts, Nearest, Flight Plan, Procedures,
// References, PFD Setup, Page Menu) shares this exact footprint.
constexpr float kWtPopoutWidthPx = 310.0f;
constexpr float kWtPopoutHeightPx = 220.0f;
constexpr float kWtPopoutBorderRadiusPx = 10.0f;
constexpr float kWtPopoutBorderPx = 3.0f;
// bottom: 26px above the softkey bar in WTG1000_PFD.css.
constexpr float kWtPopoutBottomMarginPx = 26.0f;
// Open popout is flush to the GDU right edge (WT .popout-dialog.open:
// right:-320px + translate3d(-320px,0,0) on a 310px-wide box).
constexpr float kWtPopoutRightMarginPx = 0.0f;
// PFD Page Menu list viewport inside the shell (WT .pfd-pagemenu-listcontainer).
constexpr float kWtPageMenuListHeightPx = 72.0f;
constexpr float kWtPageMenuRowHeightPx = 24.0f;
// PFD Procedures top-level menu (WT PFDProc): six fixed rows at 29 px pitch.
constexpr float kWtProcMenuRowHeightPx = 29.0f;
constexpr float kWtProcMenuListTopPadFrac = 0.25f;  // fraction of kInfoValue

inline void popoutPanelSize(float w, float h, float& outW, float& outH) {
  outW = w * (kWtPopoutWidthPx / kWtCanvasWidth);
  outH = h * (kWtPopoutHeightPx / kWtCanvasHeightPx);
}

struct Layout {
  float sx, sy, s;

  float topBarH, infoPanelH, infoPanelTop, bottomBarH;
  float attTop, attBottom;
  float attCx, attCy;
  float attRegionH;
  float attVisW;
  float rollRadius;

  float stripTop, stripH;
  float tapeBgH;  // tape gradient background height (>= stripH; ticks stay in stripH)
  float asiX, asiW, asiTop, asiH;
  float altX, altW, altTop, altH;
  float vsiX, vsiW, vsiTop, vsiH;

  // CAS annunciation window (G1000 Pilot's Guide): to the right of the VSI,
  // starting at the bottom of the altimeter instrument column.
  float casAnnunTop, casAnnunLeft, casAnnunW;

  float hsiCx, hsiCy, hsiRadius;
  // HSI Map layout: the compass rose is larger and sits lower than the standard
  // rose (WT NXi HSIMap), so its bottom runs off the screen behind the bottom
  // info panel. The course deviation / source / flight-phase band straddles the
  // top of this map (WT NXi HSIMapCourseDeviation) instead of the rose-mode
  // in-rose annunciation.
  float hsiMapCx, hsiMapCy, hsiMapRadius;

  // Vertical deviation indicator (glideslope/glidepath/VNAV) scale just left of
  // the altimeter, and the marker-beacon annunciation left of Selected Altitude.
  float vdiX, vdiW;
  float markerX, markerY, markerW, markerH;

  // PFD inset map (lower-left over the attitude window). The MFD MAP page will
  // reuse MapView with a full-screen rect instead of these layout constants.
  float insetMapX, insetMapY, insetMapW, insetMapH;
  // Display-backup mode always shows the inset in the bottom-right using
  // kReversionaryInsetMap* canvas coordinates (not the compressed PFD scale).
  bool reversionaryInsetMap = false;
};

Layout computeLayout(float w, float h);

// Pitch ladder, in Working Title 1024x768 pixel units (scaled at draw time).
// pxPerDeg is derived from the NXi SVT projection (FOV 65 deg, focal length
// 614.4/tan(32.5deg)); the non-SVT ladder uses half that pitch ratio, giving
// 8.417 px/deg. Ladder line half-widths: 10 deg -> 54 px, 5 deg -> 27 px,
// 2.5 deg -> 14 px in the 414 px-wide attitude window.
constexpr float kPitchPxPerDegWt = 8.417f;
constexpr float kPitch10HalfWt = 54.0f;
constexpr float kPitch5HalfWt = 27.0f;
constexpr float kPitch25HalfWt = 14.0f;

// Roll pointer, zero-reference triangle, and slip/skid bar — pixel measurements
// from Garmin trainer PFD Default.bmp @ 1024x768 (/Volumes/GARMIN/Screenshots/),
// attCy=278, rollRadius=193. Scale at draw time by radius / 193.
constexpr float kTrainerRollPointerApexInsetPx = 8.0f;
constexpr float kTrainerRollPointerHeightPx = 18.0f;
constexpr float kTrainerRollPointerHalfWidthPx = 9.0f;
constexpr float kTrainerRollZeroTriHeightPx = 16.0f;
constexpr float kTrainerRollZeroTriHalfWidthPx = 9.0f;
constexpr float kTrainerRollSlipGapPx = 6.0f;
constexpr float kTrainerRollSlipHeightPx = 6.0f;
constexpr float kTrainerRollSlipTopHalfPx = 12.0f;
constexpr float kTrainerRollSlipBotHalfPx = 15.0f;

// PFD inset map viewport (Garmin trainer PFD Inset Map.bmp @ 1024x768,
// /Volumes/GARMIN/Screenshots/). Square clip for the moving map in the
// lower-left; the bottom edge (y=710) sits just above the OAT row (y=720).
constexpr float kInsetMapLeftPx = 0.0f;
constexpr float kInsetMapTopPx = 482.0f;
constexpr float kInsetMapWidthPx = 242.0f;
constexpr float kInsetMapHeightPx = 229.0f;

// PFD reversionary inset map (Garmin trainer screenshot @ 1024x768, display
// backup with the EIS strip on the left). Bottom-right viewport, flush to the
// right edge; bottom chrome still at y=711 like the normal inset layout.
constexpr float kReversionaryInsetMapLeftPx = 718.0f;
constexpr float kReversionaryInsetMapTopPx = 490.0f;
constexpr float kReversionaryInsetMapWidthPx = 306.0f;   // 1024 - 718
constexpr float kReversionaryInsetMapHeightPx = 221.0f;  // 711 - 490

// HSI turn-rate tick lengths (same trainer screenshot, tick ring radius 153).
constexpr float kTrainerTurnRateStdTickPx = 15.0f;
constexpr float kTrainerTurnRateHalfTickPx = 8.0f;
constexpr float kTrainerHsiTickRingRadiusPx = 153.0f;

// Tape tick geometry as fractions of tape width. Airspeed matches WT NXi CSS
// (--airspeed-tick-minor/major-width: 12% / 24%). Altimeter tick lengths and
// label anchors match WT Altimeter.tsx SVG viewBox width 179 (minor 15, major
// 30, hundreds label anchor 133, "00" anchor 175).
constexpr float kTapeTickStrokePx = 2.0f;  // WT non-scaling-stroke at 1024 ref
constexpr float kAsiTapeMinorTickFraction = 0.12f;   // ~10 px @ 87 px tape
constexpr float kAsiTapeMajorTickFraction = 0.24f;   // ~21 px @ 87 px tape
constexpr float kAltTapeMinorTickFraction = 15.0f / 179.0f;   // ~9 px @ 108 px
constexpr float kAltTapeMajorTickFraction = 30.0f / 179.0f;   // ~18 px @ 108 px
constexpr float kAltTapeLabelHeadAnchorFraction = 133.0f / 179.0f;
constexpr float kAltTapeLabelZerosAnchorFraction = 175.0f / 179.0f;
// WT vsi-container inline svg tick x extents (48 px tape width).
constexpr float kVsiTickInnerInsetFraction = 2.0f / 48.0f;
constexpr float kVsiTickMinorOuterFraction = 10.0f / 48.0f;
constexpr float kVsiTickMajorOuterFraction = 16.0f / 48.0f;
// The NXi moving-tape windows have a 10 px rounded outer corner (WT NXi tape
// border-radius). The airspeed tape, the altimeter selected-altitude box, and
// the IAS/altitude readout boxes all derive their rounding from this so the two
// instrument columns share one corner style.
constexpr float kTapeCornerRadiusWt = 10.0f;
// GS/TAS (airspeed) and BARO (altimeter) bottom boxes share one height and sit
// flush with the bottom of the instrument column (WT NXi: bottom: 0 on the
// airspeed/altimeter containers). The scroll strip still ends at y=443; the
// extra column height is the bottom readout band down to y=482 (inset map top).
constexpr float kTapeBottomBoxHeightWt = 28.0f;
// Trainer PFD Inset Map.bmp: asi/alt y=82, GS/TAS and BARO box tops y=454,
// box bottoms y=481-482 (inset map chrome border at y=482).
constexpr float kAsiInstrumentHeightPx = 400.0f;
constexpr float kAltInstrumentHeightPx = 400.0f;
// Tape scroll viewport (trainer): y=113 h=330. The translucent tape background
// continues below the scroll clip to the GS/TAS / BARO box tops at y=454.
constexpr float kTapeScrollTopPx = 113.0f;
constexpr float kTapeScrollHeightPx = 330.0f;
constexpr float kTapeBottomBoxTopPx =
    82.0f + kAsiInstrumentHeightPx - kTapeBottomBoxHeightWt;  // 454
constexpr float kTapeBackgroundHeightPx =
    kTapeBottomBoxTopPx - kTapeScrollTopPx;  // 341
// NAV/COM bar height (PFD Default trainer screenshot, 1024x768).
constexpr float kTopBarHeightPx = 56.0f;
// Vertical offset for the decoded COM station-ID panel below the COM box.
constexpr float kComDecodePanelUpPx = 2.0f;
// On-screen softkey label row (PFD Default trainer screenshot, WT SoftKeyBar.css:
// top 734px, height 34px on the 768 canvas).
constexpr float kSoftkeyBarHeightPx = 34.0f;
constexpr float kSoftkeyBarTopPx =
    kWtCanvasHeightPx - kSoftkeyBarHeightPx;  // 734
// Bottom info panel row centers (WT BottomInfoPanel.css / Transponder.css).
constexpr float kInfoTopRowCenterPx = 14.0f;    // TMR row in the time box
constexpr float kInfoBottomRowCenterPx = 41.0f; // OAT / XPDR / UTC baseline
// OAT row when the inset map is showing (trainer PFD Inset Map.bmp): the temp
// box only occupies the strip below the square map viewport; OAT centers ~14px
// below the map bottom (y=725 vs viewport bottom y=711 on the 768 canvas).
constexpr float kInsetMapOatCyBelowViewportPx = 14.0f;
// IAS / altitude pointer boxes (PFD Default trainer screenshot, 1024x768).
constexpr float kAsiReadoutTopPx = 251.0f;
constexpr float kAltReadoutTopPx = 248.0f;
constexpr float kAsiReadoutHeightPx = 59.0f;
constexpr float kAltReadoutHeightPx = 62.0f;
// Body insets from the tape edges (screenshot-derived).
constexpr float kAsiReadoutLeftInsetPx = 5.0f;
constexpr float kAsiReadoutBodyRightInsetPx = 14.0f;
constexpr float kAltReadoutBodyRightInsetPx = 5.0f;
constexpr float kAltReadoutBodyLeftPx = 10.0f;
constexpr float kAltReadoutCaretDepthPx = 12.0f;
constexpr float kAltReadoutCaretHalfHeightPx = 6.0f;
// Stepped readout geometry: the leading-digit body height as a fraction of the
// pointer box; it is vertically centered so the drum column steps above/below.
constexpr float kAsiReadoutLeadHeightFraction = 0.707f;
constexpr float kAltReadoutLeadHeightFraction = 0.66f;
constexpr float kReadoutCornerRadiusPx = 2.5f;
constexpr float kReadoutDrumCornerRadiusPx = 2.0f;
constexpr float kReadoutOutlineWidthPx = 1.0f;
constexpr float kAsiReadoutDrumWidthPx = 21.0f;
constexpr float kAltReadoutDrumWidthPx = 32.0f;
constexpr float kAsiReadoutCaretDepthPx = 11.0f;
constexpr float kAsiReadoutCaretHalfHeightPx = 8.0f;
// Horizontal inset from the drum window's right edge to the rolling digits.
constexpr float kReadoutDrumPadFraction = 0.04f;
// fillText's NVG_ALIGN_MIDDLE centers text on the font's ascender/descender
// midline, which reserves descender space that digits/caps don't use, so a
// centered all-caps/numeric glyph rides ~0.10 em high. Add this fraction of the
// font size to a vertically-centered baseline to center the glyph INK in a box.
constexpr float kCapInkCenterNudge = 0.10f;
// Fine-tune leading-digit ink centering in the stepped band (+y = down).
constexpr float kAsiReadoutLeadInkNudge = 0.12f;
constexpr float kAltReadoutLeadInkNudge = 0.07f;
// WT altitude-scroller-background: each static digit column is 17 px wide with
// 18 px pitch; the hundreds column ends 1 px before the tens drum (70 px).
constexpr float kAltReadoutDigitPitchPx = 18.0f;
constexpr float kAltReadoutDigitColumnWidthPx = 17.0f;
constexpr float kAltReadoutLeadDrumGapPx = 1.0f;
// Per-digit dial columns (NXi DigitScroller / airspeed-ias-box-digit-bg).
constexpr int kAsiReadoutDigitColumnCount = 3;
constexpr float kReadoutDigitColumnGapPx = 1.0f;
constexpr float kReadoutDigitAreaInsetPx = 3.0f;
// NXi scroller mask: black 0–5%, fade to clear by 20%, clear 20–80%, fade to
// black by 95%, black 95–100% (airspeed-ias-box-scroller-mask-bg).
constexpr float kReadoutDrumMaskSolidFraction = 0.05f;
constexpr float kReadoutDrumMaskFadeFraction = 0.20f;

// Airspeed tape
constexpr float kAirspeedViewableKnots = 60.0f;
constexpr float kAirspeedMajorKnots = 10.0f;
constexpr float kAirspeedMinorKnots = 5.0f;
constexpr float kAirspeedMinKnots = 20.0f;
// Below the tape minimum the IAS pointer box shows one dash per digit column
// (WT DigitScroller NaN / off-scale behavior).
constexpr int kAsiReadoutOffScaleDashCount = 3;
// The color-coded speed-range strip occupies this fraction of the tape width at
// the inner edge; tape ticks are inset by it so they sit just outboard of the
// strip (white ticks beside the colored band, not under it) per the NXi.
constexpr float kAirspeedBandWidthFraction = 0.12f;

constexpr float kVsoKt = 33.0f;
constexpr float kVfeKt = 85.0f;
constexpr float kVs1Kt = 48.0f;
constexpr float kVnoKt = 129.0f;
constexpr float kVneKt = 163.0f;
// Mach replaces TAS in the bottom readout box at or above this value (TBM 900
// uses 0.30; suppressed in the normal C172 envelope).
constexpr float kMachDisplayThreshold = 0.40f;

// V-speed reference table, indexed by avionics::VspeedRef (the References
// window row order). bugLabel is the letter on the tape bug; windowLabel is
// the row label in the Timer/References window (Pilot's Guide Table 2-1).
struct VSpeedRef {
  const char* bugLabel;
  const char* windowLabel;
  float kt;
};
extern const VSpeedRef kVSpeedRefs[kVspeedRefCount];
extern const int kVSpeedRefCount;

// An autopilot selected altitude at/near 0 ft is treated as "unset": the
// selected-altitude box shows dashes and no bug is drawn on the tape.
constexpr float kAltSelectedEpsilonFt = 1.0f;
constexpr float kAsiSelectedEpsilonKt = 1.0f;
constexpr const char* kSelectedAltDashes = "-----";

// Altimeter tape. The viewable window spans +/-400 ft from the centered
// indicated altitude (800 ft total), matching the WT NXi altitude tape
// (calculateAbsoluteTapePosition divides the offset by 800).
constexpr float kAltitudeViewableFeet = 800.0f;
constexpr float kAltitudeMajorFeet = 100.0f;
constexpr float kAltitudeMinorFeet = 20.0f;
constexpr float kAltitudeMinFeet = -2000.0f;

// Garmin altitude readouts render the final two (tens) digits smaller than the
// leading hundreds digits. The tape labels and the selected-altitude box reuse
// this for those trailing digits (WT NXi: 38/44 px tape, 20/24 px box).
constexpr int kAltTrailingDigits = 2;
constexpr float kAltTapeTensScale = 0.86f;
constexpr float kAltSelectedTensScale = 0.83f;

// VSI. The scale is fixed to the panel (WT inline svg, 305 px tall): labels
// "2" at the top/bottom, "1" at +/-1000 fpm, half-ticks at +/-500. Only the
// pointer moves (WT transform 0.064 px/fpm, clamped +/-2250 fpm).
constexpr float kVsiMaxFpm = 4000.0f;
constexpr float kVsiPointerClampFpm = 2250.0f;
constexpr float kVsiPointerPxPerFpm = 0.064f;
constexpr float kVsiTapeSvgHeightPx = 305.0f;
// WT vsi-pointer / vsi-pointer-bug-background (viewBox 68 x 24 at 1024 ref).
constexpr float kVsiPointerWidthPx = 66.0f;
constexpr float kVsiPointerHeightPx = 22.0f;
// NXi VsPointerBug: no numeric readout below this magnitude.
constexpr float kVsiReadoutShowFpm = 100.0f;
// Trainer vs WT SVG text anchor (y=18 in a ~22 px body): nudge ink down 1 px.
constexpr float kVsiReadoutInkDownPx = 1.0f;
// WT vsi-tape-numbers: 20 px face reads large vs the trainer; 18 px + anchor
// at ~65% of the 48 px strip width matches the real unit more closely.
constexpr float kVsiLabelSizeWt = 18.0f;
constexpr float kVsiLabelAnchorFraction = 31.0f / 48.0f;

inline float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kWtCanvasHeightPx);
}

namespace wt {
constexpr float kTapeLabel = 21.0f;
constexpr float kReadout = 26.0f;
constexpr float kReadoutAlt = 30.0f;
constexpr float kSelectedAlt = 24.0f;
constexpr float kBaro = 20.0f;
constexpr float kVsi = 20.0f;
constexpr float kPitch = 20.0f;
constexpr float kRoseLetter = 20.0f;
constexpr float kRoseCardinal = 26.0f;
constexpr float kHeadingBox = 30.0f;
// Selected-heading (HDG) and selected-course (CRS/DTK) readouts flanking the
// heading box above the rose. Larger than the in-rose source annunciation, just
// under the rose's numeric labels, matching the G1000 NXi.
constexpr float kHsiBug = 18.0f;
constexpr float kHsiSource = 14.0f;
// The HDG/DTK reference boxes pair a small white label with a larger colored
// value (WT hdgcrs-container: 14 px "HDG"/"DTK" label, size20 cyan/magenta
// value).
constexpr float kHsiRefValue = 20.0f;
// PFD wind panel (upper-left of the HSI): Option 1/2 numeric values use the
// generic size18 face and the "KT" unit on Option 3 uses size10 (WT NXi
// WindOption*.css); the wind direction/speed text on Option 3 reuses kHsiSource.
constexpr float kWindValue = 18.0f;
constexpr float kWindUnit = 10.0f;
constexpr float kNavComFreq = 19.0f;
constexpr float kNavComLabel = 16.0f;
constexpr float kFmaActive = 23.0f;
constexpr float kFmaArmed = 19.0f;
constexpr float kFmaSmall = 13.0f;
constexpr float kInfoLabel = 16.0f;
constexpr float kInfoValue = 20.0f;
constexpr float kSoftkey = 17.0f;
}  // namespace wt

enum class NotchSide { None, Left, Right };

std::string formatInt(float value);
std::string formatHeading(float headingDeg);
const char* roseLabel(int deg);
std::string formatFreq(float mhz, int decimals);
std::string formatHms(int hour, int minute, int second);
std::string formatTimer(int totalSeconds);
std::string formatOat(float celsius);

void polarOffset(float angleDeg, float radius, float& x, float& y);

void drawReadoutBox(Renderer& r, float x, float y, float w, float h,
                    const std::string& text, float textSize, NotchSide notch,
                    const Color& boxColor = colors::kReadoutBox,
                    const Color& textColor = colors::kWhite);

// `topOuterCornerRadius` rounds the tape's top corner on the OUTER edge (the
// edge away from the attitude window: left for the airspeed tape, right for the
// altimeter), matching the NXi tape's 10 px rounded corner. 0 leaves it square.
void drawTapeBackground(Renderer& r, float x, float y, float w, float h,
                        const Color& edge, bool tapeOnRight = false,
                        float topOuterCornerRadius = 0.0f);

// `tickInset` shifts the tick marks and their labels inward from the tape's
// inner edge, leaving room for the airspeed color-band strip so the ticks are
// not hidden beneath it. 0 anchors ticks at the inner edge (altimeter).
// `labelSmallTrailing` renders that many trailing digits of each label smaller
// (G1000 NXi altitude tape: the "00" tens are smaller than the hundreds); 0
// keeps every digit the same size (airspeed).
void drawVerticalTape(Renderer& r, float tapeX, float tapeW, float stripTop,
                      float stripH, float cy, float displayH, float value,
                      float viewableUnits, float majorInterval,
                      float minorInterval, float minValue, bool tapeOnRight,
                      float minorTickFraction, float majorTickFraction,
                      float topOuterCornerRadius = 0.0f, float tickInset = 0.0f,
                      int labelSmallTrailing = 0, float tapeBgH = 0.0f);

// Draws a numeric `text` with its final `smallCount` characters reduced to
// `smallScale` of `size` and baseline-aligned with the leading characters,
// matching the G1000 NXi altitude formatting (large hundreds, small tens).
// `anchorX` is the left edge when `align == Left` and the right edge when
// `align == Right`. With `smallCount <= 0` it is a plain fillText.
void drawAltitudeNumber(Renderer& r, float anchorX, float midY,
                        const std::string& text, float size, int smallCount,
                        float smallScale, TextAlign align, const Color& color);

// Altitude tape labels: hundreds/thousands end at kAltTapeLabelHeadAnchorFraction
// and the smaller trailing "00" ends at kAltTapeLabelZerosAnchorFraction (WT NXi).
void drawAltitudeTapeLabel(Renderer& r, float tapeX, float tapeW, float midY,
                           const std::string& text, float size, float smallScale,
                           const Color& color);

// Returns the fillText y that vertically centers glyph ink within [top, bot].
float inkMidYInBand(Renderer& r, float top, float bot, float anchorX,
                    const std::string& text, float size, TextAlign align);

// Like inkMidYInBand but centers ink on rowY, then clamps ink to [top, bot].
float inkMidYAtRow(Renderer& r, float rowY, float top, float bot, float anchorX,
                   const std::string& text, float size, TextAlign align,
                   FontFace face = FontFace::Default);

// Centers leading-digit ink in the stepped band plus an em nudge (+y = down).
float tapeReadoutLeadMidY(Renderer& r, float leadTop, float leadBot,
                          float anchorX, const std::string& text, float size,
                          float inkNudgeEm, float inkNudgePx = 0.0f,
                          TextAlign align = TextAlign::Right);

struct ReadoutColumnRect {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
};

// NXi digit-box shading: black -> rgb(30,30,30) -> black per column.
Color readoutDialMidShade(const Color& edge);
void drawReadoutDigitDialShading(Renderer& r, float x, float y, float w, float h,
                                 const Color& edge);
void drawReadoutDrumScrollerMask(Renderer& r, float x, float y, float w, float h,
                                 const Color& edge);
void layoutReadoutDigitColumns(float left, float right, float top, float height,
                               int columnCount, float gapPx,
                               std::vector<ReadoutColumnRect>& out);
// Pack fixed-width columns against `right` (IAS ones column layout).
void layoutReadoutDigitColumnsPackedRight(float right, float top, float height,
                                          int columnCount, float gapPx,
                                          float colW,
                                          std::vector<ReadoutColumnRect>& out);
// Pack columns right-to-left ending before the drum (alt leading digits).
void layoutReadoutDigitColumnsBeforeDrum(float drumX, float drumGap, float top,
                                         float height, int columnCount,
                                         float pitch, float colW,
                                         std::vector<ReadoutColumnRect>& out);

// Right-align anchor for leading digits: tight-abut to the drum when there is
// room, otherwise end before drumX so the lead clip does not bisect glyphs.
float tapeReadoutLeadAnchorX(float drumX, float drumAnchorX,
                             float activeDrumDigitW, float gapPx);

struct TapeReadoutDigitMidY {
  float leadMidY = 0.0f;
  float drumMidY = 0.0f;
};

// Row-aligns leading digits with the active drum digit, clamped to the stepped
// lead band so tall glyphs do not clip.
TapeReadoutDigitMidY tapeReadoutDigitMidY(
    Renderer& r, float leadTop, float leadBot, float drumClipTop,
    float drumClipH, float leadAnchorX, const std::string& headStr,
    float headSize, float drumAnchorX, const std::string& drumStr,
    float drumSize, bool showHead);

void drawTrendVector(Renderer& r, float edgeX, float stripTop, float stripH,
                     float cy, float displayH, float ppu, float trend);

// Rounds the corners of a closed polygon: each vertex flagged with radius > 0 is
// replaced by a short quadratic arc tangent to its two edges (the vertex is the
// Bezier control point); radius <= 0 keeps the corner sharp. Shared by the
// IAS/altitude readout boxes and the selected-altitude box so both instrument
// columns share one rounded-corner style (NXi).
std::vector<Point> roundPolygonCorners(const std::vector<Point>& poly,
                                       const std::vector<float>& radius,
                                       int segments = 5);

struct TapeReadoutShape {
  float drumX = 0.0f;
  float drumW = 0.0f;
  float drumRight = 0.0f;
  float leadTop = 0.0f;
  float leadBot = 0.0f;
  std::vector<Point> silhouette;
  std::vector<float> radii;
};

// Builds the stepped IAS/altitude pointer silhouette from PFD Default screenshot
// geometry. (x,y,w,h) is the interior body rectangle; the caret protrudes from
// the outer edge (right for IAS, left for altitude).
TapeReadoutShape buildTapeReadoutShape(float x, float y, float w, float h,
                                       float drumW, NotchSide caretSide,
                                       float caretHalfH, float caretDepth,
                                       float leadHeightFraction, float cornerR,
                                       float drumCornerR);

// Minimum gap between the leading-digit run and the drum column inner edge.
constexpr float kReadoutColumnGapPx = 1.0f;

float putText(Renderer& r, float x, float y, const std::string& s, float size,
              const Color& c, float trailingGapFrac = 0.25f);

void drawArcBand(Renderer& r, float cx, float cy, float innerR, float outerR,
                 float startDeg, float endDeg, const Color& c);

// Static scale ticks drawn inside a failed moving-tape window, anchored to the
// tape's inner edge (NXi Fig 9-2: airspeed/altitude/VSI keep their ticks under
// the red X). None for non-tape failures (attitude, HSI).
enum class FailTicks { None, LeftEdge, RightEdge };

// Reversionary failure annunciation: fills the given instrument region maroon
// and draws a large red X across it, with an optional amber system label (e.g.
// "AHRS", "HDG") and optional retained tape ticks. Used when a sensor feeding
// that instrument has failed.
void drawFailureX(Renderer& r, float x, float y, float w, float h,
                  const std::string& label, float displayH,
                  FailTicks ticks = FailTicks::None,
                  float minorTickFraction = kAsiTapeMinorTickFraction,
                  float majorTickFraction = kAsiTapeMajorTickFraction);

// Per-instrument entry points (called from PrimaryFlightDisplay::render).
void drawAttitude(Renderer& r, const Layout& L, const FlightData& d, float w,
                  float h, bool powerUp = false);
void drawAirspeedTape(Renderer& r, const Layout& L, const FlightData& d,
                      const SoftkeyController& ui, float h);
void drawAltimeter(Renderer& r, const Layout& L, const FlightData& d,
                   const SoftkeyController& ui, float h);
void drawVerticalSpeedIndicator(Renderer& r, const Layout& L,
                                const FlightData& d, float h);
void drawVerticalDeviation(Renderer& r, const Layout& L, const FlightData& d,
                           float h);
// `forceVisible` draws the inset regardless of the Map/HSI softkey toggle, used
// by reversionary (display-backup) mode where the inset is always shown in the
// bottom-right (the caller relocates L.insetMap* accordingly).
void drawInsetMap(Renderer& r, const Layout& L, const MapData& map,
                  const FlightData& d, const SoftkeyController& ui, float h,
                  bool forceVisible = false);
// HSI Map layout: the moving map drawn in a square region centered on the HSI
// rose (the rose is then drawn over it with a translucent backing).
void drawHsiMap(Renderer& r, const Layout& L, const MapData& map,
                const FlightData& d, const SoftkeyController& ui, float h);
void drawHsiSection(Renderer& r, const Layout& L, const FlightData& d,
                    const SoftkeyController& ui, float h,
                    bool hsiMapMode = false, bool powerUp = false);
void drawChrome(Renderer& r, const Layout& L, const FlightData& d,
                const SoftkeyController& ui, float w, float h,
                bool powerUp = false);

// Corner profile for a NAV/COM bar panel (WT NavComBox / nav-data-bar-container).
enum class NavComPanelShape {
  Floating,   // PFD: all corners rounded (panels float over the attitude)
  MfdLeft,    // MFD: flush top/left, bottom-right rounded
  MfdCenter,  // MFD: flush top, both bottom corners rounded
  MfdRight,   // MFD: flush top/right, bottom-left rounded
};

// Near-black rounded panel used for the NAV/COM boxes and the MFD navigation
// data bar (G1000 NXi NavComBox / nav-data-bar-container).
void drawNavComPanelBg(Renderer& r, float x, float y, float pw, float ph,
                       float radius,
                       NavComPanelShape shape = NavComPanelShape::Floating);

// Draws the NAV (left) and COM (right) frequency cells of the top bar: vertical
// band labels with 1/2, the boxed standby frequency, the transfer carets, the
// active frequency, and (NAV) the decoded station ident. The NAV cells fill
// [navLeft, navLeft+navW] and the COM cells fill [comLeft, comLeft+comW]; the
// caller owns the box/background and the bar's center content. Shared by the
// PFD top bar and the MFD top data bar so the two displays render identical
// radios. `barH` is the bar height; font sizes are derived from `h`.
void drawNavComFreqCells(Renderer& r, float h, float barH, float navLeft,
                         float navW, float comLeft, float comW,
                         const FlightData& d, const SoftkeyController& ui);

// Decoded COM station identifier currently shown beneath the COM box (e.g.
// "KTRM UNICOM"), or empty when nothing is decoded. The selected/transmitting
// COM transceiver wins.
std::string navComDecodeIdent(const FlightData& d);

// Draws the decoded COM station identifier in its own black rounded panel below
// the COM box, separated by a gap so the sky shows through between the two
// (G1000 NXi COM box). No-op when `ident` is empty. `comLeft`/`comW` match the
// COM box; `cornerR` is the panel corner radius; font size derives from `h`.
void drawComDecodePanel(Renderer& r, float h, float barH, float comLeft,
                        float comW, float cornerR, const std::string& ident);

}  // namespace avionics::pfd
