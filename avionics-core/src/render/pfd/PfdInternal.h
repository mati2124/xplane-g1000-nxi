#pragma once

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

struct Layout {
  float sx, sy, s;

  float topBarH, infoPanelH, infoPanelTop, bottomBarH;
  float attTop, attBottom;
  float attCx, attCy;
  float attRegionH;
  float attVisW;
  float rollRadius;

  float stripTop, stripH;
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
  // the altimeter, and the marker-beacon annunciation box above it.
  float vdiX, vdiW;
  float markerX, markerY, markerW, markerH;

  // PFD inset map (lower-left over the attitude window). The MFD MAP page will
  // reuse MapView with a full-screen rect instead of these layout constants.
  float insetMapX, insetMapY, insetMapW, insetMapH;
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

// Tape tick geometry: minor ticks span 12% of the tape width, major ticks 24%
// (NXi airspeed/altitude tape). Readout-box heights are in WT px (airspeed IAS
// box 2.4em ~= 50 px, altitude box 70 px).
constexpr float kTapeMinorTickFraction = 0.12f;
constexpr float kTapeMajorTickFraction = 0.24f;
// The NXi moving-tape windows have a 10 px rounded outer corner (WT NXi tape
// border-radius). The airspeed tape, the altimeter selected-altitude box, and
// the IAS/altitude readout boxes all derive their rounding from this so the two
// instrument columns share one corner style.
constexpr float kTapeCornerRadiusWt = 10.0f;
// GS/TAS (airspeed) and BARO (altimeter) bottom boxes share one height and sit
// with their top edge slightly overlapping the tape scroll strip (WT NXi).
constexpr float kTapeBottomBoxHeightWt = 28.0f;
constexpr float kTapeBottomBoxTapeOverlapWt = 3.0f;
constexpr float kAsiReadoutHeightWt = 70.0f;
constexpr float kAltReadoutHeightWt = 70.0f;
// Depth of the readout box's pointing caret as a fraction of the box height. The
// altimeter readout is placed so this caret tip lands on the tape's left edge,
// keeping the whole box inside the tape's confines.
constexpr float kAltReadoutCaretDepthFraction = 0.14f;
// fillText's NVG_ALIGN_MIDDLE centers text on the font's ascender/descender
// midline, which reserves descender space that digits/caps don't use, so a
// centered all-caps/numeric glyph rides ~0.10 em high. Add this fraction of the
// font size to a vertically-centered baseline to center the glyph INK in a box.
constexpr float kCapInkCenterNudge = 0.10f;

// Airspeed tape
constexpr float kAirspeedViewableKnots = 60.0f;
constexpr float kAirspeedMajorKnots = 10.0f;
constexpr float kAirspeedMinorKnots = 5.0f;
constexpr float kAirspeedMinKnots = 20.0f;
// The color-coded speed-range strip occupies this fraction of the tape width at
// the inner edge; tape ticks are inset by it so they sit just outboard of the
// strip (white ticks beside the colored band, not under it) per the NXi.
constexpr float kAirspeedBandWidthFraction = 0.12f;

constexpr float kVsoKt = 33.0f;
constexpr float kVfeKt = 85.0f;
constexpr float kVs1Kt = 48.0f;
constexpr float kVnoKt = 129.0f;
constexpr float kVneKt = 163.0f;

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

// VSI
constexpr float kVsiMaxFpm = 2000.0f;
constexpr float kVsiScaleHalfFraction = 0.45f;

inline float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kWtCanvasHeightPx);
}

namespace wt {
constexpr float kTapeLabel = 21.0f;
constexpr float kReadout = 26.0f;
constexpr float kReadoutAlt = 30.0f;
constexpr float kSelectedAlt = 24.0f;
constexpr float kBaro = 20.0f;
constexpr float kVsi = 18.0f;
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
                      float topOuterCornerRadius = 0.0f, float tickInset = 0.0f,
                      int labelSmallTrailing = 0);

// Draws a numeric `text` with its final `smallCount` characters reduced to
// `smallScale` of `size` and baseline-aligned with the leading characters,
// matching the G1000 NXi altitude formatting (large hundreds, small tens).
// `anchorX` is the left edge when `align == Left` and the right edge when
// `align == Right`. With `smallCount <= 0` it is a plain fillText.
void drawAltitudeNumber(Renderer& r, float anchorX, float midY,
                        const std::string& text, float size, int smallCount,
                        float smallScale, TextAlign align, const Color& color);

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

float putText(Renderer& r, float x, float y, const std::string& s, float size,
              const Color& c, float trailingGapFrac = 0.25f);

void drawArcBand(Renderer& r, float cx, float cy, float innerR, float outerR,
                 float startDeg, float endDeg, const Color& c);

// Reversionary failure annunciation: blacks out the given instrument region and
// draws a large red X across it, with an optional amber system label (e.g.
// "AHRS", "HDG"). Used when a sensor feeding that instrument has failed.
void drawFailureX(Renderer& r, float x, float y, float w, float h,
                  const std::string& label, float displayH);

// Per-instrument entry points (called from PrimaryFlightDisplay::render).
void drawAttitude(Renderer& r, const Layout& L, const FlightData& d, float w,
                  float h);
void drawAirspeedTape(Renderer& r, const Layout& L, const FlightData& d,
                      const SoftkeyController& ui, float h);
void drawAltimeter(Renderer& r, const Layout& L, const FlightData& d,
                   const SoftkeyController& ui, float h);
void drawVerticalSpeedIndicator(Renderer& r, const Layout& L,
                                const FlightData& d, float h);
void drawVerticalDeviation(Renderer& r, const Layout& L, const FlightData& d,
                           float h);
void drawInsetMap(Renderer& r, const Layout& L, const MapData& map,
                  const FlightData& d, const SoftkeyController& ui, float h);
// HSI Map layout: the moving map drawn in a square region centered on the HSI
// rose (the rose is then drawn over it with a translucent backing).
void drawHsiMap(Renderer& r, const Layout& L, const MapData& map,
                const FlightData& d, const SoftkeyController& ui, float h);
void drawHsiSection(Renderer& r, const Layout& L, const FlightData& d,
                    const SoftkeyController& ui, float h,
                    bool hsiMapMode = false);
void drawChrome(Renderer& r, const Layout& L, const FlightData& d,
                const SoftkeyController& ui, float w, float h);

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
