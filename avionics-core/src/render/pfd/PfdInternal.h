#pragma once

#include <cmath>
#include <cstdio>
#include <string>

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
  // PFD Navigation Status Box: a strip just below the top NAV/COM bar (and the
  // AFCS Status Box) carrying the active flight-plan leg and DIS/BRG.
  float navStatusTop, navStatusH;
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
constexpr float kAsiReadoutHeightWt = 50.0f;
constexpr float kAltReadoutHeightWt = 70.0f;
constexpr float kReadoutOverhangFraction = 0.10f;

// Airspeed tape
constexpr float kAirspeedViewableKnots = 60.0f;
constexpr float kAirspeedMajorKnots = 10.0f;
constexpr float kAirspeedMinorKnots = 5.0f;
constexpr float kAirspeedMinKnots = 20.0f;

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

// Altimeter tape
constexpr float kAltitudeViewableFeet = 600.0f;
constexpr float kAltitudeMajorFeet = 100.0f;
constexpr float kAltitudeMinorFeet = 20.0f;
constexpr float kAltitudeMinFeet = -2000.0f;

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
constexpr float kNavComFreq = 24.0f;
constexpr float kNavComLabel = 16.0f;
constexpr float kFmaActive = 24.0f;
constexpr float kFmaArmed = 20.0f;
constexpr float kFmaSmall = 14.0f;
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

void drawTapeBackground(Renderer& r, float x, float y, float w, float h,
                        const Color& edge);

void drawVerticalTape(Renderer& r, float tapeX, float tapeW, float stripTop,
                      float stripH, float cy, float displayH, float value,
                      float viewableUnits, float majorInterval,
                      float minorInterval, float minValue, bool tapeOnRight);

void drawTrendVector(Renderer& r, float edgeX, float stripTop, float stripH,
                     float cy, float displayH, float ppu, float trend);

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

}  // namespace avionics::pfd
