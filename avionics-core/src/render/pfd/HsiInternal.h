#pragma once

#include "render/pfd/PfdInternal.h"

// Shared internals for the HSI. drawHsiSection (Hsi.cpp) is the orchestrator and
// owns the compass rose; each distinct HSI component is its own translation unit
// (WindIndicator.cpp, HsiCourseNeedle.cpp, HsiBearingPointer.cpp,
// HsiTurnRate.cpp, HsiCdiAnnunciation.cpp), mirroring the per-instrument split
// elsewhere under render/pfd.
namespace avionics::pfd {

// Turn-rate scale + magenta trend vector hugging the top of the compass ring.
void drawTurnRateIndicator(Renderer& r, float cx, float cy, float radius,
                           float turnRateDegPerSec);

// CDI course pointer: arrow, deviation dots, deviation bar, TO/FROM flag.
void drawCourseNeedle(Renderer& r, float radius, float courseDeg, float devDots,
                      bool toFlag, bool valid, bool doubleLine, const Color& c);

// Cyan bearing pointer needle (single bar = BRG1, double bar = BRG2).
void drawBearingPointer(Renderer& r, float radius, float bearingDeg, bool dbl);

// PFD wind panel (upper-left of the HSI): the Wind Option 1/2/3 formats.
void drawWindBox(Renderer& r, const Layout& L, float displayH,
                 const FlightData& d, WindOption option);

// Rose-mode nav-source annunciation inside the upper half of the rose (e.g.
// "GPS   TERM").
void drawCdiSource(Renderer& r, float cx, float cy, float radius,
                   const FlightData& d, const SoftkeyController& ui,
                   float displayH);

// HSI-Map-mode horizontal course-deviation band straddling the top of the map.
void drawHsiMapCourseBand(Renderer& r, const Layout& L, const FlightData& d,
                          const SoftkeyController& ui, float displayH);

}  // namespace avionics::pfd
