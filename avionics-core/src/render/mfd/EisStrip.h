#pragma once

#include "avionics/Eis.h"
#include "avionics/FlightData.h"
#include "avionics/Renderer.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

// EIS strip width as a fraction of the display width, matching the dedicated
// engine column on the left edge of the real MFD. The turbofan (Perspective
// Touch+) synoptic strip is noticeably wider than the piston G1000 strip. The
// PFD reuses this in reversionary (display-backup) mode so the engine column
// is the same width on both displays.
float eisStripWidthFrac(EisStripStyle style);

// Engine Indication System strip on the left edge of the MFD. The gauge layout
// comes from the per-aircraft EisLayout (g1000_eis.txt beside the .acf); live
// values are read from FlightData::eisChannels via the layout's channel ids.
// Dispatches to the piston (Cessna Nav III) or turbofan (Cirrus Vision SF50)
// renderer based on layout.style.
//
// `reduced` trims the strip to only the primary engine parameters, used by the
// PFD in reversionary (display-backup) mode where the full synoptic grid does
// not fit beside the flight instruments. The piston strip already fits, so it
// ignores the flag; the turbofan strip drops its airframe synoptics.
void drawEisStrip(Renderer& r, const FlightData& d, const EisLayout& layout,
                  const Rect& area, float displayH, bool reduced = false);

// Turbofan EIS grid (Cirrus Vision SF50, Perspective Touch+ Pilot's Guide
// Fig. 3-2). Implemented in EisStripTurbofan.cpp. When `reduced`, only the
// primary engine parameters (% thrust arc, turbine bars, fuel) are drawn.
void drawEisStripTurbofan(Renderer& r, const FlightData& d,
                          const EisLayout& layout, const Rect& area,
                          float displayH, bool reduced = false);

}  // namespace avionics::mfd
