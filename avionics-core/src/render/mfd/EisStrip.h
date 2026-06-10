#pragma once

#include "avionics/FlightData.h"
#include "avionics/Renderer.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

// Engine Indication System strip on the left edge of the MFD, modeling the
// single-engine Cessna Nav III Engine Display (G1000 Pilot's Guide for Cessna
// Nav III, Section 3.1): tachometer dial, FFLOW/OIL/EGT/VAC horizontal bar
// indicators, per-tank fuel quantity, engine hours, and the
// voltmeter/ammeter rows. Drawn on every MFD page, like the real unit.
void drawEisStrip(Renderer& r, const FlightData& d, const Rect& area,
                  float displayH);

}  // namespace avionics::mfd
