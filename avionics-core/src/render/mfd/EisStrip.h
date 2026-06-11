#pragma once

#include "avionics/Eis.h"
#include "avionics/FlightData.h"
#include "avionics/Renderer.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

// Engine Indication System strip on the left edge of the MFD. The gauge layout
// comes from the per-aircraft EisLayout (g1000_eis.txt beside the .acf); live
// values are read from FlightData::eisChannels via the layout's channel ids.
void drawEisStrip(Renderer& r, const FlightData& d, const EisLayout& layout,
                  const Rect& area, float displayH);

}  // namespace avionics::mfd
