#pragma once

#include "avionics/Renderer.h"

namespace avionics {
class WeatherRadarSource;
}

namespace avionics::map {

// Draws the NEXRAD-style weather overlay for one map view as a cached RGBA
// image color-graded from the WeatherRadarSource return-strength grid, uploaded
// to the GPU when the source revision changes, and drawn heading-up with the
// aircraft at the bottom center of the texture.
//
// Returns false while no raster is ready yet (caller skips the layer).
bool drawWeatherRaster(Renderer& r, const WeatherRadarSource& weather, float cx,
                       float cy, float pixelsPerNm, float rotationDeg);

}  // namespace avionics::map
