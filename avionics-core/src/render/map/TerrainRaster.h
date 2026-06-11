#pragma once

#include "avionics/Renderer.h"

namespace avionics {
class TerrainSource;
}

namespace avionics::map {

// How the terrain background colors elevation, mirroring the NXi TER softkey
// states: absolute topographic shading (Topo) or altitude-relative proximity
// coloring (REL: red within 100 ft below ownship, yellow within 1000 ft).
enum class TerrainRasterMode { Absolute, Relative };

// Draws the terrain background for one map view as a cached raster image:
// a north-up RGBA texture covering ~2.4x the map range, sampled per-pixel
// from the TerrainSource (with hillshading in Absolute mode), regenerated
// progressively over a few frames only when the view pans/zooms or the mode
// changes, and drawn rotated/offset under the map each frame.
//
// State is kept internally per (renderer, view center) so every map instance
// (PFD inset, MFD MAP page, embedded airport windows) gets its own raster
// without the callers managing lifetimes. Returns false while no raster is
// ready for this view yet (caller should paint its plain background).
bool drawTerrainRaster(Renderer& r, const TerrainSource& terrain,
                       TerrainRasterMode mode, float ownAltFt,
                       double viewCenterLat, double viewCenterLon, float cx,
                       float cy, float pixelsPerNm, float rotationDeg,
                       float rangeNm);

}  // namespace avionics::map
