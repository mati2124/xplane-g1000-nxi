#pragma once

#include "avionics/MapRange.h"
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
// Matches the NXi map ladder top step; Map Setup "Terrain Data" range can
// declutter below this via MapViewStyle::terrainMaxRangeNm.
inline constexpr float kTerrainMaxRangeNm = kMapRangeMaxNm;
// Below this range the topo raster samples full-resolution DSF DEM; above it
// each 1° tile contributes one max-elevation value (continental zoom).
inline constexpr float kFullDetailTerrainMaxNm = 200.0f;
// Raster edge length in pixels. Full detail at close range; halved above
// kFullDetailTerrainMaxNm where each pixel already spans several NM.
inline constexpr int kTerrainFullRasterSize = 512;
// Upper cap for full-detail hillshade rasters. The MFD's wide aspect needs
// ~2*viewHalfExtentNm*pixelsPerNm texels (see rasterSizeFor); ~1150 at 10 NM.
inline constexpr int kTerrainHiResRasterSize = 1152;
inline constexpr int kTerrainCoarseRasterSize = 256;
// Minimum hillshade texels per on-screen pixel (raster spans 2*halfNm and is
// drawn at 2*halfNm*pixelsPerNm). Below ~1.0 the MFD shows facet chords.
inline constexpr float kTerrainMinRasterCellsPerScreenPixel = 1.0f;
// Separable smoothing passes on the elevation grid before hillshade normals.
inline constexpr int kTerrainHillshadeSmoothPasses = 2;
// The hillshade elevation is low-pass filtered over this ground radius before
// normals are computed. The DSF DEM stores elevation as whole-meter integers
// (1 m steps over ~90 m posts), so near-flat coastal terrain reads as discrete
// plateaus whose boundaries the hillshade renders as facet chords. Smoothing
// over ~2-3 posts bridges those steps; the radius is small versus mountain
// relief (km-scale), so real terrain detail is preserved.
inline constexpr float kTerrainHillshadeSmoothRadiusNm = 0.12f;
// Kernel radius clamp (in raster cells): keeps a coarse raster bridging at
// least one DEM post while bounding the cost on a very fine (small-range) one.
inline constexpr int kTerrainHillshadeSmoothMinRadiusCells = 1;
inline constexpr int kTerrainHillshadeSmoothMaxRadiusCells = 12;
// Fallback footprint when viewHalfExtentNm is unavailable (inset maps without
// a viewport extent pass a zero and keep the wider corner margin).
inline constexpr float kTerrainCornerRangeFactor = 3.5f;

bool drawTerrainRaster(Renderer& r, const TerrainSource& terrain,
                       TerrainRasterMode mode, float ownAltFt,
                       double viewCenterLat, double viewCenterLon, float cx,
                       float cy, float pixelsPerNm, float rotationDeg,
                       float rangeNm, float displayRangeNm,
                       float viewHalfExtentNm, float terrainMaxRangeNm);

}  // namespace avionics::map
