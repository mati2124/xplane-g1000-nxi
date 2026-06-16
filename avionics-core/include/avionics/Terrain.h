#pragma once

namespace avionics {

// "Where ground elevation comes from" for the moving map's terrain background.
// The map samples this on a grid each frame, so any backend (a DEM file, an
// in-sim terrain probe, or the procedural model below) can drive the same
// topographic shading without the renderer knowing the difference.
class TerrainSource {
 public:
  virtual ~TerrainSource() = default;

  // Ground/terrain elevation in feet MSL at a geographic point.
  virtual float elevationFt(double lat, double lon) const = 0;

  // Samples one raster row: `count` points at constant latitude with longitude
  // stepping uniformly (lon[i] = lonStart + lonStep*i), writing feet MSL into
  // `out`. The terrain raster samples hundreds of thousands of points per
  // rebuild, so backends that lock or scan per sample (the DSF tile store)
  // override this to resolve the tile once per row instead of once per point.
  // The default just loops elevationFt for sources that are already cheap.
  virtual void elevationFtRow(double lat, double lonStart, double lonStep,
                              int count, float* out) const {
    for (int i = 0; i < count; ++i) {
      out[i] = elevationFt(lat, lonStart + lonStep * static_cast<double>(i));
    }
  }

  // Monotonic counter bumped whenever better data becomes available (e.g. a
  // DEM tile finishes loading), so cached terrain rasters know to resample.
  virtual unsigned revision() const { return 0; }
};

// Deterministic procedural terrain for the mock feed (and offline rendering):
// a smooth multi-octave height field over lat/lon. It is purely synthetic --
// it does NOT reflect real-world terrain -- but it is world-fixed, so the map
// scrolls over it correctly and the topographic background looks believable
// until a real elevation source is wired in.
class ProceduralTerrain : public TerrainSource {
 public:
  float elevationFt(double lat, double lon) const override;
};

namespace map {

// When enabled, DEM sampling and hillshade colorize for the map terrain
// background run on a background thread; only the GPU texture upload happens
// on the render thread. Both shells enable this: the standalone (its own render
// thread) and the X-Plane plugin (to keep the heavy rebuild off the sim thread,
// joined before its TerrainSource is destroyed).
void setAsyncTerrainBuilds(bool enabled);

}  // namespace map

}  // namespace avionics
