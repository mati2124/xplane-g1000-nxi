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

  // True when the backend samples real-world elevation tiles (X-Plane DSF).
  // Used to replace GSHHG chord land fills with a DEM coastline mask when
  // terrain display is off but TOPO chart land is on.
  virtual bool hasElevationTiles() const { return false; }

  // Monotonic counter bumped whenever better data becomes available (e.g. a
  // DEM tile finishes loading), so cached terrain rasters know to resample.
  virtual unsigned revision() const { return 0; }

  // Optional hook for backends that load data asynchronously (DSF tiles). The
  // terrain raster calls this before sampling so wide views do not half-fill
  // with transparent pixels while tiles stream in.
  virtual void ensureCoverage(double minLat, double maxLat, double minLon,
                              double maxLon,
                              bool waitForTiles = false) const {
    (void)minLat;
    (void)maxLat;
    (void)minLon;
    (void)maxLon;
    (void)waitForTiles;
  }

  // While true, DSF backends force-queue every tile touched by a raster rebuild
  // instead of throttling when the cache is full (continental zoom).
  virtual void setBulkTerrainSample(bool enabled) const { (void)enabled; }

  // Continental map range: prefer coarse grid summaries; full DEM is retained for
  // tiles near the view center when detailHalfNm is set.
  virtual void setCoarseTerrainSample(bool enabled) const { (void)enabled; }

  // Focus for progressive refinement: DSF workers prioritize and upgrade tiles
  // near this point to full DEM in the background.
  virtual void setTerrainViewCenter(double lat, double lon,
                                    float detailHalfNm) const {
    (void)lat;
    (void)lon;
    (void)detailHalfNm;
  }
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
