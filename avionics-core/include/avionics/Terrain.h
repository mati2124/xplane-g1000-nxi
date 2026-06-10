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

}  // namespace avionics
