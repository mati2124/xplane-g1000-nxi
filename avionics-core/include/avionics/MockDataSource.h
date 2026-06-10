#pragma once

#include <cstddef>
#include <vector>

#include "avionics/DataSource.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/Terrain.h"

namespace avionics {

// Animated, dependency-free data source for bring-up and tests: lets the
// standalone shell render something believable before (or instead of) a live
// X-Plane link. The aircraft flies a looping route, so the moving map shows
// realistic motion along a flight plan.
//
// By default it flies a built-in demo route with hand-placed features. The
// shell can make it use real data: setNavFeatureSource() swaps in actual
// X-Plane navaids/fixes near the aircraft, and setRoute() flies an actual
// flight plan (e.g. parsed from an .fms file).
class MockDataSource : public DataSource {
 public:
  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }

  // Fly along this route (needs >= 2 waypoints), looping back to the start.
  // Replaces the built-in demo route. Safe to call at runtime (e.g. once a
  // flight plan finishes loading) -- the aircraft repositions to the new start.
  void setRoute(std::vector<MapLeg> route);

  // Source of real nearby navaids/fixes. When set and ready it replaces the
  // hand-placed demo features so the mock map matches the X-Plane database.
  void setNavFeatureSource(const NavFeatureSource* source) {
    navFeatures_ = source;
  }

 private:
  void ensureRoute();              // lazily seed route + initial position
  void navigateRoute(double dt);   // advance the aircraft along the route
  void refreshFeatures(double dt); // pull nearby features (real or demo)

  FlightData data_;
  MapData map_;
  ProceduralTerrain terrain_;  // synthetic topo background for the mock map
  double elapsedSeconds_ = 0.0;

  std::vector<MapLeg> route_;
  std::size_t legIndex_ = 1;  // route_ index the aircraft is flying toward
  bool routeInitialized_ = false;

  const NavFeatureSource* navFeatures_ = nullptr;
  double sinceFeatureRebuild_ = 0.0;
};

}  // namespace avionics
