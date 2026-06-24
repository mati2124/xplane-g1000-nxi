#pragma once

#include <string>
#include <vector>

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

// Lateral guidance output from the FMS/GPS navigator (GIA-style). One struct
// feeds the PFD FMA, HSI/CDI, map active-leg highlight, VNAV, and AP coupling.
struct NavigationSolution {
  bool active = false;
  // True when navigating Direct-To (no FROM waypoint on the FMA).
  bool directTo = false;
  std::string fromWpt;
  std::string toWpt;
  // Index of the active TO waypoint in the stored flight plan (-1 when inactive
  // or in a stand-alone Direct-To with no plan match).
  int activeLegIndex = -1;
  float desiredTrackDeg = 0.0f;
  float crossTrackNm = 0.0f;  // signed; + = left of track
  float distanceToWaypointNm = 0.0f;
  float bearingToWaypointDeg = 0.0f;
};

// Authoritative GPS/FMS navigation computer for the avionics stack.
//
// Owns the stored flight plan, active leg sequencing, Direct-To, and OBS mode.
// Shells supply ownship position each frame; the navigator publishes a single
// NavigationSolution that replaces the scattered leg-resolution heuristics in
// GpsLegCourse and the DataSource implementations.
class FmsNavigator {
 public:
  void setFlightPlan(std::vector<MapLeg> plan);
  const std::vector<MapLeg>& flightPlan() const { return plan_; }

  void setObsMode(bool obs) { obsMode_ = obs; }
  bool obsMode() const { return obsMode_; }

  // Pilot-selected leg (FPL page "Activate Leg"). Index is the TO waypoint.
  void setActiveLegIndex(int toLegIndex);

  void activateDirectTo(MapLeg target, double originLat, double originLon,
                        bool originValid);
  void clearDirectTo();
  bool directToActive() const { return directToActive_; }
  const MapLeg& directToTarget() const { return directTo_; }
  bool directToOriginValid() const { return directToOriginValid_; }
  double directToOriginLat() const { return directToOriginLat_; }
  double directToOriginLon() const { return directToOriginLon_; }

  // Advance waypoint sequencing and compute lateral guidance for the current
  // ownship position. groundSpeedKts is reserved for future fly-by lead logic.
  NavigationSolution update(double lat, double lon, float groundSpeedKts);

  int activeLegIndex() const { return activeLegIndex_; }

 private:
  bool captureWaypoint(double lat, double lon, const MapLeg& wpt) const;
  void sequenceActiveLeg(double lat, double lon);
  NavigationSolution computeDirectToSolution(double lat, double lon) const;
  NavigationSolution computeLegSolution(double lat, double lon, int toIdx) const;

  std::vector<MapLeg> plan_;
  int activeLegIndex_ = -1;
  bool obsMode_ = false;

  bool directToActive_ = false;
  MapLeg directTo_;
  double directToOriginLat_ = 0.0;
  double directToOriginLon_ = 0.0;
  bool directToOriginValid_ = false;
};

// Copies navigator outputs into FlightData for PFD/FMA/HSI rendering and AP
// coupling. nmPerDot is the GPS CDI scale (full-scale = 2 dots).
void applyNavigationSolution(FlightData& data, const NavigationSolution& nav,
                             float nmPerDot);

}  // namespace avionics
