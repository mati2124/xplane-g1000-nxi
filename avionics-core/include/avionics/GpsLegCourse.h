#pragma once

#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

constexpr double kFlightPlanLegMatchNm = 0.75;

// Active GPS leg guidance derived from the flight plan (not X-Plane OBS course).
struct GpsLegNavigation {
  bool active = false;
  float courseDeg = 0.0f;
  float crossTrackNm = 0.0f;  // signed; + = left of track
};

bool flightPlanIdentsEqual(const std::string& a, const std::string& b);

int legIndexInPlan(const std::vector<MapLeg>& plan, const std::string& id);

// Matches by ident first, then by position (procedure fixes may use different
// id strings between the nav database and the stored flight plan).
int legIndexInPlan(const std::vector<MapLeg>& plan, const MapLeg& target);

// Resolves the active flight-plan leg index from FMA idents (case-insensitive).
int resolveActiveLegToIndex(const std::vector<MapLeg>& plan,
                            const std::string& fromWpt,
                            const std::string& toWpt);

// When the aircraft has passed the active waypoint, advance to the next leg.
int advanceLegIfWaypointCaptured(const std::vector<MapLeg>& plan,
                                 const MapData& map, int legIdx);

// Resolves the active leg for guidance, falling back to GPS DME when the sim
// destination string lags (common during approach sequencing).
int resolveNavLegToIndex(const std::vector<MapLeg>& plan, const FlightData& data,
                         const MapData& map);

// Computes desired track and cross-track error for the active GPS leg.
GpsLegNavigation computeGpsLegNavigation(const MapData& map,
                                         const FlightData& data, bool obsMode,
                                         CdiSource cdiSource);

// Applies GPS leg course + CDI to `data`. `nmPerDot` is the GPS CDI scale from
// the sim (X-Plane gps_hdef_nm_per_dot); full-scale is 2 dots.
void applyGpsLegNavigation(FlightData& data, const MapData& map, bool obsMode,
                           CdiSource cdiSource, float nmPerDot);

// Hides bypassed legs before an in-plan Direct-To target on the map route.
void applyInPlanDirectToRouteSlice(MapData& map, const std::vector<MapLeg>& plan,
                                   const MapLeg& target);

void clearFlightPlanRouteSlice(MapData& map);

}  // namespace avionics
