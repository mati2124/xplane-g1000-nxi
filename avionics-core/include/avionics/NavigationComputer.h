#pragma once

#include <functional>

#include "avionics/FmsNavigator.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

bool flightPlansEqual(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b);

void syncNavigatorFlightPlan(FmsNavigator& nav, const MapData& map);
void syncNavigatorDirectTo(FmsNavigator& nav, const MapData& map);

// Applies fly-by turn smoothing to the outbound leg. Before the leg sequences
// it steers the outbound DTK with a centered CDI ("… now" in the
// turn-anticipation message); after it sequences it clamps the outbound
// cross-track while the aircraft is still completing the turn, so the injected
// GPS CDI never pegs full scale and X-Plane keeps NAV engaged through sharp
// fly-bys. nmPerDot is the active GPS sensitivity used to size the clamp.
NavigationSolution applyFlyByTurnCourse(NavigationSolution sol,
                                        const MapData& map,
                                        const FlightData& data, bool obsMode,
                                        CdiSource cdiSource, float nmPerDot);

void clearNavigationFields(FlightData& data);

bool shouldSyncActiveLegToSimulator(const FmsNavigator& nav,
                                    const FlightData& data, bool obsMode);

struct NavigationCallbacks {
  // Called when an in-plan Direct-To fix is captured and leg navigation resumes.
  // Argument is the new active TO leg index in the stored flight plan.
  std::function<void(int activeLegIndex)> onDirectToCaptured;
};

// One frame of GPS/FMS navigation: sync state, sequence, apply solution.
void runNavigationFrame(FmsNavigator& navigator, const MapData& map,
                        FlightData& data, bool obsMode, CdiSource cdiSource,
                        float nmPerDot, const NavigationCallbacks& callbacks);

}  // namespace avionics
