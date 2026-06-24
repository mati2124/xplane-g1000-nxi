#pragma once

#include <functional>

#include "avionics/FmsNavigator.h"
#include "avionics/FlightData.h"
#include "avionics/MapData.h"

namespace avionics {

bool flightPlansEqual(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b);

void syncNavigatorFlightPlan(FmsNavigator& nav, const std::vector<MapLeg>& plan);
void syncNavigatorDirectTo(FmsNavigator& nav, const MapData& map);

// Applies fly-by turn anticipation to the outbound leg DTK when the turn is
// active ("… now" in the turn-anticipation message).
NavigationSolution applyFlyByTurnCourse(NavigationSolution sol,
                                        const MapData& map,
                                        const FlightData& data, bool obsMode,
                                        CdiSource cdiSource);

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
