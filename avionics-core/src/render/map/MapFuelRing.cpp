#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>

namespace avionics::mapview {
namespace {

// Fuel range ring: reserve deducted from total endurance (45 min default).
constexpr float kFuelReserveHours = 0.75f;

}  // namespace

void drawFuelRing(Renderer& r, const FlightData& flight, float ownX,
                  float ownY, float pixelsPerNm, float viewRadiusPx) {
  const float fuelGal = flight.fuelQtyLeftGal + flight.fuelQtyRightGal;
  if (flight.fuelFlowGph < 0.5f || fuelGal <= 0.0f ||
      flight.groundSpeedKts < 30.0f) {
    return;
  }
  const float enduranceH = fuelGal / flight.fuelFlowGph;
  const float reserveH = std::max(0.0f, enduranceH - kFuelReserveHours);
  const float totalPx =
      flight.groundSpeedKts * enduranceH * pixelsPerNm;
  const float reservePx =
      flight.groundSpeedKts * reserveH * pixelsPerNm;
  // Skip when even the reserve ring is far outside the viewport.
  if (reservePx > viewRadiusPx * 4.0f) return;

  constexpr int kSeg = 72;
  Point ring[kSeg + 1];
  auto buildRing = [&](float radius) {
    for (int i = 0; i <= kSeg; ++i) {
      const float a = static_cast<float>(i) / kSeg * 2.0f * 3.14159265f;
      ring[i] = {ownX + radius * std::cos(a), ownY + radius * std::sin(a)};
    }
  };
  if (reservePx > 1.0f) {
    buildRing(reservePx);
    strokeDashedPolyline(r, ring, kSeg + 1, 1.5f, colors::kActiveGreen);
  }
  if (totalPx <= viewRadiusPx * 4.0f) {
    buildRing(totalPx);
    r.strokePolyline(ring, kSeg + 1, 1.5f, colors::kActiveGreen);
  }
}

}  // namespace avionics::mapview
