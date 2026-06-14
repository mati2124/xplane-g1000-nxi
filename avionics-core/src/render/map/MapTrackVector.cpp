#include "render/map/MapViewInternal.h"

#include <cmath>

namespace avionics::mapview {
namespace {

// Track vector lookahead (Map Setup "Track Vector", 60 sec default).
constexpr float kTrackVectorSeconds = 60.0f;

}  // namespace

void drawTrackVector(Renderer& r, const FlightData& flight, float ownX,
                     float ownY, float pixelsPerNm, float rotation) {
  if (flight.groundSpeedKts < 30.0f) return;
  const float lenNm = flight.groundSpeedKts * kTrackVectorSeconds / 3600.0f;
  const float lenPx = lenNm * pixelsPerNm;
  const float angleRad =
      (flight.trackDeg - rotation) * 3.14159265f / 180.0f;
  const float ex = ownX + lenPx * std::sin(angleRad);
  const float ey = ownY - lenPx * std::cos(angleRad);
  r.strokeLine(ownX, ownY, ex, ey, 2.0f, colors::kCyan);
  // Small crossbar tip so the lookahead end is readable.
  const float tx = std::cos(angleRad) * 4.0f;
  const float ty = std::sin(angleRad) * 4.0f;
  r.strokeLine(ex - tx, ey - ty, ex + tx, ey + ty, 2.0f, colors::kCyan);
}

}  // namespace avionics::mapview
