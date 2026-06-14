#include "render/map/MapViewInternal.h"

namespace avionics::mapview {

void drawOwnshipSymbol(Renderer& r, float cx, float cy, float size,
                       float rotationDeg) {
  // Drawn as the G1000 NXi top-down airplane silhouette (a single-engine plan
  // view: nose, main wing forward, horizontal stabilizer near the tail,
  // fuselage spine), filled white with a dark outline so it reads over any map
  // layer. Modeled off the Working Title NXi own_airplane_icon (nose toward -y
  // at rotation 0, i.e. up = heading up). On a north-up map it turns with the
  // aircraft; on a track-up map it shows the crab angle (heading minus track).
  const float s = size;
  // Closed outline traced from the nose down the right side to the tail, then
  // mirrored back up the left. Coordinates are in symbol-local space (x right,
  // nose toward -y), scaled by the symbol size.
  const Point body[] = {
      {0.00f * s, -0.92f * s},   // nose
      {0.13f * s, -0.62f * s},   // right cockpit / fuselage ahead of the wing
      {0.16f * s, -0.50f * s},   // right fuselage at wing leading edge
      {0.95f * s, -0.40f * s},   // right wingtip leading edge (slight sweep)
      {0.95f * s, -0.24f * s},   // right wingtip trailing edge
      {0.18f * s, -0.30f * s},   // right fuselage at wing trailing edge
      {0.15f * s, 0.52f * s},    // right fuselage at stabilizer leading edge
      {0.40f * s, 0.60f * s},    // right stabilizer tip leading edge
      {0.40f * s, 0.72f * s},    // right stabilizer tip trailing edge
      {0.09f * s, 0.74f * s},    // right fuselage at stabilizer trailing edge
      {0.07f * s, 0.84f * s},    // right tail end
      {-0.07f * s, 0.84f * s},   // left tail end
      {-0.09f * s, 0.74f * s},   // left fuselage at stabilizer trailing edge
      {-0.40f * s, 0.72f * s},   // left stabilizer tip trailing edge
      {-0.40f * s, 0.60f * s},   // left stabilizer tip leading edge
      {-0.15f * s, 0.52f * s},   // left fuselage at stabilizer leading edge
      {-0.18f * s, -0.30f * s},  // left fuselage at wing trailing edge
      {-0.95f * s, -0.24f * s},  // left wingtip trailing edge
      {-0.95f * s, -0.40f * s},  // left wingtip leading edge
      {-0.16f * s, -0.50f * s},  // left fuselage at wing leading edge
      {-0.13f * s, -0.62f * s},  // left cockpit / fuselage ahead of the wing
  };
  constexpr int kCount = static_cast<int>(sizeof(body) / sizeof(body[0]));
  Point outline[kCount + 1];
  for (int i = 0; i < kCount; ++i) outline[i] = body[i];
  outline[kCount] = body[0];

  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(rotationDeg);
  r.fillPolygon(body, kCount, colors::kWhite);
  r.strokePolyline(outline, kCount + 1, 1.2f, colors::kMapSymbolOutline);
  r.restore();
}

}  // namespace avionics::mapview
