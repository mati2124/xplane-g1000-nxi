#include "render/map/MapViewInternal.h"

#include <cstddef>
#include <vector>

namespace avionics::mapview {

void drawFlightPlan(Renderer& r, const MapData& map, const Proj& proj,
                    const MapViewConfig& config, const FlightData& flight,
                    float symSize, float labelSize) {
  // Garmin route coloring: the whole flight plan is white except the active
  // leg (and its TO waypoint), which is magenta. The active leg is located by
  // matching the FMS active waypoint ident against the route.
  int activeTo = -1;
  if (!flight.fmaToWpt.empty()) {
    for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
      if (map.flightPlan[i].id == flight.fmaToWpt) {
        activeTo = static_cast<int>(i);
        break;
      }
    }
  }

  std::vector<Point> route;
  route.reserve(map.flightPlan.size());
  for (const MapLeg& leg : map.flightPlan) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(leg.lat, leg.lon, x, y);
    route.push_back({x, y});
  }
  r.strokePolyline(route.data(), static_cast<int>(route.size()), 2.0f,
                   colors::kWhite);
  if (activeTo >= 1) {
    const Point activeLeg[2] = {route[static_cast<size_t>(activeTo) - 1],
                                route[static_cast<size_t>(activeTo)]};
    r.strokePolyline(activeLeg, 2, 2.0f, colors::kMagenta);
  }
  for (std::size_t i = 0; i < map.flightPlan.size(); ++i) {
    const Color c = static_cast<int>(i) == activeTo ? colors::kMagenta
                                                    : colors::kWhite;
    r.fillCircle(route[i].x, route[i].y, symSize * 0.35f, c);
    if (config.style.showLabels && !map.flightPlan[i].id.empty()) {
      r.fillText(route[i].x + symSize, route[i].y - symSize * 0.3f,
                 map.flightPlan[i].id, labelSize * 0.85f, TextAlign::Left, c);
    }
  }
}

void drawProcedurePreview(Renderer& r, const Proj& proj,
                          const MapViewConfig& config, float symSize,
                          float labelSize) {
  // Dashed cyan course through the published fixes, drawn over the route like
  // the NXi procedure preview (PROC menu on the FPL page).
  std::vector<Point> preview;
  preview.reserve(config.procedurePreview->size());
  for (const MapLeg& leg : *config.procedurePreview) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(leg.lat, leg.lon, x, y);
    preview.push_back({x, y});
  }
  strokeDashedPolyline(r, preview.data(), static_cast<int>(preview.size()),
                       2.0f, colors::kCyan);
  for (std::size_t i = 0; i < preview.size(); ++i) {
    r.fillCircle(preview[i].x, preview[i].y, symSize * 0.32f, colors::kCyan);
    if (config.style.showLabels && !(*config.procedurePreview)[i].id.empty()) {
      r.fillText(preview[i].x + symSize, preview[i].y - symSize * 0.3f,
                 (*config.procedurePreview)[i].id, labelSize * 0.85f,
                 TextAlign::Left, colors::kCyan);
    }
  }
}

void drawDirectToCourse(Renderer& r, const MapData& map, const Proj& proj,
                        const MapViewConfig& config, float symSize,
                        float labelSize) {
  // A magenta line from the aircraft straight to the direct-to waypoint, drawn
  // over the flight plan (G1000 GPS Direct-To). The origin tracks ownship, so
  // the leg shrinks as the aircraft flies toward it.
  float ox = 0.0f, oy = 0.0f, tx = 0.0f, ty = 0.0f;
  proj.toPx(map.ownshipLat, map.ownshipLon, ox, oy);
  proj.toPx(map.directTo.lat, map.directTo.lon, tx, ty);
  const Point dto[2] = {{ox, oy}, {tx, ty}};
  r.strokePolyline(dto, 2, 2.0f, colors::kMagenta);
  r.fillCircle(tx, ty, symSize * 0.35f, colors::kMagenta);
  if (config.style.showLabels && !map.directTo.id.empty()) {
    r.fillText(tx + symSize, ty - symSize * 0.3f, map.directTo.id,
               labelSize * 0.85f, TextAlign::Left, colors::kMagenta);
  }
}

}  // namespace avionics::mapview
