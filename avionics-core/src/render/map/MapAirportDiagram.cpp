#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace avionics::mapview {
namespace {

constexpr float kRunwayDiagramMaxRangeNm = 5.0f;
// Taxiways appear at the same range as the runway diagram so the airport's
// pavement shows as one picture the moment you zoom in, rather than the
// taxiways lagging a zoom step behind the runways.
constexpr float kTaxiwayDiagramMaxRangeNm = 5.0f;
// SafeTaxi identifier labels (runway numbers, taxiway letters) only appear at
// close range so they don't clutter the diagram when the whole field is small.
constexpr float kAirportLabelMaxRangeNm = 2.5f;

constexpr Color kRunwayFill{0.75f, 0.75f, 0.78f, 1.0f};
// Taxiway/apron pavement: a darker gray so the lighter runway quads drawn on
// top stay distinct (Garmin SafeTaxi shows taxiways darker than runways).
constexpr Color kTaxiwayFill{0.40f, 0.40f, 0.43f, 1.0f};
// Cap on pavement-polygon vertices projected per frame, a safety valve against
// a pathological mega-polygon.
constexpr int kMaxPavementVerts = 512;

// Paints a runway-end designator near its threshold, rotated to read along the
// runway centerline (flipped so it never appears upside down), like the painted
// numbers on the SafeTaxi diagram.
void drawRunwayNumber(Renderer& r, float ex, float ey, float dirX, float dirY,
                      const std::string& id, float labelSize) {
  if (id.empty()) return;
  const float len = std::sqrt(dirX * dirX + dirY * dirY);
  if (len < 1.0f) return;
  const float ux = dirX / len;
  const float uy = dirY / len;
  // Inset the label from the threshold toward the runway center.
  const float inset = labelSize * 1.2f;
  const float tx = ex + ux * inset;
  const float ty = ey + uy * inset;
  float angle = std::atan2(uy, ux) * 180.0f / 3.14159265358979323846f;
  if (angle > 90.0f) angle -= 180.0f;
  if (angle < -90.0f) angle += 180.0f;
  r.save();
  r.translate(tx, ty);
  r.rotateDegrees(angle);
  r.fillText(0.0f, 0.0f, id, labelSize * 0.95f, TextAlign::Center,
             colors::kWhite);
  r.restore();
}

}  // namespace

void drawTaxiways(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm) {
  if (rangeNm > kTaxiwayDiagramMaxRangeNm) return;
  std::vector<Point> pts;
  for (const MapPavement& pav : map.taxiways) {
    const std::size_t count =
        std::min(pav.outline.size(), static_cast<std::size_t>(kMaxPavementVerts));
    if (count < 3) continue;
    pts.clear();
    pts.reserve(count);
    bool anyOnScreen = false;
    for (std::size_t i = 0; i < count; ++i) {
      float x = 0.0f, y = 0.0f;
      proj.toPx(pav.outline[i].lat, pav.outline[i].lon, x, y);
      if (proj.onScreen(x, y, 0.0f)) anyOnScreen = true;
      pts.push_back({x, y});
    }
    if (!anyOnScreen) continue;
    r.fillPolygon(pts.data(), static_cast<int>(pts.size()), kTaxiwayFill);
  }
}

void drawRunways(Renderer& r, const MapData& map, const Proj& proj,
                 float rangeNm, float labelSize) {
  if (rangeNm > kRunwayDiagramMaxRangeNm) return;
  constexpr float kMetersPerNm = 1852.0f;
  const bool showNumbers = rangeNm <= kAirportLabelMaxRangeNm;
  for (const MapRunway& rwy : map.runways) {
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    proj.toPx(rwy.a.lat, rwy.a.lon, ax, ay);
    proj.toPx(rwy.b.lat, rwy.b.lon, bx, by);
    if (!proj.onScreen(ax, ay, 200.0f) && !proj.onScreen(bx, by, 200.0f)) {
      continue;
    }
    const float dx = bx - ax;
    const float dy = by - ay;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f) continue;
    // Perpendicular half-width in pixels (floor so short ranges still show a
    // visible strip).
    const float halfW = std::max(
        1.5f, (rwy.widthM / kMetersPerNm) * proj.pixelsPerNm * 0.5f);
    const float px = -dy / len * halfW;
    const float py = dx / len * halfW;
    const Point quad[4] = {{ax + px, ay + py},
                           {bx + px, by + py},
                           {bx - px, by - py},
                           {ax - px, ay - py}};
    r.fillPolygon(quad, 4, kRunwayFill);
    // Only label runways long enough on screen to hold their numbers.
    if (showNumbers && len > labelSize * 4.0f) {
      drawRunwayNumber(r, ax, ay, dx, dy, rwy.idA, labelSize);
      drawRunwayNumber(r, bx, by, -dx, -dy, rwy.idB, labelSize);
    }
  }
}

void drawTaxiwayLabels(Renderer& r, const MapData& map, const Proj& proj,
                       float rangeNm, float labelSize) {
  if (rangeNm > kAirportLabelMaxRangeNm) return;
  const float fontPx = labelSize * 0.85f;
  for (const MapTaxiwayLabel& label : map.taxiwayLabels) {
    if (label.text.empty()) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.pos.lat, label.pos.lon, x, y);
    if (!proj.onScreen(x, y, 0.0f)) continue;
    const float halfW = r.measureTextWidth(label.text, fontPx) * 0.5f + 2.5f;
    const float halfH = fontPx * 0.5f + 1.5f;
    r.fillRect(x - halfW, y - halfH, halfW * 2.0f, halfH * 2.0f,
               Color{0.0f, 0.0f, 0.0f, 0.72f});
    r.fillText(x, y - fontPx * 0.5f, label.text, fontPx, TextAlign::Center,
               colors::kWhite);
  }
}

}  // namespace avionics::mapview
