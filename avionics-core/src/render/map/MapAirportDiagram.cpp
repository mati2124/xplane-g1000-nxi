#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "avionics/MapRange.h"

namespace avionics::mapview {
namespace {

constexpr Color kRunwayFill{0.75f, 0.75f, 0.78f, 1.0f};
// Dashed black runway centerline, shown at close range like the real unit.
constexpr Color kRunwayCenterline{0.0f, 0.0f, 0.0f, 1.0f};
// Taxiway/apron pavement: a darker gray so the lighter runway quads drawn on
// top stay distinct (Garmin SafeTaxi shows taxiways darker than runways).
constexpr Color kTaxiwayFill{0.40f, 0.40f, 0.43f, 1.0f};
// SafeTaxi taxiway identifiers: a black box with white text, like the real
// G1000.
constexpr Color kTaxiwaySign{0.0f, 0.0f, 0.0f, 1.0f};
// Runway-end designators: a white box with a thin black border and black text,
// placed off the approach end of the runway (not on the pavement).
constexpr Color kRunwayLabelFill{1.0f, 1.0f, 1.0f, 1.0f};
constexpr Color kRunwayLabelBorder{0.0f, 0.0f, 0.0f, 1.0f};
// Cap on pavement-polygon vertices projected per frame, a safety valve against
// a pathological mega-polygon.
constexpr int kMaxPavementVerts = 512;
// NVG_ALIGN_MIDDLE centers on the font's ascender/descender line, so a numeric
// or all-caps glyph (no descender ink) rides ~0.10 em high; add this fraction
// of the font size to the centered baseline to center the glyph INK in a box.
constexpr float kCapInkCenterNudge = 0.10f;

// Paints a runway-end designator in a bordered white box just off the approach
// end of the runway (beyond the threshold, not on the pavement), rotated to
// read along the runway centerline (flipped so it never appears upside down),
// like the real-unit SafeTaxi diagram.
void drawRunwayNumber(Renderer& r, float ex, float ey, float dirX, float dirY,
                      const std::string& id, float labelSize) {
  if (id.empty()) return;
  const float len = std::sqrt(dirX * dirX + dirY * dirY);
  if (len < 1.0f) return;
  const float ux = dirX / len;
  const float uy = dirY / len;
  const float fontPx = labelSize * 0.8f;
  // Offset the box away from the threshold (opposite the runway body) so it
  // sits on the approach end rather than on the pavement.
  const float halfW = r.measureTextWidth(id, fontPx) * 0.5f + 2.0f;
  const float halfH = fontPx * 0.5f + 1.5f;
  const float offset = halfW + labelSize * 0.4f;
  const float tx = ex - ux * offset;
  const float ty = ey - uy * offset;
  float angle = std::atan2(uy, ux) * 180.0f / 3.14159265358979323846f;
  if (angle > 90.0f) angle -= 180.0f;
  if (angle < -90.0f) angle += 180.0f;
  r.save();
  r.translate(tx, ty);
  r.rotateDegrees(angle);
  r.fillRect(-halfW, -halfH, halfW * 2.0f, halfH * 2.0f, kRunwayLabelFill);
  r.strokeRoundedRect(-halfW, -halfH, halfW * 2.0f, halfH * 2.0f, 0.0f, 1.0f,
                      kRunwayLabelBorder);
  r.fillText(0.0f, fontPx * kCapInkCenterNudge, id, fontPx, TextAlign::Center,
             colors::kBlack, FontFace::DejaVuSemiBold);
  r.restore();
}

}  // namespace

void drawTaxiways(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm, const Color& holeFill) {
  if (rangeNm > kAirportDiagramMaxRangeNm) return;
  std::vector<Point> outerPts;
  std::vector<std::vector<Point>> holePts;
  for (const MapPavement& pav : map.taxiways) {
    if (pav.contours.empty() || pav.contours[0].size() < 3) continue;

    auto projectContour = [&](const std::vector<GeoPoint>& contour,
                              std::vector<Point>& out, bool& anyOnScreen) {
      const std::size_t count =
          std::min(contour.size(), static_cast<std::size_t>(kMaxPavementVerts));
      if (count < 3) return false;
      out.clear();
      out.reserve(count);
      for (std::size_t i = 0; i < count; ++i) {
        float x = 0.0f, y = 0.0f;
        proj.toPx(contour[i].lat, contour[i].lon, x, y);
        if (proj.onScreen(x, y, 0.0f)) anyOnScreen = true;
        out.push_back({x, y});
      }
      return out.size() >= 3;
    };

    bool anyOnScreen = false;
    if (!projectContour(pav.contours[0], outerPts, anyOnScreen)) continue;

    holePts.clear();
    for (std::size_t c = 1; c < pav.contours.size(); ++c) {
      std::vector<Point> hole;
      if (!projectContour(pav.contours[c], hole, anyOnScreen)) continue;
      holePts.push_back(std::move(hole));
    }
    if (!anyOnScreen) continue;

    r.fillPolygon(outerPts.data(), static_cast<int>(outerPts.size()),
                  kTaxiwayFill);
    // Punch grass islands out of the slab. NanoVG holes need a stencil buffer,
    // so overdraw the chart base instead of relying on NVG_HOLE.
    for (const std::vector<Point>& hole : holePts) {
      r.fillPolygon(hole.data(), static_cast<int>(hole.size()), holeFill);
    }
  }
}

void drawRunways(Renderer& r, const MapData& map, const Proj& proj,
                 float rangeNm, float labelSize) {
  if (rangeNm > kAirportDiagramMaxRangeNm) return;
  constexpr float kMetersPerNm = 1852.0f;
  const ClipBounds clip{proj.minX, proj.minY, proj.maxX, proj.maxY};
  for (const MapRunway& rwy : map.runways) {
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    proj.toPx(rwy.a.lat, rwy.a.lon, ax, ay);
    proj.toPx(rwy.b.lat, rwy.b.lon, bx, by);
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
    // Cull by quad-vs-viewport intersection, not threshold containment: a long
    // runway at close zoom (e.g. the crossing runway at 1000 ft range) has both
    // thresholds far off-screen while its body still crosses the visible area.
    if (!polylineIntersectsClip(quad, 4, clip, kSymbologyClipMarginPx)) {
      continue;
    }
    r.fillPolygon(quad, 4, kRunwayFill);
    // Dashed centerline, inset from each threshold so it reads like the painted
    // runway centerline (with the runway-end numbers at SafeTaxi range).
    if (len > labelSize * 4.0f) {
      const float ux = dx / len;
      const float uy = dy / len;
      const float inset = std::min(len * 0.25f, halfW * 3.0f);
      const Point line[2] = {{ax + ux * inset, ay + uy * inset},
                             {bx - ux * inset, by - uy * inset}};
      strokeDashedPolyline(r, line, 2, std::max(1.0f, halfW * 0.18f),
                           kRunwayCenterline, &clip);
    }
    // Only label runways long enough on screen to hold their numbers.
    if (len > labelSize * 4.0f) {
      drawRunwayNumber(r, ax, ay, dx, dy, rwy.idA, labelSize);
      drawRunwayNumber(r, bx, by, -dx, -dy, rwy.idB, labelSize);
    }
  }
}

void drawTaxiwayLabels(Renderer& r, const MapData& map, const Proj& proj,
                       float rangeNm, float labelSize) {
  if (rangeNm > kAirportDiagramMaxRangeNm) return;
  const float fontPx = labelSize * 0.72f;
  for (const MapTaxiwayLabel& label : map.taxiwayLabels) {
    if (label.text.empty()) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.pos.lat, label.pos.lon, x, y);
    if (!proj.onScreen(x, y, 0.0f)) continue;
    // Black box with white text, matching the real-unit SafeTaxi labels.
    const float halfW = r.measureTextWidth(label.text, fontPx) * 0.5f + 2.0f;
    const float halfH = fontPx * 0.5f + 1.5f;
    r.fillRoundedRect(x - halfW, y - halfH, halfW * 2.0f, halfH * 2.0f,
                      halfH * 0.3f, kTaxiwaySign);
    // fillText centers on the ascender/descender line; nudge the ink down so
    // the (no-descender) letters sit centered in the placard.
    r.fillText(x, y + fontPx * kCapInkCenterNudge, label.text, fontPx,
               TextAlign::Center, colors::kWhite,
               FontFace::DejaVuSemiBold);
  }
}

}  // namespace avionics::mapview
