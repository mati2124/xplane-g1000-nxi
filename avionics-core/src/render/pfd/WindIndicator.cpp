#include <cmath>
#include <cstdio>
#include <string>

#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {
namespace {

// Rotating wind-direction arrow (points the way the wind is blowing TO,
// relative to aircraft heading) used by Wind Options 2 and 3. Reproduces the WT
// G1000 NXi WindOption arrow: a slim up-arrow in a 10x21 box, whitesmoke fill
// with a thin gray outline, drawn centered at (cx, cy) and scaled by `scale`.
void drawWindArrow(Renderer& r, float cx, float cy, float scale, float relDeg) {
  r.save();
  r.translate(cx, cy);
  r.rotateDegrees(relDeg);
  // WT path "M 6 21 L 6 5 L 10 5 L 5 0 l -5 5 l 4 0 L 4 21 z" recentered on the
  // 10x21 box midpoint (5, 10.5).
  const Point raw[7] = {{1.0f, 10.5f},  {1.0f, -5.5f},  {5.0f, -5.5f},
                        {0.0f, -10.5f}, {-5.0f, -5.5f}, {-1.0f, -5.5f},
                        {-1.0f, 10.5f}};
  Point poly[7];
  for (int i = 0; i < 7; ++i) poly[i] = {raw[i].x * scale, raw[i].y * scale};
  r.fillPolygon(poly, 7, colors::kWhitesmoke);
  const Point outline[8] = {poly[0], poly[1], poly[2], poly[3],
                            poly[4], poly[5], poly[6], poly[0]};
  r.strokePolyline(outline, 8, std::max(1.0f, scale), colors::kPanelBorder);
  r.restore();
}

// Single directional arrowhead (filled whitesmoke triangle) for Wind Option 1.
void drawWindOpt1Arrowhead(Renderer& r, const Point tri[3]) {
  r.fillPolygon(tri, 3, colors::kWhitesmoke);
}

}  // namespace

void drawWindBox(Renderer& r, const Layout& L, float displayH,
                 const FlightData& d, WindOption option) {
  // The wind data sits in its own translucent-black rounded panel at the
  // upper-left of the HSI, just right of the airspeed tape. Per the real NXi,
  // its top edge is level with the bottom of the airspeed tape -- i.e. the
  // GS/TAS boxes -- so GS, TAS, and the wind read as one horizontal band. Width
  // (83), padding (6px top), and radius (5px) follow the WT G1000 NXi
  // WindOverlay.css; the panel left (245) sits just outboard of the tape.
  if (option == WindOption::Off) return;

  const float panelX = 245.0f * L.sx;
  // Align with the GS/TAS box top (AirspeedTape.cpp boxY) = bottom of the tape.
  const float panelY = L.stripTop + L.stripH - 3.0f * L.s;
  const float panelW = 83.0f * L.sx;
  const float panelH = 53.0f * L.sy;
  r.fillRoundedRect(panelX, panelY, panelW, panelH, 5.0f * L.s,
                    colors::kWindBox);

  const float dirSize = fontPx(wt::kHsiSource, displayH);  // size14
  if (!d.windValid || d.windSpeedKts < 1.0f) {
    r.fillText(panelX + panelW * 0.5f, panelY + panelH * 0.5f, "NO WIND DATA",
               dirSize, TextAlign::Center, colors::kLabelText);
    return;
  }

  // Panel-relative canvas px (WT WindOverlay coords) -> display px. The
  // wind-overlay's 6px top padding shifts the content group down.
  constexpr float kPadTop = 6.0f;
  const auto cx = [&](float x) { return panelX + x * L.sx; };
  const auto cy = [&](float y) { return panelY + (y + kPadTop) * L.sy; };

  constexpr float kDeg2Rad = 3.14159265f / 180.0f;
  // Wind FROM direction; the arrow flies the way the wind blows TO (FROM+180).
  const float rel = d.windDirectionDeg + 180.0f - d.headingDeg;

  if (option == WindOption::Option1) {
    // Headwind/tailwind (vertical) and crosswind (horizontal) components shown
    // as a cross with directional arrowheads and numeric values (WT Option1).
    const float a = (d.headingDeg - d.windDirectionDeg) * kDeg2Rad;
    const float crosswind = d.windSpeedKts * std::sin(a);
    const float headwind = d.windSpeedKts * std::cos(a);
    const float barW = std::max(1.0f, 2.0f * L.s);
    // Base cross (WT: 20x2 horizontal bar at y11-13, 2x20 vertical bar at x16-18).
    r.strokeLine(cx(7.0f), cy(12.0f), cx(27.0f), cy(12.0f), barW,
                 colors::kWhitesmoke);
    r.strokeLine(cx(17.0f), cy(2.0f), cx(17.0f), cy(22.0f), barW,
                 colors::kWhitesmoke);
    if (crosswind > 0.0f) {  // wind from the left: arrowhead points right
      const Point tri[3] = {{cx(29.0f), cy(12.0f)}, {cx(24.0f), cy(17.0f)},
                            {cx(24.0f), cy(7.0f)}};
      drawWindOpt1Arrowhead(r, tri);
    } else if (crosswind < 0.0f) {  // wind from the right: points left
      const Point tri[3] = {{cx(5.0f), cy(12.0f)}, {cx(10.0f), cy(7.0f)},
                            {cx(10.0f), cy(17.0f)}};
      drawWindOpt1Arrowhead(r, tri);
    }
    if (headwind > 0.0f) {  // headwind: arrowhead points down
      const Point tri[3] = {{cx(17.0f), cy(24.0f)}, {cx(12.0f), cy(19.0f)},
                            {cx(22.0f), cy(19.0f)}};
      drawWindOpt1Arrowhead(r, tri);
    } else if (headwind < 0.0f) {  // tailwind: points up
      const Point tri[3] = {{cx(17.0f), cy(0.0f)}, {cx(12.0f), cy(5.0f)},
                            {cx(22.0f), cy(5.0f)}};
      drawWindOpt1Arrowhead(r, tri);
    }
    const float valSize = fontPx(wt::kWindValue, displayH);  // size18
    r.fillText(cx(34.0f), cy(11.0f), formatInt(std::fabs(crosswind)), valSize,
               TextAlign::Left, colors::kWhite);
    r.fillText(cx(17.0f), cy(34.0f), formatInt(std::fabs(headwind)), valSize,
               TextAlign::Center, colors::kWhite);
    return;
  }

  // Options 2 & 3 share the rotating wind-direction arrow (WT: box at (18, 9),
  // center (23, 19.5)).
  drawWindArrow(r, cx(23.0f), cy(19.5f), L.s, rel);
  if (option == WindOption::Option2) {
    // Speed only, right of the arrow (WT Option2 shows the number, no unit).
    r.fillText(cx(40.0f), cy(19.5f), formatInt(d.windSpeedKts),
               fontPx(wt::kWindValue, displayH), TextAlign::Left,
               colors::kWhite);
  } else {
    // Option 3: numeric wind direction (top) and speed + "KT" (bottom).
    int dir = static_cast<int>(std::lround(d.windDirectionDeg)) % 360;
    if (dir <= 0) dir += 360;  // NXi shows 360, never 000
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%03d\u00b0", dir);
    r.fillText(cx(40.0f), cy(10.0f), std::string(buf), dirSize, TextAlign::Left,
               colors::kWhite);
    const float unitSize = fontPx(wt::kWindUnit, displayH);  // size10
    float tx = putText(r, cx(40.0f), cy(32.0f), formatInt(d.windSpeedKts),
                       dirSize, colors::kWhite, 0.1f);
    putText(r, tx, cy(32.0f), "KT", unitSize, colors::kWhite);
  }
}

}  // namespace avionics::pfd
