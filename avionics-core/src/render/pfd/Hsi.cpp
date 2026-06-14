#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

// Fixed ownship symbol at the rose center: the G1000 NXi HSI airplane
// silhouette (top-down plan view, nose up toward the lubber line, main wing
// forward, horizontal stabilizer at the tail). It does not rotate -- the card
// turns beneath it. Vertices are taken from the Working Title NXi HSIRose
// symbol path (rose radius 149 in that coordinate space) and expressed as
// fractions of our rose radius; filled white with no outline like the real
// unit.
void drawHsiAircraftSymbol(Renderer& r, float cx, float cy, float radius) {
  const float s = radius;
  const Point body[] = {
      {-0.134f * s, 0.007f * s},   // left wing trailing edge at fuselage
      {-0.134f * s, -0.020f * s},  // left wingtip
      {-0.027f * s, -0.067f * s},  // left fuselage at wing leading edge
      {-0.027f * s, -0.134f * s},  // left cockpit
      {0.000f * s, -0.154f * s},   // nose
      {0.027f * s, -0.134f * s},   // right cockpit
      {0.027f * s, -0.067f * s},   // right fuselage at wing leading edge
      {0.134f * s, -0.020f * s},   // right wingtip
      {0.134f * s, 0.007f * s},    // right wing trailing edge
      {0.027f * s, 0.007f * s},    // right fuselage below wing
      {0.027f * s, 0.087f * s},    // right fuselage at stabilizer leading edge
      {0.060f * s, 0.121f * s},    // right stabilizer tip
      {0.060f * s, 0.134f * s},    // right tail trailing edge
      {-0.060f * s, 0.134f * s},   // left tail trailing edge
      {-0.060f * s, 0.121f * s},   // left stabilizer tip
      {-0.027f * s, 0.087f * s},   // left fuselage at stabilizer leading edge
      {-0.027f * s, 0.007f * s},   // left fuselage below wing
  };
  constexpr int kCount = static_cast<int>(sizeof(body) / sizeof(body[0]));
  Point pts[kCount];
  for (int i = 0; i < kCount; ++i) pts[i] = {cx + body[i].x, cy + body[i].y};
  r.fillPolygon(pts, kCount, colors::kWhite);
}

void drawTurnRateIndicator(Renderer& r, float cx, float cy, float radius,
                           float turnRateDegPerSec) {
  // The turn-rate scale hugs the top of the compass ring. The G1000 maps a
  // standard-rate turn (3 deg/sec, an 18 deg heading change in 6 s) to the long
  // outer tick and half-standard (9 deg) to the short inner tick, on each side
  // of the lubber line.
  const float stdTick = radius * 0.10f;
  const float halfTick = radius * 0.055f;

  r.save();
  r.translate(cx, cy);
  auto tick = [&](float deg, float len) {
    r.save();
    r.rotateDegrees(deg);
    r.strokeLine(0.0f, -radius, 0.0f, -radius - len, 2.0f, colors::kWhite);
    r.restore();
  };
  tick(-18.0f, stdTick);
  tick(-9.0f, halfTick);
  tick(9.0f, halfTick);
  tick(18.0f, stdTick);

  // Magenta turn-rate trend vector: an arc on the ring from the lubber line to
  // the heading predicted in six seconds at the present turn rate, capped just
  // past standard rate. Beyond 4 deg/sec an arrowhead is shown and the
  // prediction is no longer valid.
  const float predicted = turnRateDegPerSec * 6.0f;
  const float capped = std::max(-24.0f, std::min(24.0f, predicted));
  if (std::fabs(capped) > 0.5f) {
    constexpr int kSeg = 24;
    Point arc[kSeg + 1];
    for (int i = 0; i <= kSeg; ++i) {
      float x, y;
      polarOffset(capped * (i / static_cast<float>(kSeg)), radius, x, y);
      arc[i] = {x, y};
    }
    const float lineW = std::max(3.0f, radius * 0.035f);
    r.strokePolyline(arc, kSeg + 1, lineW, colors::kMagenta);

    if (std::fabs(turnRateDegPerSec) > 4.0f) {
      const float dir = capped > 0.0f ? 1.0f : -1.0f;
      float tx, ty, bx, by;
      polarOffset(capped, radius, tx, ty);
      polarOffset(capped - dir * 6.0f, radius, bx, by);
      const float ux = tx - bx, uy = ty - by;
      const float ul = std::sqrt(ux * ux + uy * uy);
      const float nx = -uy / ul, ny = ux / ul;
      const float ah = radius * 0.05f;
      const Point head[3] = {{tx, ty},
                             {bx + nx * ah, by + ny * ah},
                             {bx - nx * ah, by - ny * ah}};
      r.fillPolygon(head, 3, colors::kMagenta);
    }
  }
  r.restore();
}

void drawCourseNeedle(Renderer& r, float radius, float courseDeg, float devDots,
                      bool toFlag, bool valid, bool doubleLine, const Color& c) {
  // NXi CDI needle: a thin arrow whose head sits just inside the compass ring,
  // a short fixed shaft and tail, four deviation dots at 32 px (~0.21 r)
  // spacing, and a moving deviation bar offset by the cross-track in dots. The
  // course pointer is a single-line arrow for GPS/VOR1/LOC1 and a double-line
  // arrow for VOR2/LOC2 (Pilot's Guide, HSI).
  r.save();
  r.rotateDegrees(courseDeg);

  const float tip = radius * 0.88f;
  const float headLen = radius * 0.12f;
  const float headHalf = radius * 0.055f;
  const float dotSpacing = radius * 0.209f;
  const float barHalf = radius * 0.36f;
  const float tailInner = radius * 0.50f;
  const float tailOuter = radius * 0.80f;
  const float shaftW = std::max(2.0f, radius * 0.014f);
  const float dotR = radius * 0.018f;

  const Point head[3] = {
      {0.0f, -tip}, {-headHalf, -tip + headLen}, {headHalf, -tip + headLen}};
  r.fillPolygon(head, 3, c);
  if (doubleLine) {
    // Two parallel rails for the shaft and tail give the VOR2/LOC2 pointer its
    // double-line look while sharing the single arrowhead and deviation bar.
    const float off = radius * 0.028f;
    for (int k = 0; k < 2; ++k) {
      const float x = (k == 0 ? -off : off);
      r.strokeLine(x, -tip + headLen, x, -barHalf, shaftW, c);
      r.strokeLine(x, tailInner, x, tailOuter, shaftW, c);
    }
  } else {
    r.strokeLine(0.0f, -tip + headLen, 0.0f, -barHalf, shaftW, c);
    r.strokeLine(0.0f, tailInner, 0.0f, tailOuter, shaftW, c);
  }

  for (int i = 1; i <= 2; ++i) {
    const float dx = static_cast<float>(i) * dotSpacing;
    r.fillCircle(-dx, 0.0f, dotR, colors::kWhite);
    r.fillCircle(dx, 0.0f, dotR, colors::kWhite);
  }

  if (valid) {
    const float off = std::max(-2.0f, std::min(2.0f, devDots)) * dotSpacing;
    r.strokeLine(off, -barHalf, off, barHalf, shaftW, c);

    const float ty = radius * 0.46f;
    const float th = radius * 0.075f;
    if (toFlag) {
      const Point to[3] = {{0.0f, -ty - th}, {-th, -ty}, {th, -ty}};
      r.fillPolygon(to, 3, c);
    } else {
      const Point fr[3] = {{0.0f, ty + th}, {-th, ty}, {th, ty}};
      r.fillPolygon(fr, 3, c);
    }
  }
  r.restore();
}

void drawBearingPointer(Renderer& r, float radius, float bearingDeg, bool dbl) {
  // NXi bearing pointers are thin (2 px) cyan needles: an arrowhead just inside
  // the ring with a short upper shaft, and a separate tail near the bottom of
  // the ring, leaving the center clear. The double-bar needle (BRG2) doubles
  // the shaft/tail and uses an open (chevron) arrowhead.
  r.save();
  r.rotateDegrees(bearingDeg);
  const Color c = colors::kCyan;
  const float headTip = radius * 0.84f;
  const float headLen = radius * 0.14f;
  const float headHalf = radius * 0.085f;
  const float shoulder = headTip - headLen;
  const float upperInner = radius * 0.50f;
  const float tailInner = radius * 0.50f;
  const float tailOuter = radius * 0.78f;
  const float w = 2.0f * (radius / 153.0f);

  if (!dbl) {
    r.strokeLine(0.0f, -shoulder, 0.0f, -upperInner, w, c);
    r.strokeLine(0.0f, tailInner, 0.0f, tailOuter, w, c);
    const Point head[3] = {
        {0.0f, -headTip}, {-headHalf, -shoulder}, {headHalf, -shoulder}};
    r.fillPolygon(head, 3, c);
  } else {
    const float off = radius * 0.035f;
    for (int k = 0; k < 2; ++k) {
      const float x = (k == 0 ? -off : off);
      r.strokeLine(x, -shoulder, x, -upperInner, w, c);
      r.strokeLine(x, tailInner, x, tailOuter, w, c);
    }
    // Open chevron arrowhead.
    r.strokeLine(0.0f, -headTip, -headHalf, -shoulder, w, c);
    r.strokeLine(0.0f, -headTip, headHalf, -shoulder, w, c);
  }
  r.restore();
}

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

// PFD wind panel: the wind data sits in its own translucent-black rounded panel
// at the upper-left of the HSI, just right of the airspeed tape. Per the real
// NXi, its top edge is level with the bottom of the airspeed tape -- i.e. the
// GS/TAS boxes -- so GS, TAS, and the wind read as one horizontal band. Width
// (83), padding (6px top), and radius (5px) follow the WT G1000 NXi
// WindOverlay.css; the panel left (245) sits just outboard of the tape.
void drawWindBox(Renderer& r, const Layout& L, float displayH,
                 const FlightData& d, WindOption option) {
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

void drawHsi(Renderer& r, const Layout& L, float cx, float cy, float radius,
             float displayH, const FlightData& d, const SoftkeyController& ui,
             bool hsiMapMode) {
  const auto X = [&](float px) { return px * L.sx; };
  const auto Y = [&](float px) { return px * L.sy; };

  const float headingDeg = d.headingDeg;
  const float selectedHeadingDeg = d.selectedHeadingDeg;
  const Color navColor =
      (d.cdiSource == CdiSource::Gps) ? colors::kMagenta : colors::kActiveGreen;
  const float labelSize = fontPx(wt::kRoseLetter, displayH);
  const float headingBoxW = fontPx(wt::kHeadingBox, displayH) * 2.8f;
  const float headingBoxH = fontPx(wt::kHeadingBox, displayH) * 1.15f;
  // HSI Map: heading box sits at container top + 27 px (WT hsi-map-hdg-box).
  // Rose: just above the compass ring.
  const float headingBoxY =
      hsiMapMode ? Y(387.0f + 27.0f)
                 : cy - radius - displayH * 0.052f;

  drawTurnRateIndicator(r, cx, cy, radius, d.turnRateDegPerSec);

  // In HSI Map mode the moving map is drawn behind the rose, so the backing is
  // translucent to let the map show through; otherwise it is the solid NXi rose.
  if (hsiMapMode) {
    Color backing = colors::kRoseBackground;
    backing.a *= 0.45f;
    r.fillCircle(cx, cy, radius, backing);
  } else {
    r.fillCircle(cx, cy, radius, colors::kRoseBackground);
  }

  const float clipPad = radius * 0.10f;
  r.save();
  r.clip(cx - radius - clipPad, cy - radius - clipPad,
         2.0f * (radius + clipPad), 2.0f * (radius + clipPad));
  r.translate(cx, cy);
  r.rotateDegrees(-headingDeg);

  // The NXi compass rose has no outer ring/outline; the card boundary is
  // implied entirely by the 5deg/10deg tick marks and the cardinal labels.
  const float majorTick = radius * 0.10f;
  const float minorTick = radius * 0.052f;
  for (int deg = 0; deg < 360; deg += 5) {
    const bool major = (deg % 10) == 0;
    r.save();
    r.rotateDegrees(static_cast<float>(deg));
    const float len = major ? majorTick : minorTick;
    r.strokeLine(0.0f, -radius, 0.0f, -radius + len, major ? 2.0f : 1.5f,
                 colors::kWhite);
    r.restore();
  }

  const float cardLabelR = radius - majorTick - radius * 0.14f;
  {
    // The compass card direction labels (N/3/6/E ...) are rendered in the
    // heavier face so they read boldly against the moving map and ticks.
    const FontScope roseFont(r, FontFace::DejaVuSemiBold);
    for (int deg = 0; deg < 360; deg += 30) {
      r.save();
      r.rotateDegrees(static_cast<float>(deg));
      const bool cardinal = (deg % 90) == 0;
      r.fillText(0.0f, -cardLabelR, roseLabel(deg),
                 cardinal ? fontPx(wt::kRoseCardinal, displayH) : labelSize,
                 TextAlign::Center, colors::kWhite);
      r.restore();
    }
  }

  // Bearing pointers are shown only when enabled by the PFD Opt > Bearing 1/2
  // softkeys (G1000 NXi: the needles are off by default).
  if (d.bearing1Valid && ui.displayToggle(DisplayToggle::Bearing1)) {
    drawBearingPointer(r, radius, d.bearing1Deg, false);
  }
  if (d.bearing2Valid && ui.displayToggle(DisplayToggle::Bearing2)) {
    drawBearingPointer(r, radius, d.bearing2Deg, true);
  }
  drawCourseNeedle(r, radius, d.courseDeg, d.cdiDeviationDots, d.cdiToFlag,
                   d.navSignalValid, d.cdiSource == CdiSource::Nav2, navColor);

  {
    r.save();
    r.rotateDegrees(selectedHeadingDeg);
    const float halfW = radius * 0.085f;
    const float outR = radius + radius * 0.07f;
    const float notch = radius * 0.045f;
    const Point bug[7] = {{-halfW, -outR}, {halfW, -outR}, {halfW, -radius},
                          {notch, -radius}, {0.0f, -radius + notch},
                          {-notch, -radius}, {-halfW, -radius}};
    r.fillPolygon(bug, 7, colors::kCyan);
    r.restore();
  }

  // Current track over the ground: a magenta diamond riding the inner edge of
  // the compass ring, rotating with the card so it points at the track value.
  {
    r.save();
    r.rotateDegrees(d.trackDeg);
    const float dh = radius * 0.052f;
    const float dw = radius * 0.044f;
    const float cyD = -radius + dh;
    const Point diamond[4] = {
        {0.0f, cyD - dh}, {dw, cyD}, {0.0f, cyD + dh}, {-dw, cyD}};
    r.fillPolygon(diamond, 4, colors::kMagenta);
    r.restore();
  }
  r.restore();

  drawHsiAircraftSymbol(r, cx, cy, radius);

  // Lubber line: fixed white triangle at the top of the rose, drawn over the
  // card and rose backing so it always reads at full brightness.
  {
    const float lubW = radius * 0.06f;
    const float lubH = radius * 0.08f;
    const float lubApexY = cy - radius + lubH * 0.15f;
    const Point lubber[3] = {{cx, lubApexY},
                             {cx - lubW, lubApexY - lubH},
                             {cx + lubW, lubApexY - lubH}};
    r.fillPolygon(lubber, 3, colors::kWhite);
  }

  // The heading box shows the degree symbol, e.g. "360°" at north (NXi).
  drawReadoutBox(r, cx - headingBoxW * 0.5f, headingBoxY, headingBoxW,
                 headingBoxH, formatHeading(headingDeg) + "\u00b0",
                 fontPx(wt::kHeadingBox, displayH), NotchSide::None);

  // Selected heading (HDG) and selected course (DTK/CRS) readouts. These are
  // shared HSI chrome (WT hdgcrs-container, anchored to the #HSI parent): the
  // same translucent rounded boxes in BOTH the standard rose and HSI Map
  // layouts, so they never move or restyle when the map is toggled. Each box
  // pairs a white label with a same-size colored value (cyan for the selected
  // heading, GPS-magenta / VOR-green for the selected course). The HDG box's
  // right edge lines up under the GPS source box, and the DTK box's left edge
  // under the flight-phase box, of the course-deviation band above (the band is
  // centered on the rose; the same anchors are reused in the rose layout).
  const char* crsLabel = (d.cdiSource == CdiSource::Gps) ? "DTK " : "CRS ";
  const float refBoxH = Y(30.0f);
  const float refBoxY = Y(387.0f + 24.0f);
  const float refBoxR = 5.0f * L.s;
  const float refBoxMidY = refBoxY + refBoxH * 0.5f;
  const float refSize = fontPx(wt::kHsiRefValue, displayH);  // label and value
  const float refPad = X(8.0f);

  const float bandDevW = X(183.0f);
  const float bandGap = X(2.0f);
  const float bandDevX = L.hsiMapCx - bandDevW * 0.5f;
  const float gpsBoxRight = bandDevX - bandGap;
  const float phaseBoxLeft = bandDevX + bandDevW + bandGap;

  // Draws "<label> <value>" in a content-sized box: white label and colored
  // value at the same size, anchored either by its right edge (HDG, under the
  // GPS box) or its left edge (DTK, under the phase box).
  const auto drawRefBox = [&](float anchorX, bool anchorRight,
                              const std::string& label, const std::string& value,
                              const Color& valueColor) {
    const float labelW = r.measureTextWidth(label, refSize);
    const float valueW = r.measureTextWidth(value, refSize);
    const float boxW = labelW + valueW + 2.0f * refPad;
    const float boxX = anchorRight ? anchorX - boxW : anchorX;
    r.fillRoundedRect(boxX, refBoxY, boxW, refBoxH, refBoxR, colors::kWindBox);
    r.fillText(boxX + refPad, refBoxMidY, label, refSize, TextAlign::Left,
               colors::kWhite);
    r.fillText(boxX + refPad + labelW, refBoxMidY, value, refSize,
               TextAlign::Left, valueColor);
  };

  drawRefBox(gpsBoxRight, true, "HDG ",
             formatHeading(selectedHeadingDeg) + "\u00b0", colors::kCyan);
  drawRefBox(phaseBoxLeft, false, crsLabel,
             formatHeading(d.courseDeg) + "\u00b0", navColor);

  // Bearing-pointer source/distance windows are rendered in the bottom info
  // panel (NXi places them there, not inside the rose).
}

// Resolves the active CDI source label and its color (GPS magenta, VOR/LOC
// green), plus whether it is GPS (which adds a flight-phase annunciation).
const char* cdiSourceLabel(const FlightData& d, bool obs, Color& color,
                           bool& isGps) {
  isGps = false;
  switch (d.cdiSource) {
    case CdiSource::Gps:  color = colors::kMagenta;     isGps = true; return obs ? "OBS" : "GPS";
    case CdiSource::Nav1: color = colors::kActiveGreen;               return "VOR1";
    case CdiSource::Nav2: color = colors::kActiveGreen;               return "VOR2";
  }
  color = colors::kMagenta;
  return "GPS";
}

void drawCdiSource(Renderer& r, float cx, float cy, float radius,
                   const FlightData& d, const SoftkeyController& ui,
                   float displayH) {
  // Rose layout: nav source and (for GPS) the flight phase are annunciated
  // inside the upper half of the rose, straddling the course pointer (e.g.
  // "GPS   TERM"). When OBS mode is on, automatic waypoint sequencing is
  // suspended and "OBS"/"SUSP" annunciate (G1000 NXi Pilot's Guide, OBS).
  const bool obs = ui.displayToggle(DisplayToggle::Obs);
  Color c;
  bool isGps;
  const char* text = cdiSourceLabel(d, obs, c, isGps);
  const float size = fontPx(wt::kHsiSource, displayH);
  const float y = cy - radius * 0.20f;
  if (isGps) {
    r.fillText(cx - radius * 0.27f, y, text, size, TextAlign::Center, c);
    const std::string phase = obs ? "SUSP" : d.gpsFlightPhase;
    if (!phase.empty()) {
      // Per the G1000 Pilot's Guide (Table 2-3), the flight-phase annunciation
      // is normally magenta (amber only under cautionary conditions), matching
      // the GPS source color rather than the cyan used for selected references.
      r.fillText(cx + radius * 0.27f, y, phase, size, TextAlign::Center,
                 colors::kMagenta);
    }
  } else {
    r.fillText(cx, y, text, size, TextAlign::Center, c);
  }
}

// HSI Map layout: instead of annunciating the source/phase inside the rose, the
// NXi draws a horizontal course-deviation band straddling the top of the map --
// a translucent box holding the lateral deviation scale (four dots, a center
// line, and the TO/FROM deviation triangle), flanked by the GPS/VOR source box
// on the left and the flight-phase (sensitivity) box on the right. Geometry
// follows the WT NXi HSIMapCourseDeviation (#HSI origin 277,387; container +7).
void drawHsiMapCourseBand(Renderer& r, const Layout& L, const FlightData& d,
                          const SoftkeyController& ui, float displayH) {
  const auto X = [&](float px) { return px * L.sx; };
  const auto Y = [&](float px) { return px * L.sy; };

  const float bandY = Y(387.0f);
  const float bandH = Y(23.0f);
  const float midY = bandY + bandH * 0.5f;
  const float radius = 5.0f * L.s;
  const float devW = X(183.0f);
  const float devX = L.hsiMapCx - devW * 0.5f;  // band centered on the rose
  const float gap = X(2.0f);
  const float srcW = X(53.0f);
  const float phaseW = X(84.0f);
  const float srcX = devX - gap - srcW;
  const float phaseX = devX + devW + gap;
  const float size = fontPx(wt::kHsiBug, displayH);  // 18 px, per WT

  const bool obs = ui.displayToggle(DisplayToggle::Obs);
  Color srcColor;
  bool isGps;
  const char* srcText = cdiSourceLabel(d, obs, srcColor, isGps);

  // Source box (left).
  r.fillRoundedRect(srcX, bandY, srcW, bandH, radius, colors::kWindBox);
  r.fillText(srcX + srcW * 0.5f, midY, srcText, size, TextAlign::Center,
             srcColor);

  // Deviation box (center): translucent fill with a thin gray outline.
  r.fillRoundedRect(devX, bandY, devW, bandH, radius, colors::kWindBox);
  r.strokeRoundedRect(devX, bandY, devW, bandH, radius, 1.0f,
                      colors::kPanelBorder);

  if (d.navSignalValid) {
    r.save();
    r.clip(devX, bandY, devW, bandH);
    // Deviation scale: a center reference line and two dots either side of it.
    r.strokeLine(devX + X(90.5f), bandY + Y(1.0f), devX + X(90.5f),
                 bandY + Y(22.0f), 1.0f, colors::kPanelBorder);
    const float dotR = std::max(1.5f, X(3.0f));
    const float dotY = bandY + Y(11.0f);
    const float dotsX[4] = {X(20.0f), X(55.0f), X(126.0f), X(161.0f)};
    for (float dx : dotsX)
      r.fillCircle(devX + dx, dotY, dotR, colors::kWhite);

    // TO/FROM deviation triangle (apex up = TO), offset by the cross-track
    // deviation (full scale = two dots = 90.5 px, per WT setDeviation).
    const float frac = std::max(-1.0f, std::min(1.0f, d.cdiDeviationDots * 0.5f));
    const float devCx = devX + X(90.5f) + frac * X(90.5f);
    const float half = X(9.0f);
    const float topY = bandY + Y(2.0f);
    const float botY = bandY + Y(20.0f);
    const Color devColor = isGps ? colors::kMagenta : colors::kActiveGreen;
    if (d.cdiToFlag) {
      const Point to[3] = {
          {devCx, topY}, {devCx - half, botY}, {devCx + half, botY}};
      r.fillPolygon(to, 3, devColor);
    } else {
      const Point fr[3] = {
          {devCx, botY}, {devCx - half, topY}, {devCx + half, topY}};
      r.fillPolygon(fr, 3, devColor);
    }
    r.restore();
  } else {
    r.fillText(devX + devW * 0.5f, midY, "NO DTK", size, TextAlign::Center,
               colors::kWhitesmoke);
  }

  // Flight-phase / sensitivity box (right). For GPS this is the phase (e.g.
  // TERM/ENR); OBS suspends sequencing, annunciating SUSP. VOR/LOC has none.
  const std::string phase =
      isGps ? (obs ? "SUSP" : d.gpsFlightPhase) : std::string();
  if (!phase.empty()) {
    r.fillRoundedRect(phaseX, bandY, phaseW, bandH, radius, colors::kWindBox);
    r.fillText(phaseX + phaseW * 0.5f, midY, phase, size, TextAlign::Center,
               colors::kMagenta);
  }
}

}  // namespace

void drawHsiSection(Renderer& r, const Layout& L, const FlightData& d,
                    const SoftkeyController& ui, float h, bool hsiMapMode) {
  // The HSI Map layout uses a larger compass rose set lower on the display (its
  // bottom runs off behind the info panel); the standard rose layout is fully
  // visible and centered higher.
  const float cx = hsiMapMode ? L.hsiMapCx : L.hsiCx;
  const float cy = hsiMapMode ? L.hsiMapCy : L.hsiCy;
  const float radius = hsiMapMode ? L.hsiMapRadius : L.hsiRadius;

  // AHRS heading failure: the compass rose, CDI, and wind (all referenced to
  // heading) are replaced by a red X with an "HDG" annunciation.
  if (!d.headingValid) {
    const float rr = radius * 1.12f;
    drawFailureX(r, cx - rr, cy - rr, rr * 2.0f, rr * 2.0f, "HDG", h);
    return;
  }

  drawHsi(r, L, cx, cy, radius, h, d, ui, hsiMapMode);
  if (hsiMapMode) {
    drawHsiMapCourseBand(r, L, d, ui, h);
  } else {
    drawCdiSource(r, cx, cy, radius, d, ui, h);
  }
  // Wind panel: upper-left of the HSI, level with the bottom of the airspeed
  // tape (just right of the GS/TAS boxes) and above the inset map, per the real
  // NXi. The format follows the PFD Opt > Wind option.
  drawWindBox(r, L, h, d, ui.windOption());
}

}  // namespace avionics::pfd
