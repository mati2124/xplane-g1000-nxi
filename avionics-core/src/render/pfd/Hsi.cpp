#include "render/pfd/HsiInternal.h"

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

void drawHsi(Renderer& r, const Layout& L, float cx, float cy, float radius,
             float displayH, const FlightData& d, const SoftkeyController& ui,
             bool hsiMapMode, bool powerUp) {
  const auto X = [&](float px) { return px * L.sx; };
  const auto Y = [&](float px) { return px * L.sy; };

  const float headingDeg = d.headingDeg;
  const float selectedHeadingDeg = d.selectedHeadingDeg;
  const Color navColor =
      (d.cdiSource == CdiSource::Gps) ? colors::kMagenta : colors::kActiveGreen;
  const float labelSize = fontPx(wt::kRoseLetter, displayH);
  const float headingBoxW =
      hsiMapMode ? X(hsi::kMapHeadingBoxW)
                 : fontPx(wt::kHeadingBox, displayH) * 2.8f;
  const float headingBoxH =
      hsiMapMode ? Y(hsi::kMapHeadingBoxH)
                 : fontPx(wt::kHeadingBox, displayH) * 1.15f;
  const float headingBoxX =
      hsiMapMode ? X(hsi::kOriginX + hsi::kMapContainerLeft + hsi::kMapHeadingBoxLeft)
                 : cx - headingBoxW * 0.5f;
  // HSI Map: WT hsi-map-hdg-box at container top + 27 px. Rose: above the ring.
  const float headingBoxY =
      hsiMapMode ? Y(hsi::kOriginY + hsi::kMapHeadingBoxTop)
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
  // The compass card direction labels (N/3/6/E ...) are suppressed during PFD
  // power-up; only the tick ring is shown while the system initializes.
  if (!powerUp) {
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

  // The heading box shows the degree symbol, e.g. "360°" at north (NXi). During
  // PFD power-up the heading source is failed, so the box is drawn as a red-X'd
  // failure window in its place (NXi Maintenance Manual Fig 9-2).
  if (powerUp) {
    drawFailureX(r, headingBoxX, headingBoxY, headingBoxW, headingBoxH, "",
                 displayH);
  } else if (hsiMapMode) {
    const std::string headingText = formatHeading(headingDeg) + "\u00b0";
    const float headingSize = fontPx(wt::kHeadingBox, displayH);
    constexpr FontFace kHeadingFace = FontFace::DejaVuSemiBold;
    const float headingBottom = headingBoxY + headingBoxH;
    r.fillRect(headingBoxX, headingBoxY, headingBoxW, headingBoxH,
               colors::kReadoutBox);
    const Point outline[5] = {
        {headingBoxX, headingBoxY},
        {headingBoxX + headingBoxW, headingBoxY},
        {headingBoxX + headingBoxW, headingBottom},
        {headingBoxX, headingBottom},
        {headingBoxX, headingBoxY}};
    r.strokePolyline(outline, 5, 2.0f, colors::kWhite);
    const float textY = inkMidYAtRow(
        r, (headingBoxY + headingBottom) * 0.5f, headingBoxY, headingBottom,
        headingBoxX + headingBoxW * 0.5f, headingText, headingSize,
        TextAlign::Center, kHeadingFace);
    r.save();
    r.clip(headingBoxX, headingBoxY, headingBoxW, headingBoxH);
    r.fillText(headingBoxX + headingBoxW * 0.5f, textY, headingText,
               headingSize, TextAlign::Center, colors::kWhite, kHeadingFace);
    r.restore();
  } else {
    drawReadoutBox(r, headingBoxX, headingBoxY, headingBoxW, headingBoxH,
                   formatHeading(headingDeg) + "\u00b0",
                   fontPx(wt::kHeadingBox, displayH), NotchSide::None);
  }

  // Selected heading (HDG) and selected course (DTK/CRS) readouts. Shared HSI
  // chrome (WT hdgcrs-container on #HSI): fixed 84x26 translucent boxes at
  // left 6 / left 276, top 26, in both rose and HSI Map layouts. Each pairs a
  // 14 px white label with a 20 px colored value (cyan / GPS-magenta / green).
  const char* crsLabel = (d.cdiSource == CdiSource::Gps) ? "DTK " : "CRS ";
  const float refBoxW = X(hsi::kRefBoxW);
  const float refBoxH = Y(hsi::kRefBoxH);
  const float refBoxY = Y(hsi::kOriginY + hsi::kRefBoxTop);
  const float refBoxR = hsi::kRefBoxRadius * L.s;
  const float refBoxBottom = refBoxY + refBoxH;
  const float hdgBoxX = X(hsi::kOriginX + hsi::kRefBoxHdgLeft);
  const float dtkBoxX = X(hsi::kOriginX + hsi::kRefBoxDtkLeft);
  const float refLabelSize = fontPx(wt::kHsiSource, displayH);
  const float refValueSize = fontPx(wt::kHsiRefValue, displayH);
  constexpr FontFace kRefLabelFace = FontFace::RobotoBold;
  constexpr FontFace kRefValueFace = FontFace::DejaVuSemiBold;

  const auto drawRefBox = [&](float boxX, const std::string& label,
                              const std::string& value,
                              const Color& valueColor) {
    r.fillRoundedRect(boxX, refBoxY, refBoxW, refBoxH, refBoxR, colors::kWindBox);
    const float labelW = r.measureTextWidth(label, refLabelSize, kRefLabelFace);
    const float valueW = r.measureTextWidth(value, refValueSize, kRefValueFace);
    const float startX = boxX + (refBoxW - labelW - valueW) * 0.5f;
    const float valueY = inkMidYAtRow(
        r, (refBoxY + refBoxBottom) * 0.5f, refBoxY, refBoxBottom,
        startX + labelW, value, refValueSize, TextAlign::Left, kRefValueFace);
    const TextRect valueRect =
        r.measureTextRect(startX + labelW, valueY, value, refValueSize,
                          TextAlign::Left, kRefValueFace);
    const TextRect labelRect =
        r.measureTextRect(startX, valueY, label, refLabelSize, TextAlign::Left,
                          kRefLabelFace);
    const float labelY = valueY + (valueRect.bottom - labelRect.bottom);
    r.save();
    r.clip(boxX, refBoxY, refBoxW, refBoxH);
    r.fillText(startX, labelY, label, refLabelSize, TextAlign::Left,
               colors::kWhite, kRefLabelFace);
    r.fillText(startX + labelW, valueY, value, refValueSize, TextAlign::Left,
               valueColor, kRefValueFace);
    r.restore();
  };

  if (!powerUp) {
    drawRefBox(hdgBoxX, "HDG ", formatHeading(selectedHeadingDeg) + "\u00b0",
               colors::kCyan);
    drawRefBox(dtkBoxX, crsLabel, formatHeading(d.courseDeg) + "\u00b0",
               navColor);
  }

  // Bearing-pointer source/distance windows are rendered in the bottom info
  // panel (NXi places them there, not inside the rose).
}

}  // namespace

void drawHsiSection(Renderer& r, const Layout& L, const FlightData& d,
                    const SoftkeyController& ui, float h, bool hsiMapMode,
                    bool powerUp) {
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

  drawHsi(r, L, cx, cy, radius, h, d, ui, hsiMapMode, powerUp);
  if (hsiMapMode) {
    drawHsiMapCourseBand(r, L, d, ui, h);
  } else {
    drawCdiSource(r, cx, cy, radius, d, ui, h);
  }
  // Wind panel: upper-left of the HSI, level with the bottom of the airspeed
  // tape (just right of the GS/TAS boxes) and above the inset map, per the real
  // NXi. The format follows the PFD Opt > Wind option. Hidden during power-up.
  if (!powerUp) {
    drawWindBox(r, L, h, d, ui.windOption());
  }
}

}  // namespace avionics::pfd
