#include <algorithm>
#include <cmath>
#include <cstdio>

#include "render/pfd/PfdInternal.h"

#include "avionics/NavMath.h"
#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {
namespace {

// Placeholders shown for the chrome data readouts when the data link is down,
// so a dead feed reads as unknown rather than as stale live values.
constexpr const char* kFreqDash2 = "---.--";
constexpr const char* kFreqDash3 = "---.---";
constexpr const char* kOatDashes = "---";
constexpr const char* kXpdrDashes = "----";
constexpr const char* kTimeDashes = "--:--:--";

// Smooth Hermite ease for window slide/fade, matching the Working Title feel.
float smoothstep(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return t * t * (3.0f - 2.0f * t);
}

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Bold cyan double-headed transfer arrow (<->): a thin shaft with a solid
// filled triangle at each end (G1000 NXi NavCom box). Filled heads (rather than
// open chevron strokes) give the clean, symmetric triangles of the real unit.
// All dimensions scale with the arrow size so it reads correctly at any display
// resolution.
void drawTransferCarets(Renderer& r, float cx, float cy, float halfW) {
  const float tip = halfW;             // tip distance from the center
  // Elongated heads (slanted sides longer than the base) with a tall enough
  // base, leaving a middle shaft between them (the real unit's <-> is not
  // equilateral).
  const float headLen = halfW * 0.66f; // triangle length (tip -> base)
  const float headHalf = halfW * 0.34f; // triangle half-height at its base
  const float shaftStroke = halfW * 0.17f;
  const float baseX = tip - headLen;   // half-distance to each triangle base
  r.strokeLine(cx - baseX, cy, cx + baseX, cy, shaftStroke, colors::kCyan);
  const Point left[3] = {{cx - tip, cy},
                         {cx - tip + headLen, cy - headHalf},
                         {cx - tip + headLen, cy + headHalf}};
  const Point right[3] = {{cx + tip, cy},
                          {cx + tip - headLen, cy - headHalf},
                          {cx + tip - headLen, cy + headHalf}};
  r.fillPolygon(left, 3, colors::kCyan);
  r.fillPolygon(right, 3, colors::kCyan);
}

// COM/NAV band label: letters stacked vertically with 1/2 beside each row (WT
// NavComRadio 'navcom-title' / 'navcom-title-numbers'; no separator line).
void drawBandLabel(Renderer& r, const char* letters, float letterX,
                   float numberX, float barH, float row1Cy, float row2Cy,
                   float labelSize) {
  // The three band letters are stacked tightly around the bar's vertical
  // center using their own pitch (a touch over the cap height) so adjacent
  // letters never overlap, independent of the wider frequency-row spacing.
  const float letterPitch = labelSize * 0.82f;
  const float midCy = barH * 0.5f;
  for (int i = 0; i < 3 && letters[i] != '\0'; ++i) {
    r.fillText(letterX, midCy + (i - 1) * letterPitch, std::string(1, letters[i]),
               labelSize, TextAlign::Center, colors::kLabelText);
  }
  r.fillText(numberX, row1Cy, "1", labelSize, TextAlign::Center,
             colors::kLabelText);
  r.fillText(numberX, row2Cy, "2", labelSize, TextAlign::Center,
             colors::kLabelText);
}

// One NavCom box: a near-black rounded rectangle that floats over the attitude
// (the area outside the boxes is left transparent so the sky shows through),
// with a thin grey border (G1000 NXi NavCom/AFCS boxes).
void drawNavComPanelBg(Renderer& r, float x, float y, float pw, float ph,
                       float radius) {
  r.fillRoundedRect(x, y, pw, ph, radius, colors::kPanelBackground);
  r.strokeRoundedRect(x, y, pw, ph, radius, 1.0f, colors::kPanelBorder);
}

// Garmin Direct-To icon (a "D" with a horizontal arrow piercing it) drawn to
// the left of the active waypoint when a GPS Direct-To is active, matching the
// look of the "D" bezel key. Returns the x just past the glyph.
float drawDirectToIcon(Renderer& r, float x, float cy, float size,
                       const Color& color) {
  // The font has no bold weight, so the "D" is over-drawn at a small plus-
  // shaped halo of offsets to fatten its stroke toward the Garmin glyph.
  const float bold = std::max(1.0f, size * 0.03f);
  const float off[5][2] = {{0, 0}, {-bold, 0}, {bold, 0}, {0, -bold}, {0, bold}};
  for (const auto& o : off) {
    r.fillText(x + o[0], cy + o[1], "D", size, TextAlign::Left, color);
  }
  // Capital text is middle-aligned, so the visual center of the "D" sits a touch
  // above cy; pierce the arrow through there so it reads as centered on the D.
  const float ay = cy - size * 0.035f;
  const float dW = r.measureTextWidth("D", size);
  const float shaftL = x + dW * 0.42f;          // pierce through the D's bowl
  const float shaftR = x + dW + size * 0.26f;   // exit to the right of the D
  const float head = size * 0.34f;              // chunky arrowhead (the "carrot")
  r.strokeLine(shaftL, ay, shaftR, ay, std::max(2.5f, size * 0.13f), color);
  const Point tri[3] = {{shaftR + head * 0.55f, ay},
                        {shaftR - head * 0.30f, ay - head},
                        {shaftR - head * 0.30f, ay + head}};
  r.fillPolygon(tri, 3, color);
  return shaftR + head * 0.7f;
}

// Magenta flight-plan leg arrow between the FROM and TO fields (WT FmaLegIcon).
void drawFmaLegArrow(Renderer& r, float tipX, float cy, float size) {
  const float hh = size * 0.36f;
  const float shaft = size * 0.55f;
  const float body = size * 0.20f;
  const Point pts[7] = {
      {tipX, cy},
      {tipX - hh, cy - hh * 0.70f},
      {tipX - hh, cy - body},
      {tipX - shaft, cy - body},
      {tipX - shaft, cy + body},
      {tipX - hh, cy + body},
      {tipX - hh, cy + hh * 0.70f},
  };
  r.fillPolygon(pts, 7, colors::kMagenta);
}

// Green VS reference arrow (↑/↓) beside the active vertical mode (WT FMA).
void drawFmaVsArrow(Renderer& r, float cx, float cy, float size, bool up) {
  const float dir = up ? -1.0f : 1.0f;
  const Point pts[3] = {{cx, cy + dir * size * 0.55f},
                        {cx - size * 0.40f, cy - dir * size * 0.25f},
                        {cx + size * 0.40f, cy - dir * size * 0.25f}};
  r.fillPolygon(pts, 3, colors::kActiveGreen);
}

// PFD Navigation Status Box: top half of the center NavCom panel (WT
// CenterBarTopLeft / CenterBarTopRight). Active leg with a magenta arrow, plus
// DIS/BRG to the next waypoint (G1000 NXi Pilot's Guide, Flight Management).
void drawNavStatusBox(Renderer& r, float centerL, float centerW, float rowH,
                      float h, const FlightData& d) {
  const float legW = centerW * (284.0f / 506.0f);
  const float dataL = centerL + legW;
  // Center the row on its visual ink: text is nudged below the geometric
  // half-center to cancel the font's descender span (as the NavCom cells do).
  const float cy = rowH * 0.60f;
  const float dataSize = fontPx(wt::kFmaArmed, h);
  const float smallSize = fontPx(wt::kFmaSmall, h);
  // DIS/BRG labels read nearly as tall as their value on the real unit; only
  // the NM unit suffix is subscript-small.
  const float labelSize = dataSize * 0.85f;

  // Row separator between the navigation and AFCS halves of the center panel.
  r.strokeLine(centerL, rowH, centerL + centerW, rowH, 2.0f,
               colors::kPanelBorder);

  if (!d.dataLinkValid) return;

  // No FROM waypoint with an active TO means a GPS Direct-To: show the Direct-To
  // icon followed by the target identifier instead of a FROM -> TO leg.
  const bool directTo = d.fmaFromWpt.empty() && !d.fmaToWpt.empty();

  float legWidth = 0.0f;
  if (directTo) {
    legWidth += dataSize * 0.95f;  // Direct-To icon + gap
    legWidth += r.measureTextWidth(d.fmaToWpt, dataSize);
  } else {
    if (!d.fmaFromWpt.empty()) {
      legWidth += r.measureTextWidth(d.fmaFromWpt, dataSize) + dataSize * 0.12f;
    }
    if (!d.fmaToWpt.empty()) {
      legWidth += dataSize * 0.52f + dataSize * 0.12f;
      legWidth += r.measureTextWidth(d.fmaToWpt, dataSize);
    }
  }
  float x = centerL + std::max(centerW * 0.01f, (legW - legWidth) * 0.5f);
  if (directTo) {
    x = drawDirectToIcon(r, x, cy, dataSize, colors::kMagenta);
    x += dataSize * 0.18f;
    putText(r, x, cy, d.fmaToWpt, dataSize, colors::kMagenta);
  } else {
    if (!d.fmaFromWpt.empty()) {
      x = putText(r, x, cy, d.fmaFromWpt, dataSize, colors::kMagenta, 0.12f);
    }
    if (!d.fmaToWpt.empty()) {
      const float arrowTip = x + dataSize * 0.40f;
      drawFmaLegArrow(r, arrowTip, cy, dataSize * 0.70f);
      x = arrowTip + dataSize * 0.12f;
      putText(r, x, cy, d.fmaToWpt, dataSize, colors::kMagenta);
    }
  }

  if (!d.fmaToWpt.empty()) {
    // Vertical divider between the active-leg field and the DIS/BRG readouts
    // (G1000 NXi Navigation Status Box), meeting the row separator below.
    r.strokeLine(dataL, rowH * 0.18f, dataL, rowH, 1.5f, colors::kPanelBorder);

    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", d.fmaLegDistanceNm);
    // DIS readout left-aligned almost touching the divider.
    float bx = dataL + centerW * 0.01f;
    bx = putText(r, bx, cy, "DIS", labelSize, colors::kLabelText, 0.30f);
    bx = putText(r, bx, cy, std::string(buf), dataSize, colors::kMagenta,
                 0.18f);
    putText(r, bx, cy, "NM", smallSize, colors::kMagenta, 0.0f);

    // BRG readout right-aligned to the panel edge: the real unit spreads DIS
    // and BRG to opposite ends of the data field, with the bearing value
    // hugging the right edge and its grey label just to the left.
    const std::string brg = formatHeading(d.fmaLegBearingDeg) + "\u00b0";
    const float rightX = centerL + centerW - centerW * 0.025f;
    r.fillText(rightX, cy, brg, dataSize, TextAlign::Right, colors::kMagenta);
    const float brgLabelRight =
        rightX - r.measureTextWidth(brg, dataSize) - labelSize * 0.30f;
    r.fillText(brgLabelRight, cy, "BRG", labelSize, TextAlign::Right,
               colors::kLabelText);
  }
}

// AFCS Status Box: bottom half of the center NavCom panel (WT
// CenterBarBottomLeft / CenterBarBottomMiddle / fma-ap-vertical-modes). Shown
// only while the flight director is on (G1000 NXi Pilot's Guide, AFCS).
void drawAfcsStatusBox(Renderer& r, float centerL, float rowTop, float centerW,
                       float rowH, float h, const FlightData& d) {
  const float latW = centerW * (124.0f / 506.0f);
  const float apW = centerW * (114.0f / 506.0f);
  const float vertL = centerL + latW + apW;
  // Match the nav-status row: nudge below the geometric half-center so the
  // text ink sits vertically centered in the bottom half.
  const float cy = rowTop + rowH * 0.60f;
  const float smallSize = fontPx(wt::kFmaSmall, h);
  const float modeSize = fontPx(wt::kFmaActive, h);
  const float armedSize = fontPx(wt::kFmaArmed, h);
  // Armed vertical annunciations (ALTS, GS, …) are a touch smaller than the
  // lateral armed modes on the real unit.
  const float armedVertSize = armedSize * 0.85f;
  const float y0 = rowTop + rowH * 0.12f;
  const float y1 = rowTop + rowH * 0.88f;

  r.strokeLine(centerL + latW, y0, centerL + latW, y1, 1.5f,
               colors::kPanelBorder);
  r.strokeLine(vertL, y0, vertL, y1, 1.5f, colors::kPanelBorder);

  if (!d.flightDirectorActive || !d.dataLinkValid) return;

  float lx = centerL + centerW * 0.02f;
  if (!d.fmaLateralArmed.empty()) {
    lx = putText(r, lx, cy, d.fmaLateralArmed, armedSize, colors::kWhite,
                 0.40f);
  }
  if (!d.fmaLateralActive.empty()) {
    putText(r, lx, cy, d.fmaLateralActive, modeSize, colors::kActiveGreen);
  }

  {
    std::string apYd;
    if (d.apEngaged) apYd += "AP";
    if (d.ydEngaged) apYd += (apYd.empty() ? "YD" : "  YD");
    if (!apYd.empty()) {
      r.fillText(centerL + latW + apW * 0.5f, cy, apYd, modeSize,
                 TextAlign::Center, colors::kActiveGreen);
    }
  }

  float vx = vertL + centerW * 0.01f;
  if (!d.fmaVerticalActive.empty()) {
    vx = putText(r, vx, cy, d.fmaVerticalActive, modeSize, colors::kActiveGreen,
                 0.35f);
  }
  if (d.fmaVerticalValue != 0 && !d.fmaVerticalUnits.empty()) {
    const bool isVs = d.fmaVerticalUnits == "FPM";
    const bool refUp = d.fmaVerticalValue > 0;
    const int refMag = std::abs(d.fmaVerticalValue);
    const Color refColor = isVs ? colors::kActiveGreen : colors::kCyan;
    if (isVs) {
      drawFmaVsArrow(r, vx + modeSize * 0.25f, cy, modeSize * 0.55f, refUp);
      vx += modeSize * 0.55f;
    }
    vx = putText(r, vx, cy, formatInt(static_cast<float>(refMag)), armedSize,
                 refColor, 0.20f);
    putText(r, vx, cy, d.fmaVerticalUnits, smallSize, refColor);
  }

  // The armed vertical mode is right-aligned within the vertical-modes column,
  // well inboard of the panel edge (G1000 NXi FMA), not flush to the right.
  float rx = centerL + centerW * 0.87f;
  if (!d.fmaVerticalApproachArmed.empty()) {
    r.fillText(rx, cy, d.fmaVerticalApproachArmed, armedVertSize, TextAlign::Right,
               colors::kWhite);
    rx -= r.measureTextWidth(d.fmaVerticalApproachArmed, armedVertSize) +
          armedVertSize * 0.40f;
  }
  if (!d.fmaVerticalArmed.empty()) {
    r.fillText(rx, cy, d.fmaVerticalArmed, armedVertSize, TextAlign::Right,
               colors::kWhite);
  }
}

void drawTopBar(Renderer& r, float w, float h, const Layout& L,
                const FlightData& d, const SoftkeyController& ui) {
  const float barH = L.topBarH;
  // The three NavCom boxes float over the attitude: the area outside them is
  // left transparent so the sky shows through (no full-bar background fill).
  const float cornerR = barH * (10.0f / 56.0f);

  const float navPanelW = w * (250.0f / 1024.0f);
  const float centerL = w * (259.0f / 1024.0f);
  const float centerW = w * (506.0f / 1024.0f);
  const float comPanelL = w * (774.0f / 1024.0f);
  const float comPanelW = w - comPanelL;
  const float centerRowH = barH * 0.5f;

  drawNavComPanelBg(r, 0.0f, 0.0f, navPanelW, barH, cornerR);
  drawNavComPanelBg(r, centerL, 0.0f, centerW, barH, cornerR);
  drawNavComPanelBg(r, comPanelL, 0.0f, comPanelW, barH, cornerR);

  drawNavComFreqCells(r, h, barH, 0.0f, navPanelW, comPanelL, comPanelW, d, ui);
  // Decoded COM station identifier sits in its own panel below the COM box.
  drawComDecodePanel(r, h, barH, comPanelL, comPanelW, cornerR,
                     navComDecodeIdent(d));
  // The center status panel uses the heavier display face (matching the NavCom
  // frequency cells) so its text reads as bold as the real unit.
  {
    FontScope centerFont(r, FontFace::DejaVuSemiBold);
    drawNavStatusBox(r, centerL, centerW, centerRowH, h, d);
    drawAfcsStatusBox(r, centerL, centerRowH, centerW, centerRowH, h, d);
  }
}

// Filled box with the NXi bottom-panel gradient (lighter at top, black at the
// bottom).
void drawInfoBox(Renderer& r, float x, float top, float w, float h) {
  r.fillRectVerticalGradient(x, top, w, h, top, top + h, colors::kInfoBoxTop,
                             colors::kBlack);
}

// Small bearing-pointer icon drawn at the left of a Bearing Information Window:
// a single line for BRG1, a double line for BRG2 (G1000 NXi Pilot's Guide).
void drawBearingIcon(Renderer& r, float x, float cy, float len, bool dbl,
                     const Color& c) {
  if (!dbl) {
    r.strokeLine(x, cy, x + len, cy, 2.0f, c);
  } else {
    const float off = len * 0.16f;
    r.strokeLine(x, cy - off, x + len, cy - off, 2.0f, c);
    r.strokeLine(x, cy + off, x + len, cy + off, 2.0f, c);
  }
}

// One Bearing Information Window: pointer icon, source, station/waypoint
// identifier, and GPS-derived slant-range distance. When alignRight is set,
// `x` is the window's RIGHT edge (used for BRG2 so it never grows into the
// transponder box).
void drawBearingInfo(Renderer& r, float x, float cy, bool dbl,
                     const std::string& source, const std::string& ident,
                     float distNm, float size, bool alignRight = false) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.1fNM", distNm);

  if (alignRight) {
    float width = size * 1.2f + r.measureTextWidth(source, size) + size * 0.35f;
    if (!ident.empty()) {
      width += r.measureTextWidth(ident, size) + size * 0.5f;
    }
    width += r.measureTextWidth(buf, size);
    x -= width;
  }

  drawBearingIcon(r, x, cy, size * 0.9f, dbl, colors::kCyan);
  float bx = x + size * 1.2f;
  bx = putText(r, bx, cy, source, size, colors::kCyan, 0.35f);
  if (!ident.empty()) bx = putText(r, bx, cy, ident, size, colors::kCyan, 0.5f);
  putText(r, bx, cy, buf, size, colors::kMagenta, 0.0f);
}

void drawBottomInfoPanel(Renderer& r, float w, float h, const Layout& L,
                         const FlightData& d, const SoftkeyController& ui) {
  // NXi bottom info panel (y=679, height 55, full width 1024): only the OAT box
  // (0-101), the transponder box (715-900) and the time box (900-1024) are
  // opaque; the wide middle is transparent so the HSI compass rose shows
  // through, and the bearing-pointer info windows sit in that gap.
  const float top = L.infoPanelTop;
  const float panelH = L.infoPanelH;
  const float labelSize = fontPx(wt::kInfoLabel, h);
  const float valueSize = fontPx(wt::kInfoValue, h);

  const float oatW = 101.0f * L.sx;
  const float xpdrX = 715.0f * L.sx;
  const float xpdrW = 185.0f * L.sx;
  const float timeX = 900.0f * L.sx;
  const float timeW = w - timeX;
  // Generic timer box just left of the transponder box (Pilot's Guide Fig.
  // 2-1, Generic Timer); present only while the timer is running or holds a
  // value.
  const bool tmrVisible = ui.timerVisible();
  const float tmrX = 590.0f * L.sx;
  const float tmrW = xpdrX - tmrX;

  drawInfoBox(r, 0.0f, top, oatW, panelH);
  if (tmrVisible) drawInfoBox(r, tmrX, top, tmrW, panelH);
  drawInfoBox(r, xpdrX, top, xpdrW, panelH);
  drawInfoBox(r, timeX, top, timeW, panelH);

  const bool linkValid = d.dataLinkValid;
  const Color valueColor = linkValid ? colors::kWhite : colors::kBandYellow;

  // OAT box.
  const float oatCy = top + panelH * 0.62f;
  r.fillText(oatW * 0.06f, oatCy, "OAT", labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(oatW * 0.95f, oatCy,
             (linkValid ? formatOat(d.oatCelsius) : std::string(kOatDashes)) +
                 "\u00b0C",
             valueSize, TextAlign::Right, valueColor);

  // Bearing-pointer info windows flanking the rose (BRG1 left, BRG2 right),
  // each with a pointer icon, source, identifier, and distance.
  const float brgSize = fontPx(wt::kInfoLabel, h);
  const float brgRowCy = top + panelH * 0.55f;
  const bool brg1On = ui.displayToggle(DisplayToggle::Bearing1);
  const bool brg2On = ui.displayToggle(DisplayToggle::Bearing2);
  if (d.bearing1Valid && brg1On) {
    drawBearingInfo(r, 200.0f * L.sx, brgRowCy, false, d.bearing1Source,
                    d.bearing1Ident, d.bearing1DistanceNm, brgSize);
  }
  if (d.bearing2Valid && brg2On) {
    // BRG2 is anchored to the left edge of the transponder box (or the timer
    // box when shown) so the distance readout can never run into it.
    const float brg2Right = (tmrVisible ? tmrX : xpdrX) - 10.0f * L.sx;
    drawBearingInfo(r, brg2Right, brgRowCy, true, d.bearing2Source,
                    d.bearing2Ident, d.bearing2DistanceNm, brgSize,
                    /*alignRight=*/true);
  }

  // DME Information Window (PFD Opt > DME), shown above the BRG1 window: tuned
  // source/mode, frequency, and slant-range.
  if (ui.displayToggle(DisplayToggle::Dme) && d.dmeValid) {
    const float dmeCy = top - brgSize * 1.5f;
    float dx = 200.0f * L.sx;
    dx = putText(r, dx, dmeCy, "DME", brgSize, colors::kLabelText, 0.4f);
    dx = putText(r, dx, dmeCy, d.dmeMode, brgSize, colors::kCyan, 0.4f);
    dx = putText(r, dx, dmeCy, formatFreq(d.dmeFreqMhz, 2), brgSize,
                 colors::kCyan, 0.5f);
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1fNM", d.dmeDistanceNm);
    putText(r, dx, dmeCy, buf, brgSize, colors::kWhite, 0.0f);
  }

  // Generic timer box: label + elapsed h:mm:ss. The timer is local to the
  // display, so it keeps reading even with the data link down.
  if (tmrVisible) {
    const float tmrCy = top + panelH * 0.62f;
    r.fillText(tmrX + tmrW * 0.08f, tmrCy, "TMR", labelSize, TextAlign::Left,
               colors::kLabelText);
    r.fillText(xpdrX - tmrW * 0.08f, tmrCy, formatTimer(ui.timerSeconds()),
               valueSize, TextAlign::Right, colors::kWhite);
  }

  // Transponder box: code, mode and reply ("R"), green in the air. With the
  // link down the squawk/mode are unknown, so the box shows amber dashes. An
  // in-progress code entry (XPDR > Code) shows in place of the active code,
  // and the Ident function replaces the mode with a green IDNT for 18 seconds.
  const float xpdrCy = top + panelH * 0.62f;
  float xx = xpdrX + xpdrW * 0.04f;
  xx = putText(r, xx, xpdrCy, "XPDR", labelSize, colors::kLabelText, 0.40f);
  const std::string& pending = ui.xpdrPendingCode();
  if (!pending.empty()) {
    // Digits typed so far, with the remaining places dashed.
    std::string entry = pending;
    entry.append(4 - pending.size(), '-');
    xx = putText(r, xx, xpdrCy, entry, valueSize, colors::kWhite, 0.45f);
  } else if (linkValid) {
    char codeBuf[8];
    std::snprintf(codeBuf, sizeof(codeBuf), "%04d", d.transponderCode);
    xx = putText(r, xx, xpdrCy, codeBuf, valueSize, colors::kActiveGreen, 0.45f);
  } else {
    xx = putText(r, xx, xpdrCy, kXpdrDashes, valueSize, colors::kBandYellow,
                 0.45f);
  }
  if (ui.identActive()) {
    putText(r, xx, xpdrCy, "IDNT", valueSize, colors::kActiveGreen, 0.0f);
  } else if (linkValid) {
    xx = putText(r, xx, xpdrCy, d.transponderMode, valueSize,
                 colors::kActiveGreen, 0.45f);
    if (d.transponderReply) {
      putText(r, xx, xpdrCy, "R", valueSize, colors::kActiveGreen, 0.0f);
    }
  }

  // Time box: label + HH:MM:SS.
  const float timeCy = top + panelH * 0.62f;
  r.fillText(timeX + timeW * 0.06f, timeCy, d.clockIsUtc ? "UTC" : "LCL",
             labelSize, TextAlign::Left, colors::kLabelText);
  r.fillText(w - timeW * 0.05f, timeCy,
             linkValid ? formatHms(d.utcHour, d.utcMinute, d.utcSecond)
                       : std::string(kTimeDashes),
             valueSize, TextAlign::Right, valueColor);
}

// On-screen softkey label bar. Display only, like the real unit: the labels
// name the functions of the physical keys on the bezel directly below the
// screen, and all interaction happens through those keys. Each cell is a
// thin-framed label box; a selected key shows black text on a gray background
// (G1000 Pilot's Guide for the Diamond DA40, "Softkey Function"), and a press
// flashes the same look momentarily.
void drawSoftkeyBar(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float top = h - L.bottomBarH;
  r.fillRect(0.0f, top, w, L.bottomBarH, colors::kSoftkeyBackground);

  const float cellW = w / static_cast<float>(kSoftkeyCount);
  const float cy = top + L.bottomBarH * 0.5f;
  const float size = fontPx(wt::kSoftkey, h);
  const float insetX = cellW * 0.055f;
  const float insetY = L.bottomBarH * 0.13f;
  for (int i = 0; i < kSoftkeyCount; ++i) {
    const float bx = static_cast<float>(i) * cellW + insetX;
    const float by = top + insetY;
    const float bw = cellW - 2.0f * insetX;
    const float bh = L.bottomBarH - 2.0f * insetY;

    // Selected = steady black-on-gray; a key press flashes the same fill,
    // decaying back to white-on-black.
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 1.0f : 0.0f);
    if (level > 0.0f) {
      r.fillRect(bx, by, bw, bh, withAlpha(colors::kSoftkeySelected, level));
    }
    const Point frame[5] = {
        {bx, by}, {bx + bw, by}, {bx + bw, by + bh}, {bx, by + bh}, {bx, by}};
    r.strokePolyline(frame, 5, 1.0f, colors::kPanelSeparator);

    if (ui.label(i)[0] != '\0') {
      Color labelColor = level > 0.5f ? colors::kBlack : colors::kWhite;
      // The Alerts cell relabels and flashes by severity when alerts are
      // unacknowledged (warning red, caution amber, message advisory white).
      if (ui.softkeyFlashing(i)) {
        labelColor = (ui.alertSoftkeyLevel() == AlertLevel::Warning)
                         ? colors::kBandRed
                     : (ui.alertSoftkeyLevel() == AlertLevel::Caution)
                         ? colors::kBandYellow
                         : colors::kWhite;
        if (!ui.blinkOn()) labelColor = withAlpha(labelColor, 0.0f);
      }
      r.fillText(bx + bw * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 labelColor);
    }
  }
}

// Shared frame for the PFD pop-up windows (Alerts / References / Nearest
// Airports). Anchored above the softkey bar at the lower right and animated
// with an eased slide-up + fade, driven by the controller's 0..1 progress.
// Returns the eased alpha (0 = nothing drawn) and the panel geometry.
struct WindowFrame {
  float a = 0.0f;
  float x = 0.0f, top = 0.0f, w = 0.0f, h = 0.0f;
  float contentTop = 0.0f;  // first content line below the title bar
};

WindowFrame drawWindowFrame(Renderer& r, float w, float h, const Layout& L,
                            float rawAnim, const char* title, float panelW,
                            float panelH) {
  WindowFrame f;
  if (rawAnim <= 0.0f) return f;
  const float a = smoothstep(rawAnim);

  const float margin = w * 0.012f;
  const float panelX = w - panelW - margin;
  const float panelBottom = (h - L.bottomBarH) - h * 0.012f;
  // Slide up into place as it fades in.
  const float slide = (1.0f - a) * panelH * 0.22f;
  const float panelTop = panelBottom - panelH + slide;

  // Shared menu chrome (matches the PFD Setup Menu / real unit, Fig. 1-18):
  // opaque black rounded body with a thick light-grey rounded border drawn
  // inset by half its width so the stroke sits fully inside the panel.
  const float radius = panelH * 0.06f;
  const float borderW = 3.0f * (h / 768.0f);
  r.fillRoundedRect(panelX, panelTop, panelW, panelH, radius,
                    withAlpha(colors::kBlack, a));
  r.strokeRoundedRect(panelX + borderW * 0.5f, panelTop + borderW * 0.5f,
                      panelW - borderW, panelH - borderW, radius, borderW,
                      withAlpha(colors::kMenuBorderGray, a));

  // Cyan centered title with a white separator rule beneath it. Sized to match
  // the PFD Setup Menu's title/text (wt::kInfoLabel) so every pop-up shares the
  // same type size.
  const float titleSize = fontPx(wt::kInfoLabel, h);
  const float sepY = panelTop + titleSize * 1.5f;
  r.fillText(panelX + panelW * 0.5f, panelTop + titleSize * 0.85f, title,
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a));
  const float sepInset = borderW + panelW * 0.01f;
  r.strokeLine(panelX + sepInset, sepY, panelX + panelW - sepInset, sepY, 1.5f,
               withAlpha(colors::kWhitesmoke, a));

  f.a = a;
  f.x = panelX;
  f.top = panelTop;
  f.w = panelW;
  f.h = panelH;
  f.contentTop = sepY + titleSize * 0.35f;
  return f;
}

// Pop-up Alerts/messages window (under the "Alerts" key).
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // WT popout-dialog: 310x220 px on the 1024x768 canvas (same as Nearest, etc.).
  const float panelW = w * (310.0f / kWtCanvasWidth);
  const float panelH = h * (220.0f / kWtCanvasHeightPx);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Alerts), "ALERTS",
                      panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  // Message list (or an empty-state line). Matches the PFD Setup Menu text size.
  const float msgSize = fontPx(wt::kInfoLabel, h);
  const float lineH = msgSize * 1.6f;
  const float textX = f.x + f.w * 0.04f;
  float y = f.contentTop + lineH * 0.75f;

  const auto& msgs = ui.alerts();
  if (msgs.empty()) {
    r.fillText(f.x + f.w * 0.5f, f.top + f.h * 0.55f, "NO ACTIVE ALERTS",
               msgSize, TextAlign::Center, withAlpha(colors::kLabelText, a));
    return;
  }
  for (const AlertMessage& m : msgs) {
    if (y > f.top + f.h - lineH * 0.4f) break;  // clip overflow
    Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
              : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                 : colors::kWhite;
    r.fillText(textX, y, m.text, msgSize, TextAlign::Left, withAlpha(c, a));
    y += lineH;
  }
}

// One field of the References window: text with an optional highlight-select
// cursor (pulses cyan plate / black text vs plain cyan text). Returns the x
// just past the field.
float putField(Renderer& r, float x, float cy, const std::string& text,
               float size, const Color& color, bool highlighted, float alpha,
               bool blinkOn, float trailingGapFrac = 0.6f,
               FontFace face = FontFace::Default) {
  const float tw = r.measureTextWidth(text, size, face);
  if (highlighted && blinkOn) {
    const float padX = size * 0.25f;
    const float padY = size * 0.18f;
    r.fillRect(x - padX, cy - size * 0.5f - padY, tw + 2.0f * padX,
               size + 2.0f * padY, withAlpha(colors::kCyan, alpha));
  }
  const Color textColor = highlighted
                              ? (blinkOn ? colors::kBlack : colors::kCyan)
                              : color;
  r.fillText(x, cy, text, size, TextAlign::Left, withAlpha(textColor, alpha),
             face);
  return x + tw + size * trailingGapFrac;
}

// Timer/References window (Tmr/Ref softkey): generic timer, V-speed reference
// bugs, and barometric minimums (Pilot's Guide Fig. 2-6). The FMS rocker
// moves the cursor between fields; ENT activates the highlighted field.
void drawReferencesWindow(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::References),
                      "References", w * 0.34f, h * 0.46f);
  if (f.a <= 0.0f) return;
  const float a = f.a;
  const RefField cursor = ui.referencesCursor();
  const bool blinkOn = ui.blinkOn();

  const float size = fontPx(wt::kInfoLabel, h);
  // Unit suffixes (KT/FT) are rendered smaller than the value, like the unit.
  const float unitSize = size * 0.72f;
  // Up to seven rows (TIMER, four V-speeds, MINS, and the TEMP-COMP row).
  const float lineH = (f.top + f.h - f.contentTop) / 7.5f;
  const float labelX = f.x + f.w * 0.06f;
  const float numRightX = f.x + f.w * 0.60f;    // right edge of a V-speed value
  const float minsNumRightX = f.x + f.w * 0.80f;  // right edge of the MINS value
  const float toggleCenterX = f.x + f.w * 0.85f;  // On/Off toggle center
  const float minsModeCenterX = f.x + f.w * 0.46f;
  float cy = f.contentTop + lineH * 0.75f;

  // Small filled triangle carrot (the cyan toggle/list arrowheads flanking the
  // On/Off and MINS-mode fields on the real unit).
  const auto carrot = [&](float ax, float acy, bool pointRight) {
    const float aw = size * 0.26f;
    const float ah = size * 0.42f;
    Point t[3];
    if (pointRight) {
      t[0] = {ax, acy - ah * 0.5f};
      t[1] = {ax + aw, acy};
      t[2] = {ax, acy + ah * 0.5f};
    } else {
      t[0] = {ax + aw, acy - ah * 0.5f};
      t[1] = {ax, acy};
      t[2] = {ax + aw, acy + ah * 0.5f};
    }
    r.fillPolygon(t, 3, withAlpha(colors::kCyan, a));
  };

  // Center-anchored toggle/list field (On/Off, MINS mode): cyan value flanked
  // by carrots; pulses as a cyan plate while it is the cursor field.
  const auto toggleField = [&](float centerX, const std::string& text,
                               bool highlighted) {
    const float tw = r.measureTextWidth(text, size);
    const float gap = size * 0.30f;
    const float aw = size * 0.26f;
    if (highlighted && blinkOn) {
      const float padX = size * 0.22f;
      const float padY = size * 0.18f;
      r.fillRect(centerX - tw * 0.5f - padX, cy - size * 0.5f - padY,
                 tw + 2.0f * padX, size + 2.0f * padY,
                 withAlpha(colors::kCyan, a));
    }
    const Color textColor = highlighted ? (blinkOn ? colors::kBlack : colors::kCyan)
                                         : colors::kCyan;
    r.fillText(centerX, cy, text, size, TextAlign::Center,
               withAlpha(textColor, a));
    carrot(centerX - tw * 0.5f - gap - aw, cy, /*pointRight=*/false);
    carrot(centerX + tw * 0.5f + gap, cy, /*pointRight=*/true);
  };

  // Right-aligned numeric value with a smaller unit suffix and an optional
  // change-from-default asterisk; pulses as the cyan FMS edit field when it is
  // the cursor. `numRight` is the right edge of the number; the unit follows.
  const auto valueField = [&](float numRight, const std::string& num,
                              const char* unit, bool modified, bool highlighted) {
    const float numW = r.measureTextWidth(num, size);
    const float unitW = r.measureTextWidth(unit, unitSize);
    const float starW =
        modified ? r.measureTextWidth("*", unitSize) : 0.0f;
    const float left = numRight - numW;
    if (highlighted && blinkOn) {
      const float padX = size * 0.22f;
      const float padY = size * 0.18f;
      r.fillRect(left - padX, cy - size * 0.5f - padY,
                 numW + unitW + starW + 2.0f * padX, size + 2.0f * padY,
                 withAlpha(colors::kCyan, a));
    }
    const Color c = highlighted ? (blinkOn ? colors::kBlack : colors::kCyan)
                                 : colors::kCyan;
    r.fillText(left, cy, num, size, TextAlign::Left, withAlpha(c, a));
    r.fillText(numRight, cy, unit, unitSize, TextAlign::Left, withAlpha(c, a));
    if (modified) {
      r.fillText(numRight + unitW, cy, "*", unitSize, TextAlign::Left,
                 withAlpha(c, a));
    }
  };

  // TIMER row: elapsed time (cyan), count direction, and the command field
  // (Start?/Stop?/Reset?) drawn in a rounded box like the real unit.
  r.fillText(labelX, cy, "Timer", size, TextAlign::Left,
             withAlpha(colors::kWhite, a));
  r.fillText(f.x + f.w * 0.28f, cy, formatTimer(ui.timerSeconds()), size,
             TextAlign::Left, withAlpha(colors::kCyan, a));
  r.fillText(f.x + f.w * 0.58f, cy, "Up", size, TextAlign::Left,
             withAlpha(colors::kCyan, a));
  {
    const std::string cmd = ui.timerCommandLabel();
    const float cmdW = r.measureTextWidth(cmd, size);
    const float cmdCx = f.x + f.w * 0.84f;
    const float padX = size * 0.45f;
    const float padY = size * 0.26f;
    const float bx = cmdCx - cmdW * 0.5f - padX;
    const float by = cy - size * 0.5f - padY;
    const float bw = cmdW + 2.0f * padX;
    const float bh = size + 2.0f * padY;
    const float rad = bh * 0.5f;
    const bool hl = cursor == RefField::TimerCmd && blinkOn;
    if (hl) {
      r.fillRoundedRect(bx, by, bw, bh, rad, withAlpha(colors::kCyan, a));
    }
    r.strokeRoundedRect(bx, by, bw, bh, rad, 1.5f,
                        withAlpha(colors::kMenuBorderGray, a));
    r.fillText(cmdCx, cy, cmd, size, TextAlign::Center,
               withAlpha(hl ? colors::kBlack : colors::kWhite, a));
  }
  cy += lineH;

  // Separator rule below the Timer section (the real window groups the timer
  // above the V-speed/MINS list).
  const float sepInset = f.w * 0.04f;
  const float sepY = cy - lineH * 0.45f;
  r.strokeLine(f.x + sepInset, sepY, f.x + f.w - sepInset, sepY, 1.5f,
               withAlpha(colors::kWhitesmoke, a));

  // V-speed rows: label, reference value (cyan FMS edit field, asterisk when
  // changed from its default), and the On/Off enable toggle.
  for (int i = 0; i < kVSpeedRefCount; ++i) {
    const VSpeedRef& v = kVSpeedRefs[i];
    const bool on = ui.vspeedEnabled(static_cast<VspeedRef>(i));
    const RefField field =
        static_cast<RefField>(static_cast<int>(RefField::Glide) + i);
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(i));
    const bool modified =
        std::lround(vKt) != std::lround(kDefaultVspeedKt[i]);
    r.fillText(labelX, cy, v.windowLabel, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    valueField(numRightX, formatInt(vKt), "KT", modified, cursor == field);
    toggleField(toggleCenterX, on ? "On" : "Off", /*highlighted=*/false);
    cy += lineH;
  }

  // MINS row: mode toggle (Off / BARO / TEMP COMP) and, when set, the MDA/DH
  // altitude. TEMP COMP adds the destination-temperature row beneath.
  const MinimumsMode minsMode = ui.minimumsMode();
  const bool minsOn = minsMode != MinimumsMode::Off;
  const char* minsModeLabel = minsMode == MinimumsMode::Baro   ? "BARO"
                              : minsMode == MinimumsMode::Temp ? "TEMP COMP"
                                                              : "OFF";
  r.fillText(labelX, cy, "MINS", size, TextAlign::Left,
             withAlpha(colors::kWhite, a));
  toggleField(minsModeCenterX, minsModeLabel, cursor == RefField::MinsMode);
  if (minsOn) {
    valueField(minsNumRightX, formatInt(ui.minimumsAltitudeFt()), "FT",
               /*modified=*/false, cursor == RefField::MinsValue);
  }
  if (minsMode == MinimumsMode::Temp) {
    cy += lineH;
    r.fillText(labelX, cy, "Temp At", size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    char tbuf[16];
    std::snprintf(tbuf, sizeof(tbuf), "%+d\u00b0C",
                  static_cast<int>(std::lround(ui.minimumsTempC())));
    toggleField(minsModeCenterX, tbuf, cursor == RefField::MinsTemp);
    valueField(minsNumRightX, formatInt(ui.effectiveMinimumsFt()), "FT",
               /*modified=*/false, /*highlighted=*/false);
  }
}

// Nearest Airports window (Nearest softkey): a distance-sorted, scrollable
// list of nearby airports. Each entry is two plain-text rows with no field
// boxes, exactly like the real unit / Working Title NXi (Pilot's Guide
// Fig. 5-28; the boxes in that figure are the manual's callout annotations).
// Row 1: ident, airport symbol, bearing, distance, best approach.
// Row 2: COM service label, tunable frequency (cyan), longest runway.
void drawNearestWindow(Renderer& r, float w, float h, const Layout& L,
                       const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // WT popout: 310x220 px on the 1024x768 canvas; each entry is 58 px tall
  // with two 29 px rows.
  const float panelW = w * (310.0f / kWtCanvasWidth);
  const float panelH = h * (220.0f / kWtCanvasHeightPx);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Nearest),
                      "Nearest Airports", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const auto& list = ui.nearestAirports();
  const float size = fontPx(wt::kInfoLabel, h);
  const float smallSize = size * 0.875f;
  if (list.empty()) {
    const char* msg = "None within 200";
    const float msgW = r.measureTextWidth(msg, size);
    const float unitW = r.measureTextWidth("NM", smallSize);
    const float cx = f.x + f.w * 0.5f;
    const float cy = f.top + f.h * 0.55f;
    r.fillText(cx - (msgW + unitW) * 0.5f, cy, msg, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(cx - (msgW + unitW) * 0.5f + msgW, cy, "NM", smallSize,
               TextAlign::Left, withAlpha(colors::kWhite, a));
    return;
  }

  constexpr int kVisibleEntries = 3;
  const int cursor = ui.nearestCursor();
  const bool blinkOn = ui.blinkOn();
  const int first = std::max(0, cursor - kVisibleEntries + 1);
  const float entryH = fontPx(58.0f, h);
  const float rowH = entryH * 0.5f;
  const float padX = fontPx(5.0f, h);
  const float scrollW = fontPx(10.0f, h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX - scrollW - fontPx(4.0f, h);
  const float listW = listRight - listLeft;

  // Row 1 grid (WT: 1fr 29px 1fr 1fr 1fr): ident | icon | bearing | distance |
  // approach. The 29 px icon column is fixed; the four 1fr columns split the
  // rest evenly. Bearing/distance/approach are centered in their columns.
  const float iconW = fontPx(29.0f, h);
  const float colW = (listW - iconW) / 4.0f;
  const float identX = listLeft;
  const float iconCx = listLeft + colW + iconW * 0.5f;
  const float bearingCx = listLeft + colW + iconW + colW * 0.5f;
  const float distanceCx = listLeft + colW + iconW + colW * 1.5f;
  const float approachCx = listLeft + colW + iconW + colW * 2.5f;

  // Row 2 (WT flex): freqtype (30%, left) | frequency (30%, right) | RWY left.
  const float freqTypeX = listLeft;
  const float freqRight = listLeft + listW * 0.60f;
  const float rwyX = listLeft + listW * 0.62f;

  char buf[32];
  for (int row = 0; row < kVisibleEntries; ++row) {
    const int i = first + row;
    if (i >= static_cast<int>(list.size())) break;
    const NearestAirport& apt = list[i];
    const float top = f.contentTop + entryH * static_cast<float>(row);
    const float row1Cy = top + rowH * 0.5f;
    const float row2Cy = top + rowH + rowH * 0.5f;

    if (row > 0) {
      r.strokeLine(listLeft, top, listRight, top, 1.0f,
                   withAlpha(colors::kPanelSeparator, a));
    }

    // Row 1: ident (FMS cursor highlight), symbol, bearing, distance, approach.
    putField(r, identX, row1Cy, apt.id, size, colors::kCyan, i == cursor, a,
             blinkOn, 0.15f);
    MapFeature sym;
    sym.type = MapFeatureType::Airport;
    sym.airportTowered = apt.airportTowered;
    sym.airportServiced = apt.airportServiced;
    sym.airportKind = apt.airportKind;
    drawMapFeatureSymbol(r, sym, iconCx, row1Cy, fontPx(15.0f, h) * 0.5f);

    std::snprintf(buf, sizeof(buf), "%s\u00b0",
                  formatHeading(apt.bearingDeg).c_str());
    r.fillText(bearingCx, row1Cy, buf, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));
    std::snprintf(buf, sizeof(buf), "%.1fNM", apt.distanceNm);
    r.fillText(distanceCx, row1Cy, buf, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));
    const std::string& approach =
        apt.approachType.empty() ? std::string("VFR") : apt.approachType;
    r.fillText(approachCx, row1Cy, approach, size, TextAlign::Center,
               withAlpha(colors::kWhite, a));

    // Row 2: comm-service label, tunable frequency (cyan), longest runway.
    if (!apt.comLabel.empty()) {
      r.fillText(freqTypeX, row2Cy, apt.comLabel, smallSize, TextAlign::Left,
                 withAlpha(colors::kWhite, a));
    }
    if (apt.frequencyMhz > 0.0f) {
      r.fillText(freqRight, row2Cy, formatFreq(apt.frequencyMhz, 3), size,
                 TextAlign::Right, withAlpha(colors::kCyan, a));
    }
    const std::string rwy =
        apt.longestRunwayFt > 0
            ? formatInt(static_cast<float>(apt.longestRunwayFt)) + "FT"
            : "_____";
    r.fillText(rwyX, row2Cy, "RWY", smallSize, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(rwyX + r.measureTextWidth("RWY ", smallSize), row2Cy, rwy, size,
               TextAlign::Left, withAlpha(colors::kWhite, a));
  }

  // Scroll thumb (WT ScrollBar): only when more airports exist than fit.
  if (static_cast<int>(list.size()) > kVisibleEntries) {
    const float trackTop = f.contentTop;
    const float trackH = entryH * static_cast<float>(kVisibleEntries);
    const float trackX = f.x + f.w - padX - scrollW;
    const float thumbH =
        std::max(fontPx(18.0f, h),
                 trackH * static_cast<float>(kVisibleEntries) /
                     static_cast<float>(list.size()));
    const float maxScroll =
        static_cast<float>(list.size() - kVisibleEntries);
    const float thumbTop =
        trackTop +
        (trackH - thumbH) * static_cast<float>(first) / maxScroll;
    r.fillRect(trackX + scrollW * 0.5f - 1.0f, trackTop, 2.0f, trackH,
               withAlpha(colors::kPanelSeparator, a));
    r.fillRect(trackX, thumbTop, scrollW, thumbH,
               withAlpha(colors::kMenuBorderGray, a));
  }
}

// PFD Setup Menu (PFD MENU key, Pilot's Guide Fig. 1-18): a lower-right popout
// with a backlighting row for each display. Each row is an arrow-toggle target
// (Display / Key), a mode (Auto / Manual), and an intensity percentage. The
// large FMS knob moves the cursor between fields, the small knob edits the
// highlighted one (the green arrowhead shows the way the target can still
// toggle), and ENT steps onto the intensity once Manual is selected.
//
// Unlike the softkey-opened PFD windows, this one is drawn with the real unit's
// own chrome (Fig. 1-18): a near-black grey panel with a grey bevel border, a
// cyan title over a white separator rule, and the two rows packed into the top
// third over an otherwise empty panel -- not the cyan window frame.
void drawPfdSetupWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const float rawAnim = ui.windowAnim(PfdWindow::Setup);
  if (rawAnim <= 0.0f) return;
  const float a = smoothstep(rawAnim);
  const bool blinkOn = ui.blinkOn();
  const PfdSetupField cursor = ui.pfdSetupCursor();

  // Lower-right anchor with the shared slide-up + fade. The real menu's box is
  // ~1.43:1 (w:h) per Fig. 1-18 and fairly compact; keep the aspect but make it
  // small so the two rows sit tightly under the title.
  const float panelW = w * 0.28f;
  const float panelH = h * 0.26f;
  const float margin = w * 0.012f;
  const float panelX = w - panelW - margin;
  const float panelBottom = (h - L.bottomBarH) - h * 0.012f;
  const float panelTop = panelBottom - panelH + (1.0f - a) * panelH * 0.22f;

  // Body: near-black with rounded corners and a thick light-grey border, as on
  // the real unit (Fig. 1-18). The border is drawn inset by half its width so
  // the stroke sits fully inside the panel rather than straddling the edge.
  const float radius = panelH * 0.07f;
  const float borderW = 3.0f * (h / 768.0f);
  r.fillRoundedRect(panelX, panelTop, panelW, panelH, radius,
                    withAlpha(colors::kBlack, a));
  r.strokeRoundedRect(panelX + borderW * 0.5f, panelTop + borderW * 0.5f,
                      panelW - borderW, panelH - borderW, radius, borderW,
                      withAlpha(colors::kMenuBorderGray, a));

  // Title bar: cyan title near the top with a white separator rule beneath it.
  // The whole menu uses the bundled DejaVu Sans SemiBold display face to match
  // the real unit more closely than the primary Roboto UI font.
  const FontFace kMenuFace = FontFace::DejaVuSemiBold;
  const float titleSize = fontPx(wt::kInfoLabel, h);
  // Title and separator are anchored to the font size (not the panel height) so
  // they stay tight to the top regardless of panel size.
  const float sepY = panelTop + titleSize * 1.85f;
  r.fillText(panelX + panelW * 0.5f, panelTop + titleSize * 1.0f, "PFD Setup Menu",
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a),
             kMenuFace);
  const float sepInset = borderW + panelW * 0.01f;
  r.strokeLine(panelX + sepInset, sepY, panelX + panelW - sepInset, sepY, 1.5f,
               withAlpha(colors::kWhitesmoke, a));

  // Rows packed directly under the separator with single-line spacing (the
  // empty lower panel below matches the figure). Spacing keys off the font size
  // so the two rows never drift apart as the panel scales.
  const float size = fontPx(wt::kInfoLabel, h);
  const float labelX = panelX + panelW * 0.06f;
  const float modeX = panelX + panelW * 0.50f;
  const float valueRight = panelX + panelW * 0.96f;
  const float row0Cy = sepY + size * 1.35f;
  const float rowGap = size * 1.6f;
  const float rowCy[kPfdSetupRowCount] = {row0Cy, row0Cy + rowGap};

  // Small left/right-pointing arrowhead next to a target label.
  const auto arrow = [&](float ax, float acy, bool pointRight, const Color& c) {
    const float aw = size * 0.30f;
    const float ah = size * 0.46f;
    if (pointRight) {
      const Point t[3] = {{ax, acy - ah * 0.5f},
                          {ax + aw, acy},
                          {ax, acy + ah * 0.5f}};
      r.fillPolygon(t, 3, withAlpha(c, a));
    } else {
      const Point t[3] = {{ax + aw, acy - ah * 0.5f},
                          {ax, acy},
                          {ax + aw, acy + ah * 0.5f}};
      r.fillPolygon(t, 3, withAlpha(c, a));
    }
  };

  struct Row {
    const char* name;
    PfdSetupRow row;
    PfdSetupField target, mode, value;
  };
  const Row rows[kPfdSetupRowCount] = {
      {"PFD", PfdSetupRow::Pfd, PfdSetupField::PfdTarget, PfdSetupField::PfdMode,
       PfdSetupField::PfdValue},
      {"MFD", PfdSetupRow::Mfd, PfdSetupField::MfdTarget, PfdSetupField::MfdMode,
       PfdSetupField::MfdValue},
  };

  // The right-pointing carrot sits in a fixed column so both rows' carrots line
  // up vertically (as on the real unit) regardless of each label's width. Anchor
  // it just past the widest label among the rows.
  float maxLabelW = 0.0f;
  for (const Row& row : rows) {
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string t =
        std::string(row.name) + (isKey ? " Key" : " Display");
    maxLabelW = std::max(maxLabelW, r.measureTextWidth(t, size, kMenuFace));
  }
  const float rightArrowX = labelX + maxLabelW + size * 0.30f;

  for (int i = 0; i < kPfdSetupRowCount; ++i) {
    const Row& row = rows[i];
    const float cy = rowCy[i];
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string targetText =
        std::string(row.name) + (isKey ? " Key" : " Display");

    // Green arrowhead shows where the small knob can still toggle: right toward
    // Key when on Display, left back toward Display when on Key.
    arrow(labelX - size * 0.55f, cy, /*pointRight=*/false,
          isKey ? colors::kActiveGreen : colors::kLabelText);
    putField(r, labelX, cy, targetText, size, colors::kCyan,
             cursor == row.target, a, blinkOn, 0.0f, kMenuFace);
    arrow(rightArrowX, cy, /*pointRight=*/true,
          isKey ? colors::kLabelText : colors::kActiveGreen);

    // Mode (Auto / Manual).
    const std::string modeText =
        ui.pfdSetupMode(row.row) == BacklightMode::Manual ? "Manual" : "Auto";
    putField(r, modeX, cy, modeText, size, colors::kCyan, cursor == row.mode, a,
             blinkOn, 0.0f, kMenuFace);

    // Intensity percentage, right-aligned.
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f%%", ui.pfdSetupIntensityPct(row.row));
    const std::string valueText(buf);
    const float vx = valueRight - r.measureTextWidth(valueText, size, kMenuFace);
    putField(r, vx, cy, valueText, size, colors::kWhite, cursor == row.value, a,
             blinkOn, 0.0f, kMenuFace);
  }
}

// Always-on Crew Alerting System annunciation window on the main PFD. Per the
// G1000 Pilot's Guide (Fig. A-1) it sits to the right of the Vertical Speed
// Indicator at mid-display height, in a bordered box, with left-aligned text
// colored by severity (warning red, caution amber, advisory white). Higher
// priority messages are at the top; the list grows downward.
void drawCasAnnunciations(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
  const auto& msgs = ui.annunciations();
  if (msgs.empty()) return;

  const float msgSize = fontPx(wt::kInfoLabel, h);
  const float lineH = msgSize * 1.55f;
  const float padX = msgSize * 0.35f;
  const float padY = msgSize * 0.30f;

  // Clip the number of rows to what fits between the fixed top and the info
  // panel (matches the G1000's ~14-message capacity at this scale).
  const float maxBottom = L.infoPanelTop - lineH * 0.2f;
  const float boxW = L.casAnnunW;
  const float boxH = std::min(maxBottom - L.casAnnunTop,
                              padY * 2.0f + lineH * static_cast<float>(msgs.size()));

  // Faint plate + thin border, as the annunciation window is drawn in Fig. A-1.
  r.fillRect(L.casAnnunLeft, L.casAnnunTop, boxW, boxH,
             withAlpha(colors::kBlack, 0.55f));
  const Point border[5] = {{L.casAnnunLeft, L.casAnnunTop},
                           {L.casAnnunLeft + boxW, L.casAnnunTop},
                           {L.casAnnunLeft + boxW, L.casAnnunTop + boxH},
                           {L.casAnnunLeft, L.casAnnunTop + boxH},
                           {L.casAnnunLeft, L.casAnnunTop}};
  r.strokePolyline(border, 5, 1.5f, colors::kPanelBorder);

  const float textX = L.casAnnunLeft + padX;
  float y = L.casAnnunTop + padY + lineH * 0.5f;
  for (const AlertMessage& m : msgs) {
    if (y > L.casAnnunTop + boxH - lineH * 0.3f) break;
    const Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
                    : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                     : colors::kWhite;
    r.fillText(textX, y, m.text, msgSize, TextAlign::Left, c);
    y += lineH;
  }
}

// Draws the Direct-To identifier entry cells (shared look with the MFD): while
// the entry is active the cursor cell pulses as a cyan highlight-select plate
// with black text, the spell-ahead fill is cyan, and the typed characters are
// white. Returns the x just past the last cell (for the waypoint symbol).
float drawDtoIdentCells(Renderer& r, float startX, float cy,
                        const std::string& ident, int cursor, int typedCount,
                        bool blinkOn, float size) {
  const float tracking = size * 0.06f;
  float cx = startX;
  for (int i = 0; i < FmsWaypointEntry::kMaxChars; ++i) {
    const bool isCursor = i == cursor;
    const bool cursorOn = isCursor && blinkOn;
    const char ch = i < static_cast<int>(ident.size()) ? ident[i] : '_';
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, size);
    if (cursorOn) {
      r.fillRect(cx - tracking * 0.5f, cy - size * 0.62f, chW + tracking,
                 size * 1.24f, colors::kCyan);
    }
    const Color color = cursorOn          ? colors::kBlack
                        : isCursor        ? colors::kCyan  // blink-off pulse
                        : i >= typedCount ? colors::kCyan  // spell-ahead fill
                                          : colors::kWhite;
    r.fillText(cx, cy, text, size, TextAlign::Left, color);
    cx += chW + tracking;
  }
  return cx;
}

// Thin grey rectangular outline for the Direct-To group boxes (Fig. 5-45).
void drawDtoBox(Renderer& r, float x, float y, float w, float h) {
  const Point box[5] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}};
  r.strokePolyline(box, 5, 1.0f, colors::kGroupBoxBorder);
}

// One text-sized rounded-rect command button (Activate? / Hold?). When armed
// (highlight-select) the button pulses ~1 Hz: cyan fill with black text, then
// cyan text on black (WT highlight-select / Pilot's Guide Fig. 5-45).
float drawDtoButton(Renderer& r, float leftX, float cy, const char* label,
                    float size, bool armed, bool blinkOn) {
  const float bw = r.measureTextWidth(label, size) + size * 1.3f;
  const float bh = size * 1.7f;
  const float bx = leftX;
  const float by = cy - bh * 0.5f;
  const float radius = bh * 0.32f;
  if (armed && blinkOn) {
    r.fillRoundedRect(bx, by, bw, bh, radius, colors::kCyan);
  }
  r.strokeRoundedRect(bx, by, bw, bh, radius, 1.5f,
                      armed ? colors::kWhite : colors::kGroupBoxBorder);
  const Color textColor =
      armed ? (blinkOn ? colors::kBlack : colors::kCyan) : colors::kWhite;
  r.fillText(leftX + bw * 0.5f, cy, label, size, TextAlign::Center, textColor);
  return bw;
}

// PFD Direct-To window (Direct-To bezel key, Pilot's Guide Fig. 5-45 "Direct-to
// Window - PFD"): a lower-right popout with the shared window chrome whose body
// stacks the Ident/Facility/City box, an ALT/Offset (VNV) box, a BRG/DIS
// (Location) box, the CRS desired course, and the Activate? / Hold? buttons.
void drawDirectToWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // Pilot's Guide Fig. 5-45: the PFD Direct-To window is wider than it is tall
  // (~1.35 aspect on the 4:3 glass), sized like the other lower-right popouts.
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.directToWindowAnim(), "Direct To",
                      w * 0.36f, h * 0.36f);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  r.save();
  r.globalAlpha(a);

  const bool hasMatch = ui.directToHasMatch();
  const bool entryActive = ui.directToEntryActive();
  const bool blinkOn = ui.blinkOn();

  const float identSize = fontPx(wt::kInfoValue, h);  // prominent ident
  const float faceSize = fontPx(wt::kInfoLabel, h);   // facility / city
  const float labelSize = fontPx(wt::kInfoLabel, h);  // grey field labels
  const float valueSize = fontPx(wt::kInfoValue, h);
  const float readoutSize = fontPx(wt::kInfoValue, h);  // BRG/DIS/CRS numbers
  const float smallSize = labelSize * 0.72f;           // FT/NM unit suffixes

  const float pad = f.w * 0.045f;
  const float left = f.x + pad;
  const float right = f.x + f.w - pad;
  const float innerW = right - left;
  const float gap = fontPx(8.0f, h);

  // Buttons anchored to the bottom of the body.
  const float buttonH = valueSize * 1.7f;
  const float buttonsCy = f.top + f.h - buttonH * 0.5f - fontPx(8.0f, h);

  float y = f.contentTop + fontPx(4.0f, h);

  // ---- Ident / Facility / City box ----
  const float identBoxH = identSize + faceSize * 1.3f + fontPx(18.0f, h);
  drawDtoBox(r, left, y, innerW, identBoxH);
  {
    const float ix = left + pad * 0.5f;
    const float cy1 = y + fontPx(8.0f, h) + identSize * 0.5f;
    const bool armed = ui.directToArmed();
    float identEnd = ix;
    if (armed && hasMatch) {
      // Confirmed waypoint: full ident on a cyan plate (Fig. 5-45).
      const std::string& id = ui.directToIdent();
      const float tracking = identSize * 0.06f;
      const float tw = r.measureTextWidth(id.c_str(), identSize);
      r.fillRect(ix - tracking * 0.5f, cy1 - identSize * 0.62f, tw + tracking,
                 identSize * 1.24f, colors::kCyan);
      r.fillText(ix, cy1, id.c_str(), identSize, TextAlign::Left,
                 colors::kBlack);
      identEnd = ix + tw + tracking;
    } else {
      const int cursor = entryActive ? ui.directToCursor() : -1;
      identEnd = drawDtoIdentCells(r, ix, cy1, ui.directToIdent(), cursor,
                                   ui.directToTypedCount(), blinkOn, identSize);
    }
    if (hasMatch) {
      const MapFeature& wpt = ui.directToMatch();
      const float symR = fontPx(15.0f, h) * 0.6f;
      const float symCx = identEnd + fontPx(16.0f, h);
      drawMapFeatureSymbol(r, wpt, symCx, cy1, symR);
      // The location (city / region) reads from just right of the symbol, then
      // the facility name on the line below (Pilot's Guide Fig. 5-45).
      std::string loc = wpt.city;
      if (!wpt.region.empty()) loc += loc.empty() ? wpt.region : " " + wpt.region;
      if (!loc.empty()) {
        r.fillText(symCx + symR + fontPx(12.0f, h), cy1, loc, faceSize,
                   TextAlign::Left, colors::kCyan);
      }
    }
    const float cy2 = cy1 + identSize * 0.55f + faceSize * 0.7f;
    if (ui.directToNotFound()) {
      r.fillText(ix, cy2, "WAYPOINT NOT FOUND", faceSize, TextAlign::Left,
                 colors::kBandYellow);
    } else if (hasMatch) {
      const MapFeature& wpt = ui.directToMatch();
      if (!wpt.name.empty()) {
        r.fillText(ix, cy2, wpt.name, faceSize, TextAlign::Left, colors::kCyan);
      }
    }
  }
  y += identBoxH + gap;

  // ---- separator rule (matches the <hr> above the VNV row) ----
  r.strokeLine(left, y, right, y, 1.0f, colors::kWhitesmoke);
  y += gap;

  // ---- ALT / Offset (VNV constraints) box ----
  // No VNAV altitude/offset source in this suite, so the fields show the empty
  // dashes / +0 the real unit displays before a constraint is entered.
  {
    const float boxH = valueSize * 1.7f;
    drawDtoBox(r, left, y, innerW, boxH);
    const float cy = y + boxH * 0.5f;
    float x = left + pad * 0.5f;
    x = putText(r, x, cy, "ALT", labelSize, colors::kTitleGray, 0.4f);
    x = putText(r, x, cy, "_ _ _ _ _", valueSize, colors::kCyan, 0.1f);
    putText(r, x, cy, "FT", smallSize, colors::kCyan, 0.0f);
    float ox = left + innerW * 0.52f;
    ox = putText(r, ox, cy, "Offset", labelSize, colors::kTitleGray, 0.4f);
    r.fillText(right - pad * 0.5f, cy, "NM", smallSize, TextAlign::Right,
               colors::kCyan);
    r.fillText(right - pad * 0.5f - r.measureTextWidth("NM", smallSize) -
                   smallSize * 0.2f,
               cy, "+0", valueSize, TextAlign::Right, colors::kCyan);
    y += boxH + gap;
  }

  // ---- BRG / DIS (Location) box ----
  char buf[24];
  const bool hasGeo = ui.directToHasGeo();
  {
    const float boxH = readoutSize * 1.55f;
    drawDtoBox(r, left, y, innerW, boxH);
    const float cy = y + boxH * 0.5f;
    float x = left + pad * 0.5f;
    x = putText(r, x, cy, "BRG", labelSize, colors::kTitleGray, 0.35f);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0", ui.directToBearingDeg());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", "___\u00b0");
    }
    putText(r, x, cy, buf, readoutSize, colors::kWhite, 0.0f);
    float dx = left + innerW * 0.52f;
    dx = putText(r, dx, cy, "DIS", labelSize, colors::kTitleGray, 0.35f);
    r.fillText(right - pad * 0.5f, cy, "NM", smallSize, TextAlign::Right,
               colors::kWhite);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%.1f", ui.directToDistanceNm());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", "__._");
    }
    r.fillText(right - pad * 0.5f - r.measureTextWidth("NM", smallSize) -
                   smallSize * 0.2f,
               cy, buf, readoutSize, TextAlign::Right, colors::kWhite);
    y += boxH + gap;
  }

  // ---- CRS desired course (no box) ----
  {
    const float cy = y + readoutSize * 0.7f;
    float x = left + pad * 0.5f;
    x = putText(r, x, cy, "CRS", labelSize, colors::kTitleGray, 0.35f);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0", ui.directToBearingDeg());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", "___\u00b0");
    }
    putText(r, x, cy, buf, readoutSize, colors::kCyan, 0.0f);
  }

  // ---- Activate? / Hold? buttons ----
  drawDtoButton(r, left, buttonsCy, "Activate?", valueSize, ui.directToArmed(),
                blinkOn);
  const float holdW =
      r.measureTextWidth("Hold?", valueSize) + valueSize * 1.3f;
  drawDtoButton(r, right - holdW, buttonsCy, "Hold?", valueSize, false,
                blinkOn);

  r.restore();
}

}  // namespace

std::string navComDecodeIdent(const FlightData& d) {
  if (!d.dataLinkValid) return {};
  if (d.com1Transmitting && !d.com1Ident.empty()) return d.com1Ident;
  if (d.com2Transmitting && !d.com2Ident.empty()) return d.com2Ident;
  return {};
}

void drawComDecodePanel(Renderer& r, float h, float barH, float comLeft,
                        float comW, float cornerR, const std::string& ident) {
  if (ident.empty()) return;
  // A separate rounded panel below the COM box, with a small gap above it so the
  // sky shows through between the two -- it is not an extension of the COM box.
  const float gap = barH * 0.06f;
  const float panelH = barH * 0.40f;
  const float top = barH + gap;
  drawNavComPanelBg(r, comLeft, top, comW, panelH, cornerR);
  FontScope navComFont(r, FontFace::DejaVuSemiBold);
  // Horizontally centered in the black panel. The +size*0.10 descender
  // correction recenters the glyph ink, since the font centers its line box
  // (which includes an empty descender span) on the draw point.
  const float identCx = comLeft + comW * 0.5f;
  const float size = fontPx(wt::kNavComFreq, h) * 0.86f;
  r.fillText(identCx, top + panelH * 0.5f + size * 0.10f, ident, size,
             TextAlign::Center, colors::kActiveGreen);
}

void drawNavComFreqCells(Renderer& r, float h, float barH, float navLeft,
                         float navW, float comLeft, float comW,
                         const FlightData& d, const SoftkeyController& ui) {
  // Rows are nudged down from the symmetric 0.27/0.73 split by a small descender
  // correction: the font centers its line box (which includes the empty
  // descender span below the digits) on the draw point, so digits -- which have
  // no descenders -- otherwise sit visually high. The +0.04 recenters the glyph
  // ink within the panel while preserving the row-to-row spacing.
  const float row1Cy = barH * 0.31f;
  const float row2Cy = barH * 0.77f;
  const float freqSize = fontPx(wt::kNavComFreq, h);
  const float labelSize = fontPx(wt::kNavComLabel, h);

  FontScope navComFont(r, FontFace::DejaVuSemiBold);

  const bool linkValid = d.dataLinkValid;
  const Color standbyColor = linkValid ? colors::kWhite : colors::kBandYellow;
  auto freqText = [&](float mhz, int decimals) {
    return linkValid ? formatFreq(mhz, decimals)
                     : std::string(decimals >= 3 ? kFreqDash3 : kFreqDash2);
  };
  auto navActiveColor = [&](RadioUnit unit) -> Color {
    if (!linkValid) return colors::kBandYellow;
    const bool isCdi =
        (d.cdiSource == CdiSource::Nav1 && unit == RadioUnit::Nav1) ||
        (d.cdiSource == CdiSource::Nav2 && unit == RadioUnit::Nav2);
    return isCdi ? colors::kActiveGreen : colors::kWhite;
  };
  auto comActiveColor = [&](RadioUnit unit) -> Color {
    if (!linkValid) return colors::kBandYellow;
    const bool xmit = unit == RadioUnit::Com1 ? d.com1Transmitting
                                              : d.com2Transmitting;
    return xmit ? colors::kActiveGreen : colors::kWhite;
  };

  auto boxed = [&](RadioUnit unit, RadioUnit selected, RadioBand band) {
    if (selected != unit) return false;
    const bool flashing = ui.radioArmedBand() == band && ui.radioArmed();
    return !flashing || ui.blinkOn();
  };

  // Standby tuning cursor box: a thin cyan rectangle with even margin on all
  // sides. The box width hugs the digits; the height is fixed compact (WT
  // NavComFrequencyElement.css 24 px on the 768 canvas). It is centered on the
  // digits' visual center, which sits above the font line-middle (digits have
  // no descenders), so the box is derived from the measured glyph bounds.
  const float boxPadX = fontPx(4.0f, h);
  // Keep the box shorter than the gap between the two frequency rows so it can
  // never bleed into the row below it.
  const float boxH = (row2Cy - row1Cy) * 0.90f;
  const float caretHalf = freqSize * 0.42f;

  // Standby frequency with the thin cyan tuning-cursor box hugging the text.
  // `rightX` is the right edge the (right-aligned) frequency is anchored to.
  // Returns the box's left and right edges for caret placement.
  auto drawStandbyBoxed = [&](float rightX, float cy, const std::string& text,
                              bool selected, float& outBoxLeft,
                              float& outBoxRight) {
    const TextRect tb =
        r.measureTextRect(rightX, cy, text, freqSize, TextAlign::Right);
    outBoxLeft = tb.left - boxPadX;
    outBoxRight = rightX + boxPadX;
    if (selected) {
      // Center the box on the glyph ink (cap line to baseline), ignoring the
      // empty descender span the font metrics include below the digits.
      const float glyphCy = (tb.top + tb.bottom - freqSize * 0.18f) * 0.5f;
      r.strokeRoundedRect(outBoxLeft, glyphCy - boxH * 0.5f,
                          outBoxRight - outBoxLeft, boxH, 0.0f, 1.5f,
                          colors::kCyan);
    }
    r.fillText(rightX, cy, text, freqSize, TextAlign::Right, standbyColor);
  };

  // Audio volume indication (Pilot's Guide Fig. 4-3 / 4-8): while a VOL/SQ or
  // VOL/ID knob is turned, the level replaces the selected radio's standby
  // frequency for two seconds -- a cyan percentage toward the active frequency
  // and a smaller white "VOL" label toward the band label. The percent and
  // label swap sides between the two boxes because the standby field sits on the
  // right of the COM box but on the left of the NAV box.
  const float volLabelSize = freqSize * 0.62f;
  const float volGap = freqSize * 0.18f;
  // NAV box: "VOL" (white) to the left of the cyan percentage, the group
  // right-aligned where the NAV standby frequency normally sits.
  auto drawNavVolume = [&](float rightX, float cy, int pct) {
    const std::string p = std::to_string(pct) + "%";
    r.fillText(rightX, cy, p, freqSize, TextAlign::Right, colors::kCyan);
    const float pctLeft = rightX - r.measureTextWidth(p, freqSize);
    r.fillText(pctLeft - volGap, cy, "VOL", volLabelSize, TextAlign::Right,
               colors::kWhite);
  };
  // COM box: cyan percentage to the left of the white "VOL" label, the group
  // right-aligned where the COM standby frequency normally sits.
  auto drawComVolume = [&](float rightX, float cy, int pct) {
    r.fillText(rightX, cy, "VOL", volLabelSize, TextAlign::Right, colors::kWhite);
    const float volLeft = rightX - r.measureTextWidth("VOL", volLabelSize);
    const std::string p = std::to_string(pct) + "%";
    r.fillText(volLeft - volGap, cy, p, freqSize, TextAlign::Right, colors::kCyan);
  };

  // NAV side (mirror of COM): 'NAV' + 1/2 labels at the far left, then the
  // boxed standby frequency, the transfer carets, the active frequency, and
  // the station ident at the panel's right edge.
  drawBandLabel(r, "NAV", navLeft + navW * 0.035f, navLeft + navW * 0.10f,
                barH, row1Cy, row2Cy, labelSize);
  const float navStandbyRightX = navLeft + navW * 0.415f;
  const float navActiveLeftX = navLeft + navW * 0.505f;
  // Left edge of the active station's decoded Morse ident slot (up to 3 chars,
  // e.g. "PSP"); sits a clear gap to the right of the active frequency.
  const float navIdentLeftX = navLeft + navW * 0.795f;
  const float identGap = freqSize * 0.55f;
  struct NavRow {
    float active, standby, cy;
    const std::string& ident;
  };
  const NavRow navs[2] = {
      {d.nav1ActiveMhz, d.nav1StandbyMhz, row1Cy, d.nav1Ident},
      {d.nav2ActiveMhz, d.nav2StandbyMhz, row2Cy, d.nav2Ident}};
  for (int i = 0; i < 2; ++i) {
    const NavRow& n = navs[i];
    const RadioUnit unit = i == 0 ? RadioUnit::Nav1 : RadioUnit::Nav2;
    const bool sel = boxed(unit, ui.navSelected(), RadioBand::Nav);
    const int dec = 2;
    const float anim = ui.radioTransferAnim(unit);
    std::string activeStr = freqText(n.active, dec);
    std::string standbyStr = freqText(n.standby, dec);
    float standbyRX = navStandbyRightX;
    float activeLX = navActiveLeftX;
    if (anim > 0.0f && anim < 1.0f) {
      const float t = smoothstep(anim);
      activeStr = freqText(ui.radioTransferFromActive(unit), dec);
      standbyStr = freqText(ui.radioTransferFromStandby(unit), dec);
      const float twActive = r.measureTextWidth(activeStr, freqSize);
      const float standbyLeft0 =
          navStandbyRightX - r.measureTextWidth(standbyStr, freqSize);
      standbyRX =
          navStandbyRightX + (navActiveLeftX + twActive - navStandbyRightX) * t;
      activeLX = navActiveLeftX + (standbyLeft0 - navActiveLeftX) * t;
    }
    const bool showVol = anim <= 0.0f &&
                         ui.radioVolumeShown(RadioBand::Nav) &&
                         ui.radioVolumeUnit() == unit;
    float boxLeft = 0.0f, boxRight = 0.0f;
    if (showVol) {
      drawNavVolume(navStandbyRightX, n.cy, ui.radioVolumePct());
    } else {
      drawStandbyBoxed(standbyRX, n.cy, standbyStr, sel, boxLeft, boxRight);
    }
    r.fillText(activeLX, n.cy, activeStr, freqSize, TextAlign::Left,
               navActiveColor(unit));
    if (!n.ident.empty()) {
      const float activeRight =
          activeLX + r.measureTextWidth(activeStr, freqSize);
      const float identX = std::max(navIdentLeftX, activeRight + identGap);
      r.fillText(identX, n.cy, n.ident, freqSize, TextAlign::Left,
                 navActiveColor(unit));
    }
    // Middle slot: a white "ID" replaces the transfer arrow while Morse ident
    // audio is on for this NAV (Pilot's Guide Fig. 4-8), otherwise the carets.
    if (sel && anim <= 0.0f && !showVol) {
      const float midX = (boxRight + activeLX) * 0.5f;
      if (d.*navIdentAudioMember(unit)) {
        r.fillText(midX, n.cy, "ID", volLabelSize, TextAlign::Center,
                   colors::kWhite);
      } else {
        drawTransferCarets(r, midX, n.cy, caretHalf);
      }
    }
  }

  // COM side: active frequency left-aligned inside the panel (so it never
  // spills over the center box), the carets, the boxed standby, then the 1/2
  // and 'COM' labels packed at the far right.
  // Both COM frequencies are shifted toward the 1/2 channel labels on the
  // right, leaving a clear gap at the panel's left edge for a future RX/TX
  // transmit/receive annunciation beside the active frequency.
  const float comActiveLeftX = comLeft + comW * 0.10f;
  const float comStandbyRightX = comLeft + comW * 0.83f;
  struct RadioRow {
    float active, standby, cy;
  };
  const RadioRow coms[2] = {{d.com1ActiveMhz, d.com1StandbyMhz, row1Cy},
                            {d.com2ActiveMhz, d.com2StandbyMhz, row2Cy}};
  for (int i = 0; i < 2; ++i) {
    const RadioRow& c = coms[i];
    const RadioUnit unit = i == 0 ? RadioUnit::Com1 : RadioUnit::Com2;
    const bool sel = boxed(unit, ui.comSelected(), RadioBand::Com);
    const int dec = 3;
    const float anim = ui.radioTransferAnim(unit);
    std::string activeStr = freqText(c.active, dec);
    std::string standbyStr = freqText(c.standby, dec);
    float activeLX = comActiveLeftX;
    float standbyRX = comStandbyRightX;
    if (anim > 0.0f && anim < 1.0f) {
      const float t = smoothstep(anim);
      activeStr = freqText(ui.radioTransferFromActive(unit), dec);
      standbyStr = freqText(ui.radioTransferFromStandby(unit), dec);
      const float twActive = r.measureTextWidth(activeStr, freqSize);
      const float standbyLeft0 =
          comStandbyRightX - r.measureTextWidth(standbyStr, freqSize);
      activeLX = comActiveLeftX + (standbyLeft0 - comActiveLeftX) * t;
      standbyRX =
          comStandbyRightX + (comActiveLeftX + twActive - comStandbyRightX) * t;
    }
    r.fillText(activeLX, c.cy, activeStr, freqSize, TextAlign::Left,
               comActiveColor(unit));
    const bool showVol = anim <= 0.0f &&
                         ui.radioVolumeShown(RadioBand::Com) &&
                         ui.radioVolumeUnit() == unit;
    float boxLeft = 0.0f, boxRight = 0.0f;
    if (showVol) {
      drawComVolume(comStandbyRightX, c.cy, ui.radioVolumePct());
    } else {
      drawStandbyBoxed(standbyRX, c.cy, standbyStr, sel, boxLeft, boxRight);
    }
    if (sel && anim <= 0.0f && !showVol) {
      const float aRight =
          comActiveLeftX + r.measureTextWidth(activeStr, freqSize);
      // The transfer carets between the active and boxed standby frequency.
      // (The COM VOL/SQ press has no annunciation: X-Plane has no squelch
      // dataref, so automatic squelch is never modeled as disabled.)
      drawTransferCarets(r, (aRight + boxLeft) * 0.5f, c.cy, caretHalf);
    }
  }

  // The decoded COM station identifier is drawn by the caller in its own panel
  // below the COM box (drawComDecodePanel).

  drawBandLabel(r, "COM", comLeft + comW * 0.95f, comLeft + comW * 0.88f, barH,
                row1Cy, row2Cy, labelSize);
}

void drawChrome(Renderer& r, const Layout& L, const FlightData& d,
                const SoftkeyController& ui, float w, float h) {
  drawTopBar(r, w, h, L, d, ui);
  drawBottomInfoPanel(r, w, h, L, d, ui);
  // CAS annunciation window is always visible (when active) on the main PFD.
  drawCasAnnunciations(r, w, h, L, ui);
  // The pop-up windows (Alerts / References / Nearest Airports) share the
  // lower-right region -- one is active at a time, but each is drawn while its
  // animation is nonzero so a replaced window fades out under the new one.
  // They sit above the info panel but below the softkey bar, so the bar (and
  // its press highlights) always stay on top.
  drawAlertsWindow(r, w, h, L, ui);
  drawReferencesWindow(r, w, h, L, ui);
  drawNearestWindow(r, w, h, L, ui);
  drawPfdSetupWindow(r, w, h, L, ui);
  // The Direct-To window (Direct-To bezel key) shares the lower-right region.
  drawDirectToWindow(r, w, h, L, ui);
  drawSoftkeyBar(r, w, h, L, ui);
}

}  // namespace avionics::pfd
