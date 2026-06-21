#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

#include "avionics/NavMath.h"

namespace avionics::pfd {
namespace {

constexpr char kDeg[] = "\xC2\xB0";  // UTF-8 degree sign

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

// Failed NAV/COM frequency cells: a maroon fill spanning the two frequency rows
// with a red X over each row (G1000 NXi Maintenance Manual Fig 9-2, PFD
// Power-Up System Annunciations). Drawn in place of the frequencies/idents when
// the GIA datalink is invalid; the band labels remain.
void drawFailedRadioCells(Renderer& r, float x, float y, float cw, float ch,
                          float displayH) {
  r.save();
  r.clip(x, y, cw, ch);
  r.fillRect(x, y, cw, ch, colors::kFailedWindow);
  const float thick = std::max(1.0f, displayH * 0.0028f);
  const float ymid = y + ch * 0.5f;
  // One X per frequency row (1/2), each spanning the full cell width.
  r.strokeLine(x, y, x + cw, ymid, thick, colors::kFailedX);
  r.strokeLine(x, ymid, x + cw, y, thick, colors::kFailedX);
  r.strokeLine(x, ymid, x + cw, y + ch, thick, colors::kFailedX);
  r.strokeLine(x, y + ch, x + cw, ymid, thick, colors::kFailedX);
  const Point frame[5] = {
      {x, y}, {x + cw, y}, {x + cw, y + ch}, {x, y + ch}, {x, y}};
  r.strokePolyline(frame, 5, 1.0f, colors::kPanelBorder);
  r.restore();
}

// Single failed frequency row: a maroon cell with one red X, used when an
// individual NAV/COM receiver has failed (its own X-Plane failure dataref is
// tripped) while the other radio in the box keeps working. The row spans the
// full frequency-cell width and is centered on the given row's center.
void drawFailedRadioRow(Renderer& r, float x, float rowCy, float cw, float rowH,
                        float displayH) {
  const float y = rowCy - rowH * 0.5f;
  r.save();
  r.clip(x, y, cw, rowH);
  r.fillRect(x, y, cw, rowH, colors::kFailedWindow);
  const float thick = std::max(1.0f, displayH * 0.0028f);
  r.strokeLine(x, y, x + cw, y + rowH, thick, colors::kFailedX);
  r.strokeLine(x, y + rowH, x + cw, y, thick, colors::kFailedX);
  const Point frame[5] = {
      {x, y}, {x + cw, y}, {x + cw, y + rowH}, {x, y + rowH}, {x, y}};
  r.strokePolyline(frame, 5, 1.0f, colors::kPanelBorder);
  r.restore();
}

// Garmin Direct-To icon (a "D" with a horizontal arrow piercing it) drawn to
// the left of the active waypoint when a GPS Direct-To is active, matching the
// look of the "D" bezel key. Returns the x just past the glyph.
// (Defined in ChromeShared.cpp.)

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

// Green VS reference arrow (up/down) beside the active vertical mode (WT FMA).
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
  // Grey DIS/BRG labels are smaller than their magenta values; only the NM
  // unit suffix is subscript-small.
  const float disBrgLabelSize = dataSize * 0.72f;

  // Row separator between the navigation and AFCS halves of the center panel.
  r.strokeLine(centerL, rowH, centerL + centerW, rowH, 2.0f,
               colors::kPanelBorder);

  // The active-leg field (FROM -> TO or GPS Direct-To) is shown only with a
  // valid datalink and an active leg; DIS/BRG below are always shown.
  const bool hasLeg = d.dataLinkValid && !d.fmaToWpt.empty();

  if (d.dataLinkValid) {
    // No FROM waypoint with an active TO means a GPS Direct-To: show the
    // Direct-To icon followed by the target identifier instead of a FROM -> TO
    // leg.
    const bool directTo = d.fmaFromWpt.empty() && !d.fmaToWpt.empty();

    float legWidth = 0.0f;
    if (directTo) {
      legWidth += dataSize * 0.95f;  // Direct-To icon + gap
      legWidth += r.measureTextWidth(d.fmaToWpt, dataSize);
    } else {
      if (!d.fmaFromWpt.empty()) {
        legWidth +=
            r.measureTextWidth(d.fmaFromWpt, dataSize) + dataSize * 0.12f;
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
  }

  // Vertical divider between the active-leg field and the DIS/BRG readouts,
  // always present (G1000 NXi Navigation Status Box, Fig 9-2), meeting the row
  // separator below.
  r.strokeLine(dataL, rowH * 0.18f, dataL, rowH, 1.5f, colors::kPanelBorder);

  // DIS readout left-aligned almost touching the divider; magenta dashes when
  // there is no active leg.
  std::string disStr = "__._";
  if (hasLeg) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", d.fmaLegDistanceNm);
    disStr = buf;
  }
  float bx = dataL + centerW * 0.01f;
  bx = putText(r, bx, cy, "DIS", disBrgLabelSize, colors::kLabelText, 0.30f);
  bx = putText(r, bx, cy, disStr, dataSize, colors::kMagenta, 0.18f);
  putText(r, bx, cy, "NM", smallSize, colors::kMagenta, 0.0f);

  // BRG readout right-aligned to the panel edge: the real unit spreads DIS and
  // BRG to opposite ends of the data field, with the bearing value hugging the
  // right edge and its grey label just to the left; magenta dashes when there
  // is no active leg.
  const std::string brgVal =
      hasLeg ? formatHeading(d.fmaLegBearingDeg) : std::string("___");
  const float rightX = centerL + centerW - centerW * 0.025f;
  const float degW = r.measureTextWidth(kDeg, smallSize);
  r.fillText(rightX, cy, kDeg, smallSize, TextAlign::Right, colors::kMagenta);
  r.fillText(rightX - degW, cy, brgVal, dataSize, TextAlign::Right,
             colors::kMagenta);
  const float brgLabelRight =
      rightX - degW - r.measureTextWidth(brgVal, dataSize) -
      disBrgLabelSize * 0.30f;
  r.fillText(brgLabelRight, cy, "BRG", disBrgLabelSize, TextAlign::Right,
             colors::kLabelText);
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
  // Armed vertical annunciations (ALTS, GS, ...) are a touch smaller than the
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

}  // namespace

void drawNavComPanelBg(Renderer& r, float x, float y, float pw, float ph,
                       float radius, NavComPanelShape shape) {
  float radTL = 0.0f;
  float radTR = 0.0f;
  float radBR = 0.0f;
  float radBL = 0.0f;
  switch (shape) {
    case NavComPanelShape::Floating:
      radTL = radTR = radBR = radBL = radius;
      break;
    case NavComPanelShape::MfdLeft:
      radBR = radius;
      break;
    case NavComPanelShape::MfdCenter:
      radBR = radBL = radius;
      break;
    case NavComPanelShape::MfdRight:
      radBL = radius;
      break;
  }
  // Vertical gradient (trainer-sampled NavCom panel fill).
  r.fillRoundedRectVaryingVerticalGradient(
      x, y, pw, ph, radTL, radTR, radBR, radBL, colors::kPanelBackground,
      colors::kPanelBackgroundBottom);
  // PFD boxes float with a visible outline; the MFD panels are flush/touching
  // and the trainer shows no vertical divider lines between them.
  if (shape == NavComPanelShape::Floating) {
    r.strokeRoundedRectVarying(x, y, pw, ph, radTL, radTR, radBR, radBL, 1.0f,
                               colors::kPanelBorder);
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
  const float top = barH + gap - fontPx(kComDecodePanelUpPx, h);
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

  // GIA datalink failure (e.g. PFD power-up): the NAV and COM frequency cells
  // are red-X'd over a maroon fill and no frequencies or station idents are
  // shown; only the band labels remain (NXi Maintenance Manual Fig 9-2).
  if (!d.dataLinkValid) {
    drawBandLabel(r, "NAV", navLeft + navW * 0.035f, navLeft + navW * 0.10f,
                  barH, row1Cy, row2Cy, labelSize);
    drawBandLabel(r, "COM", comLeft + comW * 0.95f, comLeft + comW * 0.88f, barH,
                  row1Cy, row2Cy, labelSize);
    const float cellTop = barH * 0.12f;
    const float cellH = barH * 0.76f;
    drawFailedRadioCells(r, navLeft + navW * 0.155f, cellTop, navW * 0.83f,
                         cellH, h);
    drawFailedRadioCells(r, comLeft + comW * 0.04f, cellTop, comW * 0.78f, cellH,
                         h);
    return;
  }

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
    // This receiver has failed on its own: red-X just its row, leaving the
    // other NAV row live (its failure dataref is independent).
    if (!(i == 0 ? d.nav1Valid : d.nav2Valid)) {
      drawFailedRadioRow(r, navLeft + navW * 0.155f, n.cy, navW * 0.83f,
                         barH * 0.38f, h);
      continue;
    }
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
    // This transceiver has failed on its own: red-X just its row.
    if (!(i == 0 ? d.com1Valid : d.com2Valid)) {
      drawFailedRadioRow(r, comLeft + comW * 0.04f, c.cy, comW * 0.78f,
                         barH * 0.38f, h);
      continue;
    }
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

}  // namespace avionics::pfd
