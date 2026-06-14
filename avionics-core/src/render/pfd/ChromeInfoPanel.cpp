#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {
namespace {

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

}  // namespace

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

}  // namespace avionics::pfd
