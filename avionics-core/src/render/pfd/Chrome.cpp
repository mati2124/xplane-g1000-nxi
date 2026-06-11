#include <algorithm>
#include <cstdio>

#include "render/pfd/PfdInternal.h"

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

void drawTransferArrow(Renderer& r, float cx, float cy, float halfW) {
  const float a = halfW * 0.55f;
  r.strokeLine(cx - halfW, cy, cx + halfW, cy, 1.5f, colors::kCyan);
  r.strokeLine(cx - halfW, cy, cx - halfW + a, cy - a, 1.5f, colors::kCyan);
  r.strokeLine(cx - halfW, cy, cx - halfW + a, cy + a, 1.5f, colors::kCyan);
  r.strokeLine(cx + halfW, cy, cx + halfW - a, cy - a, 1.5f, colors::kCyan);
  r.strokeLine(cx + halfW, cy, cx + halfW - a, cy + a, 1.5f, colors::kCyan);
}

// AFCS Status Box: the autopilot / flight-director mode annunciations in their
// own single-row box at the top center of the PFD (G1000 NXi Pilot's Guide,
// AFCS). Lateral modes (armed white, active green) on the left, AP/YD in the
// center, vertical modes (active green, reference cyan, armed white) on the
// right. This is separate from the Navigation Status Box below.
void drawAfcsStatusBox(Renderer& r, float w, float h, float barH,
                       const FlightData& d) {
  const float left = w * 0.255f;
  const float right = w * 0.745f;
  const float centerW = right - left;
  const float pad = centerW * 0.015f;
  const float cy = barH * 0.5f;
  const float smallSize = fontPx(wt::kFmaSmall, h);
  const float modeSize = fontPx(wt::kFmaActive, h);
  const float armedSize = fontPx(wt::kFmaArmed, h);

  const float latW = centerW * 0.40f;
  const float apW = centerW * 0.18f;
  const float vertX = left + latW + apW;

  // Static cell separators keep the box's shape even when the link is down.
  r.strokeLine(left + latW, barH * 0.12f, left + latW, barH * 0.88f, 1.5f,
               colors::kPanelSeparator);
  r.strokeLine(vertX, barH * 0.12f, vertX, barH * 0.88f, 1.5f,
               colors::kPanelSeparator);

  if (!d.dataLinkValid) return;

  float lx = left + pad;
  if (!d.fmaLateralArmed.empty()) {
    lx = putText(r, lx, cy, d.fmaLateralArmed, armedSize, colors::kWhite, 0.40f);
  }
  if (!d.fmaLateralActive.empty()) {
    putText(r, lx, cy, d.fmaLateralActive, modeSize, colors::kActiveGreen);
  }

  {
    std::string apYd;
    if (d.apEngaged) apYd += "AP";
    if (d.ydEngaged) apYd += (apYd.empty() ? "YD" : "  YD");
    if (!apYd.empty()) {
      r.fillText(left + latW + apW * 0.5f, cy, apYd, modeSize,
                 TextAlign::Center, colors::kActiveGreen);
    }
  }

  float vx = vertX + pad;
  if (!d.fmaVerticalActive.empty()) {
    vx = putText(r, vx, cy, d.fmaVerticalActive, modeSize, colors::kActiveGreen,
                 0.35f);
  }
  if (d.fmaVerticalValue != 0 && !d.fmaVerticalUnits.empty()) {
    // The vertical mode reference (e.g. the captured altitude) is cyan.
    vx = putText(r, vx, cy, formatInt(static_cast<float>(d.fmaVerticalValue)),
                 armedSize, colors::kCyan, 0.20f);
    putText(r, vx, cy, d.fmaVerticalUnits, smallSize, colors::kCyan);
  }

  float rx = right - pad;
  if (!d.fmaVerticalApproachArmed.empty()) {
    r.fillText(rx, cy, d.fmaVerticalApproachArmed, armedSize, TextAlign::Right,
               colors::kWhite);
    rx -= r.measureTextWidth(d.fmaVerticalApproachArmed, armedSize) +
          armedSize * 0.40f;
  }
  if (!d.fmaVerticalArmed.empty()) {
    r.fillText(rx, cy, d.fmaVerticalArmed, armedSize, TextAlign::Right,
               colors::kWhite);
  }
}

// PFD Navigation Status Box: a strip just below the top bar / AFCS box showing
// the active flight-plan leg (FROM -> TO with a magenta leg arrow) and the
// distance/bearing to the next waypoint (G1000 NXi Pilot's Guide, Flight
// Management).
void drawNavStatusBox(Renderer& r, float w, float h, const Layout& L,
                      const FlightData& d) {
  const float left = w * 0.255f;
  const float right = w * 0.745f;
  const float top = L.navStatusTop;
  const float boxH = L.navStatusH;
  const float cy = top + boxH * 0.5f;

  // Translucent backing so the white/magenta text reads over the attitude.
  r.fillRect(left, top, right - left, boxH, withAlpha(colors::kBlack, 0.45f));

  if (!d.dataLinkValid) return;

  const float dataSize = fontPx(wt::kFmaArmed, h);
  const float smallSize = fontPx(wt::kFmaSmall, h);
  const float pad = (right - left) * 0.02f;

  float x = left + pad;
  if (!d.fmaFromWpt.empty()) {
    x = putText(r, x, cy, d.fmaFromWpt, dataSize, colors::kWhite, 0.30f);
  }
  if (!d.fmaToWpt.empty()) {
    x = putText(r, x, cy, "\u2192", dataSize, colors::kMagenta, 0.30f);
    putText(r, x, cy, d.fmaToWpt, dataSize, colors::kWhite);
  }

  // DIS/BRG block occupies the right half of the box. With no active leg there
  // is nothing to measure to, so the block is left blank (NXi).
  if (!d.fmaToWpt.empty()) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", d.fmaLegDistanceNm);
    float bx = left + (right - left) * 0.50f;
    bx = putText(r, bx, cy, "DIS", smallSize, colors::kLabelText, 0.35f);
    bx = putText(r, bx, cy, std::string(buf), dataSize, colors::kWhite, 0.20f);
    bx = putText(r, bx, cy, "NM", smallSize, colors::kLabelText, 0.80f);
    bx = putText(r, bx, cy, "BRG", smallSize, colors::kLabelText, 0.35f);
    putText(r, bx, cy, formatHeading(d.fmaLegBearingDeg) + "\u00b0", dataSize,
            colors::kMagenta);
  }
}

void drawTopBar(Renderer& r, float w, float h, const Layout& L,
                const FlightData& d, const SoftkeyController& ui) {
  const float barH = L.topBarH;
  // NavComBox dark blue-grey vertical gradient rgb(4,4,12) -> rgb(24,28,43).
  r.fillRectVerticalGradient(0.0f, 0.0f, w, barH, 0.0f, barH,
                             colors::kPanelBackground,
                             colors::kPanelBackgroundBottom);
  r.strokeLine(0.0f, barH, w, barH, 2.0f, colors::kPanelBorder);

  const float row1Cy = barH * 0.27f;
  const float row2Cy = barH * 0.73f;
  const float freqSize = fontPx(wt::kNavComFreq, h);
  const float labelSize = fontPx(wt::kNavComLabel, h);

  const float divL = w * 0.255f;
  const float divR = w * 0.745f;
  r.strokeLine(divL, barH * 0.12f, divL, barH * 0.88f, 2.0f,
               colors::kPanelSeparator);
  r.strokeLine(divR, barH * 0.12f, divR, barH * 0.88f, 2.0f,
               colors::kPanelSeparator);

  // With the link down the tuned frequencies are unknown: show amber dashes in
  // place of the active/standby readouts (the labels and transfer arrows stay).
  const bool linkValid = d.dataLinkValid;
  const Color activeColor = linkValid ? colors::kActiveGreen : colors::kBandYellow;
  const Color standbyColor = linkValid ? colors::kWhite : colors::kBandYellow;
  auto freqText = [&](float mhz, int decimals) {
    return linkValid ? formatFreq(mhz, decimals)
                     : std::string(decimals >= 3 ? kFreqDash3 : kFreqDash2);
  };

  struct Nav {
    const char* label;
    float active, standby, cy;
  };
  const Nav navs[2] = {{"NAV1", d.nav1ActiveMhz, d.nav1StandbyMhz, row1Cy},
                       {"NAV2", d.nav2ActiveMhz, d.nav2StandbyMhz, row2Cy}};
  auto drawStandby = [&](float x, float cy, const std::string& text,
                         bool selected) {
    if (selected) {
      const float tw = r.measureTextWidth(text, freqSize);
      r.fillRect(x - tw - freqSize * 0.15f, cy - freqSize * 0.62f,
                 tw + freqSize * 0.3f, freqSize * 1.24f, colors::kCyan);
      r.fillText(x, cy, text, freqSize, TextAlign::Right, colors::kBlack);
    } else {
      r.fillText(x, cy, text, freqSize, TextAlign::Right, standbyColor);
    }
  };

  // The selected COM/NAV standby is boxed in cyan; the box flashes (with the
  // shared ~1 Hz blink phase) for a few seconds after a tuning action, then
  // settles solid, matching the real unit's armed tuning cursor.
  auto boxed = [&](RadioUnit unit, RadioUnit selected, RadioBand band) {
    if (selected != unit) return false;
    const bool flashing = ui.radioArmedBand() == band && ui.radioArmed();
    return !flashing || ui.blinkOn();
  };

  for (int i = 0; i < 2; ++i) {
    const Nav& n = navs[i];
    const RadioUnit unit = i == 0 ? RadioUnit::Nav1 : RadioUnit::Nav2;
    r.fillText(w * 0.012f, n.cy, n.label, labelSize, TextAlign::Left,
               colors::kLabelText);
    drawStandby(w * 0.140f, n.cy, freqText(n.standby, 2),
                boxed(unit, ui.navSelected(), RadioBand::Nav));
    drawTransferArrow(r, w * 0.158f, n.cy, w * 0.011f);
    r.fillText(w * 0.245f, n.cy, freqText(n.active, 2), freqSize,
               TextAlign::Right, activeColor);
  }

  const Nav coms[2] = {{"COM1", d.com1ActiveMhz, d.com1StandbyMhz, row1Cy},
                       {"COM2", d.com2ActiveMhz, d.com2StandbyMhz, row2Cy}};
  for (int i = 0; i < 2; ++i) {
    const Nav& c = coms[i];
    const RadioUnit unit = i == 0 ? RadioUnit::Com1 : RadioUnit::Com2;
    // Right-align the active frequency just left of the transfer arrow so the
    // wider 3-decimal COM readouts never collide with the arrow (the NAV side
    // right-aligns away from the arrow for the same reason).
    r.fillText(w * 0.832f, c.cy, freqText(c.active, 3), freqSize,
               TextAlign::Right, activeColor);
    drawTransferArrow(r, w * 0.844f, c.cy, w * 0.011f);
    // Right-align the standby just short of the identifier (rather than
    // left-aligning it, which grows the 3-decimal readout into the COMx label).
    drawStandby(w * 0.940f, c.cy, freqText(c.standby, 3),
                boxed(unit, ui.comSelected(), RadioBand::Com));
    r.fillText(w * 0.996f, c.cy, c.label, labelSize, TextAlign::Right,
               colors::kLabelText);
  }

  drawAfcsStatusBox(r, w, h, barH, d);
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

  // Panel body and cyan window border.
  r.fillRect(panelX, panelTop, panelW, panelH,
             withAlpha(Color{0.04f, 0.05f, 0.08f, 1.0f}, 0.96f * a));
  const Point border[5] = {{panelX, panelTop},
                           {panelX + panelW, panelTop},
                           {panelX + panelW, panelTop + panelH},
                           {panelX, panelTop + panelH},
                           {panelX, panelTop}};
  r.strokePolyline(border, 5, 2.0f, withAlpha(colors::kCyan, a));

  // Title bar.
  const float titleSize = fontPx(wt::kInfoValue, h);
  const float titleH = titleSize * 1.7f;
  r.fillRect(panelX, panelTop, panelW, titleH,
             withAlpha(colors::kCyan, 0.18f * a));
  r.strokeLine(panelX, panelTop + titleH, panelX + panelW, panelTop + titleH,
               1.5f, withAlpha(colors::kPanelSeparator, a));
  r.fillText(panelX + panelW * 0.5f, panelTop + titleH * 0.5f, title,
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a));

  f.a = a;
  f.x = panelX;
  f.top = panelTop;
  f.w = panelW;
  f.h = panelH;
  f.contentTop = panelTop + titleH;
  return f;
}

// Pop-up Alerts/messages window (under the "Alerts" key).
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui) {
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Alerts), "ALERTS",
                      w * 0.42f, h * 0.42f);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  // Message list (or an empty-state line).
  const float msgSize = fontPx(wt::kInfoValue, h) * 0.9f;
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
               bool blinkOn, float trailingGapFrac = 0.6f) {
  const float tw = r.measureTextWidth(text, size);
  if (highlighted && blinkOn) {
    const float padX = size * 0.25f;
    const float padY = size * 0.18f;
    r.fillRect(x - padX, cy - size * 0.5f - padY, tw + 2.0f * padX,
               size + 2.0f * padY, withAlpha(colors::kCyan, alpha));
  }
  const Color textColor = highlighted
                              ? (blinkOn ? colors::kBlack : colors::kCyan)
                              : color;
  r.fillText(x, cy, text, size, TextAlign::Left, withAlpha(textColor, alpha));
  return x + tw + size * trailingGapFrac;
}

// Timer/References window (Tmr/Ref softkey): generic timer, V-speed reference
// bugs, and barometric minimums (Pilot's Guide Fig. 2-6). The FMS rocker
// moves the cursor between fields; ENT activates the highlighted field.
void drawReferencesWindow(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::References),
                      "REFERENCES", w * 0.34f, h * 0.46f);
  if (f.a <= 0.0f) return;
  const float a = f.a;
  const RefField cursor = ui.referencesCursor();
  const bool blinkOn = ui.blinkOn();

  const float size = fontPx(wt::kInfoValue, h) * 0.9f;
  // Up to seven rows (TIMER, four V-speeds, MINS, and the TEMP-COMP row).
  const float lineH = (f.top + f.h - f.contentTop) / 7.5f;
  const float labelX = f.x + f.w * 0.05f;
  const float valueX = f.x + f.w * 0.34f;
  const float stateX = f.x + f.w * 0.70f;
  float cy = f.contentTop + lineH * 0.75f;

  // TIMER row: elapsed time, count direction, and the Start?/Stop?/Reset?
  // command field.
  r.fillText(labelX, cy, "TIMER", size, TextAlign::Left,
             withAlpha(colors::kWhite, a));
  putField(r, valueX, cy, formatTimer(ui.timerSeconds()), size, colors::kWhite,
           false, a, blinkOn);
  putField(r, f.x + f.w * 0.58f, cy, "UP", size, colors::kCyan, false, a,
           blinkOn);
  putField(r, stateX, cy, ui.timerCommandLabel(), size, colors::kWhite,
           cursor == RefField::TimerCmd, a, blinkOn);
  cy += lineH;

  // V-speed rows: reference value (cyan) and the On/Off enable field.
  for (int i = 0; i < kVSpeedRefCount; ++i) {
    const VSpeedRef& v = kVSpeedRefs[i];
    const bool on = ui.vspeedEnabled(static_cast<VspeedRef>(i));
    const RefField field =
        static_cast<RefField>(static_cast<int>(RefField::Glide) + i);
    const float vKt = ui.vspeedValueKt(static_cast<VspeedRef>(i));
    r.fillText(labelX, cy, v.windowLabel, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    // The reference value is the FMS-cursor field (small knob edits it); the
    // On/Off enable sits to its right.
    putField(r, valueX, cy, formatInt(vKt) + "KT", size, colors::kCyan,
             cursor == field, a, blinkOn);
    putField(r, stateX, cy, on ? "ON" : "OFF", size,
             on ? colors::kWhite : colors::kLabelText, false, a, blinkOn);
    cy += lineH;
  }

  // MINS row: Off/BARO/TEMP source and, when set, the MDA/DH altitude. TEMP
  // COMP adds a destination-temperature field on the following row.
  const MinimumsMode minsMode = ui.minimumsMode();
  const bool minsOn = minsMode != MinimumsMode::Off;
  const char* minsModeLabel = minsMode == MinimumsMode::Baro   ? "BARO"
                              : minsMode == MinimumsMode::Temp ? "TEMP"
                                                              : "OFF";
  r.fillText(labelX, cy, "MINS", size, TextAlign::Left,
             withAlpha(colors::kWhite, a));
  putField(r, valueX, cy, minsModeLabel, size, colors::kWhite,
           cursor == RefField::MinsMode, a, blinkOn);
  if (minsOn) {
    putField(r, stateX, cy, formatInt(ui.minimumsAltitudeFt()) + "FT", size,
             colors::kCyan, cursor == RefField::MinsValue, a, blinkOn);
  }
  if (minsMode == MinimumsMode::Temp) {
    cy += lineH;
    r.fillText(labelX, cy, "TEMP AT DEST", size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    char tbuf[16];
    std::snprintf(tbuf, sizeof(tbuf), "%+d\u00b0C",
                  static_cast<int>(std::lround(ui.minimumsTempC())));
    putField(r, stateX, cy, tbuf, size, colors::kCyan,
             cursor == RefField::MinsTemp, a, blinkOn);
  }
}

// Nearest Airports window (Nearest softkey): a distance-sorted, scrollable
// list of nearby airports with bearing/distance, COM frequency, and longest
// runway (Pilot's Guide Fig. 5-28). Three entries are visible at a time.
void drawNearestWindow(Renderer& r, float w, float h, const Layout& L,
                       const SoftkeyController& ui) {
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Nearest),
                      "NEAREST AIRPORTS", w * 0.42f, h * 0.42f);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const auto& list = ui.nearestAirports();
  const float size = fontPx(wt::kInfoValue, h) * 0.9f;
  if (list.empty()) {
    r.fillText(f.x + f.w * 0.5f, f.top + f.h * 0.55f, "None Within 200nm",
               size, TextAlign::Center, withAlpha(colors::kWhite, a));
    return;
  }

  constexpr int kVisibleEntries = 3;
  const int cursor = ui.nearestCursor();
  const bool blinkOn = ui.blinkOn();
  // The list scrolls only once the cursor moves past the bottom visible row.
  const int first = std::max(0, cursor - kVisibleEntries + 1);
  const float entryH = (f.top + f.h - f.contentTop) /
                       static_cast<float>(kVisibleEntries);
  const float labelX = f.x + f.w * 0.05f;
  const float indentX = f.x + f.w * 0.12f;

  for (int row = 0; row < kVisibleEntries; ++row) {
    const int i = first + row;
    if (i >= static_cast<int>(list.size())) break;
    const NearestAirport& apt = list[i];
    const float top = f.contentTop + entryH * static_cast<float>(row);
    const float cy1 = top + entryH * 0.30f;
    const float cy2 = top + entryH * 0.72f;

    // Line 1: identifier (FMS cursor highlights the selected airport),
    // bearing, and distance.
    putField(r, labelX, cy1, apt.id, size, colors::kWhite, i == cursor, a,
             blinkOn);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%s\u00b0  %.1fNM",
                  formatHeading(apt.bearingDeg).c_str(), apt.distanceNm);
    r.fillText(f.x + f.w * 0.95f, cy1, buf, size, TextAlign::Right,
               withAlpha(colors::kWhite, a));

    // Line 2: tunable COM frequency (cyan) and longest-runway length, dashed
    // where the database has no data, the way the real unit shows unknowns.
    const std::string freq =
        apt.frequencyMhz > 0.0f ? formatFreq(apt.frequencyMhz, 3) : "---.---";
    r.fillText(indentX, cy2, freq, size, TextAlign::Left,
               withAlpha(colors::kCyan, a));
    const std::string rnwy =
        apt.longestRunwayFt > 0
            ? formatInt(static_cast<float>(apt.longestRunwayFt)) + "FT"
            : "----FT";
    float rx = f.x + f.w * 0.95f - r.measureTextWidth(rnwy, size);
    r.fillText(rx, cy2, rnwy, size, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    r.fillText(rx - size * 0.4f, cy2, "RNWY", size * 0.8f, TextAlign::Right,
               withAlpha(colors::kLabelText, a));

    if (row > 0) {
      r.strokeLine(f.x + f.w * 0.03f, top, f.x + f.w * 0.97f, top, 1.0f,
                   withAlpha(colors::kPanelSeparator, a));
    }
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

}  // namespace

void drawChrome(Renderer& r, const Layout& L, const FlightData& d,
                const SoftkeyController& ui, float w, float h) {
  drawTopBar(r, w, h, L, d, ui);
  drawNavStatusBox(r, w, h, L, d);
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
  drawSoftkeyBar(r, w, h, L, ui);
}

}  // namespace avionics::pfd
