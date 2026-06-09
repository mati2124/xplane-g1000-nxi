#include <algorithm>
#include <cstdio>

#include "render/pfd/PfdInternal.h"

namespace avionics::pfd {
namespace {

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

void drawFmaCenter(Renderer& r, float w, float h, float barH,
                   const FlightData& d) {
  const float left = w * 0.255f;
  const float right = w * 0.745f;
  const float centerW = right - left;
  const float rowSplitY = barH * 0.50f;
  const float pad = centerW * 0.015f;

  r.strokeLine(left, rowSplitY, right, rowSplitY, 1.5f, colors::kPanelSeparator);

  const float row1Y = barH * 0.26f;
  const float row2Y = barH * 0.76f;
  const float smallSize = fontPx(wt::kFmaSmall, h);
  const float dataSize = fontPx(wt::kFmaArmed, h);
  const float modeSize = fontPx(wt::kFmaActive, h);
  const float armedSize = fontPx(wt::kFmaArmed, h);

  const float topLeftW = centerW * 0.56f;
  const float topRightX = left + topLeftW;

  // Active-leg field: waypoint identifiers in white with a magenta leg arrow,
  // as on the real G1000 navigation status box.
  float x = left + pad;
  if (!d.fmaFromWpt.empty()) {
    x = putText(r, x, row1Y, d.fmaFromWpt, dataSize, colors::kWhite, 0.30f);
  }
  if (!d.fmaToWpt.empty()) {
    x = putText(r, x, row1Y, "\u2192", dataSize, colors::kMagenta, 0.30f);
    putText(r, x, row1Y, d.fmaToWpt, dataSize, colors::kWhite);
  }

  r.strokeLine(topRightX, barH * 0.08f, topRightX, rowSplitY - 1.0f, 1.5f,
               colors::kPanelSeparator);
  {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", d.fmaLegDistanceNm);
    float rx = topRightX + pad;
    rx = putText(r, rx, row1Y, "DIS", smallSize, colors::kLabelText, 0.35f);
    rx = putText(r, rx, row1Y, std::string(buf), dataSize, colors::kWhite,
                 0.20f);
    rx = putText(r, rx, row1Y, "NM", smallSize, colors::kLabelText, 0.80f);
    rx = putText(r, rx, row1Y, "BRG", smallSize, colors::kLabelText, 0.35f);
    putText(r, rx, row1Y, formatHeading(d.fmaLegBearingDeg) + "\u00b0",
            dataSize, colors::kMagenta);
  }

  const float latW = centerW * 0.245f;
  const float apW = centerW * 0.225f;
  const float vertX = left + latW + apW;

  r.strokeLine(left + latW, rowSplitY + 1.0f, left + latW, barH * 0.92f, 1.5f,
               colors::kPanelSeparator);
  r.strokeLine(vertX, rowSplitY + 1.0f, vertX, barH * 0.92f, 1.5f,
               colors::kPanelSeparator);

  float lx = left + pad;
  if (!d.fmaLateralArmed.empty()) {
    lx = putText(r, lx, row2Y, d.fmaLateralArmed, armedSize, colors::kWhite,
                 0.40f);
  }
  if (!d.fmaLateralActive.empty()) {
    putText(r, lx, row2Y, d.fmaLateralActive, modeSize, colors::kActiveGreen);
  }

  {
    std::string apYd;
    if (d.apEngaged) apYd += "AP";
    if (d.ydEngaged) apYd += (apYd.empty() ? "YD" : "  YD");
    if (!apYd.empty()) {
      const float cellCx = left + latW + apW * 0.5f;
      r.fillText(cellCx, row2Y, apYd, modeSize, TextAlign::Center,
                 colors::kActiveGreen);
    }
  }

  float vx = vertX + pad;
  if (!d.fmaVerticalActive.empty()) {
    vx = putText(r, vx, row2Y, d.fmaVerticalActive, modeSize,
                 colors::kActiveGreen, 0.35f);
  }
  if (d.fmaVerticalValue != 0 && !d.fmaVerticalUnits.empty()) {
    // The vertical mode reference (e.g. the captured altitude) is cyan.
    vx = putText(r, vx, row2Y, formatInt(static_cast<float>(d.fmaVerticalValue)),
                 armedSize, colors::kCyan, 0.20f);
    putText(r, vx, row2Y, d.fmaVerticalUnits, smallSize, colors::kCyan);
  }

  float rx = right - pad;
  if (!d.fmaVerticalApproachArmed.empty()) {
    r.fillText(rx, row2Y, d.fmaVerticalApproachArmed, armedSize,
               TextAlign::Right, colors::kWhite);
    rx -= r.measureTextWidth(d.fmaVerticalApproachArmed, armedSize) +
          armedSize * 0.40f;
  }
  if (!d.fmaVerticalArmed.empty()) {
    r.fillText(rx, row2Y, d.fmaVerticalArmed, armedSize, TextAlign::Right,
               colors::kWhite);
  }
}

void drawTopBar(Renderer& r, float w, float h, const Layout& L,
                const FlightData& d) {
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

  struct Nav {
    const char* label;
    float active, standby, cy;
  };
  const Nav navs[2] = {{"NAV1", d.nav1ActiveMhz, d.nav1StandbyMhz, row1Cy},
                       {"NAV2", d.nav2ActiveMhz, d.nav2StandbyMhz, row2Cy}};
  for (const Nav& n : navs) {
    r.fillText(w * 0.012f, n.cy, n.label, labelSize, TextAlign::Left,
               colors::kLabelText);
    r.fillText(w * 0.140f, n.cy, formatFreq(n.standby, 2), freqSize,
               TextAlign::Right, colors::kWhite);
    drawTransferArrow(r, w * 0.158f, n.cy, w * 0.011f);
    r.fillText(w * 0.245f, n.cy, formatFreq(n.active, 2), freqSize,
               TextAlign::Right, colors::kActiveGreen);
  }

  const Nav coms[2] = {{"COM1", d.com1ActiveMhz, d.com1StandbyMhz, row1Cy},
                       {"COM2", d.com2ActiveMhz, d.com2StandbyMhz, row2Cy}};
  for (const Nav& c : coms) {
    // Right-align the active frequency just left of the transfer arrow so the
    // wider 3-decimal COM readouts never collide with the arrow (the NAV side
    // right-aligns away from the arrow for the same reason).
    r.fillText(w * 0.832f, c.cy, formatFreq(c.active, 3), freqSize,
               TextAlign::Right, colors::kActiveGreen);
    drawTransferArrow(r, w * 0.844f, c.cy, w * 0.011f);
    // Right-align the standby just short of the identifier (rather than
    // left-aligning it, which grows the 3-decimal readout into the COMx label).
    r.fillText(w * 0.940f, c.cy, formatFreq(c.standby, 3), freqSize,
               TextAlign::Right, colors::kWhite);
    r.fillText(w * 0.996f, c.cy, c.label, labelSize, TextAlign::Right,
               colors::kLabelText);
  }

  drawFmaCenter(r, w, h, barH, d);
}

// Filled box with the NXi bottom-panel gradient (lighter at top, black at the
// bottom).
void drawInfoBox(Renderer& r, float x, float top, float w, float h) {
  r.fillRectVerticalGradient(x, top, w, h, top, top + h, colors::kInfoBoxTop,
                             colors::kBlack);
}

void drawBottomInfoPanel(Renderer& r, float w, float h, const Layout& L,
                         const FlightData& d) {
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

  drawInfoBox(r, 0.0f, top, oatW, panelH);
  drawInfoBox(r, xpdrX, top, xpdrW, panelH);
  drawInfoBox(r, timeX, top, timeW, panelH);

  // OAT box.
  const float oatCy = top + panelH * 0.62f;
  r.fillText(oatW * 0.06f, oatCy, "OAT", labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(oatW * 0.95f, oatCy, formatOat(d.oatCelsius) + "\u00b0C",
             valueSize, TextAlign::Right, colors::kWhite);

  // Bearing-pointer info windows flanking the rose (BRG1 left, BRG2 right).
  const float brgSize = fontPx(wt::kInfoLabel, h);
  const float brgRowCy = top + panelH * 0.55f;
  if (d.bearing1Valid) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1fNM", d.bearing1DistanceNm);
    float bx = 200.0f * L.sx;
    bx = putText(r, bx, brgRowCy, d.bearing1Source, brgSize, colors::kCyan,
                 0.35f);
    putText(r, bx, brgRowCy, buf, brgSize, colors::kMagenta, 0.0f);
  }
  if (d.bearing2Valid) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1fNM", d.bearing2DistanceNm);
    float bx = 600.0f * L.sx;
    bx = putText(r, bx, brgRowCy, d.bearing2Source, brgSize, colors::kCyan,
                 0.35f);
    putText(r, bx, brgRowCy, buf, brgSize, colors::kMagenta, 0.0f);
  }

  // Transponder box: code, mode and reply ("R"), green in the air.
  const float xpdrCy = top + panelH * 0.62f;
  char codeBuf[8];
  std::snprintf(codeBuf, sizeof(codeBuf), "%04d", d.transponderCode);
  float xx = xpdrX + xpdrW * 0.04f;
  xx = putText(r, xx, xpdrCy, "XPDR", labelSize, colors::kLabelText, 0.40f);
  xx = putText(r, xx, xpdrCy, codeBuf, labelSize, colors::kActiveGreen, 0.45f);
  xx = putText(r, xx, xpdrCy, d.transponderMode, labelSize, colors::kActiveGreen,
               0.45f);
  if (d.transponderReply) {
    putText(r, xx, xpdrCy, "R", labelSize, colors::kActiveGreen, 0.0f);
  }

  // Time box: label + HH:MM:SS.
  const float timeCy = top + panelH * 0.62f;
  r.fillText(timeX + timeW * 0.06f, timeCy, d.clockIsUtc ? "UTC" : "LCL",
             labelSize, TextAlign::Left, colors::kLabelText);
  r.fillText(w - timeW * 0.05f, timeCy,
             formatHms(d.utcHour, d.utcMinute, d.utcSecond), labelSize,
             TextAlign::Right, colors::kWhite);
}

void drawSoftkeyBar(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float top = h - L.bottomBarH;
  r.fillRect(0.0f, top, w, L.bottomBarH, colors::kSoftkeyBackground);
  r.strokeLine(0.0f, top, w, top, 2.0f, colors::kPanelBorder);

  const float cellW = w / static_cast<float>(kSoftkeyCount);
  const float cy = top + L.bottomBarH * 0.5f;
  const float size = fontPx(wt::kSoftkey, h);
  for (int i = 0; i < kSoftkeyCount; ++i) {
    const float cellX = static_cast<float>(i) * cellW;

    // Press flash (decaying) and the steady highlight while a cell's window is
    // open combine into one 0..1 level that lifts the cell toward cyan, like
    // the Working Title NXi key-press feedback.
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 0.6f : 0.0f);
    if (level > 0.0f && ui.label(i)[0] != '\0') {
      r.fillRectVerticalGradient(cellX, top, cellW, L.bottomBarH, top,
                                 h, withAlpha(colors::kCyan, level * 0.45f),
                                 withAlpha(colors::kCyan, level * 0.08f));
      r.strokeLine(cellX, top, cellX + cellW, top, 2.5f,
                   withAlpha(colors::kCyan, level));
    }

    if (i > 0) {
      r.strokeLine(cellX, top + L.bottomBarH * 0.15f, cellX,
                   h - L.bottomBarH * 0.15f, 1.0f, colors::kPanelSeparator);
    }
    if (ui.label(i)[0] != '\0') {
      r.fillText(cellX + cellW * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 colors::kWhite);
    }
  }
}

// Pop-up Alerts/messages window. Anchored above the softkey bar at the lower
// right (under the "Alerts" key) and animated with an eased slide-up + fade,
// driven by the controller's 0..1 progress.
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui) {
  const float raw = ui.alertsAnim();
  if (raw <= 0.0f) return;
  const float a = smoothstep(raw);

  const float panelW = w * 0.42f;
  const float panelH = h * 0.42f;
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
  r.fillText(panelX + panelW * 0.5f, panelTop + titleH * 0.5f, "ALERTS",
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a));

  // Message list (or an empty-state line).
  const float msgSize = fontPx(wt::kInfoValue, h) * 0.9f;
  const float lineH = msgSize * 1.6f;
  const float textX = panelX + panelW * 0.04f;
  float y = panelTop + titleH + lineH * 0.75f;

  const auto& msgs = ui.alerts();
  if (msgs.empty()) {
    r.fillText(panelX + panelW * 0.5f, panelTop + panelH * 0.55f,
               "NO ACTIVE ALERTS", msgSize, TextAlign::Center,
               withAlpha(colors::kLabelText, a));
    return;
  }
  for (const AlertMessage& m : msgs) {
    if (y > panelTop + panelH - lineH * 0.4f) break;  // clip overflow
    Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
              : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                 : colors::kWhite;
    r.fillText(textX, y, m.text, msgSize, TextAlign::Left, withAlpha(c, a));
    y += lineH;
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
  drawTopBar(r, w, h, L, d);
  drawBottomInfoPanel(r, w, h, L, d);
  // CAS annunciation window is always visible (when active) on the main PFD.
  drawCasAnnunciations(r, w, h, L, ui);
  // The Alerts window sits above the info panel but below the softkey bar, so
  // the bar (and its press highlights) always stay on top.
  drawAlertsWindow(r, w, h, L, ui);
  drawSoftkeyBar(r, w, h, L, ui);
}

}  // namespace avionics::pfd
