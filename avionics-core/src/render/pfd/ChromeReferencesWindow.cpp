#include <cmath>
#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// Timer/References window (Tmr/Ref softkey): generic timer, V-speed reference
// bugs, and barometric minimums (Pilot's Guide Fig. 2-6). The FMS rocker
// moves the cursor between fields; ENT activates the highlighted field.
void drawReferencesWindow(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
  // Row metrics: the References window has only a handful of rows, so the
  // panel is sized to its content (like the real unit, which grows when MINS
  // is in TEMP COMP) and the rows use a generous font/spacing. The old tight
  // 18/20 px WT base left the lower half of the fixed popout box empty.
  const float size = fontPx(21.0f, h);
  const float unitSize = fontPx(16.0f, h);
  const float lineH = fontPx(26.0f, h);

  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);  // width only; height set to content
  const bool tempComp = ui.minimumsMode() == MinimumsMode::Temp;
  // Title chrome + top padding (timer 0.85, gap 1.25), four V-speed rows, the
  // MINS row, an optional Temp-At row, and a bottom padding of ~0.9 lineH.
  const float titleChromePx = fontPx(34.0f, h);
  const float rowsLineH = (0.85f + 1.25f + kVSpeedRefCount +
                           (tempComp ? 1.0f : 0.0f) + 0.9f) * lineH;
  panelH = titleChromePx + rowsLineH;

  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::References),
                      "References", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;
  const RefField cursor = ui.referencesCursor();
  const bool blinkOn = ui.blinkOn();

  const float labelX = f.x + f.w * 0.07f;
  const float numRightX = f.x + f.w * 0.60f;    // right edge of a V-speed value
  const float minsNumRightX = f.x + f.w * 0.82f;  // right edge of the MINS value
  const float toggleCenterX = f.x + f.w * 0.85f;  // On/Off toggle center
  const float minsModeCenterX = f.x + f.w * 0.46f;
  float cy = f.contentTop + lineH * 0.85f;

  // Small filled triangle carrot; the available toggle direction is green.
  const auto carrot = [&](float ax, float acy, bool pointRight, bool active) {
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
    const Color c = active ? colors::kActiveGreen : colors::kLabelText;
    r.fillPolygon(t, 3, withAlpha(c, a));
  };

  // Center-anchored toggle/list field (On/Off, MINS mode): cyan value flanked
  // by carrots; pulses as a cyan plate while it is the cursor field.
  const auto toggleField = [&](float centerX, const std::string& text,
                               bool highlighted, bool canDec, bool canInc) {
    const float tw = r.measureTextWidth(text, size);
    const float gap = size * 0.30f;
    const float aw = size * 0.26f;
    if (highlighted && blinkOn) {
      const float padX = size * 0.22f;
      const float padY = size * 0.18f;
      r.fillRect(centerX - tw * 0.5f - padX, cy - size * 0.5f - padY,
                 tw + 2.0f * padX, size + 2.0f * padY,
                 withAlpha(colors::kPopoutCyan, a));
    }
    const Color textColor = highlighted ? (blinkOn ? colors::kBlack : colors::kPopoutCyan)
                                         : colors::kPopoutCyan;
    r.fillText(centerX, cy, text, size, TextAlign::Center,
               withAlpha(textColor, a));
    carrot(centerX - tw * 0.5f - gap - aw, cy, /*pointRight=*/false, canDec);
    carrot(centerX + tw * 0.5f + gap, cy, /*pointRight=*/true, canInc);
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
                 withAlpha(colors::kPopoutCyan, a));
    }
    const Color c = highlighted ? (blinkOn ? colors::kBlack : colors::kPopoutCyan)
                                 : colors::kPopoutCyan;
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
             TextAlign::Left, withAlpha(colors::kPopoutCyan, a));
  r.fillText(f.x + f.w * 0.58f, cy, "Up", size, TextAlign::Left,
             withAlpha(colors::kPopoutCyan, a));
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
    r.fillRoundedRect(bx, by, bw, bh, rad, withAlpha(colors::kPopoutCyan, a));
    r.fillText(cmdCx, cy, cmd, size, TextAlign::Center,
               withAlpha(colors::kBlack, a));
  }
  // Extra breathing room below the (taller) timer command button before the
  // separator rule / V-speed list, so the rounded button doesn't crowd the line.
  cy += lineH * 1.25f;

  // Separator rule below the Timer section (the real window groups the timer
  // above the V-speed/MINS list), centered in the gap between the button and
  // the first V-speed row.
  const float sepInset = f.w * 0.04f;
  const float sepY = cy - lineH * 0.6f;
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
    toggleField(toggleCenterX, on ? "On" : "Off", /*highlighted=*/false,
                /*canDec=*/on, /*canInc=*/!on);
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
  toggleField(minsModeCenterX, minsModeLabel, cursor == RefField::MinsMode,
              minsMode != MinimumsMode::Off, minsMode != MinimumsMode::Temp);
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
    toggleField(minsModeCenterX, tbuf, cursor == RefField::MinsTemp, true, true);
    valueField(minsNumRightX, formatInt(ui.effectiveMinimumsFt()), "FT",
               /*modified=*/false, /*highlighted=*/false);
  }
}

}  // namespace avionics::pfd
