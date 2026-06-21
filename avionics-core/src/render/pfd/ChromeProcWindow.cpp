#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "render/pfd/ChromeInternal.h"

#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {
namespace {

// One selectable row in the Procedures menu. Geometry matches the Active Flight
// Plan window (ChromeFlightPlanWindow): ident column inset, 29 px row pitch,
// and a text-width cyan select plate (not a full-width bar).
void drawProcMenuRow(Renderer& r, float identX, float cy, const std::string& text,
                     float size, bool selected, bool enabled, bool blinkOn,
                     float a) {
  if (selected && enabled && blinkOn) {
    const float tracking = size * 0.06f;
    const float tw = r.measureTextWidth(text.c_str(), size);
    r.fillRect(identX - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
               size * 1.24f, withAlpha(colors::kPopoutCyan, a));
    r.fillText(identX, cy, text, size, TextAlign::Left,
               withAlpha(colors::kBlack, a));
    return;
  }
  const Color color = selected && enabled ? colors::kPopoutCyan
                      : enabled            ? colors::kPopoutCyan
                                           : colors::kDisabledGray;
  r.fillText(identX, cy, text, size, TextAlign::Left, withAlpha(color, a));
}

void drawProcScrollBar(Renderer& r, float trackX, float scrollW, float trackTop,
                       float trackH, int total, int visible, int first, float a) {
  if (total <= visible) return;
  const float thumbH =
      std::max(fontPx(18.0f, trackH), trackH * static_cast<float>(visible) /
                                           static_cast<float>(total));
  const float maxScroll = static_cast<float>(total - visible);
  const float thumbTop =
      trackTop + (trackH - thumbH) * static_cast<float>(first) / maxScroll;
  r.fillRect(trackX + scrollW * 0.5f - 1.0f, trackTop, 2.0f, trackH,
             withAlpha(colors::kPanelSeparator, a));
  r.fillRect(trackX, thumbTop, scrollW, thumbH,
             withAlpha(colors::kMenuBorderGray, a));
}

float drawDtoStyleButton(Renderer& r, float leftX, float cy, const char* label,
                         float size, bool armed, bool blinkOn, float a) {
  const float bw = r.measureTextWidth(label, size) + size * 1.3f;
  const float bh = size * 1.7f;
  const float bx = leftX;
  const float by = cy - bh * 0.5f;
  const float radius = bh * 0.32f;
  if (armed && blinkOn) {
    r.fillRoundedRect(bx, by, bw, bh, radius, withAlpha(colors::kPopoutCyan, a));
  }
  r.strokeRoundedRect(bx, by, bw, bh, radius, 1.5f,
                      withAlpha(armed ? colors::kWhite : colors::kGroupBoxBorder,
                                a));
  const Color textColor =
      armed ? (blinkOn ? colors::kBlack : colors::kPopoutCyan) : colors::kWhite;
  r.fillText(leftX + bw * 0.5f, cy, label, size, TextAlign::Center,
             withAlpha(textColor, a));
  return bw;
}

void drawDashRun(Renderer& r, float x, float cy, int count, float size,
                 const Color& color, float a) {
  const float dashW = size * 0.36f;
  const float advance = size * 0.48f;
  const float dashH = size * 0.09f;
  for (int i = 0; i < count; ++i) {
    r.fillRect(x + (advance - dashW) * 0.5f, cy - dashH * 0.5f, dashW, dashH,
               withAlpha(color, a));
    x += advance;
  }
}

float dashRunWidth(int count, float size) {
  return size * 0.48f * static_cast<float>(count);
}

void drawDashRunRight(Renderer& r, float rightX, float cy, int count, float size,
                      const Color& color, float a) {
  drawDashRun(r, rightX - dashRunWidth(count, size), cy, count, size, color, a);
}

void drawCarrot(Renderer& r, float cx, float cy, float size, bool pointRight,
                bool enabled, float a) {
  const float aw = size * 0.34f;
  if (!enabled) return;
  if (pointRight) {
    const Point tri[3] = {{cx + aw * 0.55f, cy},
                          {cx - aw * 0.30f, cy - aw},
                          {cx - aw * 0.30f, cy + aw}};
    r.fillPolygon(tri, 3, withAlpha(colors::kPopoutCyan, a));
  } else {
    const Point tri[3] = {{cx - aw * 0.55f, cy},
                          {cx + aw * 0.30f, cy - aw},
                          {cx + aw * 0.30f, cy + aw}};
    r.fillPolygon(tri, 3, withAlpha(colors::kPopoutCyan, a));
  }
}

std::string restAfterRnavGps(const std::string& label) {
  if (label.size() < 4 || label.compare(0, 4, "RNAV") != 0) return label;
  std::size_t pos = 4;
  while (pos < label.size() && label[pos] == ' ') ++pos;
  if (pos < label.size() && label[pos] == '_') ++pos;
  if (label.compare(pos, 5, "(GPS)") == 0) pos += 5;
  else if (label.compare(pos, 3, "GPS") == 0) pos += 3;
  while (pos < label.size() && label[pos] == ' ') ++pos;
  return label.substr(pos);
}

constexpr float kRnavGpsSuffixGap = 0.08f;

float rnavGpsSuffixGap(float size) { return size * kRnavGpsSuffixGap; }

float measureApproachLabelWidth(Renderer& r, const std::string& label, float size,
                                bool rnavGps) {
  if (!rnavGps) return r.measureTextWidth(label.c_str(), size);
  const float subSize = size * 0.72f;
  float w = r.measureTextWidth("RNAV", size) + rnavGpsSuffixGap(size) +
            r.measureTextWidth("GPS", subSize);
  const std::string rest = restAfterRnavGps(label);
  if (!rest.empty()) {
    w += r.measureTextWidth(" ", size) + r.measureTextWidth(rest.c_str(), size);
  }
  return w;
}

void drawApproachLabel(Renderer& r, float x, float cy, const std::string& label,
                       float size, const Color& color, float a,
                       bool rnavGps) {
  if (!rnavGps) {
    r.fillText(x, cy, label, size, TextAlign::Left, withAlpha(color, a));
    return;
  }
  const float subSize = size * 0.72f;
  const Color c = withAlpha(color, a);
  float xx = x;
  r.fillText(xx, cy, "RNAV", size, TextAlign::Left, c);
  xx += r.measureTextWidth("RNAV", size) + rnavGpsSuffixGap(size);
  r.fillText(xx, cy, "GPS", subSize, TextAlign::Left, c);
  xx += r.measureTextWidth("GPS", subSize);
  const std::string rest = restAfterRnavGps(label);
  if (!rest.empty()) {
    r.fillText(xx, cy, " ", size, TextAlign::Left, withAlpha(color, a));
    xx += r.measureTextWidth(" ", size);
    r.fillText(xx, cy, rest, size, TextAlign::Left, withAlpha(color, a));
  }
}

bool labelIsRnavGps(const std::string& label) {
  return label.size() >= 4 && label.compare(0, 4, "RNAV") == 0;
}

void drawProcSubListPopup(Renderer& r, const WindowFrame& f, float h,
                          const SoftkeyController& ui, float a) {
  const std::vector<std::string> items = ui.procListItems();
  if (items.empty()) return;

  const float size = fontPx(wt::kInfoValue, h);
  const float rowH = fontPx(22.0f, h);
  const float pad = fontPx(6.0f, h);
  const int visible = 7;
  const float popupW = f.w * 0.62f;
  const float popupH = rowH * static_cast<float>(visible) + pad * 2.0f;
  const float popupX = f.x + (f.w - popupW) * 0.5f;
  const float popupY = f.top + (f.h - popupH) * 0.42f;
  const float radius = fontPx(4.0f, h);
  const float borderW = 1.5f;

  r.fillRoundedRect(popupX, popupY, popupW, popupH, radius,
                    withAlpha(colors::kPopoutBodyBottom, a * 0.98f));
  r.strokeRoundedRect(popupX + borderW * 0.5f, popupY + borderW * 0.5f,
                      popupW - borderW, popupH - borderW, radius, borderW,
                      withAlpha(colors::kPopoutBorder, a));

  const int total = static_cast<int>(items.size());
  const int selected = std::min(std::max(0, ui.procListSelected()), total - 1);
  int first = 0;
  if (total > visible) {
    first = std::max(0, std::min(selected - visible / 2, total - visible));
  }
  const int end = std::min(total, first + visible);
  const float listTop = popupY + pad;
  const float listLeft = popupX + pad;
  const float listRight = popupX + popupW - pad - fontPx(10.0f, h);
  const bool blinkOn = ui.blinkOn();

  for (int i = first; i < end; ++i) {
    const float rowCy =
        listTop + rowH * (static_cast<float>(i - first) + 0.5f);
    std::string label;
    if (ui.procStep() == SoftkeyController::ProcStep::TransitionList) {
      label = items[static_cast<std::size_t>(i)];
    } else {
      label = ui.procApproachDisplayName(i);
    }
    const bool isSel = i == selected;
    const Color color = isSel ? colors::kPopoutCyan : colors::kWhite;
    if (isSel && blinkOn) {
      const float tw = measureApproachLabelWidth(r, label, size, labelIsRnavGps(label));
      r.fillRect(listLeft - size * 0.08f, rowCy - size * 0.62f,
                 tw + size * 0.16f, size * 1.24f,
                 withAlpha(colors::kPopoutCyan, a));
      drawApproachLabel(r, listLeft, rowCy, label, size, colors::kBlack, a,
                        labelIsRnavGps(label));
    } else {
      drawApproachLabel(r, listLeft, rowCy, label, size, color, a,
                        labelIsRnavGps(label));
    }
  }

  if (total > visible) {
    const float scrollW = fontPx(8.0f, h);
    const float trackTop = listTop;
    const float trackH = rowH * static_cast<float>(visible);
    drawProcScrollBar(r, popupX + popupW - pad - scrollW, scrollW, trackTop,
                      trackH, total, visible, first, a);
  }
}

void drawApproachSelectWindow(Renderer& r, const WindowFrame& f, float w, float h,
                              const SoftkeyController& ui) {
  const float a = f.a;
  const float size = fontPx(wt::kInfoValue, h);
  const float labelSize = size * 0.92f;
  const float smallSize = size * 0.72f;
  const float padX = fontPx(8.0f, h);
  const float rowH = fontPx(24.0f, h);
  const float left = f.x + padX;
  const float right = f.x + f.w - padX;
  const float labelX = left + size * 0.12f;
  const float valueX = left + f.w * 0.26f;
  const float idLabelX = f.x + f.w * 0.68f;
  const bool blinkOn = ui.blinkOn();
  const float sepInset = f.w * 0.04f;
  const bool subListOpen = ui.procSubListOpen();

  const auto drawFieldLabel = [&](float cy, const char* label, bool cursor) {
    if (cursor && blinkOn && !subListOpen) {
      const float barW = size * 0.14f;
      r.fillRect(labelX - barW * 1.6f, cy - size * 0.55f, barW, size * 1.1f,
                 withAlpha(colors::kPopoutCyan, a));
    }
    r.fillText(labelX, cy, label, labelSize, TextAlign::Left,
               withAlpha(colors::kWhite, a));
  };

  const auto drawValuePlate = [&](float cy, const std::string& text,
                                  bool highlighted, bool rnavGps) {
    const float tw = measureApproachLabelWidth(r, text, size, rnavGps);
    if (highlighted && blinkOn && !subListOpen) {
      r.fillRect(valueX - size * 0.12f, cy - size * 0.62f, tw + size * 0.24f,
                 size * 1.24f, withAlpha(colors::kPopoutCyan, a));
      drawApproachLabel(r, valueX, cy, text, size, colors::kBlack, a, rnavGps);
    } else {
      const Color c = highlighted ? colors::kPopoutCyan : colors::kPopoutCyan;
      drawApproachLabel(r, valueX, cy, text, size, c, a, rnavGps);
    }
  };

  // Anchor field rows from the bottom so they never crowd the footer buttons.
  const float buttonsCy = f.top + f.h - padX - size * 0.85f;
  const float minHeaderCy = f.contentTop + size * 0.55f;
  constexpr float kHeaderStackRows = 4.9f;  // header..apr vertical span in rowH units
  float layoutRowH = rowH;
  const float stackH = layoutRowH * kHeaderStackRows + size * 1.2f;
  if (buttonsCy - stackH < minHeaderCy) {
    layoutRowH =
        std::max(fontPx(18.0f, h),
                 (buttonsCy - size * 1.2f - minHeaderCy) / kHeaderStackRows);
  }
  const float btnSepY = buttonsCy - size * 1.2f;
  const float primCy = btnSepY - layoutRowH * 0.35f;
  const float minsCy = primCy - layoutRowH;
  const float transCy = minsCy - layoutRowH;
  const float aprCy = transCy - layoutRowH;
  const float sepY = aprCy - layoutRowH * 0.55f;
  const float nameCy = sepY - layoutRowH * 0.5f;
  const float headerCy = nameCy - layoutRowH * 0.9f;

  // Airport header block: ICAO + icon + city on top row; name + CHNL on bottom row.
  const std::string icao = ui.procAirportIcao();
  const char* icaoText = icao.empty() ? "_____" : icao.c_str();
  r.fillText(left, headerCy, icaoText, size, TextAlign::Left,
             withAlpha(colors::kPopoutCyan, a));

  const MapFeature sym = ui.procAirportFeature();
  const float iconSize = fontPx(15.0f, h) * 0.45f;
  const float iconCx =
      left + r.measureTextWidth(icaoText, size) + iconSize * 0.9f;
  drawUiWaypointIcon(r, sym, iconCx, headerCy, iconSize);

  const std::string cityLine = ui.procAirportCityLine();
  if (!cityLine.empty()) {
    const float cityX = iconCx + iconSize * 0.9f;
    const float cityMaxW = right - cityX;
    std::string city = cityLine;
    while (!city.empty() &&
           r.measureTextWidth(city.c_str(), labelSize) > cityMaxW) {
      city.pop_back();
    }
    r.fillText(cityX, headerCy, city, labelSize, TextAlign::Left,
               withAlpha(colors::kPopoutCyan, a));
  }

  const std::string nameLine = ui.procAirportNameLine();
  constexpr int kChnlDashCount = 5;
  const float chnlDashW = dashRunWidth(kChnlDashCount, labelSize);
  const std::string chnlLabel = "CHNL";
  const float chnlLabelW = r.measureTextWidth(chnlLabel, labelSize);
  const float chnlBlockW = chnlLabelW + labelSize * 0.25f + chnlDashW;
  if (!nameLine.empty()) {
    const float nameMaxW = right - chnlBlockW - size * 0.4f - left;
    std::string name = nameLine;
    while (!name.empty() && r.measureTextWidth(name.c_str(), labelSize) > nameMaxW) {
      name.pop_back();
    }
    r.fillText(left, nameCy, name, labelSize, TextAlign::Left,
               withAlpha(colors::kPopoutCyan, a));
  }
  r.fillText(right - chnlDashW - chnlLabelW - labelSize * 0.25f, nameCy, chnlLabel,
             labelSize, TextAlign::Left, withAlpha(colors::kWhite, a));
  drawDashRunRight(r, right, nameCy, kChnlDashCount, labelSize,
                   colors::kPopoutCyan, a);

  r.strokeLine(f.x + sepInset, sepY, f.x + f.w - sepInset, sepY, 1.0f,
               withAlpha(colors::kPanelSeparator, a));

  // APR row.
  drawFieldLabel(aprCy, "APR",
                 ui.procApproachField() ==
                     SoftkeyController::ProcApproachField::Apr);
  const std::string apr = ui.procSelectedApproachDisplay();
  if (!apr.empty()) {
    drawValuePlate(aprCy, apr,
                   ui.procApproachField() ==
                       SoftkeyController::ProcApproachField::Apr,
                   labelIsRnavGps(apr));
  }

  // TRANS + ID row.
  drawFieldLabel(transCy, "TRANS",
                 ui.procApproachField() ==
                     SoftkeyController::ProcApproachField::Trans);
  const std::string trans = ui.procSelectedTransitionDisplay();
  if (!trans.empty()) {
    drawValuePlate(transCy, trans,
                   ui.procApproachField() ==
                       SoftkeyController::ProcApproachField::Trans,
                   false);
  }

  const char* idLabel = "ID";
  const float idLabelW = r.measureTextWidth(idLabel, labelSize);
  r.fillText(idLabelX, transCy, idLabel, labelSize, TextAlign::Left,
             withAlpha(colors::kWhite, a));
  if (ui.procApproachField() == SoftkeyController::ProcApproachField::Id &&
      blinkOn && !subListOpen) {
    const float barW = size * 0.14f;
    r.fillRect(idLabelX - barW * 1.6f, transCy - size * 0.55f, barW,
               size * 1.1f, withAlpha(colors::kPopoutCyan, a));
  }
  constexpr int kIdDashCount = 5;
  drawDashRunRight(r, right, transCy, kIdDashCount, labelSize, colors::kWhite, a);

  // MINS row: toggle sits in the value column, not tight against the label.
  drawFieldLabel(minsCy, "MINS",
                 ui.procApproachField() ==
                     SoftkeyController::ProcApproachField::Mins);
  const bool minsBaro = ui.minimumsMode() == MinimumsMode::Baro;
  const char* minsLabel = minsBaro ? "BARO" : "OFF";
  const float minsBaroW = r.measureTextWidth("BARO", labelSize);
  const float minsOffW = r.measureTextWidth("OFF", labelSize);
  const float minsToggleW = std::max(minsBaroW, minsOffW);
  const float minsCenterX = valueX + minsToggleW * 0.5f;
  const bool minsHighlighted =
      ui.procApproachField() == SoftkeyController::ProcApproachField::Mins;
  if (minsHighlighted && blinkOn && !subListOpen) {
    r.fillRect(minsCenterX - minsToggleW * 0.5f - labelSize * 0.15f,
               minsCy - labelSize * 0.62f, minsToggleW + labelSize * 0.3f,
               labelSize * 1.24f, withAlpha(colors::kPopoutCyan, a));
    r.fillText(minsCenterX, minsCy, minsLabel, labelSize, TextAlign::Center,
               withAlpha(colors::kBlack, a));
  } else {
    r.fillText(minsCenterX, minsCy, minsLabel, labelSize, TextAlign::Center,
               withAlpha(colors::kPopoutCyan, a));
  }
  const float minsCarrotGap = labelSize * 0.45f;
  drawCarrot(r, minsCenterX - minsToggleW * 0.5f - minsCarrotGap, minsCy,
             labelSize, false, minsBaro, a);
  drawCarrot(r, minsCenterX + minsToggleW * 0.5f + minsCarrotGap, minsCy,
             labelSize, true, !minsBaro, a);

  const float ftLabelW = r.measureTextWidth("FT", smallSize);
  const float minsAltRight = right - ftLabelW - labelSize * 0.35f;
  const float minsAltFt = ui.minimumsAltitudeFt();
  const bool minsAltHighlighted =
      ui.procApproachField() == SoftkeyController::ProcApproachField::MinsAlt;
  if (minsBaro && (minsAltFt > 0.0f || minsAltHighlighted)) {
    const std::string minsAltText = formatInt(minsAltFt);
    const float minsAltW = r.measureTextWidth(minsAltText.c_str(), labelSize);
    const float minsAltLeft = minsAltRight - minsAltW;
    if (minsAltHighlighted && blinkOn && !subListOpen) {
      r.fillRect(minsAltLeft - labelSize * 0.15f, minsCy - labelSize * 0.62f,
                 minsAltW + labelSize * 0.3f, labelSize * 1.24f,
                 withAlpha(colors::kPopoutCyan, a));
      r.fillText(minsAltLeft, minsCy, minsAltText, labelSize, TextAlign::Left,
                 withAlpha(colors::kBlack, a));
    } else {
      r.fillText(minsAltLeft, minsCy, minsAltText, labelSize, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
    }
  } else {
    constexpr int kMinsDashCount = 4;
    drawDashRunRight(r, minsAltRight, minsCy, kMinsDashCount, labelSize,
                     colors::kWhite, a);
  }
  r.fillText(right, minsCy, "FT", smallSize, TextAlign::Right,
             withAlpha(colors::kWhite, a));

  // PRIM FREQ row: dashed placeholders unless an ILS/VOR/NDB primary exists.
  constexpr const char* kPrimFreqLabel = "PRIM FREQ";
  drawFieldLabel(primCy, kPrimFreqLabel, false);
  constexpr int kPrimIdentDashCount = 5;
  const float primFreqLabelW = r.measureTextWidth(kPrimFreqLabel, labelSize);
  const float primValueX =
      std::max(valueX, labelX + primFreqLabelW + labelSize * 0.45f);
  const float freqSlotW = r.measureTextWidth(kFreqDash2, labelSize);
  const float identDashW = dashRunWidth(kPrimIdentDashCount, labelSize);
  const float identDashStart = primValueX + freqSlotW + labelSize * 0.55f;
  const bool showNavFreq = ui.procShowsPrimaryNavFreq();
  const float freq = ui.procPrimaryFreqMhz();
  const std::string ident = ui.procPrimaryIdent();

  if (showNavFreq && freq > 0.0f) {
    char freqBuf[16];
    if (ui.procPrimaryNavIsNdb()) {
      std::snprintf(freqBuf, sizeof(freqBuf), "%.1f", freq);
    } else {
      std::snprintf(freqBuf, sizeof(freqBuf), "%.2f", freq);
    }
    r.fillText(primValueX, primCy, freqBuf, size, TextAlign::Left,
               withAlpha(colors::kPopoutCyan, a));
    if (!ident.empty()) {
      r.fillText(identDashStart, primCy, ident, labelSize, TextAlign::Left,
                 withAlpha(colors::kWhite, a));
    } else {
      drawDashRunRight(r, identDashStart + identDashW, primCy,
                       kPrimIdentDashCount, labelSize, colors::kWhite, a);
    }
  } else {
    r.fillText(primValueX, primCy, kFreqDash2, labelSize, TextAlign::Left,
               withAlpha(colors::kWhite, a));
    drawDashRunRight(r, identDashStart + identDashW, primCy, kPrimIdentDashCount,
                     labelSize, colors::kWhite, a);
  }

  r.strokeLine(f.x + sepInset, btnSepY, f.x + f.w - sepInset, btnSepY, 1.0f,
               withAlpha(colors::kPanelSeparator, a));

  const float loadW = drawDtoStyleButton(
      r, left, buttonsCy, "Load?", size,
      ui.procApproachField() == SoftkeyController::ProcApproachField::Load &&
          ui.procLoadArmed(),
      blinkOn, a);
  const float activateW =
      r.measureTextWidth("Activate?", size) + size * 1.3f;
  drawDtoStyleButton(r, right - activateW, buttonsCy, "Activate?", size,
                     ui.procApproachField() ==
                         SoftkeyController::ProcApproachField::Activate &&
                         ui.procActivateArmed(),
                     blinkOn, a);
  r.fillText(left + loadW + (right - activateW - left - loadW) * 0.5f, buttonsCy,
             "or", smallSize, TextAlign::Center,
             withAlpha(colors::kTitleGray, a));

  if (ui.procSubListOpen()) {
    drawProcSubListPopup(r, f, h, ui, a);
  }
}

}  // namespace

// PFD Procedures window (PROC bezel key, Pilot's Guide 5.8 "Procedures"): the
// lower-right popout the other PFD windows share. The top-level menu lists the
// approach-activation items (disabled in this suite) and the Select Approach /
// Arrival / Departure items; selecting one switches the window to the
// procedure-selection list (procedure names, then transitions) that loads the
// chosen procedure into the active flight plan.
void drawProcWindow(Renderer& r, float w, float h, const Layout& L,
                    const SoftkeyController& ui) {
  const float rawAnim = ui.windowAnim(PfdWindow::Procedures);
  if (rawAnim <= 0.0f) return;

  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f = drawWindowFrame(r, w, h, L, rawAnim,
                                          ui.procWindowTitle(), panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  // Shared list geometry with the Active Flight Plan window (WT .fpl layout).
  const float size = fontPx(wt::kInfoValue, h);
  const float smallSize = size * 0.72f;
  const float padX = fontPx(8.0f, h);
  const float scrollW = fontPx(10.0f, h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX;
  const float listW = listRight - listLeft;
  const float identX = listLeft + listW * 0.04f;
  const float rowH = fontPx(29.0f, h);
  const bool blinkOn = ui.blinkOn();

  if (!ui.procSelectMode()) {
    // Top-level menu: the six Procedures items (WT shows all rows without
    // scrolling; shrink row pitch only if the viewport is unexpectedly tight).
    const int n = ui.procMenuItemCount();
    const float listTop = f.contentTop + size * kWtProcMenuListTopPadFrac;
    const float listBottom = f.top + f.h - padX;
    const float listH = std::max(1.0f, listBottom - listTop);
    const float menuRowH =
        std::min(rowH, listH / static_cast<float>(std::max(1, n)));
    const int selected =
        std::min(std::max(0, ui.procMenuSelected()), std::max(0, n - 1));
    for (int i = 0; i < n; ++i) {
      const float rowCy = listTop + menuRowH * (static_cast<float>(i) + 0.5f);
      drawProcMenuRow(r, identX, rowCy, ui.procMenuItemText(i), size,
                      i == selected, ui.procMenuItemEnabled(i), blinkOn, a);
    }
    return;
  }

  if (ui.procCategory() == ProcedureType::Approach) {
    drawApproachSelectWindow(r, f, w, h, ui);
    return;
  }

  // Departure / arrival selection sub-window: airport header, separator, then
  // the procedure / transition list.
  const std::string icao = ui.procAirportIcao();
  float cy = f.contentTop + size * 0.6f;
  r.fillText(identX, cy, icao.empty() ? "_____" : icao, size, TextAlign::Left,
             withAlpha(colors::kPopoutCyan, a));
  if (ui.procStep() == SoftkeyController::ProcStep::TransitionList) {
    r.fillText(listRight, cy, ui.procSelectedName(), size, TextAlign::Right,
               withAlpha(colors::kPopoutCyan, a));
  }
  const float sepY = cy + size * 0.55f;
  r.strokeLine(listLeft, sepY, listRight, sepY, 1.0f,
               withAlpha(colors::kPanelSeparator, a));
  cy = sepY + size * 0.3f;

  const std::vector<std::string> items = ui.procListItems();
  if (items.empty()) {
    r.fillText(f.x + f.w * 0.5f, cy + rowH * 0.5f, "NO PROCEDURES", smallSize,
               TextAlign::Center, withAlpha(colors::kTitleGray, a));
    return;
  }

  const int total = static_cast<int>(items.size());
  const int selected = std::min(std::max(0, ui.procListSelected()), total - 1);
  const int visible = std::max(1, static_cast<int>((f.top + f.h - padX - cy) / rowH));
  int first = 0;
  if (total > visible) {
    first = std::max(0, std::min(selected - visible / 2, total - visible));
  }
  const int end = std::min(total, first + visible);
  const bool scrolling = total > visible;
  for (int i = first; i < end; ++i) {
    const float rowCy = cy + rowH * (static_cast<float>(i - first) + 0.5f);
    drawProcMenuRow(r, identX, rowCy, items[static_cast<std::size_t>(i)], size,
                    i == selected, true, blinkOn, a);
  }
  if (scrolling) {
    const float trackTop = cy;
    const float trackH = rowH * static_cast<float>(visible);
    drawProcScrollBar(r, f.x + f.w - padX - scrollW, scrollW, trackTop, trackH,
                      total, visible, first, a);
  }
}

}  // namespace avionics::pfd
