#include <cstdio>
#include <cmath>
#include <string>

#include "render/pfd/ChromeInternal.h"

#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {
namespace {

// Placeholder dash counts (WT WaypointInputStore / PFDDirectTo.tsx).
constexpr int kDtoCityDashCount = 16;
constexpr int kDtoNameDashCount = 15;
constexpr int kDtoAltDashCount = 5;

// Trainer-measured row centers within the 310x220 popout (panel-top Y, px).
// Pixel-scanned from PFD Direct To.bmp / Filled Out.bmp on the GARMIN unit.
constexpr float kDtoIdentRowPx = 50.0f;
constexpr float kDtoNameToSepPx = 20.0f;
constexpr float kDtoWptSepTightPx = 22.0f;  // ident-only (no facility name row)
constexpr float kDtoRowGapPx = 7.0f;
constexpr float kDtoAltBelowWptSepPx = 20.0f;
constexpr float kDtoAltSepBelowWptSepPx = 31.0f;
// Vertical spacing between BRG/DIS and CRS row centers in the location section.
constexpr float kDtoLocRowStepPx = 27.0f;
constexpr float kDtoMinBrgGapBelowAltSepPx = 8.0f;
constexpr float kDtoButtonBottomPx = 8.0f;
constexpr float kDtoLocBottomPadPx = 8.0f;
constexpr float kDtoButtonGapAbovePx = 6.0f;
constexpr float kDtoCityStartPx = 142.0f;

void drawDtoSeparator(Renderer& r, float panelX, float panelW, float y, float h,
                      float a) {
  const float sepInset = fontPx(kWtPopoutBorderPx, h) + panelW * 0.01f;
  r.fillRect(panelX + sepInset, y, panelW - 2.0f * sepInset, 1.0f,
             withAlpha(colors::kWhite, a));
}

struct DashStyle {
  float width;
  float advance;
  float height;
};

DashStyle dashStyle(float size) {
  return {size * 0.36f, size * 0.48f, size * 0.09f};
}

float textCyForDashBottom(Renderer& r, float dashCy, const DashStyle& ds,
                          float textSize) {
  const float dashBottom = dashCy + ds.height * 0.5f;
  const TextRect tr =
      r.measureTextRect(0.0f, 0.0f, "X", textSize, TextAlign::Left);
  return dashBottom - tr.bottom;
}

void drawTightDash(Renderer& r, float x, float cy, const DashStyle& ds,
                   const Color& color) {
  r.fillRect(x, cy - ds.height * 0.5f, ds.width, ds.height, color);
}

float drawTightDashRun(Renderer& r, float x, float dashCy, int count, float size,
                       const Color& color) {
  const DashStyle ds = dashStyle(size);
  for (int i = 0; i < count; ++i) {
    drawTightDash(r, x + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
    x += ds.advance;
  }
  return x;
}

float drawDtoIdentCells(Renderer& r, float startX, float dashCy,
                        const std::string& ident, int cursor, int typedCount,
                        bool selectAll, bool blinkOn, float size) {
  const float tracking = size * 0.06f;
  const DashStyle ds = dashStyle(size);
  const float entryTextCy = textCyForDashBottom(r, dashCy, ds, size);
  const TextRect entryRect =
      r.measureTextRect(0.0f, entryTextCy, "X", size, TextAlign::Left);
  const float plateTop = entryRect.top;
  const float plateH = dashCy + ds.height * 0.5f - plateTop + 1.0f;
  float cx = startX;
  for (int i = 0; i < FmsWaypointEntry::kMaxChars; ++i) {
    const char ch = i < static_cast<int>(ident.size()) ? ident[i] : '_';
    const bool isBlank = ch == '_';
    const bool highlightAll = selectAll && !isBlank;
    const bool isCursor = !selectAll && i == cursor;
    const bool cursorOn = isCursor && blinkOn;
    if (isBlank) {
      if (highlightAll || cursorOn) {
        r.fillRect(cx, plateTop, ds.advance, plateH, colors::kPopoutCyan);
      }
      const Color dashColor =
          highlightAll || cursorOn ? colors::kBlack
          : isCursor        ? colors::kPopoutCyan
          : i >= typedCount ? colors::kPopoutCyan
                            : colors::kWhite;
      drawTightDash(r, cx + (ds.advance - ds.width) * 0.5f, dashCy, ds, dashColor);
      cx += ds.advance;
      continue;
    }
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, size);
    if (highlightAll || cursorOn) {
      r.fillRect(cx - tracking * 0.5f, plateTop, chW + tracking, plateH,
                 colors::kPopoutCyan);
    }
    const Color color = highlightAll || cursorOn ? colors::kBlack
                        : isCursor        ? colors::kPopoutCyan
                        : i >= typedCount ? colors::kPopoutCyan
                                          : colors::kWhite;
    r.fillText(cx, entryTextCy, text, size, TextAlign::Left, color);
    cx += chW + tracking;
  }
  return cx;
}

float drawDtoButton(Renderer& r, float leftX, float cy, const char* label,
                    float size, bool armed, bool blinkOn) {
  const float bw = r.measureTextWidth(label, size) + size * 1.3f;
  const float bh = size * 1.7f;
  const float bx = leftX;
  const float by = cy - bh * 0.5f;
  const float radius = bh * 0.32f;
  if (armed && blinkOn) {
    r.fillRoundedRect(bx, by, bw, bh, radius, colors::kPopoutCyan);
  }
  r.strokeRoundedRect(bx, by, bw, bh, radius, 1.5f,
                      armed ? colors::kWhite : colors::kGroupBoxBorder);
  const Color textColor =
      armed ? (blinkOn ? colors::kBlack : colors::kPopoutCyan) : colors::kWhite;
  r.fillText(leftX + bw * 0.5f, cy, label, size, TextAlign::Center, textColor);
  return bw;
}

float locationRowTop(Renderer& r, float dashCy, float labelSize) {
  const float labelCy =
      textCyForDashBottom(r, dashCy, dashStyle(labelSize), labelSize);
  return r.measureTextRect(0.0f, labelCy, "BRG", labelSize, TextAlign::Left).top;
}

// Place a second dash row below `aboveDashCy` without overlapping its text.
float rowDashCyBelow(Renderer& r, float aboveDashCy, float aboveSize,
                     float belowSize, float gapPx, float h) {
  const DashStyle aboveDs = dashStyle(aboveSize);
  const float aboveTextCy =
      textCyForDashBottom(r, aboveDashCy, aboveDs, aboveSize);
  const float aboveBottom =
      r.measureTextRect(0.0f, aboveTextCy, "X", aboveSize, TextAlign::Left)
          .bottom;
  const float gap = fontPx(gapPx, h);
  float belowDashCy = aboveDashCy + fontPx(16.0f, h);
  const DashStyle belowDs = dashStyle(belowSize);
  for (int i = 0; i < 6; ++i) {
    const float belowTextCy =
        textCyForDashBottom(r, belowDashCy, belowDs, belowSize);
    const float belowTop =
        r.measureTextRect(0.0f, belowTextCy, "X", belowSize, TextAlign::Left)
            .top;
    const float need = (aboveBottom + gap) - belowTop;
    if (std::abs(need) < 0.25f) break;
    belowDashCy += need;
  }
  return belowDashCy;
}

// Center the BRG/DIS and CRS rows as a block in the location section.
void locationRowDashCys(Renderer& r, float altSepY, float locBottom, float h,
                        float labelSize, float readoutSize, float& brgDashCy,
                        float& crsDashCy) {
  const float locTop = altSepY + fontPx(1.0f, h);
  const float rowStep = fontPx(kDtoLocRowStepPx, h);

  const auto crsRowBottom = [&](float crsCy) {
    const float readoutCy =
        textCyForDashBottom(r, crsCy, dashStyle(readoutSize), readoutSize);
    return r.measureTextRect(0.0f, readoutCy, "360", readoutSize, TextAlign::Left)
        .bottom;
  };

  const float probeBrg = locTop + fontPx(20.0f, h);
  const float probeCrs = probeBrg + rowStep;
  const float blockH = crsRowBottom(probeCrs) - locationRowTop(r, probeBrg, labelSize);
  const float blockTop = (locTop + locBottom - blockH) * 0.5f;

  brgDashCy = probeBrg;
  for (int i = 0; i < 4; ++i) {
    brgDashCy += blockTop - locationRowTop(r, brgDashCy, labelSize);
  }
  crsDashCy = brgDashCy + rowStep;

  const float minTop = altSepY + fontPx(kDtoMinBrgGapBelowAltSepPx, h);
  const float shiftUp = locationRowTop(r, brgDashCy, labelSize) - minTop;
  if (shiftUp < 0.0f) {
    brgDashCy -= shiftUp;
    crsDashCy -= shiftUp;
  }
  const float crsBottom = crsRowBottom(crsDashCy);
  if (crsBottom > locBottom) {
    const float shift = crsBottom - locBottom;
    brgDashCy -= shift;
    crsDashCy -= shift;
  }
}

}  // namespace

void drawDirectToWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.directToWindowAnim(), "Direct To",
                      panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  r.save();
  r.globalAlpha(a);

  const bool hasMatch = ui.directToHasMatch();
  const bool entryActive = ui.directToEntryActive();
  const bool blinkOn = ui.blinkOn();

  const float identSize = fontPx(18.0f, h);
  const float faceSize = fontPx(16.0f, h);
  const float labelSize = fontPx(14.0f, h);
  const float valueSize = fontPx(18.0f, h);
  const float readoutSize = fontPx(18.0f, h);
  const float smallSize = fontPx(14.0f, h);

  const float pad = f.w * 0.045f;
  const float left = f.x + pad;
  const float right = f.x + f.w - pad;
  const float innerW = right - left;

  const float identDashCy = f.top + fontPx(kDtoIdentRowPx, h);
  const bool showNameRow =
      hasMatch && !ui.directToNotFound() && !ui.directToMatch().name.empty();
  const bool needsSecondRow =
      showNameRow || !hasMatch || ui.directToNotFound();
  const float nameDashCy =
      needsSecondRow ? rowDashCyBelow(r, identDashCy, identSize, faceSize,
                                      kDtoRowGapPx, h)
                     : identDashCy;
  const float wptSepY =
      needsSecondRow ? nameDashCy + fontPx(kDtoNameToSepPx, h)
                     : identDashCy + fontPx(kDtoWptSepTightPx, h);
  const float altDashCy = wptSepY + fontPx(kDtoAltBelowWptSepPx, h);
  const float altSepY = wptSepY + fontPx(kDtoAltSepBelowWptSepPx, h);
  const float buttonH = valueSize * 1.7f;
  const float buttonsCy =
      f.top + panelH - fontPx(kDtoButtonBottomPx, h) - buttonH * 0.5f;
  const float locBottom =
      hasMatch ? buttonsCy - buttonH * 0.5f - fontPx(kDtoButtonGapAbovePx, h)
               : f.top + panelH - fontPx(kDtoLocBottomPadPx, h);
  float brgDashCy = 0.0f;
  float crsDashCy = 0.0f;
  locationRowDashCys(r, altSepY, locBottom, h, labelSize, readoutSize, brgDashCy,
                       crsDashCy);
  const float cityStartX = f.x + fontPx(kDtoCityStartPx, h);

  const DashStyle identDs = dashStyle(identSize);
  const float identTextCy = textCyForDashBottom(r, identDashCy, identDs, identSize);
  const DashStyle nameDs = dashStyle(faceSize);
  const float nameTextCy = textCyForDashBottom(r, nameDashCy, nameDs, faceSize);

  // ---- Ident / Facility / City ----
  {
    const float ix = left + pad * 0.5f;
    const bool armed = ui.directToArmed();
    float identEnd = ix;
    if (armed && hasMatch) {
      const std::string& id = ui.directToIdent();
      const float tracking = identSize * 0.06f;
      const float tw = r.measureTextWidth(id.c_str(), identSize);
      const TextRect entryRect =
          r.measureTextRect(0.0f, identTextCy, id.c_str(), identSize,
                            TextAlign::Left);
      const float plateH =
          identDashCy + identDs.height * 0.5f - entryRect.top + fontPx(1.0f, h);
      r.fillRect(ix - tracking * 0.5f, entryRect.top, tw + tracking, plateH,
                 colors::kPopoutCyan);
      r.fillText(ix, identTextCy, id.c_str(), identSize, TextAlign::Left,
                 colors::kBlack);
      identEnd = ix + tw + tracking;
    } else {
      const int cursor = entryActive ? ui.directToCursor() : -1;
      identEnd = drawDtoIdentCells(r, ix, identDashCy, ui.directToIdent(), cursor,
                                   ui.directToTypedCount(),
                                   ui.directToSelectAll(), blinkOn, identSize);
    }
    if (hasMatch) {
      const MapFeature& wpt = ui.directToMatch();
      const float symR = fontPx(15.0f, h) * 0.6f;
      const float symCx = identEnd + fontPx(16.0f, h);
      drawUiWaypointIcon(r, wpt, symCx, identTextCy, symR);
      std::string loc = wpt.city;
      if (wpt.region.size() == 2 &&
          std::isalpha(static_cast<unsigned char>(wpt.region[0])) &&
          std::isalpha(static_cast<unsigned char>(wpt.region[1]))) {
        loc += loc.empty() ? wpt.region : " " + wpt.region;
      }
      if (!loc.empty()) {
        r.fillText(symCx + symR + fontPx(12.0f, h), identTextCy, loc, faceSize,
                   TextAlign::Left, colors::kPopoutCyan);
      }
    } else {
      drawTightDashRun(r, cityStartX, identDashCy, kDtoCityDashCount, faceSize,
                       colors::kPopoutCyan);
    }
    if (ui.directToNotFound()) {
      r.fillText(ix, nameTextCy, "WAYPOINT NOT FOUND", faceSize, TextAlign::Left,
                 colors::kBandYellow);
    } else if (hasMatch) {
      const MapFeature& wpt = ui.directToMatch();
      if (!wpt.name.empty()) {
        r.fillText(ix, nameTextCy, wpt.name, faceSize, TextAlign::Left,
                   colors::kPopoutCyan);
      }
    } else {
      drawTightDashRun(r, ix, nameDashCy, kDtoNameDashCount, faceSize,
                       colors::kPopoutCyan);
    }
  }

  // ---- ALT / Offset ----
  {
    const DashStyle altDs = dashStyle(valueSize);
    const float altLabelCy =
        textCyForDashBottom(r, altDashCy, dashStyle(labelSize), labelSize);
    const float altValueCy = textCyForDashBottom(r, altDashCy, altDs, valueSize);
    const float altSmallCy =
        textCyForDashBottom(r, altDashCy, dashStyle(smallSize), smallSize);
    float x = left + pad * 0.5f;
    x = putText(r, x, altLabelCy, "ALT", labelSize, colors::kTitleGray, 0.4f);
    x = drawTightDashRun(r, x, altDashCy, kDtoAltDashCount, valueSize,
                         colors::kPopoutCyan);
    putText(r, x, altSmallCy, "FT", smallSize, colors::kPopoutCyan, 0.0f);
    float ox = left + innerW * 0.52f;
    ox = putText(r, ox, altLabelCy, "Offset", labelSize, colors::kTitleGray,
                 0.4f);
    const char* offsetNm = "NM";
    const float offsetNmW = r.measureTextWidth(offsetNm, smallSize);
    r.fillText(right - pad * 0.5f, altSmallCy, offsetNm, smallSize,
               TextAlign::Right, colors::kPopoutCyan);
    r.fillText(right - pad * 0.5f - offsetNmW, altValueCy, "+0", valueSize,
               TextAlign::Right, colors::kPopoutCyan);
  }

  // ---- BRG / DIS ----
  char buf[24];
  const bool hasGeo = ui.directToHasGeo();
  {
    const float brgLabelCy =
        textCyForDashBottom(r, brgDashCy, dashStyle(labelSize), labelSize);
    const float brgReadoutCy =
        textCyForDashBottom(r, brgDashCy, dashStyle(readoutSize), readoutSize);
    const float brgSmallCy =
        textCyForDashBottom(r, brgDashCy, dashStyle(smallSize), smallSize);
    float x = left + pad * 0.5f;
    x = putText(r, x, brgLabelCy, "BRG", labelSize, colors::kTitleGray, 0.35f);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0", ui.directToBearingDeg());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", "360\u00b0");
    }
    putText(r, x, brgReadoutCy, buf, readoutSize, colors::kWhite, 0.0f);
    float dx = left + innerW * 0.52f;
    dx = putText(r, dx, brgLabelCy, "DIS", labelSize, colors::kTitleGray, 0.35f);
    const char* disNm = "NM";
    const float disNmW = r.measureTextWidth(disNm, smallSize);
    r.fillText(right - pad * 0.5f, brgSmallCy, disNm, smallSize, TextAlign::Right,
               colors::kWhite);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%.1f", ui.directToDistanceNm());
      r.fillText(right - pad * 0.5f - disNmW, brgReadoutCy, buf, readoutSize,
                 TextAlign::Right, colors::kWhite);
    } else {
      const DashStyle ds = dashStyle(readoutSize);
      float disX = right - pad * 0.5f - disNmW;
      disX -= ds.advance * 3.0f;
      drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, brgDashCy, ds,
                    colors::kWhite);
      disX += ds.advance;
      drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, brgDashCy, ds,
                    colors::kWhite);
      disX += ds.advance;
      r.fillText(disX, brgReadoutCy, ".", readoutSize, TextAlign::Left,
                 colors::kWhite);
      disX += r.measureTextWidth(".", readoutSize) * 0.55f;
      drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, brgDashCy, ds,
                    colors::kWhite);
    }
  }

  // ---- CRS ----
  {
    const float crsLabelCy =
        textCyForDashBottom(r, crsDashCy, dashStyle(labelSize), labelSize);
    const float crsReadoutCy =
        textCyForDashBottom(r, crsDashCy, dashStyle(readoutSize), readoutSize);
    float x = left + pad * 0.5f;
    x = putText(r, x, crsLabelCy, "CRS", labelSize, colors::kTitleGray, 0.35f);
    if (hasGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0", ui.directToBearingDeg());
    } else {
      std::snprintf(buf, sizeof(buf), "%s", "360\u00b0");
    }
    putText(r, x, crsReadoutCy, buf, readoutSize, colors::kPopoutCyan, 0.0f);
  }

  if (hasMatch) {
    drawDtoButton(r, left, buttonsCy, "Activate?", valueSize, ui.directToArmed(),
                  blinkOn);
    const float holdW =
        r.measureTextWidth("Hold?", valueSize) + valueSize * 1.3f;
    drawDtoButton(r, right - holdW, buttonsCy, "Hold?", valueSize, false,
                  blinkOn);
  }

  drawDtoSeparator(r, f.x, f.w, wptSepY, h, a);
  drawDtoSeparator(r, f.x, f.w, altSepY, h, a);

  r.restore();
}

}  // namespace avionics::pfd
