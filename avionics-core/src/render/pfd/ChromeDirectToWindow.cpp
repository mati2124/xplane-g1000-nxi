#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {
namespace {

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

}  // namespace

// PFD Direct-To window (Direct-To bezel key, Pilot's Guide Fig. 5-45 "Direct-to
// Window - PFD"): a lower-right popout with the shared window chrome whose body
// stacks the Ident/Facility/City box, an ALT/Offset (VNV) box, a BRG/DIS
// (Location) box, the CRS desired course, and the Activate? / Hold? buttons.
void drawDirectToWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  // Taller 310-wide popout: reaches up near the altimeter BARO box so the five
  // stacked sections plus the command buttons fit (Pilot's Guide Fig. 5-45).
  float panelW = 0.0f;
  float panelH = 0.0f;
  tallPopoutPanelSize(w, h, panelW, panelH);
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
  // Tight inter-section spacing so all five sections plus the command buttons
  // clear each other within the popout body.
  const float gap = fontPx(6.0f, h);

  // Buttons anchored to the bottom of the body.
  const float buttonH = valueSize * 1.7f;
  const float buttonsCy = f.top + f.h - buttonH * 0.5f - fontPx(8.0f, h);

  float y = f.contentTop + fontPx(4.0f, h);

  // ---- Ident / Facility / City box ----
  const float identBoxH = identSize + faceSize * 1.3f + fontPx(14.0f, h);
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
    x = putText(r, x, cy, "_____", valueSize, colors::kCyan, 0.1f);
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

}  // namespace avionics::pfd
