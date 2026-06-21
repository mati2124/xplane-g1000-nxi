#include <cmath>
#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

#include "avionics/NavMath.h"
#include "avionics/render/MapSymbols.h"

namespace avionics::pfd {
namespace {

// PFD Waypoint Information popout (Pilot's Guide Fig. 5-48 insert window).
constexpr int kCityDashCount = 16;
constexpr int kNameDashCount = 15;

constexpr float kIdentRowPx = 50.0f;
constexpr float kNameRowPx = 72.0f;
constexpr float kSepPx = 95.0f;
constexpr float kBrgRowPx = 118.0f;
constexpr float kLatRowPx = 148.0f;
constexpr float kLonRowPx = 168.0f;
constexpr float kPromptRowPx = 202.0f;
constexpr float kCityStartPx = 142.0f;

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

void drawSeparator(Renderer& r, float panelX, float panelW, float y, float h,
                   float a) {
  const float sepInset = fontPx(kWtPopoutBorderPx, h) + panelW * 0.01f;
  r.fillRect(panelX + sepInset, y, panelW - 2.0f * sepInset, 1.0f,
             withAlpha(colors::kWhite, a));
}

float drawIdentCells(Renderer& r, float startX, float dashCy,
                     const std::string& ident, int cursor, int typedCount,
                     bool blinkOn, float size) {
  const float tracking = size * 0.06f;
  const DashStyle ds = dashStyle(size);
  const float entryTextCy = textCyForDashBottom(r, dashCy, ds, size);
  const TextRect entryRect =
      r.measureTextRect(0.0f, entryTextCy, "X", size, TextAlign::Left);
  const float plateTop = entryRect.top;
  const float plateH = dashCy + ds.height * 0.5f - plateTop + 1.0f;
  float cx = startX;
  for (int i = 0; i < FmsWaypointEntry::kMaxChars; ++i) {
    const bool isCursor = i == cursor;
    const bool cursorOn = isCursor && blinkOn;
    const char ch = i < static_cast<int>(ident.size()) ? ident[i] : '_';
    const bool isBlank = ch == '_';
    if (isBlank) {
      if (cursorOn) {
        r.fillRect(cx, plateTop, ds.advance, plateH, colors::kPopoutCyan);
      }
      const Color dashColor =
          cursorOn          ? colors::kBlack
          : isCursor        ? colors::kPopoutCyan
          : i >= typedCount ? colors::kPopoutCyan
                            : colors::kWhite;
      drawTightDash(r, cx + (ds.advance - ds.width) * 0.5f, dashCy, ds, dashColor);
      cx += ds.advance;
      continue;
    }
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, size);
    if (cursorOn) {
      r.fillRect(cx - tracking * 0.5f, plateTop, chW + tracking, plateH,
                 colors::kPopoutCyan);
    }
    const Color color = cursorOn          ? colors::kBlack
                        : isCursor        ? colors::kPopoutCyan
                        : i >= typedCount ? colors::kPopoutCyan
                                          : colors::kWhite;
    r.fillText(cx, entryTextCy, text, size, TextAlign::Left, color);
    cx += chW + tracking;
  }
  return cx;
}

std::string formatLatLon(double value, bool isLat) {
  const char hemi = isLat ? (value >= 0.0 ? 'N' : 'S')
                          : (value >= 0.0 ? 'E' : 'W');
  const double a = std::fabs(value);
  const int deg = static_cast<int>(a);
  const double minutes = (a - deg) * 60.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%c%0*d%s%05.2f'", hemi, isLat ? 2 : 3, deg,
                "\xC2\xB0", minutes);
  return buf;
}

void drawCoordPlaceholder(Renderer& r, float rightX, float dashCy, float size,
                          const Color& color) {
  const DashStyle ds = dashStyle(size);
  const float readoutCy = textCyForDashBottom(r, dashCy, ds, size);
  float x = rightX;
  for (int i = 0; i < 2; ++i) {
    x -= ds.advance;
    drawTightDash(r, x + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
  }
  x -= size * 0.18f;
  r.fillText(x, readoutCy, "\xC2\xB0", size, TextAlign::Right, color);
  x -= size * 0.22f;
  for (int i = 0; i < 2; ++i) {
    x -= ds.advance;
    drawTightDash(r, x + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
  }
  x -= size * 0.12f;
  r.fillText(x, readoutCy, ".", size, TextAlign::Right, color);
  x -= size * 0.12f;
  for (int i = 0; i < 2; ++i) {
    x -= ds.advance;
    drawTightDash(r, x + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
  }
  x -= size * 0.10f;
  r.fillText(x, readoutCy, "'", size, TextAlign::Right, color);
}

}  // namespace

void drawWaypointInformationWindow(Renderer& r, float w, float h, const Layout& L,
                                   const SoftkeyController& ui) {
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::FlightPlan),
                      "Waypoint Information", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  r.save();
  r.globalAlpha(a);

  const bool hasMatch = ui.flightPlanEntryHasMatch();
  const bool blinkOn = ui.blinkOn();

  const float identSize = fontPx(18.0f, h);
  const float faceSize = fontPx(16.0f, h);
  const float labelSize = fontPx(14.0f, h);
  const float readoutSize = fontPx(18.0f, h);
  const float smallSize = fontPx(14.0f, h);
  const float promptSize = fontPx(14.0f, h);

  const float pad = f.w * 0.045f;
  const float left = f.x + pad;
  const float right = f.x + f.w - pad;
  const float innerW = right - left;

  const float identDashCy = f.top + fontPx(kIdentRowPx, h);
  const DashStyle identDs = dashStyle(identSize);
  const float identTextCy = textCyForDashBottom(r, identDashCy, identDs, identSize);
  const DashStyle nameDs = dashStyle(faceSize);
  const float nameDashCy = f.top + fontPx(kNameRowPx, h);
  const float nameTextCy = textCyForDashBottom(r, nameDashCy, nameDs, faceSize);
  const float sepY = f.top + fontPx(kSepPx, h);
  const float brgDashCy = f.top + fontPx(kBrgRowPx, h);
  const float latDashCy = f.top + fontPx(kLatRowPx, h);
  const float lonDashCy = f.top + fontPx(kLonRowPx, h);
  const float promptCy = f.top + fontPx(kPromptRowPx, h);
  const float cityStartX = f.x + fontPx(kCityStartPx, h);

  // ---- Ident / facility / city ----
  {
    const float ix = left + pad * 0.5f;
    const float identEnd =
        drawIdentCells(r, ix, identDashCy, ui.flightPlanEntryIdent(),
                       ui.flightPlanEntryCursor(), ui.flightPlanEntryTypedCount(),
                       blinkOn, identSize);
    if (hasMatch) {
      const MapFeature& wpt = ui.flightPlanEntryMatch();
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
      drawTightDashRun(r, cityStartX, identDashCy, kCityDashCount, faceSize,
                       colors::kPopoutCyan);
    }
    if (ui.flightPlanEntryNotFound()) {
      r.fillText(ix, nameTextCy, "WAYPOINT NOT FOUND", faceSize, TextAlign::Left,
                 colors::kBandYellow);
    } else if (hasMatch) {
      const MapFeature& wpt = ui.flightPlanEntryMatch();
      if (!wpt.name.empty()) {
        r.fillText(ix, nameTextCy, wpt.name, faceSize, TextAlign::Left,
                   colors::kPopoutCyan);
      }
    } else {
      drawTightDashRun(r, ix, nameDashCy, kNameDashCount, faceSize,
                       colors::kPopoutCyan);
    }
  }

  drawSeparator(r, f.x, f.w, sepY, h, a);

  // ---- BRG / DIS + coordinates ----
  char buf[24];
  const bool hasGeo = ui.flightPlanEntryHasGeo();
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
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0",
                    ui.flightPlanEntryBearingDeg());
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
      std::snprintf(buf, sizeof(buf), "%.1f", ui.flightPlanEntryDistanceNm());
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

  if (hasMatch) {
    const MapFeature& wpt = ui.flightPlanEntryMatch();
    const float latCy =
        textCyForDashBottom(r, latDashCy, dashStyle(readoutSize), readoutSize);
    const float lonCy =
        textCyForDashBottom(r, lonDashCy, dashStyle(readoutSize), readoutSize);
    r.fillText(right - pad * 0.5f, latCy, formatLatLon(wpt.lat, true), readoutSize,
               TextAlign::Right, colors::kWhite);
    r.fillText(right - pad * 0.5f, lonCy, formatLatLon(wpt.lon, false),
               readoutSize, TextAlign::Right, colors::kWhite);
  } else {
    drawCoordPlaceholder(r, right - pad * 0.5f, latDashCy, readoutSize,
                         colors::kWhite);
    drawCoordPlaceholder(r, right - pad * 0.5f, lonDashCy, readoutSize,
                         colors::kWhite);
  }

  r.fillText(f.x + f.w * 0.5f, promptCy, "Press \"ENT\" to accept", promptSize,
             TextAlign::Center, colors::kWhite);

  r.restore();
}

}  // namespace avionics::pfd
