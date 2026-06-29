#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/NavMath.h"
#include "avionics/ProcedureSupport.h"
#include "avionics/render/CursorHighlight.h"
#include "avionics/FplRouteEdit.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/ChromeInternal.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics::pfd {
namespace {

// Centered modal confirmation box (Remove <wpt>? / Delete Flight Plan?) with an
// OK / CANCEL pair; the highlighted choice gets the cyan select plate.
void drawFplConfirm(Renderer& r, const WindowFrame& f, float size,
                    const std::string& line1, const std::string& line2,
                    bool okSelected, float a, const std::string& line3 = {}) {
  const bool threeLine = !line3.empty();
  const float boxW = f.w * 0.82f;
  const float boxH = size * (threeLine ? 7.3f : 6.2f);
  const float bx = f.x + (f.w - boxW) * 0.5f;
  const float by = f.top + (f.h - boxH) * 0.5f;
  const float radius = size * 0.4f;
  r.fillRoundedRect(bx, by, boxW, boxH, radius,
                    withAlpha(colors::kMfdPanelGray, a));
  r.strokeRoundedRect(bx, by, boxW, boxH, radius, 1.5f,
                      withAlpha(colors::kMenuBorderGray, a));

  const float cx = bx + boxW * 0.5f;
  // Keep long procedure subjects (e.g. "SKNNR-RW08B.BANNG3.LUCKK") inside the
  // box by shrinking any line that would overrun the available text width.
  const float maxTextW = boxW - size * 1.0f;
  const auto fitSize = [&](const std::string& s) {
    const float w = r.measureTextWidth(s, size);
    if (w <= maxTextW || w <= 0.0f) return size;
    return size * (maxTextW / w);
  };
  r.fillText(cx, by + size * 1.2f, line1, fitSize(line1), TextAlign::Center,
             withAlpha(colors::kWhite, a));
  if (!line2.empty()) {
    r.fillText(cx, by + size * 2.3f, line2, fitSize(line2), TextAlign::Center,
               withAlpha(colors::kWhite, a));
  }
  if (threeLine) {
    r.fillText(cx, by + size * 3.4f, line3, fitSize(line3), TextAlign::Center,
               withAlpha(colors::kWhite, a));
  }

  // OK / CANCEL buttons.
  const float btnCy = by + boxH - size * 1.3f;
  const float gap = size * 1.0f;
  const float okW = r.measureTextWidth("OK", size) + size * 1.4f;
  const float canW = r.measureTextWidth("CANCEL", size) + size * 1.4f;
  const float totalW = okW + gap + canW;
  float x = cx - totalW * 0.5f;
  const float btnH = size * 1.7f;
  const auto button = [&](float left, float width, const char* label,
                          bool selected) {
    if (selected) {
      r.fillRoundedRect(left, btnCy - btnH * 0.5f, width, btnH, btnH * 0.32f,
                        withAlpha(colors::kPopoutCyan, a));
    }
    r.strokeRoundedRect(left, btnCy - btnH * 0.5f, width, btnH, btnH * 0.32f,
                        1.2f, withAlpha(colors::kGroupBoxBorder, a));
    r.fillText(left + width * 0.5f, btnCy, label, size, TextAlign::Center,
               withAlpha(selected ? colors::kBlack : colors::kWhite, a));
  };
  button(x, okW, "OK", okSelected);
  x += okW + gap;
  button(x, canW, "CANCEL", !okSelected);
}

// Tight underscore dashes (same glyphs as the Direct-To window).
constexpr int kFplDashCount = 5;
constexpr int kFplOriginDashCount = 4;
constexpr int kFplApproachSepDashCount = 7;

constexpr char kDeg[] = "\xC2\xB0";  // UTF-8 degree sign

bool fplRowIdentBlink(bool showSelection, bool showActive, int legIdx,
                      int activeLegIdx, int cursorLegIdx, bool cursorOn,
                      int listCursorRow, int activeSelectableRow, bool blinkOn) {
  if (showSelection) return blinkOn;
  if (!showActive) return false;
  return fplActiveNavRowBlink(legIdx, activeLegIdx, cursorLegIdx, cursorOn,
                              listCursorRow, activeSelectableRow) &&
         blinkOn;
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

bool labelIsRnavGps(const std::string& label) {
  return label.size() >= 4 && label.compare(0, 4, "RNAV") == 0;
}

float rnavGpsSuffixGap(float size) { return size * 0.08f; }

// Trainer FPL after load: KFMY-RNAV GPS 05 LPV (no dash between RNAV and GPS).
void drawFplApproachHeader(Renderer& r, float x, float cy, const std::string& icao,
                           const std::string& label, float size, const Color& color,
                           float a, float maxWidth = 0.0f) {
  const std::string prefix = icao + "-";
  // Shrink the whole header uniformly when it would run past the list edge /
  // scroll bar (long procedure subjects like "SKNNR-RW08B.BANNG3.LUCKK").
  if (maxWidth > 0.0f) {
    float full = r.measureTextWidth(prefix, size);
    if (labelIsRnavGps(label)) {
      const float subSize = size * 0.72f;
      full += r.measureTextWidth("RNAV", size) + rnavGpsSuffixGap(size) +
              r.measureTextWidth("GPS", subSize);
      const std::string rest = restAfterRnavGps(label);
      if (!rest.empty()) full += r.measureTextWidth(" " + rest, size);
    } else {
      full += r.measureTextWidth(label, size);
    }
    if (full > maxWidth && full > 0.0f) size *= maxWidth / full;
  }
  float xx = x;
  r.fillText(xx, cy, prefix, size, TextAlign::Left, withAlpha(color, a));
  xx += r.measureTextWidth(prefix, size);
  if (labelIsRnavGps(label)) {
    const float subSize = size * 0.72f;
    const Color c = withAlpha(color, a);
    r.fillText(xx, cy, "RNAV", size, TextAlign::Left, c);
    xx += r.measureTextWidth("RNAV", size) + rnavGpsSuffixGap(size);
    r.fillText(xx, cy, "GPS", subSize, TextAlign::Left, c);
    xx += r.measureTextWidth("GPS", subSize);
    const std::string rest = restAfterRnavGps(label);
    if (!rest.empty()) {
      r.fillText(xx, cy, " " + rest, size, TextAlign::Left, withAlpha(color, a));
    }
  } else {
    r.fillText(xx, cy, label, size, TextAlign::Left, withAlpha(color, a));
  }
}

void drawFplDtkValue(Renderer& r, float rightX, float rowCy, double dtkDeg,
                     float size, float smallSize, const Color& color) {
  (void)smallSize;
  const std::string num = formatHeading(static_cast<float>(dtkDeg));
  mfd::drawValueWithUnit(r, rightX, rowCy, num, kDeg, size, color);
}

void drawFplDisValue(Renderer& r, float rightX, float rowCy, double disNm,
                     float size, float smallSize, const Color& color) {
  (void)smallSize;
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.1f", disNm);
  mfd::drawValueWithUnit(r, rightX, rowCy, buf, "NM", size, color);
}

void drawFplValueWithDeg(Renderer& r, float rightX, float rowCy,
                         const char* value, float size, float smallSize,
                         const Color& color) {
  (void)smallSize;
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%s\u00b0", value);
  r.fillText(rightX, rowCy, buf, size, TextAlign::Right, color);
}

std::string fplApproachLegRole(const MapLeg& leg, const std::string& transition) {
  // The fix row shows its genuine procedure role; a published hold is rendered
  // on its own dedicated "HOLD" row, so do not also tag the fix with "hold".
  const std::string fromLeg = leg.procedureRole;
  if (!fromLeg.empty()) return fromLeg;
  if (transition.size() >= 2 && transition[0] == 'R' && transition[1] == 'W') {
    return {};
  }
  std::string transUpper = transition;
  for (char& c : transUpper) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  if (transUpper == "VECTORS") return {};
  if (!transition.empty() && leg.id == transition) return "iaf";
  return {};
}

float measureFplApproachLegIdentWidth(Renderer& r, const MapLeg& leg,
                                      const std::string& transition, float size,
                                      float smallSize) {
  const std::string role = fplApproachLegRole(leg, transition);
  float w = r.measureTextWidth(leg.id.c_str(), size);
  if (!role.empty()) {
    w += size * 0.14f + r.measureTextWidth(role.c_str(), smallSize);
  }
  return w;
}

void drawFplApproachLegIdent(Renderer& r, float x, float cy, const MapLeg& leg,
                             const std::string& transition, float size,
                             float smallSize, const Color& identColor,
                             const Color& roleColor, bool showSelection,
                             bool active, bool blinkOn, float a) {
  const std::string role = fplApproachLegRole(leg, transition);
  if (active) {
    const float tracking = size * 0.06f;
    const float tw = measureFplApproachLegIdentWidth(r, leg, transition, size,
                                                     smallSize);
    const Color magenta = withAlpha(colors::kMagenta, a);
    const Color black = withAlpha(colors::kBlack, a);
    if (blinkOn) {
      r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
                 size * 1.24f, magenta);
      r.fillText(x, cy, leg.id, size, TextAlign::Left, black);
      if (!role.empty()) {
        const float roleX =
            x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left, black);
      }
    } else {
      r.fillText(x, cy, leg.id, size, TextAlign::Left, magenta);
      if (!role.empty()) {
        const float roleX =
            x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left, magenta);
      }
    }
    return;
  }
  if (showSelection) {
    const float tracking = size * 0.06f;
    const float tw = measureFplApproachLegIdentWidth(r, leg, transition, size,
                                                     smallSize);
    if (blinkOn) {
      r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
                 size * 1.24f, withAlpha(colors::kPopoutCyan, a));
      r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kBlack);
      if (!role.empty()) {
        const float roleX =
            x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left, colors::kBlack);
      }
    } else {
      r.fillText(x, cy, leg.id, size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
      if (!role.empty()) {
        const float roleX =
            x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left,
                   withAlpha(colors::kWhite, a));
      }
    }
    return;
  }
  r.fillText(x, cy, leg.id, size, TextAlign::Left, identColor);
  if (!role.empty()) {
    const float roleX =
        x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
    r.fillText(roleX, cy, role, smallSize, TextAlign::Left, roleColor);
  }
}

void drawFplActiveLegArrow(Renderer& r, float ax, float cy, float rowSize,
                           const Color& color) {
  const Point arrow[7] = {
      {ax + rowSize * 1.0f, cy},
      {ax + rowSize * 0.65f, cy - rowSize * 0.35f},
      {ax + rowSize * 0.65f, cy - rowSize * 0.10f},
      {ax, cy - rowSize * 0.10f},
      {ax, cy + rowSize * 0.10f},
      {ax + rowSize * 0.65f, cy + rowSize * 0.10f},
      {ax + rowSize * 0.65f, cy + rowSize * 0.35f}};
  r.fillPolygon(arrow, 7, color);
}

// Magenta bracket from a filled origin down to the active leg (trainer: short
// horizontal tick beside the origin ident, vertical shaft past Enroute, then a
// horizontal run with arrowhead into the active row).
void drawFplLegConnector(Renderer& r, float listLeft, float sectionIdentX,
                         float originCy, float targetCy, float rowSize,
                         const Color& color, float clipTop, float clipBottom) {
  const float w = std::max(2.5f, rowSize * 0.11f);
  const float shaftX = listLeft;
  const float tickEnd =
      std::min(sectionIdentX - rowSize * 0.12f, shaftX + rowSize * 0.65f);

  // Vertical shaft welded through the active-row junction, clipped to the
  // scrollable body so scrolled-out rows do not paint outside the popup.
  const float shaftTop =
      std::max(std::min(originCy, targetCy) - w * 0.5f, clipTop);
  const float shaftBottom =
      std::min(std::max(originCy, targetCy) + w * 0.5f, clipBottom);
  if (shaftBottom <= shaftTop) return;
  r.fillRect(shaftX, shaftTop, w, shaftBottom - shaftTop, color);

  // Short horizontal tick at the origin (extends right from the shaft).
  if (originCy >= clipTop && originCy <= clipBottom && tickEnd > shaftX + w) {
    r.fillRect(shaftX + w, originCy - w * 0.5f, tickEnd - shaftX - w, w, color);
  }

  // Horizontal run at the active row into the arrow stem, then the arrowhead.
  if (targetCy < clipTop || targetCy > clipBottom) return;
  const float runStart = shaftX + w;
  const float arrowLen = rowSize * 0.88f;
  const float arrowTipX = sectionIdentX - rowSize * 0.06f;
  const float arrowBaseX = arrowTipX - arrowLen;
  const float arrowStemFrontX = arrowBaseX + arrowLen * 0.65f;
  if (arrowStemFrontX > runStart) {
    r.fillRect(runStart, targetCy - w * 0.5f, arrowStemFrontX - runStart, w,
               color);
  }
  drawFplActiveLegArrow(r, arrowBaseX, targetCy, arrowLen, color);
}

void drawFplActiveIdent(Renderer& r, float x, float cy, const std::string& ident,
                        float size, bool blinkOn, float a) {
  const Color magenta = withAlpha(colors::kMagenta, a);
  const Color black = withAlpha(colors::kBlack, a);
  const float tracking = size * 0.06f;
  const float tw = r.measureTextWidth(ident.c_str(), size);
  if (blinkOn) {
    r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
               size * 1.24f, magenta);
    r.fillText(x, cy, ident, size, TextAlign::Left, black);
  } else {
    r.fillText(x, cy, ident, size, TextAlign::Left, magenta);
  }
}

void drawFplLegRowValues(Renderer& r, float dtkRight, float disRight, float rowCy,
                         float size, float smallSize, double dtkDeg,
                         double disNm, const Color& color) {
  drawFplDtkValue(r, dtkRight, rowCy, dtkDeg, size, smallSize, color);
  drawFplDisValue(r, disRight, rowCy, disNm, size, smallSize, color);
}

void drawFplLegRowValues(Renderer& r, float dtkRight, float rowRight, float rowCy,
                         float size, float smallSize, const MapLeg& prev,
                         const MapLeg& leg, const Color& color) {
  drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                      navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon),
                      navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon),
                      color);
}

void drawFplActiveRowValues(Renderer& r, float dtkRight, float rowRight,
                            float rowCy, float size, float smallSize,
                            float bearingDeg, float distanceNm, float a) {
  drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                      static_cast<double>(bearingDeg),
                      static_cast<double>(distanceNm),
                      withAlpha(colors::kMagenta, a));
}

struct DashStyle {
  float width;
  float advance;
  float height;
};

DashStyle dashStyle(float size) {
  return {size * 0.36f, size * 0.48f, size * 0.09f};
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

float tightDashRunWidth(int count, float size) {
  return dashStyle(size).advance * static_cast<float>(count);
}

void drawFplDashValueWithDeg(Renderer& r, float rightX, float rowCy, int dashCount,
                             float size, float smallSize, const Color& color) {
  const float dashRunW = tightDashRunWidth(dashCount, size);
  const float degW = r.measureTextWidth(kDeg, smallSize);
  const float gap = size * 0.06f;
  const float left = rightX - degW - gap - dashRunW;
  drawTightDashRun(r, left, rowCy, dashCount, size, color);
  r.fillText(left + dashRunW + gap, rowCy, kDeg, smallSize, TextAlign::Left, color);
}

void drawFplApproachDtkDisDashes(Renderer& r, float dtkRight, float disRight,
                                 float rowCy, float size, float smallSize,
                                 float a) {
  const Color c = withAlpha(colors::kWhitesmoke, a);
  constexpr int kFplDtkDashCount = 3;
  constexpr int kFplDisDashCount = 2;
  drawFplDashValueWithDeg(r, dtkRight, rowCy, kFplDtkDashCount, size, smallSize, c);

  const float nmW = r.measureTextWidth("NM", smallSize);
  const float dotW = r.measureTextWidth(".", size);
  const float dashRunW = tightDashRunWidth(kFplDisDashCount, size);
  const float gap = size * 0.06f;
  const float dotLeft = disRight - nmW - gap - dashRunW - gap - dotW;
  r.fillText(dotLeft, rowCy, ".", size, TextAlign::Left, c);
  drawTightDashRun(r, dotLeft + dotW + gap, rowCy, kFplDisDashCount, size, c);
  r.fillText(dotLeft + dotW + gap + dashRunW + gap, rowCy, "NM", smallSize,
             TextAlign::Left, c);
}

void drawFplDepartureLegNavColumns(Renderer& r, float dtkRight, float disRight,
                                   float rowCy, float size, float smallSize,
                                   const std::vector<MapLeg>& legs, int legIdx,
                                   bool showActive, const FlightData& d,
                                   float alpha) {
  const MapLeg& leg = legs[static_cast<std::size_t>(legIdx)];
  const Color rowColor =
      withAlpha(showActive ? colors::kMagenta : colors::kWhitesmoke, alpha);
  if (isRunwayDepartureLegId(leg.id)) {
    drawFplApproachDtkDisDashes(r, dtkRight, disRight, rowCy, size, smallSize,
                                alpha);
    return;
  }
  if (isHeadingDepartureLeg(leg)) {
    float course = leg.legCourseDeg;
    if (showActive && d.courseDeg > 0.0f) course = d.courseDeg;
    // Published magnetic course may be absent on loaded SID legs; fall back to the
    // track between the previous fix and this leg (prior PFD FPL behavior).
    if (course <= 0.0f && legIdx > 0) {
      const MapLeg& prev = legs[static_cast<std::size_t>(legIdx - 1)];
      course = static_cast<float>(
          navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon));
    }
    if (course > 0.0f) {
      char buf[24];
      std::snprintf(buf, sizeof(buf), "hdg %03.0f", course);
      mfd::drawValueWithUnit(r, dtkRight, rowCy, buf, kDeg, size, rowColor);
    } else {
      mfd::drawValueWithUnit(r, dtkRight, rowCy, "hdg ___", kDeg, size,
                             rowColor);
    }
    return;
  }
  if (legIdx > 0) {
    const MapLeg& prev = legs[static_cast<std::size_t>(legIdx - 1)];
    if (showActive) {
      drawFplActiveRowValues(r, dtkRight, disRight, rowCy, size, smallSize,
                             d.fmaLegBearingDeg, d.fmaLegDistanceNm, alpha);
    } else {
      drawFplLegRowValues(r, dtkRight, disRight, rowCy, size, smallSize, prev,
                          leg, rowColor);
    }
  }
}

void drawFplDashRow(Renderer& r, float x, float cy, int count, float size,
                    const Color& color, bool highlighted, bool blinkOn,
                    float a) {
  const DashStyle ds = dashStyle(size);
  const float runW = ds.advance * static_cast<float>(count);
  if (highlighted && blinkOn) {
    r.fillRect(x, cy - size * 0.62f, runW, size * 1.24f,
               withAlpha(colors::kPopoutCyan, a));
    drawTightDashRun(r, x, cy, count, size, colors::kBlack);
  } else if (highlighted) {
    drawTightDashRun(r, x, cy, count, size, withAlpha(colors::kPopoutCyan, a));
  } else {
    drawTightDashRun(r, x, cy, count, size, color);
  }
}

// Draws "Origin - ____" / "Destination - ____" with optional cursor highlight.
// The section label stays cyan; only the ident/dash field pulses (trainer
// highlight-select on the editable slot, not the whole row).
void drawOriginDestLine(Renderer& r, float x, float cy, const char* prefix,
                        const std::string& ident, bool blankIdent,
                        bool highlighted, bool blinkOn, float size, float a) {
  const Color labelColor = withAlpha(colors::kPopoutCyan, a);
  r.fillText(x, cy, prefix, size, TextAlign::Left, labelColor);
  const float prefixW = r.measureTextWidth(prefix, size);
  const float fieldX = x + prefixW;
  if (blankIdent) {
    const DashStyle ds = dashStyle(size);
    const float dashW =
        ds.advance * static_cast<float>(kFplOriginDashCount);
    if (highlighted && blinkOn) {
      r.fillRect(fieldX, cy - size * 0.62f, dashW, size * 1.24f,
                 withAlpha(colors::kPopoutCyan, a));
      drawTightDashRun(r, fieldX, cy, kFplOriginDashCount, size,
                       colors::kBlack);
    } else {
      drawTightDashRun(r, fieldX, cy, kFplOriginDashCount, size, labelColor);
    }
  } else if (highlighted) {
    render::drawCursorSelect(r, fieldX, cy, ident, size, TextAlign::Left,
                             blinkOn, a);
  } else {
    r.fillText(fieldX, cy, ident, size, TextAlign::Left, labelColor);
  }
}

// Filled section slots show the ident only; empty slots keep "Origin -" /
// "Destination -" with dashes (trainer drops the label once the field is set).
void drawFplSectionIdent(Renderer& r, float x, float cy, const char* prefix,
                         const std::string& ident, bool blankIdent,
                         bool highlighted, bool blinkOn, float size, float a,
                         const Color& textColor) {
  if (blankIdent) {
    drawOriginDestLine(r, x, cy, prefix, ident, true, highlighted, blinkOn,
                       size, a);
    return;
  }
  if (highlighted) {
    render::drawCursorSelect(r, x, cy, ident, size, TextAlign::Left, blinkOn,
                             a);
  } else {
    r.fillText(x, cy, ident, size, TextAlign::Left, withAlpha(textColor, a));
  }
}

// Thin rule under the active leg row (WT .mfd-flightplan-hr).
void drawFplHrRule(Renderer& r, float left, float right, float y, float a) {
  r.strokeLine(left, y, right, y, 1.0f,
               withAlpha(Color{0.596f, 0.624f, 0.682f, 1.0f}, a));
}

void drawFplHeaderOrigDest(Renderer& r, float x, float cy, const std::string& orig,
                           const std::string& dest, bool blankOrig, bool blankDest,
                           float size, const Color& color) {
  float hx = x;
  if (blankOrig) {
    hx = drawTightDashRun(r, hx, cy, kFplDashCount, size, color);
  } else {
    r.fillText(hx, cy, orig, size, TextAlign::Left, color);
    hx += r.measureTextWidth(orig.c_str(), size);
  }
  const char* slash = " / ";
  r.fillText(hx, cy, slash, size, TextAlign::Left, color);
  hx += r.measureTextWidth(slash, size);
  if (blankDest) {
    drawTightDashRun(r, hx, cy, kFplDashCount, size, color);
  } else {
    r.fillText(hx, cy, dest, size, TextAlign::Left, color);
  }
}

void drawFplDestinationLabelRow(Renderer& r, float x, float cy, float size,
                                float a) {
  const Color c = withAlpha(colors::kPopoutCyan, a);
  const char* prefix = "Destination - ";
  r.fillText(x, cy, prefix, size, TextAlign::Left, c);
  float rx = x + r.measureTextWidth(prefix, size);
  r.fillText(rx, cy, "RW", size, TextAlign::Left, c);
  rx += r.measureTextWidth("RW", size);
  drawTightDashRun(r, rx, cy, 2, size, c);
}

}  // namespace
// as the same lower-right popout the other PFD windows use (Pilot's Guide
// Fig. 5-48 "Active Flight Plan Window on PFD"). The header carries the origin /
// destination idents; the body groups legs under Origin, Enroute, and
// Destination headings (Pilot's Guide Fig. 5-48). GPS Direct-To shows the D→
// glyph and active target above the columns while the section rows stay blank.
void drawFlightPlanWindow(Renderer& r, float w, float h, const Layout& L,
                          const FlightData& d, const SoftkeyController& ui) {
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::FlightPlan),
                      "Flight Plan", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const std::vector<MapLeg>& legs = ui.flightPlanLegs();
  const bool cursorOn = ui.flightPlanCursorOn();
  const bool blinkOn = ui.blinkOn();
  const float size = fontPx(wt::kInfoValue, h);
  const float smallSize = size * 0.72f;  // smaller "NM"/unit affix
  const float padX = fontPx(8.0f, h);
  const float scrollW = wtScrollBarLane(h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX;
  const float listW = listRight - listLeft;

  // Column geometry mirrors the Working Title PFD .fpl layout: ident on the
  // left, DTK right edge ~58%, DIS at the list edge (keep columns separated so
  // wide DIS values do not overlap DTK).
  const float identX = listLeft + listW * 0.04f;
  const float sectionIdentX = identX + fontPx(kFplSectionIdentIndentPx, h);
  constexpr float kFplDtkDisLeftPx = 10.0f;
  // Nudge the DTK column right so right-aligned DTK values (e.g. "hdg 077°") no
  // longer overlap the waypoint idents on their left.
  constexpr float kFplDtkShiftPx = 30.0f;
  const float dtkDisLeft = fontPx(kFplDtkDisLeftPx, h);
  const float disRight = listRight - dtkDisLeft;
  const float dtkRight =
      listLeft + listW * 0.58f - dtkDisLeft + fontPx(kFplDtkShiftPx, h);

  // Active navigation header + DTK/DIS column labels (WT Fig. 5-48).
  constexpr float kFplActiveRowPx = 22.0f;
  constexpr float kFplHeaderPx = 40.0f;
  constexpr float kFplBodyRowPx = 24.0f;
  const float activeLegSize = fontPx(16.0f, h);
  const bool approachLoadedUi = ui.flightPlanHasLoadedApproach();
  int arrStart = ui.flightPlanArrivalLegStart();
  int arrCount = ui.flightPlanArrivalLegCount();
  std::string arrHeader = ui.flightPlanArrivalHeaderLabel();
  const std::string arrAirport = ui.flightPlanArrivalAirportIcao();
  int depStart = ui.flightPlanDepartureLegStart();
  int depCount = ui.flightPlanDepartureLegCount();
  std::string depHeader = ui.flightPlanDepartureHeaderLabel();
  const std::string depAirport = ui.flightPlanDepartureAirportIcao();
  // Drop terminal-procedure (departure/arrival) blocks whose indices no longer
  // fit the current plan. This happens when a shorter plan replaces a longer
  // route (e.g. loading an approach) without the grouping being cleared; the
  // stale block would otherwise force the procedure display path and hide the
  // real legs behind a phantom header + blank rows.
  const int legPlanSize = static_cast<int>(legs.size());
  if (!fplBlockFitsPlan(depStart, depCount, legPlanSize)) {
    depStart = 0;
    depCount = 0;
    depHeader.clear();
  }
  if (!fplBlockFitsPlan(arrStart, arrCount, legPlanSize)) {
    arrStart = 0;
    arrCount = 0;
    arrHeader.clear();
  }
  // The approach is the procedure tail after any loaded arrival/STAR block, so
  // its inference must skip the STAR legs (which can carry procedureRole tags).
  const int arrivalEnd = arrCount > 0 ? arrStart + arrCount : 0;
  // The approach can never begin within or before a loaded SID/departure block;
  // this keeps the approach inference's untagged-feeder walk-back from swallowing
  // the SID/enroute legs when the destination-airport waypoint is absent (the
  // sim drops it once an approach is loaded).
  const int departureEnd = depCount > 0 ? depStart + depCount : 0;
  int approachStart = ui.flightPlanApproachLegStart();
  int approachCount = ui.flightPlanApproachLegCount();
  const InferredProcedureBlock approachBlock = resolveApproachBlockInPlan(
      legs, approachStart, approachCount, ui.flightPlanApproachTransition(),
      arrivalEnd, departureEnd);
  approachStart = approachBlock.start;
  approachCount = approachBlock.count;
  if (approachCount > 0 && approachStart >= 0 &&
      approachStart + approachCount < static_cast<int>(legs.size())) {
    approachCount = fplNormalizedApproachCount(
        approachStart, approachCount, static_cast<int>(legs.size()));
  }
  const bool approachLoaded = approachCount > 0;
  const bool layoutDestFilled = ui.flightPlanDestinationFilledForLayout();
  // A loaded approach always ends the route at the approach airport, so the
  // destination section is filled — keep the destination airport (e.g. KJAX)
  // in the approach header rather than letting it fall into the Enroute list.
  // The display resolves the approach block locally, so do not depend solely
  // on the controller's stored approach count (which can lag for imported
  // routes whose approach is only inferred here).
  const bool destFilled = layoutDestFilled || approachLoaded;
  // A plan carrying loaded-airway legs also uses the procedure display rows so
  // the enroute legs group under "Airway -" headers and honor the collapse
  // toggle (Pilot's Guide, Load Airway).
  const bool hasAirwayLegs = fplPlanHasAirwayLegs(legs);
  const bool airwaysCollapsed = ui.flightPlanAirwaysCollapsed();
  const bool procedureDisplay =
      fplUsesProcedureDisplayRows(depHeader, depCount, arrHeader, arrCount,
                                  approachCount) ||
      hasAirwayLegs;
  std::string approachAirport;
  if (approachLoaded) {
    approachAirport = ui.flightPlanApproachAirportIcao();
    if (approachAirport.empty()) {
      // The approach was inferred from leg roles but its airport identity was
      // dropped (e.g. a Direct-To or external/sim resync cleared the loaded
      // approach). The destination served by the STAR/arrival is the same
      // airport as the approach, so prefer it over the naive "airport before
      // the approach", which is the origin when no destination waypoint
      // precedes the approach.
      if (!arrAirport.empty()) {
        approachAirport = arrAirport;
      } else if (approachStart > 0 &&
                 approachStart <= static_cast<int>(legs.size())) {
        const std::string& before =
            legs[static_cast<std::size_t>(approachStart - 1)].id;
        if (!(approachStart == 1 && !legs.empty() && before == legs.front().id &&
              isAirportIdent(before))) {
          approachAirport = before;
        }
      }
    }
  }
  const int sectionLegCount =
      approachLoaded ? fplEnrouteDisplayLegCount(legs, approachStart)
                     : static_cast<int>(legs.size());
  // Direct-To: copy the PFD Navigation Status Box (D→ + target, magenta); the
  // section template below stays blank until the pilot builds a flight plan.
  const bool directToFplView =
      navDirectToActive(d) && !ui.flightPlanLocalDraft();
  const float activeLegCy = f.contentTop + fontPx(kFplActiveRowPx * 0.52f, h);
  const Color headerCyan = withAlpha(colors::kPopoutCyan, a);
  if (directToFplView) {
    const std::string dest = d.fmaToWpt;
    drawNavDirectToHeader(r, identX, activeLegCy, dest, activeLegSize,
                          withAlpha(colors::kMagenta, a));
  } else {
    const bool blankOriginHeader =
        approachLoaded && approachStart <= 1 && !legs.empty() &&
        !approachAirport.empty() && legs.front().id == approachAirport;
    const bool destOnlyPlan =
        ui.flightPlanDestinationFilled() && legs.size() == 1;
    const std::string orig = fplHeaderOriginIdent(
        legs, approachStart, approachCount, approachLoaded, blankOriginHeader,
        destOnlyPlan, ui.activeWaypointId());
    const std::string destIdent = fplHeaderDestinationIdent(
        legs, ui.flightPlanDestinationFilled(), approachStart, approachLoaded,
        approachAirport);
    const bool blankOrig = orig.empty();
    const bool blankDest = destIdent.empty();
    drawFplHeaderOrigDest(r, identX, activeLegCy, orig, destIdent, blankOrig,
                          blankDest, activeLegSize, headerCyan);
  }

  const float activeLegRuleY =
      f.contentTop + fontPx(kFplActiveRowPx + 1.0f, h);
  drawFplHrRule(r, listLeft, listRight, activeLegRuleY, a);

  const float dtkCy = f.contentTop + fontPx(kFplActiveRowPx + 10.0f, h);
  if (!directToFplView) {
    r.fillText(dtkRight, dtkCy, "DTK", smallSize, TextAlign::Right,
               withAlpha(colors::kWhite, a));
    r.fillText(disRight, dtkCy, "DIS", smallSize, TextAlign::Right,
               withAlpha(colors::kWhite, a));
  }
  const float bodyTop = f.contentTop + fontPx(kFplHeaderPx, h);

  // During Direct-To the enroute template stays blank until a route is built.
  // A loaded approach keeps that template (Origin / Enroute) then shows the
  // approach header and legs below — not mixed into the Enroute section.
  const bool destOnlyPlan =
      ui.flightPlanDestinationFilled() && legs.size() == 1;
  const bool blankOriginSection =
      directToFplView || destOnlyPlan || ui.flightPlanHasLoadedDeparture() ||
      (approachLoaded && approachStart <= 1 && !legs.empty() &&
       !approachAirport.empty() && legs.front().id == approachAirport);
  const int bodyLegCount =
      directToFplView && !procedureDisplay ? 0
                                           : static_cast<int>(legs.size());
  const bool directToPlanBody = directToFplView;
  // Filled waypoint idents (origin, enroute, destination) indent under their
  // section labels; blank "Origin - ____" rows stay at identX.
  const float filledIdentX = sectionIdentX;
  const std::vector<FplDisplayRow> procedureDisplayRows =
      procedureDisplay
          ? buildFplProcedureDisplayRows(
                legs, depStart, depCount, depHeader, arrStart, arrCount,
                arrHeader, approachStart, approachCount, blankOriginSection,
                destFilled, airwaysCollapsed)
          : std::vector<FplDisplayRow>{};
  const std::vector<FplSectionRow> sectionRows =
      procedureDisplay ? std::vector<FplSectionRow>{}
                       : fplFilterDuplicateLegSectionRows(
                             buildFplSectionRows(bodyLegCount,
                                                 layoutDestFilled,
                                                 directToPlanBody),
                             legs);
  const int sectionCursor = ui.flightPlanCursor();

  const bool dtoNavActive =
      navDirectToActive(d) || ui.mapDirectToActive();
  const bool directToHoldNav = ui.mapDirectToHold();
  int activeLegIdx = fplDirectToTargetLegIndex(
      legs, d, ui.activeWaypointId(), dtoNavActive);
  if (activeLegIdx < 0) {
    activeLegIdx =
        fplResolvedActiveLegIndex(legs, d, ui.activeWaypointId());
  }
  // Enroute-template rows (origin / destination) use the pre-approach leg span.
  const int fplActiveLayoutLegCount =
      approachLoaded ? sectionLegCount : bodyLegCount;
  const bool fplActiveLayoutDestFilled = layoutDestFilled;
  const int listCursorRow = sectionCursor;
  int activeSelectableRow =
      procedureDisplay
          ? fplProcedureSelectableRowForLegIndex(
                activeLegIdx, legs, depStart, depCount, depHeader, arrStart,
                arrCount, arrHeader, approachStart, approachCount,
                blankOriginSection, destFilled, airwaysCollapsed)
          : fplSectionSelectableRowForLegIndex(
                activeLegIdx, sectionRows, bodyLegCount,
                layoutDestFilled, directToPlanBody);
  if (procedureDisplay && activeLegIdx >= 0 &&
      activeLegIdx < static_cast<int>(legs.size())) {
    const MapLeg& activeLeg =
        legs[static_cast<std::size_t>(activeLegIdx)];
    if (fplHoldNavActiveOnLeg(d, directToHoldNav, ui.mapDirectToActive(),
                              dtoNavActive, activeLegIdx, activeLegIdx,
                              activeLeg)) {
      const int holdRow = fplApproachSelectableRowForHoldLegIndex(
          activeLegIdx, legs, approachStart, approachCount, blankOriginSection,
          destFilled);
      if (holdRow >= 0) activeSelectableRow = holdRow;
    }
  }
  const bool pinActiveApproachLeg = fplPinActiveApproachLeg(
      approachLoaded, activeLegIdx, approachStart, approachCount,
      ui.flightPlanLocalDraft(),
      directToFplView || ui.mapDirectToActive());
  int cursorLegIdx = -1;
  if (procedureDisplay) {
    cursorLegIdx = fplProcedureLegIndexForSelectable(
        listCursorRow, legs, depStart, depCount, depHeader, arrStart, arrCount,
        arrHeader, approachStart, approachCount, blankOriginSection, destFilled,
        airwaysCollapsed);
  } else {
    cursorLegIdx = fplSectionLegIndexForSelectable(
        listCursorRow, sectionRows, bodyLegCount,
        layoutDestFilled);
  }
  const bool activeHighlight =
      (dtoNavActive && activeLegIdx >= 0) ||
      fplShowActiveLegHighlight(activeLegIdx, cursorLegIdx, activeSelectableRow,
                                listCursorRow, pinActiveApproachLeg);
  const std::string& navToIdent = d.fmaToWpt;

  const int displayRowCount =
      procedureDisplay ? static_cast<int>(procedureDisplayRows.size())
                       : static_cast<int>(sectionRows.size());
  const float bodyBottom = f.top + f.h - fontPx(4.0f, h);
  const float bodyH = std::max(1.0f, bodyBottom - bodyTop);
  const float rowH = fontPx(kFplBodyRowPx, h);
  const bool scrolling =
      static_cast<float>(displayRowCount) * rowH > bodyH + 0.5f;
  const int visible = scrolling
                          ? std::max(1, static_cast<int>(bodyH / rowH))
                          : displayRowCount;
  int first = 0;
  if (scrolling && displayRowCount > visible) {
    int cursorDisplayRow = 0;
    if (procedureDisplay) {
      cursorDisplayRow = fplProcedureDisplayRowIndexForSelectable(
          sectionCursor, legs, depStart, depCount, depHeader, arrStart,
          arrCount, arrHeader, approachStart, approachCount, blankOriginSection,
          destFilled, airwaysCollapsed);
    } else {
      int selectableBefore = 0;
      for (int i = 0; i < displayRowCount; ++i) {
        const FplSectionRow& sr = sectionRows[static_cast<std::size_t>(i)];
        if (!fplSectionRowIsSelectable(sr, bodyLegCount,
                                       layoutDestFilled)) {
          continue;
        }
        if (selectableBefore == sectionCursor) {
          cursorDisplayRow = i;
          break;
        }
        ++selectableBefore;
      }
    }
    int pinnedRow = -1;
    if (pinActiveApproachLeg && procedureDisplay) {
      if (activeLegIdx >= 0 &&
          activeLegIdx < static_cast<int>(legs.size()) &&
          fplHoldNavActiveOnLeg(
              d, directToHoldNav, ui.mapDirectToActive(), dtoNavActive,
              activeLegIdx, activeLegIdx,
              legs[static_cast<std::size_t>(activeLegIdx)])) {
        pinnedRow = fplApproachDisplayRowIndexForHoldLegIndex(
            activeLegIdx, legs, approachStart, approachCount, blankOriginSection,
            destFilled);
      } else {
        pinnedRow = fplApproachDisplayRowIndexForLegIndex(
            activeLegIdx, legs, approachStart, approachCount, blankOriginSection,
            destFilled);
      }
    }
    first = fplListScrollFirst(cursorDisplayRow, pinnedRow, displayRowCount,
                               visible);
  }
  const float disColumnRight =
      scrolling ? disRight - scrollW - fontPx(6.0f, h) : disRight;
  const float rowRight = disColumnRight;

  const bool originFilled = bodyLegCount > 0 && !legs.empty();
  const bool useConnector =
      activeHighlight && originFilled && activeLegIdx > 0 && !blankOriginSection &&
      (!procedureDisplay ||
       (activeLegIdx >= 0 && activeLegIdx < approachStart));
  bool connectorOriginValid = false;
  bool connectorTargetValid = false;
  float connectorOriginCy = 0.0f;
  float connectorTargetCy = 0.0f;

  int selectableIdx = 0;
  if (first > 0) {
    if (procedureDisplay) {
      for (int i = 0; i < first; ++i) {
        if (fplProcedureDisplayRowSelectable(
                procedureDisplayRows[static_cast<std::size_t>(i)].kind)) {
          ++selectableIdx;
        }
      }
    } else {
      for (int i = 0; i < first; ++i) {
        const FplSectionRow& sr = sectionRows[static_cast<std::size_t>(i)];
        if (fplSectionRowIsSelectable(sr, bodyLegCount,
                                      layoutDestFilled)) {
          ++selectableIdx;
        }
      }
    }
  }
  const int end = std::min(displayRowCount, first + visible);

  if (useConnector) {
    // Map row indices into the visible viewport (same formula as the draw loop).
    for (int idx = 0; idx < displayRowCount; ++idx) {
      const float rowCy =
          bodyTop + rowH * (static_cast<float>(idx - first) + 0.5f);
      if (procedureDisplay) {
        const FplDisplayRow& dr =
            procedureDisplayRows[static_cast<std::size_t>(idx)];
        switch (dr.kind) {
          case FplDisplayRowKind::Origin:
            if (dr.legIndex >= 0) {
              connectorOriginCy = rowCy;
              connectorOriginValid = true;
            }
            break;
          case FplDisplayRowKind::Destination: {
            const FplSectionRow destSr{FplSectionRow::Kind::Destination,
                                       dr.legIndex};
            if (dr.legIndex >= 0 &&
                fplSectionRowIsActiveDisplay(
                    destSr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled,
                    activeLegIdx)) {
              connectorTargetCy = rowCy;
              connectorTargetValid = true;
            }
            break;
          }
          case FplDisplayRowKind::EnrouteLeg: {
            const FplSectionRow enrouteSr{FplSectionRow::Kind::EnrouteLeg,
                                          dr.legIndex};
            if (fplSectionRowIsActiveDisplay(
                    enrouteSr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled,
                    activeLegIdx)) {
              connectorTargetCy = rowCy;
              connectorTargetValid = true;
            }
            break;
          }
          default:
            break;
        }
      } else {
        const FplSectionRow& sr = sectionRows[static_cast<std::size_t>(idx)];
        if (sr.kind == FplSectionRow::Kind::EnrouteLabel ||
            sr.kind == FplSectionRow::Kind::DestinationLabel) {
          continue;
        }
        if (sr.kind == FplSectionRow::Kind::Origin && sr.legIndex >= 0) {
          connectorOriginCy = rowCy;
          connectorOriginValid = true;
        }
        if (fplSectionRowIsActiveDisplay(
                sr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled,
                activeLegIdx)) {
          connectorTargetCy = rowCy;
          connectorTargetValid = true;
        }
      }
    }
    if (connectorOriginValid && connectorTargetValid) {
      drawFplLegConnector(r, listLeft, sectionIdentX, connectorOriginCy,
                          connectorTargetCy, size,
                          withAlpha(colors::kMagenta, a), bodyTop, bodyBottom);
    }
  }

  // Procedure header rows span the full list width (no DTK/DIS columns), so
  // they may shrink to the right edge, reserving the scroll-bar lane.
  const float headerRight = scrolling ? listRight - scrollW : listRight;
  const float headerMaxW = headerRight - identX - fontPx(2.0f, h);
  const auto headerFitSize = [&](const std::string& s) {
    const float w = r.measureTextWidth(s, size);
    if (w <= headerMaxW || w <= 0.0f) return size;
    return size * (headerMaxW / w);
  };

  for (int idx = first; idx < end; ++idx) {
    const float rowCy =
        bodyTop + rowH * (static_cast<float>(idx - first) + 0.5f);
    if (procedureDisplay) {
      const FplDisplayRow& dr =
          procedureDisplayRows[static_cast<std::size_t>(idx)];
      switch (dr.kind) {
        case FplDisplayRowKind::DepartureHeader: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          if (showSelection) {
            const std::string t = depAirport + "-" + depHeader;
            render::drawCursorSelect(r, identX, rowCy, t, headerFitSize(t),
                                     TextAlign::Left, blinkOn, a);
          } else {
            drawFplApproachHeader(r, identX, rowCy, depAirport, depHeader, size,
                                  colors::kPopoutCyan, a, headerMaxW);
          }
          continue;
        }
        case FplDisplayRowKind::ArrivalHeader: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          if (showSelection) {
            const std::string t = arrAirport + "-" + arrHeader;
            render::drawCursorSelect(r, identX, rowCy, t, headerFitSize(t),
                                     TextAlign::Left, blinkOn, a);
          } else {
            drawFplApproachHeader(r, identX, rowCy, arrAirport, arrHeader, size,
                                  colors::kPopoutCyan, a, headerMaxW);
          }
          continue;
        }
        case FplDisplayRowKind::SepDash: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          drawFplDashRow(r, identX, rowCy, kFplApproachSepDashCount, size,
                         withAlpha(colors::kPopoutCyan, a), showSelection, blinkOn, a);
          continue;
        }
        case FplDisplayRowKind::ApproachHeader: {
          // Procedure title aligns with section labels (Enroute); legs indent below.
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          if (showSelection) {
            const std::string t =
                approachAirport + "-" + ui.flightPlanApproachHeaderLabel();
            render::drawCursorSelect(r, identX, rowCy, t, headerFitSize(t),
                                     TextAlign::Left, blinkOn, a);
          } else {
            drawFplApproachHeader(r, identX, rowCy, approachAirport,
                                  ui.flightPlanApproachHeaderLabel(), size,
                                  colors::kPopoutCyan, a, headerMaxW);
          }
          continue;
        }
        case FplDisplayRowKind::AirwayHeader: {
          // "Airway - <name>.<exit>" parent row above a loaded airway segment
          // (Pilot's Guide, Load Airway); aligned with Enroute. Selectable
          // cursor stop: CLR on it removes the whole airway segment.
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          if (dr.legIndex < 0 ||
              dr.legIndex >= static_cast<int>(legs.size())) {
            continue;
          }
          const MapLeg& exitLeg = legs[static_cast<std::size_t>(dr.legIndex)];
          const std::string awLabel =
              "Airway - " + exitLeg.viaAirway + "." + exitLeg.id;
          if (showSelection) {
            render::drawCursorSelect(r, identX, rowCy, awLabel, size,
                                     TextAlign::Left, blinkOn, a);
          } else {
            r.fillText(identX, rowCy, awLabel, size, TextAlign::Left,
                       withAlpha(colors::kPopoutCyan, a));
          }
          continue;
        }
        case FplDisplayRowKind::Hold: {
          if (dr.legIndex < 0 ||
              dr.legIndex >= static_cast<int>(legs.size())) {
            continue;
          }
          const MapLeg& leg = legs[static_cast<std::size_t>(dr.legIndex)];
          const MapHoldPattern& hold = leg.hold;
          if (!hold.active) continue;
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          const bool showActive = fplHoldNavActiveOnLeg(
              d, directToHoldNav, ui.mapDirectToActive(), dtoNavActive,
              dr.legIndex, activeLegIdx, leg);
          const bool activeNavBlink =
              showActive &&
              fplActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
                                   cursorOn, listCursorRow, activeSelectableRow);
          if (showActive && !useConnector) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          if (showSelection) {
            render::drawCursorSelect(r, sectionIdentX, rowCy, "HOLD", size,
                                     TextAlign::Left, blinkOn, a);
          } else if (showActive) {
            const Color holdColor =
                activeNavBlink && blinkOn
                    ? withAlpha(colors::kMagenta, a)
                    : withAlpha(colors::kMagenta, a);
            r.fillText(sectionIdentX, rowCy, "HOLD", size, TextAlign::Left,
                       holdColor);
          } else {
            r.fillText(sectionIdentX, rowCy, "HOLD", size, TextAlign::Left,
                       withAlpha(colors::kWhite, a));
          }
          if (showActive) {
            drawFplActiveRowValues(r, dtkRight, disColumnRight, rowCy, size,
                                   smallSize, d.fmaLegBearingDeg,
                                   d.fmaLegDistanceNm, a);
          } else {
            drawFplDtkValue(r, dtkRight, rowCy,
                            static_cast<double>(hold.inboundCourseDeg), size,
                            smallSize, withAlpha(colors::kWhitesmoke, a));
            if (hold.legLengthNm > 0.0f) {
              drawFplDisValue(r, disColumnRight, rowCy,
                              static_cast<double>(hold.legLengthNm), size,
                              smallSize, withAlpha(colors::kWhitesmoke, a));
            }
          }
          continue;
        }
        case FplDisplayRowKind::ApproachLeg: {
          const int legIdx = dr.legIndex;
          const MapLeg& leg = legs[static_cast<std::size_t>(legIdx)];
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          const bool holdNavOnLeg = fplHoldNavActiveOnLeg(
              d, directToHoldNav, ui.mapDirectToActive(), dtoNavActive, legIdx,
              activeLegIdx, leg);
          const bool showActive =
              !holdNavOnLeg &&
              fplShowActiveNavRow(activeHighlight, legIdx, activeLegIdx, leg,
                                  navToIdent);
          const std::string approachTransition =
              ui.flightPlanApproachTransition();
          if (showActive && !useConnector) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          drawFplApproachLegIdent(
              r, sectionIdentX, rowCy, leg, approachTransition, size, smallSize,
              withAlpha(colors::kPopoutCyan, a),
              withAlpha(colors::kWhite, a), showSelection, showActive,
              fplRowIdentBlink(showSelection, showActive, legIdx, activeLegIdx,
                               cursorLegIdx, cursorOn, listCursorRow,
                               activeSelectableRow, blinkOn),
              a);
          if (showActive) {
            drawFplActiveRowValues(r, dtkRight, disColumnRight, rowCy, size,
                                   smallSize, d.fmaLegBearingDeg,
                                   d.fmaLegDistanceNm, a);
          } else if (legIdx > 0) {
            const MapLeg& prev = legs[static_cast<std::size_t>(legIdx - 1)];
            drawFplLegRowValues(r, dtkRight, disColumnRight, rowCy, size,
                                smallSize, prev, leg,
                                withAlpha(colors::kWhitesmoke, a));
          } else {
            drawFplApproachDtkDisDashes(r, dtkRight, disColumnRight, rowCy, size,
                                        smallSize, a);
          }
          continue;
        }
        case FplDisplayRowKind::EnrouteLabel:
          r.fillText(identX, rowCy, "Enroute", size, TextAlign::Left,
                     withAlpha(colors::kPopoutCyan, a));
          continue;
        case FplDisplayRowKind::Origin: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          if (dr.legIndex >= 0) {
            const std::string ident =
                legs[static_cast<std::size_t>(dr.legIndex)].id;
            const FplSectionRow originSr{FplSectionRow::Kind::Origin,
                                         dr.legIndex};
            const MapLeg& origLeg =
                legs[static_cast<std::size_t>(dr.legIndex)];
            const bool showActive = fplShowActiveNavRow(
                activeHighlight, dr.legIndex, activeLegIdx, origLeg, navToIdent);
            if (showActive && !useConnector) {
              drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                    withAlpha(colors::kMagenta, a));
            }
            if (showActive) {
              drawFplActiveIdent(
                  r, filledIdentX, rowCy, ident, size,
                  fplRowIdentBlink(showSelection, showActive, dr.legIndex,
                                   activeLegIdx, cursorLegIdx, cursorOn,
                                   listCursorRow, activeSelectableRow, blinkOn),
                  a);
            } else if (showSelection) {
              render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                         TextAlign::Left, blinkOn, a);
            } else {
              r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                         withAlpha(colors::kPopoutCyan, a));
            }
            if (showActive) {
              drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size,
                                     smallSize, d.fmaLegBearingDeg,
                                     d.fmaLegDistanceNm, a);
            } else if (dr.legIndex > 0) {
              const MapLeg& leg =
                  legs[static_cast<std::size_t>(dr.legIndex)];
              const MapLeg& prev =
                  legs[static_cast<std::size_t>(dr.legIndex - 1)];
              drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size,
                                  smallSize, prev, leg,
                                  withAlpha(colors::kWhitesmoke, a));
            }
          } else {
            drawFplSectionIdent(r, identX, rowCy, "Origin - ", std::string(),
                                true, showSelection, blinkOn, size, a,
                                colors::kPopoutCyan);
          }
          continue;
        }
        case FplDisplayRowKind::Destination: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          const FplSectionRow destSr{FplSectionRow::Kind::Destination,
                                     dr.legIndex};
          const bool showActive =
              dr.legIndex >= 0
                  ? fplShowActiveNavRow(
                        activeHighlight, dr.legIndex, activeLegIdx,
                        legs[static_cast<std::size_t>(dr.legIndex)],
                        navToIdent)
                  : false;
          if (showActive && !useConnector) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          if (dr.legIndex >= 0) {
            const std::string ident =
                legs[static_cast<std::size_t>(dr.legIndex)].id;
            if (showActive) {
              drawFplActiveIdent(
                  r, filledIdentX, rowCy, ident, size,
                  fplRowIdentBlink(showSelection, showActive, dr.legIndex,
                                   activeLegIdx, cursorLegIdx, cursorOn,
                                   listCursorRow, activeSelectableRow, blinkOn),
                  a);
            } else if (showSelection) {
              render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                         TextAlign::Left, blinkOn, a);
            } else {
              r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                         withAlpha(colors::kPopoutCyan, a));
            }
          } else {
            drawFplSectionIdent(r, identX, rowCy, "Destination - ",
                                std::string(), true, showSelection, blinkOn, size, a,
                                colors::kPopoutCyan);
          }
          if (showActive) {
            drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                   d.fmaLegBearingDeg, d.fmaLegDistanceNm, a);
          } else if (dr.legIndex > 0) {
            const MapLeg& leg =
                legs[static_cast<std::size_t>(dr.legIndex)];
            const MapLeg& prev =
                legs[static_cast<std::size_t>(dr.legIndex - 1)];
            drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                prev, leg, withAlpha(colors::kWhitesmoke, a));
          }
          continue;
        }
        case FplDisplayRowKind::DestinationLabel:
          drawFplDestinationLabelRow(r, identX, rowCy, size, a);
          continue;
        case FplDisplayRowKind::DestinationBlank: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                         withAlpha(colors::kPopoutCyan, a), showSelection,
                         blinkOn, a);
          continue;
        }
        case FplDisplayRowKind::OriginBlank: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                         withAlpha(colors::kPopoutCyan, a), showSelection, blinkOn, a);
          continue;
        }
        case FplDisplayRowKind::EnrouteLeg:
        case FplDisplayRowKind::ArrivalLeg: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          const std::string ident =
              legs[static_cast<std::size_t>(dr.legIndex)].id;
          const FplSectionRow enrouteSr{FplSectionRow::Kind::EnrouteLeg,
                                          dr.legIndex};
          const MapLeg& enrouteLeg =
              legs[static_cast<std::size_t>(dr.legIndex)];
          const bool showActive = fplShowActiveNavRow(
              activeHighlight, dr.legIndex, activeLegIdx, enrouteLeg, navToIdent);
          if (showActive && !useConnector) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          if (showActive) {
            drawFplActiveIdent(
                r, filledIdentX, rowCy, ident, size,
                fplRowIdentBlink(showSelection, showActive, dr.legIndex,
                                 activeLegIdx, cursorLegIdx, cursorOn,
                                 listCursorRow, activeSelectableRow, blinkOn),
                a);
          } else if (showSelection) {
            render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                       TextAlign::Left, blinkOn, a);
          } else {
            r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                       withAlpha(colors::kPopoutCyan, a));
          }
          if (showActive) {
            drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                   d.fmaLegBearingDeg, d.fmaLegDistanceNm, a);
          } else if (dr.legIndex > 0) {
            const MapLeg& leg =
                legs[static_cast<std::size_t>(dr.legIndex)];
            const MapLeg& prev =
                legs[static_cast<std::size_t>(dr.legIndex - 1)];
            drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                prev, leg, withAlpha(colors::kWhitesmoke, a));
          }
          continue;
        }
        case FplDisplayRowKind::DepartureLeg: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          const std::string ident =
              legs[static_cast<std::size_t>(dr.legIndex)].id;
          const MapLeg& depLeg =
              legs[static_cast<std::size_t>(dr.legIndex)];
          const bool showActive = fplShowActiveNavRow(
              activeHighlight, dr.legIndex, activeLegIdx, depLeg, navToIdent);
          if (showActive && !useConnector) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          if (showActive) {
            drawFplActiveIdent(
                r, filledIdentX, rowCy, ident, size,
                fplRowIdentBlink(showSelection, showActive, dr.legIndex,
                                 activeLegIdx, cursorLegIdx, cursorOn,
                                 listCursorRow, activeSelectableRow, blinkOn),
                a);
          } else if (showSelection) {
            render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                       TextAlign::Left, blinkOn, a);
          } else {
            r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                       withAlpha(colors::kPopoutCyan, a));
          }
          drawFplDepartureLegNavColumns(r, dtkRight, rowRight, rowCy, size,
                                        smallSize, legs, dr.legIndex,
                                        showActive, d, a);
          continue;
        }
        case FplDisplayRowKind::EnrouteBlank: {
          const bool showSelection = fplShowListRowSelection(
              selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
          ++selectableIdx;
          drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                         withAlpha(colors::kPopoutCyan, a), showSelection, blinkOn, a);
          continue;
        }
        default:
          continue;
      }
    }

    const FplSectionRow& sr = sectionRows[static_cast<std::size_t>(idx)];

    if (sr.kind == FplSectionRow::Kind::EnrouteLabel) {
      r.fillText(identX, rowCy, "Enroute", size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
      continue;
    }

    if (sr.kind == FplSectionRow::Kind::DestinationLabel) {
      drawFplDestinationLabelRow(r, identX, rowCy, size, a);
      continue;
    }

    if (sr.kind == FplSectionRow::Kind::Origin && sr.legIndex < 0 &&
        !fplSectionRowIsSelectable(sr, bodyLegCount,
                                   layoutDestFilled)) {
      drawFplSectionIdent(r, identX, rowCy, "Origin - ", std::string(), true,
                          false, false, size, a, colors::kPopoutCyan);
      continue;
    }

    const bool showSelection = fplShowListRowSelection(
        selectableIdx, listCursorRow, activeSelectableRow, cursorOn);

    std::string ident;
    switch (sr.kind) {
      case FplSectionRow::Kind::OriginBlank:
        drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                       withAlpha(colors::kPopoutCyan, a), showSelection, blinkOn,
                       a);
        ++selectableIdx;
        continue;
      case FplSectionRow::Kind::Origin:
        if (sr.legIndex >= 0) {
          ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
          drawFplSectionIdent(r, filledIdentX, rowCy, "Origin - ", ident,
                              false, showSelection, blinkOn, size, a,
                              colors::kPopoutCyan);
        } else {
          drawFplSectionIdent(r, identX, rowCy, "Origin - ", std::string(),
                              true, showSelection, blinkOn, size, a,
                              colors::kPopoutCyan);
        }
        ++selectableIdx;
        continue;
      case FplSectionRow::Kind::EnrouteBlank: {
        const bool active = fplSectionRowIsActiveDisplay(
            sr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled, activeLegIdx);
            const bool showActive = active && activeHighlight;
        if (showActive && !useConnector) {
          drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                withAlpha(colors::kMagenta, a));
        }
        drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                       withAlpha(showActive ? colors::kMagenta
                                            : colors::kPopoutCyan,
                                 a),
                       showSelection, blinkOn, a);
        if (showActive) {
          drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                 d.fmaLegBearingDeg, d.fmaLegDistanceNm, a);
        }
        ++selectableIdx;
        continue;
      }
      case FplSectionRow::Kind::EnrouteLeg:
        ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
        break;
      case FplSectionRow::Kind::Destination:
        if (sr.legIndex >= 0) {
          ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
          const bool splitLabel = fplShowsDestinationLabelRow(
              bodyLegCount, layoutDestFilled);
          const bool showActive = fplShowActiveNavRow(
              activeHighlight, sr.legIndex, activeLegIdx,
              legs[static_cast<std::size_t>(sr.legIndex)], navToIdent);
          if (showActive) {
            drawFplActiveIdent(
                r, filledIdentX, rowCy, ident, size,
                fplRowIdentBlink(showSelection, showActive, sr.legIndex,
                                 activeLegIdx, cursorLegIdx, cursorOn,
                                 listCursorRow, activeSelectableRow, blinkOn),
                a);
          } else if (splitLabel) {
            if (showSelection) {
              render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                         TextAlign::Left, blinkOn, a);
            } else {
              r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                         withAlpha(colors::kPopoutCyan, a));
            }
          } else {
            drawFplSectionIdent(r, filledIdentX, rowCy, "Destination - ",
                                ident, false, showSelection, blinkOn, size, a,
                                colors::kPopoutCyan);
          }
          if (showActive) {
            drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size,
                                   smallSize, d.fmaLegBearingDeg,
                                   d.fmaLegDistanceNm, a);
          } else if (sr.legIndex > 0) {
            const MapLeg& leg =
                legs[static_cast<std::size_t>(sr.legIndex)];
            const MapLeg& prev =
                legs[static_cast<std::size_t>(sr.legIndex - 1)];
            drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                                prev, leg, withAlpha(colors::kWhitesmoke, a));
          }
        } else {
          drawFplSectionIdent(r, identX, rowCy, "Destination - ", ident, true,
                              showSelection, blinkOn, size, a, colors::kPopoutCyan);
        }
        ++selectableIdx;
        continue;
      case FplSectionRow::Kind::DestinationBlank:
        drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                       withAlpha(colors::kPopoutCyan, a), showSelection, blinkOn,
                       a);
        ++selectableIdx;
        continue;
      default:
        continue;
    }

    ++selectableIdx;

    const MapLeg& rowLeg = legs[static_cast<std::size_t>(sr.legIndex)];
    const bool showActive = fplShowActiveNavRow(
        activeHighlight, sr.legIndex, activeLegIdx, rowLeg, navToIdent);

    if (showActive && !useConnector) {
      drawFplActiveLegArrow(r, listLeft, rowCy, size,
                            withAlpha(colors::kMagenta, a));
    }

    if (showActive) {
      drawFplActiveIdent(
          r, filledIdentX, rowCy, ident, size,
          fplRowIdentBlink(showSelection, showActive, sr.legIndex, activeLegIdx,
                           cursorLegIdx, cursorOn, listCursorRow,
                           activeSelectableRow, blinkOn),
          a);
    } else if (showSelection) {
      render::drawCursorSelect(r, filledIdentX, rowCy, ident, size,
                                 TextAlign::Left, blinkOn, a);
    } else {
      r.fillText(filledIdentX, rowCy, ident, size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
    }

    if (showActive) {
      drawFplActiveRowValues(r, dtkRight, rowRight, rowCy, size, smallSize,
                             d.fmaLegBearingDeg, d.fmaLegDistanceNm, a);
    } else if (sr.legIndex > 0) {
      const MapLeg& leg = legs[static_cast<std::size_t>(sr.legIndex)];
      const MapLeg& prev =
          legs[static_cast<std::size_t>(sr.legIndex - 1)];
      drawFplLegRowValues(r, dtkRight, rowRight, rowCy, size, smallSize, prev,
                          leg, withAlpha(colors::kWhitesmoke, a));
    }
  }

  // Scroll thumb when the list is longer than the window (WT ScrollBar).
  if (scrolling) {
    const float trackTop = bodyTop;
    const float trackH = rowH * static_cast<float>(visible);
    const float trackX = f.x + f.w - padX - scrollW;
    drawWtScrollBar(r, h, trackX, trackTop, trackH, displayRowCount, visible,
                    first, a);
  }

  // Modal confirmation prompt on top of everything else.
  const SoftkeyController::FplConfirm confirm = ui.flightPlanConfirm();
  if (confirm == SoftkeyController::FplConfirm::RemoveWaypoint) {
    drawFplConfirm(r, f, size, "Remove " + ui.flightPlanRemoveIdent(),
                   "from flight plan?", ui.flightPlanConfirmOk(), a);
  } else if (confirm == SoftkeyController::FplConfirm::RemoveDeparture ||
             confirm == SoftkeyController::FplConfirm::RemoveArrival ||
             confirm == SoftkeyController::FplConfirm::RemoveApproach ||
             confirm == SoftkeyController::FplConfirm::RemoveAirway) {
    // The procedure / airway subject (e.g. "KATL-BBABE.CHPPR1.RW08B" or
    // "Airway V16") gets its own line so it fits the narrow PFD window
    // (trainer "Remove <subject> from flight plan?").
    drawFplConfirm(r, f, size, "Remove", ui.flightPlanRemoveIdent(),
                   ui.flightPlanConfirmOk(), a, "from flight plan?");
  } else if (confirm == SoftkeyController::FplConfirm::DeleteFlightPlan) {
    drawFplConfirm(r, f, size, "Delete the active", "flight plan?",
                   ui.flightPlanConfirmOk(), a);
  }
}

}  // namespace avionics::pfd
