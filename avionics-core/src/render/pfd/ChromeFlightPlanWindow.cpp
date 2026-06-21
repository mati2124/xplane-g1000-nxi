#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/NavMath.h"
#include "render/pfd/ChromeInternal.h"
#include "render/pfd/PfdFlightPlanSections.h"

namespace avionics::pfd {
namespace {

// Centered modal confirmation box (Remove <wpt>? / Delete Flight Plan?) with an
// OK / CANCEL pair; the highlighted choice gets the cyan select plate.
void drawFplConfirm(Renderer& r, const WindowFrame& f, float size,
                    const std::string& line1, const std::string& line2,
                    bool okSelected, float a) {
  const float boxW = f.w * 0.82f;
  const float boxH = size * 6.2f;
  const float bx = f.x + (f.w - boxW) * 0.5f;
  const float by = f.top + (f.h - boxH) * 0.5f;
  const float radius = size * 0.4f;
  r.fillRoundedRect(bx, by, boxW, boxH, radius,
                    withAlpha(colors::kMfdPanelGray, a));
  r.strokeRoundedRect(bx, by, boxW, boxH, radius, 1.5f,
                      withAlpha(colors::kMenuBorderGray, a));

  const float cx = bx + boxW * 0.5f;
  r.fillText(cx, by + size * 1.2f, line1, size, TextAlign::Center,
             withAlpha(colors::kWhite, a));
  if (!line2.empty()) {
    r.fillText(cx, by + size * 2.3f, line2, size, TextAlign::Center,
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
                           float a) {
  const std::string prefix = icao + "-";
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
                     float size, const Color& color) {
  const std::string text =
      formatHeading(static_cast<float>(dtkDeg)) + "\u00b0";
  r.fillText(rightX, rowCy, text, size, TextAlign::Right, color);
}

void drawFplDisValue(Renderer& r, float rightX, float rowCy, double disNm,
                     float size, float smallSize, const Color& color) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.1f", disNm);
  const float numW = r.measureTextWidth(buf, size);
  const float nmW = r.measureTextWidth("NM", smallSize);
  const float gap = size * 0.06f;
  const float left = rightX - nmW - gap - numW;
  r.fillText(left, rowCy, buf, size, TextAlign::Left, color);
  r.fillText(left + numW + gap, rowCy, "NM", smallSize, TextAlign::Left, color);
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
  if (!leg.procedureRole.empty()) return leg.procedureRole;
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
                             const Color& roleColor, bool isCursorRow,
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
  if (isCursorRow) {
    const float tracking = size * 0.06f;
    const float tw = measureFplApproachLegIdentWidth(r, leg, transition, size,
                                                     smallSize);
    r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
               size * 1.24f, withAlpha(colors::kPopoutCyan, a));
    r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kBlack);
    if (!role.empty()) {
      const float roleX =
          x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
      r.fillText(roleX, cy, role, smallSize, TextAlign::Left, colors::kBlack);
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
  drawFplDtkValue(r, dtkRight, rowCy, dtkDeg, size, color);
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
  const float degW = r.measureTextWidth("\u00b0", smallSize);
  const float gap = size * 0.06f;
  const float left = rightX - degW - gap - dashRunW;
  drawTightDashRun(r, left, rowCy, dashCount, size, color);
  r.fillText(rightX, rowCy, "\u00b0", smallSize, TextAlign::Right, color);
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

void drawFplDashRow(Renderer& r, float x, float cy, int count, float size,
                    const Color& color, bool highlighted, float a) {
  const DashStyle ds = dashStyle(size);
  const float runW = ds.advance * static_cast<float>(count);
  if (highlighted) {
    r.fillRect(x, cy - size * 0.62f, runW, size * 1.24f,
               withAlpha(colors::kPopoutCyan, a));
    drawTightDashRun(r, x, cy, count, size, colors::kBlack);
  } else {
    drawTightDashRun(r, x, cy, count, size, color);
  }
}

// Draws "Origin - ____" / "Destination - ____" with optional cursor highlight.
// Blank idents use tight dashes (Direct-To style), not font underscore glyphs.
void drawOriginDestLine(Renderer& r, float x, float cy, const char* prefix,
                        const std::string& ident, bool blankIdent,
                        bool highlighted, float size, float a) {
  const float prefixW = r.measureTextWidth(prefix, size);
  if (highlighted) {
    const DashStyle ds = dashStyle(size);
    const float dashW =
        blankIdent ? ds.advance * static_cast<float>(kFplOriginDashCount)
                   : r.measureTextWidth(ident.c_str(), size);
    const float tracking = size * 0.06f;
    const float tw = prefixW + (blankIdent ? 0.0f : tracking) + dashW;
    r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
               size * 1.24f, withAlpha(colors::kPopoutCyan, a));
    r.fillText(x, cy, prefix, size, TextAlign::Left,
               withAlpha(colors::kBlack, a));
    if (blankIdent) {
      drawTightDashRun(r, x + prefixW, cy, kFplOriginDashCount, size,
                       colors::kBlack);
    } else {
      r.fillText(x + prefixW + tracking, cy, ident, size, TextAlign::Left,
                 withAlpha(colors::kBlack, a));
    }
  } else {
    r.fillText(x, cy, prefix, size, TextAlign::Left,
               withAlpha(colors::kPopoutCyan, a));
    if (blankIdent) {
      drawTightDashRun(r, x + prefixW, cy, kFplOriginDashCount, size,
                       withAlpha(colors::kPopoutCyan, a));
    } else {
      r.fillText(x + prefixW, cy, ident, size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
    }
  }
}

// Thin rule under the active leg row (WT .mfd-flightplan-hr).
void drawFplHrRule(Renderer& r, float left, float right, float y, float a) {
  r.strokeLine(left, y, right, y, 1.0f,
               withAlpha(Color{0.596f, 0.624f, 0.682f, 1.0f}, a));
}

}  // namespace

// Active Flight Plan window (FPL bezel key): the active flight-plan legs shown
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
  const float scrollW = fontPx(10.0f, h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX;
  const float listW = listRight - listLeft;

  // Column geometry mirrors the Working Title PFD .fpl layout: ident on the
  // left, DTK right edge ~58%, DIS at the list edge (keep columns separated so
  // wide DIS values do not overlap DTK).
  const float identX = listLeft + listW * 0.04f;
  const float approachLegX = identX + size * 1.35f;
  const float disRight = listRight;
  const float dtkRight = listLeft + listW * 0.58f;

  // Active navigation header + DTK/DIS column labels (WT Fig. 5-48).
  constexpr float kFplActiveRowPx = 22.0f;
  constexpr float kFplHeaderPx = 40.0f;
  constexpr float kFplBodyRowPx = 24.0f;
  const float activeLegSize = fontPx(16.0f, h);
  const bool approachLoaded = ui.flightPlanHasLoadedApproach();
  const int approachStart = ui.flightPlanApproachLegStart();
  const int approachCount = ui.flightPlanApproachLegCount();
  const std::string approachAirport =
      approachLoaded ? ui.flightPlanApproachAirportIcao() : std::string();
  const bool directToHeader =
      ui.mapDirectToActive() ||
      (d.fmaFromWpt.empty() && !d.fmaToWpt.empty() && legs.size() <= 1) ||
      (approachLoaded && approachStart <= 1 && !approachAirport.empty());
  const float activeLegCy = f.contentTop + fontPx(kFplActiveRowPx * 0.52f, h);
  const Color headerCyan = withAlpha(colors::kPopoutCyan, a);
  if (directToHeader) {
    const std::string dest =
        (approachLoaded && !approachAirport.empty())
            ? approachAirport
            : (!d.fmaToWpt.empty()
                   ? d.fmaToWpt
                   : (legs.empty() ? "_____" : legs.front().id));
    float hx = identX;
    hx = drawDirectToIcon(r, hx, activeLegCy, activeLegSize,
                          withAlpha(colors::kMagenta, a));
    hx += activeLegSize * 0.10f;
    const std::string slash = " / ";
    r.fillText(hx, activeLegCy, slash, activeLegSize, TextAlign::Left,
               headerCyan);
    hx += r.measureTextWidth(slash, activeLegSize);
    r.fillText(hx, activeLegCy, dest, activeLegSize, TextAlign::Left,
               headerCyan);
  } else {
    const std::string orig = legs.empty() ? "_____" : legs.front().id;
    const bool destFilled = ui.flightPlanDestinationFilled();
    const std::string dest =
        legs.size() >= 3 || (legs.size() == 2 && destFilled) ? legs.back().id
                                                             : "_____";
    r.fillText(identX, activeLegCy, orig + " / " + dest, activeLegSize,
               TextAlign::Left, headerCyan);
  }

  const float activeLegRuleY =
      f.contentTop + fontPx(kFplActiveRowPx + 1.0f, h);
  drawFplHrRule(r, listLeft, listRight, activeLegRuleY, a);

  const float dtkCy = f.contentTop + fontPx(kFplActiveRowPx + 10.0f, h);
  r.fillText(dtkRight, dtkCy, "DTK", smallSize, TextAlign::Right,
             withAlpha(colors::kWhite, a));
  r.fillText(disRight, dtkCy, "DIS", smallSize, TextAlign::Right,
             withAlpha(colors::kWhite, a));
  const float bodyTop = f.contentTop + fontPx(kFplHeaderPx, h);

  // During Direct-To the enroute template stays blank until a route is built.
  // A loaded approach keeps that template (Origin / Enroute) then shows the
  // approach header and legs below — not mixed into the Enroute section.
  const bool blankOriginSection =
      directToHeader ||
      (approachLoaded && approachStart <= 1 && !legs.empty() &&
       !approachAirport.empty() && legs.front().id == approachAirport);
  const int bodyLegCount =
      directToHeader && !approachLoaded && legs.size() <= 1
          ? 0
          : static_cast<int>(legs.size());
  const std::vector<FplDisplayRow> approachDisplayRows =
      approachLoaded
          ? buildFplApproachDisplayRows(approachStart, approachCount,
                                        blankOriginSection)
          : std::vector<FplDisplayRow>{};
  const std::vector<FplSectionRow> sectionRows =
      approachLoaded ? std::vector<FplSectionRow>{}
                     : buildFplSectionRows(bodyLegCount,
                                           ui.flightPlanDestinationFilled());
  const int sectionCursor = ui.flightPlanCursor();

  const int displayRowCount =
      approachLoaded ? static_cast<int>(approachDisplayRows.size())
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
    if (approachLoaded) {
      for (int i = 0; i < displayRowCount; ++i) {
        const FplDisplayRow& dr = approachDisplayRows[static_cast<std::size_t>(i)];
        if (dr.kind == FplDisplayRowKind::ApproachLeg &&
            dr.legIndex == sectionCursor) {
          cursorDisplayRow = i;
          break;
        }
      }
    } else {
      int selectableIdx = 0;
      for (int i = 0; i < displayRowCount; ++i) {
        const FplSectionRow::Kind kind =
            sectionRows[static_cast<std::size_t>(i)].kind;
        if (kind == FplSectionRow::Kind::EnrouteLabel) {
          continue;
        }
        if (selectableIdx == sectionCursor) {
          cursorDisplayRow = i;
          break;
        }
        ++selectableIdx;
      }
    }
    first = std::max(0, std::min(cursorDisplayRow - visible / 2,
                                 displayRowCount - visible));
  }
  const float disColumnRight =
      scrolling ? disRight - scrollW - fontPx(6.0f, h) : disRight;
  const float rowRight = disColumnRight;

  int selectableIdx = 0;
  const int end = std::min(displayRowCount, first + visible);
  for (int idx = first; idx < end; ++idx) {
    const float rowCy =
        bodyTop + rowH * (static_cast<float>(idx - first) + 0.5f);

    if (approachLoaded) {
      const FplDisplayRow& dr =
          approachDisplayRows[static_cast<std::size_t>(idx)];
      switch (dr.kind) {
        case FplDisplayRowKind::SepDash:
          drawFplDashRow(r, identX, rowCy, kFplApproachSepDashCount, size,
                         withAlpha(colors::kPopoutCyan, a), false, a);
          continue;
        case FplDisplayRowKind::ApproachHeader:
          drawFplApproachHeader(r, identX, rowCy, approachAirport,
                                ui.flightPlanApproachHeaderLabel(), size,
                                colors::kPopoutCyan, a);
          continue;
        case FplDisplayRowKind::ApproachLeg: {
          const int legIdx = dr.legIndex;
          const MapLeg& leg = legs[static_cast<std::size_t>(legIdx)];
          const bool isCursorRow = cursorOn && legIdx == sectionCursor;
          const bool active =
              !ui.activeWaypointId().empty() && leg.id == ui.activeWaypointId();
          const std::string approachTransition =
              ui.flightPlanApproachTransition();
          if (active) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          drawFplApproachLegIdent(
              r, approachLegX, rowCy, leg, approachTransition, size, smallSize,
              withAlpha(colors::kPopoutCyan, a),
              withAlpha(colors::kWhite, a), isCursorRow, active, blinkOn, a);
          if (active) {
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
          const bool isCursorRow = cursorOn && selectableIdx == sectionCursor;
          ++selectableIdx;
          drawOriginDestLine(r, identX, rowCy, "Origin - ",
                             dr.legIndex >= 0
                                 ? legs[static_cast<std::size_t>(dr.legIndex)].id
                                 : std::string(),
                             dr.legIndex < 0, isCursorRow, size, a);
          continue;
        }
        case FplDisplayRowKind::EnrouteLeg: {
          const bool isCursorRow = cursorOn && selectableIdx == sectionCursor;
          ++selectableIdx;
          const std::string ident =
              legs[static_cast<std::size_t>(dr.legIndex)].id;
          const bool active =
              !ui.activeWaypointId().empty() && ident == ui.activeWaypointId();
          if (active) {
            drawFplActiveLegArrow(r, listLeft, rowCy, size,
                                  withAlpha(colors::kMagenta, a));
          }
          if (active) {
            drawFplActiveIdent(r, identX, rowCy, ident, size, blinkOn, a);
          } else if (isCursorRow) {
            const float tracking = size * 0.06f;
            const float tw = r.measureTextWidth(ident.c_str(), size);
            r.fillRect(identX - tracking * 0.5f, rowCy - size * 0.62f,
                       tw + tracking, size * 1.24f,
                       withAlpha(colors::kPopoutCyan, a));
            r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                       withAlpha(colors::kBlack, a));
          } else {
            r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                       withAlpha(colors::kPopoutCyan, a));
          }
          if (active) {
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

    const bool isCursorRow =
        cursorOn && selectableIdx == sectionCursor;
    ++selectableIdx;

    std::string ident;
    const char* prefix = "";
    switch (sr.kind) {
      case FplSectionRow::Kind::Origin:
        prefix = "Origin - ";
        if (sr.legIndex >= 0) {
          ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
          drawOriginDestLine(r, identX, rowCy, prefix, ident, false,
                             isCursorRow, size, a);
        } else {
          drawOriginDestLine(r, identX, rowCy, prefix, ident, true, isCursorRow,
                             size, a);
        }
        continue;
      case FplSectionRow::Kind::EnrouteBlank:
        drawFplDashRow(r, identX, rowCy, kFplDashCount, size,
                       withAlpha(colors::kTitleGray, a), isCursorRow, a);
        continue;
      case FplSectionRow::Kind::EnrouteLeg:
        ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
        break;
      case FplSectionRow::Kind::Destination:
        prefix = "Destination - ";
        if (sr.legIndex >= 0) {
          ident = legs[static_cast<std::size_t>(sr.legIndex)].id;
          drawOriginDestLine(r, identX, rowCy, prefix, ident, false,
                             isCursorRow, size, a);
        } else {
          drawOriginDestLine(r, identX, rowCy, prefix, ident, true, isCursorRow,
                             size, a);
        }
        continue;
      default:
        break;
    }

    const bool active =
        sr.legIndex >= 0 && !ui.activeWaypointId().empty() &&
        legs[static_cast<std::size_t>(sr.legIndex)].id == ui.activeWaypointId();

    if (prefix[0] != '\0') {
      // Origin/Destination handled above.
    } else if (active) {
      drawFplActiveLegArrow(r, listLeft, rowCy, size,
                            withAlpha(colors::kMagenta, a));
      drawFplActiveIdent(r, identX, rowCy, ident, size, blinkOn, a);
    } else if (isCursorRow) {
      const float tracking = size * 0.06f;
      const float tw = r.measureTextWidth(ident.c_str(), size);
      r.fillRect(identX - tracking * 0.5f, rowCy - size * 0.62f, tw + tracking,
                 size * 1.24f, withAlpha(colors::kPopoutCyan, a));
      r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                 withAlpha(colors::kBlack, a));
    } else {
      r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                 withAlpha(colors::kPopoutCyan, a));
    }

    if (active) {
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
    const float thumbH = std::max(fontPx(18.0f, h),
                                  trackH * static_cast<float>(visible) /
                                      static_cast<float>(displayRowCount));
    const float maxScroll = static_cast<float>(displayRowCount - visible);
    const float thumbTop =
        trackTop + (trackH - thumbH) * static_cast<float>(first) / maxScroll;
    r.fillRect(trackX + scrollW * 0.5f - 1.0f, trackTop, 2.0f, trackH,
               withAlpha(colors::kPanelSeparator, a));
    r.fillRect(trackX, thumbTop, scrollW, thumbH,
               withAlpha(colors::kMenuBorderGray, a));
  }

  // Modal confirmation prompt on top of everything else.
  const SoftkeyController::FplConfirm confirm = ui.flightPlanConfirm();
  if (confirm == SoftkeyController::FplConfirm::RemoveWaypoint) {
    drawFplConfirm(r, f, size, "Remove " + ui.flightPlanRemoveIdent(),
                   "from flight plan?", ui.flightPlanConfirmOk(), a);
  } else if (confirm == SoftkeyController::FplConfirm::DeleteFlightPlan) {
    drawFplConfirm(r, f, size, "Delete the active", "flight plan?",
                   ui.flightPlanConfirmOk(), a);
  }
}

}  // namespace avionics::pfd
