#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/MapRange.h"
#include "avionics/NavMath.h"
#include "avionics/ProcedureMenuTypes.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/ChromeInternal.h"
#include "render/pfd/PfdFlightPlanSections.h"

using avionics::pfd::FplDisplayRow;
using avionics::pfd::FplDisplayRowKind;
using avionics::pfd::FplSectionRow;
using avionics::pfd::buildFplApproachDisplayRows;
using avionics::pfd::buildFplSectionRows;
using avionics::pfd::fplFilterDuplicateLegSectionRows;
using avionics::pfd::fplApproachDisplayRowIndexForSelectable;
using avionics::pfd::fplApproachDisplayRowSelectable;
using avionics::pfd::fplHeaderDestinationIdent;
using avionics::pfd::fplHeaderOriginIdent;
using avionics::pfd::fplSectionRowIsActiveDisplay;
using avionics::pfd::fplSectionRowIsSelectable;
using avionics::pfd::fplApproachSelectableRowForLegIndex;
using avionics::pfd::fplSectionSelectableRowForLegIndex;
using avionics::pfd::fplApproachDisplayRowIndexForLegIndex;
using avionics::pfd::fplApproachLegIndexForSelectable;
using avionics::pfd::fplLegIndexForSectionRow;
using avionics::pfd::fplListScrollFirst;
using avionics::pfd::fplPinActiveApproachLeg;
using avionics::pfd::fplShowActiveLegHighlight;
using avionics::pfd::fplShowListRowSelection;
using avionics::pfd::fplShowsDestinationBlankRow;
using avionics::fplApproachLayoutDestFilled;

namespace avionics::mfd {

namespace {

const char* fplFeatureTypeName(MapFeatureType type) {
  switch (type) {
    case MapFeatureType::Airport:
      return "AIRPORT";
    case MapFeatureType::Vor:
      return "VOR";
    case MapFeatureType::Ndb:
      return "NDB";
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      break;
  }
  return "INTERSECTION";
}

std::string fplActiveToIdent(const FlightData& d, const MapData& map,
                             const std::vector<MapLeg>& plan) {
  if (plan.empty()) return {};
  (void)map;
  return d.fmaToWpt;
}

void drawFplActiveIdentFlash(Renderer& r, float x, float cy,
                             const std::string& ident, float size, bool blinkOn) {
  const float tracking = size * 0.06f;
  const float tw = r.measureTextWidth(ident.c_str(), size);
  if (blinkOn) {
    r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
               size * 1.24f, colors::kMagenta);
    r.fillText(x, cy, ident, size, TextAlign::Left, colors::kBlack);
  } else {
    r.fillText(x, cy, ident, size, TextAlign::Left, colors::kMagenta);
  }
}

// Draws the FMS identifier entry cells starting at startX, baseline cy: the
// cursor cell as a pulsing highlight-select cell (WT InputComponent puts the
// pulsing focus class on the selected character even while active), the
// spell-ahead fill in cyan, and the typed characters in white (shared by the
// FPL insert and Direct-To windows). Cells are contiguous and near-monospace
// so the identifier reads as one tight field ("KEND"), matching the real unit
// rather than widely spaced letters. Returns the x just past the last cell
// (for placing the waypoint symbol).
float drawIdentEntryCells(Renderer& r, float startX, float cy,
                          const std::string& ident, int cursor, int typedCount,
                          bool selectAll, bool blinkOn, float displayH) {
  const float cellSize = mfdFontPx(kWtIdentLarge, displayH);
  // Advance by each glyph's actual width (the render font is proportional, so a
  // fixed-width cell would clip wide letters like 'W' into their neighbours),
  // with a little tracking so the identifier reads as one tight field.
  const float tracking = cellSize * 0.06f;
  float cx = startX;
  for (int i = 0; i < MfdController::kFplEntryMaxChars; ++i) {
    const char ch = i < static_cast<int>(ident.size()) ? ident[i] : '_';
    const bool isBlank = ch == '_';
    const bool highlightAll = selectAll && !isBlank;
    const bool isCursor = !selectAll && i == cursor;
    const bool cursorOn = isCursor && blinkOn;
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, cellSize);
    if (highlightAll || cursorOn) {
      r.fillRect(cx - tracking * 0.5f, cy - cellSize * 0.62f, chW + tracking,
                 cellSize * 1.24f, colors::kPopoutCyan);
    }
    const Color color = highlightAll || cursorOn ? colors::kBlack
                        : isCursor ? colors::kPopoutCyan  // blink-off half pulses cyan
                        : isBlank  ? colors::kPopoutCyan
                        : i >= typedCount ? colors::kPopoutCyan  // spell-ahead fill
                                          : colors::kWhite;
    r.fillText(cx, cy, text, cellSize, TextAlign::Left, color);
    cx += chW + tracking;
  }
  return cx;
}

// Draws the five-digit VNAV altitude-constraint entry field (ALT column),
// right-aligned to end at rightX with an "FT" suffix: the cursor cell shows in
// reverse video, the rest white (Pilot's Guide, Section 6 altitude entry).
void drawAltEntryCells(Renderer& r, float rightX, float cy,
                       const std::string& digits, int cursor, float displayH) {
  const float cellSize = mfdFontPx(kWtRow, displayH);
  const float cellW = cellSize * 0.66f;
  const float cellGap = cellSize * 0.08f;
  const int n = 5;
  const float unitW = mfdFontPx(34.0f, displayH);
  r.fillText(rightX, cy, "FT", cellSize * 0.8f, TextAlign::Right,
             colors::kWhitesmoke);
  float cx = rightX - unitW - (cellW + cellGap) * static_cast<float>(n);
  for (int i = 0; i < n; ++i) {
    const bool isCursor = i == cursor;
    const char ch = i < static_cast<int>(digits.size()) ? digits[i] : '0';
    const char text[2] = {ch, '\0'};
    if (isCursor) {
      r.fillRect(cx, cy - cellSize * 0.62f, cellW, cellSize * 1.24f,
                 colors::kPopoutCyan);
    }
    r.fillText(cx + cellW * 0.5f, cy, text, cellSize, TextAlign::Center,
               isCursor ? colors::kBlack : colors::kWhite);
    cx += cellW + cellGap;
  }
}

constexpr int kFplApproachSepDashCount = 7;
constexpr int kFplDashCount = 5;
constexpr int kFplOriginDashCount = 4;

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

// Direct-To window placeholder dash counts (trainer MFD Direct To window).
constexpr int kDtoCityDashCount = 16;
constexpr int kDtoNameDashCount = 15;
constexpr int kDtoRegionDashCount = 10;
constexpr int kDtoAltDashCount = 5;

bool mapFeatureHasGeo(const MapFeature& f) {
  return f.lat != 0.0 || f.lon != 0.0;
}

const MapFeature* directToInsetCenter(const MapData& map, bool hasMatch,
                                    const MapFeature& wpt,
                                    MapFeature& resolved) {
  if (!hasMatch) return nullptr;
  resolved = wpt;
  if (!mapFeatureHasGeo(resolved)) {
    for (const MapFeature& f : map.features) {
      if (f.id == resolved.id && mapFeatureHasGeo(f)) {
        resolved = f;
        break;
      }
    }
  }
  return mapFeatureHasGeo(resolved) ? &resolved : nullptr;
}

float textCyForDashBottom(Renderer& r, float dashCy, const DashStyle& ds,
                          float textSize) {
  const float dashBottom = dashCy + ds.height * 0.5f;
  const TextRect tr =
      r.measureTextRect(0.0f, 0.0f, "X", textSize, TextAlign::Left);
  return dashBottom - tr.bottom;
}

void drawRightTightDashRun(Renderer& r, float rightX, float dashCy, int count,
                           float size, const Color& color) {
  drawTightDashRun(r, rightX - tightDashRunWidth(count, size), dashCy, count,
                   size, color);
}

// Ident entry cells for the MFD Direct-To window: tight dash placeholders and
// the pulsing cyan cursor plate, matching the real unit and the PFD popout.
float drawDtoIdentEntryCells(Renderer& r, float startX, float dashCy,
                             const std::string& ident, int cursor,
                             int typedCount, bool selectAll, bool blinkOn,
                             float displayH) {
  const float cellSize = mfdFontPx(kWtDtoIdent, displayH);
  const float tracking = cellSize * 0.06f;
  const DashStyle ds = dashStyle(cellSize);
  const float entryTextCy = textCyForDashBottom(r, dashCy, ds, cellSize);
  const TextRect entryRect =
      r.measureTextRect(0.0f, entryTextCy, "X", cellSize, TextAlign::Left);
  const float plateTop = entryRect.top;
  const float plateH = dashCy + ds.height * 0.5f - plateTop + 1.0f;
  float cx = startX;
  for (int i = 0; i < MfdController::kFplEntryMaxChars; ++i) {
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
      drawTightDash(r, cx + (ds.advance - ds.width) * 0.5f, dashCy, ds,
                    dashColor);
      cx += ds.advance;
      continue;
    }
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, cellSize);
    if (highlightAll || cursorOn) {
      r.fillRect(cx - tracking * 0.5f, plateTop, chW + tracking, plateH,
                 colors::kPopoutCyan);
    }
    const Color color = highlightAll || cursorOn ? colors::kBlack
                        : isCursor        ? colors::kPopoutCyan
                        : i >= typedCount ? colors::kPopoutCyan
                                          : colors::kWhite;
    r.fillText(cx, entryTextCy, text, cellSize, TextAlign::Left, color);
    cx += chW + tracking;
  }
  return cx;
}

void drawDtoDisPlaceholder(Renderer& r, float rightX, float pad, float dashCy,
                           float readoutSize, const Color& color) {
  const DashStyle ds = dashStyle(readoutSize);
  const float readoutCy = textCyForDashBottom(r, dashCy, ds, readoutSize);
  const float smallSize = readoutSize * kUnitEm;
  const float smallCy = textCyForDashBottom(r, dashCy, dashStyle(smallSize),
                                            smallSize);
  const char* disNm = "NM";
  const float disNmW = r.measureTextWidth(disNm, smallSize);
  r.fillText(rightX - pad, smallCy, disNm, smallSize, TextAlign::Right, color);
  float disX = rightX - pad - disNmW;
  disX -= ds.advance * 3.0f;
  drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
  disX += ds.advance;
  drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
  disX += ds.advance;
  r.fillText(disX, readoutCy, ".", readoutSize, TextAlign::Left, color);
  disX += r.measureTextWidth(".", readoutSize) * 0.55f;
  drawTightDash(r, disX + (ds.advance - ds.width) * 0.5f, dashCy, ds, color);
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

void drawFplDashRow(Renderer& r, float x, float cy, int count, float size,
                    const Color& color, bool highlighted, bool blinkOn) {
  const DashStyle ds = dashStyle(size);
  const float runW = ds.advance * static_cast<float>(count);
  if (highlighted && blinkOn) {
    r.fillRect(x, cy - size * 0.62f, runW, size * 1.24f, colors::kPopoutCyan);
    drawTightDashRun(r, x, cy, count, size, colors::kBlack);
  } else if (highlighted) {
    drawTightDashRun(r, x, cy, count, size, colors::kPopoutCyan);
  } else {
    drawTightDashRun(r, x, cy, count, size, color);
  }
}

void drawFplApproachHeader(Renderer& r, float x, float cy, const std::string& icao,
                           const std::string& label, float size,
                           const Color& color) {
  const std::string prefix = icao + "-";
  float xx = x;
  r.fillText(xx, cy, prefix, size, TextAlign::Left, color);
  xx += r.measureTextWidth(prefix, size);
  if (labelIsRnavGps(label)) {
    const float subSize = size * 0.72f;
    r.fillText(xx, cy, "RNAV", size, TextAlign::Left, color);
    xx += r.measureTextWidth("RNAV", size) + size * 0.08f;
    r.fillText(xx, cy, "GPS", subSize, TextAlign::Left, color);
    xx += r.measureTextWidth("GPS", subSize);
    const std::string rest = restAfterRnavGps(label);
    if (!rest.empty()) {
      r.fillText(xx, cy, " " + rest, size, TextAlign::Left, color);
    }
  } else {
    r.fillText(xx, cy, label, size, TextAlign::Left, color);
  }
}

// Rendered width of an approach label, accounting for the smaller "GPS" affix
// on RNAV GPS procedures (matches drawApproachLabel below).
float measureApproachLabelWidth(Renderer& r, const std::string& label,
                                float size) {
  if (!labelIsRnavGps(label)) return r.measureTextWidth(label, size);
  const float subSize = size * 0.72f;
  float w = r.measureTextWidth("RNAV", size) + size * 0.08f +
            r.measureTextWidth("GPS", subSize);
  const std::string rest = restAfterRnavGps(label);
  if (!rest.empty()) w += r.measureTextWidth(" " + rest, size);
  return w;
}

// Left-aligned approach label with the PFD's RNAV GPS styling: "RNAV" at full
// size, a smaller "GPS" (no underscore), then the rest (e.g. "05 LPV"). Plain
// labels (ILS/VOR/VISUAL) draw unchanged.
void drawApproachLabel(Renderer& r, float x, float cy, const std::string& label,
                       float size, const Color& color) {
  if (!labelIsRnavGps(label)) {
    r.fillText(x, cy, label, size, TextAlign::Left, color);
    return;
  }
  const float subSize = size * 0.72f;
  float xx = x;
  r.fillText(xx, cy, "RNAV", size, TextAlign::Left, color);
  xx += r.measureTextWidth("RNAV", size) + size * 0.08f;
  r.fillText(xx, cy, "GPS", subSize, TextAlign::Left, color);
  xx += r.measureTextWidth("GPS", subSize);
  const std::string rest = restAfterRnavGps(label);
  if (!rest.empty()) {
    r.fillText(xx, cy, " " + rest, size, TextAlign::Left, color);
  }
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

void drawFplApproachLegIdent(Renderer& r, float x, float cy, const MapLeg& leg,
                             const std::string& transition, float size,
                             float smallSize, const Color& identColor,
                             const Color& roleColor, bool isCursor,
                             bool active, bool blinkOn) {
  const std::string role = fplApproachLegRole(leg, transition);
  const float tracking = size * 0.06f;
  if (active) {
    const float tw = r.measureTextWidth(leg.id.c_str(), size) +
                     (role.empty() ? 0.0f
                                   : size * 0.14f +
                                         r.measureTextWidth(role.c_str(), smallSize));
    if (blinkOn) {
      r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
                 size * 1.24f, colors::kMagenta);
      r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kBlack);
    } else {
      r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kMagenta);
    }
    if (!role.empty()) {
      const float roleX = x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
      const Color rc = blinkOn ? colors::kBlack : colors::kMagenta;
      r.fillText(roleX, cy, role, smallSize, TextAlign::Left, rc);
    }
    return;
  }
  if (isCursor) {
    const float tw = r.measureTextWidth(leg.id.c_str(), size) +
                     (role.empty() ? 0.0f
                                   : size * 0.14f +
                                         r.measureTextWidth(role.c_str(), smallSize));
    if (blinkOn) {
      r.fillRect(x - tracking * 0.5f, cy - size * 0.62f, tw + tracking,
                 size * 1.24f, colors::kPopoutCyan);
      r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kBlack);
      if (!role.empty()) {
        const float roleX = x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left, colors::kBlack);
      }
    } else {
      r.fillText(x, cy, leg.id, size, TextAlign::Left, colors::kPopoutCyan);
      if (!role.empty()) {
        const float roleX = x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
        r.fillText(roleX, cy, role, smallSize, TextAlign::Left, colors::kWhite);
      }
    }
    return;
  }
  r.fillText(x, cy, leg.id, size, TextAlign::Left, identColor);
  if (!role.empty()) {
    const float roleX = x + r.measureTextWidth(leg.id.c_str(), size) + size * 0.14f;
    r.fillText(roleX, cy, role, smallSize, TextAlign::Left, roleColor);
  }
}

void drawOriginDestLine(Renderer& r, float x, float cy, const char* prefix,
                        const std::string& ident, bool blank, float size,
                        const Color& color, bool highlighted, bool blinkOn) {
  r.fillText(x, cy, prefix, size, TextAlign::Left, color);
  const float prefixW = r.measureTextWidth(prefix, size);
  const float fieldX = x + prefixW;
  if (blank) {
    const DashStyle ds = dashStyle(size);
    const float dashW =
        ds.advance * static_cast<float>(kFplOriginDashCount);
    if (highlighted && blinkOn) {
      r.fillRect(fieldX, cy - size * 0.62f, dashW, size * 1.24f,
                 colors::kPopoutCyan);
      drawTightDashRun(r, fieldX, cy, kFplOriginDashCount, size,
                       colors::kBlack);
    } else if (highlighted) {
      drawTightDashRun(r, fieldX, cy, kFplOriginDashCount, size,
                       colors::kPopoutCyan);
    } else {
      drawTightDashRun(r, fieldX, cy, kFplOriginDashCount, size, color);
    }
  } else if (highlighted) {
    drawCursorSelect(r, fieldX, cy, ident, size, TextAlign::Left, blinkOn);
  } else {
    r.fillText(fieldX, cy, ident, size, TextAlign::Left, color);
  }
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

void drawFplSectionIdent(Renderer& r, float x, float cy, const char* prefix,
                         const std::string& ident, bool blankIdent,
                         bool highlighted, bool blinkOn, float size,
                         const Color& textColor) {
  if (blankIdent) {
    drawOriginDestLine(r, x, cy, prefix, ident, true, size, textColor,
                       highlighted, blinkOn);
    return;
  }
  if (highlighted) {
    drawCursorSelect(r, x, cy, ident, size, TextAlign::Left, blinkOn);
  } else {
    r.fillText(x, cy, ident, size, TextAlign::Left, textColor);
  }
}

void drawFplDestinationLabelRow(Renderer& r, float x, float cy, float size) {
  const Color c = colors::kPopoutCyan;
  const char* prefix = "Destination - ";
  r.fillText(x, cy, prefix, size, TextAlign::Left, c);
  float rx = x + r.measureTextWidth(prefix, size);
  r.fillText(rx, cy, "RW", size, TextAlign::Left, c);
  rx += r.measureTextWidth("RW", size);
  drawTightDashRun(r, rx, cy, 2, size, c);
}

bool sectionRowIsSelectable(const FplSectionRow& sr, int legCount,
                            bool destinationFilled) {
  return fplSectionRowIsSelectable(sr, legCount, destinationFilled);
}

int sectionDisplayRowForSelectable(int selectableRow,
                                   const std::vector<FplSectionRow>& rows,
                                   int legCount, bool destinationFilled) {
  int sel = 0;
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    if (!sectionRowIsSelectable(rows[static_cast<std::size_t>(i)], legCount,
                                destinationFilled)) {
      continue;
    }
    if (sel == selectableRow) return i;
    ++sel;
  }
  return 0;
}

void countSectionSelectablesBefore(int before,
                                   const std::vector<FplSectionRow>& rows,
                                   int legCount, bool destinationFilled,
                                   int& selectableIdx) {
  for (int i = 0; i < before; ++i) {
    if (sectionRowIsSelectable(rows[static_cast<std::size_t>(i)], legCount,
                               destinationFilled)) {
      ++selectableIdx;
    }
  }
}

void drawFplLegRow(Renderer& r, const FlightData& d, const MapData& map,
                   const std::vector<MapLeg>& plan, int legIdx,
                   const std::string& activeToIdent, int activeLegIdx,
                   float innerX, float fixX, float colDtkR, float colDisR,
                   float colAltR, float cy, float rowSize, float rowH,
                   float displayH, const MfdController& ui, bool showSelection,
                   bool showActive, bool activeNavBlink, bool useApproachIdent,
                   const std::string& approachTransition) {
  if (legIdx < 0 || legIdx >= static_cast<int>(plan.size())) return;
  (void)activeToIdent;
  (void)activeLegIdx;
  const MapLeg& leg = plan[static_cast<std::size_t>(legIdx)];
  const Color rowColor = showActive ? colors::kMagenta : colors::kWhitesmoke;
  char buf[24];

  const bool identSel =
      showSelection &&
      (!ui.fplCursorOn() ||
       ui.fplCursorCol() == MfdController::FplCursorCol::Ident);
  const bool altSel =
      showSelection && ui.fplCursorOn() &&
      ui.fplCursorCol() == MfdController::FplCursorCol::Altitude;
  const bool rowBlink =
      identSel ? ui.blinkOn()
               : (showActive && activeNavBlink ? ui.blinkOn() : false);

  if (showActive) {
    const float ax = innerX;
    const Point arrow[7] = {
        {ax + rowSize * 1.0f, cy},
        {ax + rowSize * 0.65f, cy - rowSize * 0.35f},
        {ax + rowSize * 0.65f, cy - rowSize * 0.10f},
        {ax, cy - rowSize * 0.10f},
        {ax, cy + rowSize * 0.10f},
        {ax + rowSize * 0.65f, cy + rowSize * 0.10f},
        {ax + rowSize * 0.65f, cy + rowSize * 0.35f}};
    r.fillPolygon(arrow, 7, colors::kMagenta);
  }

  if (useApproachIdent) {
    drawFplApproachLegIdent(r, fixX, cy, leg, approachTransition, rowSize,
                            rowSize * 0.72f, colors::kCyan, colors::kWhite,
                            identSel, showActive, rowBlink);
  } else if (showActive) {
    drawFplActiveIdentFlash(r, fixX, cy, leg.id, rowSize,
                            activeNavBlink ? ui.blinkOn() : false);
  } else if (identSel) {
    drawCursorSelect(r, fixX, cy, leg.id, rowSize, TextAlign::Left,
                     ui.blinkOn());
  } else {
    r.fillText(fixX, cy, leg.id, rowSize, TextAlign::Left, colors::kCyan);
  }

  if (legIdx > 0) {
    const MapLeg& prev = plan[static_cast<std::size_t>(legIdx - 1)];
    const double dtk =
        showActive ? static_cast<double>(d.fmaLegBearingDeg)
                   : navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
    const double dis =
        showActive ? static_cast<double>(d.fmaLegDistanceNm)
                   : navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
    std::snprintf(buf, sizeof(buf), "%03.0f", dtk);
    drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, rowColor);
    std::snprintf(buf, sizeof(buf), "%.1f", dis);
    drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, rowColor);
  }

  if (ui.fplAltEntryActive() && ui.fplAltEntryRow() == legIdx) {
    drawAltEntryCells(r, colAltR, cy, ui.fplAltEntryDigits(),
                      ui.fplAltEntryCursor(), displayH);
  } else if (leg.altitudeConstraint != AltConstraintType::None &&
             leg.altitudeConstraintFt > 0) {
    std::snprintf(buf, sizeof(buf), "%d", leg.altitudeConstraintFt);
    const Color altColor =
        leg.altitudeDesignated ? colors::kCyan : colors::kWhite;
    if (altSel) {
      std::string altText = std::string(buf) + "FT";
      drawCursorSelect(r, colAltR, cy, altText, rowSize, TextAlign::Right,
                       ui.blinkOn());
    } else {
      drawValueWithUnit(r, colAltR, cy, buf, "FT", rowSize, altColor);
    }
  } else if (altSel) {
    drawCursorSelect(r, colAltR, cy, "_____FT", rowSize, TextAlign::Right,
                     ui.blinkOn());
  } else {
    drawValueWithUnit(r, colAltR, cy, "_____", "FT", rowSize,
                      colors::kWhitesmoke);
  }
  (void)rowH;
}

// Draws the resolved-waypoint description (type/region) and the geographic
// line (bearing/distance from ownship), or the not-found caution. Returns the
// next y below the block.
float drawWaypointMatchInfo(Renderer& r, const Rect& inner, float fy,
                            float rowH, bool notFound, bool hasMatch,
                            const MapFeature& match, const MapData& map,
                            float displayH) {
  const float rowSize = mfdFontPx(16.0f, displayH);
  if (notFound) {
    r.fillText(inner.x, fy, "WAYPOINT NOT FOUND", rowSize, TextAlign::Left,
               colors::kBandYellow);
    return fy + rowH * 0.8f;
  }
  if (hasMatch) {
    // Prefer the published facility name (the real entry window shows it);
    // fall back to the feature type plus region.
    std::string desc = match.name.empty() ? fplFeatureTypeName(match.type)
                                          : match.name;
    if (!match.region.empty()) desc += "  " + match.region;
    while (desc.size() > 4 && r.measureTextWidth(desc, rowSize) > inner.w) {
      desc.pop_back();
    }
    r.fillText(inner.x, fy, desc, rowSize, TextAlign::Left,
               colors::kWhitesmoke);
    if (map.positionValid) {
      const double brg = navBearingDeg(map.ownshipLat, map.ownshipLon,
                                       match.lat, match.lon);
      const double dis = navDistanceNm(map.ownshipLat, map.ownshipLon,
                                       match.lat, match.lon);
      char buf[32];
      std::snprintf(buf, sizeof(buf), "BRG %03.0f%s  DIS %.1fNM", brg, kDeg,
                    dis);
      r.fillText(inner.x, fy + rowH * 0.8f, buf, rowSize, TextAlign::Left,
                 colors::kWhitesmoke);
    }
    return fy + rowH * 1.6f;
  }
  return fy;
}

// The Waypoint Information entry window opened by the small FMS knob: the
// spelled identifier in character cells above the matched waypoint's
// description.
void drawFplEntryWindow(Renderer& r, const MfdController& ui,
                        const MapData& map, const Rect& panel,
                        float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float rowH = mfdFontPx(kWtFieldValue, displayH) * 1.9f;
  const float margin = P(6.0f);
  const float promptReserve = mfdFontPx(20.0f, displayH) + P(8.0f);
  const float boxW = panel.w - 2.0f * margin;
  const float boxH = rowH * 4.6f;
  Rect inner = drawDialog(
      r, Rect{panel.x + margin, panel.y + panel.h - boxH - promptReserve, boxW,
              boxH},
      "Waypoint Information", displayH);

  const float cy = inner.y + rowH * 0.8f;
  drawIdentEntryCells(r, inner.x, cy, ui.fplEntryIdent(), ui.fplEntryCursor(),
                      ui.fplEntryTypedCount(), false, ui.blinkOn(), displayH);
  drawWaypointMatchInfo(r, inner, cy + rowH * 1.1f, rowH, ui.fplEntryNotFound(),
                        ui.fplEntryHasMatch(), ui.fplEntryMatch(), map,
                        displayH);
}

}  // namespace

void drawDirectToWindow(Renderer& r, const FlightData& d, const MapData& map,
                        const MfdController& ui, float x, float y, float w,
                        float h, float displayH) {
  // Real MFD Direct-To window (Pilot's Guide Figure 5-44 "Direct-to Window -
  // MFD"): a tall right-side popup of stacked, labelled group boxes -- Ident/
  // Facility/City, VNV, Map, Location (bearing/distance), Course -- over the
  // Activate? / Hold? buttons.
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float boxH = h - P(18.0f);
  const float boxW = boxH / 2.17f;  // real window aspect (Fig 5-44 ~2.17:1)
  Rect inner =
      drawDialog(r, Rect{x + w - boxW - P(12.0f), y + P(9.0f), boxW, boxH},
                 "Direct To", displayH);

  const bool hasMatch = ui.directToHasMatch();
  const MapFeature& wpt = ui.directToMatch();
  const float gap = P(4.0f);
  const float labelSize = mfdFontPx(kWtDtoLabel, displayH);
  const float valueSize = mfdFontPx(kWtDtoValue, displayH);
  const float rowSize = mfdFontPx(kWtDtoFace, displayH);

  // Bearing / distance / direct course from the present position.
  double brg = 0.0;
  double dis = 0.0;
  const bool haveGeo = hasMatch && map.positionValid;
  if (haveGeo) {
    brg = navBearingDeg(map.ownshipLat, map.ownshipLon, wpt.lat, wpt.lon);
    dis = navDistanceNm(map.ownshipLat, map.ownshipLon, wpt.lat, wpt.lon);
  }
  char buf[16];

  // ---- top stack: Ident/Facility/City, then VNV ----
  float topY = inner.y;
  {
    Rect ic = drawGroupBox(r, Rect{inner.x, topY, inner.w, P(94.0f)},
                           "Ident, Facility, City", displayH, colors::kBlack);
    topY += P(94.0f) + gap;
    const float identSize = mfdFontPx(kWtDtoIdent, displayH);
    const float regionSize = mfdFontPx(kWtDtoFace, displayH);
    const float cy1 = ic.y + identSize;
    const float identEnd = drawDtoIdentEntryCells(
        r, ic.x, cy1, ui.directToIdent(), ui.directToCursor(),
        ui.directToTypedCount(), ui.directToSelectAll(), ui.blinkOn(),
        displayH);
    if (hasMatch) {
      drawWaypointIcon(r, identEnd + P(12.0f), cy1, P(16.0f), &wpt, wpt.type);
      if (wpt.region.empty()) {
        drawRightTightDashRun(r, ic.x + ic.w, cy1, kDtoRegionDashCount,
                              regionSize, colors::kWhite);
      } else {
        r.fillText(ic.x + ic.w, cy1, wpt.region, regionSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
    } else {
      drawRightTightDashRun(r, ic.x + ic.w, cy1, kDtoRegionDashCount,
                            regionSize, colors::kWhite);
    }
    // Facility name and city rows (Fig 5-45). The feature type stands in for
    // the facility name when the database has none.
    const float cy2 = cy1 + rowSize * 1.5f;
    const float cy3 = cy2 + rowSize * 1.3f;
    if (ui.directToNotFound()) {
      r.fillText(ic.x, cy2, "WAYPOINT NOT FOUND", rowSize, TextAlign::Left,
                 colors::kBandYellow);
    } else if (hasMatch) {
      auto clipRow = [&](std::string text) {
        while (text.size() > 4 &&
               r.measureTextWidth(text, rowSize) > ic.w) {
          text.pop_back();
        }
        return text;
      };
      r.fillText(ic.x, cy2,
                 clipRow(wpt.name.empty() ? fplFeatureTypeName(wpt.type)
                                          : wpt.name),
                 rowSize, TextAlign::Left, colors::kPopoutCyan);
      if (wpt.city.empty()) {
        drawTightDashRun(r, ic.x, cy3, kDtoCityDashCount, rowSize,
                         colors::kPopoutCyan);
      } else {
        r.fillText(ic.x, cy3, clipRow(wpt.city), rowSize, TextAlign::Left,
                   colors::kPopoutCyan);
      }
    } else {
      drawTightDashRun(r, ic.x, cy2, kDtoNameDashCount, rowSize,
                       colors::kPopoutCyan);
      drawTightDashRun(r, ic.x, cy3, kDtoCityDashCount, rowSize,
                       colors::kPopoutCyan);
    }
  }
  {
    Rect vc =
        drawGroupBox(r, Rect{inner.x, topY, inner.w, P(50.0f)}, "VNV", displayH,
                     colors::kBlack);
    topY += P(50.0f) + gap;
    const float cy = vc.y + vc.h * 0.5f;
    const float altSmallCy =
        textCyForDashBottom(r, cy, dashStyle(valueSize * kUnitEm),
                            valueSize * kUnitEm);
    float ax = vc.x + P(8.0f);
    ax = drawTightDashRun(r, ax, cy, kDtoAltDashCount, valueSize,
                          colors::kPopoutCyan);
    r.fillText(ax, altSmallCy, "FT", valueSize * kUnitEm, TextAlign::Left,
               colors::kPopoutCyan);
    drawValueWithUnit(r, vc.x + vc.w, cy, "+0", "NM", valueSize,
                      colors::kPopoutCyan);
  }

  // ---- bottom stack (laid out upward): buttons, Course, Location ----
  // Readout values (BRG/DIS/Course) draw larger than the small field labels,
  // matching the real unit.
  const float readoutSize = mfdFontPx(kWtDtoReadout, displayH);
  const float buttonsH = hasMatch ? valueSize * 1.7f : 0.0f;
  const float buttonGap = hasMatch ? P(18.0f) : P(4.0f);
  const float buttonsY = inner.y + inner.h - P(4.0f) - buttonsH;
  const float courseH = P(42.0f);
  const float courseY = buttonsY - buttonGap - courseH;
  const float locH = P(52.0f);
  const float locY = courseY - gap - locH;

  // Location box (full width): bearing and distance from present position.
  {
    Rect lc = drawGroupBox(r, Rect{inner.x, locY, inner.w, locH}, "Location",
                           displayH, colors::kBlack);
    const float cy = lc.y + lc.h * 0.5f;
    r.fillText(lc.x, cy, "BRG", labelSize, TextAlign::Left, colors::kTitleGray);
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f", brg);
      drawValueWithUnit(r, lc.x + lc.w * 0.46f, cy, buf, kDeg, readoutSize,
                        colors::kWhite);
    } else {
      drawValueWithUnit(r, lc.x + lc.w * 0.46f, cy, "360", kDeg, readoutSize,
                        colors::kWhite);
    }
    r.fillText(lc.x + lc.w * 0.52f, cy, "DIS", labelSize, TextAlign::Left,
               colors::kTitleGray);
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      drawValueWithUnit(r, lc.x + lc.w, cy, buf, "NM", readoutSize,
                        colors::kWhite);
    } else {
      drawDtoDisPlaceholder(r, lc.x + lc.w, P(2.0f), cy, readoutSize,
                            colors::kWhite);
    }
  }
  // Course box (about half width): the GPS direct course (cyan), equal to the
  // bearing until the direct-to is activated.
  {
    Rect cc = drawGroupBox(r, Rect{inner.x, courseY, inner.w * 0.52f, courseH},
                           "Course", displayH, colors::kBlack);
    const float cy = cc.y + cc.h * 0.5f;
    const float vx = cc.x + P(12.0f);
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f", brg);
      r.fillText(vx, cy, buf, readoutSize, TextAlign::Left,
                 colors::kPopoutCyan);
      r.fillText(vx + r.measureTextWidth(buf, readoutSize), cy, kDeg,
                 readoutSize * kUnitEm, TextAlign::Left, colors::kPopoutCyan);
    } else {
      r.fillText(vx, cy, "360", readoutSize, TextAlign::Left,
                 colors::kPopoutCyan);
      r.fillText(vx + r.measureTextWidth("360", readoutSize), cy, kDeg,
                 readoutSize * kUnitEm, TextAlign::Left, colors::kPopoutCyan);
    }
  }

  // ---- Map box fills the middle ----
  {
    const float mapBot = locY - gap;
    Rect mc = drawGroupBox(r, Rect{inner.x, topY, inner.w, mapBot - topY},
                           "Map", displayH, colors::kBlack);
    float dtoRangeNm = mfd::directToInsetRangeNm(map, wpt);
    MapFeature dtoCenter;
    const MapFeature* center = directToInsetCenter(map, hasMatch, wpt, dtoCenter);
    drawPageMap(r, d, map, mc, dtoRangeNm, center, displayH, false, nullptr,
                0.0f, TerrainDisplay::Off, true);
  }

  // ---- Activate? / Hold? buttons ----
  // Text-sized rounded-rect buttons with a thin gray outline; Activate? at the
  // left, Hold? at the right. Activate? pulses cyan when armed (~1 Hz).
  if (hasMatch) {
    auto drawButton = [&](float leftX, const char* label, bool armed) {
      const float bw = r.measureTextWidth(label, valueSize) + valueSize * 1.3f;
      const Rect b{leftX, buttonsY, bw, buttonsH};
      Point pts[kRoundedRectPoints + 1];
      const int closed = buildRoundedRect(pts, b, buttonsH * 0.32f);
      const bool blinkOn = ui.blinkOn();
      if (armed && blinkOn) r.fillPolygon(pts, kRoundedRectPoints, colors::kCyan);
      r.strokePolyline(pts, closed, 1.0f, colors::kGroupBoxBorder);
      const Color textColor =
          armed ? (blinkOn ? colors::kBlack : colors::kCyan) : colors::kWhitesmoke;
      r.fillText(b.x + b.w * 0.5f, b.y + b.h * 0.5f, label, valueSize,
                 TextAlign::Center, textColor);
      return bw;
    };
    drawButton(inner.x + P(6.0f), "Activate?", ui.directToArmed());
    const float holdW = r.measureTextWidth("Hold?", valueSize) + valueSize * 1.3f;
    drawButton(inner.x + inner.w - holdW - P(6.0f), "Hold?", false);
  }
}

namespace {

// The OK/CANCEL confirmation window ("Remove <wpt>?" from CLR on a leg row,
// "Delete all waypoints in flight plan?" from the page menu).
void drawFplConfirmWindow(Renderer& r, const MfdController& ui, float x,
                          float y, float w, float h, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  const float rowH = mfdFontPx(kWtFieldValue, displayH) * 1.9f;
  const float boxW = w * 0.36f;
  const float boxH = rowH * 3.6f;
  Rect inner = drawDialog(
      r, Rect{x + (w - boxW) * 0.5f, y + (h - boxH) * 0.38f, boxW, boxH},
      nullptr, displayH);

  const float rowSize = mfdFontPx(18.0f, displayH);
  const std::string question =
      ui.fplConfirm() == MfdController::FplConfirm::RemoveWaypoint
          ? "Remove " + ui.fplRemoveIdent() + "?"
          : "Delete all waypoints in flight plan?";
  r.fillText(inner.x + inner.w * 0.5f, inner.y + rowH * 0.6f, question,
             rowSize, TextAlign::Center, colors::kWhite);

  // OK / CANCEL choices; ENT executes the highlighted one, the FMS knob
  // moves the highlight.
  const float choiceY = inner.y + rowH * 2.0f;
  const float choiceSize = mfdFontPx(kWtFieldValue, displayH);
  struct Choice {
    const char* label;
    float cx;
    bool highlighted;
  };
  const Choice choices[2] = {
      {"OK", inner.x + inner.w * 0.32f, ui.fplConfirmOk()},
      {"CANCEL", inner.x + inner.w * 0.68f, !ui.fplConfirmOk()},
  };
  for (const Choice& c : choices) {
    if (c.highlighted) {
      drawCursorSelect(r, c.cx, choiceY, c.label, choiceSize, TextAlign::Center,
                       ui.blinkOn());
    } else {
      r.fillText(c.cx, choiceY, c.label, choiceSize, TextAlign::Center,
                 colors::kWhite);
    }
  }
}

// ---- Procedures window (PROC key) ----

void drawProcLoadedDashes(Renderer& r, float rightX, float cy, float size) {
  drawRightTightDashRun(r, rightX, cy, 4, size, colors::kWhite);
}

void drawProcCarrot(Renderer& r, float cx, float cy, float size, bool pointRight,
                    bool enabled) {
  if (!enabled) return;
  const float aw = size * 0.34f;
  if (pointRight) {
    const Point tri[3] = {{cx + aw * 0.55f, cy},
                          {cx - aw * 0.30f, cy - aw},
                          {cx - aw * 0.30f, cy + aw}};
    r.fillPolygon(tri, 3, colors::kPopoutCyan);
  } else {
    const Point tri[3] = {{cx - aw * 0.55f, cy},
                          {cx + aw * 0.30f, cy - aw},
                          {cx + aw * 0.30f, cy + aw}};
    r.fillPolygon(tri, 3, colors::kPopoutCyan);
  }
}

// Text-sized rounded-rect button (Load? / Activate?), matching the Direct-To
// window's Activate?/Hold? buttons: thin gray outline, pulsing cyan fill when
// armed. Returns the button width.
float drawProcButton(Renderer& r, float leftX, float cy, const char* label,
                     float size, bool armed, bool blinkOn) {
  const float bw = r.measureTextWidth(label, size) + size * 1.3f;
  const float bh = size * 1.7f;
  const Rect b{leftX, cy - bh * 0.5f, bw, bh};
  Point pts[kRoundedRectPoints + 1];
  const int closed = buildRoundedRect(pts, b, bh * 0.32f);
  if (armed && blinkOn) r.fillPolygon(pts, kRoundedRectPoints, colors::kCyan);
  r.strokePolyline(pts, closed, 1.5f,
                   armed ? colors::kWhite : colors::kGroupBoxBorder);
  const Color textColor =
      armed ? (blinkOn ? colors::kBlack : colors::kCyan) : colors::kWhitesmoke;
  r.fillText(leftX + bw * 0.5f, cy, label, size, TextAlign::Center, textColor);
  return bw;
}

// Top-level Procedures menu: Options and Loaded group boxes (trainer MFD PROC
// page), with a footer hint to press PROC again.
void drawProcMenuWindow(Renderer& r, const MfdController& ui, float x, float y,
                        float w, float h, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const int n = ui.procMenuItemCount();
  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float rowH = rowSize * 1.5f;
  const float footSize = P(15.0f);
  const float titleSize = mfdFontPx(kWtBoxTitle, displayH);
  const float pad = P(10.0f);
  const float gap = P(16.0f);

  const float optionsSlotH = titleSize * 0.55f + pad + n * rowH + pad * 0.8f;
  const float loadedSlotH = titleSize * 0.55f + pad + 3.0f * rowH + pad * 0.8f;
  const float boxW = w * 0.40f;
  const float boxH = h - P(4.0f);
  Rect inner = drawProcOverlayPanel(
      r, Rect{x + w - boxW - P(2.0f), y + P(2.0f), boxW, boxH}, displayH);
  float slotY = inner.y;
  Rect options = drawGroupBox(r, Rect{inner.x, slotY, inner.w, optionsSlotH},
                              "Options", displayH, colors::kMfdOverlayGray);
  float fy = options.y;
  for (int i = 0; i < n; ++i) {
    const float cy = fy + rowH * 0.5f;
    const std::string& text = ui.procMenuItemText(i);
    const bool enabled = ui.procMenuItemEnabled(i);
    if (i == ui.procMenuSelected() && enabled) {
      if (ui.blinkOn()) {
        r.fillRect(options.x - pad * 0.4f, cy - rowSize * 0.62f, options.w,
                   rowSize * 1.24f, colors::kPopoutCyan);
        r.fillText(options.x, cy, text, rowSize, TextAlign::Left,
                   colors::kBlack);
      } else {
        r.fillText(options.x, cy, text, rowSize, TextAlign::Left,
                   colors::kPopoutCyan);
      }
    } else {
      r.fillText(options.x, cy, text, rowSize, TextAlign::Left,
                 enabled ? colors::kWhite : colors::kDisabledGray);
    }
    fy += rowH;
  }

  slotY += optionsSlotH + gap;
  Rect loaded = drawGroupBox(r, Rect{inner.x, slotY, inner.w, loadedSlotH},
                             "Loaded", displayH, colors::kMfdOverlayGray);
  struct LoadedRow {
    const char* label;
    std::string value;
  };
  LoadedRow rows[3] = {
      {"Approach:",
       ui.fplHasLoadedApproach() ? ui.fplApproachHeaderLabel() : std::string()},
      {"Arrival:", {}},
      {"Departure:", {}},
  };
  fy = loaded.y;
  for (const LoadedRow& row : rows) {
    const float cy = fy + rowH * 0.5f;
    r.fillText(loaded.x, cy, row.label, rowSize, TextAlign::Left,
               colors::kWhite);
    if (row.value.empty()) {
      drawProcLoadedDashes(r, loaded.x + loaded.w, cy, rowSize);
    } else {
      const float valueW = measureApproachLabelWidth(r, row.value, rowSize);
      drawApproachLabel(r, loaded.x + loaded.w - valueW, cy, row.value, rowSize,
                        colors::kCyan);
    }
    fy += rowH;
  }

  const float footCx = inner.x + inner.w * 0.5f;
  const float footLineH = footSize * 1.4f;
  const float footBottom = inner.y + inner.h;
  r.fillText(footCx, footBottom - footLineH * 1.6f, "Press the \"PROC\" key to",
             footSize, TextAlign::Center, colors::kTitleGray);
  r.fillText(footCx, footBottom - footLineH * 0.5f, "view the previous page",
             footSize, TextAlign::Center, colors::kTitleGray);
}

// Inner approach / transition selection popup over the Select Approach form.
void drawProcSubList(Renderer& r, const MfdController& ui, const Rect& inner,
                     const Rect& anchor, float displayH) {
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const std::vector<std::string> items = ui.procListItems();
  const bool transitionList =
      ui.procStep() == ProcStep::TransitionList;
  const float size = mfdFontPx(kWtRow, displayH);
  const float rowH = P(26.0f);
  const float pad = P(6.0f);
  const int total = static_cast<int>(items.size());

  const float popupX = anchor.x;
  const float popupY = anchor.y + anchor.h;
  // Grow the popup to fill the room beneath the field, keeping the footer
  // buttons clear. If the list still overflows the available rows, a WT scroll
  // bar appears (matching the PFD popout lists).
  const float bottomLimit = inner.y + inner.h - P(44.0f);
  const int fitRows = std::max(
      1, static_cast<int>((bottomLimit - popupY - pad * 2.0f) / rowH));
  const int visible = std::max(1, std::min(fitRows, std::max(1, total)));
  const bool scrolling = total > visible;
  const float scrollReserve = scrolling ? avionics::pfd::wtScrollBarLane(displayH)
                                         : 0.0f;
  const int sel = std::min(std::max(0, ui.procListSelected()), std::max(0, total - 1));
  int first = 0;
  if (total > visible) first = std::max(0, std::min(sel - visible / 2, total - visible));
  const int end = std::min(total, first + visible);

  float maxW = 0.0f;
  for (int i = 0; i < total; ++i) {
    const std::string lbl =
        transitionList ? items[static_cast<std::size_t>(i)]
                       : ui.procApproachDisplayName(i);
    maxW = std::max(maxW, measureApproachLabelWidth(r, lbl, size));
  }
  const float popupW =
      std::min(anchor.w, std::max(P(120.0f), maxW + pad * 2.0f + scrollReserve +
                                                 P(10.0f)));
  const float popupH = rowH * static_cast<float>(visible) + pad * 2.0f;
  const float radius = P(4.0f);
  r.fillRoundedRect(popupX, popupY, popupW, popupH, radius,
                    colors::kPopoutBodyBottom);
  r.strokeRoundedRect(popupX + 0.75f, popupY + 0.75f, popupW - 1.5f,
                      popupH - 1.5f, radius, 1.5f, colors::kPopoutBorder);
  if (total == 0) {
    r.fillText(popupX + popupW * 0.5f, popupY + popupH * 0.5f, "NO PROCEDURES",
               size, TextAlign::Center, colors::kTitleGray);
    return;
  }
  float yy = popupY + pad;
  for (int i = first; i < end; ++i) {
    const float cy = yy + rowH * 0.5f;
    const std::string lbl =
        transitionList ? items[static_cast<std::size_t>(i)]
                       : ui.procApproachDisplayName(i);
    if (i == sel) {
      if (ui.blinkOn()) {
        const float tw = measureApproachLabelWidth(r, lbl, size);
        r.fillRect(popupX + pad - size * 0.08f, cy - size * 0.62f,
                   tw + size * 0.16f, size * 1.24f, colors::kPopoutCyan);
        drawApproachLabel(r, popupX + pad, cy, lbl, size, colors::kBlack);
      } else {
        drawApproachLabel(r, popupX + pad, cy, lbl, size, colors::kPopoutCyan);
      }
    } else {
      drawApproachLabel(r, popupX + pad, cy, lbl, size, colors::kWhite);
    }
    yy += rowH;
  }

  if (scrolling) {
    const float scrollW = avionics::pfd::wtScrollBarLane(displayH);
    avionics::pfd::drawWtScrollBar(r, displayH, popupX + popupW - pad - scrollW,
                                   popupY + pad,
                                   rowH * static_cast<float>(visible), total,
                                   visible, first, 1.0f);
  }
}

void drawProcSequenceRows(Renderer& r, const Rect& area, float displayH,
                          const std::vector<MapLeg>& legs, const MapData& map,
                          int selected, bool focused, bool blinkOn) {
  if (legs.empty()) return;
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float rowSize = mfdFontPx(kWtFieldValue, displayH);
  const float rowH = P(26.0f);
  const int visible = std::max(1, static_cast<int>(area.h / rowH));
  const int total = static_cast<int>(legs.size());
  // Scroll the selected row into view and reserve a lane for the WT scroll bar
  // when the list overflows, matching the approach/transition sub-list popup.
  const bool scrolling = total > visible;
  const float scrollReserve =
      scrolling ? avionics::pfd::wtScrollBarLane(displayH) : 0.0f;
  const int sel = std::min(std::max(0, selected), std::max(0, total - 1));
  int first = 0;
  if (scrolling) {
    first = std::max(0, std::min(sel - visible / 2, total - visible));
  }
  const int end = std::min(total, first + visible);
  const float rowsRight = area.x + area.w - scrollReserve;
  char buf[32];
  for (int i = first; i < end; ++i) {
    const MapLeg& leg = legs[static_cast<std::size_t>(i)];
    const float cy = area.y + rowH * (static_cast<float>(i - first) + 0.5f);
    if (focused && i == sel) {
      drawCursorSelect(r, area.x, cy, leg.id, rowSize, TextAlign::Left, blinkOn);
    } else {
      r.fillText(area.x, cy, leg.id, rowSize, TextAlign::Left,
                 colors::kPopoutCyan);
    }
    // The leg-type role (iaf / faf / mapt / mahp) is white, set apart from the
    // cyan waypoint ident, matching the unit's sequence list.
    if (!leg.procedureRole.empty()) {
      const float roleX =
          area.x + r.measureTextWidth(leg.id, rowSize) + rowSize * 0.35f;
      r.fillText(roleX, cy, leg.procedureRole, rowSize, TextAlign::Left,
                 colors::kWhite);
    }
    // DTK / DIS columns (matching the PFD/FPL style: number with a smaller
    // degree and NM affix). The first leg has no preceding fix, so it shows
    // no track or distance, just like the real unit.
    if (i > 0) {
      const MapLeg& prev = legs[static_cast<std::size_t>(i - 1)];
      const double dtk = navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
      const double dis = navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
      const float colDisR = rowsRight;
      const float disColW = r.measureTextWidth("00.0", rowSize) +
                            r.measureTextWidth("NM", rowSize * kUnitEm) +
                            rowSize * 0.5f;
      const float colDtkR = colDisR - disColW;
      std::snprintf(buf, sizeof(buf), "%03.0f", dtk);
      drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, colors::kWhite);
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, colors::kWhite);
    }
  }

  if (scrolling) {
    const float scrollW = avionics::pfd::wtScrollBarLane(displayH);
    avionics::pfd::drawWtScrollBar(r, displayH, area.x + area.w - scrollW,
                                   area.y, rowH * static_cast<float>(visible),
                                   total, visible, first, 1.0f);
  }
  (void)map;
}

// Select Approach / Approach Loading detail form: stacked group boxes on the grey
// overlay panel (Airport, Approach Channel, Approach, Transition, Minimums,
// Primary Frequency, Sequence) and a centered Activate? button.
void drawProcApproachForm(Renderer& r, const MfdController& ui,
                          const MapData& map, const Rect& inner,
                          float displayH) {
  using Field = ProcApproachField;
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
  const float valueSize = mfdFontPx(kWtFieldValue, displayH);
  const float smallSize = valueSize * kUnitEm;
  const float gap = P(10.0f);
  const bool blinkOn = ui.blinkOn();
  const bool sub = ui.procSubListOpen();
  const bool seqFocused = ui.procSequenceFocused();
  const Color titleBg = colors::kMfdOverlayGray;

  const float buttonsH = P(38.0f);
  const float buttonsY = inner.y + inner.h - buttonsH;
  float slotY = inner.y;

  // drawGroupBox returns a content rect that sits just below the title but is
  // trimmed by its bottom pad, so a single value centered on the content rect
  // reads high. Center single-value rows between the content top and the box's
  // bottom edge so they sit in the optical middle of the area below the title.
  const auto rowCenterY = [](const Rect& slot, const Rect& content) {
    return (content.y + slot.y + slot.h) * 0.5f;
  };

  const float airportH = P(88.0f);
  {
    Rect ac = drawGroupBox(r, Rect{inner.x, slotY, inner.w, airportH},
                           "Airport", displayH, titleBg);
    const float cy = ac.y + ac.h * 0.32f;
    const std::string icao = ui.procAirportIcao();
    const bool airportHi =
        !seqFocused && ui.procApproachField() == Field::Airport;
    if (airportHi && blinkOn && !sub) {
      drawCursorSelect(r, ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                       TextAlign::Left, true);
    } else {
      r.fillText(ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                 TextAlign::Left, colors::kPopoutCyan);
    }
    const MapFeature sym = ui.procAirportFeature();
    if (!icao.empty()) {
      const float iconX =
          ac.x + r.measureTextWidth(icao, valueSize) + P(16.0f);
      drawWaypointIcon(r, iconX, cy, P(22.0f), &sym, sym.type);
      const char* usage = airportUsageType(sym);
      if (usage != nullptr) {
        r.fillText(ac.x + ac.w, cy, usage, labelSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
    }
    const std::string city = ui.procAirportCityLine();
    if (!city.empty()) {
      const float nameCy = cy + valueSize * 1.25f;
      r.fillText(ac.x, nameCy, city, labelSize, TextAlign::Left,
                 colors::kPopoutCyan);
    }
  }
  slotY += airportH + gap;

  const float channelH = P(48.0f);
  {
    const Rect channelSlot{inner.x, slotY, inner.w, channelH};
    Rect ch = drawGroupBox(r, channelSlot, "Approach Channel", displayH,
                           titleBg);
    const float cy = rowCenterY(channelSlot, ch);
    const char* channelLabel = "Channel";
    const float channelLabelW = r.measureTextWidth(channelLabel, labelSize);
    r.fillText(ch.x, cy, channelLabel, labelSize, TextAlign::Left,
               colors::kWhite);
    drawTightDashRun(r, ch.x + channelLabelW + P(6.0f), cy, 5, valueSize,
                     colors::kPopoutCyan);
    const char* idLabel = "ID";
    const float idLabelW = r.measureTextWidth(idLabel, labelSize);
    const float idX = ch.x + ch.w * 0.55f;
    r.fillText(idX, cy, idLabel, labelSize, TextAlign::Left, colors::kWhite);
    if (!seqFocused && ui.procApproachField() == Field::Id && blinkOn && !sub) {
      const float barW = valueSize * 0.14f;
      r.fillRect(idX - barW * 1.6f, cy - valueSize * 0.55f, barW,
                 valueSize * 1.1f, colors::kPopoutCyan);
    }
    drawRightTightDashRun(r, ch.x + ch.w, cy, 5, valueSize, colors::kWhite);
    (void)idLabelW;
  }
  slotY += channelH + gap;

  const float fieldH = P(44.0f);
  const auto drawFieldBox = [&](const Rect& slot, const char* title,
                                const std::string& value, Field field) {
    Rect box = drawGroupBox(r, slot, title, displayH, titleBg);
    const float cy = rowCenterY(slot, box);
    if (value.empty()) {
      drawTightDashRun(r, box.x, cy, 8, valueSize, colors::kPopoutCyan);
      return;
    }
    if (!seqFocused && ui.procApproachField() == field && blinkOn && !sub) {
      drawCursorSelect(r, box.x, cy, value, valueSize, TextAlign::Left, true);
    } else {
      drawApproachLabel(r, box.x, cy, value, valueSize, colors::kPopoutCyan);
    }
  };

  const Rect aprSlot{inner.x, slotY, inner.w, fieldH};
  drawFieldBox(aprSlot, "Approach", ui.procSelectedApproachDisplay(),
               Field::Apr);
  slotY += fieldH + gap;
  const Rect transSlot{inner.x, slotY, inner.w, fieldH};
  drawFieldBox(transSlot, "Transition", ui.procSelectedTransitionDisplay(),
               Field::Trans);
  slotY += fieldH + gap;

  const float minsH = P(86.0f);
  {
    const Rect minsSlot{inner.x, slotY, inner.w, minsH};
    Rect mc = drawGroupBox(r, minsSlot, "Minimums", displayH, titleBg);
    const float minsMidY = rowCenterY(minsSlot, mc);
    const float minsRowHalf = valueSize * 0.95f;
    const float cy1 = minsMidY - minsRowHalf;
    const float cy2 = minsMidY + minsRowHalf;
    const bool baro = ui.minimumsBaroOn();
    const char* minsLabel = baro ? "BARO" : "OFF";
    const float toggleW = r.measureTextWidth("BARO", labelSize);
    const float centerX = mc.x + toggleW * 0.75f;
    const bool minsHi = !seqFocused && ui.procApproachField() == Field::Mins;
    if (minsHi && blinkOn && !sub) {
      r.fillRect(centerX - toggleW * 0.5f - labelSize * 0.15f,
                 cy1 - labelSize * 0.62f, toggleW + labelSize * 0.3f,
                 labelSize * 1.24f, colors::kPopoutCyan);
      r.fillText(centerX, cy1, minsLabel, labelSize, TextAlign::Center,
                 colors::kBlack);
    } else {
      r.fillText(centerX, cy1, minsLabel, labelSize, TextAlign::Center,
                 colors::kPopoutCyan);
    }
    drawProcCarrot(r, centerX - toggleW * 0.5f - labelSize * 0.45f, cy1,
                   labelSize, false, baro);
    drawProcCarrot(r, centerX + toggleW * 0.5f + labelSize * 0.45f, cy1,
                   labelSize, true, !baro);
    const float ftW = r.measureTextWidth("FT", smallSize);
    if (baro) {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%.0f", ui.minimumsAltitudeFt());
      const bool altHi = !seqFocused && ui.procApproachField() == Field::MinsAlt;
      const float altR = mc.x + mc.w - ftW - P(2.0f);
      if (altHi && blinkOn && !sub) {
        drawCursorSelect(r, altR, cy1, buf, valueSize, TextAlign::Right, true);
      } else {
        r.fillText(altR, cy1, buf, valueSize, TextAlign::Right,
                   colors::kPopoutCyan);
      }
      r.fillText(mc.x + mc.w, cy1, "FT", smallSize, TextAlign::Right,
                 colors::kWhitesmoke);
    } else {
      drawRightTightDashRun(r, mc.x + mc.w - ftW, cy1, 4, labelSize,
                            colors::kWhite);
      r.fillText(mc.x + mc.w, cy1, "FT", smallSize, TextAlign::Right,
                 colors::kWhitesmoke);
    }

    std::string tempLabel = "TEMP At ";
    tempLabel += ui.procAirportIcao();
    r.fillText(mc.x, cy2, tempLabel, labelSize, TextAlign::Left,
               colors::kWhite);
    char tbuf[16];
    std::snprintf(tbuf, sizeof(tbuf), "%d",
                  static_cast<int>(std::lround(ui.minimumsTempC() * 9.0f / 5.0f +
                                               32.0f)));
    const float tempX = mc.x + r.measureTextWidth(tempLabel, labelSize) +
                        P(8.0f);
    r.fillText(tempX, cy2, tbuf, valueSize, TextAlign::Left,
               colors::kPopoutCyan);
    drawRightTightDashRun(r, mc.x + mc.w - ftW, cy2, 4, labelSize,
                          colors::kWhite);
    r.fillText(mc.x + mc.w, cy2, "FT", smallSize, TextAlign::Right,
               colors::kWhitesmoke);
  }
  slotY += minsH + gap;

  const float primH = P(44.0f);
  {
    const Rect primSlot{inner.x, slotY, inner.w, primH};
    Rect fc = drawGroupBox(r, primSlot, "Primary Frequency", displayH, titleBg);
    const float cy = rowCenterY(primSlot, fc);
    if (ui.procShowsPrimaryNavFreq() && ui.procPrimaryFreqMhz() > 0.0f) {
      char buf[16];
      if (ui.procPrimaryNavIsNdb()) {
        std::snprintf(buf, sizeof(buf), "%.1f", ui.procPrimaryFreqMhz());
      } else {
        std::snprintf(buf, sizeof(buf), "%.2f", ui.procPrimaryFreqMhz());
      }
      const std::string ident = ui.procPrimaryIdent();
      if (!ident.empty()) {
        r.fillText(fc.x, cy, ident, valueSize, TextAlign::Left,
                   colors::kPopoutCyan);
      } else {
        drawTightDashRun(r, fc.x, cy, 5, labelSize, colors::kWhite);
      }
      const float pillW =
          r.measureTextWidth(buf, valueSize) + valueSize * 1.1f;
      const Rect pill{fc.x + fc.w - pillW, cy - valueSize * 0.72f, pillW,
                      valueSize * 1.44f};
      drawPill(r, pill, buf, valueSize, colors::kPopoutCyan);
    } else {
      drawTightDashRun(r, fc.x, cy, 5, labelSize, colors::kWhite);
    }
  }
  slotY += primH + gap;

  const float seqH = buttonsY - gap - slotY;
  {
    Rect sc = drawGroupBox(r, Rect{inner.x, slotY, inner.w, seqH}, "Sequence",
                           displayH, titleBg);
    drawProcSequenceRows(r, sc, displayH, ui.procPreviewLegs(), map,
                         ui.procSequenceSelected(), ui.procSequenceFocused(),
                         blinkOn);
  }

  // Footer: "Load?  or  Activate?" (trainer Approach Loading), each a
  // text-sized rounded-rect button with the lowercase "or" between them.
  const float btnCy = buttonsY + buttonsH * 0.5f;
  const float loadW =
      r.measureTextWidth("Load?", valueSize) + valueSize * 1.3f;
  const float actW =
      r.measureTextWidth("Activate?", valueSize) + valueSize * 1.3f;
  const float orW = r.measureTextWidth("or", valueSize);
  const float orPad = valueSize * 0.6f;
  const float groupW = loadW + orPad + orW + orPad + actW;
  float btnX = inner.x + (inner.w - groupW) * 0.5f;
  drawProcButton(r, btnX, btnCy, "Load?", valueSize,
                 !seqFocused && ui.procApproachField() == Field::Load &&
                     ui.procLoadArmed(),
                 blinkOn);
  btnX += loadW + orPad;
  r.fillText(btnX + orW * 0.5f, btnCy, "or", valueSize, TextAlign::Center,
             colors::kWhitesmoke);
  btnX += orW + orPad;
  drawProcButton(r, btnX, btnCy, "Activate?", valueSize,
                 !seqFocused && ui.procApproachField() == Field::Activate &&
                     ui.procActivateArmed(),
                 blinkOn);

  if (sub) {
    const Rect& anchor =
        ui.procStep() == ProcStep::TransitionList ? transSlot : aprSlot;
    drawProcSubList(r, ui, inner, anchor, displayH);
  }
}

// Select Departure / Arrival sub-window: airport header, then the procedure
// (or transition) list.
void drawProcDepArrList(Renderer& r, const MfdController& ui, const Rect& inner,
                        float displayH) {
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float valueSize = mfdFontPx(kWtFieldValue, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float rowH = P(28.0f);
  const float left = inner.x;
  const float right = inner.x + inner.w;
  float yy = inner.y;
  const std::string icao = ui.procAirportIcao();
  r.fillText(left, yy + rowH * 0.5f, icao.empty() ? "_____" : icao, valueSize,
             TextAlign::Left, colors::kCyan);
  if (ui.procStep() == ProcStep::TransitionList) {
    r.fillText(right, yy + rowH * 0.5f, ui.procSelectedName(), valueSize,
               TextAlign::Right, colors::kCyan);
  }
  yy += rowH * 0.9f;
  r.strokeLine(left, yy, right, yy, 1.0f, colors::kMenuBorderGray);
  yy += rowH * 0.3f;

  const std::vector<std::string> items = ui.procListItems();
  if (items.empty()) {
    r.fillText(left + inner.w * 0.5f, yy + rowH * 0.5f, "NO PROCEDURES",
               rowSize, TextAlign::Center, colors::kTitleGray);
    return;
  }
  const int total = static_cast<int>(items.size());
  const int sel = std::min(std::max(0, ui.procListSelected()), total - 1);
  const int visible =
      std::max(1, static_cast<int>((inner.y + inner.h - yy) / rowH));
  int first = 0;
  if (total > visible) first = std::max(0, std::min(sel - visible / 2, total - visible));
  const int end = std::min(total, first + visible);
  for (int i = first; i < end; ++i) {
    const float cy = yy + rowH * 0.5f;
    const std::string& lbl = items[static_cast<std::size_t>(i)];
    if (i == sel) {
      drawCursorSelect(r, left, cy, lbl, rowSize, TextAlign::Left, ui.blinkOn());
    } else {
      r.fillText(left, cy, lbl, rowSize, TextAlign::Left, colors::kCyan);
    }
    yy += rowH;
  }
}

// Selection sub-window frame (Select Approach / Arrival / Departure): a tall
// right-side popup like the Direct-To window.
void drawProcSelectWindow(Renderer& r, const MfdController& ui,
                          const MapData& map, float x, float y, float w,
                          float h, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float boxW = w * 0.40f;
  const float boxH = h - P(4.0f);
  Rect inner = drawProcOverlayPanel(
      r, Rect{x + w - boxW - P(2.0f), y + P(2.0f), boxW, boxH}, displayH);
  if (ui.procCategory() == ProcedureType::Approach) {
    drawProcApproachForm(r, ui, map, inner, displayH);
  } else {
    drawProcDepArrList(r, ui, inner, displayH);
  }
}

}  // namespace

void drawProcWindow(Renderer& r, const FlightData& d, const MapData& map,
                    const MfdController& ui, float x, float y, float w, float h,
                    float displayH) {
  (void)d;
  if (ui.procSelectMode()) {
    drawProcSelectWindow(r, ui, map, x, y, w, h, displayH);
  } else {
    drawProcMenuWindow(r, ui, x, y, w, h, displayH);
  }
}

void drawActiveFlightPlanPage(Renderer& r, const FlightData& d,
                              const MapData& map, MfdController& ui,
                              float x, float y, float w, float h,
                              float displayH) {
  // FPL Active Flight Plan (WT MFDFPLPage): map on the LEFT, the wide gray
  // panel on the right with the Active Flight Plan box (origin/destination
  // header, DTK/DIS/ALT columns, magenta active leg) over the Active VNV
  // Profile box and the FPL-key prompt. The FMS knob edits the list: push for
  // the cursor, large knob selects a row, small knob opens waypoint entry,
  // CLR removes the selected waypoint.
  PageFrame f = beginPanelPage(r, x, y, w, h, true);

  // The controller's copy of the plan shows pending edits immediately (the
  // data sources echo them a frame later).
  const std::vector<MapLeg>& plan = ui.fplLegs();
  const std::string activeToIdent = fplActiveToIdent(d, map, plan);

  // Route preview map around ownship, wide enough to show the plan. The RANGE
  // knob zooms this inset (Pilot's Guide); auto-frame the route until the pilot
  // turns the knob, then honor the manual ladder step.
  float previewRangeNm = 25.0f;
  if (!ui.fplPreviewRangeManual()) {
    if (map.positionValid && plan.size() >= 2) {
      double maxNm = 0.0;
      for (const MapLeg& leg : plan) {
        maxNm = std::max(maxNm, navDistanceNm(map.ownshipLat, map.ownshipLon,
                                              leg.lat, leg.lon));
      }
      previewRangeNm =
          std::max(10.0f, std::min(150.0f, static_cast<float>(maxNm) * 1.2f));
    }
    ui.setFplPreviewFitRange(mapRangeIndexForNm(previewRangeNm));
  }
  previewRangeNm = ui.rangeNm();
  const float previewDisplayRangeNm = ui.displayRangeNm();
  const std::vector<MapLeg> procPreview =
      ui.procMenuOpen() ? ui.procPreviewLegs() : std::vector<MapLeg>{};
  const std::vector<MapLeg>* procPreviewPtr =
      procPreview.empty() ? nullptr : &procPreview;
  drawPageMap(r, d, map, f.map, previewRangeNm, nullptr, displayH, false,
              procPreviewPtr, previewDisplayRangeNm);

  PanelStack stack(f.panel, displayH);
  const float promptWt = 34.0f;
  const float vnvWt = 124.0f;
  const float wxWt = 90.0f;

  // Active Flight Plan box.
  {
    Rect inner = drawGroupBox(
        r, stack.slot(stack.remainingWt(promptWt + vnvWt + wxWt + 20.0f)),
        "Active Flight Plan", displayH);

    const float headerSize = mfdFontPx(kWtRow, displayH);
    // Resolve approach grouping the same way as the PFD Active Flight Plan
    // window: stored start/count when valid, otherwise infer from procedureRole.
    int approachStart = ui.fplApproachLegStart();
    int approachCount = ui.fplApproachLegCount();
    if (!ui.fplHasLoadedApproach() || approachCount <= 0) {
      const InferredProcedureBlock block = inferProcedureBlockInPlan(plan);
      if (block.valid()) {
        approachStart = block.start;
        approachCount = block.count;
      }
    } else {
      FlightPlanApproachState stored;
      stored.legStart = approachStart;
      stored.legCount = approachCount;
      if (!approachStateFitsPlan(stored, plan)) {
        const InferredProcedureBlock block = inferProcedureBlockInPlan(plan);
        if (block.valid()) {
          approachStart = block.start;
          approachCount = block.count;
        } else {
          approachStart = 0;
          approachCount = 0;
        }
      }
    }
    if (approachCount > 0 && approachStart >= 0 &&
        approachStart + approachCount < static_cast<int>(plan.size())) {
      approachCount = fplNormalizedApproachCount(
          approachStart, approachCount, static_cast<int>(plan.size()));
    }
    const bool approachLoaded = approachCount > 0;
    std::string approachAirport;
    if (approachLoaded) {
      approachAirport = ui.fplApproachAirportIcao();
      if (approachAirport.empty() && approachStart > 0 &&
          approachStart <= static_cast<int>(plan.size())) {
        approachAirport = plan[static_cast<std::size_t>(approachStart - 1)].id;
      }
    }
    // Direct-To: copy the PFD Navigation Status Box (D→ + target, magenta); the
    // section template below stays blank until the pilot builds a flight plan.
    const bool directToFplView =
        navDirectToActive(d) && !ui.fplLocalDraft();
    const bool destOnlyPlan =
        ui.fplDestinationFilled() && plan.size() == 1;
    const float colDtkR = inner.x + mfdFontPx(210.0f, displayH);
    const float colDisR = inner.x + mfdFontPx(330.0f, displayH);

    std::string orig;
    std::string dest;
    bool blankOrig = false;
    bool blankDest = false;
    if (approachLoaded && plan.empty() && !approachAirport.empty()) {
      blankOrig = true;
      dest = approachAirport;
    } else if (!directToFplView) {
      const bool blankOriginHeader =
          approachLoaded && approachStart <= 1 && !plan.empty() &&
          !approachAirport.empty() && plan.front().id == approachAirport;
      orig = fplHeaderOriginIdent(plan, approachStart, approachCount,
                                  approachLoaded, blankOriginHeader,
                                  destOnlyPlan, activeToIdent);
      blankOrig = orig.empty();
      dest = fplHeaderDestinationIdent(plan, ui.fplDestinationFilled(), approachStart,
                                       approachLoaded, approachAirport);
      blankDest = dest.empty();
    }
    float fy = inner.y;
    const float headerX = inner.x + mfdFontPx(40.0f, displayH);
    const float headerBandH = headerSize * 1.5f;
    const float headerCy =
        directToFplView ? inner.y + headerBandH * 0.5f
                        : fy + headerSize * 0.6f;
    if (directToFplView) {
      const std::string dest = d.fmaToWpt;
      pfd::drawNavDirectToHeader(r, headerX, headerCy, dest, headerSize,
                                 colors::kMagenta);
    } else if (approachLoaded && plan.empty() && !approachAirport.empty()) {
      drawFplHeaderOrigDest(r, headerX, headerCy, std::string(), dest, true, false,
                            headerSize, colors::kCyan);
    } else {
      drawFplHeaderOrigDest(r, headerX, headerCy, orig, dest, blankOrig, blankDest,
                            headerSize, colors::kCyan);
    }
    fy += headerBandH;

    // Column headers + the rule under them (WT .mfd-flightplan-hr).
    const float colHdrSize = mfdFontPx(kWtHeader, displayH);
    const float colAltR = inner.x + inner.w - mfdFontPx(5.0f, displayH);
    if (!directToFplView) {
      r.fillText(inner.x + mfdFontPx(170.0f, displayH), fy + colHdrSize * 0.6f,
                 "DTK", colHdrSize, TextAlign::Left, colors::kWhite);
      r.fillText(inner.x + mfdFontPx(250.0f, displayH), fy + colHdrSize * 0.6f,
                 "DIS", colHdrSize, TextAlign::Left, colors::kWhite);
      r.fillText(inner.x + mfdFontPx(320.0f, displayH), fy + colHdrSize * 0.6f,
                 "ALT", colHdrSize, TextAlign::Left, colors::kWhite);
      fy += colHdrSize * 1.4f;
    }
    r.strokeLine(inner.x, fy, inner.x + inner.w, fy, 1.0f,
                 Color{0.596f, 0.624f, 0.682f, 1.0f});
    fy += mfdFontPx(4.0f, displayH);

    const float rowH = mfdFontPx(32.0f, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    const float labelX = inner.x + mfdFontPx(25.0f, displayH);
    const float filledIdentX =
        labelX + mfdFontPx(kFplSectionIdentIndentPx, displayH);

    const bool cursorOn = ui.fplCursorOn();
    const bool blinkOn = ui.blinkOn();
    const int listCursorRow = ui.fplCursorRow();
    const int wptCount = static_cast<int>(plan.size());
    const bool destFilled =
        ui.fplDestinationFilled() || approachStart >= 2;
    const bool directToPlanBody = directToFplView;
    const bool blankOriginSection =
        directToFplView || destOnlyPlan ||
        (approachLoaded && approachStart <= 1 && !plan.empty() &&
         !approachAirport.empty() && plan.front().id == approachAirport);
    const int bodyLegCount =
        directToFplView && !approachLoaded ? 0 : wptCount;
    int activeLegIdx = fplDirectToTargetLegIndex(
        plan, d, activeToIdent,
        directToFplView || map.directToActive);
    if (activeLegIdx < 0) {
      activeLegIdx = fplResolvedActiveLegIndex(plan, d, activeToIdent);
    }
    const int sectionLegCount =
        approachLoaded
            ? (blankOriginSection ? 0
                                  : fplEnrouteDisplayLegCount(plan, approachStart))
            : wptCount;
    const int fplActiveLayoutLegCount =
        approachLoaded ? sectionLegCount : bodyLegCount;
    const bool fplActiveLayoutDestFilled =
        approachLoaded
            ? (ui.fplDestinationFilled() || approachStart >= 2)
            : ui.fplDestinationFilled();
    const std::vector<FplDisplayRow> approachDisplayRows =
        approachLoaded
            ? buildFplApproachDisplayRows(
                  plan, approachStart, approachCount, blankOriginSection,
                  destFilled)
            : std::vector<FplDisplayRow>{};
    const std::vector<FplSectionRow> sectionRows =
        approachLoaded ? std::vector<FplSectionRow>{}
                       : fplFilterDuplicateLegSectionRows(
                             buildFplSectionRows(bodyLegCount,
                                                 ui.fplDestinationFilled(),
                                                 directToPlanBody),
                             plan);
    const int displayRowCount =
        approachLoaded ? static_cast<int>(approachDisplayRows.size())
                       : static_cast<int>(sectionRows.size());
    const int totalRows = displayRowCount;

    const int activeSelectableRow =
        approachLoaded
            ? fplApproachSelectableRowForLegIndex(
                  activeLegIdx, plan, approachStart, approachCount,
                  blankOriginSection, destFilled)
            : fplSectionSelectableRowForLegIndex(
                  activeLegIdx, sectionRows, bodyLegCount,
                  ui.fplDestinationFilled(), directToPlanBody);
    const bool pinActiveApproachLeg = fplPinActiveApproachLeg(
        approachLoaded, activeLegIdx, approachStart, approachCount,
        ui.fplLocalDraft(),
        directToFplView || map.directToActive);
    int cursorLegIdx = -1;
    if (approachLoaded) {
      cursorLegIdx = fplApproachLegIndexForSelectable(
          listCursorRow, plan, approachStart, approachCount, blankOriginSection,
          destFilled);
    } else {
      cursorLegIdx = fplLegIndexForSectionRow(
          listCursorRow, bodyLegCount, ui.fplDestinationFilled(),
          directToPlanBody);
    }
    const bool activeHighlight =
        ((directToFplView || map.directToActive) && activeLegIdx >= 0) ||
        fplShowActiveLegHighlight(activeLegIdx, cursorLegIdx, activeSelectableRow,
                                  listCursorRow, pinActiveApproachLeg);
    const std::string& navToIdent = d.fmaToWpt;

    if (totalRows == 0) {
      r.fillText(inner.x + inner.w * 0.5f, fy + rowH,
                 "NO ACTIVE FLIGHT PLAN", rowSize, TextAlign::Center,
                 colors::kTitleGray);
    }

    int scrollAnchor = 0;
    if (approachLoaded) {
      scrollAnchor = fplApproachDisplayRowIndexForSelectable(
          listCursorRow, plan, approachStart, approachCount,
          blankOriginSection, destFilled);
    } else if (!sectionRows.empty()) {
      scrollAnchor = sectionDisplayRowForSelectable(
          listCursorRow, sectionRows, bodyLegCount,
          ui.fplDestinationFilled());
    }

    const int maxRows =
        std::max(1, static_cast<int>((inner.y + inner.h - fy) / rowH));
    int start = 0;
    if (totalRows > maxRows) {
      int pinnedRow = -1;
      if (pinActiveApproachLeg && approachLoaded) {
        pinnedRow = fplApproachDisplayRowIndexForLegIndex(
            activeLegIdx, plan, approachStart, approachCount,
            blankOriginSection, destFilled);
      }
      start = fplListScrollFirst(scrollAnchor, pinnedRow, totalRows, maxRows);
    }

    const std::string approachTransition = ui.fplApproachTransition();
    const int end = std::min(totalRows, start + maxRows);
    int selectableIdx = 0;
    if (start > 0) {
      if (approachLoaded) {
        for (int i = 0; i < start; ++i) {
          if (fplApproachDisplayRowSelectable(
                  approachDisplayRows[static_cast<std::size_t>(i)].kind)) {
            ++selectableIdx;
          }
        }
      } else {
        countSectionSelectablesBefore(start, sectionRows, bodyLegCount,
                                      ui.fplDestinationFilled(), selectableIdx);
      }
    }
    for (int row = start; row < end; ++row) {
      const float cy = fy + rowH * 0.5f;

      if (approachLoaded) {
        const FplDisplayRow& dr =
            approachDisplayRows[static_cast<std::size_t>(row)];
        const bool showSelection = fplShowListRowSelection(
            selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
        switch (dr.kind) {
          case FplDisplayRowKind::SepDash:
            ++selectableIdx;
            drawFplDashRow(r, labelX, cy, kFplApproachSepDashCount, rowSize,
                           colors::kPopoutCyan, showSelection, blinkOn);
            fy += rowH;
            continue;
          case FplDisplayRowKind::ApproachHeader:
            drawFplApproachHeader(r, labelX, cy, approachAirport,
                                  ui.fplApproachHeaderLabel(), rowSize,
                                  colors::kCyan);
            fy += rowH;
            continue;
          case FplDisplayRowKind::EnrouteLabel:
            r.fillText(labelX, cy, "Enroute", rowSize, TextAlign::Left,
                       colors::kCyan);
            fy += rowH;
            continue;
          case FplDisplayRowKind::Origin: {
            ++selectableIdx;
            if (dr.legIndex >= 0) {
              const bool showActive = fplShowActiveNavRow(
                  activeHighlight, dr.legIndex, activeLegIdx,
                  plan[static_cast<std::size_t>(dr.legIndex)], navToIdent);
              const bool activeNavBlink =
                  showActive &&
                  fplActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
                                       cursorOn, listCursorRow,
                                       activeSelectableRow);
              drawFplLegRow(r, d, map, plan, dr.legIndex, activeToIdent, activeLegIdx,
                            inner.x, filledIdentX, colDtkR, colDisR, colAltR,
                            cy, rowSize, rowH, displayH, ui, showSelection,
                            showActive, activeNavBlink, false, approachTransition);
            } else {
              drawFplSectionIdent(r, labelX, cy, "Origin - ", std::string(),
                                  true, showSelection, blinkOn, rowSize,
                                  colors::kPopoutCyan);
            }
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::OriginBlank: {
            ++selectableIdx;
            drawFplDashRow(r, labelX, cy, kFplDashCount, rowSize,
                           colors::kPopoutCyan, showSelection, blinkOn);
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::Destination: {
            ++selectableIdx;
            if (dr.legIndex >= 0) {
              const bool showActive = fplShowActiveNavRow(
                  activeHighlight, dr.legIndex, activeLegIdx,
                  plan[static_cast<std::size_t>(dr.legIndex)], navToIdent);
              const bool activeNavBlink =
                  showActive &&
                  fplActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
                                       cursorOn, listCursorRow,
                                       activeSelectableRow);
              drawFplLegRow(r, d, map, plan, dr.legIndex, activeToIdent, activeLegIdx,
                            inner.x, filledIdentX, colDtkR, colDisR, colAltR,
                            cy, rowSize, rowH, displayH, ui, showSelection,
                            showActive, activeNavBlink, false, approachTransition);
            } else {
              drawFplSectionIdent(r, labelX, cy, "Destination - ",
                                  std::string(), true, showSelection, blinkOn,
                                  rowSize, colors::kPopoutCyan);
            }
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::EnrouteBlank: {
            ++selectableIdx;
            drawFplDashRow(r, labelX, cy, kFplDashCount, rowSize,
                           colors::kTitleGray, showSelection, blinkOn);
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::EnrouteLeg:
          case FplDisplayRowKind::ApproachLeg: {
            ++selectableIdx;
            const bool showActive = fplShowActiveNavRow(
                activeHighlight, dr.legIndex, activeLegIdx,
                plan[static_cast<std::size_t>(dr.legIndex)], navToIdent);
            const bool activeNavBlink =
                showActive &&
                fplActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
                                     cursorOn, listCursorRow,
                                     activeSelectableRow);
            drawFplLegRow(r, d, map, plan, dr.legIndex, activeToIdent, activeLegIdx, inner.x,
                          filledIdentX, colDtkR, colDisR, colAltR, cy, rowSize,
                          rowH, displayH, ui, showSelection, showActive, activeNavBlink,
                          dr.kind == FplDisplayRowKind::ApproachLeg,
                          approachTransition);
            fy += rowH;
            continue;
          }
          default:
            break;
        }
        continue;
      }

      const FplSectionRow& sr = sectionRows[static_cast<std::size_t>(row)];
      if (sr.kind == FplSectionRow::Kind::EnrouteLabel) {
        r.fillText(labelX, cy, "Enroute", rowSize, TextAlign::Left,
                   colors::kCyan);
        fy += rowH;
        continue;
      }
      if (sr.kind == FplSectionRow::Kind::DestinationLabel) {
        drawFplDestinationLabelRow(r, labelX, cy, rowSize);
        fy += rowH;
        continue;
      }
      if (sr.kind == FplSectionRow::Kind::Origin && sr.legIndex < 0 &&
          !sectionRowIsSelectable(sr, bodyLegCount, ui.fplDestinationFilled())) {
        drawFplSectionIdent(r, labelX, cy, "Origin - ", std::string(), true,
                            false, false, rowSize, colors::kPopoutCyan);
        fy += rowH;
        continue;
      }
      if (sr.kind == FplSectionRow::Kind::OriginBlank ||
          sr.kind == FplSectionRow::Kind::DestinationBlank) {
        drawFplDashRow(r, labelX, cy, kFplDashCount, rowSize,
                       colors::kPopoutCyan, false, false);
        fy += rowH;
        continue;
      }

      if (!sectionRowIsSelectable(sr, bodyLegCount, ui.fplDestinationFilled())) {
        fy += rowH;
        continue;
      }

      const bool showSelection = fplShowListRowSelection(
          selectableIdx, listCursorRow, activeSelectableRow, cursorOn);
      switch (sr.kind) {
        case FplSectionRow::Kind::Origin: {
          if (sr.legIndex >= 0) {
            const bool showActive = fplShowActiveNavRow(
                activeHighlight, sr.legIndex, activeLegIdx,
                plan[static_cast<std::size_t>(sr.legIndex)], navToIdent);
            const bool activeNavBlink =
                showActive &&
                fplActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
                                     cursorOn, listCursorRow,
                                     activeSelectableRow);
            drawFplLegRow(r, d, map, plan, sr.legIndex, activeToIdent, activeLegIdx, inner.x,
                          filledIdentX, colDtkR, colDisR, colAltR, cy, rowSize,
                          rowH, displayH, ui, showSelection, showActive, activeNavBlink,
                          false, approachTransition);
          } else {
            drawFplSectionIdent(r, labelX, cy, "Origin - ", std::string(),
                                true, showSelection, blinkOn, rowSize,
                                colors::kPopoutCyan);
          }
          break;
        }
        case FplSectionRow::Kind::OriginBlank:
          break;
        case FplSectionRow::Kind::EnrouteBlank: {
          const bool active = fplSectionRowIsActiveDisplay(
              sr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled,
              activeLegIdx);
          const bool showActive = active && activeHighlight;
          drawFplDashRow(
              r, labelX, cy, kFplDashCount, rowSize,
              showActive ? colors::kMagenta : colors::kPopoutCyan,
              showSelection, blinkOn);
          if (showActive) {
            char buf[24];
            std::snprintf(buf, sizeof(buf), "%03.0f",
                          static_cast<double>(d.fmaLegBearingDeg));
            drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize,
                              colors::kMagenta);
            std::snprintf(buf, sizeof(buf), "%.1f",
                          static_cast<double>(d.fmaLegDistanceNm));
            drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize,
                              colors::kMagenta);
          }
          break;
        }
        case FplSectionRow::Kind::EnrouteLeg: {
          const bool showActive = fplShowActiveNavRow(
              activeHighlight, sr.legIndex, activeLegIdx,
              plan[static_cast<std::size_t>(sr.legIndex)], navToIdent);
          const bool activeNavBlink =
              showActive &&
              fplActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
                                   cursorOn, listCursorRow, activeSelectableRow);
          drawFplLegRow(r, d, map, plan, sr.legIndex, activeToIdent, activeLegIdx, inner.x,
                        filledIdentX, colDtkR, colDisR, colAltR, cy, rowSize,
                        rowH, displayH, ui, showSelection, showActive, activeNavBlink,
                        false, approachTransition);
          break;
        }
        case FplSectionRow::Kind::Destination:
          if (sr.legIndex >= 0) {
            const bool showActive = fplShowActiveNavRow(
                activeHighlight, sr.legIndex, activeLegIdx,
                plan[static_cast<std::size_t>(sr.legIndex)], navToIdent);
            const bool activeNavBlink =
                showActive &&
                fplActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
                                     cursorOn, listCursorRow, activeSelectableRow);
            drawFplLegRow(r, d, map, plan, sr.legIndex, activeToIdent, activeLegIdx, inner.x,
                          filledIdentX, colDtkR, colDisR, colAltR, cy, rowSize,
                          rowH, displayH, ui, showSelection, showActive, activeNavBlink,
                          false, approachTransition);
          } else {
            drawFplSectionIdent(r, labelX, cy, "Destination - ", std::string(),
                                true, showSelection, blinkOn, rowSize,
                                colors::kPopoutCyan);
          }
          break;
        case FplSectionRow::Kind::DestinationBlank:
          break;
        default:
          break;
      }
      ++selectableIdx;
      fy += rowH;
    }
  }

  // Active VNV Profile: target waypoint/altitude, target & required vertical
  // speed, time to top of descent, flight-path angle, and path deviation
  // (Pilot's Guide, Section 6 "Vertical Navigation").
  {
    Rect inner = drawGroupBox(r, stack.slot(vnvWt), "Active VNV Profile",
                              displayH);
    const float rowH = inner.h / 3.0f;
    const Rect left{inner.x, inner.y, inner.w * 0.52f, inner.h};
    const Rect right{inner.x + inner.w * 0.56f, inner.y, inner.w * 0.44f,
                     inner.h};
    const VnvProfile& vnv = d.vnv;
    char vb[24];

    std::string wptVal = "_ _ _ _ _ _";
    if (vnv.active && !vnv.targetWpt.empty()) {
      std::snprintf(vb, sizeof(vb), "%s %d" "FT", vnv.targetWpt.c_str(),
                    vnv.targetAltFt);
      wptVal = vb;
    }
    std::string vsTgt = "_____FPM";
    std::string vsReq = "_____FPM";
    if (vnv.active) {
      std::snprintf(vb, sizeof(vb), "%+dFPM",
                    static_cast<int>(std::lround(vnv.vsTargetFpm)));
      vsTgt = vb;
      std::snprintf(vb, sizeof(vb), "%+dFPM",
                    static_cast<int>(std::lround(vnv.vsRequiredFpm)));
      vsReq = vb;
    }
    std::string tod = kDashTime;
    if (vnv.active) {
      const int secs = std::max(0, vnv.timeToTodSec);
      std::snprintf(vb, sizeof(vb), "%02d:%02d", (secs / 60) % 100, secs % 60);
      tod = vb;
    }
    std::string fpa = "_ _ _ _\xC2\xB0";
    if (vnv.active) {
      std::snprintf(vb, sizeof(vb), "%.1f\xC2\xB0",
                    static_cast<double>(-std::fabs(vnv.fpaDeg)));
      fpa = vb;
    }
    std::string vdev = "_____FT";
    if (vnv.active && vnv.capturing) {
      std::snprintf(vb, sizeof(vb), "%+dFT",
                    static_cast<int>(std::lround(vnv.verticalDeviationFt)));
      vdev = vb;
    }

    float ly = left.y;
    ly = drawField(r, left, ly, rowH, "WPT", wptVal, displayH,
                   vnv.active ? colors::kCyan : colors::kWhitesmoke);
    ly = drawField(r, left, ly, rowH, "VS TGT", vsTgt, displayH,
                   colors::kWhitesmoke);
    ly = drawField(r, left, ly, rowH, "VS REQ", vsReq, displayH,
                   colors::kWhitesmoke);
    float ry = right.y;
    ry = drawField(r, right, ry, rowH, "TOD", tod, displayH,
                   colors::kWhitesmoke);
    ry = drawField(r, right, ry, rowH, "FPA", fpa, displayH, colors::kCyan);
    ry = drawField(r, right, ry, rowH, "V DEV", vdev, displayH,
                   colors::kWhitesmoke);
  }

  // Selected Waypoint Weather (WT MFDFPLPage): the box for the highlighted
  // waypoint's datalink (SiriusXM/Connext) weather. With no datalink weather
  // source wired in, the body stays empty, matching the trainer's MFD Active
  // Flight Plan page when no weather product has been received.
  drawGroupBox(r, stack.slot(wxWt), "Selected Waypoint Weather", displayH);

  // Bottom prompt (WT .mfd-fpl-bottom-prompt, exact text).
  r.fillText(f.panel.x + f.panel.w * 0.5f,
             f.panel.y + f.panel.h - mfdFontPx(20.0f, displayH),
             "Press the \"FPL\" key to view the previous page",
             mfdFontPx(16.0f, displayH), TextAlign::Center,
             colors::kWhitesmoke);

  // Editing overlays above the page content.
  if (ui.fplEntryActive()) {
    drawFplEntryWindow(r, ui, map, f.panel, displayH);
  } else if (ui.fplConfirm() != MfdController::FplConfirm::None) {
    drawFplConfirmWindow(r, ui, x, y, w, h, displayH);
  }
  // The Active Flight Plan Page Menu (MENU key) renders via the shared
  // drawPageMenu overlay in MultiFunctionDisplay, like the Navigation Map.
}

}  // namespace avionics::mfd
