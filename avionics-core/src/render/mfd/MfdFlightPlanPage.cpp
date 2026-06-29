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
#include "avionics/ProcedureSupport.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/ChromeInternal.h"
#include "render/pfd/PfdFlightPlanSections.h"

using avionics::pfd::FplDisplayRow;
using avionics::pfd::FplDisplayRowKind;
using avionics::pfd::FplSectionRow;
using avionics::pfd::buildFplApproachDisplayRows;
using avionics::pfd::buildFplProcedureDisplayRows;
using avionics::pfd::buildFplSectionRows;
using avionics::pfd::fplUsesProcedureDisplayRows;
using avionics::pfd::fplProcedureDisplayRowIndexForSelectable;
using avionics::pfd::fplProcedureDisplayRowSelectable;
using avionics::pfd::fplProcedureLegIndexForSelectable;
using avionics::pfd::fplProcedureSelectableRowForLegIndex;
using avionics::pfd::fplFilterDuplicateLegSectionRows;
using avionics::pfd::fplApproachDisplayRowIndexForSelectable;
using avionics::pfd::fplApproachDisplayRowSelectable;
using avionics::pfd::fplHeaderDestinationIdent;
using avionics::pfd::fplHeaderOriginIdent;
using avionics::pfd::fplSectionRowIsActiveDisplay;
using avionics::pfd::fplSectionRowIsSelectable;
using avionics::pfd::fplApproachSelectableRowForLegIndex;
using avionics::pfd::fplApproachSelectableRowForHoldLegIndex;
using avionics::pfd::fplSectionSelectableRowForLegIndex;
using avionics::pfd::fplApproachDisplayRowIndexForLegIndex;
using avionics::pfd::fplApproachDisplayRowIndexForHoldLegIndex;
using avionics::pfd::fplApproachLegIndexForSelectable;
using avionics::pfd::fplSectionLegIndexForSelectable;
using avionics::pfd::fplListScrollFirst;
using avionics::pfd::fplPinActiveApproachLeg;
using avionics::pfd::fplShowActiveLegHighlight;
using avionics::pfd::fplShowListRowSelection;
using avionics::pfd::fplShowsDestinationBlankRow;
using avionics::fplApproachLayoutDestFilled;

namespace avionics::mfd {

namespace {

// MFD-specific active-leg flash. Unlike the PFD FPL window (which opens with the
// list cursor live, so the active fix flashes immediately), the MFD FPL page
// opens with the FMS cursor inactive and the knobs doing page navigation. Until
// the FMS knob is pushed, nothing on the page flashes — the active TO fix is
// steady magenta. Once the cursor is engaged, defer to the shared rule (the
// cursored leg flashes).
bool mfdActiveNavRowBlink(int legIdx, int activeLegIdx, int cursorLegIdx,
                          bool cursorOn, int listCursorRow,
                          int activeSelectableRow) {
  if (!cursorOn) return false;
  return ::avionics::fplActiveNavRowBlink(legIdx, activeLegIdx, cursorLegIdx,
                                          cursorOn, listCursorRow,
                                          activeSelectableRow);
}

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

// Draws the five-digit VNAV altitude-constraint entry field (ALT column),
// right-aligned to end at rightX with an "FT" suffix: the cursor cell shows in
// reverse video, the rest white (Pilot's Guide, Section 6 altitude entry).
void drawAltEntryCells(Renderer& r, float rightX, float cy,
                       const std::string& digits, int cursor, float displayH) {
  const float cellSize = mfdFontPx(kWtRow, displayH);
  const float cellW = cellSize * 0.56f;
  const float cellGap = cellSize * 0.04f;
  const int n = 5;
  const float unitSize = cellSize * kUnitEm;
  const float unitW =
      r.measureTextWidth("FT", unitSize) + mfdFontPx(2.0f, displayH);
  r.fillText(rightX, cy, "FT", unitSize, TextAlign::Right, colors::kWhitesmoke);
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

const MapFeature* directToInsetCenter(const MapData& map, bool hasMatch,
                                    const MapFeature& wpt,
                                    MapFeature& resolved) {
  if (!hasMatch) return nullptr;
  resolved = mfd::resolveWaypointGeo(map, wpt);
  return mfd::mapFeatureHasGeo(resolved) ? &resolved : nullptr;
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

// Empty VNAV altitude-constraint field on the FPL list: five evenly spaced
// underscores with a small "FT" suffix, cyan, right-aligned to end at rightX
// (trainer FPL ALT column). The spaced dash run reads as five distinct
// underscores instead of a merged bar and keeps a gap from the DIS column.
// When parked under the FMS cursor it shows the pulsing cyan select plate.
void drawFplAltPlaceholder(Renderer& r, float rightX, float cy, float size,
                           bool selected, bool blinkOn) {
  const float unitSize = size * kUnitEm;
  const float unitW = r.measureTextWidth("FT", unitSize);
  const float unitGap = size * 0.12f;
  const int n = 5;
  const float runW = tightDashRunWidth(n, size);
  const float fieldLeft = rightX - unitW - unitGap - runW;
  const bool plate = selected && blinkOn;
  const Color fg = plate ? colors::kBlack : colors::kCyan;
  if (plate) {
    r.fillRect(fieldLeft, cy - size * 0.62f, rightX - fieldLeft, size * 1.24f,
               colors::kPopoutCyan);
  }
  r.fillText(rightX, cy, "FT", unitSize, TextAlign::Right, fg);
  // Underscores sit low at the text baseline, like the real unit.
  drawTightDashRun(r, fieldLeft, cy + size * 0.30f, n, size, fg);
}

// VNAV altitude-constraint type cue on the FPL ALT column (trainer / Pilot's
// Guide Section 6). The constraint *value* is the number; the *type* is shown
// as a thin bar relative to it, matching the real unit:
//   AtOrAbove -> bar UNDER the value  (you must be at or above this altitude)
//   AtOrBelow -> bar OVER  the value  (you must be at or below this altitude)
//   At        -> no bar (a mandatory single altitude is just the number; the
//                bar-above-and-below "box" is reserved for block/between
//                altitudes, which a single-altitude leg does not represent).
// The bar spans the full "<value>FT" readout, which is right-aligned to rightX.
void drawFplAltConstraintBars(Renderer& r, float rightX, float cy, float size,
                              const std::string& value, AltConstraintType type,
                              const Color& color) {
  if (type == AltConstraintType::None) return;
  const float unitSize = size * kUnitEm;
  const float valueW = r.measureTextWidth(value, size);
  const float unitW = r.measureTextWidth("FT", unitSize);
  const float left = rightX - valueW - unitW;
  const float thickness = std::max(1.0f, size * 0.08f);
  const float overY = cy - size * 0.50f;
  const float underY = cy + size * 0.42f;
  const bool drawOver = type == AltConstraintType::AtOrBelow;
  const bool drawUnder = type == AltConstraintType::AtOrAbove;
  if (drawOver) {
    r.fillRect(left, overY, rightX - left, thickness, color);
  }
  if (drawUnder) {
    r.fillRect(left, underY, rightX - left, thickness, color);
  }
}

void drawFplHeadingDepartureColumn(Renderer& r, float colDtkR, float cy,
                                   float rowSize, const MapLeg& leg,
                                   bool showActive, float activeCourseDeg,
                                   const Color& rowColor) {
  float course = leg.legCourseDeg;
  if (showActive && activeCourseDeg > 0.0f) course = activeCourseDeg;
  if (course > 0.0f) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "hdg %03.0f", course);
    drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, rowColor);
  } else {
    drawValueWithUnit(r, colDtkR, cy, "hdg ___", kDeg, rowSize, rowColor);
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

  if (isRunwayDepartureLegId(leg.id)) {
    // Runway threshold row: blank DTK/DIS/ALT columns (trainer SID list).
  } else if (isHeadingDepartureLeg(leg)) {
    drawFplHeadingDepartureColumn(r, colDtkR, cy, rowSize, leg, showActive,
                                  d.courseDeg, rowColor);
  } else if (legIdx > 0) {
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
    // Constraint-type cue (line over/under), drawn for both the static and
    // cursor-parked states so the at/at-or-above/at-or-below sense stays
    // visible like the real unit.
    drawFplAltConstraintBars(r, colAltR, cy, rowSize, buf,
                             leg.altitudeConstraint, altColor);
  } else {
    drawFplAltPlaceholder(r, colAltR, cy, rowSize, altSel, ui.blinkOn());
  }
  (void)rowH;
}

// "HOLD" line beneath a fix that carries a published hold: ident column shows
// HOLD, DTK is the hold inbound course, DIS is the hold leg length (trainer
// FPL list). No altitude constraint column.
void drawFplHoldRow(Renderer& r, const FlightData& d,
                    const std::vector<MapLeg>& plan, int legIdx, float innerX,
                    float fixX, float colDtkR, float colDisR, float cy,
                    float rowSize, float smallSize, bool showSelection,
                    bool showActive, bool activeNavBlink, bool blinkOn) {
  if (legIdx < 0 || legIdx >= static_cast<int>(plan.size())) return;
  const MapHoldPattern& hold = plan[static_cast<std::size_t>(legIdx)].hold;
  if (!hold.active) return;

  if (showActive) {
    const Point arrow[7] = {
        {innerX + rowSize * 1.0f, cy},
        {innerX + rowSize * 0.65f, cy - rowSize * 0.35f},
        {innerX + rowSize * 0.65f, cy - rowSize * 0.10f},
        {innerX, cy - rowSize * 0.10f},
        {innerX, cy + rowSize * 0.10f},
        {innerX + rowSize * 0.65f, cy + rowSize * 0.10f},
        {innerX + rowSize * 0.65f, cy + rowSize * 0.35f}};
    r.fillPolygon(arrow, 7, colors::kMagenta);
  }

  if (showSelection) {
    drawCursorSelect(r, fixX, cy, "HOLD", rowSize, TextAlign::Left, blinkOn);
  } else if (showActive) {
    const Color holdColor =
        activeNavBlink && blinkOn ? colors::kMagenta : colors::kMagenta;
    r.fillText(fixX, cy, "HOLD", rowSize, TextAlign::Left, holdColor);
  } else {
    r.fillText(fixX, cy, "HOLD", rowSize, TextAlign::Left, colors::kWhite);
  }

  char buf[24];
  if (showActive) {
    std::snprintf(buf, sizeof(buf), "%03.0f",
                  static_cast<double>(d.fmaLegBearingDeg));
    drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, colors::kMagenta);
    std::snprintf(buf, sizeof(buf), "%.1f",
                  static_cast<double>(d.fmaLegDistanceNm));
    drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, colors::kMagenta);
  } else {
    std::snprintf(buf, sizeof(buf), "%03.0f",
                  static_cast<double>(hold.inboundCourseDeg));
    drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, colors::kWhitesmoke);
    if (hold.legLengthNm > 0.0f) {
      std::snprintf(buf, sizeof(buf), "%.1f",
                    static_cast<double>(hold.legLengthNm));
      drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, colors::kWhitesmoke);
    }
  }
}

// Right-aligned lat/lon coordinate placeholder for the Location box when no
// waypoint has resolved: "-- ° -- . -- '" (matches the PFD Waypoint
// Information popout and the trainer dashes).
void drawCoordPlaceholder(Renderer& r, float rightX, float dashCy, float size,
                          const Color& color) {
  const DashStyle ds = dashStyle(size);
  const float readoutCy = textCyForDashBottom(r, dashCy, ds, size);
  const float degW = r.measureTextWidth(kDeg, size);
  const float dotW = r.measureTextWidth(".", size);
  const float tickW = r.measureTextWidth("'", size);
  const float gap = size * 0.10f;
  const float totalW =
      6.0f * ds.advance + degW + dotW + tickW + 4.0f * gap;
  float x = rightX - totalW;
  x = drawTightDashRun(r, x, dashCy, 2, size, color);  // degrees
  x += gap;
  r.fillText(x, readoutCy, kDeg, size, TextAlign::Left, color);
  x += degW + gap;
  x = drawTightDashRun(r, x, dashCy, 2, size, color);  // minutes
  r.fillText(x, readoutCy, ".", size, TextAlign::Left, color);
  x += dotW;
  x = drawTightDashRun(r, x, dashCy, 2, size, color);  // decimal minutes
  r.fillText(x + gap, readoutCy, "'", size, TextAlign::Left, color);
}

// The Waypoint Information entry window opened by the small FMS knob on the
// FPL page (trainer "Waypoint Information"): a full-height right-panel window
// with the spelled identifier / facility / city group box, a route map inset,
// and a Location box (bearing, distance, and lat/lon from present position),
// over the "Press ENT to accept" prompt.
void drawFplEntryWindow(Renderer& r, const FlightData& d,
                        const MfdController& ui, const MapData& map,
                        const Rect& panel, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  // Fixed-width window covering only the right portion of the FPL panel so the
  // active flight-plan list stays visible behind it. Trainer measurement: the
  // window spans 292 px of the 1024-wide display, right-aligned with a 3 px
  // margin (panel.x + panel.w == the display's right edge).
  const float popupW = mfdFontPx(292.0f, displayH);
  const float boxX = panel.x + panel.w - P(3.0f) - popupW;
  Rect inner = drawDialog(
      r, Rect{boxX, panel.y + P(4.0f), popupW, panel.h - P(8.0f)},
      "Waypoint Information", displayH);

  const bool hasMatch = ui.fplEntryHasMatch();
  const MapFeature& wpt = ui.fplEntryMatch();
  const float gap = P(4.0f);
  const float regionSize = mfdFontPx(kWtDtoFace, displayH);
  const float rowSize = mfdFontPx(kWtDtoFace, displayH);
  const float labelSize = mfdFontPx(kWtDtoLabel, displayH);
  const float readoutSize = mfdFontPx(kWtDtoReadout, displayH);

  // Resolve the matched waypoint to a geo-bearing feature (the entry match may
  // carry an id only) for the map inset and the bearing/distance/coordinates.
  MapFeature centerResolved;
  const MapFeature* center =
      directToInsetCenter(map, hasMatch, wpt, centerResolved);
  const bool haveGeo = center != nullptr && map.positionValid;
  double brg = 0.0;
  double dis = 0.0;
  if (haveGeo) {
    brg = navBearingDeg(map.ownshipLat, map.ownshipLon, center->lat,
                        center->lon);
    dis = navDistanceNm(map.ownshipLat, map.ownshipLon, center->lat,
                        center->lon);
  }
  char buf[16];

  // ---- top: Ident, Facility, City ----
  float topY = inner.y;
  {
    Rect ic = drawGroupBox(r, Rect{inner.x, topY, inner.w, P(94.0f)},
                           "Ident, Facility, City", displayH, colors::kBlack);
    topY += P(94.0f) + gap;
    const float identSize = mfdFontPx(kWtDtoIdent, displayH);
    const float cy1 = ic.y + identSize;
    const float identEnd = drawDtoIdentEntryCells(
        r, ic.x, cy1, ui.fplEntryIdent(), ui.fplEntryCursor(),
        ui.fplEntryTypedCount(), false, ui.blinkOn(), displayH);
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
    const float cy2 = cy1 + rowSize * 1.5f;
    const float cy3 = cy2 + rowSize * 1.3f;
    if (ui.fplEntryNotFound()) {
      r.fillText(ic.x, cy2, "WAYPOINT NOT FOUND", rowSize, TextAlign::Left,
                 colors::kBandYellow);
    } else if (hasMatch) {
      auto clipRow = [&](std::string text) {
        while (text.size() > 4 && r.measureTextWidth(text, rowSize) > ic.w) {
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

  // ---- bottom: "Press ENT to accept" prompt, Location box above it ----
  const float promptSize = mfdFontPx(16.0f, displayH);
  const float promptCy = inner.y + inner.h - promptSize * 0.6f;
  const float locH = P(66.0f);
  const float locY = promptCy - promptSize - gap - locH;

  {
    Rect lc = drawGroupBox(r, Rect{inner.x, locY, inner.w, locH}, "Location",
                           displayH, colors::kBlack);
    // Two rows. Everything on a row (label, value, unit, dashes, lat/lon)
    // shares a common dash baseline so the columns line up: text is anchored
    // by textCyForDashBottom so its bottom matches the dash bottom.
    const float unitSize = readoutSize * kUnitEm;
    // Center the two-row block vertically in the box. Text bottom-aligns to the
    // dash baseline, so place the two baselines symmetric about the box center,
    // offset down by half the glyph height (the text occupies the space above
    // its baseline).
    const TextRect glyph =
        r.measureTextRect(0.0f, 0.0f, "0", readoutSize, TextAlign::Left);
    const float capH = glyph.bottom - glyph.top;
    const float rowGap = readoutSize * 1.3f;
    const float midCy = lc.y + lc.h * 0.5f;
    const float dashCy1 = midCy + capH * 0.5f - rowGap * 0.5f;
    const float dashCy2 = midCy + capH * 0.5f + rowGap * 0.5f;
    const float labelCy1 =
        textCyForDashBottom(r, dashCy1, dashStyle(labelSize), labelSize);
    const float labelCy2 =
        textCyForDashBottom(r, dashCy2, dashStyle(labelSize), labelSize);
    const float valCy1 =
        textCyForDashBottom(r, dashCy1, dashStyle(readoutSize), readoutSize);
    const float valCy2 =
        textCyForDashBottom(r, dashCy2, dashStyle(readoutSize), readoutSize);
    const float unitCy2 =
        textCyForDashBottom(r, dashCy2, dashStyle(unitSize), unitSize);
    // Values start past the widest label so the BRG and DIS columns align.
    const float valX =
        lc.x + r.measureTextWidth("BRG", labelSize) + readoutSize * 0.5f;

    // BRG (row 1): white = computed.
    r.fillText(lc.x, labelCy1, "BRG", labelSize, TextAlign::Left,
               colors::kTitleGray);
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f", brg);
      r.fillText(valX, valCy1, buf, readoutSize, TextAlign::Left,
                 colors::kWhite);
      r.fillText(valX + r.measureTextWidth(buf, readoutSize), valCy1, kDeg,
                 unitSize, TextAlign::Left, colors::kWhite);
    } else {
      const float dx =
          drawTightDashRun(r, valX, dashCy1, 3, readoutSize, colors::kWhite);
      r.fillText(dx, valCy1, kDeg, unitSize, TextAlign::Left, colors::kWhite);
    }

    // DIS (row 2).
    r.fillText(lc.x, labelCy2, "DIS", labelSize, TextAlign::Left,
               colors::kTitleGray);
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      r.fillText(valX, valCy2, buf, readoutSize, TextAlign::Left,
                 colors::kWhite);
      r.fillText(valX + r.measureTextWidth(buf, readoutSize), unitCy2, "NM",
                 unitSize, TextAlign::Left, colors::kWhite);
    } else {
      float dx =
          drawTightDashRun(r, valX, dashCy2, 2, readoutSize, colors::kWhite);
      r.fillText(dx, valCy2, ".", readoutSize, TextAlign::Left, colors::kWhite);
      dx += r.measureTextWidth(".", readoutSize);
      dx = drawTightDashRun(r, dx, dashCy2, 1, readoutSize, colors::kWhite);
      r.fillText(dx, unitCy2, "NM", unitSize, TextAlign::Left, colors::kWhite);
    }

    // Latitude / longitude, right-aligned, sharing the BRG / DIS baselines.
    if (haveGeo) {
      r.fillText(lc.x + lc.w, valCy1, formatLatLon(center->lat, true),
                 readoutSize, TextAlign::Right, colors::kWhite);
      r.fillText(lc.x + lc.w, valCy2, formatLatLon(center->lon, false),
                 readoutSize, TextAlign::Right, colors::kWhite);
    } else {
      drawCoordPlaceholder(r, lc.x + lc.w, dashCy1, readoutSize,
                           colors::kWhite);
      drawCoordPlaceholder(r, lc.x + lc.w, dashCy2, readoutSize,
                           colors::kWhite);
    }
  }

  // ---- map inset fills the middle ----
  {
    const float mapBot = locY - gap;
    Rect mc = drawGroupBox(r, Rect{inner.x, topY, inner.w, mapBot - topY},
                           "Map", displayH, colors::kBlack);
    const DirectToInsetView dv = directToInsetView(map, wpt);
    drawPageMap(r, d, map, mc, dv.rangeNm, center, displayH, false, nullptr,
                0.0f, TerrainDisplay::Off, true, AirwayDisplay::Off, false,
                dv.centerLat, dv.centerLon);
  }

  r.fillText(inner.x + inner.w * 0.5f, promptCy, "Press \"ENT\" to accept",
             promptSize, TextAlign::Center, colors::kWhite);
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
    const mfd::DirectToInsetView dv = mfd::directToInsetView(map, wpt);
    MapFeature dtoCenter;
    const MapFeature* center = directToInsetCenter(map, hasMatch, wpt, dtoCenter);
    drawPageMap(r, d, map, mc, dv.rangeNm, center, displayH, false, nullptr,
                0.0f, TerrainDisplay::Off, true, AirwayDisplay::Off, false,
                dv.centerLat, dv.centerLon);
  }

  // ---- Activate? button ----
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
  }
}

namespace {

// Defined below; reused for the confirmation window's OK / CANCEL buttons so
// they match the Direct-To / Procedures rounded-button style.
float drawProcButton(Renderer& r, float leftX, float cy, const char* label,
                     float size, bool armed, bool blinkOn);

// Greedy word-wrap: split `text` into lines that each fit within `maxWidth` at
// the given font size, breaking on spaces (the real unit wraps long prompts).
std::vector<std::string> wrapTextToWidth(Renderer& r, const std::string& text,
                                         float size, float maxWidth) {
  std::vector<std::string> lines;
  std::string line;
  std::size_t i = 0;
  while (i < text.size()) {
    const std::size_t sp = text.find(' ', i);
    const std::string word =
        text.substr(i, sp == std::string::npos ? std::string::npos : sp - i);
    const std::string candidate = line.empty() ? word : line + " " + word;
    if (!line.empty() && r.measureTextWidth(candidate, size) > maxWidth) {
      lines.push_back(line);
      line = word;
    } else {
      line = candidate;
    }
    if (sp == std::string::npos) break;
    i = sp + 1;
  }
  if (!line.empty()) lines.push_back(line);
  if (lines.empty()) lines.push_back(text);
  return lines;
}

// The OK/CANCEL confirmation window ("Remove <wpt>?" from CLR on a leg row,
// "Delete all waypoints in flight plan?" from the page menu). Trainer
// screenshot043: prompt wrapped to fit the box, a blank line, then the
// "OK or CANCEL" rounded-button row with the highlighted choice filled cyan.
void drawFplConfirmWindow(Renderer& r, const MfdController& ui,
                          const Rect& panel, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };

  std::string question;
  switch (ui.fplConfirm()) {
    case MfdController::FplConfirm::RemoveWaypoint:
      question = "Remove " + ui.fplRemoveIdent() + "?";
      break;
    case MfdController::FplConfirm::RemoveDeparture:
    case MfdController::FplConfirm::RemoveArrival:
    case MfdController::FplConfirm::RemoveApproach:
    case MfdController::FplConfirm::RemoveAirway:
      // fplRemoveIdent() carries the removal subject (e.g. "CSHEL6 departure"
      // or "Airway V16").
      question = "Remove " + ui.fplRemoveIdent() + " from flight plan?";
      break;
    case MfdController::FplConfirm::DeleteFlightPlan:
    case MfdController::FplConfirm::None:
      question = "Delete all waypoints in flight plan?";
      break;
  }

  const float textSize = mfdFontPx(18.0f, displayH);
  const float lineH = textSize * 1.4f;
  const float buttonSize = mfdFontPx(kWtFieldValue, displayH);
  const float buttonRowH = buttonSize * 1.7f;

  // The confirmation sits over the right-hand Active Flight Plan panel
  // (trainer screenshot043), right-aligned like the Waypoint Information
  // window — not centered on the whole MFD where it would cover the map.
  const float boxW = mfdFontPx(292.0f, displayH);
  const float dialogPad = mfdFontPx(10.0f, displayH);
  const std::vector<std::string> lines =
      wrapTextToWidth(r, question, textSize, boxW - 2.0f * dialogPad - P(8.0f));

  // Vertical layout: top pad, the wrapped prompt, a blank spacer line, the
  // button row, bottom pad. Size the outer box so its inner rect fits.
  const float topPad = lineH * 0.6f;
  const float spacer = lineH;  // the blank line between the prompt and buttons
  const float bottomPad = lineH * 0.55f;
  const float contentH = topPad + lineH * static_cast<float>(lines.size()) +
                         spacer + buttonRowH + bottomPad;
  const float boxH = contentH + mfdFontPx(6.0f, displayH) + dialogPad;
  const float boxX = panel.x + panel.w - P(3.0f) - boxW;
  const float boxY = panel.y + (panel.h - boxH) * 0.38f;
  Rect inner = drawDialog(r, Rect{boxX, boxY, boxW, boxH}, nullptr, displayH);

  // Wrapped, centered prompt.
  float ty = inner.y + topPad + textSize * 0.5f;
  for (const std::string& line : lines) {
    r.fillText(inner.x + inner.w * 0.5f, ty, line, textSize, TextAlign::Center,
               colors::kWhite);
    ty += lineH;
  }

  // "OK  or  CANCEL" row: rounded buttons (app style), the highlighted choice
  // filled cyan, with plain "or" between them. ENT executes the highlight; the
  // FMS knob toggles it.
  const float okW = r.measureTextWidth("OK", buttonSize) + buttonSize * 1.3f;
  const float cancelW = r.measureTextWidth("CANCEL", buttonSize) + buttonSize * 1.3f;
  const float orW = r.measureTextWidth("or", buttonSize);
  const float gap = buttonSize * 0.6f;
  const float rowW = okW + gap + orW + gap + cancelW;
  const float buttonCy = inner.y + topPad +
                         lineH * static_cast<float>(lines.size()) + spacer +
                         buttonRowH * 0.5f;
  float bx = inner.x + (inner.w - rowW) * 0.5f;
  drawProcButton(r, bx, buttonCy, "OK", buttonSize, ui.fplConfirmOk(),
                 ui.blinkOn());
  bx += okW + gap;
  r.fillText(bx + orW * 0.5f, buttonCy, "or", buttonSize, TextAlign::Center,
             colors::kWhite);
  bx += orW + gap;
  drawProcButton(r, bx, buttonCy, "CANCEL", buttonSize, !ui.fplConfirmOk(),
                 ui.blinkOn());
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
      {"Arrival:",
       ui.fplHasLoadedArrival() ? ui.fplArrivalHeaderLabel() : std::string()},
      {"Departure:",
       ui.fplHasLoadedDeparture() ? ui.fplDepartureHeaderLabel() : std::string()},
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
  // Procedure names get the approach label formatting; transition / runway /
  // airport rows are shown verbatim from the list items.
  const bool rawItems = ui.procStep() != ProcStep::ProcedureList;
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
        rawItems ? items[static_cast<std::size_t>(i)]
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
        rawItems ? items[static_cast<std::size_t>(i)]
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
    // The leg-type role (iaf / faf / hold / mapt / mahp) is white, set apart from the
    // cyan waypoint ident, matching the unit's sequence list.
    const std::string role = fplLegDisplayRole(leg);
    if (!role.empty()) {
      const float roleX =
          area.x + r.measureTextWidth(leg.id, rowSize) + rowSize * 0.35f;
      r.fillText(roleX, cy, role, rowSize, TextAlign::Left,
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
    const bool entering = ui.procAirportEntryActive();
    float cellsEndX = ac.x;
    if (entering) {
      cellsEndX = drawIdentEntryCells(
          r, ac.x, cy, ui.procAirportEntryIdent(), ui.procAirportEntryCursor(),
          ui.procAirportEntryTypedCount(), ui.procAirportEntrySelectAll(),
          blinkOn, displayH, kWtFieldValue);
    } else if (airportHi && blinkOn && !sub) {
      drawCursorSelect(r, ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                       TextAlign::Left, true);
    } else {
      r.fillText(ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                 TextAlign::Left, colors::kPopoutCyan);
    }
    // While typing, mirror the FPL waypoint entry: show the spell-ahead match's
    // icon/usage and facility/city line so the pilot can confirm the airport.
    const MapFeature sym =
        entering ? ui.procAirportEntryMatch() : ui.procAirportFeature();
    const std::string entryIdent = ui.procAirportEntryIdent();
    const bool showSym = entering ? (ui.procAirportEntryHasMatch() &&
                                     !entryIdent.empty())
                                  : !icao.empty();
    if (showSym) {
      const float iconX =
          entering ? cellsEndX + P(16.0f)
                   : ac.x + r.measureTextWidth(icao, valueSize) + P(16.0f);
      drawWaypointIcon(r, iconX, cy, P(22.0f), &sym, sym.type);
      const char* usage = airportUsageType(sym);
      if (usage != nullptr) {
        r.fillText(ac.x + ac.w, cy, usage, labelSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
    }
    const std::string city =
        entering ? ui.procAirportEntryCityLine() : ui.procAirportCityLine();
    if ((entering ? ui.procAirportEntryHasMatch() : true) && !city.empty()) {
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

// PROC – Arrival / Departure Loading detail form. Stacked group boxes on the
// grey overlay panel matching the trainer field order (Arrival: Airport,
// Arrival, Transition, Runway, Sequence, Load?; Departure swaps Transition and
// Runway). Mirrors drawProcApproachForm's widgets without the approach-only
// Channel / Minimums / Primary Frequency boxes.
void drawProcArrDepForm(Renderer& r, const MfdController& ui,
                        const MapData& map, const Rect& inner,
                        float displayH) {
  using Field = ProcApproachField;
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
  const float valueSize = mfdFontPx(kWtFieldValue, displayH);
  const float gap = P(10.0f);
  const bool blinkOn = ui.blinkOn();
  const bool sub = ui.procSubListOpen();
  const bool seqFocused = ui.procSequenceFocused();
  const Color titleBg = colors::kMfdOverlayGray;
  const bool departure = ui.procCategory() == ProcedureType::Departure;

  const float buttonsH = P(38.0f);
  const float buttonsY = inner.y + inner.h - buttonsH;
  float slotY = inner.y;

  const auto rowCenterY = [](const Rect& slot, const Rect& content) {
    return (content.y + slot.y + slot.h) * 0.5f;
  };

  // Airport box (ICAO + waypoint icon + usage + city line).
  const float airportH = P(88.0f);
  {
    Rect ac = drawGroupBox(r, Rect{inner.x, slotY, inner.w, airportH},
                           "Airport", displayH, titleBg);
    const float cy = ac.y + ac.h * 0.32f;
    const std::string icao = ui.procAirportIcao();
    const bool airportHi =
        !seqFocused && ui.procApproachField() == Field::Airport;
    const bool entering = ui.procAirportEntryActive();
    float cellsEndX = ac.x;
    if (entering) {
      cellsEndX = drawIdentEntryCells(
          r, ac.x, cy, ui.procAirportEntryIdent(), ui.procAirportEntryCursor(),
          ui.procAirportEntryTypedCount(), ui.procAirportEntrySelectAll(),
          blinkOn, displayH, kWtFieldValue);
    } else if (airportHi && blinkOn && !sub) {
      drawCursorSelect(r, ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                       TextAlign::Left, true);
    } else {
      r.fillText(ac.x, cy, icao.empty() ? "_____" : icao, valueSize,
                 TextAlign::Left, colors::kPopoutCyan);
    }
    const MapFeature sym =
        entering ? ui.procAirportEntryMatch() : ui.procAirportFeature();
    const std::string entryIdent = ui.procAirportEntryIdent();
    const bool showSym = entering ? (ui.procAirportEntryHasMatch() &&
                                     !entryIdent.empty())
                                  : !icao.empty();
    if (showSym) {
      const float iconX =
          entering ? cellsEndX + P(16.0f)
                   : ac.x + r.measureTextWidth(icao, valueSize) + P(16.0f);
      drawWaypointIcon(r, iconX, cy, P(22.0f), &sym, sym.type);
      const char* usage = airportUsageType(sym);
      if (usage != nullptr) {
        r.fillText(ac.x + ac.w, cy, usage, labelSize, TextAlign::Right,
                   colors::kWhitesmoke);
      }
    }
    const std::string city =
        entering ? ui.procAirportEntryCityLine() : ui.procAirportCityLine();
    if ((entering ? ui.procAirportEntryHasMatch() : true) && !city.empty()) {
      const float nameCy = cy + valueSize * 1.25f;
      r.fillText(ac.x, nameCy, city, labelSize, TextAlign::Left,
                 colors::kPopoutCyan);
    }
  }
  slotY += airportH + gap;

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
  drawFieldBox(aprSlot, departure ? "Departure" : "Arrival",
               ui.procSelectedApproachDisplay(), Field::Apr);
  slotY += fieldH + gap;

  // Transition / Runway order differs by category (trainer): Arrival shows
  // Transition then Runway; Departure shows Runway then Transition.
  Rect transSlot{};
  Rect rwySlot{};
  if (departure) {
    rwySlot = Rect{inner.x, slotY, inner.w, fieldH};
    drawFieldBox(rwySlot, "Runway", ui.procSelectedRunwayDisplay(),
                 Field::Runway);
    slotY += fieldH + gap;
    transSlot = Rect{inner.x, slotY, inner.w, fieldH};
    drawFieldBox(transSlot, "Transition", ui.procSelectedTransitionDisplay(),
                 Field::Trans);
    slotY += fieldH + gap;
  } else {
    transSlot = Rect{inner.x, slotY, inner.w, fieldH};
    drawFieldBox(transSlot, "Transition", ui.procSelectedTransitionDisplay(),
                 Field::Trans);
    slotY += fieldH + gap;
    rwySlot = Rect{inner.x, slotY, inner.w, fieldH};
    drawFieldBox(rwySlot, "Runway", ui.procSelectedRunwayDisplay(),
                 Field::Runway);
    slotY += fieldH + gap;
  }

  const float seqH = buttonsY - gap - slotY;
  {
    Rect sc = drawGroupBox(r, Rect{inner.x, slotY, inner.w, seqH}, "Sequence",
                           displayH, titleBg);
    drawProcSequenceRows(r, sc, displayH, ui.procPreviewLegs(), map,
                         ui.procSequenceSelected(), ui.procSequenceFocused(),
                         blinkOn);
  }

  // Footer: single centered Load? button (Arrival/Departure have no Activate).
  const float btnCy = buttonsY + buttonsH * 0.5f;
  const float loadW =
      r.measureTextWidth("Load?", valueSize) + valueSize * 1.3f;
  drawProcButton(r, inner.x + (inner.w - loadW) * 0.5f, btnCy, "Load?",
                 valueSize,
                 !seqFocused && ui.procApproachField() == Field::Load &&
                     ui.procLoadArmed(),
                 blinkOn);

  if (sub) {
    const Rect* anchor = &aprSlot;
    if (ui.procStep() == ProcStep::TransitionList) {
      anchor = &transSlot;
    } else if (ui.procStep() == ProcStep::RunwayList) {
      anchor = &rwySlot;
    }
    drawProcSubList(r, ui, inner, *anchor, displayH);
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
    drawProcArrDepForm(r, ui, map, inner, displayH);
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

void drawLoadAirwayWindow(Renderer& r, const FlightData& d, const MapData& map,
                          const MfdController& ui, float x, float y, float w,
                          float h, float displayH) {
  (void)d;
  (void)map;
  using Field = MfdController::LoadAirwayField;
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float labelSize = mfdFontPx(kWtFieldValue, displayH);
  const bool blinkOn = ui.blinkOn();
  const Color titleBg = colors::kMfdOverlayGray;

  // Tall right-side panel like the Select Approach window.
  const float boxW = w * 0.40f;
  const float boxH = h - P(4.0f);
  Rect inner = drawProcOverlayPanel(
      r, Rect{x + w - boxW - P(2.0f), y + P(2.0f), boxW, boxH}, displayH);

  const float fieldH = P(44.0f);
  const float gap = P(20.0f);
  float slotY = inner.y + P(4.0f);

  // A single-value field box: the value is highlighted (pulsing cyan) when the
  // field cursor is parked on it.
  const auto drawValueField = [&](const char* title, const std::string& value,
                                  bool focused) {
    Rect box = drawGroupBox(r, Rect{inner.x, slotY, inner.w, fieldH}, title,
                            displayH, titleBg);
    const float cy = box.y + box.h * 0.5f;
    const std::string shown = value.empty() ? std::string("_____") : value;
    if (focused) {
      drawCursorSelect(r, box.x, cy, shown, labelSize, TextAlign::Left, blinkOn);
    } else {
      r.fillText(box.x, cy, shown, labelSize, TextAlign::Left,
                 colors::kPopoutCyan);
    }
    slotY += fieldH + gap;
  };

  const Field field = ui.loadAirwayField();
  drawValueField("Entry", ui.loadAirwayEntryIdent(), false);
  const float airwayBoxTop = slotY;  // captured before drawValueField advances
  drawValueField("Airway", ui.loadAirwayName(), field == Field::Airway);
  const float exitBoxTop = slotY;
  drawValueField("Exit", ui.loadAirwayExitIdent(), field == Field::Exit);

  // Footer Load? button.
  const float buttonsH = P(38.0f);
  const float buttonsY = inner.y + inner.h - buttonsH;
  const float btnCy = buttonsY + buttonsH * 0.5f;
  drawProcButton(
      r, inner.x + (inner.w - (r.measureTextWidth("Load?", labelSize) +
                               labelSize * 1.3f)) *
                       0.5f,
      btnCy, "Load?", labelSize,
      field == Field::Load && ui.loadAirwayCanLoad(), blinkOn);

  const std::vector<MapLeg>& fixes = ui.loadAirwayFixes();
  const int exitSel = ui.loadAirwayExitSel();
  const float rowH = labelSize * 1.5f;

  // Sequence group box (always shown): the chosen entry->exit fix chain, each
  // leg showing its DTK/DIS inline in the right columns (no free-floating
  // course readout).
  {
    const Rect seqSlot{inner.x, slotY, inner.w,
                       std::max(P(24.0f), buttonsY - gap - slotY)};
    const Rect seq = drawGroupBox(r, seqSlot, "Sequence", displayH, titleBg);
    const int total = std::min(exitSel + 1, static_cast<int>(fixes.size()));
    const int maxRows = std::max(1, static_cast<int>(seq.h / rowH));
    int first = 0;
    if (total > maxRows) {
      first = std::min(std::max(0, exitSel - maxRows / 2), total - maxRows);
    }
    const int last = std::min(total, first + maxRows);
    const float seqRight = seq.x + seq.w;
    float ry = seq.y;
    for (int i = first; i < last; ++i) {
      const float cy = ry + rowH * 0.5f;
      const std::string& id = fixes[static_cast<std::size_t>(i)].id;
      if (i == exitSel) {
        drawCursorSelect(r, seq.x, cy, id, labelSize, TextAlign::Left, blinkOn);
      } else {
        r.fillText(seq.x, cy, id, labelSize, TextAlign::Left,
                   colors::kPopoutCyan);
      }
      // Each leg (every fix after the entry) shows the course/distance into it.
      if (i >= 1) {
        const MapLeg& a = fixes[static_cast<std::size_t>(i - 1)];
        const MapLeg& b = fixes[static_cast<std::size_t>(i)];
        const float dis = static_cast<float>(navDistanceNm(a.lat, a.lon, b.lat, b.lon));
        const float dtk = static_cast<float>(navBearingDeg(a.lat, a.lon, b.lat, b.lon));
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.1f", dis);
        drawValueWithUnit(r, seqRight, cy, buf, "NM", labelSize,
                          colors::kWhitesmoke);
        const float disW =
            r.measureTextWidth("NM", labelSize * kUnitEm) +
            r.measureTextWidth(buf, labelSize);
        std::snprintf(buf, sizeof(buf), "%03.0f", dtk);
        drawValueWithUnit(r, seqRight - disW - labelSize * 0.6f, cy, buf, kDeg,
                          labelSize, colors::kWhitesmoke);
      }
      ry += rowH;
    }
  }

  // Dropdown overlay (drawn last so it floats above the Exit/Sequence boxes):
  // a popup list anchored under the active Airway/Exit field. The Sequence box
  // behind it carries the course/distance, so no separate readout is drawn.
  const bool airwayDropdown = field == Field::Airway;
  const bool exitDropdown = field == Field::Exit;
  if (airwayDropdown || exitDropdown) {
    std::vector<std::string> items;
    int sel = 0;
    int greyed = -1;
    float dropTop = 0.0f;
    if (airwayDropdown) {
      items = ui.loadAirwayAirways();
      sel = ui.loadAirwayAirwaySel();
      dropTop = airwayBoxTop + fieldH * 0.62f;
    } else {
      items.reserve(fixes.size());
      for (const MapLeg& fx : fixes) items.push_back(fx.id);
      sel = exitSel;
      greyed = 0;  // the entry fix can never be the exit
      dropTop = exitBoxTop + fieldH * 0.62f;
    }

    constexpr int kDropdownRows = 7;
    const int n = static_cast<int>(items.size());
    const int visible = std::max(1, std::min(n, kDropdownRows));
    int dfirst = 0;
    if (n > visible) {
      dfirst = std::max(0, std::min(sel - visible / 2, n - visible));
    }
    float maxW = 0.0f;
    for (const std::string& it : items) {
      maxW = std::max(maxW, r.measureTextWidth(it.c_str(), labelSize));
    }
    const float dPadX = labelSize * 0.5f;
    const float dropW = std::min(inner.w * 0.6f, maxW + 2.0f * dPadX);
    const float dropH = rowH * static_cast<float>(visible) + labelSize * 0.4f;
    r.fillRoundedRect(inner.x, dropTop, dropW, dropH, P(6.0f), colors::kBlack);
    r.strokeRoundedRect(inner.x + 0.75f, dropTop + 0.75f, dropW - 1.5f,
                        dropH - 1.5f, P(6.0f), 1.5f, colors::kMenuBorderGray);
    const int dlast = std::min(n, dfirst + visible);
    float dy = dropTop + labelSize * 0.2f;
    for (int i = dfirst; i < dlast; ++i) {
      const float cy = dy + rowH * 0.5f;
      const std::string& it = items[static_cast<std::size_t>(i)];
      if (i == sel) {
        drawCursorText(r, inner.x + dPadX, cy, it, labelSize, TextAlign::Left);
      } else {
        const Color c = i == greyed ? colors::kTitleGray : colors::kPopoutCyan;
        r.fillText(inner.x + dPadX, cy, it, labelSize, TextAlign::Left, c);
      }
      dy += rowH;
    }
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
  const std::vector<MapLeg> procPreview =
      ui.procMenuOpen() ? ui.procPreviewLegs() : std::vector<MapLeg>{};
  const std::vector<MapLeg>* procPreviewPtr =
      procPreview.empty() ? nullptr : &procPreview;

  // While PROC -> Select is open the inset frames the highlighted procedure's
  // legs the way the navigation Map page does (Pilot's Guide 5.8), centering
  // and zooming so the approach is visible; the RANGE knob then zooms the
  // preview until the pilot turns it. The FPL map sits left of the PROC window,
  // so the legs are centered without the Map page's east shift. Otherwise the
  // inset auto-frames the active route around ownship.
  double viewCenterLat = kNoViewCenter;
  double viewCenterLon = kNoViewCenter;
  const MapFeature procApt = ui.procAirportFeature();
  const MapFeature* procAptPtr =
      mfd::mapFeatureHasGeo(procApt) ? &procApt : nullptr;
  const ProcPreviewFit procFit =
      procPreviewMapFit(procPreview, procAptPtr, f.map.w, f.map.h);
  // Flight-plan fix under the FMS list cursor (valid geo only). When parked on
  // a fix the route preview centers on and zooms into it.
  int cursorFixLeg = -1;
  if (ui.fplCursorOn()) {
    const int cl = ui.fplCursorLegIndexPublic();
    if (cl >= 0 && cl < static_cast<int>(plan.size()) &&
        (plan[static_cast<std::size_t>(cl)].lat != 0.0 ||
         plan[static_cast<std::size_t>(cl)].lon != 0.0)) {
      cursorFixLeg = cl;
    }
  }
  if (procFit.valid) {
    ui.setProcPreviewFitRange(procFit.ladderIndex);
    viewCenterLat = procFit.centerLat;
    viewCenterLon = procFit.centerLon;
  } else if (cursorFixLeg >= 0) {
    // FMS list cursor parked on a fix: center the preview on it and zoom in
    // tight (keeping the drawn route and the TER/AWY/NEXRAD map-display
    // settings), so the highlighted waypoint reads clearly like the trainer.
    // A manual RANGE knob turn still wins (we then only recenter).
    const MapLeg& cursorFix = plan[static_cast<std::size_t>(cursorFixLeg)];
    viewCenterLat = cursorFix.lat;
    viewCenterLon = cursorFix.lon;
    ui.setFplPreviewFitRange(mapRangeIndexForNm(kFplFixFocusRangeNm));
  } else if (!ui.fplPreviewRangeManual()) {
    float routeRangeNm = 25.0f;
    if (map.positionValid && plan.size() >= 2) {
      double maxNm = 0.0;
      for (const MapLeg& leg : plan) {
        maxNm = std::max(maxNm, navDistanceNm(map.ownshipLat, map.ownshipLon,
                                              leg.lat, leg.lon));
      }
      routeRangeNm =
          std::max(10.0f, std::min(150.0f, static_cast<float>(maxNm) * 1.2f));
    }
    ui.setFplPreviewFitRange(mapRangeIndexForNm(routeRangeNm));
  }
  const float previewRangeNm = ui.rangeNm();
  const float previewDisplayRangeNm = ui.displayRangeNm();
  // Honor the map-display softkeys (Map Opt: TER / AWY / NEXRAD) on the FPL
  // page's route preview, so the inset matches the navigation map instead of
  // always drawing topo terrain.
  drawPageMap(r, d, map, f.map, previewRangeNm, nullptr, displayH, false,
              procPreviewPtr, previewDisplayRangeNm, ui.terrainDisplay(), false,
              ui.airwayDisplay(), ui.showWeather(), viewCenterLat,
              viewCenterLon);

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
    int arrStart = ui.fplArrivalLegStart();
    int arrCount = ui.fplArrivalLegCount();
    std::string arrHeader = ui.fplArrivalHeaderLabel();
    const std::string arrAirport = ui.fplArrivalAirportIcao();
    int depStart = ui.fplDepartureLegStart();
    int depCount = ui.fplDepartureLegCount();
    std::string depHeader = ui.fplDepartureHeaderLabel();
    const std::string depAirport = ui.fplDepartureAirportIcao();
    // Drop terminal-procedure (departure/arrival) blocks whose indices no longer
    // fit the current plan. This happens when a shorter plan replaces a longer
    // route (e.g. loading an approach) without the grouping being cleared; the
    // stale block would otherwise force the procedure display path and hide the
    // real legs behind a phantom header + blank rows.
    const int planSize = static_cast<int>(plan.size());
    if (!avionics::pfd::fplBlockFitsPlan(depStart, depCount, planSize)) {
      depStart = 0;
      depCount = 0;
      depHeader.clear();
    }
    if (!avionics::pfd::fplBlockFitsPlan(arrStart, arrCount, planSize)) {
      arrStart = 0;
      arrCount = 0;
      arrHeader.clear();
    }
    // The approach is the procedure tail after any loaded arrival/STAR block, so
    // its inference must skip the STAR legs (which can carry procedureRole tags).
    const int arrivalEnd = arrCount > 0 ? arrStart + arrCount : 0;
    int approachStart = ui.fplApproachLegStart();
    int approachCount = ui.fplApproachLegCount();
    const InferredProcedureBlock approachBlock = resolveApproachBlockInPlan(
        plan, approachStart, approachCount, ui.fplApproachTransition(),
        arrivalEnd);
    approachStart = approachBlock.start;
    approachCount = approachBlock.count;
    if (approachCount > 0 && approachStart >= 0 &&
        approachStart + approachCount < static_cast<int>(plan.size())) {
      approachCount = fplNormalizedApproachCount(
          approachStart, approachCount, static_cast<int>(plan.size()));
    }
    const bool approachLoaded = approachCount > 0;
    // A plan carrying loaded-airway legs also uses the procedure display rows so
    // the enroute legs can be grouped under "Airway -" headers and honor the
    // collapse/expand toggle (Pilot's Guide, Load Airway).
    const bool hasAirwayLegs = avionics::pfd::fplPlanHasAirwayLegs(plan);
    const bool airwaysCollapsed = ui.fplAirwaysCollapsed();
    const bool procedureDisplay =
        fplUsesProcedureDisplayRows(depHeader, depCount, arrHeader, arrCount,
                                    approachCount) ||
        hasAirwayLegs;
    std::string approachAirport;
    if (approachLoaded) {
      approachAirport = ui.fplApproachAirportIcao();
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
                   approachStart <= static_cast<int>(plan.size())) {
          approachAirport = plan[static_cast<std::size_t>(approachStart - 1)].id;
        }
      }
    }
    // Direct-To: copy the PFD Navigation Status Box (D→ + target, magenta); the
    // section template below stays blank until the pilot builds a flight plan.
    const bool directToFplView =
        navDirectToActive(d) && !ui.fplLocalDraft();
    const bool destOnlyPlan =
        ui.fplDestinationFilled() && plan.size() == 1;
    const float colDtkR = inner.x + mfdFontPx(210.0f, displayH);
    // DIS sits left of the trainer's value column so the longest distance
    // ("999NM") clears the ALT constraint field / entry cells to its right.
    const float colDisR = inner.x + mfdFontPx(300.0f, displayH);

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
      r.fillText(inner.x + mfdFontPx(235.0f, displayH), fy + colHdrSize * 0.6f,
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
    const bool layoutDestFilled = ui.fplDestinationFilledForLayout();
    // A loaded approach always ends the route at the approach airport, so the
    // destination section is filled — keep the destination airport (e.g. KJAX)
    // in the approach header rather than letting it fall into the Enroute list.
    // The display resolves the approach block locally, so do not depend solely
    // on the controller's stored approach count (which can lag for imported
    // routes whose approach is only inferred here).
    const bool destFilled = layoutDestFilled || approachLoaded;
    const bool directToPlanBody = directToFplView;
    const bool blankOriginSection =
        directToFplView || destOnlyPlan || ui.fplHasLoadedDeparture() ||
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
    const bool fplActiveLayoutDestFilled = layoutDestFilled;
    const std::vector<FplDisplayRow> procedureDisplayRows =
        procedureDisplay
            ? buildFplProcedureDisplayRows(
                  plan, depStart, depCount, depHeader, arrStart, arrCount,
                  arrHeader, approachStart, approachCount, blankOriginSection,
                  destFilled, airwaysCollapsed)
            : std::vector<FplDisplayRow>{};
    const std::vector<FplSectionRow> sectionRows =
        procedureDisplay ? std::vector<FplSectionRow>{}
                       : fplFilterDuplicateLegSectionRows(
                             buildFplSectionRows(bodyLegCount,
                                                 layoutDestFilled,
                                                 directToPlanBody),
                             plan);
    const int displayRowCount =
        procedureDisplay ? static_cast<int>(procedureDisplayRows.size())
                       : static_cast<int>(sectionRows.size());
    const int totalRows = displayRowCount;

    const int activeSelectableRow =
        [&]() {
          int row =
              procedureDisplay
                  ? fplProcedureSelectableRowForLegIndex(
                        activeLegIdx, plan, depStart, depCount, depHeader,
                        arrStart, arrCount, arrHeader, approachStart,
                        approachCount, blankOriginSection, destFilled,
                        airwaysCollapsed)
                  : fplSectionSelectableRowForLegIndex(
                        activeLegIdx, sectionRows, bodyLegCount,
                        layoutDestFilled, directToPlanBody);
          if (procedureDisplay && activeLegIdx >= 0 &&
              activeLegIdx < static_cast<int>(plan.size())) {
            const MapLeg& activeLeg =
                plan[static_cast<std::size_t>(activeLegIdx)];
            const bool dtoNavActive =
                navDirectToActive(d) || map.directToActive;
            if (fplHoldNavActiveOnLeg(d, map.directToHold, map.directToActive,
                                      dtoNavActive, activeLegIdx, activeLegIdx,
                                      activeLeg)) {
              const int holdRow = fplApproachSelectableRowForHoldLegIndex(
                  activeLegIdx, plan, approachStart, approachCount,
                  blankOriginSection, destFilled);
              if (holdRow >= 0) row = holdRow;
            }
          }
          return row;
        }();
    const bool pinActiveApproachLeg = fplPinActiveApproachLeg(
        approachLoaded, activeLegIdx, approachStart, approachCount,
        ui.fplLocalDraft(),
        directToFplView || map.directToActive);
    int cursorLegIdx = -1;
    if (procedureDisplay) {
      cursorLegIdx = fplProcedureLegIndexForSelectable(
          listCursorRow, plan, depStart, depCount, depHeader, arrStart,
          arrCount, arrHeader, approachStart, approachCount, blankOriginSection,
          destFilled, airwaysCollapsed);
    } else {
      cursorLegIdx = fplSectionLegIndexForSelectable(
          listCursorRow, sectionRows, bodyLegCount, layoutDestFilled);
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
    if (procedureDisplay) {
      scrollAnchor = fplProcedureDisplayRowIndexForSelectable(
          listCursorRow, plan, depStart, depCount, depHeader, arrStart,
          arrCount, arrHeader, approachStart, approachCount, blankOriginSection,
          destFilled, airwaysCollapsed);
    } else if (!sectionRows.empty()) {
      scrollAnchor = sectionDisplayRowForSelectable(
          listCursorRow, sectionRows, bodyLegCount,
          layoutDestFilled);
    }

    const int maxRows =
        std::max(1, static_cast<int>((inner.y + inner.h - fy) / rowH));
    int start = 0;
    if (totalRows > maxRows) {
      int pinnedRow = -1;
      if (pinActiveApproachLeg && approachLoaded) {
        const bool dtoNavActive =
            navDirectToActive(d) || map.directToActive;
        if (activeLegIdx >= 0 &&
            activeLegIdx < static_cast<int>(plan.size()) &&
            fplHoldNavActiveOnLeg(d, map.directToHold, map.directToActive,
                                  dtoNavActive, activeLegIdx, activeLegIdx,
                                  plan[static_cast<std::size_t>(activeLegIdx)])) {
          pinnedRow = fplApproachDisplayRowIndexForHoldLegIndex(
              activeLegIdx, plan, approachStart, approachCount,
              blankOriginSection, destFilled);
        } else {
          pinnedRow = fplApproachDisplayRowIndexForLegIndex(
              activeLegIdx, plan, approachStart, approachCount,
              blankOriginSection, destFilled);
        }
      }
      start = fplListScrollFirst(scrollAnchor, pinnedRow, totalRows, maxRows);
    }

    const std::string approachTransition = ui.fplApproachTransition();
    const int end = std::min(totalRows, start + maxRows);
    int selectableIdx = 0;
      if (start > 0) {
      if (procedureDisplay) {
        for (int i = 0; i < start; ++i) {
          if (fplProcedureDisplayRowSelectable(
                  procedureDisplayRows[static_cast<std::size_t>(i)].kind)) {
            ++selectableIdx;
          }
        }
      } else {
        countSectionSelectablesBefore(start, sectionRows, bodyLegCount,
                                      layoutDestFilled, selectableIdx);
      }
    }
    for (int row = start; row < end; ++row) {
      const float cy = fy + rowH * 0.5f;

      if (procedureDisplay) {
        const FplDisplayRow& dr =
            procedureDisplayRows[static_cast<std::size_t>(row)];
        const bool showSelection =
            cursorOn && fplShowListRowSelection(selectableIdx, listCursorRow,
                                                activeSelectableRow, cursorOn);
        switch (dr.kind) {
          case FplDisplayRowKind::DepartureHeader:
            ++selectableIdx;
            if (showSelection) {
              drawCursorSelect(r, labelX, cy, depAirport + "-" + depHeader,
                               rowSize, TextAlign::Left, blinkOn);
            } else {
              drawFplApproachHeader(r, labelX, cy, depAirport, depHeader, rowSize,
                                    colors::kCyan);
            }
            fy += rowH;
            continue;
          case FplDisplayRowKind::ArrivalHeader:
            ++selectableIdx;
            if (showSelection) {
              drawCursorSelect(r, labelX, cy, arrAirport + "-" + arrHeader,
                               rowSize, TextAlign::Left, blinkOn);
            } else {
              drawFplApproachHeader(r, labelX, cy, arrAirport, arrHeader, rowSize,
                                    colors::kCyan);
            }
            fy += rowH;
            continue;
          case FplDisplayRowKind::SepDash:
            ++selectableIdx;
            drawFplDashRow(r, labelX, cy, kFplApproachSepDashCount, rowSize,
                           colors::kPopoutCyan, showSelection, blinkOn);
            fy += rowH;
            continue;
          case FplDisplayRowKind::ApproachHeader:
            ++selectableIdx;
            if (showSelection) {
              drawCursorSelect(r, labelX, cy,
                               approachAirport + "-" + ui.fplApproachHeaderLabel(),
                               rowSize, TextAlign::Left, blinkOn);
            } else {
              drawFplApproachHeader(r, labelX, cy, approachAirport,
                                    ui.fplApproachHeaderLabel(), rowSize,
                                    colors::kCyan);
            }
            fy += rowH;
            continue;
          case FplDisplayRowKind::AirwayHeader: {
            // "Airway - <name>.<exit>" parent row above a loaded airway segment
            // (Pilot's Guide, Load Airway). dr.legIndex points at the exit fix.
            // Selectable cursor stop: CLR on it removes the whole airway.
            ++selectableIdx;
            const MapLeg& exitLeg =
                plan[static_cast<std::size_t>(dr.legIndex)];
            const std::string awLabel =
                "Airway - " + exitLeg.viaAirway + "." + exitLeg.id;
            if (showSelection) {
              drawCursorSelect(r, labelX, cy, awLabel, rowSize, TextAlign::Left,
                               blinkOn);
            } else {
              r.fillText(labelX, cy, awLabel, rowSize, TextAlign::Left,
                         colors::kCyan);
            }
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::Hold: {
            // Use the outer showSelection (computed at this row's own
            // selectable index) before advancing; recomputing after the
            // increment would key the HOLD row off the next row, so the cursor
            // would skip it and the following row would flash with it.
            ++selectableIdx;
            const MapLeg& leg =
                plan[static_cast<std::size_t>(dr.legIndex)];
            const bool dtoNavActive =
                navDirectToActive(d) || map.directToActive;
            const bool showActive = fplHoldNavActiveOnLeg(
                d, map.directToHold, map.directToActive, dtoNavActive,
                dr.legIndex, activeLegIdx, leg);
            const bool activeNavBlink =
                showActive &&
                mfdActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
                                     cursorOn, listCursorRow,
                                     activeSelectableRow);
            drawFplHoldRow(r, d, plan, dr.legIndex, inner.x, filledIdentX,
                           colDtkR, colDisR, cy, rowSize, colHdrSize,
                           showSelection, showActive, activeNavBlink,
                           showSelection ? blinkOn : activeNavBlink);
            fy += rowH;
            continue;
          }
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
                  mfdActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
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
            drawFplDashRow(r, filledIdentX, cy, kFplDashCount, rowSize,
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
                  mfdActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
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
          case FplDisplayRowKind::DestinationLabel:
            drawFplDestinationLabelRow(r, labelX, cy, rowSize);
            fy += rowH;
            continue;
          case FplDisplayRowKind::DestinationBlank:
            ++selectableIdx;
            drawFplDashRow(r, filledIdentX, cy, kFplDashCount, rowSize,
                           colors::kPopoutCyan, showSelection, blinkOn);
            fy += rowH;
            continue;
          case FplDisplayRowKind::EnrouteBlank: {
            ++selectableIdx;
            drawFplDashRow(r, filledIdentX, cy, kFplDashCount, rowSize,
                           colors::kPopoutCyan, showSelection, blinkOn);
            fy += rowH;
            continue;
          }
          case FplDisplayRowKind::EnrouteLeg:
          case FplDisplayRowKind::DepartureLeg:
          case FplDisplayRowKind::ArrivalLeg:
          case FplDisplayRowKind::ApproachLeg: {
            ++selectableIdx;
            const MapLeg& leg =
                plan[static_cast<std::size_t>(dr.legIndex)];
            const bool dtoNavActive =
                navDirectToActive(d) || map.directToActive;
            const bool holdNavOnLeg = fplHoldNavActiveOnLeg(
                d, map.directToHold, map.directToActive, dtoNavActive,
                dr.legIndex, activeLegIdx, leg);
            const bool showActive =
                !holdNavOnLeg &&
                fplShowActiveNavRow(activeHighlight, dr.legIndex, activeLegIdx,
                                    leg, navToIdent);
            const bool activeNavBlink =
                showActive &&
                mfdActiveNavRowBlink(dr.legIndex, activeLegIdx, cursorLegIdx,
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
          !sectionRowIsSelectable(sr, bodyLegCount, layoutDestFilled)) {
        drawFplSectionIdent(r, labelX, cy, "Origin - ", std::string(), true,
                            false, false, rowSize, colors::kPopoutCyan);
        fy += rowH;
        continue;
      }
      if (!sectionRowIsSelectable(sr, bodyLegCount, layoutDestFilled)) {
        fy += rowH;
        continue;
      }

      const bool showSelection =
          cursorOn && fplShowListRowSelection(selectableIdx, listCursorRow,
                                              activeSelectableRow, cursorOn);
      switch (sr.kind) {
        case FplSectionRow::Kind::Origin: {
          if (sr.legIndex >= 0) {
            const bool showActive = fplShowActiveNavRow(
                activeHighlight, sr.legIndex, activeLegIdx,
                plan[static_cast<std::size_t>(sr.legIndex)], navToIdent);
            const bool activeNavBlink =
                showActive &&
                mfdActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
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
          drawFplDashRow(r, filledIdentX, cy, kFplDashCount, rowSize,
                         colors::kPopoutCyan, showSelection, blinkOn);
          break;
        case FplSectionRow::Kind::EnrouteBlank: {
          const bool active = fplSectionRowIsActiveDisplay(
              sr, fplActiveLayoutLegCount, fplActiveLayoutDestFilled,
              activeLegIdx);
          const bool showActive = active && activeHighlight;
          drawFplDashRow(
              r, filledIdentX, cy, kFplDashCount, rowSize,
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
              mfdActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
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
                mfdActiveNavRowBlink(sr.legIndex, activeLegIdx, cursorLegIdx,
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
          drawFplDashRow(r, filledIdentX, cy, kFplDashCount, rowSize,
                         colors::kPopoutCyan, showSelection, blinkOn);
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
    // FPA / V DEV labels start ~0.56 across with their values right-aligned at
    // the box edge. TOD (top row) is indented further right to match the
    // trainer, where it sits closer to its value than FPA/V DEV do.
    const Rect right{inner.x + inner.w * 0.56f, inner.y, inner.w * 0.44f,
                     inner.h};
    const Rect todRow{inner.x + inner.w * 0.72f, inner.y, inner.w * 0.28f,
                      inner.h};
    const VnvProfile& vnv = d.vnv;
    char vb[24];

    std::string vsTgt = "_ _ _ _ _FPM";
    std::string vsReq = "_ _ _ _ _FPM";
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
    std::string vdev = "_ _ _ _ _FT";
    if (vnv.active && vnv.capturing) {
      std::snprintf(vb, sizeof(vb), "%+dFT",
                    static_cast<int>(std::lround(vnv.verticalDeviationFt)));
      vdev = vb;
    }

    float ly = left.y;
    // WPT row: gray label, then the target fix rendered like the flight-plan
    // list above -- ident, the small procedure role (iaf/faf/...) when this fix
    // is a procedure leg, and the altitude constraint with a smaller "FT" unit.
    // Drawn whitesmoke (not cyan) to match the trainer, with extra spacing
    // ahead of the altitude.
    {
      const float cy = ly + rowH * 0.5f;
      r.fillText(left.x, cy, "WPT", mfdFontPx(kWtFieldLabel, displayH),
                 TextAlign::Left, colors::kTitleGray);
      // The waypoint + altitude span a wider field than the VS TGT/REQ values,
      // so right-align it further across (matching the trainer) for clear
      // spacing between the ident/role and the altitude.
      const float xR = inner.x + inner.w * 0.66f;
      const float valSize = mfdFontPx(kWtVnvValue, displayH);
      if (vnv.active && !vnv.targetWpt.empty()) {
        std::string role;
        for (const MapLeg& leg : plan) {
          if (leg.id == vnv.targetWpt) {
            role = fplApproachLegRole(leg, ui.fplApproachTransition());
            break;
          }
        }
        const float roleSize = valSize * 0.72f;
        const float unitSize = valSize * kUnitEm;
        std::snprintf(vb, sizeof(vb), "%d", vnv.targetAltFt);
        const std::string altNum = vb;
        const float identW = r.measureTextWidth(vnv.targetWpt, valSize);
        const float roleGap = role.empty() ? 0.0f : valSize * 0.20f;
        const float roleW =
            role.empty() ? 0.0f : r.measureTextWidth(role, roleSize);
        const float altGap = valSize * 0.55f;
        const float altW = r.measureTextWidth(altNum, valSize);
        const float unitW = r.measureTextWidth("FT", unitSize);
        float tx =
            xR - (identW + roleGap + roleW + altGap + altW + unitW);
        r.fillText(tx, cy, vnv.targetWpt, valSize, TextAlign::Left,
                   colors::kWhitesmoke);
        tx += identW;
        if (!role.empty()) {
          tx += roleGap;
          r.fillText(tx, cy, role, roleSize, TextAlign::Left,
                     colors::kWhitesmoke);
          tx += roleW;
        }
        tx += altGap;
        r.fillText(tx, cy, altNum, valSize, TextAlign::Left,
                   colors::kWhitesmoke);
        tx += altW;
        r.fillText(tx, cy, "FT", unitSize, TextAlign::Left,
                   colors::kWhitesmoke);
      } else {
        // Empty profile: a long spaced ident dash run on the left and the
        // altitude placeholder (spaced dashes + small FT) right-aligned at xR,
        // matching the trainer's two-group layout.
        const float unitSize = valSize * kUnitEm;
        const std::string altDash = "_ _ _ _ _";
        const float unitW = r.measureTextWidth("FT", unitSize);
        const float altW = r.measureTextWidth(altDash, valSize);
        r.fillText(xR, cy, "FT", unitSize, TextAlign::Right,
                   colors::kWhitesmoke);
        r.fillText(xR - unitW, cy, altDash, valSize, TextAlign::Right,
                   colors::kWhitesmoke);
        const float identStart = inner.x + inner.w * 0.16f;
        r.fillText(identStart, cy, "_ _ _ _ _ _ _ _", valSize, TextAlign::Left,
                   colors::kWhitesmoke);
      }
      ly += rowH;
    }
    ly = drawField(r, left, ly, rowH, "VS TGT", vsTgt, displayH,
                   colors::kWhitesmoke, kWtVnvValue);
    ly = drawField(r, left, ly, rowH, "VS REQ", vsReq, displayH,
                   colors::kWhitesmoke, kWtVnvValue);
    float ry = right.y;
    ry = drawField(r, todRow, ry, rowH, "TOD", tod, displayH,
                   colors::kWhitesmoke, kWtVnvValue);
    ry = drawField(r, right, ry, rowH, "FPA", fpa, displayH,
                   colors::kWhitesmoke, kWtVnvValue);
    ry = drawField(r, right, ry, rowH, "V DEV", vdev, displayH,
                   colors::kWhitesmoke, kWtVnvValue);
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
    drawFplEntryWindow(r, d, ui, map, f.panel, displayH);
  } else if (ui.fplConfirm() != MfdController::FplConfirm::None) {
    drawFplConfirmWindow(r, ui, f.panel, displayH);
  }
  // The Active Flight Plan Page Menu (MENU key) renders via the shared
  // drawPageMenu overlay in MultiFunctionDisplay, like the Navigation Map.
}

}  // namespace avionics::mfd
