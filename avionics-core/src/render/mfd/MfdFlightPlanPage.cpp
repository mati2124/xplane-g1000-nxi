#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/NavMath.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

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
                          bool blinkOn, float displayH) {
  const float cellSize = mfdFontPx(kWtIdentLarge, displayH);
  // Advance by each glyph's actual width (the render font is proportional, so a
  // fixed-width cell would clip wide letters like 'W' into their neighbours),
  // with a little tracking so the identifier reads as one tight field.
  const float tracking = cellSize * 0.06f;
  float cx = startX;
  for (int i = 0; i < MfdController::kFplEntryMaxChars; ++i) {
    const bool isCursor = i == cursor;
    const bool cursorOn = isCursor && blinkOn;
    const char ch = i < static_cast<int>(ident.size()) ? ident[i] : '_';
    const char text[2] = {ch, '\0'};
    const float chW = r.measureTextWidth(text, cellSize);
    if (cursorOn) {
      r.fillRect(cx - tracking * 0.5f, cy - cellSize * 0.62f, chW + tracking,
                 cellSize * 1.24f, colors::kCyan);
    }
    const Color color = cursorOn ? colors::kBlack
                        : isCursor ? colors::kCyan  // blink-off half pulses cyan
                        : i >= typedCount ? colors::kCyan  // spell-ahead fill
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
                 colors::kCyan);
    }
    r.fillText(cx + cellW * 0.5f, cy, text, cellSize, TextAlign::Center,
               isCursor ? colors::kBlack : colors::kWhite);
    cx += cellW + cellGap;
  }
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
                        const MapData& map, float x, float y, float w, float h,
                        float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  const float rowH = mfdFontPx(kWtFieldValue, displayH) * 1.9f;
  const float boxW = w * 0.40f;
  const float boxH = rowH * 4.6f;
  Rect inner = drawDialog(
      r, Rect{x + (w - boxW) * 0.5f, y + (h - boxH) * 0.38f, boxW, boxH},
      "Waypoint Information", displayH);

  const float cy = inner.y + rowH * 0.8f;
  drawIdentEntryCells(r, inner.x, cy, ui.fplEntryIdent(), ui.fplEntryCursor(),
                      ui.fplEntryTypedCount(), ui.blinkOn(), displayH);
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
  const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
  const float valueSize = mfdFontPx(kWtFieldValue, displayH);
  const float rowSize = mfdFontPx(kWtRow, displayH);

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
    Rect ic = drawGroupBox(r, Rect{inner.x, topY, inner.w, P(104.0f)},
                           "Ident, Facility, City", displayH);
    topY += P(104.0f) + gap;
    const float identSize = mfdFontPx(kWtIdentLarge, displayH);
    const float cy1 = ic.y + identSize * 0.62f;
    const float identEnd = drawIdentEntryCells(
        r, ic.x, cy1, ui.directToIdent(), ui.directToCursor(),
        ui.directToTypedCount(), ui.blinkOn(), displayH);
    if (hasMatch) {
      drawWaypointIcon(r, identEnd + P(16.0f), cy1, P(22.0f), &wpt, wpt.type);
      r.fillText(ic.x + ic.w, cy1, wpt.region.empty() ? kDash : wpt.region,
                 mfdFontPx(16.0f, displayH), TextAlign::Right,
                 colors::kWhitesmoke);
    }
    // Facility name and city rows (Fig 5-45). The feature type stands in for
    // the facility name when the database has none.
    const float cy2 = cy1 + rowSize * 1.5f;
    const float cy3 = cy2 + rowSize * 1.3f;
    if (ui.directToNotFound()) {
      r.fillText(ic.x, cy2, "WAYPOINT NOT FOUND", mfdFontPx(16.0f, displayH),
                 TextAlign::Left, colors::kBandYellow);
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
                 rowSize, TextAlign::Left, colors::kCyan);
      r.fillText(ic.x, cy3,
                 wpt.city.empty() ? std::string(kDash) : clipRow(wpt.city),
                 rowSize, TextAlign::Left, colors::kCyan);
    }
  }
  {
    Rect vc =
        drawGroupBox(r, Rect{inner.x, topY, inner.w, P(58.0f)}, "VNV", displayH);
    topY += P(58.0f) + gap;
    const float cy = vc.y + vc.h * 0.5f;
    drawValueWithUnit(r, vc.x + P(110.0f), cy, "_ _ _ _ _", "FT", valueSize,
                      colors::kWhitesmoke);
    drawValueWithUnit(r, vc.x + vc.w, cy, "+0", "NM", valueSize, colors::kCyan);
  }

  // ---- bottom stack (laid out upward): buttons, Course, Location ----
  // Readout values (BRG/DIS/Course) draw larger than the small field labels,
  // matching the real unit.
  const float readoutSize = mfdFontPx(24.0f, displayH);
  const float buttonsH = P(42.0f);
  const float buttonsY = inner.y + inner.h - buttonsH - P(4.0f);
  const float courseH = P(60.0f);
  const float courseY = buttonsY - P(24.0f) - courseH;  // gap above the buttons
  const float locH = P(60.0f);
  const float locY = courseY - gap - locH;

  // Location box (full width): bearing and distance from present position.
  {
    Rect lc = drawGroupBox(r, Rect{inner.x, locY, inner.w, locH}, "Location",
                           displayH);
    const float cy = lc.y + lc.h * 0.5f;
    r.fillText(lc.x, cy, "BRG", labelSize, TextAlign::Left, colors::kTitleGray);
    std::string brgStr = kDash;
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f", brg);
      brgStr = buf;
    }
    drawValueWithUnit(r, lc.x + lc.w * 0.46f, cy, brgStr, kDeg, readoutSize,
                      colors::kWhitesmoke);
    r.fillText(lc.x + lc.w * 0.52f, cy, "DIS", labelSize, TextAlign::Left,
               colors::kTitleGray);
    std::string disStr = kDash;
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      disStr = buf;
    }
    drawValueWithUnit(r, lc.x + lc.w, cy, disStr, "NM", readoutSize,
                      colors::kWhitesmoke);
  }
  // Course box (about half width): the GPS direct course (cyan), equal to the
  // bearing until the direct-to is activated.
  {
    Rect cc = drawGroupBox(r, Rect{inner.x, courseY, inner.w * 0.52f, courseH},
                           "Course", displayH);
    const float cy = cc.y + cc.h * 0.5f;
    std::string crs = kDash;
    if (haveGeo) {
      std::snprintf(buf, sizeof(buf), "%03.0f", brg);
      crs = buf;
    }
    const float vx = cc.x + P(12.0f);
    r.fillText(vx, cy, crs, readoutSize, TextAlign::Left, colors::kCyan);
    r.fillText(vx + r.measureTextWidth(crs, readoutSize), cy, kDeg,
               readoutSize * kUnitEm, TextAlign::Left, colors::kCyan);
  }

  // ---- Map box fills the middle ----
  {
    const float mapBot = locY - gap;
    Rect mc = drawGroupBox(r, Rect{inner.x, topY, inner.w, mapBot - topY},
                           "Map", displayH);
    float dtoRangeNm = 10.0f;
    if (haveGeo && dis > 0.1) {
      dtoRangeNm = static_cast<float>(std::max(2.0, std::min(250.0, dis * 1.2)));
    }
    const MapFeature* center = hasMatch ? &wpt : nullptr;
    drawPageMap(r, d, map, mc, dtoRangeNm, center, displayH);
  }

  // ---- Activate? / Hold? buttons ----
  // Text-sized rounded-rect buttons with a thin gray outline; Activate? at the
  // left, Hold? at the right. Activate? pulses cyan when armed (~1 Hz).
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

// The FPL page menu (MENU key): its one supported option, Delete Flight Plan.
void drawFplMenuWindow(Renderer& r, const MfdController& ui, float x, float y,
                       float w, float h, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);
  const float rowH = mfdFontPx(kWtFieldValue, displayH) * 1.9f;
  const float boxW = w * 0.40f;
  const float boxH = rowH * 4.0f;
  Rect inner = drawDialog(
      r, Rect{x + (w - boxW) * 0.5f, y + (h - boxH) * 0.38f, boxW, boxH},
      "PAGE MENU", displayH);

  float fy = inner.y;
  r.fillText(inner.x + inner.w * 0.5f, fy + rowH * 0.5f, "OPTIONS",
             mfdFontPx(kWtHeader, displayH), TextAlign::Center,
             colors::kTitleGray);
  fy += rowH;

  // The single option, highlighted (ENT selects, CLR/MENU backs out).
  drawCursorSelect(r, inner.x + inner.w * 0.5f, fy + rowH * 0.5f,
                   "Delete Flight Plan", mfdFontPx(kWtFieldValue, displayH),
                   TextAlign::Center, ui.blinkOn());
  fy += rowH;

  r.fillText(inner.x + inner.w * 0.5f, fy + rowH * 0.5f,
             "Press the FMS CRSR knob to return to base page",
             mfdFontPx(12.0f, displayH), TextAlign::Center,
             colors::kTitleGray);
}

}  // namespace

void drawActiveFlightPlanPage(Renderer& r, const FlightData& d,
                              const MapData& map, const MfdController& ui,
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

  // Route preview map around ownship, wide enough to show the plan.
  float previewRangeNm = 25.0f;
  if (map.positionValid && plan.size() >= 2) {
    double maxNm = 0.0;
    for (const MapLeg& leg : plan) {
      maxNm = std::max(maxNm, navDistanceNm(map.ownshipLat, map.ownshipLon,
                                            leg.lat, leg.lon));
    }
    previewRangeNm =
        std::max(10.0f, std::min(150.0f, static_cast<float>(maxNm) * 1.2f));
  }
  const std::vector<MapLeg> procPreview =
      ui.procMenuOpen() ? ui.procPreviewLegs() : std::vector<MapLeg>{};
  const std::vector<MapLeg>* procPreviewPtr =
      procPreview.empty() ? nullptr : &procPreview;
  drawPageMap(r, d, map, f.map, previewRangeNm, nullptr, displayH, false,
              procPreviewPtr);

  PanelStack stack(f.panel, displayH);
  const float promptWt = 34.0f;
  const float vnvWt = 124.0f;

  // Active Flight Plan box.
  {
    Rect inner = drawGroupBox(
        r, stack.slot(stack.remainingWt(promptWt + vnvWt + 10.0f)),
        "Active Flight Plan", displayH);

    const float headerSize = mfdFontPx(kWtRow, displayH);
    // Origin / destination header (cyan, dashes while the plan is empty).
    const std::string orig = plan.empty() ? "_____" : plan.front().id;
    const std::string dest = plan.size() < 2 ? "_____" : plan.back().id;
    float fy = inner.y;
    r.fillText(inner.x + mfdFontPx(40.0f, displayH), fy + headerSize * 0.6f,
               orig + " / " + dest, headerSize, TextAlign::Left, colors::kCyan);
    fy += headerSize * 1.5f;

    // Column headers + the rule under them (WT .mfd-flightplan-hr).
    const float colHdrSize = mfdFontPx(kWtHeader, displayH);
    const float colDtkR = inner.x + mfdFontPx(210.0f, displayH);
    const float colDisR = inner.x + mfdFontPx(330.0f, displayH);
    const float colAltR = inner.x + inner.w - mfdFontPx(5.0f, displayH);
    r.fillText(inner.x + mfdFontPx(170.0f, displayH), fy + colHdrSize * 0.6f,
               "DTK", colHdrSize, TextAlign::Left, colors::kWhite);
    r.fillText(inner.x + mfdFontPx(250.0f, displayH), fy + colHdrSize * 0.6f,
               "DIS", colHdrSize, TextAlign::Left, colors::kWhite);
    r.fillText(inner.x + mfdFontPx(320.0f, displayH), fy + colHdrSize * 0.6f,
               "ALT", colHdrSize, TextAlign::Left, colors::kWhite);
    fy += colHdrSize * 1.4f;
    r.strokeLine(inner.x, fy, inner.x + inner.w, fy, 1.0f,
                 Color{0.596f, 0.624f, 0.682f, 1.0f});
    fy += mfdFontPx(4.0f, displayH);

    const float rowH = mfdFontPx(32.0f, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    const float fixX = inner.x + mfdFontPx(25.0f, displayH);

    const bool cursorOn = ui.fplCursorOn();
    const int wptCount = static_cast<int>(plan.size());
    // With the cursor on, a blank slot after the last waypoint takes entries
    // that extend the plan (and is how a plan is built from scratch).
    const int totalRows = wptCount + (cursorOn ? 1 : 0);

    if (totalRows == 0) {
      r.fillText(inner.x + inner.w * 0.5f, fy + rowH,
                 "NO ACTIVE FLIGHT PLAN", rowSize, TextAlign::Center,
                 colors::kTitleGray);
    }

    // Scroll to keep the cursor row visible.
    const int maxRows =
        std::max(1, static_cast<int>((inner.y + inner.h - fy) / rowH));
    int start = 0;
    if (cursorOn && totalRows > maxRows) {
      start = ui.fplCursorRow() - maxRows / 2;
      start = std::max(0, std::min(start, totalRows - maxRows));
    }

    char buf[24];
    const int end = std::min(totalRows, start + maxRows);
    for (int row = start; row < end; ++row) {
      const float cy = fy + rowH * 0.5f;
      const bool isCursor = cursorOn && row == ui.fplCursorRow();

      if (row == wptCount) {
        // The blank append slot.
        if (isCursor) {
          drawCursorSelect(r, fixX, cy, "_____", rowSize, TextAlign::Left,
                           ui.blinkOn());
        } else {
          r.fillText(fixX, cy, "_____", rowSize, TextAlign::Left,
                     colors::kTitleGray);
        }
        fy += rowH;
        continue;
      }

      const MapLeg& leg = plan[row];
      const bool active = !d.fmaToWpt.empty() && leg.id == d.fmaToWpt;
      // Active leg: the whole row magenta with the leg arrow ahead of the
      // ident (WT .active-wpt + FplActiveLegArrow).
      const Color rowColor = active ? colors::kMagenta : colors::kWhitesmoke;
      if (active) {
        const float ax = inner.x;
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
      // The cursor highlights either the identifier or the ALT field (the
      // large knob steps between them); the small knob edits the highlighted
      // one (Pilot's Guide, Section 6 "Vertical Navigation").
      const bool identSel =
          isCursor && ui.fplCursorCol() == MfdController::FplCursorCol::Ident;
      const bool altSel =
          isCursor && ui.fplCursorCol() == MfdController::FplCursorCol::Altitude;
      if (identSel) {
        drawCursorSelect(r, fixX, cy, leg.id, rowSize, TextAlign::Left,
                         ui.blinkOn());
      } else {
        r.fillText(fixX, cy, leg.id, rowSize, TextAlign::Left,
                   active ? colors::kMagenta : colors::kCyan);
      }
      if (row > 0) {
        const MapLeg& prev = plan[row - 1];
        const double dtk =
            navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
        const double dis = navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
        std::snprintf(buf, sizeof(buf), "%03.0f", dtk);
        drawValueWithUnit(r, colDtkR, cy, buf, kDeg, rowSize, rowColor);
        std::snprintf(buf, sizeof(buf), "%.1f", dis);
        drawValueWithUnit(r, colDisR, cy, buf, "NM", rowSize, rowColor);
      }
      // ALT column: a manually entered (designated) constraint draws cyan, a
      // procedure constraint white, and an unconstrained leg shows dashes.
      if (ui.fplAltEntryActive() && ui.fplAltEntryRow() == row) {
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

  // Bottom prompt (WT .mfd-fpl-bottom-prompt, exact text).
  r.fillText(f.panel.x + f.panel.w * 0.5f,
             f.panel.y + f.panel.h - mfdFontPx(20.0f, displayH),
             "Press the \"FPL\" key to view the previous page",
             mfdFontPx(16.0f, displayH), TextAlign::Center,
             colors::kWhitesmoke);

  // Editing overlays above the page content.
  if (ui.fplEntryActive()) {
    drawFplEntryWindow(r, ui, map, x, y, w, h, displayH);
  } else if (ui.fplConfirm() != MfdController::FplConfirm::None) {
    drawFplConfirmWindow(r, ui, x, y, w, h, displayH);
  } else if (ui.fplMenuOpen()) {
    drawFplMenuWindow(r, ui, x, y, w, h, displayH);
  } else if (ui.procMenuOpen()) {
    const FontScope fs(r, FontFace::DejaVuSemiBold);
    const float rowH = mfdFontPx(kWtFieldValue, displayH) * 1.9f;
    const float boxW = w * 0.48f;
    const float boxH = rowH * 7.0f;
    const ProcedureType cat = ui.procCategory();
    const bool pickingTransition =
        ui.procStep() == MfdController::ProcMenuStep::TransitionList;
    const char* title =
        pickingTransition ? "Select Transition"
        : cat == ProcedureType::Departure ? "Select Departure"
        : cat == ProcedureType::Arrival   ? "Select Arrival"
                                          : "Select Approach";
    Rect inner = drawDialog(
        r, Rect{x + (w - boxW) * 0.5f, y + (h - boxH) * 0.32f, boxW, boxH},
        title, displayH);

    const float tabSize = mfdFontPx(16.0f, displayH);
    const float tabY = inner.y + tabSize * 0.55f;
    if (!pickingTransition) {
      struct Tab {
        const char* label;
        ProcedureType type;
      };
      const Tab tabs[3] = {{"DEP", ProcedureType::Departure},
                           {"ARR", ProcedureType::Arrival},
                           {"APP", ProcedureType::Approach}};
      float tabX = inner.x;
      for (const Tab& tab : tabs) {
        const bool active = tab.type == cat;
        r.fillText(tabX, tabY, tab.label, tabSize, TextAlign::Left,
                   active ? colors::kCyan : colors::kTitleGray);
        tabX += r.measureTextWidth(tab.label, tabSize) + tabSize * 1.2f;
      }
    } else {
      r.fillText(inner.x, tabY, ui.procSelectedName().c_str(), tabSize,
                 TextAlign::Left, colors::kCyan);
    }

    char icaoBuf[16];
    std::snprintf(icaoBuf, sizeof(icaoBuf), "%s", ui.procAirportIcao().c_str());
    r.fillText(inner.x + inner.w, tabY, icaoBuf, tabSize, TextAlign::Right,
               colors::kWhitesmoke);

    const std::vector<std::string> rows =
        pickingTransition ? ui.procTransitions(cat, ui.procSelectedName())
                          : ui.procProcedureNames(cat);
    const float listRowH = mfdFontPx(kWtListRow, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    float yy = inner.y + listRowH * 1.15f;
    if (rows.empty()) {
      r.fillText(inner.x + inner.w * 0.5f, yy + listRowH * 0.5f,
                 "NO PROCEDURES", rowSize, TextAlign::Center,
                 colors::kTitleGray);
    } else {
      const int fit = std::max(
          1, static_cast<int>((inner.h - (yy - inner.y)) / listRowH));
      const int count = std::min(fit, static_cast<int>(rows.size()));
      for (int i = 0; i < count; ++i) {
        const float cy = yy + listRowH * 0.5f;
        const std::string& row = rows[static_cast<std::size_t>(i)];
        if (i == ui.procSelected()) {
          drawCursorSelect(r, inner.x, cy, row, rowSize, TextAlign::Left,
                           ui.blinkOn());
        } else {
          r.fillText(inner.x, cy, row, rowSize, TextAlign::Left,
                     colors::kCyan);
        }
        yy += listRowH;
      }
    }
  }
}

}  // namespace avionics::mfd
