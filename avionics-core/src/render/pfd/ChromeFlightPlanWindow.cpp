#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/NavMath.h"
#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {
namespace {

// Draws the waypoint-ident entry cells on a leg row while an insertion is in
// progress (shared look with the Direct-To window): the cursor cell pulses as a
// cyan highlight-select plate with black text, the spell-ahead fill is cyan,
// and the typed characters are white. Returns the x just past the last cell.
float drawFplIdentCells(Renderer& r, float startX, float cy,
                        const std::string& ident, int cursor, int typedCount,
                        bool blinkOn, float size, float a) {
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
                 size * 1.24f, withAlpha(colors::kPopoutCyan, a));
    }
    const Color color = cursorOn          ? colors::kBlack
                        : isCursor        ? colors::kPopoutCyan
                        : i >= typedCount ? colors::kPopoutCyan
                                          : colors::kWhite;
    r.fillText(cx, cy, text, size, TextAlign::Left, withAlpha(color, a));
    cx += chW + tracking;
  }
  return cx;
}

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

}  // namespace

// Active Flight Plan window (FPL bezel key): the active flight-plan legs shown
// as the same lower-right popout the other PFD windows use (Pilot's Guide
// Fig. 5-48 "Active Flight Plan Window on PFD"). The header carries the origin /
// destination idents; each leg row shows the waypoint ident (cyan, magenta on
// the active leg) with the desired track (DTK) and leg distance (DIS) right-
// aligned in their columns. With the FMS cursor off the knob scrolls; pushing
// it turns the cursor on so the pilot can insert (small knob) or remove (CLR)
// waypoints, which is how the origin and destination are changed.
void drawFlightPlanWindow(Renderer& r, float w, float h, const Layout& L,
                          const SoftkeyController& ui) {
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
  const bool entryActive = ui.flightPlanEntryActive();
  const bool blinkOn = ui.blinkOn();
  const float size = fontPx(wt::kInfoValue, h);
  const float smallSize = size * 0.72f;  // smaller "NM"/unit affix
  const float padX = fontPx(8.0f, h);
  const float scrollW = fontPx(10.0f, h);
  const float listLeft = f.x + padX;
  const float listRight = f.x + f.w - padX;
  const float listW = listRight - listLeft;

  // Column geometry mirrors the Working Title PFD .fpl layout: ident column on
  // the left, the DTK value right edge at ~0.72, the DIS value at the list edge.
  const float identX = listLeft + listW * 0.04f;
  const float dtkRight = listLeft + listW * 0.72f;
  const float disRight = listRight;

  // Origin / destination header (cyan, dashes while the plan is empty).
  const std::string orig = legs.empty() ? "_____" : legs.front().id;
  const std::string dest = legs.size() < 2 ? "_____" : legs.back().id;
  float cy = f.contentTop + size * 0.6f;
  r.fillText(identX, cy, orig + " / " + dest, size, TextAlign::Left,
             withAlpha(colors::kPopoutCyan, a));
  cy += size * 1.35f;

  // Column header row (DTK / DIS), right-aligned over their value columns, with
  // a separator rule beneath (WT .mfd-flightplan-hr).
  r.fillText(dtkRight, cy, "DTK", smallSize, TextAlign::Right,
             withAlpha(colors::kWhite, a));
  r.fillText(disRight, cy, "DIS", smallSize, TextAlign::Right,
             withAlpha(colors::kWhite, a));
  const float sepY = cy + size * 0.35f;
  r.strokeLine(listLeft, sepY, listRight, sepY, 1.0f,
               withAlpha(colors::kPanelSeparator, a));
  cy = sepY + size * 0.3f;

  const int legCount = static_cast<int>(legs.size());
  const int cursorPos =
      std::max(0, std::min(legCount, ui.flightPlanCursor()));

  // Build the display rows. A waypoint-ident entry inserts a new row before the
  // cursor leg (the rest shift down, like the real unit's Waypoint Information
  // insertion); otherwise a blank append slot trails the list while the cursor
  // is on, so the pilot can add a new destination at the end.
  struct DisplayRow {
    enum class Kind { Leg, Entry, Append } kind;
    int legIndex;  // valid for Leg rows
  };
  std::vector<DisplayRow> rows;
  rows.reserve(static_cast<std::size_t>(legCount) + 1);
  int cursorRowIndex = 0;
  for (int i = 0; i < legCount; ++i) {
    if (entryActive && i == cursorPos) {
      cursorRowIndex = static_cast<int>(rows.size());
      rows.push_back({DisplayRow::Kind::Entry, -1});
    }
    if (!entryActive && cursorOn && i == cursorPos) {
      cursorRowIndex = static_cast<int>(rows.size());
    }
    rows.push_back({DisplayRow::Kind::Leg, i});
  }
  if (entryActive && cursorPos >= legCount) {
    cursorRowIndex = static_cast<int>(rows.size());
    rows.push_back({DisplayRow::Kind::Entry, -1});
  } else if (cursorOn && !entryActive) {
    if (cursorPos >= legCount) cursorRowIndex = static_cast<int>(rows.size());
    rows.push_back({DisplayRow::Kind::Append, -1});
  }

  const int rowCount = static_cast<int>(rows.size());
  if (rowCount == 0) {
    r.fillText(f.x + f.w * 0.5f, f.top + f.h * 0.62f, "NO ACTIVE FLIGHT PLAN",
               smallSize, TextAlign::Center, withAlpha(colors::kTitleGray, a));
    return;
  }

  const float rowH = fontPx(29.0f, h);
  const int visible = std::max(1, static_cast<int>((f.top + f.h - cy) / rowH));
  int first = 0;
  if (rowCount > visible) {
    first = std::max(0,
                     std::min(cursorRowIndex - visible / 2, rowCount - visible));
  }
  const bool scrolling = rowCount > visible;
  const float rowRight =
      scrolling ? disRight - scrollW - fontPx(4.0f, h) : disRight;

  char buf[24];
  const int end = std::min(rowCount, first + visible);
  for (int idx = first; idx < end; ++idx) {
    const DisplayRow& row = rows[idx];
    const float rowCy = cy + rowH * (static_cast<float>(idx - first) + 0.5f);

    // Entry row: spell cells with the resolved facility name to the right.
    if (row.kind == DisplayRow::Kind::Entry) {
      const float identEnd =
          drawFplIdentCells(r, identX, rowCy, ui.flightPlanEntryIdent(),
                            ui.flightPlanEntryCursor(),
                            ui.flightPlanEntryTypedCount(), blinkOn, size, a);
      if (ui.flightPlanEntryNotFound()) {
        r.fillText(identEnd + size * 0.6f, rowCy, "INVALID", smallSize,
                   TextAlign::Left, withAlpha(colors::kBandYellow, a));
      } else if (ui.flightPlanEntryHasMatch()) {
        const MapFeature& wpt = ui.flightPlanEntryMatch();
        if (!wpt.name.empty()) {
          r.fillText(identEnd + size * 0.6f, rowCy, wpt.name, smallSize,
                     TextAlign::Left, withAlpha(colors::kPopoutCyan, a));
        }
      }
      continue;
    }

    const bool isAppend = row.kind == DisplayRow::Kind::Append;
    const bool isCursorRow = cursorOn && !entryActive && idx == cursorRowIndex;
    const std::string ident = isAppend ? "_____" : legs[row.legIndex].id;
    const bool active = !isAppend && !ui.activeWaypointId().empty() &&
                        legs[row.legIndex].id == ui.activeWaypointId();

    if (isCursorRow) {
      const float tracking = size * 0.06f;
      const float tw = r.measureTextWidth(ident.c_str(), size);
      r.fillRect(identX - tracking * 0.5f, rowCy - size * 0.62f, tw + tracking,
                 size * 1.24f, withAlpha(colors::kPopoutCyan, a));
      r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                 withAlpha(colors::kBlack, a));
    } else {
      r.fillText(identX, rowCy, ident, size, TextAlign::Left,
                 withAlpha(active ? colors::kMagenta : colors::kPopoutCyan, a));
    }

    // DTK / DIS are leg-to-leg values, so the origin row and append slot none.
    if (!isAppend && row.legIndex > 0) {
      const MapLeg& leg = legs[row.legIndex];
      const MapLeg& prev = legs[row.legIndex - 1];
      const Color rowColor = active ? colors::kMagenta : colors::kWhitesmoke;
      const double dtk = navBearingDeg(prev.lat, prev.lon, leg.lat, leg.lon);
      const double dis = navDistanceNm(prev.lat, prev.lon, leg.lat, leg.lon);
      std::snprintf(buf, sizeof(buf), "%03.0f\u00b0", dtk);
      r.fillText(dtkRight, rowCy, buf, size, TextAlign::Right,
                 withAlpha(rowColor, a));
      std::snprintf(buf, sizeof(buf), "%.1f", dis);
      const float numW = r.measureTextWidth(buf, size);
      const float nmW = r.measureTextWidth("NM", smallSize);
      const float gap = size * 0.06f;
      const float numLeft = rowRight - nmW - gap - numW;
      r.fillText(numLeft, rowCy, buf, size, TextAlign::Left,
                 withAlpha(rowColor, a));
      r.fillText(numLeft + numW + gap, rowCy, "NM", smallSize, TextAlign::Left,
                 withAlpha(rowColor, a));
    }
  }

  // Scroll thumb when the list is longer than the window (WT ScrollBar).
  if (scrolling) {
    const float trackTop = cy;
    const float trackH = rowH * static_cast<float>(visible);
    const float trackX = f.x + f.w - padX - scrollW;
    const float thumbH = std::max(fontPx(18.0f, h),
                                  trackH * static_cast<float>(visible) /
                                      static_cast<float>(rowCount));
    const float maxScroll = static_cast<float>(rowCount - visible);
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
