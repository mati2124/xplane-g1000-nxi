#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/FlightPlanCatalog.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

// FPL - Flight Plan Catalog (the FPL group's 2nd page; NXi trainer screenshot
// 060). Layout: the navigation map on the LEFT, and on the right a wide gray
// panel of three group boxes -- the "Flight Plan Catalog" Used/Empty summary,
// the scrollable "Flight Plan List" of stored plans, and the "Flight Plan Info"
// box for the highlighted slot. Imported SimBrief OFPs are stored here; the
// pilot previews and Activates a stored plan to load it into the active route.
namespace avionics::mfd {

namespace {

// Six cyan underscores per ident endpoint for an empty slot (trainer 060).
// Space-separated so they render as discrete placeholders, not one solid bar --
// matching the empty-field style used elsewhere (e.g. MfdChartsPage "_ _ _ _").
inline constexpr const char* kCatalogEmptyIdent = "_ _ _ _ _ _";

std::string identOrDashes(const std::string& ident) {
  return ident.empty() ? std::string(kCatalogEmptyIdent) : ident;
}

// The OK / CANCEL action-confirmation window (Activate / Invert & Activate /
// Delete / Delete All). Mirrors the active page's confirmation: a centered
// prompt over the right panel with the highlighted choice as a cyan plate.
void drawCatalogConfirmWindow(Renderer& r, const MfdController& ui,
                              const Rect& panel, float displayH) {
  std::string question;
  switch (ui.catalogConfirm()) {
    case MfdController::CatalogConfirm::Activate:
      question = "Activate stored flight plan?";
      break;
    case MfdController::CatalogConfirm::InvertActivate:
      question = "Invert and activate stored flight plan?";
      break;
    case MfdController::CatalogConfirm::Delete:
      question = "Delete flight plan?";
      break;
    case MfdController::CatalogConfirm::DeleteAll:
      question = "Delete all flight plans?";
      break;
    case MfdController::CatalogConfirm::None:
      return;
  }

  auto P = [&](float v) { return mfdFontPx(v, displayH); };
  const float textSize = mfdFontPx(18.0f, displayH);
  const float lineH = textSize * 1.4f;
  const float buttonSize = mfdFontPx(kWtFieldValue, displayH);
  const float buttonRowH = buttonSize * 1.7f;

  const float boxW = mfdFontPx(292.0f, displayH);
  const float dialogPad = P(10.0f);
  const float maxTextW = boxW - 2.0f * dialogPad - P(8.0f);
  const float wrapW = std::max(P(40.0f), maxTextW);

  // Greedy word-wrap so long prompts fit the box.
  std::vector<std::string> lines;
  {
    std::string line;
    std::size_t i = 0;
    while (i < question.size()) {
      const std::size_t sp = question.find(' ', i);
      const std::string word = question.substr(
          i, sp == std::string::npos ? std::string::npos : sp - i);
      const std::string candidate = line.empty() ? word : line + " " + word;
      if (!line.empty() && r.measureTextWidth(candidate, textSize) > wrapW) {
        lines.push_back(line);
        line = word;
      } else {
        line = candidate;
      }
      if (sp == std::string::npos) break;
      i = sp + 1;
    }
    if (!line.empty()) lines.push_back(line);
    if (lines.empty()) lines.push_back(question);
  }

  const float topPad = lineH * 0.6f;
  const float spacer = lineH;
  const float bottomPad = lineH * 0.55f;
  const float contentH = topPad + lineH * static_cast<float>(lines.size()) +
                         spacer + buttonRowH + bottomPad;
  const float boxH = contentH + P(6.0f) + dialogPad;
  const float boxX = panel.x + panel.w - P(3.0f) - boxW;
  const float boxY = panel.y + (panel.h - boxH) * 0.38f;
  Rect inner = drawDialog(r, Rect{boxX, boxY, boxW, boxH}, nullptr, displayH);

  float ty = inner.y + topPad + textSize * 0.5f;
  for (const std::string& line : lines) {
    r.fillText(inner.x + inner.w * 0.5f, ty, line, textSize, TextAlign::Center,
               colors::kWhite);
    ty += lineH;
  }

  // "OK  or  CANCEL": the highlighted choice is the pulsing select cursor.
  const float gap = buttonSize * 0.9f;
  const float okW = r.measureTextWidth("OK", buttonSize);
  const float orW = r.measureTextWidth("or", buttonSize);
  const float cancelW = r.measureTextWidth("CANCEL", buttonSize);
  const float rowW = okW + gap + orW + gap + cancelW;
  const float buttonCy = inner.y + topPad +
                         lineH * static_cast<float>(lines.size()) + spacer +
                         buttonRowH * 0.5f;
  float bx = inner.x + (inner.w - rowW) * 0.5f;
  if (ui.catalogConfirmOk()) {
    drawCursorSelect(r, bx, buttonCy, "OK", buttonSize, TextAlign::Left,
                     ui.blinkOn());
  } else {
    r.fillText(bx, buttonCy, "OK", buttonSize, TextAlign::Left, colors::kWhite);
  }
  bx += okW + gap;
  r.fillText(bx + orW * 0.5f, buttonCy, "or", buttonSize, TextAlign::Center,
             colors::kWhite);
  bx += orW + gap;
  if (!ui.catalogConfirmOk()) {
    drawCursorSelect(r, bx, buttonCy, "CANCEL", buttonSize, TextAlign::Left,
                     ui.blinkOn());
  } else {
    r.fillText(bx, buttonCy, "CANCEL", buttonSize, TextAlign::Left,
               colors::kWhite);
  }
}

}  // namespace

void drawFlightPlanCatalogPage(Renderer& r, const FlightData& d,
                               const MapData& map, MfdController& ui, float x,
                               float y, float w, float h, float displayH) {
  PageFrame f = beginPanelPage(r, x, y, w, h, /*widePanel=*/true);

  // Navigation map on the left, like the Active Flight Plan page (the catalog
  // does not change what is flown, so the map shows the current active route).
  drawPageMap(r, d, map, f.map, ui.rangeNm(), nullptr, displayH, false, nullptr,
              ui.displayRangeNm(), ui.terrainDisplay(), false,
              ui.airwayDisplay(), ui.showWeather());

  const FlightPlanCatalog& catalog = ui.flightPlanCatalog();
  const int used = catalog.size();
  const int emptySlots = kFlightPlanCatalogMaxPlans - used;
  const int selected = ui.catalogSelected();

  PanelStack stack(f.panel, displayH);
  const float summaryWt = 44.0f;
  const float infoWt = 132.0f;

  // ---- Flight Plan Catalog summary box (Used / Empty) ----
  {
    Rect inner = drawGroupBox(r, stack.slot(summaryWt), "Flight Plan Catalog",
                              displayH);
    const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
    const float valueSize = mfdFontPx(kWtFieldValue, displayH);
    const float cy = inner.y + inner.h * 0.5f;
    char buf[16];
    r.fillText(inner.x, cy, "Used", labelSize, TextAlign::Left,
               colors::kTitleGray);
    std::snprintf(buf, sizeof(buf), "%d", used);
    r.fillText(inner.x + mfdFontPx(96.0f, displayH), cy, buf, valueSize,
               TextAlign::Right, colors::kWhite);
    const float emptyLabelX = inner.x + inner.w * 0.52f;
    r.fillText(emptyLabelX, cy, "Empty", labelSize, TextAlign::Left,
               colors::kTitleGray);
    std::snprintf(buf, sizeof(buf), "%d", emptySlots);
    r.fillText(inner.x + inner.w, cy, buf, valueSize, TextAlign::Right,
               colors::kWhite);
  }

  // ---- Flight Plan List (the catalog itself) ----
  {
    Rect inner = drawGroupBox(
        r, stack.slot(stack.remainingWt(infoWt + 20.0f)), "Flight Plan List",
        displayH);
    const float rowPx = mfdFontPx(kWtListRow, displayH);
    const float numSize = mfdFontPx(kWtFieldValue, displayH);
    const float identSize = mfdFontPx(kWtFieldValue, displayH);
    const int visibleRows =
        std::max(1, static_cast<int>(inner.h / rowPx));
    // Always show at least the visible window of slots (empty slots included,
    // like the trainer's dashed rows). Scroll to keep the selection on screen.
    const int totalRows = std::max(visibleRows, used);
    int firstRow = 0;
    if (totalRows > visibleRows) {
      firstRow = std::max(0, std::min(selected - visibleRows / 2,
                                      totalRows - visibleRows));
    }
    const float numColR = inner.x + mfdFontPx(26.0f, displayH);
    const float identX = inner.x + mfdFontPx(40.0f, displayH);
    float rowY = inner.y;
    for (int row = firstRow; row < firstRow + visibleRows; ++row) {
      const float cy = rowY + rowPx * 0.5f;
      const bool isSelected =
          ui.catalogCursorOn() && used > 0 && row == selected;
      if (isSelected) {
        render::drawCursorRowSelect(r, inner.x - mfdFontPx(2.0f, displayH), rowY,
                                    inner.w + mfdFontPx(4.0f, displayH), rowPx,
                                    ui.blinkOn());
      }
      const Color textColor =
          (isSelected && ui.blinkOn()) ? colors::kBlack : colors::kCyan;
      const Color numColor =
          (isSelected && ui.blinkOn()) ? colors::kBlack : colors::kWhite;
      char num[8];
      std::snprintf(num, sizeof(num), "%d", row + 1);
      r.fillText(numColR, cy, num, numSize, TextAlign::Right, numColor);

      std::string orig = kCatalogEmptyIdent;
      std::string dest = kCatalogEmptyIdent;
      if (row < used) {
        orig = identOrDashes(FlightPlanCatalog::originIdent(catalog.plan(row)));
        dest = identOrDashes(FlightPlanCatalog::destIdent(catalog.plan(row)));
      }
      r.fillText(identX, cy, orig + " / " + dest, identSize, TextAlign::Left,
                 textColor);
      rowY += rowPx;
    }
  }

  // ---- Flight Plan Info (highlighted slot) ----
  {
    Rect inner = drawGroupBox(r, stack.slot(infoWt), "Flight Plan Info",
                              displayH);
    const float rowH = mfdFontPx(kWtListRow, displayH);
    const bool hasPlan = used > 0 && selected >= 0 && selected < used;
    std::string departure;
    std::string destination;
    std::string totalDist = kDash;
    if (hasPlan) {
      const PersistedFlightPlan& plan = catalog.plan(selected);
      departure = FlightPlanCatalog::originIdent(plan);
      destination = FlightPlanCatalog::destIdent(plan);
      const double distNm = FlightPlanCatalog::totalDistanceNm(plan);
      if (distNm > 0.0) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%.0f%s", distNm, "NM");
        totalDist = buf;
      }
    }
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "Departure",
                   departure.empty() ? std::string(kDash) : departure, displayH,
                   colors::kCyan);
    fy = drawField(r, inner, fy, rowH, "Destination",
                   destination.empty() ? std::string(kDash) : destination,
                   displayH, colors::kCyan);
    fy = drawField(r, inner, fy, rowH, "Total Distance", totalDist, displayH,
                   colors::kWhitesmoke);
    // Enroute Safe Altitude needs a terrain query this page does not perform,
    // so it is shown dashed (the field is still listed, like the real unit).
    fy = drawField(r, inner, fy, rowH, "Enroute Safe Altitude", kDash, displayH,
                   colors::kWhitesmoke);
    (void)fy;
  }

  if (ui.catalogConfirm() != MfdController::CatalogConfirm::None) {
    drawCatalogConfirmWindow(r, ui, f.panel, displayH);
  }
}

}  // namespace avionics::mfd
