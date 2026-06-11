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

}

namespace {

// Small checkbox with a green check when done, matching the NXi checklist item
// marker. Drawn left of the item description.
void drawCheckbox(Renderer& r, float x, float cy, float size, bool checked) {
  const Point box[5] = {{x, cy - size * 0.5f},
                        {x + size, cy - size * 0.5f},
                        {x + size, cy + size * 0.5f},
                        {x, cy + size * 0.5f},
                        {x, cy - size * 0.5f}};
  r.strokePolyline(box, 5, 1.2f, colors::kTitleGray);
  if (checked) {
    r.strokeLine(x + size * 0.18f, cy + size * 0.02f, x + size * 0.42f,
                 cy + size * 0.30f, 2.0f, colors::kActiveGreen);

}  // namespace

}

}  // namespace

void drawChecklistPage(Renderer& r, const ChecklistData& checklist,
                       const MfdController& ui, float x, float y, float w,
                       float h, float displayH) {
  // Checklist page: a single group box on the gray page, with the pulsing
  // highlight-select item cursor.
  r.fillRect(x, y, w, h, colors::kMfdPanelGray);
  const float pad = mfdFontPx(10.0f, displayH);
  const Rect box{x + pad * 2.0f, y + pad, w - pad * 4.0f, h - 2.0f * pad};

  const int total = checklist.totalChecklists();
  const std::string* groupName = nullptr;
  const Checklist* cl =
      total > 0 ? checklist.at(ui.checklistIndex(), &groupName) : nullptr;

  if (cl == nullptr) {
    Rect inner = drawGroupBox(r, box, "Checklist", displayH);
    r.fillText(inner.x + inner.w * 0.5f, inner.y + inner.h * 0.4f,
               "NO CHECKLIST AVAILABLE", mfdFontPx(kWtRow, displayH),
               TextAlign::Center, colors::kTitleGray);
    r.fillText(inner.x + inner.w * 0.5f,
               inner.y + inner.h * 0.4f + mfdFontPx(kWtRow, displayH) * 1.6f,
               "ADD A CHECKLIST FILE FOR THIS AIRCRAFT",
               mfdFontPx(kWtHeader, displayH), TextAlign::Center,
               colors::kTitleGray);
    return;
  }

  Rect inner = drawGroupBox(r, box, cl->title.c_str(), displayH);

  const float rowSize = mfdFontPx(kWtRow, displayH);
  const float headerSize = mfdFontPx(16.0f, displayH);
  const float rowH = mfdFontPx(32.0f, displayH);

  // Header row: owning group name (left) and the "n of m" position (right).
  float fy = inner.y;
  if (groupName != nullptr && !groupName->empty()) {
    r.fillText(inner.x, fy + rowH * 0.5f, *groupName, headerSize,
               TextAlign::Left, colors::kCyan);
  }
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Checklist %d of %d",
                  ui.checklistIndex() + 1, total);
    r.fillText(inner.x + inner.w, fy + rowH * 0.5f, buf, headerSize,
               TextAlign::Right, colors::kTitleGray);
  }
  fy += rowH;

  // The list area below the header holds the items plus the trailing
  // "go to next checklist?" prompt, scrolling to keep the cursor visible.
  const float listTop = fy;
  const float listH = inner.y + inner.h - listTop;
  const int itemCount = static_cast<int>(cl->items.size());
  const int totalRows = itemCount + 1;  // +1 for the prompt line
  const int visibleRows = std::max(1, static_cast<int>(listH / rowH));
  const int cursor = ui.checklistCursor();
  int start = 0;
  if (totalRows > visibleRows) {
    start = cursor - visibleRows / 2;
    start = std::max(0, std::min(start, totalRows - visibleRows));
  }
  const int end = std::min(totalRows, start + visibleRows);

  const float checkSize = rowSize * 0.85f;
  const float textX = inner.x + checkSize * 2.0f;
  for (int row = start; row < end; ++row) {
    const float ry = listTop + (row - start) * rowH;
    const float cy = ry + rowH * 0.5f;
    const bool isCursor = row == cursor;
    const bool cursorOn = isCursor && ui.blinkOn();
    if (isCursor) {
      render::drawCursorRowSelect(r, inner.x - inner.w * 0.01f, ry,
                                  inner.w * 1.02f, rowH, ui.blinkOn());
    }
    const Color textColor =
        cursorOn ? colors::kBlack : (isCursor ? colors::kCyan : colors::kWhitesmoke);

    if (row == itemCount) {
      // Trailing prompt below the last item.
      r.fillText(inner.x, cy, "Go to next checklist?", rowSize,
                 TextAlign::Left,
                 cursorOn ? colors::kBlack
                          : (isCursor ? colors::kCyan : colors::kActiveGreen));
      continue;
    }

    const ChecklistItem& item = cl->items[static_cast<std::size_t>(row)];
    const bool checked = ui.checklistItemChecked(ui.checklistIndex(), row);
    drawCheckbox(r, inner.x, cy, checkSize, checked && !cursorOn);
    if (cursorOn && checked) {
      // On the highlight the green check is hard to read; show a dark tick.
      r.strokeLine(inner.x + checkSize * 0.18f, cy + checkSize * 0.02f,
                   inner.x + checkSize * 0.42f, cy + checkSize * 0.30f, 2.0f,
                   colors::kBlack);
      r.strokeLine(inner.x + checkSize * 0.42f, cy + checkSize * 0.30f,
                   inner.x + checkSize * 0.82f, cy - checkSize * 0.32f, 2.0f,
                   colors::kBlack);
    }
    r.fillText(textX, cy, item.text, rowSize, TextAlign::Left, textColor);
    if (!item.response.empty()) {
      r.fillText(inner.x + inner.w, cy, item.response, rowSize,
                 TextAlign::Right, textColor);
    }
  }
}

}  // namespace avionics::mfd
