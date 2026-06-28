#include <algorithm>
#include <string>
#include <vector>

#include "render/pfd/ChromeInternal.h"

// FPL - Select Airway window (PFD FPL page MENU -> Load Airway, trainer "Select
// Airway"). The compact PFD popout: a fixed Entry fix, an Airway field, an Exit
// field, and a Load? button. The active field shows a steady cyan plate; the
// Airway and Exit fields open a small dropdown list while highlighted (trainer
// Q109/Q118 and the fix chain). Unlike the MFD window there is no persistent fix
// list or DTK/DIS column.
namespace avionics::pfd {
namespace {

using LoadAirwayField = SoftkeyController::LoadAirwayField;

// Visible rows in the Exit dropdown (matches the trainer's seven-row list box).
constexpr int kExitDropdownRows = 7;

// One label + value row. The value plate is steady cyan while `selected`.
void drawSelectAirwayRow(Renderer& r, float labelX, float valueX, float cy,
                         const char* label, const std::string& value,
                         float labelSize, float valueSize, bool selected,
                         float a) {
  r.fillText(labelX, cy, label, labelSize, TextAlign::Left,
             withAlpha(colors::kTitleGray, a));
  if (value.empty()) return;
  if (selected) {
    const float tracking = valueSize * 0.18f;
    const float tw = r.measureTextWidth(value.c_str(), valueSize);
    r.fillRect(valueX - tracking * 0.5f, cy - valueSize * 0.62f, tw + tracking,
               valueSize * 1.24f, withAlpha(colors::kPopoutCyan, a));
    r.fillText(valueX, cy, value, valueSize, TextAlign::Left,
               withAlpha(colors::kBlack, a));
  } else {
    r.fillText(valueX, cy, value, valueSize, TextAlign::Left,
               withAlpha(colors::kPopoutCyan, a));
  }
}

// Dropdown list anchored under a field value, showing `items` with `selIndex`
// highlighted. Entries flagged in `greyed` (the fixed entry fix) read dim.
void drawDropdown(Renderer& r, float x, float top, float valueSize,
                  const std::vector<std::string>& items, int selIndex,
                  int greyedIndex, float a) {
  if (items.empty()) return;
  const int n = static_cast<int>(items.size());
  const int visible = std::min(n, kExitDropdownRows);
  int first = 0;
  if (n > visible) {
    first = std::max(0, std::min(selIndex - visible / 2, n - visible));
  }
  const float rowH = valueSize * 1.5f;
  float maxW = 0.0f;
  for (const std::string& it : items) {
    maxW = std::max(maxW, r.measureTextWidth(it.c_str(), valueSize));
  }
  const float padX = valueSize * 0.5f;
  const float boxW = maxW + 2.0f * padX;
  const float boxH = rowH * static_cast<float>(visible) + valueSize * 0.4f;
  r.fillRect(x, top, boxW, boxH, withAlpha(colors::kMfdPanelGray, a));
  r.strokeRoundedRect(x, top, boxW, boxH, 0.0f, 1.5f,
                      withAlpha(colors::kMenuBorderGray, a));
  for (int i = 0; i < visible; ++i) {
    const int idx = first + i;
    const float cy = top + valueSize * 0.2f + rowH * (static_cast<float>(i) + 0.5f);
    const std::string& it = items[static_cast<std::size_t>(idx)];
    if (idx == selIndex) {
      const float tw = r.measureTextWidth(it.c_str(), valueSize);
      r.fillRect(x + padX - valueSize * 0.15f, cy - valueSize * 0.62f,
                 tw + valueSize * 0.3f, valueSize * 1.24f,
                 withAlpha(colors::kPopoutCyan, a));
      r.fillText(x + padX, cy, it, valueSize, TextAlign::Left,
                 withAlpha(colors::kBlack, a));
    } else {
      const Color c =
          idx == greyedIndex ? colors::kTitleGray : colors::kPopoutCyan;
      r.fillText(x + padX, cy, it, valueSize, TextAlign::Left, withAlpha(c, a));
    }
  }
}

void drawLoadButton(Renderer& r, float cx, float cy, float size, bool selected,
                    bool blinkOn, float a) {
  const char* label = "Load?";
  const float bw = r.measureTextWidth(label, size) + size * 1.4f;
  const float bh = size * 1.7f;
  const float bx = cx - bw * 0.5f;
  const float by = cy - bh * 0.5f;
  const float radius = bh * 0.32f;
  if (selected && blinkOn) {
    r.fillRoundedRect(bx, by, bw, bh, radius, withAlpha(colors::kPopoutCyan, a));
  }
  r.strokeRoundedRect(bx, by, bw, bh, radius, 1.5f,
                      withAlpha(selected ? colors::kWhite : colors::kGroupBoxBorder,
                                a));
  const Color textColor = selected
                              ? (blinkOn ? colors::kBlack : colors::kPopoutCyan)
                              : colors::kWhite;
  r.fillText(cx, cy, label, size, TextAlign::Center, withAlpha(textColor, a));
}

}  // namespace

void drawSelectAirwayWindow(Renderer& r, float w, float h, const Layout& L,
                            const SoftkeyController& ui) {
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f = drawWindowFrame(r, w, h, L, ui.loadAirwayWindowAnim(),
                                        "Select Airway", panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const float labelSize = fontPx(16.0f, h);
  const float valueSize = fontPx(18.0f, h);
  const float pad = f.w * 0.06f;
  const float labelX = f.x + pad;
  const float valueX = f.x + f.w * 0.42f;
  const LoadAirwayField field = ui.loadAirwayField();

  const float rowStep = fontPx(34.0f, h);
  const float row0 = f.contentTop + fontPx(20.0f, h);
  const float entryCy = row0;
  const float airwayCy = row0 + rowStep;
  const float exitCy = row0 + rowStep * 2.0f;

  drawSelectAirwayRow(r, labelX, valueX, entryCy, "Entry",
                      ui.loadAirwayEntryIdent(), labelSize, valueSize, false, a);
  drawSelectAirwayRow(r, labelX, valueX, airwayCy, "Airway",
                      ui.loadAirwayName(), labelSize, valueSize,
                      field == LoadAirwayField::Airway, a);
  drawSelectAirwayRow(r, labelX, valueX, exitCy, "Exit", ui.loadAirwayExitIdent(),
                      labelSize, valueSize, field == LoadAirwayField::Exit, a);

  const float buttonCy = f.top + panelH - fontPx(22.0f, h);
  drawLoadButton(r, f.x + f.w * 0.5f, buttonCy, valueSize,
                 field == LoadAirwayField::Load, ui.blinkOn(), a);

  // Dropdowns draw last so they overlay the rows below them.
  if (field == LoadAirwayField::Airway && ui.loadAirwayAirways().size() > 1) {
    drawDropdown(r, valueX, airwayCy + valueSize * 0.7f, valueSize,
                 ui.loadAirwayAirways(), ui.loadAirwayAirwaySel(), -1, a);
  } else if (field == LoadAirwayField::Exit && ui.loadAirwayFixes().size() > 1) {
    std::vector<std::string> idents;
    idents.reserve(ui.loadAirwayFixes().size());
    for (const MapLeg& fix : ui.loadAirwayFixes()) idents.push_back(fix.id);
    drawDropdown(r, valueX, exitCy + valueSize * 0.7f, valueSize, idents,
                 ui.loadAirwayExitSel(), /*greyedIndex=*/0, a);
  }
}

}  // namespace avionics::pfd
