#include "avionics/render/MultiFunctionDisplay.h"

#include <algorithm>
#include <cstdio>

#include "avionics/Color.h"
#include "avionics/render/MapView.h"

namespace avionics {
namespace {

// The MFD shares the PFD's 1024x768 GDU canvas, so chrome heights and fonts are
// expressed against it and scaled to the live display.
constexpr float kCanvasHeight = 768.0f;
constexpr float kTopBarHeightPx = 34.0f;
constexpr float kBottomBarHeightPx = 35.0f;

constexpr float kTitleFontWt = 22.0f;
constexpr float kInfoLabelWt = 16.0f;
constexpr float kInfoValueWt = 20.0f;
constexpr float kSoftkeyFontWt = 17.0f;
constexpr float kPageTitleFontWt = 30.0f;

float fontPx(float wtPx, float displayH) {
  return wtPx * (displayH / kCanvasHeight);
}

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

const char* pageGroupTitle(MfdPageGroup group) {
  switch (group) {
    case MfdPageGroup::Map:
      return "NAVIGATION MAP";
    case MfdPageGroup::Waypoint:
      return "WAYPOINT";
    case MfdPageGroup::Aux:
      return "AUX - SYSTEM";
    case MfdPageGroup::Nearest:
      return "NEAREST";
  }
  return "";
}

// Top status strip: page-group title on the left, ground speed / true airspeed
// on the right, over the same dark blue-grey gradient as the PFD NAV/COM bar.
void drawTopBar(Renderer& r, float w, float h, float barH,
                const FlightData& d, const MfdController& ui) {
  r.fillRectVerticalGradient(0.0f, 0.0f, w, barH, 0.0f, barH,
                             colors::kPanelBackground,
                             colors::kPanelBackgroundBottom);
  r.strokeLine(0.0f, barH, w, barH, 2.0f, colors::kPanelBorder);

  const float cy = barH * 0.5f;
  r.fillText(w * 0.012f, cy, pageGroupTitle(ui.pageGroup()),
             fontPx(kTitleFontWt, h), TextAlign::Left, colors::kWhite);

  const float labelSize = fontPx(kInfoLabelWt, h);
  const float valueSize = fontPx(kInfoValueWt, h);
  char buf[16];

  std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(d.groundSpeedKts));
  r.fillText(w * 0.74f, cy, "GS", labelSize, TextAlign::Left, colors::kLabelText);
  r.fillText(w * 0.83f, cy, std::string(buf) + "KT", valueSize, TextAlign::Right,
             colors::kWhite);

  std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(d.tasKts));
  r.fillText(w * 0.86f, cy, "TAS", labelSize, TextAlign::Left,
             colors::kLabelText);
  r.fillText(w * 0.985f, cy, std::string(buf) + "KT", valueSize,
             TextAlign::Right, colors::kWhite);
}

void drawSoftkeyBar(Renderer& r, float w, float h, float barH,
                    const MfdController& ui) {
  const float top = h - barH;
  r.fillRect(0.0f, top, w, barH, colors::kSoftkeyBackground);
  r.strokeLine(0.0f, top, w, top, 2.0f, colors::kPanelBorder);

  const float cellW = w / static_cast<float>(MfdController::kSoftkeyCount);
  const float cy = top + barH * 0.5f;
  const float size = fontPx(kSoftkeyFontWt, h);
  for (int i = 0; i < MfdController::kSoftkeyCount; ++i) {
    const float cellX = static_cast<float>(i) * cellW;

    // Press flash plus the steady radio highlight combine into one 0..1 level
    // that lifts the cell toward cyan, matching the PFD softkey feedback.
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 0.6f : 0.0f);
    if (level > 0.0f && !ui.label(i).empty()) {
      r.fillRectVerticalGradient(cellX, top, cellW, barH, top, h,
                                 withAlpha(colors::kCyan, level * 0.45f),
                                 withAlpha(colors::kCyan, level * 0.08f));
      r.strokeLine(cellX, top, cellX + cellW, top, 2.5f,
                   withAlpha(colors::kCyan, level));
    }

    if (i > 0) {
      r.strokeLine(cellX, top + barH * 0.15f, cellX, h - barH * 0.15f, 1.0f,
                   colors::kPanelSeparator);
    }
    if (!ui.label(i).empty()) {
      r.fillText(cellX + cellW * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 colors::kWhite);
    }
  }
}

// Full-screen moving map on the MAP page group, reusing the shared MapView.
// North-up is the G1000 MFD default, distinguishing it from the track-up PFD
// inset, and the range comes from the MFD's own (independent) range control.
void drawMapPage(Renderer& r, const FlightData& d, const MapData& map,
                 const MfdController& ui, float x, float y, float w, float h,
                 float displayH) {
  MapViewConfig config;
  config.x = x;
  config.y = y;
  config.w = w;
  config.h = h;
  config.orientation = MapOrientation::NorthUp;
  config.rangeNm = ui.rangeNm();
  config.style.showChrome = true;
  config.style.showTerrain = ui.showTerrain();
  config.style.labelFontWt = 16.0f;
  MapView::render(r, map, d, config, displayH);
}

// Placeholder body for page groups whose pages are not built yet.
void drawPlaceholderPage(Renderer& r, MfdPageGroup group, float x, float y,
                         float w, float h, float displayH) {
  r.fillRect(x, y, w, h, Color{0.0f, 0.0f, 0.0f, 0.82f});
  r.strokeLine(x, y, x + w, y, 2.0f, colors::kTapeTopBorder);
  r.fillText(x + w * 0.5f, y + h * 0.46f, pageGroupTitle(group),
             fontPx(kPageTitleFontWt, displayH), TextAlign::Center,
             colors::kWhite);
  r.fillText(x + w * 0.5f, y + h * 0.54f, "PAGE NOT AVAILABLE",
             fontPx(kInfoLabelWt, displayH), TextAlign::Center,
             colors::kLabelText);
}

}  // namespace

void MultiFunctionDisplay::render(Renderer& r, const FlightData& d,
                                  const MapData& map, const MfdController& ui,
                                  int widthPx, int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float topBarH = h * (kTopBarHeightPx / kCanvasHeight);
  const float bottomBarH = h * (kBottomBarHeightPx / kCanvasHeight);

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  const float bodyY = topBarH;
  const float bodyH = h - topBarH - bottomBarH;
  if (ui.pageGroup() == MfdPageGroup::Map) {
    drawMapPage(r, d, map, ui, 0.0f, bodyY, w, bodyH, h);
  } else {
    drawPlaceholderPage(r, ui.pageGroup(), 0.0f, bodyY, w, bodyH, h);
  }

  drawTopBar(r, w, h, topBarH, d, ui);
  drawSoftkeyBar(r, w, h, bottomBarH, ui);
}

}  // namespace avionics
