#include <algorithm>
#include <cstdio>
#include <string>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {

// PFD Setup Menu (PFD MENU key, Pilot's Guide Fig. 1-18): a lower-right popout
// with a backlighting row for each display. Each row is an arrow-toggle target
// (Display / Key), a mode (Auto / Manual), and an intensity percentage. The
// large FMS knob moves the cursor between fields, the small knob edits the
// highlighted one (the green arrowhead shows the way the target can still
// toggle), and ENT steps onto the intensity once Manual is selected.
void drawPfdSetupWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const float rawAnim = ui.windowAnim(PfdWindow::Setup);
  if (rawAnim <= 0.0f) return;
  const float a = smoothstep(rawAnim);
  const PfdSetupField cursor = ui.pfdSetupCursor();

  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, rawAnim, "PFD Setup Menu", panelW, panelH);
  if (f.a <= 0.0f) return;

  // WT .pfd-setup-item-block: 18 px body on the shared 310x220 shell.
  const float size = fontPx(18.0f, h);
  const float labelX = f.x + f.w * 0.06f;
  const float modeX = f.x + f.w * 0.55f;
  const float valueRight = f.x + f.w * 0.975f;
  const float row0Cy = f.contentTop + size * 0.85f;
  const float rowGap = size * 1.55f;
  const float rowCy[kPfdSetupRowCount] = {row0Cy, row0Cy + rowGap};

  const auto arrow = [&](float ax, float acy, bool pointRight, const Color& c) {
    const float aw = size * 0.30f;
    const float ah = size * 0.46f;
    if (pointRight) {
      const Point t[3] = {{ax, acy - ah * 0.5f},
                          {ax + aw, acy},
                          {ax, acy + ah * 0.5f}};
      r.fillPolygon(t, 3, withAlpha(c, a));
    } else {
      const Point t[3] = {{ax + aw, acy - ah * 0.5f},
                          {ax, acy},
                          {ax + aw, acy + ah * 0.5f}};
      r.fillPolygon(t, 3, withAlpha(c, a));
    }
  };

  struct Row {
    const char* name;
    PfdSetupRow row;
    PfdSetupField target, mode, value;
  };
  const Row rows[kPfdSetupRowCount] = {
      {"PFD", PfdSetupRow::Pfd, PfdSetupField::PfdTarget, PfdSetupField::PfdMode,
       PfdSetupField::PfdValue},
      {"MFD", PfdSetupRow::Mfd, PfdSetupField::MfdTarget, PfdSetupField::MfdMode,
       PfdSetupField::MfdValue},
  };

  float maxLabelW = 0.0f;
  for (const Row& row : rows) {
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string t =
        std::string(row.name) + (isKey ? " Key" : " Display");
    maxLabelW = std::max(maxLabelW, r.measureTextWidth(t, size));
  }
  const float rightArrowX = labelX + maxLabelW + size * 0.30f;

  for (int i = 0; i < kPfdSetupRowCount; ++i) {
    const Row& row = rows[i];
    const float cy = rowCy[i];
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string targetText =
        std::string(row.name) + (isKey ? " Key" : " Display");

    arrow(labelX - size * 0.55f, cy, /*pointRight=*/false,
          isKey ? colors::kLabelText : colors::kPopoutCyan);
    putField(r, labelX, cy, targetText, size, colors::kPopoutCyan,
             cursor == row.target, a, 0.0f);
    arrow(rightArrowX, cy, /*pointRight=*/true,
          isKey ? colors::kPopoutCyan : colors::kLabelText);

    const std::string modeText =
        ui.pfdSetupMode(row.row) == BacklightMode::Manual ? "Manual" : "Auto";
    putField(r, modeX, cy, modeText, size, colors::kPopoutCyan, cursor == row.mode, a,
             0.0f);

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f%%", ui.pfdSetupIntensityPct(row.row));
    const std::string valueText(buf);
    const float vx = valueRight - r.measureTextWidth(valueText, size);
    putField(r, vx, cy, valueText, size, colors::kWhite, cursor == row.value, a,
             0.0f);
  }
}

}  // namespace avionics::pfd
