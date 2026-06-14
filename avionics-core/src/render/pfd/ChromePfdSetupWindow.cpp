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
//
// Unlike the softkey-opened PFD windows, this one is drawn with the real unit's
// own chrome (Fig. 1-18): a near-black grey panel with a grey bevel border, a
// cyan title over a white separator rule, and the two rows packed into the top
// third over an otherwise empty panel -- not the cyan window frame.
void drawPfdSetupWindow(Renderer& r, float w, float h, const Layout& L,
                        const SoftkeyController& ui) {
  const float rawAnim = ui.windowAnim(PfdWindow::Setup);
  if (rawAnim <= 0.0f) return;
  const float a = smoothstep(rawAnim);
  const bool blinkOn = ui.blinkOn();
  const PfdSetupField cursor = ui.pfdSetupCursor();

  // Lower-right anchor with the shared slide-up + fade. The real menu's box is
  // ~1.43:1 (w:h) per Fig. 1-18 and fairly compact; keep the aspect but make it
  // small so the two rows sit tightly under the title.
  const float panelW = w * 0.28f;
  const float panelH = h * 0.26f;
  const float margin = w * 0.012f;
  const float panelX = w - panelW - margin;
  const float panelBottom = (h - L.bottomBarH) - h * 0.012f;
  const float panelTop = panelBottom - panelH + (1.0f - a) * panelH * 0.22f;

  // Body: near-black with rounded corners and a thick light-grey border, as on
  // the real unit (Fig. 1-18). The border is drawn inset by half its width so
  // the stroke sits fully inside the panel rather than straddling the edge.
  const float radius = panelH * 0.07f;
  const float borderW = 3.0f * (h / 768.0f);
  r.fillRoundedRect(panelX, panelTop, panelW, panelH, radius,
                    withAlpha(colors::kBlack, a));
  r.strokeRoundedRect(panelX + borderW * 0.5f, panelTop + borderW * 0.5f,
                      panelW - borderW, panelH - borderW, radius, borderW,
                      withAlpha(colors::kMenuBorderGray, a));

  // Title bar: cyan title near the top with a white separator rule beneath it.
  // The whole menu uses the bundled DejaVu Sans SemiBold display face to match
  // the real unit more closely than the primary Roboto UI font.
  const FontFace kMenuFace = FontFace::DejaVuSemiBold;
  const float titleSize = fontPx(wt::kInfoLabel, h);
  // Title and separator are anchored to the font size (not the panel height) so
  // they stay tight to the top regardless of panel size.
  const float sepY = panelTop + titleSize * 1.85f;
  r.fillText(panelX + panelW * 0.5f, panelTop + titleSize * 1.0f, "PFD Setup Menu",
             titleSize, TextAlign::Center, withAlpha(colors::kCyan, a),
             kMenuFace);
  const float sepInset = borderW + panelW * 0.01f;
  r.strokeLine(panelX + sepInset, sepY, panelX + panelW - sepInset, sepY, 1.5f,
               withAlpha(colors::kWhitesmoke, a));

  // Rows packed directly under the separator with single-line spacing (the
  // empty lower panel below matches the figure). Spacing keys off the font size
  // so the two rows never drift apart as the panel scales.
  const float size = fontPx(wt::kInfoLabel, h);
  const float labelX = panelX + panelW * 0.06f;
  const float modeX = panelX + panelW * 0.50f;
  const float valueRight = panelX + panelW * 0.96f;
  const float row0Cy = sepY + size * 1.35f;
  const float rowGap = size * 1.6f;
  const float rowCy[kPfdSetupRowCount] = {row0Cy, row0Cy + rowGap};

  // Small left/right-pointing arrowhead next to a target label.
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

  // The right-pointing carrot sits in a fixed column so both rows' carrots line
  // up vertically (as on the real unit) regardless of each label's width. Anchor
  // it just past the widest label among the rows.
  float maxLabelW = 0.0f;
  for (const Row& row : rows) {
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string t =
        std::string(row.name) + (isKey ? " Key" : " Display");
    maxLabelW = std::max(maxLabelW, r.measureTextWidth(t, size, kMenuFace));
  }
  const float rightArrowX = labelX + maxLabelW + size * 0.30f;

  for (int i = 0; i < kPfdSetupRowCount; ++i) {
    const Row& row = rows[i];
    const float cy = rowCy[i];
    const bool isKey = ui.pfdSetupTarget(row.row) == BacklightTarget::Key;
    const std::string targetText =
        std::string(row.name) + (isKey ? " Key" : " Display");

    // Green arrowhead shows where the small knob can still toggle: right toward
    // Key when on Display, left back toward Display when on Key.
    arrow(labelX - size * 0.55f, cy, /*pointRight=*/false,
          isKey ? colors::kActiveGreen : colors::kLabelText);
    putField(r, labelX, cy, targetText, size, colors::kCyan,
             cursor == row.target, a, blinkOn, 0.0f, kMenuFace);
    arrow(rightArrowX, cy, /*pointRight=*/true,
          isKey ? colors::kLabelText : colors::kActiveGreen);

    // Mode (Auto / Manual).
    const std::string modeText =
        ui.pfdSetupMode(row.row) == BacklightMode::Manual ? "Manual" : "Auto";
    putField(r, modeX, cy, modeText, size, colors::kCyan, cursor == row.mode, a,
             blinkOn, 0.0f, kMenuFace);

    // Intensity percentage, right-aligned.
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f%%", ui.pfdSetupIntensityPct(row.row));
    const std::string valueText(buf);
    const float vx = valueRight - r.measureTextWidth(valueText, size, kMenuFace);
    putField(r, vx, cy, valueText, size, colors::kWhite, cursor == row.value, a,
             blinkOn, 0.0f, kMenuFace);
  }
}

}  // namespace avionics::pfd
