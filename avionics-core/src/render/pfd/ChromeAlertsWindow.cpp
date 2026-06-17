#include <algorithm>
#include <string>
#include <vector>

#include "render/pfd/ChromeInternal.h"

namespace avionics::pfd {
namespace {

// WT Alerts.css on the 310x220 popout.
constexpr float kAlertsPadPx = 4.0f;
constexpr float kAlertsContainerHPx = 180.0f;
constexpr float kAlertsLineHeightPx = 26.0f;
constexpr float kAlertsEntryHeightPx = 56.0f;

std::string clipToWidth(Renderer& r, const std::string& text, float maxW,
                        float size) {
  std::string clipped = text;
  while (!clipped.empty() && r.measureTextWidth(clipped, size) > maxW) {
    clipped.pop_back();
  }
  return clipped;
}

// Word-wrap `text` into at most `maxLines` lines that fit `maxW`.
std::vector<std::string> wrapAlertText(Renderer& r, const std::string& text,
                                       float maxW, float size, int maxLines) {
  std::vector<std::string> lines;
  if (text.empty() || maxLines <= 0) return lines;

  std::string remaining = text;
  while (!remaining.empty() && static_cast<int>(lines.size()) < maxLines) {
    std::size_t best = 0;
    for (std::size_t i = 1; i <= remaining.size(); ++i) {
      if (r.measureTextWidth(remaining.substr(0, i), size) <= maxW) {
        best = i;
      } else {
        break;
      }
    }
    if (best == 0) best = 1;

    std::size_t breakAt = best;
    const std::size_t spacePos = remaining.rfind(' ', best);
    if (spacePos != std::string::npos && spacePos > 0) breakAt = spacePos;

    std::string line = remaining.substr(0, breakAt);
    while (!line.empty() && line.back() == ' ') line.pop_back();
    if (!line.empty()) {
      lines.push_back(clipToWidth(r, line, maxW, size));
    }

    remaining.erase(0, breakAt);
    while (!remaining.empty() && remaining.front() == ' ') remaining.erase(0, 1);
  }
  return lines;
}

void drawAlertEntry(Renderer& r, float x, float entryTop, float entryW,
                    float lineH, float entryH, const std::string& text,
                    float size, const Color& color, float a) {
  const int maxLines =
      std::max(1, static_cast<int>(entryH / lineH + 0.001f));
  const std::vector<std::string> lines =
      wrapAlertText(r, text, entryW, size, maxLines);
  float cy = entryTop + lineH * 0.5f;
  for (const std::string& line : lines) {
    if (cy > entryTop + entryH - lineH * 0.25f) break;
    r.fillText(x, cy, line, size, TextAlign::Left, withAlpha(color, a));
    cy += lineH;
  }
}

}  // namespace

// Pop-up Alerts/messages window (under the "Alerts" key).
void drawAlertsWindow(Renderer& r, float w, float h, const Layout& L,
                      const SoftkeyController& ui) {
  float panelW = 0.0f;
  float panelH = 0.0f;
  popoutPanelSize(w, h, panelW, panelH);
  const WindowFrame f =
      drawWindowFrame(r, w, h, L, ui.windowAnim(PfdWindow::Alerts), "Alerts",
                      panelW, panelH);
  if (f.a <= 0.0f) return;
  const float a = f.a;

  const float msgSize = fontPx(wt::kInfoLabel, h);
  const float lineH = fontPx(kAlertsLineHeightPx, h);
  const float entryH = fontPx(kAlertsEntryHeightPx, h);
  const float pad = fontPx(kAlertsPadPx, h);
  const float contentLeft = f.x + pad;
  const float contentRight = f.x + f.w - pad;
  const float contentW = contentRight - contentLeft;
  const float contentTop = f.contentTop + pad;
  const float contentBottom = contentTop + fontPx(kAlertsContainerHPx, h);

  const auto& msgs = ui.alerts();
  if (msgs.empty()) {
    r.fillText(f.x + f.w * 0.5f, contentTop + (contentBottom - contentTop) * 0.5f,
               "NO ACTIVE ALERTS", msgSize, TextAlign::Center,
               withAlpha(colors::kLabelText, a));
    return;
  }

  float entryTop = contentTop;
  for (std::size_t i = 0; i < msgs.size(); ++i) {
    if (entryTop + entryH > contentBottom + 0.5f) break;

    const AlertMessage& m = msgs[i];
    const Color c = (m.level == AlertLevel::Warning)   ? colors::kBandRed
                    : (m.level == AlertLevel::Caution) ? colors::kBandYellow
                                                       : colors::kLabelText;
    drawAlertEntry(r, contentLeft, entryTop, contentW, lineH, entryH, m.text,
                   msgSize, c, a);

    if (i + 1 < msgs.size() && entryTop + entryH <= contentBottom) {
      const float sepY = entryTop + entryH - 1.0f;
      r.fillRect(contentLeft, sepY, contentW, 1.0f,
                 withAlpha(colors::kPanelSeparator, a));
    }

    entryTop += entryH;
  }
}

}  // namespace avionics::pfd
