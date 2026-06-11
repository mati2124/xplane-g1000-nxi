#include "avionics/render/MultiFunctionDisplay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "render/mfd/EisStrip.h"
#include "render/mfd/MfdPages.h"
#include "render/mfd/MfdStyle.h"

namespace avionics {
namespace {

using mfd::mfdAlpha;
using mfd::mfdFontPx;

// Top bar height and typography match the PFD's NAV/COM bar (56 px, 24/16 px
// fonts on the 768 canvas) -- on the real unit both GDUs share the same bar.
constexpr float kTopBarHeightPx = 56.0f;
constexpr float kBottomBarHeightPx = 35.0f;

// EIS engine strip width as a fraction of the screen, matching the dedicated
// engine column on the left edge of the real MFD.
constexpr float kEisWidthFrac = 150.0f / 1024.0f;

constexpr float kFreqLabelWt = 16.0f;
constexpr float kFreqValueWt = 24.0f;
constexpr float kDataFieldLabelWt = 15.0f;
constexpr float kDataFieldValueWt = 20.0f;
constexpr float kPageTitleWt = 16.0f;
constexpr float kSoftkeyFontWt = 17.0f;

// Placeholders shown for the frequency readouts when the data link is down,
// matching the PFD chrome (unknown rather than stale).
constexpr const char* kFreqDash2 = "---.--";
constexpr const char* kFreqDash3 = "---.---";

// Group-prefixed page title shown in the navigation data bar, verbatim from
// the WT NXi MFDUiPage titles ("Map – Navigation Map" mixed case).
const char* pageTitle(MfdPage page) {
  switch (page) {
    case MfdPage::NavigationMap:
      return "Map \xE2\x80\x93 Navigation Map";
    case MfdPage::TrafficMap:
      return "Map \xE2\x80\x93 Traffic Map";
    case MfdPage::WeatherRadar:
      return "Map \xE2\x80\x93 Weather Radar";
    case MfdPage::AirportInformation:
      return "WPT \xE2\x80\x93 Airport Information";
    case MfdPage::IntersectionInformation:
      return "WPT \xE2\x80\x93 Intersection Information";
    case MfdPage::NdbInformation:
      return "WPT \xE2\x80\x93 NDB Information";
    case MfdPage::VorInformation:
      return "WPT \xE2\x80\x93 VOR Information";
    case MfdPage::TripPlanning:
      return "Aux \xE2\x80\x93 Trip Planning";
    case MfdPage::Utility:
      return "Aux \xE2\x80\x93 Utility";
    case MfdPage::GpsStatus:
      return "Aux \xE2\x80\x93 GPS Status";
    case MfdPage::SystemSetup:
      return "Aux \xE2\x80\x93 System Setup";
    case MfdPage::SystemStatus:
      return "Aux \xE2\x80\x93 System Status";
    case MfdPage::SimBrief:
      return "Aux \xE2\x80\x93 SimBrief";
    case MfdPage::NearestAirports:
      return "NRST \xE2\x80\x93 Nearest Airports";
    case MfdPage::NearestIntersections:
      return "NRST \xE2\x80\x93 Nearest Intersections";
    case MfdPage::NearestNdb:
      return "NRST \xE2\x80\x93 Nearest NDB";
    case MfdPage::NearestVor:
      return "NRST \xE2\x80\x93 Nearest VOR";
    case MfdPage::NearestFrequencies:
      return "NRST \xE2\x80\x93 Nearest Frequencies";
    case MfdPage::NearestAirspaces:
      return "NRST \xE2\x80\x93 Nearest Airspaces";
    case MfdPage::ActiveFlightPlan:
      return "FPL \xE2\x80\x93 Active Flight Plan";
  }
  return "";
}

// Bare page names per group, for the page-select popup's page list.
const char* const kMapPages[] = {"Navigation Map", "Traffic Map",
                                 "Weather Radar"};
const char* const kWptPages[] = {"Airport Information",
                                 "Intersection Information",
                                 "NDB Information", "VOR Information"};
const char* const kAuxPages[] = {"Trip Planning", "Utility",
                                 "GPS Status",     "System Setup",
                                 "System Status",  "SimBrief"};
const char* const kNrstPages[] = {"Nearest Airports", "Nearest Intersections",
                                  "Nearest NDB",      "Nearest VOR",
                                  "Nearest Frequencies", "Nearest Airspaces"};
const char* const kFplPages[] = {"Active Flight Plan"};

// Double-headed cyan frequency transfer arrow, identical to the PFD's.
void drawTransferArrow(Renderer& r, float cx, float cy, float halfW) {
  const float a = halfW * 0.55f;
  r.strokeLine(cx - halfW, cy, cx + halfW, cy, 1.5f, colors::kCyan);
  r.strokeLine(cx - halfW, cy, cx - halfW + a, cy - a, 1.5f, colors::kCyan);
  r.strokeLine(cx - halfW, cy, cx - halfW + a, cy + a, 1.5f, colors::kCyan);
  r.strokeLine(cx + halfW, cy, cx + halfW - a, cy - a, 1.5f, colors::kCyan);
  r.strokeLine(cx + halfW, cy, cx + halfW - a, cy + a, 1.5f, colors::kCyan);
}

// 3-digit bearing/track with degree sign; north reads 360, never 000.
std::string formatBearing(float deg) {
  int v = static_cast<int>(std::lround(deg)) % 360;
  if (v < 0) v += 360;
  if (v == 0) v = 360;
  char buf[12];
  std::snprintf(buf, sizeof(buf), "%03d\xC2\xB0", v);
  return buf;
}

// ETE to the active waypoint: MM:SS under an hour, H:MM above. The G1000
// blanks the field below ~30 kt ground speed (the estimate is meaningless
// while stationary/taxiing).
std::string formatEte(float distNm, float gsKts) {
  if (gsKts < 30.0f || distNm <= 0.0f) return "__:__";
  const long totalSec = std::lround(distNm / gsKts * 3600.0f);
  char buf[12];
  if (totalSec >= 3600) {
    std::snprintf(buf, sizeof(buf), "%ld:%02ld", totalSec / 3600,
                  (totalSec % 3600) / 60);
  } else {
    std::snprintf(buf, sizeof(buf), "%02ld:%02ld", totalSec / 60,
                  totalSec % 60);
  }
  return buf;
}

// Top data bar, mirroring the real NXi MFD: NAV1/NAV2 pairs (left, active
// nearest center), the navigation data bar (GS/DTK/TRK/ETE fields over the
// cyan page title) in the middle, COM1/COM2 pairs (right, active nearest
// center). Frequency cells share the PFD's layout fractions so the two
// displays' bars line up when side by side.
void drawNavComBar(Renderer& r, float w, float h, float barH,
                   const FlightData& d, const std::string& title) {
  r.fillRectVerticalGradient(0.0f, 0.0f, w, barH, 0.0f, barH,
                             colors::kPanelBackground,
                             colors::kPanelBackgroundBottom);
  r.strokeLine(0.0f, barH, w, barH, 2.0f, colors::kPanelBorder);

  // Same row fractions as the PFD bar so the rows line up across displays.
  const float row1 = barH * 0.27f;
  const float row2 = barH * 0.73f;
  const float labelSize = mfdFontPx(kFreqLabelWt, h);
  const float freqSize = mfdFontPx(kFreqValueWt, h);

  // With the link down the tuned frequencies are unknown: amber dashes in
  // place of the active/standby readouts (labels and arrows stay).
  const bool linkValid = d.dataLinkValid;
  const Color activeColor =
      linkValid ? colors::kActiveGreen : colors::kBandYellow;
  const Color standbyColor = linkValid ? colors::kWhite : colors::kBandYellow;
  auto freqText = [&](float mhz, int decimals) -> std::string {
    if (!linkValid) return decimals >= 3 ? kFreqDash3 : kFreqDash2;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals,
                  static_cast<double>(mhz));
    return buf;
  };

  struct Pair {
    const char* label;
    float active, standby, cy;
  };
  const Pair navs[2] = {{"NAV1", d.nav1ActiveMhz, d.nav1StandbyMhz, row1},
                        {"NAV2", d.nav2ActiveMhz, d.nav2StandbyMhz, row2}};
  for (const Pair& n : navs) {
    r.fillText(w * 0.012f, n.cy, n.label, labelSize, TextAlign::Left,
               colors::kLabelText);
    r.fillText(w * 0.140f, n.cy, freqText(n.standby, 2), freqSize,
               TextAlign::Right, standbyColor);
    drawTransferArrow(r, w * 0.158f, n.cy, w * 0.011f);
    r.fillText(w * 0.245f, n.cy, freqText(n.active, 2), freqSize,
               TextAlign::Right, activeColor);
  }

  const Pair coms[2] = {{"COM1", d.com1ActiveMhz, d.com1StandbyMhz, row1},
                        {"COM2", d.com2ActiveMhz, d.com2StandbyMhz, row2}};
  for (const Pair& c : coms) {
    r.fillText(w * 0.832f, c.cy, freqText(c.active, 3), freqSize,
               TextAlign::Right, activeColor);
    drawTransferArrow(r, w * 0.844f, c.cy, w * 0.011f);
    r.fillText(w * 0.940f, c.cy, freqText(c.standby, 3), freqSize,
               TextAlign::Right, standbyColor);
    r.fillText(w * 0.996f, c.cy, c.label, labelSize, TextAlign::Right,
               colors::kLabelText);
  }

  // Navigation data bar fields (default NXi set: GS, DTK, TRK, ETE), grey
  // labels with magenta GPS-derived values, dashed when unknown.
  char gsBuf[12];
  std::snprintf(gsBuf, sizeof(gsBuf), "%dKT",
                static_cast<int>(std::lround(d.groundSpeedKts)));
  const bool hasActiveWpt = !d.fmaToWpt.empty();
  const bool gpsDtk = hasActiveWpt && d.cdiSource == CdiSource::Gps;
  struct Field {
    const char* label;
    std::string value;
  };
  const Field fields[4] = {
      {"GS", linkValid ? std::string(gsBuf) : std::string("___KT")},
      {"DTK", linkValid && gpsDtk ? formatBearing(d.courseDeg)
                                  : std::string("___\xC2\xB0")},
      {"TRK", linkValid ? formatBearing(d.trackDeg)
                        : std::string("___\xC2\xB0")},
      {"ETE", linkValid && hasActiveWpt
                  ? formatEte(d.fmaLegDistanceNm, d.groundSpeedKts)
                  : std::string("__:__")},
  };
  const float fieldLabelSize = mfdFontPx(kDataFieldLabelWt, h);
  const float fieldValueSize = mfdFontPx(kDataFieldValueWt, h);
  const float fieldW = w * 0.115f;
  float fx = w * 0.27f;
  for (const Field& f : fields) {
    r.fillText(fx, row1, f.label, fieldLabelSize, TextAlign::Left,
               colors::kLabelText);
    const float lw = r.measureTextWidth(f.label, fieldLabelSize);
    r.fillText(fx + lw + w * 0.008f, row1, f.value, fieldValueSize,
               TextAlign::Left, colors::kMagenta);
    fx += fieldW;
  }

  // Page title, centered below the data fields (cyan, like the NXi).
  r.fillText(w * 0.5f, row2, title, mfdFontPx(kPageTitleWt, h),
             TextAlign::Center, colors::kCyan);
}

// Transient page-select popup in the lower-right corner, in the NXi style
// (WT MFDPageSelect): a black box with a 1px gray border holding the active
// group's page list (gray items, the current page cyan) above a row of
// trapezoid group tabs with the active tab "open" into the box. Appears on
// any group/page change and auto-closes after a few seconds idle.
void drawPageIndicator(Renderer& r, float w, float h, float bottomBarTop,
                       const MfdController& ui, int checklistPageCount) {
  const float secs = ui.pageSelectSecondsLeft();
  if (secs <= 0.0f) return;
  const float alpha = std::min(1.0f, secs / 0.25f);  // quick fade-out
  const FontScope fs(r, FontFace::DejaVuSemiBold);

  struct GroupTab {
    const char* label;
    MfdPageGroup group;
  };
  const GroupTab tabs[] = {{"Map", MfdPageGroup::Map},
                           {"WPT", MfdPageGroup::Waypoint},
                           {"Aux", MfdPageGroup::Aux},
                           {"FPL", MfdPageGroup::FlightPlan},
                           {"NRST", MfdPageGroup::Nearest},
                           {"CHK", MfdPageGroup::Checklist}};
  const int tabCount = static_cast<int>(sizeof(tabs) / sizeof(tabs[0]));

  // The active group's page list. The Checklist group's pages are dynamic
  // (one per loaded checklist), so those entries are synthesized.
  const char* const* pages = nullptr;
  int pageCount = 0;
  switch (ui.pageGroup()) {
    case MfdPageGroup::Map:
      pages = kMapPages;
      pageCount = 3;
      break;
    case MfdPageGroup::Waypoint:
      pages = kWptPages;
      pageCount = 4;
      break;
    case MfdPageGroup::Aux:
      pages = kAuxPages;
      pageCount = 6;
      break;
    case MfdPageGroup::Nearest:
      pages = kNrstPages;
      pageCount = 6;
      break;
    case MfdPageGroup::FlightPlan:
      pages = kFplPages;
      pageCount = 1;
      break;
    case MfdPageGroup::Checklist:
      pageCount = std::max(1, checklistPageCount);
      break;
  }

  const float itemSize = mfdFontPx(16.0f, h);
  const float itemH = itemSize * 1.5f;
  const float tabSize = mfdFontPx(14.0f, h);
  const float tabH = tabSize * 1.9f;
  const float pad = itemSize * 0.55f;
  // Cap the (checklist) list at what fits sensibly.
  const int listRows = std::min(pageCount, 8);
  const float boxW = w * (260.0f / 1024.0f);
  const float boxH = pad + listRows * itemH + pad * 0.4f + tabH;
  const float bx = w - boxW - w * 0.004f;
  const float by = bottomBarTop - boxH - h * 0.006f;

  // Shared menu chrome: black rounded body with a thick light-grey rounded
  // border (matches the dialog/pop-up look).
  const float radius = itemSize * 0.5f;
  const float borderW = 2.5f * (h / 768.0f);
  r.fillRoundedRect(bx, by, boxW, boxH, radius, mfdAlpha(colors::kBlack, alpha));
  r.strokeRoundedRect(bx + borderW * 0.5f, by + borderW * 0.5f, boxW - borderW,
                      boxH - borderW, radius, borderW,
                      mfdAlpha(colors::kMenuBorderGray, alpha));

  // Page list: gray entries, the current page cyan. Long checklist groups
  // scroll the window around the current page.
  int firstRow = 0;
  if (pageCount > listRows) {
    firstRow = std::max(
        0, std::min(ui.pageIndex() - listRows / 2, pageCount - listRows));
  }
  float iy = by + pad;
  char chkBuf[32];
  for (int i = firstRow; i < firstRow + listRows; ++i) {
    const float cy = iy + itemH * 0.5f;
    const char* name;
    if (pages != nullptr) {
      name = pages[i];
    } else {
      std::snprintf(chkBuf, sizeof(chkBuf), "Checklist %d", i + 1);
      name = chkBuf;
    }
    const Color c = i == ui.pageIndex()
                        ? mfdAlpha(colors::kCyan, alpha)
                        : mfdAlpha(colors::kTitleGray, alpha);
    r.fillText(bx + pad * 1.4f, cy, name, itemSize, TextAlign::Left, c);
    iy += itemH;
  }

  // Group tabs along the bottom: a border line across the top of the row,
  // broken over the active tab whose sides splay outward (the trapezoid
  // "open folder tab" look).
  const float tabTop = by + boxH - tabH;
  const float tabBottom = by + boxH;
  const float cellW = boxW / static_cast<float>(tabCount);
  const float slant = tabH * 0.28f;
  const Color lineColor = mfdAlpha(colors::kGroupBoxBorder, alpha);
  for (int i = 0; i < tabCount; ++i) {
    const float x0 = bx + cellW * static_cast<float>(i);
    const float x1 = x0 + cellW;
    const bool active = ui.pageGroup() == tabs[i].group;
    if (active) {
      // Slanted sides from the border line down to the box bottom.
      r.strokeLine(x0, tabTop, x0 - slant, tabBottom, 1.0f, lineColor);
      r.strokeLine(x1, tabTop, x1 + slant, tabBottom, 1.0f, lineColor);
    } else {
      r.strokeLine(x0, tabTop, x1, tabTop, 1.0f, lineColor);
    }
    r.fillText(x0 + cellW * 0.5f, tabTop + tabH * 0.52f, tabs[i].label,
               tabSize, TextAlign::Center,
               active ? mfdAlpha(colors::kCyan, alpha)
                      : mfdAlpha(colors::kTitleGray, alpha));
  }
}

// On-screen softkey label bar. Display only, like the real unit: the labels
// name the functions of the physical keys on the bezel directly below the
// screen. Selected keys show black text on a gray background (G1000 Pilot's
// Guide for the Diamond DA40, "Softkey Function"); a press flashes the same
// look momentarily. Layout matches the PFD bar so the displays line up.
void drawSoftkeyBar(Renderer& r, float w, float h, float barH,
                    const MfdController& ui) {
  const float top = h - barH;
  r.fillRect(0.0f, top, w, barH, colors::kSoftkeyBackground);

  const float cellW = w / static_cast<float>(MfdController::kSoftkeyCount);
  const float cy = top + barH * 0.5f;
  const float size = mfdFontPx(kSoftkeyFontWt, h);
  const float insetX = cellW * 0.055f;
  const float insetY = barH * 0.13f;
  for (int i = 0; i < MfdController::kSoftkeyCount; ++i) {
    const float bx = static_cast<float>(i) * cellW + insetX;
    const float by = top + insetY;
    const float bw = cellW - 2.0f * insetX;
    const float bh = barH - 2.0f * insetY;

    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 1.0f : 0.0f);
    if (level > 0.0f) {
      r.fillRect(bx, by, bw, bh, mfdAlpha(colors::kSoftkeySelected, level));
    }
    const Point frame[5] = {
        {bx, by}, {bx + bw, by}, {bx + bw, by + bh}, {bx, by + bh}, {bx, by}};
    r.strokePolyline(frame, 5, 1.0f, colors::kPanelSeparator);

    if (!ui.label(i).empty()) {
      r.fillText(bx + bw * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 level > 0.5f ? colors::kBlack : colors::kWhite);
    }
  }
}

// The Page Menu popout (MENU bezel key, Pilot's Guide Fig. 5-6): a "Page Menu"
// dialog holding an "Options" group box with the current page's option list and
// a footer hint. The highlighted option pulses with the highlight-select
// cursor; disabled options (features not yet modeled) are greyed, like the real
// unit. Drawn over the page body, under the top/bottom chrome bars.
void drawPageMenu(Renderer& r, const MfdController& ui, float x, float y,
                  float w, float h, float displayH) {
  const int n = ui.pageMenuItemCount();
  if (n <= 0) return;

  const FontScope fs(r, FontFace::DejaVuSemiBold);
  using mfd::drawDialog;
  using mfd::drawGroupBox;
  using mfd::Rect;

  // Option rows match the PFD Setup Menu's row size (pfd::wt::kInfoLabel, 16px
  // on the 768 canvas) so every pop-up menu reads at the same scale.
  const float rowSize = mfd::mfdFontPx(16.0f, displayH);
  const float rowH = rowSize * 1.55f;
  const float footSize = mfd::mfdFontPx(13.0f, displayH);
  const float titleSize = mfd::mfdFontPx(16.0f, displayH);
  const float pad = mfd::mfdFontPx(10.0f, displayH);

  // Group-box slot height: the title overhang, top/bottom padding, plus a row
  // per option (drawGroupBox's own insets are folded in here).
  const float groupSlotH = titleSize * 0.55f + pad + n * rowH + pad * 0.8f;
  const float footH = footSize * 3.4f;
  const float boxW = w * 0.40f;
  const float boxH =
      titleSize * 1.6f + mfd::mfdFontPx(6.0f, displayH) + groupSlotH + footH +
      pad * 2.0f;

  // Anchored to the top-right corner of the display, flush under the top bar,
  // matching the real unit (Pilot's Guide Fig. 5-6) rather than centered.
  const float margin = mfd::mfdFontPx(6.0f, displayH);
  const Rect inner = drawDialog(
      r, Rect{x + w - boxW - margin, y + margin, boxW, boxH}, "Page Menu",
      displayH);

  const Rect group = drawGroupBox(
      r, Rect{inner.x, inner.y, inner.w, groupSlotH}, "Options", displayH);

  float fy = group.y;
  for (int i = 0; i < n; ++i) {
    const float cy = fy + rowH * 0.5f;
    const std::string& text = ui.pageMenuItemText(i);
    const bool enabled = ui.pageMenuItemEnabled(i);
    if (i == ui.pageMenuSelected() && enabled) {
      // Full-width highlight bar with black text (pulses ~1 Hz), matching the
      // figure's selected option.
      if (ui.blinkOn()) {
        r.fillRect(group.x - pad * 0.4f, cy - rowSize * 0.62f, group.w,
                   rowSize * 1.24f, colors::kCyan);
        r.fillText(group.x, cy, text, rowSize, TextAlign::Left, colors::kBlack);
      } else {
        r.fillText(group.x, cy, text, rowSize, TextAlign::Left, colors::kCyan);
      }
    } else {
      r.fillText(group.x, cy, text, rowSize, TextAlign::Left,
                 enabled ? colors::kWhite : colors::kDisabledGray);
    }
    fy += rowH;
  }

  const float footCx = inner.x + inner.w * 0.5f;
  const float footY = group.y + n * rowH + footSize * 1.2f;
  r.fillText(footCx, footY, "Press the FMS CRSR knob to return", footSize,
             TextAlign::Center, colors::kTitleGray);
  r.fillText(footCx, footY + footSize * 1.3f, "to base page", footSize,
             TextAlign::Center, colors::kTitleGray);
}

}  // namespace

void MultiFunctionDisplay::render(Renderer& r, const FlightData& d,
                                  const MapData& map,
                                  const ChecklistData& checklist,
                                  const EisLayout& eisLayout,
                                  const MfdController& ui, int widthPx,
                                  int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float topBarH = h * (kTopBarHeightPx / mfd::kCanvasHeight);
  const float bottomBarH = h * (kBottomBarHeightPx / mfd::kCanvasHeight);

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  // The EIS engine strip owns the left edge on every page, like the real MFD;
  // the page body fills the remaining width.
  const float eisW = w * kEisWidthFrac;
  const float bodyX = eisW;
  const float bodyY = topBarH;
  const float bodyW = w - eisW;
  const float bodyH = h - topBarH - bottomBarH;
  // The Checklist group's pages are dynamic (one per loaded checklist) and so
  // sit outside the static MfdPage enum; dispatch it before the page switch.
  std::string title;
  if (ui.pageGroup() == MfdPageGroup::Checklist) {
    mfd::drawChecklistPage(r, checklist, ui, bodyX, bodyY, bodyW, bodyH, h);
    if (const Checklist* cl = checklist.at(ui.checklistIndex())) {
      title = std::string("Checklist \xE2\x80\x93 ") + cl->title;
    } else {
      title = "Checklist";
    }
  } else {
  switch (ui.page()) {
    case MfdPage::NavigationMap:
      mfd::drawMapPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::TrafficMap:
      mfd::drawTrafficMapPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::WeatherRadar:
      mfd::drawWeatherRadarPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::AirportInformation:
      mfd::drawWaypointPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::IntersectionInformation:
      mfd::drawWaypointNavaidPage(r, d, map, ui, MapFeatureType::Fix, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NdbInformation:
      mfd::drawWaypointNavaidPage(r, d, map, ui, MapFeatureType::Ndb, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::VorInformation:
      mfd::drawWaypointNavaidPage(r, d, map, ui, MapFeatureType::Vor, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::TripPlanning:
      mfd::drawTripPlanningPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::Utility:
      mfd::drawUtilityPage(r, d, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::GpsStatus:
      mfd::drawGpsStatusPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::SystemSetup:
      mfd::drawSystemSetupPage(r, d, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::SystemStatus:
      mfd::drawSystemStatusPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::SimBrief:
      mfd::drawSimBriefPage(r, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestAirports:
      mfd::drawNearestAirportsPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH,
                                   h);
      break;
    case MfdPage::NearestIntersections:
      mfd::drawNearestFeaturePage(r, d, map, ui, MapFeatureType::Fix, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestNdb:
      mfd::drawNearestFeaturePage(r, d, map, ui, MapFeatureType::Ndb, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestVor:
      mfd::drawNearestFeaturePage(r, d, map, ui, MapFeatureType::Vor, bodyX,
                                  bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestFrequencies:
      mfd::drawNearestFrequenciesPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestAirspaces:
      mfd::drawNearestAirspacesPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH,
                                    h);
      break;
    case MfdPage::ActiveFlightPlan:
      mfd::drawActiveFlightPlanPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH,
                                    h);
      break;
  }
  title = pageTitle(ui.page());
  }

  mfd::drawEisStrip(r, d, eisLayout, mfd::Rect{0.0f, bodyY, eisW, bodyH}, h);
  // The Direct-To window overlays whatever page is up (it is opened by the
  // Direct-To bezel key from anywhere), drawn over the body but under the
  // top/bottom chrome bars.
  // Both pop-ups slide up and fade in/out with the shared window animation
  // (the same logic as the PFD menus), so they are drawn whenever their open
  // progress is nonzero rather than only while strictly open.
  const float dtoAnim = ui.directToWindowAnim();
  if (dtoAnim > 0.0f) {
    r.save();
    r.globalAlpha(mfd::mfdSmoothstep(dtoAnim));
    r.translate(0.0f, mfd::mfdWindowSlide(dtoAnim, h));
    mfd::drawDirectToWindow(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
    r.restore();
  }
  // The Page Menu (MENU key) overlays the base page, like the Direct-To window.
  const float pageMenuAnim = ui.pageMenuAnim();
  if (pageMenuAnim > 0.0f) {
    r.save();
    r.globalAlpha(mfd::mfdSmoothstep(pageMenuAnim));
    r.translate(0.0f, mfd::mfdWindowSlide(pageMenuAnim, h));
    drawPageMenu(r, ui, bodyX, bodyY, bodyW, bodyH, h);
    r.restore();
  }
  // The Map Settings window (MENU -> Map Settings) overlays the navigation map,
  // animating in like the Page Menu.
  const float mapSettingsAnim = ui.mapSettingsAnim();
  if (mapSettingsAnim > 0.0f) {
    r.save();
    r.globalAlpha(mfd::mfdSmoothstep(mapSettingsAnim));
    r.translate(0.0f, mfd::mfdWindowSlide(mapSettingsAnim, h));
    mfd::drawMapSettingsWindow(r, ui, bodyX, bodyY, bodyW, bodyH, h);
    r.restore();
  }
  drawPageIndicator(r, w, h, h - bottomBarH, ui, checklist.totalChecklists());
  drawNavComBar(r, w, h, topBarH, d, title);
  drawSoftkeyBar(r, w, h, bottomBarH, ui);
}

}  // namespace avionics
