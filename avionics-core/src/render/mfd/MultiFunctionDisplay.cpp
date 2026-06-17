#include "avionics/render/MultiFunctionDisplay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/render/SoftkeyLabelBar.h"
#include "render/mfd/EisStrip.h"
#include "render/mfd/MfdPages.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/PfdInternal.h"

namespace avionics {
namespace {

using mfd::mfdAlpha;
using mfd::mfdFontPx;

// Top bar height matches the PFD's NAV/COM bar (56 px on the 768 canvas) -- on
// the real unit both GDUs share the same bar.
constexpr float kTopBarHeightPx = pfd::kTopBarHeightPx;
constexpr float kBottomBarHeightPx = pfd::kSoftkeyBarHeightPx;

// Navigation data bar typography (WT MFDNavDataBar.css: 20 px fields, labels
// at 0.75em, page title 20 px cyan).
constexpr float kDataFieldLabelWt = 15.0f;
constexpr float kDataFieldValueWt = 20.0f;
constexpr float kPageTitleWt = 20.0f;
constexpr float kSoftkeyFontWt = 17.0f;

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

// WT WTG1000_MFD.css NavComBox: left 254 px, center 516 px at x=254, right
// 254 px at x=770 -- the three panels are flush with each other and the top
// edge of the display.
constexpr float kMfdNavPanelWFrac = 254.0f / 1024.0f;
constexpr float kMfdCenterPanelLFrac = 254.0f / 1024.0f;
constexpr float kMfdCenterPanelWFrac = 516.0f / 1024.0f;
constexpr float kMfdComPanelLFrac = 770.0f / 1024.0f;
constexpr float kMfdComPanelWFrac = 254.0f / 1024.0f;

// Top data bar, mirroring the real NXi MFD: three flush rounded panels (NAV
// left, navigation data bar center, COM right). The NAV/COM frequency cells
// are drawn by the shared PFD routine (pfd::drawNavComFreqCells). The decoded
// COM station ID is PFD-only and is not drawn on the MFD.
void drawNavComBar(Renderer& r, float w, float h, float barH,
                   const FlightData& d, const SoftkeyController& radios,
                   const std::string& title) {
  const float cornerR = barH * (10.0f / pfd::kTopBarHeightPx);

  const float navLeft = 0.0f;
  const float navW = w * kMfdNavPanelWFrac;
  const float centerL = w * kMfdCenterPanelLFrac;
  const float centerW = w * kMfdCenterPanelWFrac;
  const float comLeft = w * kMfdComPanelLFrac;
  const float comW = w * kMfdComPanelWFrac;

  pfd::drawNavComPanelBg(r, navLeft, 0.0f, navW, barH, cornerR,
                         pfd::NavComPanelShape::MfdLeft);
  pfd::drawNavComPanelBg(r, centerL, 0.0f, centerW, barH, cornerR,
                         pfd::NavComPanelShape::MfdCenter);
  pfd::drawNavComPanelBg(r, comLeft, 0.0f, comW, barH, cornerR,
                         pfd::NavComPanelShape::MfdRight);

  pfd::drawNavComFreqCells(r, h, barH, navLeft, navW, comLeft, comW, d, radios);

  // Thin grey outline along the outer left/right edges and the bar foot
  // (trainer MFD Default.bmp; no vertical rules between the touching panels).
  r.strokeLine(0.0f, 0.0f, 0.0f, barH, 1.0f, colors::kPanelBorder);
  r.strokeLine(w, 0.0f, w, barH, 1.0f, colors::kPanelBorder);
  r.strokeLine(0.0f, barH, w, barH, 1.0f, colors::kPanelBorder);

  const bool linkValid = d.dataLinkValid;

  // Navigation data bar fields (default NXi set: GS, DTK, TRK, ETE), grey
  // labels with magenta GPS-derived values, dashed when unknown. The top row
  // occupies the upper half of the center panel (WT .nav-data-bar height 50%).
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
  const FontScope centerFont(r, FontFace::DejaVuSemiBold);
  const float fieldLabelSize = mfdFontPx(kDataFieldLabelWt, h);
  const float fieldValueSize = mfdFontPx(kDataFieldValueWt, h);
  const float fieldRowCy = barH * 0.25f;
  const float fieldPadX = centerW * 0.02f;
  const float fieldW = (centerW - fieldPadX * 2.0f) / 4.0f;
  float fx = centerL + fieldPadX;
  for (const Field& f : fields) {
    r.fillText(fx, fieldRowCy, f.label, fieldLabelSize, TextAlign::Left,
               colors::kLabelText);
    const float lw = r.measureTextWidth(f.label, fieldLabelSize);
    r.fillText(fx + lw + fieldValueSize * 0.08f, fieldRowCy, f.value,
               fieldValueSize, TextAlign::Left, colors::kMagenta);
    fx += fieldW;
  }

  // Page title, centered below the data fields (cyan, WT top 75%).
  r.fillText(centerL + centerW * 0.5f, barH * 0.75f,
             title, mfdFontPx(kPageTitleWt, h), TextAlign::Center, colors::kCyan);
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
      // Drops to 2 when the airframe has no weather radar (the Weather Radar
      // page, last in kMapPages, is not offered then).
      pageCount = ui.pageCount(MfdPageGroup::Map);
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
  render::drawSoftkeyBarBackground(r, w, top, barH,
                                   MfdController::kSoftkeyCount);

  const float cellW = w / static_cast<float>(MfdController::kSoftkeyCount);
  const float size = mfdFontPx(kSoftkeyFontWt, h);
  for (int i = 0; i < MfdController::kSoftkeyCount; ++i) {
    const float level =
        std::max(ui.pressLevel(i), ui.keyActive(i) ? 1.0f : 0.0f);
    render::drawSoftkeyCell(r, static_cast<float>(i) * cellW, top, cellW, barH,
                            ui.label(i), level, colors::kWhite, size);
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
                                  const MfdController& ui,
                                  const SoftkeyController& radios, int widthPx,
                                  int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float topBarH = h * (kTopBarHeightPx / mfd::kCanvasHeight);
  const float bottomBarH = h * (kBottomBarHeightPx / mfd::kCanvasHeight);

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  // The EIS engine strip owns the left edge on every page, like the real MFD;
  // the page body fills the remaining width.
  const float eisW = w * mfd::eisStripWidthFrac(eisLayout.style);
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
  drawNavComBar(r, w, h, topBarH, d, radios, title);
  drawSoftkeyBar(r, w, h, bottomBarH, ui);
}

}  // namespace avionics
