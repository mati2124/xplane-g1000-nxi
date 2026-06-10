#include "avionics/render/MultiFunctionDisplay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

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

// Group-prefixed page title shown in the navigation data bar, in the NXi's
// "GROUP - Page Name" style (WT NXi MFDUiPage titles). Page names match the
// G1000 Pilot's Guide for Cessna Nav III verbatim.
const char* pageTitle(MfdPage page) {
  switch (page) {
    case MfdPage::NavigationMap:
      return "MAP \xE2\x80\x93 NAVIGATION MAP";
    case MfdPage::AirportInformation:
      return "WPT \xE2\x80\x93 AIRPORT INFORMATION";
    case MfdPage::IntersectionInformation:
      return "WPT \xE2\x80\x93 INTERSECTION INFORMATION";
    case MfdPage::NdbInformation:
      return "WPT \xE2\x80\x93 NDB INFORMATION";
    case MfdPage::VorInformation:
      return "WPT \xE2\x80\x93 VOR INFORMATION";
    case MfdPage::TripPlanning:
      return "AUX \xE2\x80\x93 TRIP PLANNING";
    case MfdPage::GpsStatus:
      return "AUX \xE2\x80\x93 GPS STATUS";
    case MfdPage::SystemStatus:
      return "AUX \xE2\x80\x93 SYSTEM STATUS";
    case MfdPage::NearestAirports:
      return "NRST \xE2\x80\x93 NEAREST AIRPORTS";
    case MfdPage::NearestIntersections:
      return "NRST \xE2\x80\x93 NEAREST INTERSECTIONS";
    case MfdPage::NearestNdb:
      return "NRST \xE2\x80\x93 NEAREST NDB";
    case MfdPage::NearestVor:
      return "NRST \xE2\x80\x93 NEAREST VOR";
    case MfdPage::NearestAirspaces:
      return "NRST \xE2\x80\x93 NEAREST AIRSPACES";
    case MfdPage::ActiveFlightPlan:
      return "FPL \xE2\x80\x93 ACTIVE FLIGHT PLAN";
  }
  return "";
}

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

// Page group / page indicator in the lower-right corner of the page body: the
// five group tabs with the selected group highlighted, and one box per page in
// that group with the selected page filled (G1000 Pilot's Guide for Cessna
// Nav III, Figure 1-22).
void drawPageIndicator(Renderer& r, float w, float h, float bottomBarTop,
                       const MfdController& ui, int checklistPageCount) {
  struct GroupTab {
    const char* label;
    MfdPageGroup group;
  };
  const GroupTab tabs[] = {{"MAP", MfdPageGroup::Map},
                           {"WPT", MfdPageGroup::Waypoint},
                           {"AUX", MfdPageGroup::Aux},
                           {"FPL", MfdPageGroup::FlightPlan},
                           {"NRST", MfdPageGroup::Nearest},
                           {"CHK", MfdPageGroup::Checklist}};

  const float labelSize = mfdFontPx(13.0f, h);
  const float boxH = labelSize * 2.6f;
  const float boxW = w * 0.215f;
  const float bx = w - boxW - w * 0.008f;
  const float by = bottomBarTop - boxH - h * 0.006f;

  r.fillRect(bx, by, boxW, boxH, mfdAlpha(colors::kBlack, 0.55f));
  const Point border[5] = {{bx, by},
                           {bx + boxW, by},
                           {bx + boxW, by + boxH},
                           {bx, by + boxH},
                           {bx, by}};
  r.strokePolyline(border, 5, 1.0f, colors::kPanelBorder);

  // Group tabs along the top of the box.
  const float tabY = by + boxH * 0.30f;
  const int tabCount = static_cast<int>(sizeof(tabs) / sizeof(tabs[0]));
  for (int i = 0; i < tabCount; ++i) {
    const float cx =
        bx + boxW * (static_cast<float>(i) + 0.5f) / static_cast<float>(tabCount);
    const bool active = ui.pageGroup() == tabs[i].group;
    r.fillText(cx, tabY, tabs[i].label, labelSize, TextAlign::Center,
               active ? colors::kCyan : colors::kLabelText);
  }

  // One box per page of the selected group, selected page filled. The
  // Checklist group's count is dynamic (one page per loaded checklist).
  const int pages = ui.pageGroup() == MfdPageGroup::Checklist
                        ? std::max(1, checklistPageCount)
                        : MfdController::pageCount(ui.pageGroup());
  const float squares = labelSize * 0.62f;
  const float gap = squares * 0.55f;
  const float rowW = pages * squares + (pages - 1) * gap;
  float sx = bx + (boxW - rowW) * 0.5f;
  const float sy = by + boxH * 0.72f - squares * 0.5f;
  for (int i = 0; i < pages; ++i) {
    if (i == ui.pageIndex()) {
      r.fillRect(sx, sy, squares, squares, colors::kCyan);
    } else {
      const Point box[5] = {{sx, sy},
                            {sx + squares, sy},
                            {sx + squares, sy + squares},
                            {sx, sy + squares},
                            {sx, sy}};
      r.strokePolyline(box, 5, 1.0f, colors::kLabelText);
    }
    sx += squares + gap;
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

}  // namespace

void MultiFunctionDisplay::render(Renderer& r, const FlightData& d,
                                  const MapData& map,
                                  const ChecklistData& checklist,
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
    title = "CHKLIST \xE2\x80\x93 CHECKLIST";
  } else {
  switch (ui.page()) {
    case MfdPage::NavigationMap:
      mfd::drawMapPage(r, d, map, ui, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::AirportInformation:
      mfd::drawWaypointPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::IntersectionInformation:
      mfd::drawWaypointNavaidPage(r, d, map, MapFeatureType::Fix, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::NdbInformation:
      mfd::drawWaypointNavaidPage(r, d, map, MapFeatureType::Ndb, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::VorInformation:
      mfd::drawWaypointNavaidPage(r, d, map, MapFeatureType::Vor, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::TripPlanning:
      mfd::drawTripPlanningPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::GpsStatus:
      mfd::drawGpsStatusPage(r, d, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::SystemStatus:
      mfd::drawSystemStatusPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestAirports:
      mfd::drawNearestAirportsPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::NearestIntersections:
      mfd::drawNearestFeaturePage(r, d, map, MapFeatureType::Fix, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::NearestNdb:
      mfd::drawNearestFeaturePage(r, d, map, MapFeatureType::Ndb, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::NearestVor:
      mfd::drawNearestFeaturePage(r, d, map, MapFeatureType::Vor, bodyX, bodyY,
                                  bodyW, bodyH, h);
      break;
    case MfdPage::NearestAirspaces:
      mfd::drawNearestAirspacesPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
    case MfdPage::ActiveFlightPlan:
      mfd::drawActiveFlightPlanPage(r, d, map, bodyX, bodyY, bodyW, bodyH, h);
      break;
  }
  title = pageTitle(ui.page());
  }

  mfd::drawEisStrip(r, d, mfd::Rect{0.0f, bodyY, eisW, bodyH}, h);
  drawPageIndicator(r, w, h, h - bottomBarH, ui, checklist.totalChecklists());
  drawNavComBar(r, w, h, topBarH, d, title);
  drawSoftkeyBar(r, w, h, bottomBarH, ui);
}

}  // namespace avionics
