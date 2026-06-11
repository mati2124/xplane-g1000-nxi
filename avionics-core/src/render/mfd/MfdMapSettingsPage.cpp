#include "render/mfd/MfdMapSettings.h"

#include <cstdio>
#include <string>

#include "avionics/Color.h"
#include "avionics/MapRange.h"
#include "render/mfd/MfdPages.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

using S = MapSetting;
using K = MsKind;

// Map group, verbatim from Pilot's Guide Fig. 5-7 (label / left control / right
// value). The look-ahead times, Auto Zoom max and Fuel Range reserve are
// dependent read-outs on the real unit (no carets), so they are display-only
// Value cells the cursor skips.
constexpr MsRow kMapRows[] = {
    {"Orientation", false, {K::Enum, S::Orientation}, {}},
    {"North Up Above", false, {K::Toggle, S::NorthUpAboveOn},
     {K::Range, S::NorthUpAboveRange}},
    {"Terrain Display", false, {K::Enum, S::TerrainMode},
     {K::Range, S::TerrainRange}},
    {"Topo Scale", false, {K::Toggle, S::TopoScaleOn}, {}},
    {"Obstacle Data", false, {K::Toggle, S::ObstacleOn},
     {K::Range, S::ObstacleRange}},
    {"Auto Zoom", false, {K::Toggle, S::AutoZoomOn}, {K::Value, S::AutoZoomMax}},
    {"Max Look FWD", true, {}, {K::Value, S::MaxLookFwd}},
    {"Min Look FWD", true, {}, {K::Value, S::MinLookFwd}},
    {"Time Out", true, {}, {K::Value, S::TimeOut}},
    {"Track Vector", false, {K::Toggle, S::TrackVectorOn},
     {K::Value, S::TrackVectorTime}},
    {"Select ALT Arc", false, {K::Toggle, S::AltArcOn}, {}},
    {"Wind Vector", false, {K::Toggle, S::WindVectorOn}, {}},
    {"Fuel Range (RSV)", false, {K::Toggle, S::FuelRangeOn},
     {K::Value, S::FuelRangeRsv}},
    {"Field of View", false, {K::Toggle, S::FieldOfViewOn}, {}},
};

// Weather / Traffic / Aviation / Airspace / Land groups, from the WT NXi
// MFDMapSettings setting groups (the guide only figures the Map group).
constexpr MsRow kWeatherRows[] = {
    {"NEXRAD Data", false, {K::Toggle, S::NexradOn}, {K::Range, S::NexradRange}},
};

constexpr MsRow kTrafficRows[] = {
    {"Traffic", false, {K::Toggle, S::TrafficOn}, {}},
    {"Traffic Mode", false, {K::Enum, S::TrafficMode}, {}},
    {"Traffic Symbols", false, {}, {K::Range, S::TrafficSymbolsRange}},
    {"Traffic Labels", false, {K::Toggle, S::TrafficLabelsOn},
     {K::Range, S::TrafficLabelsRange}},
};

constexpr MsRow kAviationRows[] = {
    {"Large Airport", false, {K::Toggle, S::LargeAirportOn},
     {K::Range, S::LargeAirportRange}},
    {"Medium Airport", false, {K::Toggle, S::MediumAirportOn},
     {K::Range, S::MediumAirportRange}},
    {"Small Airport", false, {K::Toggle, S::SmallAirportOn},
     {K::Range, S::SmallAirportRange}},
    {"INT", false, {K::Toggle, S::IntOn}, {K::Range, S::IntRange}},
    {"NDB", false, {K::Toggle, S::NdbOn}, {K::Range, S::NdbRange}},
    {"VOR", false, {K::Toggle, S::VorOn}, {K::Range, S::VorRange}},
};

constexpr MsRow kAirspaceRows[] = {
    {"Class B/TMA", false, {K::Toggle, S::ClassBOn}, {K::Range, S::ClassBRange}},
    {"Class C/TCA", false, {K::Toggle, S::ClassCOn}, {K::Range, S::ClassCRange}},
    {"Class D", false, {K::Toggle, S::ClassDOn}, {K::Range, S::ClassDRange}},
    {"Restricted", false, {K::Toggle, S::RestrictedOn},
     {K::Range, S::RestrictedRange}},
    {"MOA (Military)", false, {K::Toggle, S::MoaOn}, {K::Range, S::MoaRange}},
    {"Other/ADIZ", false, {K::Toggle, S::OtherOn}, {K::Range, S::OtherRange}},
};

constexpr MsRow kLandRows[] = {
    {"User Waypoint", false, {K::Toggle, S::UserWaypointOn},
     {K::Range, S::UserWaypointRange}},
};

template <int N>
constexpr int rowCount(const MsRow (&)[N]) {
  return N;
}

bool fieldEditable(const MsField& f) {
  return f.kind == K::Toggle || f.kind == K::Enum || f.kind == K::Range;
}

}  // namespace

const MsRow* msRows(MapSettingsGroup group, int& count) {
  switch (group) {
    case MapSettingsGroup::Map:
      count = rowCount(kMapRows);
      return kMapRows;
    case MapSettingsGroup::Weather:
      count = rowCount(kWeatherRows);
      return kWeatherRows;
    case MapSettingsGroup::Traffic:
      count = rowCount(kTrafficRows);
      return kTrafficRows;
    case MapSettingsGroup::Aviation:
      count = rowCount(kAviationRows);
      return kAviationRows;
    case MapSettingsGroup::Airspace:
      count = rowCount(kAirspaceRows);
      return kAirspaceRows;
    case MapSettingsGroup::Land:
      count = rowCount(kLandRows);
      return kLandRows;
  }
  count = 0;
  return nullptr;
}

const char* msGroupName(MapSettingsGroup group) {
  switch (group) {
    case MapSettingsGroup::Map:
      return "Map";
    case MapSettingsGroup::Weather:
      return "Weather";
    case MapSettingsGroup::Traffic:
      return "Traffic";
    case MapSettingsGroup::Aviation:
      return "Aviation";
    case MapSettingsGroup::Airspace:
      return "Airspace";
    case MapSettingsGroup::Land:
      return "Land";
  }
  return "";
}

MsKind msControlKind(MapSetting id) {
  for (int g = 0; g <= static_cast<int>(MapSettingsGroup::Land); ++g) {
    int count = 0;
    const MsRow* rows = msRows(static_cast<MapSettingsGroup>(g), count);
    for (int i = 0; i < count; ++i) {
      if (rows[i].left.kind != K::None && rows[i].left.id == id) {
        return rows[i].left.kind;
      }
      if (rows[i].right.kind != K::None && rows[i].right.id == id) {
        return rows[i].right.kind;
      }
    }
  }
  return K::None;
}

int msEditableFields(MapSettingsGroup group, MapSetting* out, int maxOut) {
  int count = 0;
  const MsRow* rows = msRows(group, count);
  int n = 0;
  for (int i = 0; i < count; ++i) {
    if (fieldEditable(rows[i].left) && n < maxOut) out[n++] = rows[i].left.id;
    if (fieldEditable(rows[i].right) && n < maxOut) out[n++] = rows[i].right.id;
  }
  return n;
}

namespace {

// Small cyan caret flanking a toggle value (the ‹ › in Fig. 5-7). The tip sits
// at xTip; dir +1 draws a left-pointing caret (base to the right), dir -1 a
// right-pointing caret (base to the left). Centered vertically on cy.
void drawCaret(Renderer& r, float xTip, float cy, float size, int dir) {
  const float w = size * 0.34f;
  const float hh = size * 0.40f;
  const Point tri[3] = {{xTip, cy},
                        {xTip + dir * w, cy - hh},
                        {xTip + dir * w, cy + hh}};
  r.fillPolygon(tri, 3, colors::kCyan);
}

// One toggle control: ‹ value › with the carets in cyan and the On/Off text
// pulsing as a highlight-select field while the cursor is on it. The carets are
// kept a clear gap outside the value text so they never overlap it.
void drawToggleControl(Renderer& r, float x, float cy, const std::string& text,
                       float size, bool highlighted, bool blinkOn) {
  const float caretW = size * 0.34f;
  const float gap = size * 0.32f;
  const float textX = x + caretW + gap;
  const float tw = r.measureTextWidth(text, size);
  drawCaret(r, x, cy, size, +1);  // left caret, tip at x, points left
  if (highlighted) {
    drawCursorSelect(r, textX, cy, text, size, TextAlign::Left, blinkOn);
  } else {
    r.fillText(textX, cy, text, size, TextAlign::Left, colors::kCyan);
  }
  // Right caret base a gap past the text end; tip a caret-width beyond that.
  drawCaret(r, textX + tw + gap + caretW, cy, size, -1);
}

}  // namespace

void drawMapSettingsWindow(Renderer& r, const MfdController& ui, float x,
                           float y, float w, float h, float displayH) {
  const FontScope fs(r, FontFace::DejaVuSemiBold);

  const MapSettingsGroup group = ui.mapSettingsGroup();
  int rowCnt = 0;
  const MsRow* rows = msRows(group, rowCnt);

  const float titleSize = mfdFontPx(16.0f, displayH);
  const float labelSize = mfdFontPx(15.0f, displayH);
  const float valueSize = mfdFontPx(16.0f, displayH);
  const float footSize = mfdFontPx(13.0f, displayH);
  const float rowH = valueSize * 1.55f;
  const float pad = mfdFontPx(10.0f, displayH);

  // Group selector box height (drawGroupBox overhang + a single field row).
  const float groupSlotH = mfdFontPx(14.0f, displayH) * 0.55f + pad +
                           valueSize * 1.4f + pad * 0.6f;

  const float boxW = w * 0.38f;
  const float titleBandH = titleSize * 1.7f;
  const float footBandH = footSize * 2.4f;
  const float boxH = pad + titleBandH + groupSlotH + pad * 0.6f +
                     rowCnt * rowH + footBandH + pad;

  const float margin = mfdFontPx(6.0f, displayH);
  const float bx = x + w - boxW - margin;
  const float by = y + margin;

  // Popout chrome: gray rounded body with a thick light-grey rounded border and
  // a cyan centered title (the real Map Settings window body is panel-gray, not
  // the black of the page-menu popout).
  const float radius = mfdFontPx(10.0f, displayH);
  const float borderW = mfdFontPx(3.0f, displayH);
  r.fillRoundedRect(bx, by, boxW, boxH, radius, colors::kMfdPanelGray);
  r.strokeRoundedRect(bx + borderW * 0.5f, by + borderW * 0.5f, boxW - borderW,
                      boxH - borderW, radius, borderW, colors::kMenuBorderGray);

  float cy = by + pad + titleSize * 0.7f;
  r.fillText(bx + boxW * 0.5f, cy, "Map Settings", titleSize, TextAlign::Center,
             colors::kCyan);

  // Group selector group box with the active group's name. The field shows the
  // highlight-select cursor (a cyan bar) while the cursor is parked on it.
  const Rect groupSlot{bx + pad, by + pad + titleBandH, boxW - 2.0f * pad,
                       groupSlotH};
  const Rect groupInner = drawGroupBox(r, groupSlot, "Group", displayH);
  const float groupFieldCy = groupInner.y + valueSize * 0.7f;
  const bool onGroup = ui.mapSettingsCursor() == 0;
  if (onGroup && ui.blinkOn()) {
    r.fillRect(groupInner.x, groupFieldCy - valueSize * 0.7f, groupInner.w,
               valueSize * 1.4f, colors::kCyan);
    r.fillText(groupInner.x + pad * 0.4f, groupFieldCy, msGroupName(group),
               valueSize, TextAlign::Left, colors::kBlack);
  } else {
    r.fillText(groupInner.x + pad * 0.4f, groupFieldCy, msGroupName(group),
               valueSize, TextAlign::Left, colors::kCyan);
  }

  // Settings rows. Editable controls get sequential cursor indices starting at
  // 1 (the Group selector is 0), matching the controller's field walk.
  const float labelX = bx + pad * 1.4f;
  const float controlX = bx + boxW * 0.52f;
  const float valueRightX = bx + boxW - pad * 1.4f;
  float ry = groupSlot.y + groupSlotH + pad * 0.6f;
  int field = 1;
  const int cursor = ui.mapSettingsCursor();
  const bool blinkOn = ui.blinkOn();

  auto drawValueField = [&](const MsField& f, float xRight, float rcy,
                            bool editable) {
    const std::string text = ui.mapSettingText(f.id);
    const bool hi = editable && field == cursor;
    if (hi) {
      drawCursorSelect(r, xRight, rcy, text, valueSize, TextAlign::Right,
                       blinkOn);
    } else {
      r.fillText(xRight, rcy, text, valueSize, TextAlign::Right, colors::kCyan);
    }
  };

  for (int i = 0; i < rowCnt; ++i) {
    const MsRow& row = rows[i];
    const float rcy = ry + rowH * 0.5f;
    const float lx = row.indent ? labelX + pad * 1.6f : labelX;
    r.fillText(lx, rcy, row.label, labelSize, TextAlign::Left,
               colors::kTitleGray);

    // Left control column (Toggle / Enum).
    if (row.left.kind == K::Toggle) {
      const bool hi = field == cursor;
      drawToggleControl(r, controlX, rcy, ui.mapSettingText(row.left.id),
                        valueSize, hi, blinkOn);
      ++field;
    } else if (row.left.kind == K::Enum) {
      const std::string text = ui.mapSettingText(row.left.id);
      if (field == cursor) {
        drawCursorSelect(r, controlX, rcy, text, valueSize, TextAlign::Left,
                         blinkOn);
      } else {
        r.fillText(controlX, rcy, text, valueSize, TextAlign::Left,
                   colors::kCyan);
      }
      ++field;
    }

    // Right value column (Range editable, or a display-only Value).
    if (row.right.kind == K::Range) {
      drawValueField(row.right, valueRightX, rcy, /*editable=*/true);
      ++field;
    } else if (row.right.kind == K::Value) {
      drawValueField(row.right, valueRightX, rcy, /*editable=*/false);
    }

    ry += rowH;
  }

  r.fillText(bx + boxW * 0.5f, by + boxH - footBandH * 0.5f,
             "Press FMS Knob To Return", footSize, TextAlign::Center,
             colors::kTitleGray);
}

}  // namespace avionics::mfd
