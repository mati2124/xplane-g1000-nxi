#include "avionics/render/BootScreen.h"

#include <cstdio>
#include <string>

#include "avionics/Color.h"
#include "avionics/render/PrimaryFlightDisplay.h"
#include "render/mfd/MfdStyle.h"
#include "render/pfd/PfdInternal.h"

namespace avionics {
namespace {

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Layout fractions tuned against Garmin G1000 Pilot's Guide Figure 1-8 (MFD
// Power-Up Screen, 172R) and Section 1.3 (Cessna Nav III wording).

constexpr float kLogoWordmarkSize = 0.14f;
constexpr float kLogoTriangleHeight = 0.055f;

constexpr float kBottomBarFrac = 35.0f / 768.0f;

constexpr const char* kAirframeType = "Cessna 172S";
constexpr const char* kSystemVersion = "2026.1";
constexpr const char* kCopyright =
    "Copyright 2008-2026 Garmin Ltd. or its subsidiaries";

constexpr const char* kNavRowName = "Navigation";

struct StaticDatabaseRow {
  const char* name;
  const char* detail;
  bool warn;  // expired / caution (yellow detail text)
};

constexpr StaticDatabaseRow kStaticDatabases[] = {
    {"Checklist File", "N/A", false},
    {"Basemap Land", "4.00", false},
    {"SafeTaxi", "Expires 05-JUN-26", false},
    {"Terrain", "2.04", false},
    {"Airport Terrain", "2.04", false},
    {"Obstacle", "Expires 03-JUL-26", false},
    {kNavRowName, "-", false},
    {"Apt Directory", "Expires 03-JUL-26", false},
    {"Chart Data", "N/A", false},
};

FlightData sensorsFailedBootData(const FlightData& data) {
  FlightData d = data;
  // During PFD power-up the ADC/AHRS attitude and air-data fields are red-X'd
  // while the AHRS aligns, but the magnetometer-driven heading and HSI compass
  // rose remain drawn (NXi Maintenance Manual Fig 9-2).
  d.attitudeValid = false;
  d.headingValid = true;
  d.airspeedValid = false;
  d.altitudeValid = false;
  d.verticalSpeedValid = false;
  d.navSignalValid = false;
  d.windValid = false;
  d.bearing1Valid = false;
  d.bearing2Valid = false;
  d.dataLinkValid = false;
  // No marker-beacon annunciation or vertical-deviation (glideslope/glidepath)
  // guidance during power-up.
  d.markerBeacon = MarkerBeacon::None;
  d.vdiKind = VerticalDeviationKind::None;
  return d;
}

void drawStatusIcon(Renderer& r, float cx, float cy, float size, bool ok,
                    bool warn, float alpha) {
  if (ok) {
    const float s = size * 0.42f;
    const Color c = withAlpha(colors::kActiveGreen, alpha);
    r.strokeLine(cx - s, cy, cx - s * 0.2f, cy + s * 0.8f, 2.0f, c);
    r.strokeLine(cx - s * 0.2f, cy + s * 0.8f, cx + s, cy - s * 0.8f, 2.0f, c);
    return;
  }
  if (warn) {
    // Yellow caution triangle with exclamation mark.
    const float h = size * 0.9f;
    const float w = h * 0.95f;
    const Point tri[3] = {
        {cx, cy - h * 0.45f},
        {cx + w * 0.5f, cy + h * 0.45f},
        {cx - w * 0.5f, cy + h * 0.45f},
    };
    r.fillPolygon(tri, 3, withAlpha(colors::kBandYellow, alpha));
    r.fillText(cx, cy + h * 0.08f, "!", size * 0.38f, TextAlign::Center,
               withAlpha(colors::kBlack, alpha));
    return;
  }
  // Grey dash for N/A rows.
  r.fillText(cx, cy, "-", size * 0.55f, TextAlign::Center,
             withAlpha(colors::kTitleGray, alpha));
}

// Simplified side-view aircraft silhouette (Figure 1-8 center graphic).
void drawAircraftSilhouette(Renderer& r, float cx, float cy, float h,
                             float alpha) {
  const float scale = h * 0.34f;
  const Color body = withAlpha(Color{0.45f, 0.45f, 0.45f, 1.0f}, alpha);
  const Color wing = withAlpha(Color{0.38f, 0.38f, 0.38f, 1.0f}, alpha);

  const Point fuselage[8] = {
      {cx + scale * 0.55f, cy - scale * 0.06f},
      {cx + scale * 0.48f, cy - scale * 0.10f},
      {cx - scale * 0.35f, cy - scale * 0.08f},
      {cx - scale * 0.50f, cy},
      {cx - scale * 0.35f, cy + scale * 0.08f},
      {cx + scale * 0.30f, cy + scale * 0.10f},
      {cx + scale * 0.52f, cy + scale * 0.06f},
      {cx + scale * 0.55f, cy - scale * 0.06f},
  };
  r.fillPolygon(fuselage, 8, body);

  const Point wingPoly[4] = {
      {cx - scale * 0.05f, cy - scale * 0.02f},
      {cx + scale * 0.18f, cy - scale * 0.34f},
      {cx + scale * 0.28f, cy - scale * 0.30f},
      {cx + scale * 0.05f, cy + scale * 0.02f},
  };
  r.fillPolygon(wingPoly, 4, wing);

  const Point tail[3] = {
      {cx - scale * 0.42f, cy - scale * 0.02f},
      {cx - scale * 0.58f, cy - scale * 0.28f},
      {cx - scale * 0.38f, cy + scale * 0.02f},
  };
  r.fillPolygon(tail, 3, wing);
}

void drawGarminG1000Brand(Renderer& r, float cx, float topY, float h,
                          float alpha) {
  const float brandSize = h * 0.072f;
  const float g1000Size = h * 0.034f;
  const float wordW = r.measureTextWidth("GARMIN", brandSize);
  const float gap = h * 0.012f;
  const float totalW = wordW + gap + r.measureTextWidth("G1000", g1000Size) +
                       h * 0.028f;
  const float left = cx - totalW * 0.5f;
  r.fillText(left + wordW * 0.5f, topY, "GARMIN", brandSize, TextAlign::Center,
             withAlpha(colors::kWhite, alpha));
  const float g1000X = left + wordW + gap + r.measureTextWidth("G1000", g1000Size) * 0.5f;
  r.fillText(g1000X, topY + brandSize * 0.08f, "G1000", g1000Size,
             TextAlign::Center, withAlpha(colors::kLabelText, alpha));
  const float triH = g1000Size * 1.1f;
  const float triW = triH * 0.95f;
  const float triX = g1000X + r.measureTextWidth("G1000", g1000Size) * 0.5f + triW * 0.35f;
  const float triTop = topY - brandSize * 0.15f;
  const Point tri[3] = {
      {triX, triTop},
      {triX + triW * 0.5f, triTop + triH},
      {triX - triW * 0.5f, triTop + triH},
  };
  r.fillPolygon(tri, 3, withAlpha(colors::kGarminLogoBlue, alpha));
}

void drawMfdBootSoftkeys(Renderer& r, float w, float h, float barH,
                         const MfdController& ui, float alpha) {
  const float top = h - barH;
  r.fillRect(0.0f, top, w, barH, withAlpha(colors::kSoftkeyBackground, alpha));

  const float cellW = w / static_cast<float>(MfdController::kSoftkeyCount);
  const float cy = top + barH * 0.5f;
  const float size = mfd::mfdFontPx(17.0f, h);
  const float insetX = cellW * 0.055f;
  const float insetY = barH * 0.13f;
  for (int i = 0; i < MfdController::kSoftkeyCount; ++i) {
    const float bx = static_cast<float>(i) * cellW + insetX;
    const float by = top + insetY;
    const float bw = cellW - 2.0f * insetX;
    const float bh = barH - 2.0f * insetY;

    const bool highlightEnt = (i == MfdController::kSoftkeyCount - 1);
    if (highlightEnt) {
      r.fillRect(bx, by, bw, bh,
                 withAlpha(colors::kSoftkeySelected, alpha * 0.85f));
    }
    const Point frame[5] = {
        {bx, by}, {bx + bw, by}, {bx + bw, by + bh}, {bx, by + bh}, {bx, by}};
    r.strokePolyline(frame, 5, 1.0f, withAlpha(colors::kPanelSeparator, alpha));

    if (!ui.label(i).empty()) {
      r.fillText(bx + bw * 0.5f, cy, ui.label(i), size, TextAlign::Center,
                 highlightEnt ? withAlpha(colors::kBlack, alpha)
                              : withAlpha(colors::kWhite, alpha));
    }
  }
}

void renderLogo(Renderer& r, float alpha, int widthPx, int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float cx = w * 0.5f;
  const float cy = h * 0.5f;

  // The black background is opaque; only the logo marks fade up from it.
  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  const float wordSize = h * kLogoWordmarkSize;
  const float wordWidth = r.measureTextWidth("GARMIN", wordSize);
  r.fillText(cx, cy, "GARMIN", wordSize, TextAlign::Center,
             withAlpha(colors::kWhite, alpha));

  const float triH = h * kLogoTriangleHeight;
  const float triW = triH * 1.1f;
  const float triRight = cx + wordWidth * 0.5f + triW * 0.5f;
  const float triTop = cy - wordSize * 0.55f - triH;
  const Point tri[3] = {
      {triRight - triW * 0.5f, triTop},
      {triRight, triTop + triH},
      {triRight - triW, triTop + triH},
  };
  r.fillPolygon(tri, 3, withAlpha(colors::kGarminLogoBlue, alpha));
}

void renderPfdPowerUp(Renderer& r, const FlightData& flightData,
                      const MapData& map, const SoftkeyController& ui,
                      float alpha, int widthPx, int heightPx) {
  if (alpha <= 0.0f) return;

  // The power-up init view is never reversionary (no EIS strip on the PFD).
  PrimaryFlightDisplay::render(r, sensorsFailedBootData(flightData), map, ui,
                               EisLayout{}, /*reversionary=*/false, widthPx,
                               heightPx, /*powerUp=*/true);

  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const pfd::Layout L = pfd::computeLayout(w, h);

  // The PFD inset map is not shown during power-up (NXi Fig 9-2); the lower-left
  // stays black background while the HSI compass rose remains drawn.
  r.fillRect(L.insetMapX, L.insetMapY, L.insetMapW, L.insetMapH, colors::kBlack);
}

void renderMfdPowerUp(Renderer& r, const MfdController& mfdUi,
                      const NavDatabaseInfo& navDatabase, bool awaitingAck,
                      float alpha, int widthPx, int heightPx) {
  if (alpha <= 0.0f) return;

  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float barH = h * kBottomBarFrac;
  const float bodyH = h - barH;

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  drawGarminG1000Brand(r, w * 0.36f, bodyH * 0.11f, h, alpha);

  const float headerSize = h * 0.022f;
  const float smallSize = h * 0.017f;
  const Color headerColor = withAlpha(colors::kWhite, alpha);
  const Color mutedColor = withAlpha(colors::kLabelText, alpha);
  r.fillText(w * 0.72f, bodyH * 0.09f, kAirframeType, headerSize,
             TextAlign::Left, headerColor);
  char sysBuf[48];
  std::snprintf(sysBuf, sizeof(sysBuf), "System %s", kSystemVersion);
  r.fillText(w * 0.72f, bodyH * 0.13f, sysBuf, smallSize, TextAlign::Left,
             mutedColor);
  r.fillText(w * 0.72f, bodyH * 0.165f, kCopyright, smallSize * 0.88f,
             TextAlign::Left, mutedColor);

  drawAircraftSilhouette(r, w * 0.33f, bodyH * 0.52f, h, alpha);

  // Database list on the right (Figure 1-8).
  const float listLeft = w * 0.58f;
  const float listRight = w * 0.94f;
  const float rowSize = h * 0.024f;
  const float iconX = listLeft + rowSize * 0.55f;
  const float nameX = listLeft + rowSize * 1.35f;
  const float detailRight = listRight;

  r.fillText(listLeft, bodyH * 0.24f, "DATABASE", headerSize * 0.9f,
             TextAlign::Left, mutedColor);
  r.strokeLine(listLeft, bodyH * 0.265f, listRight, bodyH * 0.265f, 1.0f,
               withAlpha(colors::kPanelSeparator, alpha));

  const float rowStep = h * 0.048f;
  float rowY = bodyH * 0.305f;
  for (const StaticDatabaseRow& row : kStaticDatabases) {
    const float cy = rowY + rowStep * 0.42f;
    std::string detail = row.detail;
    bool warn = row.warn;
    bool ok = !warn && detail != "N/A" && detail[0] != '-';

    if (row.name == kNavRowName && navDatabase.available) {
      if (!navDatabase.expires.empty()) {
        detail = std::string("Expires ") + navDatabase.expires;
      } else if (!navDatabase.cycle.empty()) {
        detail = navDatabase.cycle;
      }
      warn = navDatabase.expired;
      ok = !navDatabase.expired;
    }

    const bool na = detail == "N/A";
    drawStatusIcon(r, iconX, cy, rowSize, ok, warn && !na, alpha);

    Color nameColor = withAlpha(colors::kWhite, alpha);
    Color detailColor = warn ? withAlpha(colors::kBandYellow, alpha)
                             : withAlpha(colors::kWhite, alpha);
    if (na) detailColor = mutedColor;

    r.fillText(nameX, cy, row.name, rowSize, TextAlign::Left, nameColor);
    r.fillText(detailRight, cy, detail.c_str(), rowSize * 0.92f,
               TextAlign::Right, detailColor);
    rowY += rowStep;
  }

  if (awaitingAck) {
    const float promptSize = h * 0.021f;
    r.fillText(listRight, bodyH * 0.90f,
               "Press 'ENT' or rightmost softkey to continue", promptSize,
               TextAlign::Right, withAlpha(colors::kWhite, alpha));
  }

  drawMfdBootSoftkeys(r, w, h, barH, mfdUi, alpha);
}

}  // namespace

void BootScreen::render(Renderer& r, Target target, Phase phase,
                        const FlightData& flightData, const MapData& map,
                        const SoftkeyController& pfdUi,
                        const MfdController& mfdUi,
                        const NavDatabaseInfo& navDatabase, bool awaitingAck,
                        float phaseAlpha, int widthPx, int heightPx) {
  if (phase == Phase::Logo) {
    renderLogo(r, phaseAlpha, widthPx, heightPx);
    return;
  }

  if (target == Target::Pfd) {
    renderPfdPowerUp(r, flightData, map, pfdUi, phaseAlpha, widthPx, heightPx);
    return;
  }

  renderMfdPowerUp(r, mfdUi, navDatabase, awaitingAck, phaseAlpha, widthPx,
                   heightPx);
}

}  // namespace avionics
