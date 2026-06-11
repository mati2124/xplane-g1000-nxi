#include "avionics/render/BootScreen.h"

#include "avionics/Color.h"

namespace avionics {

namespace {

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Faithful recreation of the Garmin G1000 MFD power-up page: the GARMIN
// wordmark, system/copyright lines, the installed-database table (Region /
// Cycle / Effective / Expires), and the bottom acknowledgement prompt that the
// real unit shows once initialization completes.

// All sizes / positions are fractions of the display so the screen scales with
// the window (the PFD uses the same approach via fontPx()).

// Logo splash: the centered Garmin wordmark plus its blue triangle trademark,
// sized as fractions of the display height.
constexpr float kLogoWordmarkSize = 0.14f;
constexpr float kLogoTriangleHeight = 0.055f;

constexpr float kBrandSize = 0.075f;
constexpr float kVersionSize = 0.020f;
constexpr float kCopyrightSize = 0.016f;
constexpr float kColumnHeaderSize = 0.019f;
constexpr float kRowSize = 0.024f;
constexpr float kPromptSize = 0.022f;

// Column anchors (fractions of width), left-aligned like the real grid.
constexpr float kColName = 0.10f;
constexpr float kColRegion = 0.44f;
constexpr float kColCycle = 0.60f;
constexpr float kColEffective = 0.70f;
constexpr float kColExpires = 0.85f;

struct DatabaseRow {
  const char* name;
  const char* region;
  const char* cycle;
  const char* effective;
  const char* expires;
};

// Representative database set as listed on a Cessna Nav III G1000. The
// AVIATION row is filled from the live NavDatabaseInfo at render time; the
// rest are static placeholders (those databases have no live backing store).
constexpr const char* kAviationName = "AVIATION";
constexpr DatabaseRow kDatabases[] = {
    {"BASEMAP", "WORLDWIDE", "-", "-", "-"},
    {"SAFETAXI", "US", "26S2", "-", "-"},
    {"TERRAIN", "WORLDWIDE", "-", "-", "-"},
    {"AIRPORT TERRAIN", "WORLDWIDE", "-", "-", "-"},
    {"OBSTACLE", "US", "26B2", "05-JUN-26", "03-JUL-26"},
    {kAviationName, "WORLDWIDE", "-", "-", "-"},
    {"AIRPORT DIRECTORY", "US", "26D2", "05-JUN-26", "03-JUL-26"},
};
constexpr int kDatabaseCount =
    static_cast<int>(sizeof(kDatabases) / sizeof(kDatabases[0]));

void strokeBox(Renderer& r, float x, float y, float w, float h, float widthPx,
               const Color& c) {
  r.strokeLine(x, y, x + w, y, widthPx, c);
  r.strokeLine(x, y + h, x + w, y + h, widthPx, c);
  r.strokeLine(x, y, x, y + h, widthPx, c);
  r.strokeLine(x + w, y, x + w, y + h, widthPx, c);
}

// Phase 1 of power-up: the centered Garmin logo on black, shown before the
// database Power-up Page (G1000 NXi: a Garmin logo splash appears first). The
// wordmark is drawn as text and the blue triangle trademark sits just above its
// trailing edge, as on the corporate logo.
void renderLogo(Renderer& r, int widthPx, int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float cx = w * 0.5f;
  const float cy = h * 0.5f;

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  const float wordSize = h * kLogoWordmarkSize;
  const float wordWidth = r.measureTextWidth("GARMIN", wordSize);
  r.fillText(cx, cy, "GARMIN", wordSize, TextAlign::Center, colors::kWhite);

  // Blue triangle trademark, sitting above the wordmark's trailing edge.
  const float triH = h * kLogoTriangleHeight;
  const float triW = triH * 1.1f;
  const float triRight = cx + wordWidth * 0.5f + triW * 0.5f;
  const float triTop = cy - wordSize * 0.55f - triH;
  const Point tri[3] = {
      {triRight - triW * 0.5f, triTop},      // apex
      {triRight, triTop + triH},             // bottom-right
      {triRight - triW, triTop + triH},      // bottom-left
  };
  r.fillPolygon(tri, 3, colors::kGarminLogoBlue);
}

// Phase 2: MFD Power-up Page. Fades in over kBootPowerUpFadeSeconds (the logo
// is removed at the start of the fade, as on the real unit).
void renderPowerUp(Renderer& r, const std::string& sourceLabel,
                   const NavDatabaseInfo& navDatabase, bool awaitingAck,
                   float alpha, int widthPx, int heightPx) {
  if (alpha <= 0.0f) return;

  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float cx = w * 0.5f;

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  // Brand + system identification.
  r.fillText(cx, h * 0.13f, "GARMIN", h * kBrandSize, TextAlign::Center,
             withAlpha(colors::kWhite, alpha));
  r.fillText(cx, h * 0.205f, "SYSTEM SOFTWARE VERSION 2026.1", h * kVersionSize,
             TextAlign::Center, withAlpha(colors::kLabelText, alpha));
  r.fillText(cx, h * 0.235f,
             "Copyright 2026 Garmin Ltd. or its subsidiaries", h * kCopyrightSize,
             TextAlign::Center, withAlpha(colors::kLabelText, alpha));

  // Divider above the database table.
  r.strokeLine(w * kColName, h * 0.27f, w * 0.90f, h * 0.27f, 1.0f,
               withAlpha(colors::kPanelBorder, alpha));

  // Database table header.
  const float headerY = h * 0.31f;
  r.fillText(w * kColName, headerY, "DATABASE", h * kColumnHeaderSize,
             TextAlign::Left, withAlpha(colors::kLabelText, alpha));
  r.fillText(w * kColRegion, headerY, "REGION", h * kColumnHeaderSize,
             TextAlign::Left, withAlpha(colors::kLabelText, alpha));
  r.fillText(w * kColCycle, headerY, "CYCLE", h * kColumnHeaderSize,
             TextAlign::Left, withAlpha(colors::kLabelText, alpha));
  r.fillText(w * kColEffective, headerY, "EFFECTIVE", h * kColumnHeaderSize,
             TextAlign::Left, withAlpha(colors::kLabelText, alpha));
  r.fillText(w * kColExpires, headerY, "EXPIRES", h * kColumnHeaderSize,
             TextAlign::Left, withAlpha(colors::kLabelText, alpha));
  r.strokeLine(w * kColName, h * 0.335f, w * 0.90f, h * 0.335f, 1.0f,
               withAlpha(colors::kPanelSeparator, alpha));

  // Database rows. The AVIATION (navigation database) row reflects the loaded
  // data's actual AIRAC cycle; when that cycle has expired, the row is shown
  // in amber so the pilot is alerted while reviewing currency, matching the
  // real power-up page (G1000 NXi Pilot's Guide, Appendix B).
  const float rowStep = h * 0.052f;
  float rowY = h * 0.375f;
  for (int i = 0; i < kDatabaseCount; ++i) {
    DatabaseRow d = kDatabases[i];
    Color rowColor = colors::kWhite;
    if (d.name == kAviationName && navDatabase.available) {
      d.cycle = navDatabase.cycle.c_str();
      d.effective = navDatabase.effective.c_str();
      d.expires = navDatabase.expires.c_str();
      if (navDatabase.expired) rowColor = colors::kBandYellow;
    }
    rowColor = withAlpha(rowColor, alpha);
    r.fillText(w * kColName, rowY, d.name, h * kRowSize, TextAlign::Left,
               rowColor);
    r.fillText(w * kColRegion, rowY, d.region, h * kRowSize, TextAlign::Left,
               rowColor);
    r.fillText(w * kColCycle, rowY, d.cycle, h * kRowSize, TextAlign::Left,
               rowColor);
    r.fillText(w * kColEffective, rowY, d.effective, h * kRowSize,
               TextAlign::Left, rowColor);
    r.fillText(w * kColExpires, rowY, d.expires, h * kRowSize, TextAlign::Left,
               rowColor);
    rowY += rowStep;
  }

  // Bottom status / acknowledgement prompt in the cyan-bordered box.
  const float boxW = w * 0.62f;
  const float boxH = h * 0.075f;
  const float boxX = cx - boxW * 0.5f;
  const float boxY = h * 0.83f;
  r.fillRect(boxX, boxY, boxW, boxH, withAlpha(colors::kPanelBackground, alpha));
  strokeBox(r, boxX, boxY, boxW, boxH, 2.0f, withAlpha(colors::kCyan, alpha));

  const float boxCenterY = boxY + boxH * 0.5f;
  const Color promptColor = withAlpha(colors::kWhite, alpha);
  if (!awaitingAck) {
    r.fillText(cx, boxCenterY, "INITIALIZING SYSTEM", h * kPromptSize,
               TextAlign::Center, promptColor);
  } else {
    r.fillText(cx, boxCenterY - h * 0.012f,
               "Press the ENT Key to acknowledge", h * kPromptSize,
               TextAlign::Center, promptColor);
    r.fillText(cx, boxCenterY + h * 0.016f, "all database information",
               h * kPromptSize, TextAlign::Center, promptColor);
  }

  // Discreet footer noting the active data feed (mock vs live). Not part of the
  // real Garmin page, but useful while developing/iterating on the connection.
  if (!sourceLabel.empty()) {
    r.fillText(cx, h * 0.965f, sourceLabel, h * 0.015f, TextAlign::Center,
               withAlpha(colors::kPanelSeparator, alpha));
  }
}

}  // namespace

void BootScreen::render(Renderer& r, Phase phase, const std::string& sourceLabel,
                        const NavDatabaseInfo& navDatabase, bool awaitingAck,
                        float powerUpAlpha, int widthPx, int heightPx) {
  if (phase == Phase::Logo) {
    renderLogo(r, widthPx, heightPx);
    return;
  }

  renderPowerUp(r, sourceLabel, navDatabase, awaitingAck, powerUpAlpha, widthPx,
                heightPx);
}

}  // namespace avionics
