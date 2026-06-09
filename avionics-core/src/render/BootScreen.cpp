#include "avionics/render/BootScreen.h"

#include "avionics/Color.h"

namespace avionics {

namespace {

// Faithful recreation of the Garmin G1000 MFD power-up page: the GARMIN
// wordmark, system/copyright lines, the installed-database table (Region /
// Cycle / Effective / Expires), and the bottom acknowledgement prompt that the
// real unit shows once initialization completes.

// All sizes / positions are fractions of the display so the screen scales with
// the window (the PFD uses the same approach via fontPx()).
constexpr float kBrandSize = 0.075f;
constexpr float kVersionSize = 0.020f;
constexpr float kCopyrightSize = 0.016f;
constexpr float kColumnHeaderSize = 0.019f;
constexpr float kRowSize = 0.024f;
constexpr float kPromptSize = 0.022f;

// Phase where the page switches from "INITIALIZING SYSTEM" to the database
// acknowledgement prompt, mirroring the real power-up sequence.
constexpr float kAcknowledgeAfter = 0.65f;

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

// Representative database set as listed on a Cessna Nav III G1000.
constexpr DatabaseRow kDatabases[] = {
    {"BASEMAP", "WORLDWIDE", "-", "-", "-"},
    {"SAFETAXI", "US", "26S2", "-", "-"},
    {"TERRAIN", "WORLDWIDE", "-", "-", "-"},
    {"AIRPORT TERRAIN", "WORLDWIDE", "-", "-", "-"},
    {"OBSTACLE", "US", "26B2", "05-JUN-26", "03-JUL-26"},
    {"AVIATION", "WORLDWIDE", "2606", "05-JUN-26", "03-JUL-26"},
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

}  // namespace

void BootScreen::render(Renderer& r, const std::string& sourceLabel,
                        float progress01, int widthPx, int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float cx = w * 0.5f;

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  // Brand + system identification.
  r.fillText(cx, h * 0.13f, "GARMIN", h * kBrandSize, TextAlign::Center,
             colors::kWhite);
  r.fillText(cx, h * 0.205f, "SYSTEM SOFTWARE VERSION 2026.1", h * kVersionSize,
             TextAlign::Center, colors::kLabelText);
  r.fillText(cx, h * 0.235f,
             "Copyright 2026 Garmin Ltd. or its subsidiaries", h * kCopyrightSize,
             TextAlign::Center, colors::kLabelText);

  // Divider above the database table.
  r.strokeLine(w * kColName, h * 0.27f, w * 0.90f, h * 0.27f, 1.0f,
               colors::kPanelBorder);

  // Database table header.
  const float headerY = h * 0.31f;
  r.fillText(w * kColName, headerY, "DATABASE", h * kColumnHeaderSize,
             TextAlign::Left, colors::kLabelText);
  r.fillText(w * kColRegion, headerY, "REGION", h * kColumnHeaderSize,
             TextAlign::Left, colors::kLabelText);
  r.fillText(w * kColCycle, headerY, "CYCLE", h * kColumnHeaderSize,
             TextAlign::Left, colors::kLabelText);
  r.fillText(w * kColEffective, headerY, "EFFECTIVE", h * kColumnHeaderSize,
             TextAlign::Left, colors::kLabelText);
  r.fillText(w * kColExpires, headerY, "EXPIRES", h * kColumnHeaderSize,
             TextAlign::Left, colors::kLabelText);
  r.strokeLine(w * kColName, h * 0.335f, w * 0.90f, h * 0.335f, 1.0f,
               colors::kPanelSeparator);

  // Database rows.
  const float rowStep = h * 0.052f;
  float rowY = h * 0.375f;
  for (int i = 0; i < kDatabaseCount; ++i) {
    const DatabaseRow& d = kDatabases[i];
    r.fillText(w * kColName, rowY, d.name, h * kRowSize, TextAlign::Left,
               colors::kWhite);
    r.fillText(w * kColRegion, rowY, d.region, h * kRowSize, TextAlign::Left,
               colors::kWhite);
    r.fillText(w * kColCycle, rowY, d.cycle, h * kRowSize, TextAlign::Left,
               colors::kWhite);
    r.fillText(w * kColEffective, rowY, d.effective, h * kRowSize,
               TextAlign::Left, colors::kWhite);
    r.fillText(w * kColExpires, rowY, d.expires, h * kRowSize, TextAlign::Left,
               colors::kWhite);
    rowY += rowStep;
  }

  // Bottom status / acknowledgement prompt in the cyan-bordered box.
  const float boxW = w * 0.62f;
  const float boxH = h * 0.075f;
  const float boxX = cx - boxW * 0.5f;
  const float boxY = h * 0.83f;
  r.fillRect(boxX, boxY, boxW, boxH, colors::kPanelBackground);
  strokeBox(r, boxX, boxY, boxW, boxH, 2.0f, colors::kCyan);

  const float boxCenterY = boxY + boxH * 0.5f;
  if (progress01 < kAcknowledgeAfter) {
    r.fillText(cx, boxCenterY, "INITIALIZING SYSTEM", h * kPromptSize,
               TextAlign::Center, colors::kWhite);
  } else {
    r.fillText(cx, boxCenterY - h * 0.012f,
               "Press the ENT Key to acknowledge", h * kPromptSize,
               TextAlign::Center, colors::kWhite);
    r.fillText(cx, boxCenterY + h * 0.016f, "all database information",
               h * kPromptSize, TextAlign::Center, colors::kWhite);
  }

  // Discreet footer noting the active data feed (mock vs live). Not part of the
  // real Garmin page, but useful while developing/iterating on the connection.
  if (!sourceLabel.empty()) {
    r.fillText(cx, h * 0.965f, sourceLabel, h * 0.015f, TextAlign::Center,
               colors::kPanelSeparator);
  }
}

}  // namespace avionics
