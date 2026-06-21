#include "avionics/render/BootScreen.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "avionics/AssetPaths.h"
#include "avionics/AircraftProfile.h"
#include "avionics/Color.h"
#include "avionics/Version.h"
#include "avionics/render/PrimaryFlightDisplay.h"
#include "render/pfd/PfdInternal.h"

#if defined(AVIONICS_HAS_STBIMAGE)
#define STBI_NO_STDIO
#include "stb_image.h"
#endif

namespace avionics {
namespace {

Color withAlpha(Color c, float a) {
  c.a *= a;
  return c;
}

// Layout tuned against the Cessna NAV III Trainer v20.05 MFD power-up screen
// (Cessna 172S) and Pilot's Guide Figure 1-8.

constexpr float kLogoWordmarkSize = 0.14f;
constexpr float kLogoTriangleHeight = 0.055f;

// MFD power-up left panel (G1000 NXi logo + hero art) measured from
// screenshot001.bmp at 1024×768 — one bitmap, not logo + hero separately.
constexpr float kTrainerRefWidthPx = 1024.0f;
constexpr float kTrainerRefHeightPx = 768.0f;
constexpr float kTrainerHeroX = 20.0f;
constexpr float kTrainerHeroY = 12.0f;
constexpr float kTrainerHeroW = 581.0f;
constexpr float kTrainerHeroH = 737.0f;

// Database list column measured from screenshot001.bmp at 1024×768.
constexpr float kTrainerIconXPx = 632.0f;
constexpr float kTrainerLabelXPx = 708.0f;
constexpr float kTrainerListRightPx = 828.0f;
constexpr float kTrainerFirstRowYPx = 58.0f;
constexpr float kTrainerRowStepPx = 45.0f;
constexpr float kTrainerLabelSizePx = 14.0f;
constexpr float kTrainerValueSizePx = 12.0f;
constexpr float kTrainerIconSizePx = 20.0f;
// Extra downward offset for the checklist path (trainer ref at 1024×768).
constexpr float kChecklistPathExtraDownPx = 10.0f;

// Only the first database row after the airframe line (Checklist File).
constexpr std::size_t kVisibleDatabaseRows = 1;

enum class BootIcon {
  Airframe,
  CheckOk,
  Basemap,
  SafeTaxi,
  Terrain,
  Obstacle,
  Navigation,
  AptDir,
  FliteCharts,
  Charts,
};

struct BootDbRow {
  BootIcon icon;
  const char* label;
  const char* detail;
  bool warn;  // yellow label + detail (expired / disabled)
  bool navRow;
};

constexpr BootDbRow kDatabaseRows[] = {
    {BootIcon::CheckOk, "Checklist File", "N/A", false, false},
    {BootIcon::Basemap, "Basemap Land", "5.13", false, false},
    {BootIcon::SafeTaxi, "SafeTaxi Data", "Expires 29-MAR-2018", true, false},
    {BootIcon::Terrain, "Terrain Data", "3.00", false, false},
    {BootIcon::Obstacle, "Obstacle Data", "Expires 29-MAR-2018", true, false},
    {BootIcon::Navigation, "Navigation Data", "", false, true},
    {BootIcon::AptDir, "Apt Directory", "Expires 29-MAR-2018", true, false},
    {BootIcon::FliteCharts, "FliteCharts Data", "Disabled", true, false},
    {BootIcon::Charts, "IFR/VFR charts", "Date: 5-JAN-2017", false, false},
};

struct BootRaster {
  int imageId = -1;
  int widthPx = 0;
  int heightPx = 0;
};

struct BootRasterCache {
  BootRaster hero;
  std::string heroPath;
};

BootRasterCache gBootRasters;

std::string bootAssetDevFallback(const char* relative) {
  return std::string(AVIONICS_CORE_ASSET_DIR) + "/" + relative;
}

std::vector<unsigned char> readFileBytes(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  in.seekg(0, std::ios::end);
  const std::streamoff len = in.tellg();
  if (len <= 0) return {};
  std::vector<unsigned char> buf(static_cast<size_t>(len));
  in.seekg(0);
  in.read(reinterpret_cast<char*>(buf.data()), len);
  if (!in) return {};
  return buf;
}

BootRaster loadBootRasterFile(Renderer& r, const std::string& path) {
  BootRaster out;
#if defined(AVIONICS_HAS_STBIMAGE)
  const std::vector<unsigned char> bytes = readFileBytes(path);
  if (bytes.empty()) return out;

  int w = 0;
  int h = 0;
  unsigned char* rgba =
      stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w,
                            &h, nullptr, 4);
  if (rgba == nullptr || w <= 0 || h <= 0) {
    if (rgba != nullptr) stbi_image_free(rgba);
    return out;
  }

  out.imageId = r.createImageRGBA(w, h, rgba);
  out.widthPx = w;
  out.heightPx = h;
  stbi_image_free(rgba);
#endif
  return out;
}

BootRaster loadBootRaster(Renderer& r, const char* relativePath) {
  const std::string path =
      assets::resolve(relativePath, bootAssetDevFallback(relativePath));
  return loadBootRasterFile(r, path);
}

void ensureBootHero(Renderer& r, const std::string& heroPath) {
  const int trainerHeroW = static_cast<int>(kTrainerHeroW);
  const int trainerHeroH = static_cast<int>(kTrainerHeroH);

  if (heroPath.empty()) {
    if (gBootRasters.hero.imageId >= 0) {
      r.deleteImage(gBootRasters.hero.imageId);
      gBootRasters.hero = BootRaster{};
    }
    gBootRasters.heroPath.clear();
    return;
  }

  const bool cacheValid =
      heroPath == gBootRasters.heroPath && gBootRasters.hero.imageId >= 0 &&
      gBootRasters.hero.widthPx == trainerHeroW &&
      gBootRasters.hero.heightPx == trainerHeroH;
  if (cacheValid) return;

  if (gBootRasters.hero.imageId >= 0) {
    r.deleteImage(gBootRasters.hero.imageId);
    gBootRasters.hero = BootRaster{};
  }
  gBootRasters.heroPath = heroPath;
  gBootRasters.hero = loadBootRasterFile(r, heroPath);
  if (gBootRasters.hero.widthPx != trainerHeroW ||
      gBootRasters.hero.heightPx != trainerHeroH) {
    std::fprintf(stderr,
                 "Boot hero %s is %dx%d; expected trainer crop %dx%d\n",
                 heroPath.c_str(), gBootRasters.hero.widthPx,
                 gBootRasters.hero.heightPx, trainerHeroW, trainerHeroH);
  }
}

void drawBootRaster(const BootRaster& img, Renderer& r, float x, float y,
                    float w, float h, float alpha) {
  if (img.imageId < 0 || w <= 0.0f || h <= 0.0f) return;
  r.drawImage(img.imageId, x, y, w, h, alpha);
}

void drawTrainerHeroPanel(Renderer& r, const BootRaster& hero, float alpha,
                          int widthPx, int heightPx) {
  if (hero.imageId < 0) return;

  const float drawW = static_cast<float>(hero.widthPx);
  const float drawH = static_cast<float>(hero.heightPx);
  if (drawW <= 0.0f || drawH <= 0.0f) return;

  if (widthPx == static_cast<int>(kTrainerRefWidthPx) &&
      heightPx == static_cast<int>(kTrainerRefHeightPx)) {
    drawBootRaster(hero, r, kTrainerHeroX, kTrainerHeroY, drawW, drawH, alpha);
    return;
  }

  const float layoutScaleX =
      static_cast<float>(widthPx) / kTrainerRefWidthPx;
  const float layoutScaleY =
      static_cast<float>(heightPx) / kTrainerRefHeightPx;
  drawBootRaster(hero, r, kTrainerHeroX * layoutScaleX, kTrainerHeroY * layoutScaleY,
                 drawW * layoutScaleX, drawH * layoutScaleY, alpha);
}

void drawBootIcon(Renderer& r, BootIcon icon, float cx, float cy, float size,
                  float alpha) {
  const float s = size;
  switch (icon) {
    case BootIcon::Airframe: {
      const float h = s * 0.85f;
      const float w = h * 0.9f;
      const Point tri[3] = {
          {cx, cy - h * 0.45f},
          {cx + w * 0.5f, cy + h * 0.45f},
          {cx - w * 0.5f, cy + h * 0.45f},
      };
      r.fillPolygon(tri, 3, withAlpha(colors::kGarminLogoBlue, alpha));
      break;
    }
    case BootIcon::CheckOk: {
      const Color box = withAlpha(colors::kWhite, alpha * 0.85f);
      const float half = s * 0.38f;
      r.strokeRoundedRect(cx - half, cy - half, 2.0f * half, 2.0f * half,
                          half * 0.2f, 1.2f, box);
      const float chk = s * 0.22f;
      const Color ok = withAlpha(colors::kActiveGreen, alpha);
      r.strokeLine(cx - chk, cy, cx - chk * 0.15f, cy + chk * 0.75f, 2.0f,
                   ok);
      r.strokeLine(cx - chk * 0.15f, cy + chk * 0.75f, cx + chk, cy - chk,
                   2.0f, ok);
      break;
    }
    case BootIcon::Basemap: {
      const float w = s * 0.72f;
      const float h = s * 0.82f;
      const float left = cx - w * 0.5f;
      const float top = cy - h * 0.5f;
      r.fillRoundedRect(left, top, w, h, s * 0.08f,
                        withAlpha(Color{0.12f, 0.35f, 0.78f, 1.0f}, alpha));
      r.fillText(cx, cy + s * 0.04f, "80", s * 0.34f, TextAlign::Center,
                 withAlpha(colors::kWhite, alpha));
      break;
    }
    case BootIcon::SafeTaxi: {
      const float w = s * 0.78f;
      const float h = s * 0.72f;
      const float left = cx - w * 0.5f;
      const float top = cy - h * 0.5f;
      r.fillRect(left, top, w, h,
                 withAlpha(colors::kBandYellow, alpha * 0.95f));
      r.fillText(cx - s * 0.08f, cy + s * 0.02f, "A", s * 0.38f,
                 TextAlign::Center, withAlpha(colors::kBlack, alpha));
      const float ax = cx + s * 0.18f;
      const Point arrow[3] = {
          {ax, cy - s * 0.12f},
          {ax + s * 0.18f, cy + s * 0.12f},
          {ax - s * 0.04f, cy + s * 0.12f},
      };
      r.fillPolygon(arrow, 3, withAlpha(colors::kBlack, alpha));
      break;
    }
    case BootIcon::Terrain: {
      const Point peaks[5] = {
          {cx - s * 0.42f, cy + s * 0.28f},
          {cx - s * 0.18f, cy - s * 0.22f},
          {cx + s * 0.02f, cy + s * 0.05f},
          {cx + s * 0.28f, cy - s * 0.32f},
          {cx + s * 0.42f, cy + s * 0.28f},
      };
      r.fillPolygon(peaks, 5,
                    withAlpha(Color{0.45f, 0.55f, 0.28f, 1.0f}, alpha));
      const Point peakAccent[3] = {
          {cx + s * 0.18f, cy - s * 0.05f},
          {cx + s * 0.28f, cy - s * 0.32f},
          {cx + s * 0.36f, cy - s * 0.05f},
      };
      r.fillPolygon(peakAccent, 3,
                    withAlpha(Color{0.78f, 0.55f, 0.18f, 1.0f}, alpha));
      break;
    }
    case BootIcon::Obstacle: {
      const Color c = withAlpha(colors::kWhite, alpha);
      r.strokeLine(cx, cy - s * 0.34f, cx, cy + s * 0.28f, 1.5f, c);
      r.strokeLine(cx - s * 0.22f, cy - s * 0.12f, cx + s * 0.22f,
                   cy - s * 0.12f, 1.5f, c);
      r.strokeLine(cx - s * 0.16f, cy - s * 0.24f, cx + s * 0.16f,
                   cy - s * 0.24f, 1.5f, c);
      r.strokeLine(cx - s * 0.28f, cy + s * 0.28f, cx + s * 0.28f,
                   cy + s * 0.28f, 1.5f, c);
      break;
    }
    case BootIcon::Navigation: {
      r.fillCircle(cx, cy, s * 0.34f,
                   withAlpha(Color{0.82f, 0.28f, 0.55f, 1.0f}, alpha));
      r.strokeLine(cx, cy - s * 0.34f, cx, cy + s * 0.34f, 1.2f,
                   withAlpha(colors::kWhite, alpha));
      r.strokeLine(cx - s * 0.34f, cy, cx + s * 0.34f, cy, 1.2f,
                   withAlpha(colors::kWhite, alpha));
      break;
    }
    case BootIcon::AptDir: {
      const float w = s * 0.62f;
      const float h = s * 0.72f;
      const float left = cx - w * 0.5f;
      const float top = cy - h * 0.5f;
      r.fillRoundedRect(left, top, w, h, s * 0.06f,
                        withAlpha(Color{0.18f, 0.42f, 0.78f, 1.0f}, alpha));
      r.fillRect(left + w * 0.12f, top, w * 0.18f, h * 0.22f,
                 withAlpha(Color{0.82f, 0.28f, 0.55f, 1.0f}, alpha));
      r.fillRect(left + w * 0.38f, top, w * 0.18f, h * 0.22f,
                 withAlpha(Color{0.25f, 0.72f, 0.35f, 1.0f}, alpha));
      break;
    }
    case BootIcon::FliteCharts: {
      const float w = s * 0.62f;
      const float h = s * 0.72f;
      const float left = cx - w * 0.5f;
      const float top = cy - h * 0.5f;
      r.fillRect(left, top, w, h, withAlpha(colors::kWhite, alpha * 0.9f));
      r.fillRect(left, top, w, h * 0.18f,
                 withAlpha(Color{0.25f, 0.72f, 0.35f, 1.0f}, alpha));
      r.strokeLine(left + w * 0.2f, cy, left + w * 0.8f, cy, 1.0f,
                   withAlpha(colors::kTitleGray, alpha));
      break;
    }
    case BootIcon::Charts: {
      const float w = s * 0.72f;
      const float h = s * 0.58f;
      const float left = cx - w * 0.5f;
      const float top = cy - h * 0.35f;
      r.fillRect(left, top, w, h,
                 withAlpha(Color{0.55f, 0.72f, 0.82f, 1.0f}, alpha));
      r.fillCircle(cx + w * 0.18f, top + h * 0.35f, s * 0.12f,
                   withAlpha(colors::kBandYellow, alpha));
      break;
    }
  }
}

void drawValueConnector(Renderer& r, float stemX, float stemTop, float armEndX,
                        float armY, float alpha) {
  const Color c = withAlpha(colors::kWhite, alpha * 0.75f);
  r.strokeLine(stemX, stemTop, stemX, armY, 1.0f, c);
  r.strokeLine(stemX, armY, armEndX, armY, 1.0f, c);
}

float bootIconBottomY(BootIcon icon, float iconCy, float iconSize) {
  switch (icon) {
    case BootIcon::Airframe:
      return iconCy + iconSize * 0.85f * 0.45f;
    case BootIcon::CheckOk:
      return iconCy + iconSize * 0.38f;
    default:
      return iconCy + iconSize * 0.4f;
  }
}

float bootHeroRightPx(float layoutScaleX) {
  return (kTrainerHeroX + kTrainerHeroW) * layoutScaleX;
}

float bootDetailMaxWidth(float listRight, float layoutScaleX) {
  constexpr float kHeroMarginPx = 6.0f;
  return std::max(0.0f,
                  listRight - bootHeroRightPx(layoutScaleX) -
                      kHeroMarginPx * layoutScaleX);
}

float maxBootTextWidth(Renderer& r, const std::vector<std::string>& lines,
                       float sizePx) {
  float w = 0.0f;
  for (const std::string& line : lines) {
    w = std::max(w, r.measureTextWidth(line, sizePx));
  }
  return w;
}

// Split a long path into up to two lines that stay right of the hero panel.
std::vector<std::string> splitPathToFitWidth(Renderer& r, const std::string& text,
                                             float sizePx, float maxWidth) {
  if (text.empty()) return {text};
  if (maxWidth <= 0.0f || r.measureTextWidth(text, sizePx) <= maxWidth) {
    return {text};
  }

  for (std::size_t i = text.size(); i > 0; --i) {
    const char c = text[i - 1];
    if (c != '/' && c != '\\') continue;
    const std::string top = text.substr(0, i);
    const std::string bottom = text.substr(i);
    if (bottom.empty()) continue;
    if (r.measureTextWidth(top, sizePx) <= maxWidth &&
        r.measureTextWidth(bottom, sizePx) <= maxWidth) {
      return {top, bottom};
    }
  }

  for (std::size_t suffixLen = 1; suffixLen < text.size(); ++suffixLen) {
    const std::string bottom = text.substr(text.size() - suffixLen);
    if (r.measureTextWidth(bottom, sizePx) > maxWidth) break;
    const std::string top = text.substr(0, text.size() - suffixLen);
    if (r.measureTextWidth(top, sizePx) <= maxWidth) {
      return {top, bottom};
    }
  }

  const std::size_t mid = text.size() / 2;
  return {text.substr(0, mid), text.substr(mid)};
}

void drawBootDetailLines(Renderer& r, float anchorX, float bottomY, float sizePx,
                         TextAlign align, const Color& color,
                         const std::vector<std::string>& lines) {
  const float lineStep = sizePx * 1.12f;
  if (lines.size() <= 1) {
    r.fillText(anchorX, bottomY,
               lines.empty() ? std::string() : lines.front(), sizePx, align,
               color);
    return;
  }
  r.fillText(anchorX, bottomY, lines.back(), sizePx, align, color);
  r.fillText(anchorX, bottomY - lineStep, lines.front(), sizePx, align, color);
}

float bootDetailBottomY(float rowY, float rowStep, float valueSize,
                        float iconBottomY, std::size_t lineCount,
                        float extraDownPx) {
  float bottomY = rowY + rowStep * 0.72f + extraDownPx;
  if (lineCount <= 1) return bottomY;

  const float lineStep = valueSize * 1.12f;
  const float minTopCenterY = iconBottomY + valueSize * 0.55f + 2.0f;
  const float preferredTopY = bottomY - lineStep;
  if (preferredTopY < minTopCenterY) {
    bottomY = minTopCenterY + lineStep;
  }
  return bottomY;
}

float bootDetailConnectorY(float bottomY, float valueSize,
                           std::size_t lineCount) {
  if (lineCount <= 1) return bottomY;
  return bottomY - valueSize * 1.12f * 0.5f;
}

FlightData sensorsFailedBootData(const FlightData& data) {
  FlightData d = data;
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
  d.markerBeacon = MarkerBeacon::None;
  d.vdiKind = VerticalDeviationKind::None;
  return d;
}

void renderLogo(Renderer& r, float alpha, int widthPx, int heightPx) {
  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const float cx = w * 0.5f;
  const float cy = h * 0.5f;

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

  PrimaryFlightDisplay::render(r, sensorsFailedBootData(flightData), map, ui,
                               EisLayout{}, /*reversionary=*/false, widthPx,
                               heightPx, /*powerUp=*/true);

  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);
  const pfd::Layout L = pfd::computeLayout(w, h);

  r.fillRect(L.insetMapX, L.insetMapY, L.insetMapW, L.insetMapH, colors::kBlack);
}

void renderMfdPowerUp(Renderer& r, const MfdController& /*mfdUi*/,
                      const NavDatabaseInfo& navDatabase, bool awaitingAck,
                      const std::string& aircraftIcaoType,
                      const std::string& aircraftAcfRelativePath,
                      const std::string& checklistSourcePath, float alpha,
                      int widthPx, int heightPx) {
  if (alpha <= 0.0f) return;

  const AircraftProfile profile =
      resolveAircraftProfile(aircraftIcaoType, aircraftAcfRelativePath);
  const std::string airframeName =
      profile.bootAirframeName.empty() ? aircraftIcaoType
                                        : profile.bootAirframeName;
  const std::string heroPath =
      resolveBootHeroAsset(aircraftIcaoType, aircraftAcfRelativePath);

  ensureBootHero(r, heroPath);

  const float w = static_cast<float>(widthPx);
  const float h = static_cast<float>(heightPx);

  r.fillRect(0.0f, 0.0f, w, h, colors::kBlack);

  // Left panel: logo + hero art as a single trainer bitmap.
  drawTrainerHeroPanel(r, gBootRasters.hero, alpha, widthPx, heightPx);

  // Database column (trainer screenshot001.bmp at 1024×768).
  const float layoutScaleX =
      static_cast<float>(widthPx) / kTrainerRefWidthPx;
  const float layoutScaleY =
      static_cast<float>(heightPx) / kTrainerRefHeightPx;
  const float listRight = kTrainerListRightPx * layoutScaleX;
  const float iconX = kTrainerIconXPx * layoutScaleX;
  const float labelX = kTrainerLabelXPx * layoutScaleX;
  const float labelSize = kTrainerLabelSizePx * layoutScaleY;
  const float valueSize = kTrainerValueSizePx * layoutScaleY;
  const float iconSize = kTrainerIconSizePx * layoutScaleY;
  const float rowStep = kTrainerRowStepPx * layoutScaleY;
  const Color white = withAlpha(colors::kWhite, alpha);
  const Color muted = withAlpha(colors::kLabelText, alpha);
  const Color yellow = withAlpha(colors::kBandYellow, alpha);

  float rowY = kTrainerFirstRowYPx * layoutScaleY;

  // Airframe + this application's build version on the System line.
  {
    const float labelY = rowY + rowStep * 0.30f;
    const float valueY = rowY + rowStep * 0.72f;
    const float iconCy = rowY + rowStep * 0.50f;
    drawBootIcon(r, BootIcon::Airframe, iconX, iconCy, iconSize, alpha);
    r.fillText(labelX, labelY, airframeName, labelSize, TextAlign::Left, white);
    char sysBuf[48];
    std::snprintf(sysBuf, sizeof(sysBuf), "System %s", kVersion);
    const float sysW = r.measureTextWidth(sysBuf, valueSize);
    const float iconBottomY =
        bootIconBottomY(BootIcon::Airframe, iconCy, iconSize);
    drawValueConnector(r, iconX, iconBottomY,
                       listRight - sysW - valueSize * 0.15f, valueY, alpha);
    r.fillText(listRight, valueY, sysBuf, valueSize, TextAlign::Right, white);
    rowY += rowStep;
  }

  const float maxDetailW = bootDetailMaxWidth(listRight, layoutScaleX);
  std::vector<std::string> checklistDetailLines;
  if (!checklistSourcePath.empty()) {
    checklistDetailLines =
        splitPathToFitWidth(r, checklistSourcePath, valueSize, maxDetailW);
  }
  if (checklistDetailLines.size() > 1) {
    rowY += valueSize * 1.65f;
  }

  for (std::size_t i = 0; i < kVisibleDatabaseRows; ++i) {
    const BootDbRow& row = kDatabaseRows[i];
    const float labelY = rowY + rowStep * 0.30f;
    const float iconCy = rowY + rowStep * 0.50f;
    std::string detail = row.detail;
    bool warn = row.warn;

    if (row.icon == BootIcon::CheckOk) {
      detail = checklistSourcePath.empty() ? "Not found" : checklistSourcePath;
      warn = false;
    } else if (row.navRow && navDatabase.available) {
      if (!navDatabase.expires.empty()) {
        detail = std::string("Expires ") + navDatabase.expires;
      } else if (!navDatabase.cycle.empty()) {
        detail = navDatabase.cycle;
      }
      warn = navDatabase.expired;
    } else if (row.navRow) {
      detail = "N/A";
    }

    const bool na = detail == "Not found";
    const Color labelColor = warn ? yellow : white;
    const Color detailColor = warn ? yellow : (na ? muted : white);

    drawBootIcon(r, row.icon, iconX, iconCy, iconSize, alpha);

    const std::string label = std::string(row.label) + ":";
    r.fillText(labelX, labelY, label, labelSize, TextAlign::Left, labelColor);

    std::vector<std::string> detailLines = {detail};
    const bool checklistRow = row.icon == BootIcon::CheckOk;
    float detailLeftX = listRight;
    float detailMaxW = maxDetailW;
    if (checklistRow && !checklistSourcePath.empty()) {
      detailLines = checklistDetailLines.empty()
                        ? splitPathToFitWidth(r, checklistSourcePath, valueSize,
                                              maxDetailW)
                        : checklistDetailLines;
      // Left edge aligns with the checkbox column (just right of the icon).
      detailLeftX = iconX + iconSize * 0.42f;
      detailMaxW = std::max(0.0f, listRight - detailLeftX - valueSize * 0.15f);
      if (detailLines.size() > 1 ||
          r.measureTextWidth(checklistSourcePath, valueSize) > detailMaxW) {
        detailLines =
            splitPathToFitWidth(r, checklistSourcePath, valueSize, detailMaxW);
      }
    } else if (checklistRow) {
      detailLeftX = iconX + iconSize * 0.42f;
    } else if (r.measureTextWidth(detail, valueSize) > maxDetailW) {
      detailLines = splitPathToFitWidth(r, detail, valueSize, maxDetailW);
    }

    const float iconBottomY = bootIconBottomY(row.icon, iconCy, iconSize);
    const float pathExtraDown =
        checklistRow ? kChecklistPathExtraDownPx * layoutScaleY : 0.0f;
    const float detailBottomY =
        bootDetailBottomY(rowY, rowStep, valueSize, iconBottomY,
                          detailLines.size(), pathExtraDown);
    const float connectorY =
        bootDetailConnectorY(detailBottomY, valueSize, detailLines.size());
    const float detailW = maxBootTextWidth(r, detailLines, valueSize);
    const float connectorEndX =
        checklistRow ? detailLeftX : listRight - detailW - valueSize * 0.15f;
    drawValueConnector(r, iconX, iconBottomY, connectorEndX, connectorY, alpha);
    if (checklistRow) {
      drawBootDetailLines(r, detailLeftX, detailBottomY, valueSize,
                          TextAlign::Left, detailColor, detailLines);
    } else {
      drawBootDetailLines(r, listRight, detailBottomY, valueSize,
                          TextAlign::Right, detailColor, detailLines);
    }
    rowY += rowStep;
  }

  if (awaitingAck) {
    const float promptSize = h * 0.020f;
    const float promptRight = w * 0.988f;
    r.fillText(promptRight, h * 0.935f,
               "Press ENT or rightmost softkey to continue", promptSize,
               TextAlign::Right, white);
  }
}

}  // namespace

void BootScreen::render(Renderer& r, Target target, Phase phase,
                        const FlightData& flightData, const MapData& map,
                        const SoftkeyController& pfdUi,
                        const MfdController& mfdUi,
                        const NavDatabaseInfo& navDatabase, bool awaitingAck,
                        const std::string& aircraftIcaoType,
                        const std::string& aircraftAcfRelativePath,
                        const std::string& checklistSourcePath,
                        float phaseAlpha, int widthPx, int heightPx) {
  if (phase == Phase::Logo) {
    renderLogo(r, phaseAlpha, widthPx, heightPx);
    return;
  }

  if (target == Target::Pfd) {
    renderPfdPowerUp(r, flightData, map, pfdUi, phaseAlpha, widthPx, heightPx);
    return;
  }

  renderMfdPowerUp(r, mfdUi, navDatabase, awaitingAck, aircraftIcaoType,
                   aircraftAcfRelativePath, checklistSourcePath, phaseAlpha,
                   widthPx, heightPx);
}

}  // namespace avionics
