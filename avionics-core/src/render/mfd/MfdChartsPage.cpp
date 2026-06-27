#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "render/map/MapViewInternal.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

#if defined(AVIONICS_HAS_STBIMAGE)
#include "stb_image.h"
#endif

namespace avionics::mfd {

namespace {

struct ChartTextureCache {
  unsigned generation = 0;
  int imageId = -1;
  int width = 0;
  int height = 0;
  Renderer* owner = nullptr;
};
ChartTextureCache gChartCache;

void releaseChartTexture(Renderer& r) {
  if (gChartCache.imageId >= 0 && gChartCache.owner == &r) {
    r.deleteImage(gChartCache.imageId);
  }
  gChartCache = ChartTextureCache{};
}

int ensureChartTexture(Renderer& r, const ChartImage& image) {
  const bool haveBytes = image.generation != 0 && image.pngBytes != nullptr &&
                         !image.pngBytes->empty();
  if (!haveBytes) {
    if (gChartCache.imageId >= 0) releaseChartTexture(r);
    return -1;
  }
  if (gChartCache.imageId >= 0 && gChartCache.owner == &r &&
      gChartCache.generation == image.generation) {
    return gChartCache.imageId;
  }
  releaseChartTexture(r);
#if defined(AVIONICS_HAS_STBIMAGE)
  int w = 0;
  int h = 0;
  int comp = 0;
  unsigned char* pixels = stbi_load_from_memory(
      image.pngBytes->data(), static_cast<int>(image.pngBytes->size()), &w, &h,
      &comp, 4);
  if (pixels == nullptr || w <= 0 || h <= 0) {
    if (pixels != nullptr) stbi_image_free(pixels);
    return -1;
  }
  const int id = r.createImageRGBA(w, h, pixels);
  stbi_image_free(pixels);
  if (id < 0) return -1;
  gChartCache.generation = image.generation;
  gChartCache.imageId = id;
  gChartCache.width = w;
  gChartCache.height = h;
  gChartCache.owner = &r;
  return id;
#else
  return -1;
#endif
}

std::string chartDisplayName(const ChartListItem& chart) {
  if (chart.name.empty()) return chart.indexNumber;
  if (chart.indexNumber.empty()) return chart.name;
  return chart.name + ", (" + chart.indexNumber + ")";
}

std::string truncateToWidth(Renderer& r, const std::string& text, float sizePx,
                            float maxWidth) {
  if (r.measureTextWidth(text, sizePx) <= maxWidth) return text;
  std::string out = text;
  while (!out.empty() &&
         r.measureTextWidth(out + "...", sizePx) > maxWidth) {
    out.pop_back();
  }
  return out + "...";
}

void drawCenteredMessage(Renderer& r, float x, float y, float w, float h,
                         const std::string& line1, const std::string& line2,
                         float displayH, const Color& color) {
  const float size = mfdFontPx(kWtRow, displayH);
  const float cx = x + w * 0.5f;
  const float cy = y + h * 0.5f;
  r.fillText(cx, cy, line1, size, TextAlign::Center, color);
  if (!line2.empty()) {
    r.fillText(cx, cy + size * 1.6f, line2, mfdFontPx(kWtFieldLabel, displayH),
               TextAlign::Center, colors::kWhitesmoke);
  }
}

// Second-field label in the full-screen header / data window, per the chart's
// category (Fig 8-21/8-22 show "Info" for the airport diagram, "Approach" for
// an approach plate, etc.).
const char* chartCategoryLabel(ChartCategory c) {
  switch (c) {
    case ChartCategory::Departure:
      return "Departure";
    case ChartCategory::Arrival:
      return "Arrival";
    case ChartCategory::Approach:
      return "Approach";
    case ChartCategory::Airport:
    case ChartCategory::Reference:
      return "Info";
    default:
      return "Chart";
  }
}

// Section label from the active category filter, used when no chart is selected
// yet (the airport diagram / Info chart is the default per Pilot's Guide §8.3).
const char* chartFilterLabel(ChartsCategoryFilter f) {
  switch (f) {
    case ChartsCategoryFilter::Departure:
      return "Departure";
    case ChartsCategoryFilter::Arrival:
      return "Arrival";
    case ChartsCategoryFilter::Approach:
      return "Approach";
    case ChartsCategoryFilter::Airport:
    case ChartsCategoryFilter::All:
    default:
      return "Info";
  }
}

// Full-screen header strip (Fig 8-21): a single row with the airport ident, the
// chart field label + name, and the chart source, drawn only when the right
// data window is hidden.
void drawChartsHeader(Renderer& r, float x, float y, float w, float barH,
                      float displayH, const std::string& airportIcao,
                      const char* fieldLabel, const std::string& chartTitle) {
  const float labelSize = mfdFontPx(kWtFieldLabel, displayH);
  const float valueSize = mfdFontPx(kWtRow, displayH);
  r.fillRect(x, y, w, barH, colors::kBlack);

  const float pad = mfdFontPx(8.0f, displayH);
  const float cy = y + barH * 0.55f;
  float cx = x + pad;

  r.fillText(cx, cy, "Airport", labelSize, TextAlign::Left, colors::kTitleGray);
  cx += r.measureTextWidth("Airport", labelSize) + pad * 0.5f;
  const std::string ident = airportIcao.empty() ? "_ _ _ _" : airportIcao;
  r.fillText(cx, cy, ident, valueSize, TextAlign::Left, colors::kCyan);

  const float midX = x + w * 0.30f;
  r.fillText(midX, cy, fieldLabel, labelSize, TextAlign::Left,
             colors::kTitleGray);
  const float titleX =
      midX + r.measureTextWidth(fieldLabel, labelSize) + pad * 0.5f;
  const float srcLabelW = r.measureTextWidth("Source", labelSize);
  const float srcValW = r.measureTextWidth("Navigraph", valueSize);
  const float titleMaxW =
      (x + w - pad - srcValW - pad - srcLabelW - pad) - titleX;
  r.fillText(titleX, cy, truncateToWidth(r, chartTitle, valueSize, titleMaxW),
             valueSize, TextAlign::Left, colors::kCyan);

  const float srcValX = x + w - pad;
  r.fillText(srcValX, cy, "Navigraph", valueSize, TextAlign::Right,
             colors::kWhitesmoke);
  r.fillText(srcValX - srcValW - pad, cy, "Source", labelSize, TextAlign::Right,
             colors::kTitleGray);

  r.strokeLine(x, y + barH, x + w, y + barH, 1.0f, colors::kGroupBoxBorder);
}

const MapFeature* airportFeature(const MapData& map, const std::string& icao) {
  if (icao.empty()) return nullptr;
  for (const MapFeature& f : map.features) {
    if (f.type == MapFeatureType::Airport && f.id == icao) return &f;
  }
  return nullptr;
}

void strokeHighlightBox(Renderer& r, const Rect& box, float displayH,
                        const Color& c) {
  const float w = mfdFontPx(2.0f, displayH);
  r.strokeRoundedRect(box.x, box.y, box.w, box.h, mfdFontPx(4.0f, displayH), w,
                      c);
}

// Crop the chart image to the CHRT Opt view (approximate vertical slices).
void chartViewCrop(ChartsViewMode mode, float& srcY, float& srcH) {
  switch (mode) {
    case ChartsViewMode::Header:
      srcY = 0.0f;
      srcH = 0.22f;
      break;
    case ChartsViewMode::Plan:
      srcY = 0.18f;
      srcH = 0.52f;
      break;
    case ChartsViewMode::Profile:
      srcY = 0.55f;
      srcH = 0.28f;
      break;
    case ChartsViewMode::Minimums:
      srcY = 0.78f;
      srcH = 0.22f;
      break;
    case ChartsViewMode::All:
    default:
      srcY = 0.0f;
      srcH = 1.0f;
      break;
  }
}

bool chartMatchesFilter(const ChartListItem& chart, ChartsCategoryFilter filter) {
  switch (filter) {
    case ChartsCategoryFilter::All:
      return true;
    case ChartsCategoryFilter::Departure:
      return chart.category == ChartCategory::Departure;
    case ChartsCategoryFilter::Arrival:
      return chart.category == ChartCategory::Arrival;
    case ChartsCategoryFilter::Approach:
      return chart.category == ChartCategory::Approach;
    case ChartsCategoryFilter::Airport:
      return chart.category == ChartCategory::Airport;
  }
  return true;
}

bool pixelInRect(const ChartPixelRect& rect, float x, float y) {
  const float minX = std::min(rect.x1, rect.x2);
  const float maxX = std::max(rect.x1, rect.x2);
  const float minY = std::min(rect.y1, rect.y2);
  const float maxY = std::max(rect.y1, rect.y2);
  return x >= minX && x <= maxX && y >= minY && y <= maxY;
}

bool latLngInPlanview(const ChartGeoref& georef, double lat, double lon) {
  const double minLat = std::min(georef.planviewLat1, georef.planviewLat2);
  const double maxLat = std::max(georef.planviewLat1, georef.planviewLat2);
  const double minLng = std::min(georef.planviewLng1, georef.planviewLng2);
  const double maxLng = std::max(georef.planviewLng1, georef.planviewLng2);
  return lat >= minLat && lat <= maxLat && lon >= minLng && lon <= maxLng;
}

bool latLngToChartPixel(const ChartGeoref& georef, double lat, double lon,
                        float& outX, float& outY) {
  const double dLng = georef.planviewLng2 - georef.planviewLng1;
  const double dLat = georef.planviewLat2 - georef.planviewLat1;
  if (std::abs(dLng) < 1e-12 || std::abs(dLat) < 1e-12) return false;
  const double fx = (lon - georef.planviewLng1) / dLng;
  const double fy = (lat - georef.planviewLat1) / dLat;
  const ChartPixelRect& pv = georef.planviewPixels;
  outX = static_cast<float>(pv.x1 + fx * (static_cast<double>(pv.x2) - pv.x1));
  outY = static_cast<float>(pv.y1 + fy * (static_cast<double>(pv.y2) - pv.y1));
  return true;
}

struct ChartImageLayout {
  float dx = 0.0f;
  float dy = 0.0f;
  float drawW = 0.0f;
  float drawH = 0.0f;
  float fullDrawH = 0.0f;
  float offsetY = 0.0f;
  float srcY = 0.0f;
  float srcH = 1.0f;
  int imageW = 0;
  int imageH = 0;
};

ChartImageLayout computeChartImageLayout(const Rect& chartArea, int imageW,
                                         int imageH, ChartsViewMode viewMode,
                                         float zoom, float panXFrac,
                                         float panYFrac, bool fitWidth) {
  ChartImageLayout layout;
  layout.imageW = imageW;
  layout.imageH = imageH;
  chartViewCrop(viewMode, layout.srcY, layout.srcH);

  const float cropAspect =
      static_cast<float>(imageW) /
      (static_cast<float>(imageH) * layout.srcH);
  // Base fit: width-fit (Fit WDTH) or contain (default), then user zoom.
  float baseW = chartArea.w;
  if (!fitWidth) {
    float baseH = baseW / cropAspect;
    if (baseH > chartArea.h) baseW = chartArea.h * cropAspect;
  }
  const float z = std::max(1.0f, zoom);
  layout.drawW = baseW * z;
  layout.drawH = layout.drawW / cropAspect;
  layout.dx = chartArea.x + (chartArea.w - layout.drawW) * 0.5f +
              panXFrac * layout.drawW;
  layout.dy = chartArea.y + (chartArea.h - layout.drawH) * 0.5f +
              panYFrac * layout.drawH;
  layout.fullDrawH = layout.drawH / std::max(layout.srcH, 0.01f);
  layout.offsetY =
      layout.dy -
      (layout.fullDrawH - layout.drawH) * layout.srcY / std::max(layout.srcH, 0.01f);
  return layout;
}

bool chartPixelToScreen(const ChartImageLayout& layout, float chartX,
                        float chartY, float& screenX, float& screenY) {
  if (layout.imageW <= 0 || layout.imageH <= 0) return false;
  const float normX = chartX / static_cast<float>(layout.imageW);
  const float normY =
      1.0f - chartY / static_cast<float>(layout.imageH);  // bottom-left -> top-left
  screenX = layout.dx + normX * layout.drawW;
  screenY = layout.offsetY + normY * layout.fullDrawH;
  return true;
}

bool chartOwnshipVisible(const ChartGeoref& georef, const ChartImageLayout& layout,
                         double lat, double lon, float& screenX, float& screenY) {
  if (!georef.isGeoreferenced) return false;
  float chartX = 0.0f;
  float chartY = 0.0f;
  if (!latLngToChartPixel(georef, lat, lon, chartX, chartY)) return false;
  if (!latLngInPlanview(georef, lat, lon)) return false;
  for (const ChartPixelRect& inset : georef.insetPixels) {
    if (pixelInRect(inset, chartX, chartY)) return false;
  }
  if (!chartPixelToScreen(layout, chartX, chartY, screenX, screenY)) {
    return false;
  }
  const float normY =
      1.0f - chartY / static_cast<float>(std::max(layout.imageH, 1));
  if (normY < layout.srcY || normY > layout.srcY + layout.srcH) return false;
  return screenX >= layout.dx && screenX <= layout.dx + layout.drawW &&
         screenY >= layout.dy && screenY <= layout.dy + layout.drawH;
}

// NXi "Aircraft Not Shown" annunciation (Pilot's Guide §8.3): lower-right of the
// chart when the chart is not geo-referenced or ownship is outside planview.
void drawAircraftNotShownIcon(Renderer& r, const Rect& chartArea,
                              float displayH) {
  const float iconSize = mfdFontPx(22.0f, displayH);
  const float pad = mfdFontPx(6.0f, displayH);
  const float cx = chartArea.x + chartArea.w - pad - iconSize * 0.5f;
  const float cy = chartArea.y + chartArea.h - pad - iconSize * 0.5f;
  mapview::drawOwnshipSymbol(r, cx, cy, iconSize * 0.45f, 0.0f,
                             colors::kTitleGray);
  r.strokeLine(cx - iconSize * 0.35f, cy + iconSize * 0.35f,
               cx + iconSize * 0.35f, cy - iconSize * 0.35f, 2.0f,
               colors::kTitleGray);
}

}  // namespace

void drawChartsPage(Renderer& r, const FlightData& d, const MapData& map,
                    const MfdController& ui, float x, float y, float w, float h,
                    float displayH) {
  const bool gpsOk = d.dataLinkValid && map.positionValid;
  r.fillRect(x, y, w, h, colors::kMfdPanelGray);
  const ChartsState& cs = ui.chartsState();

  if (!cs.commAllowed) {
    drawCenteredMessage(r, x, y, w, h, "SIM OFFLINE", "CONNECT X-PLANE",
                        displayH, colors::kBandYellow);
    return;
  }
  if (cs.loginPhase != NavigraphLoginPhase::LoggedIn) {
    drawCenteredMessage(r, x, y, w, h, "SIGN IN TO NAVIGRAPH",
                        "USE THE AUX - NAVIGRAPH PAGE", displayH,
                        colors::kCyan);
    return;
  }

  const float pad = mfdFontPx(6.0f, displayH);
  const std::string airport = cs.airportIcao.empty() ? ui.chartsDesiredAirport()
                                                     : cs.airportIcao;
  const MapFeature* apt = airportFeature(map, airport);

  const ChartListItem* sel = nullptr;
  if (ui.chartsSelected() >= 0 &&
      ui.chartsSelected() < static_cast<int>(cs.charts.size())) {
    sel = &cs.charts[static_cast<std::size_t>(ui.chartsSelected())];
  }

  // The section label tracks the displayed chart's type (Fig 8-22: "Info" for
  // the airport diagram, "Approach" for an approach), not a fixed "Approach".
  const char* fieldLabel = sel != nullptr
                               ? chartCategoryLabel(sel->category)
                               : chartFilterLabel(ui.chartsCategoryFilter());

  const bool showPanel = !ui.chartsFullScreen();
  const float bodyX = x + pad;
  const float bodyW = w - 2.0f * pad;

  // The Airport/field/Source header strip only appears in full-screen view
  // (Fig 8-21); with the data window shown those fields live in its boxes
  // (Fig 8-20), so the chart fills the page from the top.
  float bodyTop = y + pad;
  if (!showPanel) {
    const float headerH = mfdFontPx(26.0f, displayH);
    std::string chartTitle;
    if (sel != nullptr) {
      chartTitle = chartDisplayName(*sel);
    } else if (cs.status == ChartsStatus::Loading) {
      chartTitle = "LOADING...";
    } else if (cs.status == ChartsStatus::Error) {
      chartTitle = cs.error.empty() ? "FAIL" : cs.error;
    } else if (cs.charts.empty()) {
      chartTitle = airport.empty() ? "NO FLIGHT PLAN" : "NO CHARTS";
    } else {
      chartTitle = "SELECT A CHART";
    }
    drawChartsHeader(r, bodyX, bodyTop, bodyW, headerH, displayH, airport,
                     fieldLabel, chartTitle);
    bodyTop += headerH + mfdFontPx(4.0f, displayH);
  }

  const float bodyY = bodyTop;
  const float bodyH = h - (bodyY - y) - pad;
  const float panelW = showPanel ? bodyW * kPanelWFrac : 0.0f;
  const float chartW = bodyW - panelW - (showPanel ? pad : 0.0f);

  // ---- Chart image / nav map (left / full width) ----
  {
    const Rect chartArea{bodyX, bodyY, chartW, bodyH};
    r.fillRect(chartArea.x, chartArea.y, chartArea.w, chartArea.h,
               colors::kBlack);

    if (ui.chartsShowMap()) {
      // Chart softkey toggled to the associated nav map (airport-centered).
      r.save();
      r.clip(chartArea.x, chartArea.y, chartArea.w, chartArea.h);
      drawPageMap(r, d, map, chartArea, ui.rangeNm(), apt, displayH,
                  /*showFixes=*/false, /*procedurePreview=*/nullptr,
                  ui.displayRangeNm());
      r.restore();
      // Skip the chart-image / panel? No — panel still draws below.
    } else {
    const int imageId = ensureChartTexture(r, cs.image);
    if (imageId >= 0 && gChartCache.width > 0 && gChartCache.height > 0) {
      const ChartImageLayout layout = computeChartImageLayout(
          chartArea, gChartCache.width, gChartCache.height,
          ui.chartsViewMode(), ui.chartsZoom(), ui.chartsPanXFrac(),
          ui.chartsPanYFrac(), ui.chartsFitWidth());

      r.save();
      r.clip(chartArea.x, chartArea.y, chartArea.w, chartArea.h);
      r.drawImage(imageId, layout.dx, layout.offsetY, layout.drawW,
                  layout.fullDrawH, 1.0f);

      if (sel != nullptr && gpsOk) {
        float ownX = 0.0f;
        float ownY = 0.0f;
        if (chartOwnshipVisible(sel->georef, layout, map.ownshipLat,
                                map.ownshipLon, ownX, ownY)) {
          const float ownSize =
              std::max(8.0f, mfdFontPx(mapview::kOwnshipSymbolWt, displayH));
          mapview::drawOwnshipSymbol(r, ownX, ownY, ownSize, d.headingDeg,
                                     colors::kMagenta);
        } else if (!sel->georef.isGeoreferenced ||
                   ui.chartsViewMode() == ChartsViewMode::All ||
                   ui.chartsViewMode() == ChartsViewMode::Plan) {
          drawAircraftNotShownIcon(r, chartArea, displayH);
        }
      } else if (sel != nullptr &&
                 (!sel->georef.isGeoreferenced ||
                  ui.chartsViewMode() == ChartsViewMode::All ||
                  ui.chartsViewMode() == ChartsViewMode::Plan)) {
        drawAircraftNotShownIcon(r, chartArea, displayH);
      }
      r.restore();
    } else {
      const char* msg = sel != nullptr ? "LOADING CHART..." : "SELECT A CHART";
      r.fillText(chartArea.x + chartArea.w * 0.5f,
                 chartArea.y + chartArea.h * 0.5f, msg,
                 mfdFontPx(kWtRow, displayH), TextAlign::Center,
                 sel != nullptr ? colors::kCyan : colors::kWhitesmoke);
    }
    }  // end chart-image branch
  }

  // ---- Right data window (Airport / Approach) ----
  if (showPanel) {
    const float panelX = bodyX + chartW + pad;
    const Rect panel{panelX, bodyY, panelW, bodyH};
    r.fillRect(panel.x, panel.y, panel.w, panel.h, colors::kMfdPanelGray);

    PanelStack stack(panel, displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);

    // Solid cyan field highlight behind the active selection value (the NXi
    // highlight-select look, Fig 8-20) -- not a pulsing border box.
    auto highlightValue = [&](float tx, float baselineY, const std::string& text,
                              float size) {
      const float tw = r.measureTextWidth(text, size);
      const float padX = mfdFontPx(3.0f, displayH);
      r.fillRect(tx - padX, baselineY - size * 0.72f, tw + 2.0f * padX,
                 size * 1.25f, colors::kCyan);
    };

    // Airport box (Fig 8-20): cyan ident, small airport symbol, usage type,
    // then the facility name and city rows.
    {
      const bool hi = ui.chartsField() == ChartsField::Airport && ui.blinkOn();
      Rect inner = drawGroupBox(r, stack.slot(150.0f), "Airport", displayH);
      const float identSize = mfdFontPx(kWtIdentLarge, displayH);
      const float identCy = inner.y + identSize * 0.62f;
      const bool entering = ui.chartsAirportEntryActive();
      // While typing, resolve the spell-ahead match so the page shows the
      // airport's icon/usage and facility name/city -- exactly like the FPL and
      // PROC airport entry -- so the pilot can confirm the airport.
      const MapFeature entryMatch = ui.chartsAirportEntryMatch();
      const bool entryHasMatch =
          entering && ui.chartsAirportEntryHasMatch() &&
          !ui.chartsAirportEntryIdent().empty();
      const MapFeature* infoApt =
          entering ? (entryHasMatch ? &entryMatch : nullptr) : apt;
      float identEndX = inner.x;
      if (entering) {
        // Free ICAO entry in progress: per-cell ident field with the pulsing
        // cursor cell, white typed characters, and cyan spell-ahead fill --
        // identical to the FPL insert and PROC airport entry.
        identEndX = drawIdentEntryCells(
            r, inner.x, identCy, ui.chartsAirportEntryIdent(),
            ui.chartsAirportEntryCursor(), ui.chartsAirportEntryTypedCount(),
            ui.chartsAirportEntrySelectAll(), ui.blinkOn(), displayH);
      } else {
        const std::string ident =
            apt != nullptr ? apt->id
                           : (airport.empty() ? std::string("_ _ _ _") : airport);
        if (hi) highlightValue(inner.x, identCy, ident, identSize);
        r.fillText(inner.x, identCy, ident, identSize, TextAlign::Left,
                   hi ? colors::kBlack : colors::kCyan);
        identEndX = inner.x + r.measureTextWidth(ident, identSize);
      }
      if (infoApt != nullptr) {
        drawWaypointIcon(r, identEndX + mfdFontPx(14.0f, displayH), identCy,
                         mfdFontPx(15.0f, displayH), infoApt,
                         MapFeatureType::Airport);
        const char* usage = airportUsageType(*infoApt);
        if (usage != nullptr) {
          r.fillText(inner.x + inner.w, identCy, usage,
                     mfdFontPx(kWtFieldLabel, displayH), TextAlign::Right,
                     colors::kWhitesmoke);
        }
      }
      drawFacilityNameCity(r, inner, inner.y + identSize * 1.5f, infoApt,
                           displayH);
    }

    // Chart-selection box (label tracks the category: Info / Departure /
    // Arrival / Approach) with optional dropdown list.
    const float srcSlotH = 56.0f;
    {
      const bool onField = ui.chartsField() == ChartsField::Approach;
      const bool hi = onField && ui.blinkOn();
      Rect inner = drawGroupBox(r, stack.slot(stack.remainingWt(srcSlotH)),
                                fieldLabel, displayH);
      const float valY = inner.y + rowSize * 0.7f;
      // While the popup is open the box shows the highlighted (pending) chart;
      // otherwise the committed chart that is actually displayed.
      const ChartListItem* boxItem = sel;
      if (onField && ui.chartsPending() >= 0 &&
          ui.chartsPending() < static_cast<int>(cs.charts.size())) {
        boxItem = &cs.charts[static_cast<std::size_t>(ui.chartsPending())];
      }
      if (boxItem != nullptr) {
        const std::string line =
            truncateToWidth(r, chartDisplayName(*boxItem), rowSize, inner.w);
        if (hi) highlightValue(inner.x, valY, line, rowSize);
        r.fillText(inner.x, valY, line, rowSize, TextAlign::Left,
                   hi ? colors::kBlack : colors::kCyan);
      } else {
        r.fillText(inner.x, valY, "_ _ _ _ _ _ _", rowSize, TextAlign::Left,
                   colors::kWhitesmoke);
      }

      // Dropdown overlay when Approach box is selected (Fig 8-20).
      if (onField && !cs.charts.empty()) {
        const float rowH = mfdFontPx(kWtListRow, displayH);
        std::vector<int> filtered;
        filtered.reserve(cs.charts.size());
        for (int i = 0; i < static_cast<int>(cs.charts.size()); ++i) {
          if (chartMatchesFilter(cs.charts[static_cast<std::size_t>(i)],
                                 ui.chartsCategoryFilter())) {
            filtered.push_back(i);
          }
        }
        const int total = static_cast<int>(filtered.size());
        int selFiltered = 0;
        for (int i = 0; i < total; ++i) {
          if (filtered[static_cast<std::size_t>(i)] == ui.chartsPending()) {
            selFiltered = i;
            break;
          }
        }
        const int visible = std::max(1, static_cast<int>(inner.h / rowH));
        int first = std::max(0, selFiltered - visible / 2);
        if (first > total - visible) first = std::max(0, total - visible);

        const float listY = inner.y + rowSize * 1.4f;
        const float listH =
            inner.y + inner.h - listY - mfdFontPx(4.0f, displayH);
        if (listH > rowH) {
          r.fillRect(inner.x, listY, inner.w, listH, colors::kBlack);
          strokeHighlightBox(r, Rect{inner.x, listY, inner.w, listH}, displayH,
                             colors::kGroupBoxBorder);
          int drawn = 0;
          for (int row = 0; row < visible && first + row < total; ++row) {
            const int i = filtered[static_cast<std::size_t>(first + row)];
            const ChartListItem& chart = cs.charts[static_cast<std::size_t>(i)];
            const float ry = listY + rowH * static_cast<float>(drawn);
            const float cy = ry + rowH * 0.5f;
            const bool selected = i == ui.chartsPending();
            const bool selectedOn = selected && ui.blinkOn();
            if (selectedOn) {
              r.fillRect(inner.x, ry, inner.w, rowH, colors::kCyan);
            }
            const Color textColor =
                selectedOn ? colors::kBlack
                           : (selected ? colors::kCyan : colors::kWhitesmoke);
            const std::string line =
                truncateToWidth(r, chartDisplayName(chart), rowSize, inner.w);
            r.fillText(inner.x + mfdFontPx(4.0f, displayH), cy, line, rowSize,
                       TextAlign::Left, textColor);
            ++drawn;
          }
        }
      }
    }

    // Source box (Fig 8-15/8-22/8-23): the active chart source.
    {
      Rect inner = drawGroupBox(r, stack.slot(srcSlotH), "Source", displayH);
      r.fillText(inner.x, inner.y + rowSize * 0.7f, "Navigraph", rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    }
  }
}

}  // namespace avionics::mfd
