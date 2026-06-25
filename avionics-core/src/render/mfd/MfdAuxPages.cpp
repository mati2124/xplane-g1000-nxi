#include "render/mfd/MfdPages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avionics/Color.h"
#include "avionics/NavMath.h"
#include "render/mfd/MfdPageSupport.h"
#include "render/mfd/MfdStyle.h"

namespace avionics::mfd {

namespace {

// Lays out the AUX pages' column grid (WT System Setup: gray page, three
// columns of group boxes with 20px gaps).
struct AuxGrid {
  Rect body;
  float displayH;

  float px(float v) const { return mfdFontPx(v, displayH); }

  Rect column(int index, int count) const {
    const float gap = px(20.0f);
    const float colW = (body.w - gap * static_cast<float>(count + 1)) /
                       static_cast<float>(count);
    return Rect{body.x + gap + (colW + gap) * static_cast<float>(index),
                body.y + px(10.0f), colW, body.h - px(20.0f)};
  }
};

AuxGrid beginAuxPage(Renderer& r, float x, float y, float w, float h,
                     float displayH) {
  r.fillRect(x, y, w, h, colors::kMfdPanelGray);
  return AuxGrid{Rect{x, y, w, h}, displayH};
}

}  // namespace

void drawTripPlanningPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH) {
  // AUX Trip Planning, automatic page mode for the remaining active leg
  // (present position -> active waypoint): Input Data + the leg preview on
  // top, Trip / Fuel / Other Stats along the bottom (G1000 Pilot's Guide for
  // Cessna Nav III, Section 5.9). Fields whose inputs this suite has no
  // source for (ESA, sunrise/sunset) are dashed, as the real unit dashes
  // stats it cannot compute.
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);
  char buf[32];

  const bool linkValid = d.dataLinkValid;
  const bool hasLeg = linkValid && !d.fmaToWpt.empty();
  const float gs = d.groundSpeedKts;
  const float distNm = d.fmaLegDistanceNm;
  const long ete = hasLeg ? eteSeconds(distNm, gs) : -1;
  const float fuelOnBoard = d.fuelQtyLeftGal + d.fuelQtyRightGal;
  const bool fuelFlowValid = linkValid && d.fuelFlowGph > 0.1f;

  const float gap = grid.px(20.0f);
  const float topH = (h - grid.px(20.0f)) * 0.52f;
  const float colW = (w - 3.0f * gap) / 2.0f;

  // Input Data (top left).
  {
    Rect inner = drawGroupBox(
        r, Rect{x + gap, y + grid.px(10.0f), colW, topH}, "Input Data",
        displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "PAGE MODE", "Automatic", displayH,
                   colors::kCyan);
    fy = drawField(r, inner, fy, rowH, "DEP TIME",
                   linkValid ? formatClock(d.utcHour, d.utcMinute) + " UTC"
                             : std::string(kDashTime),
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT", static_cast<int>(std::lround(gs)));
    fy = drawField(r, inner, fy, rowH, "GS", linkValid ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1fGPH",
                  static_cast<double>(d.fuelFlowGph));
    fy = drawField(r, inner, fy, rowH, "FUEL FLOW", linkValid ? buf : kDash,
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1fGAL",
                  static_cast<double>(fuelOnBoard));
    fy = drawField(r, inner, fy, rowH, "FUEL ONBOARD", linkValid ? buf : kDash,
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(d.airspeedKts)));
    fy = drawField(r, inner, fy, rowH, "CALIBRATED AS",
                   linkValid ? buf : kDash, displayH, colors::kWhitesmoke);
  }

  // Leg preview map (top right).
  {
    // Plain hyphen joiner: the loaded font has no arrow glyph (U+2192).
    const std::string legTitle =
        hasLeg ? "P.POS - " + d.fmaToWpt : "No Active Leg";
    Rect inner = drawGroupBox(
        r, Rect{x + colW + 2.0f * gap, y + grid.px(10.0f), colW, topH},
        legTitle.c_str(), displayH);
    drawPageMap(r, d, map, inner,
                std::max(10.0f, std::min(100.0f, distNm * 1.4f)), nullptr,
                displayH);
  }

  // Bottom row: Trip Stats / Fuel Stats / Other Stats.
  const float statsY = y + grid.px(10.0f) + topH + grid.px(10.0f);
  const float statsH = y + h - grid.px(10.0f) - statsY;
  const float statsW = (w - 4.0f * gap) / 3.0f;

  {
    Rect inner = drawGroupBox(r, Rect{x + gap, statsY, statsW, statsH},
                              "Trip Stats", displayH);
    const float rowH = inner.h / 7.0f;
    float fy = inner.y;
    const bool gpsDtk = hasLeg && d.cdiSource == CdiSource::Gps;
    if (gpsDtk) {
      std::snprintf(buf, sizeof(buf), "%03d%s",
                    static_cast<int>(std::lround(d.courseDeg)), kDeg);
    }
    fy = drawField(r, inner, fy, rowH, "DTK", gpsDtk ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1fNM", static_cast<double>(distNm));
    fy = drawField(r, inner, fy, rowH, "DIS", hasLeg ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "ETE", formatDuration(ete), displayH,
                   colors::kWhitesmoke);
    std::string eta = kDashTime;
    if (ete >= 0) {
      const long arrive =
          (d.utcHour * 3600L + d.utcMinute * 60L + d.utcSecond + ete) % 86400L;
      eta = formatClock(static_cast<int>(arrive / 3600),
                        static_cast<int>((arrive % 3600) / 60)) +
            " UTC";
    }
    fy = drawField(r, inner, fy, rowH, "ETA", eta, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "ESA", kDash, displayH,
                   colors::kWhitesmoke);
    // Sunrise/sunset at the destination waypoint for the sim date (the page's
    // automatic mode computes them at the "to" point); present position when
    // no leg is active.
    std::string sunrise = kDashTime;
    std::string sunset = kDashTime;
    if (linkValid && map.positionValid) {
      double sunLat = map.ownshipLat;
      double sunLon = map.ownshipLon;
      if (hasLeg) {
        for (const MapLeg& leg : map.flightPlan) {
          if (leg.id == d.fmaToWpt) {
            sunLat = leg.lat;
            sunLon = leg.lon;
            break;
          }
        }
      }
      double riseH = 0.0;
      double setH = 0.0;
      if (sunriseSunsetUtc(d.utcDayOfYear, sunLat, sunLon, riseH, setH)) {
        sunrise = formatClock(static_cast<int>(riseH),
                              static_cast<int>(riseH * 60.0) % 60) +
                  " UTC";
        sunset = formatClock(static_cast<int>(setH),
                             static_cast<int>(setH * 60.0) % 60) +
                 " UTC";
      }
    }
    fy = drawField(r, inner, fy, rowH, "SUNRISE", sunrise, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "SUNSET", sunset, displayH,
                   colors::kWhitesmoke);
  }

  {
    Rect inner = drawGroupBox(
        r, Rect{x + 2.0f * gap + statsW, statsY, statsW, statsH}, "Fuel Stats",
        displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    const long endurance =
        fuelFlowValid ? std::lround(fuelOnBoard / d.fuelFlowGph * 3600.0f)
                      : -1;
    const float fuelReq =
        fuelFlowValid && ete >= 0
            ? d.fuelFlowGph * static_cast<float>(ete) / 3600.0f
            : -1.0f;

    if (fuelFlowValid && gs >= 30.0f) {
      std::snprintf(buf, sizeof(buf), "%.1fNM/GAL",
                    static_cast<double>(gs / d.fuelFlowGph));
    }
    fy = drawField(r, inner, fy, rowH, "EFFICIENCY",
                   fuelFlowValid && gs >= 30.0f ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TOTAL ENDUR",
                   formatDuration(endurance), displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1fGAL",
                  static_cast<double>(fuelOnBoard - fuelReq));
    fy = drawField(r, inner, fy, rowH, "REM FUEL",
                   fuelReq >= 0.0f ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "REM ENDUR",
                   endurance >= 0 && ete >= 0
                       ? formatDuration(endurance - ete)
                       : std::string(kDashTime),
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%.1fGAL", static_cast<double>(fuelReq));
    fy = drawField(r, inner, fy, rowH, "FUEL REQ",
                   fuelReq >= 0.0f ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    if (endurance >= 0 && gs >= 30.0f) {
      std::snprintf(buf, sizeof(buf), "%.0fNM",
                    static_cast<double>(gs * endurance / 3600.0f));
    }
    fy = drawField(r, inner, fy, rowH, "TOTAL RANGE",
                   endurance >= 0 && gs >= 30.0f ? buf : kDash, displayH,
                   colors::kWhitesmoke);
  }

  {
    Rect inner = drawGroupBox(
        r, Rect{x + 3.0f * gap + 2.0f * statsW, statsY, statsW, statsH},
        "Other Stats", displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    // Density altitude from pressure altitude and the ISA temperature delta.
    const float pressureAltFt =
        d.altitudeFt + (29.92f - d.baroSettingInHg) * 1000.0f;
    const float isaTempC = 15.0f - 1.98f * d.altitudeFt / 1000.0f;
    const float densityAltFt =
        pressureAltFt + 118.8f * (d.oatCelsius - isaTempC);
    std::snprintf(buf, sizeof(buf), "%dFT",
                  static_cast<int>(std::lround(densityAltFt / 10.0f) * 10));
    fy = drawField(r, inner, fy, rowH, "DENSITY ALT", linkValid ? buf : kDash,
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(d.tasKts)));
    fy = drawField(r, inner, fy, rowH, "TRUE AIRSPEED",
                   linkValid ? buf : kDash, displayH, colors::kWhitesmoke);
  }
}

namespace {

// Fixed satellite constellation for the GPS Status sky view / signal bars.
// The real page draws the tracked SVs; without a per-satellite source this
// renders a deterministic, plausible set (positions/SNRs derived from the
// PRN) so the page reads like the real one.
struct GpsSat {
  int prn;
  float azDeg;
  float elDeg;
  float snr;  // 0..1 of full bar
};

constexpr int kGpsSatCount = 10;

GpsSat gpsSat(int i) {
  static const int kPrns[kGpsSatCount] = {2, 5, 10, 12, 15, 18, 20, 25, 29, 31};
  const int prn = kPrns[i];
  return GpsSat{prn, static_cast<float>((prn * 73) % 360),
                static_cast<float>(15 + (prn * 37) % 70),
                0.55f + static_cast<float>((prn * 13) % 40) / 100.0f};
}

}  // namespace

void drawGpsStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                       float x, float y, float w, float h, float displayH) {
  // AUX GPS Status (G1000 Pilot's Guide, "GPS Status Page"): the satellite
  // sky view and signal-strength bars on the left, position / status /
  // accuracy data on the right. Accuracy figures have no source here and
  // dash, as the real unit dashes values it cannot compute.
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);
  char buf[32];
  const bool gpsOk = d.dataLinkValid && map.positionValid;

  Rect col0 = grid.column(0, 3);
  Rect col1 = grid.column(1, 3);
  Rect col2 = grid.column(2, 3);

  // Sky view: concentric horizon/45-degree circles with the satellites
  // plotted by azimuth/elevation.
  const float skyH = col0.h * 0.62f;
  {
    Rect inner = drawGroupBox(r, Rect{col0.x, col0.y, col0.w, skyH},
                              "Sky View", displayH);
    const float cx = inner.x + inner.w * 0.5f;
    const float cy = inner.y + inner.h * 0.52f;
    const float radius = std::min(inner.w, inner.h) * 0.42f;
    strokeCircle(r, cx, cy, radius, 1.0f, colors::kGroupBoxBorder);
    strokeCircle(r, cx, cy, radius * 0.5f, 1.0f, colors::kGroupBoxBorder);
    const float labelSize = mfdFontPx(14.0f, displayH);
    r.fillText(cx, cy - radius - labelSize * 0.7f, "N", labelSize,
               TextAlign::Center, colors::kTitleGray);
    r.fillText(cx + radius + labelSize * 0.7f, cy, "E", labelSize,
               TextAlign::Center, colors::kTitleGray);
    r.fillText(cx, cy + radius + labelSize * 0.8f, "S", labelSize,
               TextAlign::Center, colors::kTitleGray);
    r.fillText(cx - radius - labelSize * 0.7f, cy, "W", labelSize,
               TextAlign::Center, colors::kTitleGray);
    if (gpsOk) {
      for (int i = 0; i < kGpsSatCount; ++i) {
        const GpsSat sat = gpsSat(i);
        const float rr = radius * (1.0f - sat.elDeg / 90.0f);
        const float a = (sat.azDeg - 90.0f) * 0.01745329f;
        const float sx = cx + rr * std::cos(a);
        const float sy = cy + rr * std::sin(a);
        r.fillCircle(sx, sy, mfdFontPx(3.5f, displayH), colors::kWhite);
        std::snprintf(buf, sizeof(buf), "%d", sat.prn);
        r.fillText(sx, sy - mfdFontPx(8.0f, displayH),
                   buf, mfdFontPx(12.0f, displayH), TextAlign::Center,
                   colors::kTitleGray);
      }
    }
  }

  // Signal strength bars under the sky view.
  {
    Rect inner = drawGroupBox(
        r, Rect{col0.x, col0.y + skyH + grid.px(10.0f), col0.w,
                col0.h - skyH - grid.px(10.0f)},
        "Signal Strength", displayH);
    const float labelSize = mfdFontPx(12.0f, displayH);
    const float baseY = inner.y + inner.h - labelSize * 1.6f;
    const float maxBarH = baseY - inner.y - mfdFontPx(6.0f, displayH);
    const float slotW = inner.w / static_cast<float>(kGpsSatCount);
    const float barW = slotW * 0.55f;
    for (int i = 0; i < kGpsSatCount; ++i) {
      const GpsSat sat = gpsSat(i);
      const float bx = inner.x + slotW * (static_cast<float>(i) + 0.5f);
      if (gpsOk) {
        const float bh = maxBarH * sat.snr;
        r.fillRect(bx - barW * 0.5f, baseY - bh, barW, bh,
                   colors::kActiveGreen);
      } else {
        const Point frame[5] = {{bx - barW * 0.5f, baseY - maxBarH * 0.3f},
                                {bx + barW * 0.5f, baseY - maxBarH * 0.3f},
                                {bx + barW * 0.5f, baseY},
                                {bx - barW * 0.5f, baseY},
                                {bx - barW * 0.5f, baseY - maxBarH * 0.3f}};
        r.strokePolyline(frame, 5, 1.0f, colors::kPanelSeparator);
      }
      std::snprintf(buf, sizeof(buf), "%d", sat.prn);
      r.fillText(bx, baseY + labelSize * 0.9f, buf, labelSize,
                 TextAlign::Center, colors::kTitleGray);
    }
  }

  // GPS Status / Accuracy (middle column).
  {
    const float statusH = col1.h * 0.55f;
    Rect inner = drawGroupBox(r, Rect{col1.x, col1.y, col1.w, statusH},
                              "GPS Status", displayH);
    const float rowH = inner.h / 7.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "SOLUTION", gpsOk ? "3D NAV" : "ACQUIRING",
                   displayH,
                   gpsOk ? colors::kActiveGreen : colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "PHASE", d.gpsFlightPhase, displayH,
                   colors::kWhitesmoke);
    // CDI source annunciation, colored like the HSI: GPS magenta, VOR green.
    const char* cdiSrc = "GPS";
    Color cdiColor = colors::kMagenta;
    switch (d.cdiSource) {
      case CdiSource::Gps:
        break;
      case CdiSource::Nav1:
        cdiSrc = "VOR1";
        cdiColor = colors::kActiveGreen;
        break;
      case CdiSource::Nav2:
        cdiSrc = "VOR2";
        cdiColor = colors::kActiveGreen;
        break;
    }
    fy = drawField(r, inner, fy, rowH, "CDI SRC", cdiSrc, displayH, cdiColor);
    fy = drawField(r, inner, fy, rowH, "LATERAL", d.fmaLateralActive, displayH,
                   colors::kActiveGreen);
    fy = drawField(r, inner, fy, rowH, "VERTICAL", d.fmaVerticalActive,
                   displayH, colors::kActiveGreen);
    // GP DEBUG (temporary glidepath/AP coupling diagnostic): the value string
    // can be long, so it gets its own full-width row drawn left-aligned and
    // shrunk to fit the column. Right-aligning it via drawField made it spill
    // left across the page and overlap the fields above.
    {
      const float labelCy = fy + rowH * 0.5f;
      r.fillText(inner.x, labelCy, "GP DEBUG", mfdFontPx(kWtFieldLabel, displayH),
                 TextAlign::Left, colors::kTitleGray);
      fy += rowH;
      const std::string dbg =
          d.gpCouplingDebug.empty() ? std::string(kDash) : d.gpCouplingDebug;
      float dbgSize = mfdFontPx(kWtFieldValue, displayH) * 0.85f;
      const float dbgW = r.measureTextWidth(dbg, dbgSize);
      if (dbgW > inner.w && dbgW > 0.0f) dbgSize *= inner.w / dbgW;
      r.fillText(inner.x, fy + rowH * 0.5f, dbg, dbgSize, TextAlign::Left,
                 colors::kTitleGray);
      fy += rowH;
    }

    Rect accInner = drawGroupBox(
        r, Rect{col1.x, col1.y + statusH + grid.px(10.0f), col1.w,
                col1.h - statusH - grid.px(10.0f)},
        "Accuracy", displayH);
    const float accRowH = accInner.h / 4.0f;
    float ay = accInner.y;
    ay = drawField(r, accInner, ay, accRowH, "EPU", kDash, displayH,
                   colors::kWhitesmoke);
    ay = drawField(r, accInner, ay, accRowH, "DOP", kDash, displayH,
                   colors::kWhitesmoke);
    ay = drawField(r, accInner, ay, accRowH, "HFOM", kDash, displayH,
                   colors::kWhitesmoke);
    ay = drawField(r, accInner, ay, accRowH, "VFOM", kDash, displayH,
                   colors::kWhitesmoke);
  }

  // Position (right column).
  {
    Rect inner = drawGroupBox(r, Rect{col2.x, col2.y, col2.w, col2.h},
                              "Position", displayH);
    const float rowH = inner.h / 7.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "LAT",
                   gpsOk ? formatLatLon(map.ownshipLat, true)
                         : std::string(kDash),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "LON",
                   gpsOk ? formatLatLon(map.ownshipLon, false)
                         : std::string(kDash),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TIME",
                   gpsOk ? formatClock(d.utcHour, d.utcMinute) + " UTC"
                         : std::string(kDashTime),
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dFT", static_cast<int>(d.altitudeFt));
    fy = drawField(r, inner, fy, rowH, "ALT", gpsOk ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(d.groundSpeedKts));
    fy = drawField(r, inner, fy, rowH, "GS", gpsOk ? buf : kDash, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%03.0f%s",
                  static_cast<double>(d.trackDeg), kDeg);
    fy = drawField(r, inner, fy, rowH, "TRK", gpsOk ? buf : kDash, displayH,
                   colors::kWhitesmoke);
  }
}

void drawSystemStatusPage(Renderer& r, const FlightData& d, const MapData& map,
                          float x, float y, float w, float h, float displayH) {
  // AUX System Status: LRU health on the left, airframe and database info in
  // the other columns. LRU statuses are derived from the live sensor-validity
  // flags (a failed source annunciates a red X, like the real LRU list).
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);

  Rect col0 = grid.column(0, 3);
  Rect col1 = grid.column(1, 3);
  Rect col2 = grid.column(2, 3);

  {
    Rect inner =
        drawGroupBox(r, Rect{col0.x, col0.y, col0.w, col0.h}, "LRU Info",
                     displayH);
    struct Lru {
      const char* name;
      bool ok;
    };
    const bool link = d.dataLinkValid;
    const Lru lrus[] = {
        {"ADC1", link && d.airspeedValid && d.altitudeValid},
        {"AHRS1", link && d.attitudeValid},
        {"COM1", link},
        {"COM2", link},
        {"GEA1", link},
        {"GIA1", link},
        {"GIA2", link},
        {"GMU1", link && d.headingValid},
        {"GPS1", link && map.positionValid},
        {"GPS2", link && map.positionValid},
        {"GTX1", link},
        {"NAV1", link && d.navSignalValid},
        {"NAV2", link},
    };
    const int count = static_cast<int>(sizeof(lrus) / sizeof(lrus[0]));
    const float rowH = inner.h / static_cast<float>(count + 1);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    r.fillText(inner.x, inner.y + rowH * 0.5f, "LRU",
               mfdFontPx(kWtHeader, displayH), TextAlign::Left,
               colors::kTitleGray);
    r.fillText(inner.x + inner.w, inner.y + rowH * 0.5f, "STATUS",
               mfdFontPx(kWtHeader, displayH), TextAlign::Right,
               colors::kTitleGray);
    float fy = inner.y + rowH;
    for (const Lru& lru : lrus) {
      const float cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy, lru.name, rowSize, TextAlign::Left,
                 colors::kWhitesmoke);
      // Status icon: green check for OK, red X for failed.
      const float sx = inner.x + inner.w - rowSize * 0.6f;
      const float s = rowSize * 0.40f;
      if (lru.ok) {
        r.strokeLine(sx - s, cy, sx - s * 0.2f, cy + s * 0.8f, 2.0f,
                     colors::kActiveGreen);
        r.strokeLine(sx - s * 0.2f, cy + s * 0.8f, sx + s, cy - s * 0.8f, 2.0f,
                     colors::kActiveGreen);
      } else {
        r.strokeLine(sx - s, cy - s, sx + s, cy + s, 2.0f, colors::kBandRed);
        r.strokeLine(sx - s, cy + s, sx + s, cy - s, 2.0f, colors::kBandRed);
      }
      fy += rowH;
    }
  }

  {
    const float airframeH = col1.h * 0.36f;
    Rect inner = drawGroupBox(r, Rect{col1.x, col1.y, col1.w, airframeH},
                              "Airframe", displayH);
    const float rowH = inner.h / 3.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "AIRFRAME", "Cessna 172S", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "SYS SOFTWARE", "0563.00", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "CRG PART NUMBER", kDash, displayH,
                   colors::kWhitesmoke);
  }

  {
    Rect inner = drawGroupBox(r, Rect{col2.x, col2.y, col2.w, col2.h * 0.60f},
                              "MFD1 Database", displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    const bool navDb = !map.features.empty();
    const bool airspaceDb = !map.airspaces.empty();
    const bool terrainDb = map.terrain != nullptr;
    fy = drawField(r, inner, fy, rowH, "NAVIGATION DATA",
                   navDb ? "AVAILABLE" : kDash, displayH,
                   navDb ? colors::kActiveGreen : colors::kWhitesmoke);
    // Navigation database currency: like the real AUX databases listing, an
    // active database whose expiration date is in the past has its cycle and
    // dates highlighted in amber (G1000 NXi Pilot's Guide, Appendix B).
    const NavDatabaseInfo& db = map.navDatabase;
    const Color& dbColor =
        db.expired ? colors::kBandYellow : colors::kWhitesmoke;
    fy = drawField(r, inner, fy, rowH, "NAV DB CYCLE",
                   db.available ? db.cycle.c_str() : kDash, displayH,
                   db.available ? dbColor : colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "NAV DB EXPIRES",
                   db.available ? db.expires.c_str() : kDash, displayH,
                   db.available ? dbColor : colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "AIRSPACE DATA",
                   airspaceDb ? "AVAILABLE" : kDash, displayH,
                   airspaceDb ? colors::kActiveGreen : colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TERRAIN DATA",
                   terrainDb ? "AVAILABLE" : kDash, displayH,
                   terrainDb ? colors::kActiveGreen : colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "FLIGHT PLAN",
                   map.flightPlan.empty() ? kDash : "LOADED", displayH,
                   map.flightPlan.empty() ? colors::kWhitesmoke
                                          : colors::kActiveGreen);
  }
}

void drawUtilityPage(Renderer& r, const FlightData& d, const MfdController& ui,
                     float x, float y, float w, float h, float displayH) {
  // AUX Utility (G1000 Pilot's Guide, Section 5, AUX - Utility): the Timers
  // group and Scheduler on the left, Trip Statistics on the right. Timers and
  // trip totals run from the controller's accumulated session stats (since
  // power-on, like the real unit's own accumulation).
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);
  char buf[32];
  const bool linkValid = d.dataLinkValid;
  const FlightSessionStats& stats = ui.flightStats();

  // HH:MM:SS readout for the running timers (the page's timer format).
  auto hms = [&buf](double seconds) {
    const long t = std::max(0L, std::lround(seconds));
    std::snprintf(buf, sizeof(buf), "%02ld:%02ld:%02ld", t / 3600,
                  (t % 3600) / 60, t % 60);
    return std::string(buf);
  };

  Rect col0 = grid.column(0, 2);
  Rect col1 = grid.column(1, 2);

  // Timers (top left).
  const float timersH = col0.h * 0.52f;
  {
    Rect inner = drawGroupBox(r, Rect{col0.x, col0.y, col0.w, timersH},
                              "Timers", displayH);
    const float rowH = inner.h / 4.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "GENERIC UP",
                   linkValid ? hms(stats.genericTimerSec)
                             : std::string("00:00:00"),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "FLIGHT",
                   linkValid ? hms(stats.flightTimerSec)
                             : std::string("00:00:00"),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "DEPARTURE TIME",
                   linkValid && stats.departureHour >= 0
                       ? formatClock(stats.departureHour,
                                     stats.departureMinute) +
                             " UTC"
                       : std::string(kDashTime),
                   displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", d.utcHour, d.utcMinute,
                  d.utcSecond);
    fy = drawField(r, inner, fy, rowH, "SYSTEM TIME",
                   linkValid ? std::string(buf) : std::string("__:__:__"),
                   displayH, colors::kWhitesmoke);
  }

  // Scheduler (bottom left). No scheduled-message source: all messages off.
  {
    Rect inner = drawGroupBox(
        r, Rect{col0.x, col0.y + timersH + grid.px(10.0f), col0.w,
                col0.h - timersH - grid.px(10.0f)},
        "Scheduler", displayH);
    const float rowH = inner.h / 4.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "MESSAGE", kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TYPE", "One Time", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TIME", kDashTime, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "REMAINING", kDashTime, displayH,
                   colors::kWhitesmoke);
  }

  // Trip Statistics (right): session odometers, max/average groundspeed from
  // the accumulated stats, and the live groundspeed.
  {
    Rect inner = drawGroupBox(r, Rect{col1.x, col1.y, col1.w, col1.h},
                              "Trip Statistics", displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    std::snprintf(buf, sizeof(buf), "%.1fNM", stats.odometerNm);
    fy = drawField(r, inner, fy, rowH, "ODOMETER",
                   linkValid ? buf : kDash, displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TRIP ODOMETER",
                   linkValid ? buf : kDash, displayH, colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(stats.maxGroundSpeedKts)));
    fy = drawField(r, inner, fy, rowH, "MAX GROUNDSPEED",
                   linkValid ? buf : kDash, displayH, colors::kWhitesmoke);
    // Average over moving time only, dashed until the aircraft has moved.
    std::string avg = kDash;
    if (linkValid && stats.movingTimeSec >= 1.0) {
      std::snprintf(buf, sizeof(buf), "%dKT",
                    static_cast<int>(std::lround(
                        stats.odometerNm / stats.movingTimeSec * 3600.0)));
      avg = buf;
    }
    fy = drawField(r, inner, fy, rowH, "AVERAGE SPEED", avg, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%dKT",
                  static_cast<int>(std::lround(d.groundSpeedKts)));
    fy = drawField(r, inner, fy, rowH, "GROUNDSPEED",
                   linkValid ? std::string(buf) : std::string(kDash), displayH,
                   colors::kWhitesmoke);
  }
}

void drawSystemSetupPage(Renderer& r, const FlightData& d, float x, float y,
                         float w, float h, float displayH) {
  // AUX System Setup (G1000 Pilot's Guide, Section 5, AUX - System Setup):
  // Date/Time, Display Units, and Airspace Alerts. Unit selections reflect the
  // delivered configuration (read-only here); the live system clock is shown
  // where the page has a real source.
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);
  char buf[32];
  const bool linkValid = d.dataLinkValid;

  Rect col0 = grid.column(0, 3);
  Rect col1 = grid.column(1, 3);
  Rect col2 = grid.column(2, 3);

  {
    Rect inner = drawGroupBox(r, Rect{col0.x, col0.y, col0.w, col0.h * 0.46f},
                              "Date / Time", displayH);
    const float rowH = inner.h / 4.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "DATE", kDash, displayH,
                   colors::kWhitesmoke);
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", d.utcHour, d.utcMinute,
                  d.utcSecond);
    fy = drawField(r, inner, fy, rowH, "TIME",
                   linkValid ? std::string(buf) : std::string("__:__:__"),
                   displayH, colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TIME FORMAT", "UTC", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TIME OFFSET", "+00:00", displayH,
                   colors::kWhitesmoke);
  }

  {
    Rect inner = drawGroupBox(r, Rect{col1.x, col1.y, col1.w, col1.h * 0.60f},
                              "Display Units", displayH);
    const float rowH = inner.h / 7.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "NAV ANGLE", "Magnetic", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "DIS, SPD", "Nautical", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "ALT, VS", "Feet", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "TEMP", "Celsius", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "FUEL", "Gallons", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "WEIGHT", "Pounds", displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "POSITION", "HDDD\xC2\xB0MM.MM'",
                   displayH, colors::kWhitesmoke);
  }

  {
    Rect inner = drawGroupBox(r, Rect{col2.x, col2.y, col2.w, col2.h * 0.60f},
                              "Airspace Alerts", displayH);
    const float rowH = inner.h / 6.0f;
    float fy = inner.y;
    const char* kNames[] = {"CLASS B/TMA", "CLASS C/TCA", "CLASS D",
                            "RESTRICTED", "MOA (MILITARY)", "OTHER/ADIZ"};
    for (const char* name : kNames) {
      fy = drawField(r, inner, fy, rowH, name, "Off", displayH,
                     colors::kWhitesmoke);
    }
  }
}

void drawSimBriefPage(Renderer& r, const MfdController& ui, float x, float y,
                      float w, float h, float displayH) {
  // AUX SimBrief (no real-unit counterpart; styled like the other AUX pages):
  // account + fetch status and the latest OFP summary on the left, the filed
  // route on the right. The Pilot ID edit cursor uses the steady reverse-video
  // style shared with the FMS cursor fields.
  AuxGrid grid = beginAuxPage(r, x, y, w, h, displayH);
  const float gap = grid.px(20.0f);
  const float colW = (w - 3.0f * gap) / 2.0f;
  const float colH = h - grid.px(20.0f);
  const float topY = y + grid.px(10.0f);

  const SimBriefState& sb = ui.simbriefState();

  // Left top: account / fetch status.
  const float acctH = colH * 0.42f;
  {
    Rect inner = drawGroupBox(r, Rect{x + gap, topY, colW, acctH}, "SimBrief",
                              displayH);
    const float rowH = inner.h / 4.0f;
    float fy = inner.y;

    // PILOT ID row. While the ID softkey entry is open, show the pending
    // digits with a trailing cursor underscore in reverse video, like an
    // active cursor field on the real unit.
    if (ui.simbriefIdEntryActive()) {
      const std::string pending = ui.simbriefPendingId() + "_";
      const float cy = fy + rowH * 0.5f;
      r.fillText(inner.x, cy, "PILOT ID", mfdFontPx(kWtFieldLabel, displayH),
                 TextAlign::Left, colors::kTitleGray);
      drawCursorText(r, inner.x + inner.w, cy, pending,
                     mfdFontPx(kWtFieldValue, displayH), TextAlign::Right);
      fy += rowH;
    } else {
      fy = drawField(r, inner, fy, rowH, "PILOT ID",
                     ui.simbriefPilotId().empty() ? kDash
                                                  : ui.simbriefPilotId(),
                     displayH, colors::kCyan);
    }

    const char* statusText = kDash;
    Color statusColor = colors::kWhitesmoke;
    switch (sb.status) {
      case SimBriefStatus::NotConfigured:
        statusText = "NO PILOT ID";
        break;
      case SimBriefStatus::Idle:
        statusText = "READY";
        break;
      case SimBriefStatus::Fetching:
        statusText = "FETCHING...";
        statusColor = colors::kCyan;
        break;
      case SimBriefStatus::Ok:
        statusText = "OFP LOADED";
        statusColor = colors::kActiveGreen;
        break;
      case SimBriefStatus::Error:
        statusText = "FAIL";
        statusColor = colors::kBandYellow;
        break;
    }
    fy = drawField(r, inner, fy, rowH, "STATUS", statusText, displayH,
                   statusColor);

    // Failure detail on its own line, amber like a caution message.
    if (sb.status == SimBriefStatus::Error && !sb.error.empty()) {
      r.fillText(inner.x, fy + rowH * 0.5f, sb.error,
                 mfdFontPx(kWtFieldLabel, displayH), TextAlign::Left,
                 colors::kBandYellow);
    }
  }

  // Left bottom: latest OFP summary.
  {
    Rect inner = drawGroupBox(
        r, Rect{x + gap, topY + acctH + grid.px(10.0f), colW,
                colH - acctH - grid.px(10.0f)},
        "Latest OFP", displayH);
    const bool haveOfp = sb.status == SimBriefStatus::Ok;
    const float rowH = inner.h / 5.0f;
    float fy = inner.y;
    fy = drawField(r, inner, fy, rowH, "ORIGIN",
                   haveOfp ? sb.originIcao : kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "DESTINATION",
                   haveOfp ? sb.destinationIcao : kDash, displayH,
                   colors::kWhitesmoke);
    fy = drawField(r, inner, fy, rowH, "GENERATED",
                   haveOfp ? sb.generatedUtc : kDash, displayH,
                   colors::kWhitesmoke);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d", sb.waypointCount);
    fy = drawField(r, inner, fy, rowH, "WAYPOINTS", haveOfp ? buf : kDash,
                   displayH, colors::kWhitesmoke);
  }

  // Right column: the filed route string, word-wrapped.
  {
    Rect inner = drawGroupBox(
        r, Rect{x + colW + 2.0f * gap, topY, colW, colH}, "Route", displayH);
    const float rowSize = mfdFontPx(kWtRow, displayH);
    const float lineH = rowSize * 1.6f;
    if (sb.status != SimBriefStatus::Ok || sb.route.empty()) {
      r.fillText(inner.x, inner.y + lineH * 0.5f, kDash, rowSize,
                 TextAlign::Left, colors::kWhitesmoke);
    } else {
      // Word wrap: the full text is the origin, route string, destination,
      // the way the OFP header reads.
      const std::string text =
          sb.originIcao + " " + sb.route + " " + sb.destinationIcao;
      std::string line;
      float fy = inner.y + lineH * 0.5f;
      std::size_t pos = 0;
      while (pos < text.size() && fy < inner.y + inner.h - lineH) {
        std::size_t next = text.find(' ', pos);
        if (next == std::string::npos) next = text.size();
        const std::string word = text.substr(pos, next - pos);
        const std::string candidate = line.empty() ? word : line + " " + word;
        if (!line.empty() &&
            r.measureTextWidth(candidate, rowSize) > inner.w) {
          r.fillText(inner.x, fy, line, rowSize, TextAlign::Left,
                     colors::kWhitesmoke);
          fy += lineH;
          line = word;
        } else {
          line = candidate;
        }
        pos = next + 1;
      }
      if (!line.empty() && fy < inner.y + inner.h) {
        r.fillText(inner.x, fy, line, rowSize, TextAlign::Left,
                   colors::kWhitesmoke);
      }
    }
  }
}

}  // namespace avionics::mfd
