#include <algorithm>
#include <string>

#include "render/pfd/HsiInternal.h"

namespace avionics::pfd {
namespace {

// Resolves the active CDI source label and its color (GPS magenta, VOR/LOC
// green), plus whether it is GPS (which adds a flight-phase annunciation).
const char* cdiSourceLabel(const FlightData& d, bool obs, Color& color,
                           bool& isGps) {
  isGps = false;
  switch (d.cdiSource) {
    case CdiSource::Gps:  color = colors::kMagenta;     isGps = true; return obs ? "OBS" : "GPS";
    case CdiSource::Nav1: color = colors::kActiveGreen;               return "VOR1";
    case CdiSource::Nav2: color = colors::kActiveGreen;               return "VOR2";
  }
  color = colors::kMagenta;
  return "GPS";
}

}  // namespace

void drawCdiSource(Renderer& r, float cx, float cy, float radius,
                   const FlightData& d, const SoftkeyController& ui,
                   float displayH) {
  // Rose layout: nav source and (for GPS) the flight phase are annunciated
  // inside the upper half of the rose, straddling the course pointer (e.g.
  // "GPS   TERM"). When OBS mode is on, automatic waypoint sequencing is
  // suspended and "OBS"/"SUSP" annunciate (G1000 NXi Pilot's Guide, OBS).
  const bool obs = ui.displayToggle(DisplayToggle::Obs);
  Color c;
  bool isGps;
  const char* text = cdiSourceLabel(d, obs, c, isGps);
  const float size = fontPx(wt::kHsiSource, displayH);
  const float y = cy - radius * 0.20f;
  if (isGps) {
    r.fillText(cx - radius * 0.27f, y, text, size, TextAlign::Center, c);
    const std::string phase = obs ? "SUSP" : d.gpsFlightPhase;
    if (!phase.empty()) {
      // Per the G1000 Pilot's Guide (Table 2-3), the flight-phase annunciation
      // is normally magenta (amber only under cautionary conditions), matching
      // the GPS source color rather than the cyan used for selected references.
      r.fillText(cx + radius * 0.27f, y, phase, size, TextAlign::Center,
                 colors::kMagenta);
    }
  } else {
    r.fillText(cx, y, text, size, TextAlign::Center, c);
  }
}

void drawHsiMapCourseBand(Renderer& r, const Layout& L, const FlightData& d,
                          const SoftkeyController& ui, float displayH) {
  // HSI Map layout: instead of annunciating the source/phase inside the rose,
  // the NXi draws a horizontal course-deviation band straddling the top of the
  // map -- a translucent box holding the lateral deviation scale (four dots, a
  // center line, and the TO/FROM deviation triangle), flanked by the GPS/VOR
  // source box on the left and the flight-phase (sensitivity) box on the right.
  // Geometry follows the WT NXi HSIMapCourseDeviation (#HSI origin 277,387;
  // container +7).
  const auto X = [&](float px) { return px * L.sx; };
  const auto Y = [&](float px) { return px * L.sy; };

  const float bandY = Y(387.0f);
  const float bandH = Y(23.0f);
  const float midY = bandY + bandH * 0.5f;
  const float radius = 5.0f * L.s;
  const float devW = X(183.0f);
  const float devX = L.hsiMapCx - devW * 0.5f;  // band centered on the rose
  const float gap = X(2.0f);
  const float srcW = X(53.0f);
  const float phaseW = X(84.0f);
  const float srcX = devX - gap - srcW;
  const float phaseX = devX + devW + gap;
  const float size = fontPx(wt::kHsiBug, displayH);  // 18 px, per WT

  const bool obs = ui.displayToggle(DisplayToggle::Obs);
  Color srcColor;
  bool isGps;
  const char* srcText = cdiSourceLabel(d, obs, srcColor, isGps);

  // Source box (left).
  r.fillRoundedRect(srcX, bandY, srcW, bandH, radius, colors::kWindBox);
  r.fillText(srcX + srcW * 0.5f, midY, srcText, size, TextAlign::Center,
             srcColor);

  // Deviation box (center): translucent fill with a thin gray outline.
  r.fillRoundedRect(devX, bandY, devW, bandH, radius, colors::kWindBox);
  r.strokeRoundedRect(devX, bandY, devW, bandH, radius, 1.0f,
                      colors::kPanelBorder);

  if (d.navSignalValid) {
    r.save();
    r.clip(devX, bandY, devW, bandH);
    // Deviation scale: a center reference line and two dots either side of it.
    r.strokeLine(devX + X(90.5f), bandY + Y(1.0f), devX + X(90.5f),
                 bandY + Y(22.0f), 1.0f, colors::kPanelBorder);
    const float dotR = std::max(1.5f, X(3.0f));
    const float dotY = bandY + Y(11.0f);
    const float dotsX[4] = {X(20.0f), X(55.0f), X(126.0f), X(161.0f)};
    for (float dx : dotsX)
      r.fillCircle(devX + dx, dotY, dotR, colors::kWhite);

    // TO/FROM deviation triangle (apex up = TO), offset by the cross-track
    // deviation (full scale = two dots = 90.5 px, per WT setDeviation).
    const float frac = std::max(-1.0f, std::min(1.0f, d.cdiDeviationDots * 0.5f));
    const float devCx = devX + X(90.5f) + frac * X(90.5f);
    const float half = X(9.0f);
    const float topY = bandY + Y(2.0f);
    const float botY = bandY + Y(20.0f);
    const Color devColor = isGps ? colors::kMagenta : colors::kActiveGreen;
    if (d.cdiToFlag) {
      const Point to[3] = {
          {devCx, topY}, {devCx - half, botY}, {devCx + half, botY}};
      r.fillPolygon(to, 3, devColor);
    } else {
      const Point fr[3] = {
          {devCx, botY}, {devCx - half, topY}, {devCx + half, topY}};
      r.fillPolygon(fr, 3, devColor);
    }
    r.restore();
  } else {
    r.fillText(devX + devW * 0.5f, midY, "NO DTK", size, TextAlign::Center,
               colors::kWhitesmoke);
  }

  // Flight-phase / sensitivity box (right). For GPS this is the phase (e.g.
  // TERM/ENR); OBS suspends sequencing, annunciating SUSP. VOR/LOC has none.
  const std::string phase =
      isGps ? (obs ? "SUSP" : d.gpsFlightPhase) : std::string();
  if (!phase.empty()) {
    r.fillRoundedRect(phaseX, bandY, phaseW, bandH, radius, colors::kWindBox);
    r.fillText(phaseX + phaseW * 0.5f, midY, phase, size, TextAlign::Center,
               colors::kMagenta);
  }
}

}  // namespace avionics::pfd
