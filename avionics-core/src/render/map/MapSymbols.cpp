#include "avionics/render/MapSymbols.h"

#include <cmath>

namespace avionics {
namespace {

void strokeCircle(Renderer& r, float cx, float cy, float radius, float width,
                  const Color& c) {
  constexpr int kSeg = 24;
  Point ring[kSeg + 1];
  for (int i = 0; i <= kSeg; ++i) {
    const float a = static_cast<float>(i) / static_cast<float>(kSeg) * 2.0f *
                    3.14159265f;
    ring[i] = {cx + radius * std::cos(a), cy + radius * std::sin(a)};
  }
  r.strokePolyline(ring, kSeg + 1, width, c);
}

// Four cardinal "fuel tab" nubs (top/bottom/left/right) marking a serviced
// airport, matching the Garmin NXi map icon. The nubs sit just outside the
// airport circle and carry the same fill plus the dark outline.
void drawFuelTabs(Renderer& r, float x, float y, float rad, const Color& fill) {
  const float gap = rad * 0.10f;
  const float len = rad * 0.62f;
  const float half = rad * 0.42f;
  const float in = rad + gap;
  const float out = in + len;
  const struct {
    float x0, y0, w, h;
  } tabs[4] = {
      {x - half, y - out, 2.0f * half, len},  // top
      {x - half, y + in, 2.0f * half, len},   // bottom
      {x - out, y - half, len, 2.0f * half},  // left
      {x + in, y - half, len, 2.0f * half},   // right
  };
  for (const auto& t : tabs) {
    r.fillRect(t.x0, t.y0, t.w, t.h, fill);
    r.strokeLine(t.x0, t.y0, t.x0 + t.w, t.y0, 1.0f, colors::kMapSymbolOutline);
    r.strokeLine(t.x0, t.y0 + t.h, t.x0 + t.w, t.y0 + t.h, 1.0f,
                 colors::kMapSymbolOutline);
    r.strokeLine(t.x0, t.y0, t.x0, t.y0 + t.h, 1.0f, colors::kMapSymbolOutline);
    r.strokeLine(t.x0 + t.w, t.y0, t.x0 + t.w, t.y0 + t.h, 1.0f,
                 colors::kMapSymbolOutline);
  }
}

// UI popup airport icon (icons-map/airport_*.png): a filled diamond with a
// white center dot and optional cardinal fuel tabs when serviced.
void drawUiAirportSymbol(Renderer& r, float x, float y, float s, const Color& c,
                         AirportFacilityKind kind, bool serviced) {
  if (kind == AirportFacilityKind::Heliport ||
      kind == AirportFacilityKind::Private) {
    const float h = s * 0.95f;
    const Point diamond[4] = {{x, y - h}, {x + h, y}, {x, y + h}, {x - h, y}};
    r.fillPolygon(diamond, 4, c);
    const Point outline[5] = {diamond[0], diamond[1], diamond[2], diamond[3],
                              diamond[0]};
    r.strokePolyline(outline, 5, 1.2f, colors::kMapSymbolOutline);
    const char* glyph = kind == AirportFacilityKind::Heliport ? "H" : "R";
    r.fillText(x, y, glyph, s * 1.1f, TextAlign::Center, colors::kWhite);
    return;
  }

  const float h = s * 0.95f;
  const Point diamond[4] = {{x, y - h}, {x + h, y}, {x, y + h}, {x - h, y}};
  r.fillPolygon(diamond, 4, c);
  const Point outline[5] = {diamond[0], diamond[1], diamond[2], diamond[3],
                            diamond[0]};
  r.strokePolyline(outline, 5, 1.2f, colors::kMapSymbolOutline);
  r.fillCircle(x, y, s * 0.20f, colors::kWhite);

  if (serviced) {
    const float tabLen = s * 0.34f;
    const float tabHalf = s * 0.20f;
    const struct {
      float x0, y0, w, h;
    } tabs[4] = {
        {x - tabHalf, y - h - tabLen, 2.0f * tabHalf, tabLen},  // north
        {x - tabHalf, y + h, 2.0f * tabHalf, tabLen},           // south
        {x - h - tabLen, y - tabHalf, tabLen, 2.0f * tabHalf},  // west
        {x + h, y - tabHalf, tabLen, 2.0f * tabHalf},           // east
    };
    for (const auto& t : tabs) {
      r.fillRect(t.x0, t.y0, t.w, t.h, c);
      r.strokeLine(t.x0, t.y0, t.x0 + t.w, t.y0, 1.0f,
                   colors::kMapSymbolOutline);
      r.strokeLine(t.x0, t.y0 + t.h, t.x0 + t.w, t.y0 + t.h, 1.0f,
                   colors::kMapSymbolOutline);
      r.strokeLine(t.x0, t.y0, t.x0, t.y0 + t.h, 1.0f,
                   colors::kMapSymbolOutline);
      r.strokeLine(t.x0 + t.w, t.y0, t.x0 + t.w, t.y0 + t.h, 1.0f,
                   colors::kMapSymbolOutline);
    }
  }
}

// Garmin NXi airport symbol: a filled disc (paved) or hollow ring (soft surface
// / seaplane) for the field, four cardinal fuel tabs when serviced, and the "H"
// / "R" glyph variants for heliports and private fields.
void drawAirportSymbol(Renderer& r, float x, float y, float s, const Color& c,
                       AirportFacilityKind kind, bool serviced) {
  const float rad = s * 0.85f;

  if (kind == AirportFacilityKind::Heliport ||
      kind == AirportFacilityKind::Private) {
    r.fillCircle(x, y, rad, c);
    strokeCircle(r, x, y, rad, 1.2f, colors::kMapSymbolOutline);
    const char* glyph = kind == AirportFacilityKind::Heliport ? "H" : "R";
    // fillText is vertically centered (NVG_ALIGN_MIDDLE), so the glyph centers
    // on the circle's center; size it to sit inside the disc (diameter 1.7*s).
    r.fillText(x, y, glyph, s * 1.2f, TextAlign::Center, colors::kWhite);
    return;
  }

  if (serviced) drawFuelTabs(r, x, y, rad, c);

  if (kind == AirportFacilityKind::Seaplane) {
    // Soft-surface style: hollow ring so the map background shows through.
    strokeCircle(r, x, y, rad, 2.0f, c);
    strokeCircle(r, x, y, rad, 3.4f, colors::kMapSymbolOutline);
    strokeCircle(r, x, y, rad, 2.0f, c);
    return;
  }

  r.fillCircle(x, y, rad, c);
  strokeCircle(r, x, y, rad, 1.2f, colors::kMapSymbolOutline);
}

// VOR: a hollow hexagon ring with a filled center dot (a square DME box would
// frame it on a VOR/DME, omitted at map scale).
void drawVorSymbol(Renderer& r, float x, float y, float s, const Color& c) {
  Point hex[7];
  for (int i = 0; i <= 6; ++i) {
    const float a = 1.0471976f * static_cast<float>(i) + 0.5235988f;
    hex[i] = {x + s * std::cos(a), y + s * std::sin(a)};
  }
  r.strokePolyline(hex, 7, 2.0f, c);
  r.fillCircle(x, y, s * 0.22f, c);
}

// NDB: a center dot surrounded by a ring of small dots (stippled circle).
void drawNdbSymbol(Renderer& r, float x, float y, float s, const Color& c) {
  r.fillCircle(x, y, s * 0.32f, c);
  constexpr int kDots = 10;
  for (int i = 0; i < kDots; ++i) {
    const float a = static_cast<float>(i) / static_cast<float>(kDots) * 2.0f *
                    3.14159265f;
    r.fillCircle(x + s * std::cos(a), y + s * std::sin(a), s * 0.14f, c);
  }
}

// Intersection / waypoint: a filled triangle (point up) with a dark outline.
void drawIntersectionSymbol(Renderer& r, float x, float y, float s,
                            const Color& c) {
  const Point tri[3] = {{x, y - s * 0.85f},
                        {x - s * 0.78f, y + s * 0.6f},
                        {x + s * 0.78f, y + s * 0.6f}};
  r.fillPolygon(tri, 3, c);
  const Point outline[4] = {tri[0], tri[1], tri[2], tri[0]};
  r.strokePolyline(outline, 4, 1.0f, colors::kMapSymbolOutline);
}

}  // namespace

Color mapFeatureColor(const MapFeature& feature) {
  if (feature.type == MapFeatureType::Airport) {
    if (feature.airportKind == AirportFacilityKind::Heliport ||
        feature.airportKind == AirportFacilityKind::Private) {
      return colors::kAirportNonTowered;
    }
    return feature.airportTowered ? colors::kAirportTowered
                                  : colors::kAirportNonTowered;
  }
  return mapFeatureColor(feature.type);
}

Color mapFeatureColor(MapFeatureType type) {
  switch (type) {
    case MapFeatureType::Airport:
      return colors::kAirportNonTowered;
    case MapFeatureType::Vor:
      return colors::kNavaidCyan;
    case MapFeatureType::Ndb:
      return colors::kNdbMagenta;
    case MapFeatureType::Waypoint:
    case MapFeatureType::Fix:
      return colors::kNavaidCyan;
  }
  return colors::kLabelText;
}

void drawMapFeatureSymbol(Renderer& r, const MapFeature& feature, float x,
                          float y, float size) {
  drawMapFeatureSymbol(r, feature.type, x, y, size, mapFeatureColor(feature),
                       feature.airportKind, feature.airportTowered,
                       feature.airportServiced);
}

void drawMapFeatureSymbol(Renderer& r, MapFeatureType type, float x, float y,
                          float size, const Color& c,
                          AirportFacilityKind airportKind,
                          bool /*airportTowered*/, bool airportServiced) {
  switch (type) {
    case MapFeatureType::Airport:
      drawAirportSymbol(r, x, y, size, c, airportKind, airportServiced);
      break;
    case MapFeatureType::Vor:
      drawVorSymbol(r, x, y, size, c);
      break;
    case MapFeatureType::Ndb:
      drawNdbSymbol(r, x, y, size, c);
      break;
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      drawIntersectionSymbol(r, x, y, size, c);
      break;
  }
}

void drawUiWaypointIcon(Renderer& r, const MapFeature& feature, float x,
                        float y, float size) {
  drawUiWaypointIcon(r, feature.type, x, y, size, mapFeatureColor(feature),
                     feature.airportKind, feature.airportTowered,
                     feature.airportServiced);
}

void drawUiWaypointIcon(Renderer& r, MapFeatureType type, float x, float y,
                        float size, const Color& c,
                        AirportFacilityKind airportKind,
                        bool /*airportTowered*/, bool airportServiced) {
  switch (type) {
    case MapFeatureType::Airport:
      drawUiAirportSymbol(r, x, y, size, c, airportKind, airportServiced);
      break;
    case MapFeatureType::Vor:
      drawVorSymbol(r, x, y, size, c);
      break;
    case MapFeatureType::Ndb:
      drawNdbSymbol(r, x, y, size, c);
      break;
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      drawIntersectionSymbol(r, x, y, size, c);
      break;
  }
}

}  // namespace avionics
