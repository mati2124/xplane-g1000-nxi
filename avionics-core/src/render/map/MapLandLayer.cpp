#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "avionics/NavMath.h"

namespace avionics::mapview {
namespace {

// Layer range declutter (NM), mirroring the G1000 Map Setup maximum-range
// defaults: each layer disappears once the range opens past its threshold.
constexpr float kRoadMaxRangeNm = 60.0f;
constexpr float kRiverMaxRangeNm = 150.0f;
// Lakes stay visible on the continental chart (Great Lakes tier at 1000 NM).
constexpr float kLakeMaxRangeNm = kWideChartRangeNm;
// Railroads are a close-in detail feature on the NXi Land group; declutter
// past short range so their crosstie ticks don't clutter a wide view.
constexpr float kRailroadMaxRangeNm = 30.0f;
// State/province lines declutter past regional scale so a continental view
// keeps only nation borders (not coastlines — landmass fill meets the ocean).
constexpr float kStateBorderMaxRangeNm = 200.0f;
// City label tiers by Natural Earth rank (higher rank = larger city).
constexpr float kCityLargeMaxRangeNm = 150.0f;
constexpr float kCityMediumMaxRangeNm = 50.0f;
constexpr float kCitySmallMaxRangeNm = 20.0f;
constexpr float kCityCapitalMaxRangeNm = 500.0f;
constexpr FontFace kGeoLabelFace = kMapLabelFace;

// Land-data styling to sit under the white/cyan/magenta symbology like the
// Garmin base map.
constexpr Color kRiverStroke{0.22f, 0.42f, 0.68f, 1.0f};
constexpr Color kRoadStroke{0.38f, 0.30f, 0.20f, 1.0f};
constexpr Color kBorderStroke{0.75f, 0.75f, 0.75f, 0.9f};
// Nation political borders on the continental chart: solid white.
constexpr Color kChartOutlineStroke{1.0f, 1.0f, 1.0f, 1.0f};
// State/province boundaries: dimmer than nation borders so the political
// hierarchy reads at a glance (nation lines dominate).
constexpr Color kStateBorderStroke{0.42f, 0.42f, 0.42f, 0.7f};
constexpr Color kRailroadStroke{0.62f, 0.62f, 0.62f, 0.8f};
constexpr Color kCityDot{0.85f, 0.78f, 0.45f, 1.0f};
// Hydro labels (lakes, gulfs, bays): bright cyan-blue from the PC Trainer
// continental chart (MFD Default.bmp).
constexpr Color kHydroLabel{0.0f, 0.58f, 1.0f, 1.0f};

float maxLabelRangeNm(const MapLandCity& label) {
  switch (label.labelKind) {
    case LandLabelKind::Hydro:
      if (label.rank >= 10) return 2000.0f;
      if (label.rank >= 8) return kWideChartRangeNm;
      if (label.rank >= 6) return kContinentalChartRangeNm;
      if (label.rank >= 5) return 200.0f;
      return 25.0f;
    case LandLabelKind::Region:
      if (label.rank >= 10) return 2000.0f;
      if (label.rank >= 8) return kWideChartRangeNm;
      if (label.rank >= 6) return 200.0f;
      return 100.0f;
    case LandLabelKind::City:
      break;
  }
  if (label.rank >= 9) return kCityCapitalMaxRangeNm;
  if (label.rank >= 8) return kCityLargeMaxRangeNm;
  if (label.rank >= 4) return kCityMediumMaxRangeNm;
  return kCitySmallMaxRangeNm;
}

bool continentalLabelVisible(const MapLandCity& label, float rangeNm) {
  switch (label.labelKind) {
    case LandLabelKind::Region:
      if (rangeNm >= kWideChartRangeNm) return label.rank >= 8;
      if (rangeNm > kContinentalChartRangeNm) return label.rank >= 8;
      if (rangeNm > 200.0f) return label.rank >= 6;
      return true;
    case LandLabelKind::Hydro:
      if (rangeNm >= kWideChartRangeNm) return label.rank >= 8;
      if (rangeNm > kContinentalChartRangeNm) return label.rank >= 7;
      if (rangeNm > 150.0f) return label.rank >= 5;
      return true;
    case LandLabelKind::City:
      if (rangeNm >= kWideChartRangeNm) return label.rank >= 8;
      if (rangeNm > kContinentalChartRangeNm) return label.rank >= 8;
      return true;
  }
  return true;
}

float chartOutlineWidthPx(float rangeNm) {
  if (rangeNm >= kContinentalChartRangeNm) return 3.0f;
  if (rangeNm >= 150.0f) return 2.0f;
  return 1.25f;
}

float regionLabelSize(float labelSize, int rank, float rangeNm) {
  float scale = 1.0f;
  if (rangeNm >= kWideChartRangeNm) {
    scale = rank >= 8 ? 0.86f : 0.80f;
  } else if (rangeNm >= kContinentalChartRangeNm) {
    scale = rank >= 8 ? 0.88f : 0.82f;
  } else {
    scale = rank >= 8 ? 0.85f : 0.80f;
  }
  return labelSize * scale * kMapGeoLabelScale;
}

float hydroLabelSize(float labelSize, int rank, float rangeNm) {
  float scale = 1.0f;
  if (rangeNm >= kWideChartRangeNm) {
    scale = rank >= 8 ? 0.84f : 0.78f;
  } else if (rangeNm >= kContinentalChartRangeNm) {
    scale = rank >= 8 ? 0.86f : 0.80f;
  } else if (rangeNm >= 150.0f) {
    scale = rank >= 8 ? 0.84f : 0.76f;
  } else {
    scale = rank >= 9 ? 0.82f : 0.72f;
  }
  return labelSize * scale * kMapGeoLabelScale;
}

bool textRectsOverlap(const TextRect& a, const TextRect& b, float pad) {
  return !(a.right + pad < b.left || b.right + pad < a.left ||
           a.bottom + pad < b.top || b.bottom + pad < a.top);
}

TextRect expandTextRect(const TextRect& tr, float pad) {
  return {tr.left - pad, tr.top - pad, tr.right + pad, tr.bottom + pad};
}

struct GeoLabelDraw {
  const MapLandCity* label = nullptr;
  float x = 0.0f;
  float y = 0.0f;
  float size = 0.0f;
  bool hydro = false;
};

// Continental chart: clip long border edges to the viewport so segments that
// cross the map edge still draw when both endpoints are off-screen.
constexpr float kChartBorderClipMarginPx = 80.0f;

void visibleGeoBounds(const Proj& proj, float rangeNm, float marginPx,
                      double& latMin, double& latMax, double& lonMin,
                      double& lonMax) {
  const float corners[4][2] = {
      {proj.minX - marginPx, proj.minY - marginPx},
      {proj.maxX + marginPx, proj.minY - marginPx},
      {proj.maxX + marginPx, proj.maxY + marginPx},
      {proj.minX - marginPx, proj.maxY + marginPx},
  };
  latMin = 90.0;
  latMax = -90.0;
  double lonMinDelta = 180.0;
  double lonMaxDelta = -180.0;
  const double pxPer = static_cast<double>(proj.mercatorPxPerRad);
  for (const auto& c : corners) {
    const double mapEast = (c[0] - proj.cx) / pxPer;
    const double mapNorth = (proj.cy - c[1]) / pxPer;
    const double eastRad = mapEast * proj.cosR + mapNorth * proj.sinR;
    const double northRad = -mapEast * proj.sinR + mapNorth * proj.cosR;
    const double mercY = proj.mercatorYCenter + northRad;
    const double lat =
        std::clamp(map::mercatorLatDegFromY(mercY), -89.5, 89.5);
    const double lonDelta = eastRad / map::kDegToRad;
    latMin = std::min(latMin, lat);
    latMax = std::max(latMax, lat);
    lonMinDelta = std::min(lonMinDelta, lonDelta);
    lonMaxDelta = std::max(lonMaxDelta, lonDelta);
  }
  // When the view nears a pole, inverse Mercator clamps several corners to
  // the same latitude; fall back to a range-scaled band around the center so
  // continental landmass fills still clip to the visible chart.
  const double halfBand =
      static_cast<double>(rangeNm / map::kNmPerDegLat) * 2.5;
  if (latMax - latMin < 0.5) {
    latMin = std::min(latMin, proj.centerLat - halfBand);
    latMax = std::max(latMax, proj.centerLat + halfBand);
  }
  if (lonMaxDelta <= lonMinDelta) {
    const double cosEdge =
        std::max(0.05, std::cos(proj.centerLat * map::kDegToRad));
    const double halfLon =
        (static_cast<double>(rangeNm) / (map::kNmPerDegLat * cosEdge)) * 2.5;
    lonMinDelta = -halfLon;
    lonMaxDelta = halfLon;
  }
  lonMin = proj.centerLon + lonMinDelta;
  lonMax = proj.centerLon + lonMaxDelta;
}

void clipGeoRingAgainstEdge(const std::vector<GeoPoint>& in,
                            std::vector<GeoPoint>& out, double edge,
                            bool vertical, bool keepGreater) {
  out.clear();
  if (in.empty()) return;
  const int n = static_cast<int>(in.size());
  GeoPoint prev = in[n - 1];
  bool prevInside =
      vertical ? (keepGreater ? prev.lat >= edge : prev.lat <= edge)
               : (keepGreater ? prev.lon >= edge : prev.lon <= edge);
  for (int i = 0; i < n; ++i) {
    const GeoPoint& curr = in[i];
    const bool currInside =
        vertical ? (keepGreater ? curr.lat >= edge : curr.lat <= edge)
                 : (keepGreater ? curr.lon >= edge : curr.lon <= edge);
    if (currInside) {
      if (!prevInside) {
        if (vertical) {
          const double denom = curr.lat - prev.lat;
          if (std::fabs(denom) > 1e-9) {
            const double t = (edge - prev.lat) / denom;
            out.push_back({edge, prev.lon + t * (curr.lon - prev.lon)});
          }
        } else {
          const double denom = curr.lon - prev.lon;
          if (std::fabs(denom) > 1e-9) {
            const double t = (edge - prev.lon) / denom;
            out.push_back({prev.lat + t * (curr.lat - prev.lat), edge});
          }
        }
      }
      out.push_back(curr);
    } else if (prevInside) {
      if (vertical) {
        const double denom = curr.lat - prev.lat;
        if (std::fabs(denom) > 1e-9) {
          const double t = (edge - prev.lat) / denom;
          out.push_back({edge, prev.lon + t * (curr.lon - prev.lon)});
        }
      } else {
        const double denom = curr.lon - prev.lon;
        if (std::fabs(denom) > 1e-9) {
          const double t = (edge - prev.lon) / denom;
          out.push_back({prev.lat + t * (curr.lat - prev.lat), edge});
        }
      }
    }
    prev = curr;
    prevInside = currInside;
  }
}

void clipGeoPolygonToBox(const std::vector<GeoPoint>& in, double latMin,
                         double latMax, double lonMin, double lonMax,
                         std::vector<GeoPoint>& out) {
  static thread_local std::vector<GeoPoint> stage;
  stage = in;
  clipGeoRingAgainstEdge(stage, out, latMin, true, true);
  if (out.empty()) return;
  clipGeoRingAgainstEdge(out, stage, latMax, true, false);
  if (stage.empty()) {
    out.clear();
    return;
  }
  clipGeoRingAgainstEdge(stage, out, lonMin, false, true);
  if (out.empty()) return;
  clipGeoRingAgainstEdge(out, stage, lonMax, false, false);
  out = stage;
}

bool clipGeoSegmentToBox(const GeoPoint& a, const GeoPoint& b, double latMin,
                         double latMax, double lonMin, double lonMax,
                         GeoPoint& outA, GeoPoint& outB) {
  double tLo = 0.0;
  double tHi = 1.0;
  const double dLat = b.lat - a.lat;
  const double dLon = b.lon - a.lon;
  const double p[4] = {-dLat, dLat, -dLon, dLon};
  const double q[4] = {a.lat - latMin, latMax - a.lat, a.lon - lonMin,
                       lonMax - a.lon};
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(p[i]) < 1e-12) {
      if (q[i] < 0.0) return false;
    } else {
      const double t = q[i] / p[i];
      if (p[i] < 0.0) {
        if (t > tLo) tLo = t;
      } else {
        if (t < tHi) tHi = t;
      }
    }
  }
  if (tHi < tLo) return false;
  outA = {a.lat + tLo * dLat, a.lon + tLo * dLon};
  outB = {a.lat + tHi * dLat, a.lon + tHi * dLon};
  return true;
}

// Mercator subdivision budget: shore edges (landmass, coast, lakes) are
// tessellated to ~kMaxShoreEdgePx so the land/water boundary stays smooth at
// every range. Political borders keep a wider chord budget.
double maxGeoSegNmFor(const MapLandLine& line, float rangeNm,
                      float pixelsPerNm) {
  switch (line.landClass) {
    case LandClass::LandMass:
    case LandClass::Coast:
    case LandClass::Lake: {
      const double pxNm = std::max(0.01, static_cast<double>(pixelsPerNm));
      const double shorePx =
          rangeNm <= 2.5f ? 1.0 : rangeNm <= 5.0f ? 1.25 : 2.0;
      const double fromScreen = shorePx / pxNm;
      if (line.landClass == LandClass::LandMass &&
          line.points.size() <= 8000) {
        if (rangeNm <= 15.0f) return fromScreen;
        const double coarseNm =
            std::max(0.2, static_cast<double>(rangeNm) * 0.012);
        return std::max(fromScreen, coarseNm);
      }
      if (line.landClass == LandClass::LandMass) {
        if (rangeNm <= kDetailLandMaxRangeNm && line.points.size() > 8000) {
          return fromScreen;
        }
        const double fineCap =
            std::max(0.12, static_cast<double>(rangeNm) * 0.01);
        return std::min(fromScreen, fineCap);
      }
      return fromScreen;
    }
    default:
      break;
  }
  return std::max(25.0, static_cast<double>(rangeNm) * 0.08);
}

void subdivideGeoEndpoints(const GeoPoint& start, const GeoPoint& end,
                           double maxSegNm, std::vector<GeoPoint>& out) {
  out.clear();
  out.push_back(start);
  GeoPoint head = start;
  while (true) {
    const double distNm =
        navDistanceNm(head.lat, head.lon, end.lat, end.lon);
    if (distNm <= maxSegNm) {
      out.push_back(end);
      break;
    }
    const double bearing =
        navBearingDeg(head.lat, head.lon, end.lat, end.lon);
    const double cosLat =
        std::max(0.05, std::cos(head.lat * map::kDegToRad));
    const double brgRad = bearing * map::kDegToRad;
    const double stepLat = (maxSegNm * std::cos(brgRad)) / map::kNmPerDegLat;
    const double stepLon =
        (maxSegNm * std::sin(brgRad)) / map::kNmPerDegLat / cosLat;
    head = {head.lat + stepLat, head.lon + stepLon};
    out.push_back(head);
  }
}

void drawContinentalChartOutline(Renderer& r, const MapLandLine& line,
                                 const Proj& proj, float rangeNm,
                                 const ClipBounds& clip, float widthPx,
                                 const Color& color, float marginPx) {
  if (line.points.size() < 2) return;
  double latMin = 0.0, latMax = 0.0, lonMin = 0.0, lonMax = 0.0;
  visibleGeoBounds(proj, rangeNm, marginPx, latMin, latMax, lonMin, lonMax);
  const double maxSegNm = maxGeoSegNmFor(line, rangeNm, proj.pixelsPerNm);
  static thread_local std::vector<GeoPoint> chain;
  for (size_t i = 0; i + 1 < line.points.size(); ++i) {
    GeoPoint a = line.points[i];
    GeoPoint b = line.points[i + 1];
    if (!clipGeoSegmentToBox(a, b, latMin, latMax, lonMin, lonMax, a, b)) {
      continue;
    }
    subdivideGeoEndpoints(a, b, maxSegNm, chain);
    for (size_t j = 0; j + 1 < chain.size(); ++j) {
      float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
      proj.toPx(chain[j].lat, chain[j].lon, x0, y0);
      proj.toPx(chain[j + 1].lat, chain[j + 1].lon, x1, y1);
      const Point seg[2] = {{x0, y0}, {x1, y1}};
      strokeClippedPolyline(r, seg, 2, widthPx, color, clip, marginPx);
    }
  }
}

bool projectGeoLine(std::vector<GeoPoint> work, const MapLandLine& line,
                    const Proj& proj, float rangeNm, std::vector<Point>& pts) {
  pts.clear();
  if (work.size() < 2) return false;

  // Mercator x-scale grows with sec(lat). At continental range a single
  // high-latitude edge can project to tens of thousands of pixels, so a
  // clipped stroke still spans the viewport and fills paint spurious wedges.
  // Subdivide long geographic segments before projection.
  static thread_local std::vector<GeoPoint> geoPts;
  geoPts.clear();
  geoPts.push_back(work[0]);
  const double maxSegNm = maxGeoSegNmFor(line, rangeNm, proj.pixelsPerNm);
  for (size_t i = 1; i < work.size(); ++i) {
    const GeoPoint& target = work[i];
    while (true) {
      const GeoPoint& start = geoPts.back();
      const double distNm =
          navDistanceNm(start.lat, start.lon, target.lat, target.lon);
      if (distNm <= maxSegNm) break;
      const double bearing =
          navBearingDeg(start.lat, start.lon, target.lat, target.lon);
      const double cosLat =
          std::max(0.05, std::cos(start.lat * map::kDegToRad));
      const double brgRad = bearing * map::kDegToRad;
      const double stepLat =
          (maxSegNm * std::cos(brgRad)) / map::kNmPerDegLat;
      const double stepLon =
          (maxSegNm * std::sin(brgRad)) / map::kNmPerDegLat / cosLat;
      geoPts.push_back({start.lat + stepLat, start.lon + stepLon});
    }
    geoPts.push_back(target);
  }

  pts.reserve(geoPts.size());
  for (const GeoPoint& g : geoPts) {
    float x = 0.0f, y = 0.0f;
    proj.toPx(g.lat, g.lon, x, y);
    pts.push_back({x, y});
  }
  return pts.size() >= 2;
}

bool projectLine(const MapLandLine& line, const Proj& proj, float rangeNm,
                 std::vector<Point>& pts) {
  static thread_local std::vector<GeoPoint> work;
  work.assign(line.points.begin(), line.points.end());
  return projectGeoLine(std::move(work), line, proj, rangeNm, pts);
}

bool projectFillLine(const MapLandLine& line, const Proj& proj, float rangeNm,
                     std::vector<Point>& pts) {
  static thread_local std::vector<GeoPoint> work;
  work.assign(line.points.begin(), line.points.end());
  float geoSpan = 0.0f;
  if (!work.empty()) {
    double minLat = work[0].lat, maxLat = work[0].lat;
    double minLon = work[0].lon, maxLon = work[0].lon;
    for (const GeoPoint& g : work) {
      minLat = std::min(minLat, g.lat);
      maxLat = std::max(maxLat, g.lat);
      minLon = std::min(minLon, g.lon);
      maxLon = std::max(maxLon, g.lon);
    }
    geoSpan = static_cast<float>((maxLat - minLat) + (maxLon - minLon));
  }
  const bool coarseSilhouetteFill =
      line.landClass == LandClass::LandMass &&
      line.points.size() <= 8000 && rangeNm > kDetailLandMaxRangeNm &&
      geoSpan <= 40.0f;
  if (work.size() >= 3 && !coarseSilhouetteFill) {
    double latMin = 0.0, latMax = 0.0, lonMin = 0.0, lonMax = 0.0;
    visibleGeoBounds(proj, rangeNm, kChartBorderClipMarginPx, latMin, latMax,
                     lonMin, lonMax);
    static thread_local std::vector<GeoPoint> clippedGeo;
    clipGeoPolygonToBox(work, latMin, latMax, lonMin, lonMax, clippedGeo);
    if (clippedGeo.size() < 3) return false;
    work.swap(clippedGeo);
  }
  return projectGeoLine(std::move(work), line, proj, rangeNm, pts);
}

std::size_t maxLandFillVertsFor(float rangeNm) {
  if (rangeNm <= 2.5f) return 3072;
  if (rangeNm <= 5.0f) return 2560;
  if (rangeNm <= 15.0f) return 2048;
  if (rangeNm <= 50.0f) return 1536;
  return kMaxLandFillVerts;
}

double screenRingSignedArea(const std::vector<Point>& pts) {
  double area = 0.0;
  const std::size_t n = pts.size();
  if (n < 3) return 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    area += static_cast<double>(pts[j].x + pts[i].x) *
            static_cast<double>(pts[j].y - pts[i].y);
  }
  return area * 0.5;
}

void drawLandMassFillFromScreenPts(Renderer& r, const std::vector<Point>& pts,
                                   const ClipBounds& clip, float rangeNm) {
  const int n = static_cast<int>(pts.size());
  if (n < 3) return;
  static thread_local std::vector<Point> clipped;
  static thread_local std::vector<Point> simplified;
  static thread_local std::vector<Point> fillPts;
  clipPolygonToRect(pts.data(), n, clip, 0.0f, clipped);
  if (clipped.size() < 3) return;
  simplifyColinearRing(clipped,
                       rangeNm <= 2.5f ? 0.04f
                                       : rangeNm <= 5.0f ? 0.05f
                                                          : rangeNm <= 15.0f
                                                                ? 0.08f
                                                                : 0.25f,
                       simplified);
  const std::vector<Point>& src =
      simplified.size() >= 3 ? simplified : clipped;
  decimateClosedPolygon(src, maxLandFillVertsFor(rangeNm), fillPts);
  if (fillPts.size() < 3) {
    return;
  }
  float minX = fillPts[0].x;
  float maxX = fillPts[0].x;
  float minY = fillPts[0].y;
  float maxY = fillPts[0].y;
  for (const Point& p : fillPts) {
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
  }
  const float viewW = clip.maxX - clip.minX;
  const float viewH = clip.maxY - clip.minY;
  const float fillW = maxX - minX;
  const float fillH = maxY - minY;
  // Mercator lon-band slices project as tall wedges or diagonal triangles over
  // the ocean when a coarse geographic ring is filled with screen-space chords.
  if (rangeNm >= 75.0f &&
      fillH > viewH * 0.35f && fillW < viewW * 0.28f) {
    return;
  }
  const double fillArea = std::fabs(screenRingSignedArea(fillPts));
  const double bboxArea = static_cast<double>(fillW) * static_cast<double>(fillH);
  const double minFillDensity =
      rangeNm <= 5.0f ? 0.10 : rangeNm <= 15.0f ? 0.12 : 0.22;
  if (rangeNm <= 120.0f && bboxArea > 1.0 && fillH > viewH * 0.45f &&
      fillArea / bboxArea < minFillDensity) {
    return;
  }
  if (rangeNm >= kContinentalChartRangeNm &&
      fillW >= viewW * 0.2f &&
      fillH > viewH * 0.65f && fillW < viewW * 0.55f) {
    return;
  }
  if (screenRingSignedArea(fillPts) < 0.0) {
    std::reverse(fillPts.begin(), fillPts.end());
  }
  r.fillPolygon(fillPts.data(), static_cast<int>(fillPts.size()),
                kMapLandFill);
}

void drawProjectedLine(Renderer& r, const MapLandLine& line,
                       const std::vector<Point>& pts, float rangeNm,
                       const ClipBounds& clip) {
  const int n = static_cast<int>(pts.size());
  switch (line.landClass) {
    case LandClass::LandMass:
      drawLandMassFillFromScreenPts(r, pts, clip, rangeNm);
      break;
    case LandClass::Lake: {
      static thread_local std::vector<Point> clipped;
      static thread_local std::vector<Point> fillPts;
      clipPolygonToRect(pts.data(), n, clip, 0.0f, clipped);
      if (clipped.size() >= 3) {
        decimateClosedPolygon(clipped, kMaxLandFillVerts, fillPts);
        if (fillPts.size() >= 3) {
          r.fillPolygon(fillPts.data(), static_cast<int>(fillPts.size()),
                        kMapLakeFill);
        }
      }
      break;
    }
    case LandClass::River:
      strokeClippedPolyline(r, pts.data(), n, 1.2f, kRiverStroke, clip,
                            kSymbologyClipMarginPx);
      break;
    case LandClass::Road:
      strokeClippedPolyline(r, pts.data(), n, 1.2f, kRoadStroke, clip,
                            kSymbologyClipMarginPx);
      break;
    case LandClass::Border:
      if (rangeNm >= 150.0f) {
        strokeClippedPolyline(r, pts.data(), n, chartOutlineWidthPx(rangeNm),
                              kChartOutlineStroke, clip,
                              kChartBorderClipMarginPx);
      } else {
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kBorderStroke, &clip);
      }
      break;
    case LandClass::StateBorder:
      strokeDashedPolyline(r, pts.data(), n, 1.0f, kStateBorderStroke, &clip);
      break;
    case LandClass::Coast:
      break;
    case LandClass::Railroad:
      drawRailroad(r, pts.data(), n, kRailroadStroke, &clip);
      break;
    case LandClass::City:
      break;
  }
}

bool shouldDrawLine(const MapLandLine& line, float rangeNm) {
  switch (line.landClass) {
    case LandClass::LandMass:
      return true;
    case LandClass::Road:
      return rangeNm <= kRoadMaxRangeNm;
    case LandClass::River:
      return rangeNm <= kRiverMaxRangeNm;
    case LandClass::Lake:
      return rangeNm <= kLakeMaxRangeNm;
    case LandClass::StateBorder:
      return rangeNm <= kStateBorderMaxRangeNm;
    case LandClass::Coast:
      return false;
    case LandClass::Railroad:
      return rangeNm <= kRailroadMaxRangeNm;
    default:
      break;
  }
  return true;
}

}  // namespace

void drawLandData(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm, bool skipLandMassFill) {
  const ClipBounds clip{proj.minX, proj.minY, proj.maxX, proj.maxY};
  std::vector<Point> pts;

  // Continent fills first so lakes and detail lines layer on top. Draw large
  // continental rings before regional/island rings so local fills stay on top.
  {
    static thread_local std::vector<std::size_t> landIdx;
    landIdx.clear();
    landIdx.reserve(map.landLines.size());
    for (std::size_t i = 0; i < map.landLines.size(); ++i) {
      const MapLandLine& line = map.landLines[i];
      if (line.landClass == LandClass::LandMass && line.points.size() >= 2) {
        landIdx.push_back(i);
      }
    }
    std::sort(landIdx.begin(), landIdx.end(), [&](std::size_t a, std::size_t b) {
      const std::size_t pa = map.landLines[a].points.size();
      const std::size_t pb = map.landLines[b].points.size();
      if (rangeNm <= 120.0f) {
        // Continental silhouette under regional lon-bands under local islands.
        auto fillTier = [](std::size_t npts) {
          if (npts > 8000) return 1;
          if (npts > 3500) return 0;
          return 2;
        };
        const int ta = fillTier(pa);
        const int tb = fillTier(pb);
        if (ta != tb) return ta < tb;
        return pa > pb;
      }
      const int ta = pa <= 8000 ? 0 : 1;
      const int tb = pb <= 8000 ? 0 : 1;
      if (ta != tb) return ta < tb;
      return pa > pb;
    });
    for (std::size_t i : landIdx) {
      const MapLandLine& line = map.landLines[i];
      if (skipLandMassFill && line.points.size() > 8000) continue;
      if (!projectFillLine(line, proj, rangeNm, pts)) continue;
      if (!polylineIntersectsClip(pts.data(), static_cast<int>(pts.size()), clip,
                                  kChartBorderClipMarginPx)) {
        continue;
      }
      drawProjectedLine(r, line, pts, rangeNm, clip);
    }
  }

  for (const MapLandLine& line : map.landLines) {
    if (line.landClass == LandClass::LandMass || line.points.size() < 2) {
      continue;
    }
    if (line.landClass == LandClass::Border ||
        line.landClass == LandClass::Coast) {
      continue;
    }
    if (!shouldDrawLine(line, rangeNm)) continue;
    const bool isFill = line.landClass == LandClass::Lake;
    if (!(isFill ? projectFillLine(line, proj, rangeNm, pts)
                 : projectLine(line, proj, rangeNm, pts))) {
      continue;
    }
    if (!polylineIntersectsClip(pts.data(), static_cast<int>(pts.size()), clip,
                                kSymbologyClipMarginPx)) {
      continue;
    }
    drawProjectedLine(r, line, pts, rangeNm, clip);
  }

  // Political borders only at wide range; shoreline is land fill vs ocean.
  for (const MapLandLine& line : map.landLines) {
    if (line.landClass != LandClass::Border) {
      continue;
    }
    if (line.points.size() < 2) continue;
    if (!shouldDrawLine(line, rangeNm)) continue;
    if (rangeNm >= kContinentalChartRangeNm) {
      drawContinentalChartOutline(r, line, proj, rangeNm, clip,
                                  chartOutlineWidthPx(rangeNm),
                                  kChartOutlineStroke,
                                  kChartBorderClipMarginPx);
      continue;
    }
    if (!projectLine(line, proj, rangeNm, pts)) continue;
    if (!polylineIntersectsClip(pts.data(), static_cast<int>(pts.size()), clip,
                                kChartBorderClipMarginPx)) {
      continue;
    }
    drawProjectedLine(r, line, pts, rangeNm, clip);
  }
}

void drawCityDots(Renderer& r, const MapData& map, const Proj& proj,
                  float rangeNm, float symSize) {
  for (const MapLandCity& label : map.cities) {
    if (label.labelKind != LandLabelKind::City) continue;
    if (rangeNm > maxLabelRangeNm(label)) continue;
    if (!continentalLabelVisible(label, rangeNm)) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.lat, label.lon, x, y);
    if (!proj.onScreen(x, y, symSize)) continue;
    r.fillCircle(x, y, symSize * 0.28f, kCityDot);
  }
}

void drawMapPlaceLabels(Renderer& r, const MapData& map, const Proj& proj,
                        float rangeNm, float labelSize) {
  struct GeoLabelDraw {
    const MapLandCity* label;
    float x;
    float y;
    float size;
    bool hydro;
  };

  std::vector<GeoLabelDraw> regions;
  std::vector<GeoLabelDraw> hydros;
  regions.reserve(map.cities.size());
  hydros.reserve(map.cities.size());

  for (const MapLandCity& label : map.cities) {
    if (label.labelKind == LandLabelKind::City) continue;
    if (rangeNm > maxLabelRangeNm(label)) continue;
    if (!continentalLabelVisible(label, rangeNm)) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.lat, label.lon, x, y);
    if (!proj.onScreen(x, y, labelSize)) continue;

    const bool hydro = label.labelKind == LandLabelKind::Hydro;
    const float size =
        hydro ? hydroLabelSize(labelSize, label.rank, rangeNm)
              : regionLabelSize(labelSize, label.rank, rangeNm);
    (hydro ? hydros : regions).push_back({&label, x, y, size, hydro});
  }

  auto sortGeo = [](const GeoLabelDraw& a, const GeoLabelDraw& b) {
    if (a.label->rank != b.label->rank) {
      return a.label->rank > b.label->rank;
    }
    return a.label->name < b.label->name;
  };
  std::sort(regions.begin(), regions.end(), sortGeo);
  std::sort(hydros.begin(), hydros.end(), sortGeo);

  auto drawGeoBatch = [&](const std::vector<GeoLabelDraw>& batch, bool hydro) {
    std::vector<TextRect> placed;
    placed.reserve(batch.size());
    for (const GeoLabelDraw& item : batch) {
      const TextRect tr = r.measureTextRect(item.x, item.y, item.label->name,
                                            item.size, TextAlign::Center,
                                            kGeoLabelFace);
      const float pad =
          item.size *
          ((hydro && rangeNm >= kWideChartRangeNm) ? 0.06f : 0.12f);
      bool clash = false;
      for (const TextRect& p : placed) {
        if (textRectsOverlap(tr, p, pad)) {
          clash = true;
          break;
        }
      }
      if (clash) continue;

      r.fillText(item.x, item.y - kMapLabelLiftPx, item.label->name, item.size,
                 TextAlign::Center, hydro ? kHydroLabel : colors::kWhite,
                 kGeoLabelFace);
      placed.push_back(expandTextRect(tr, pad));
    }
  };

  drawGeoBatch(regions, false);
  drawGeoBatch(hydros, true);

  for (const MapLandCity& label : map.cities) {
    if (label.labelKind != LandLabelKind::City) continue;
    if (rangeNm > maxLabelRangeNm(label)) continue;
    if (!continentalLabelVisible(label, rangeNm)) continue;
    float x = 0.0f, y = 0.0f;
    proj.toPx(label.lat, label.lon, x, y);
    if (!proj.onScreen(x, y, labelSize)) continue;
    const float textSize = labelSize * kMapIdentLabelScale;
    r.fillText(x, y - textSize * 0.85f - kMapLabelLiftPx, label.name, textSize,
               TextAlign::Center, colors::kWhite, kMapLabelFace);
  }
}

void drawCities(Renderer& r, const MapData& map, const Proj& proj,
                float rangeNm, float symSize, float labelSize) {
  drawCityDots(r, map, proj, rangeNm, symSize);
  drawMapPlaceLabels(r, map, proj, rangeNm, labelSize);
}

}  // namespace avionics::mapview
