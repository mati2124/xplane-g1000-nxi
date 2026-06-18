#include "render/map/MapViewInternal.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "avionics/MapRange.h"
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
// State/province lines: see kStateBorderMaxRangeNm in MapRange.h.
// GSHHG crude silhouettes (South America, North America, …) span well over
// 40°; at 500+ NM clip them to the visible chart before chord tessellation.
constexpr float kCoarseSilhouetteMaxGeoSpanDeg = 40.0f;
constexpr float kContinentalSilhouetteMinGeoSpanDeg = 50.0f;
constexpr std::size_t kSilhouetteLandMaxPts = 8000;
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
      if (label.rank >= 5) return kStateBorderMaxRangeNm;
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
      if (rangeNm > kStateBorderMaxRangeNm) return label.rank >= 6;
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
  const float x0 = proj.minX - marginPx;
  const float x1 = proj.maxX + marginPx;
  const float y0 = proj.minY - marginPx;
  const float y1 = proj.maxY + marginPx;
  latMin = 90.0;
  latMax = -90.0;
  double lonMinDelta = 180.0;
  double lonMaxDelta = -180.0;
  const double pxPer = static_cast<double>(proj.mercatorPxPerRad);
  auto sampleCorner = [&](float sx, float sy) {
    const double mapEast = (sx - proj.cx) / pxPer;
    const double mapNorth = (proj.cy - sy) / pxPer;
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
  };
  constexpr int kEdgeSamples = 8;
  for (int i = 0; i <= kEdgeSamples; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kEdgeSamples);
    sampleCorner(x0 + t * (x1 - x0), y0);
    sampleCorner(x0 + t * (x1 - x0), y1);
  }
  for (int i = 1; i < kEdgeSamples; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kEdgeSamples);
    sampleCorner(x0, y0 + t * (y1 - y0));
    sampleCorner(x1, y0 + t * (y1 - y0));
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
      const double shorePx = rangeNm <= 5.0f ? 1.0 : 2.0;
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

float landLineGeoSpan(const MapLandLine& line) {
  if (line.points.empty()) return 0.0f;
  double minLat = line.points[0].lat, maxLat = line.points[0].lat;
  double minLon = line.points[0].lon, maxLon = line.points[0].lon;
  for (const GeoPoint& g : line.points) {
    minLat = std::min(minLat, g.lat);
    maxLat = std::max(maxLat, g.lat);
    minLon = std::min(minLon, g.lon);
    maxLon = std::max(maxLon, g.lon);
  }
  return static_cast<float>((maxLat - minLat) + (maxLon - minLon));
}

bool projectFillLine(const MapLandLine& line, const Proj& proj, float rangeNm,
                     std::vector<Point>& pts) {
  static thread_local std::vector<GeoPoint> work;
  work.assign(line.points.begin(), line.points.end());
  const float geoSpan = landLineGeoSpan(line);
  const bool coarseSilhouetteFill =
      line.landClass == LandClass::LandMass &&
      line.points.size() <= kSilhouetteLandMaxPts &&
      rangeNm > kDetailLandMaxRangeNm &&
      geoSpan <= kCoarseSilhouetteMaxGeoSpanDeg;
  // Geo-clip every wide landmass (including continental silhouettes) to the
  // visible chart before Mercator chord tessellation; an unclipped Americas
  // ring fills the entire viewport black.
  if (work.size() >= 3 && !coarseSilhouetteFill) {
    double latMin = 0.0, latMax = 0.0, lonMin = 0.0, lonMax = 0.0;
    visibleGeoBounds(proj, rangeNm, kChartBorderClipMarginPx, latMin, latMax,
                     lonMin, lonMax);
    // Below 5 NM the asymmetric western clip leaves a straight vertical ocean
    // seam on the chart edge; above 5 NM it keeps regional lon-band chord fills
    // out of barrier-island sounds (see 4-point bbox ring filter too).
    if (rangeNm > 5.0f && rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
        line.landClass == LandClass::LandMass &&
        line.points.size() > kSilhouetteLandMaxPts &&
        geoSpan >= kRegionalLonBandMinGeoSpanDeg) {
      lonMin += (lonMax - lonMin) * 0.11;
    }
    static thread_local std::vector<GeoPoint> clippedGeo;
    clipGeoPolygonToBox(work, latMin, latMax, lonMin, lonMax, clippedGeo);
    if (clippedGeo.size() < 3) return false;
    work.swap(clippedGeo);
  }
  return projectGeoLine(std::move(work), line, proj, rangeNm, pts);
}

std::size_t maxLandFillVertsFor(float rangeNm) {
  if (rangeNm <= 5.0f) return 3072;
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

// Mercator chord fills on coarse GSHHG rings project as diagonal triangles that
// swamp Pine Island–scale inlet detail at 10 NM even when geo-clipped.
bool isCloseRangeMercatorWedge(float rangeNm, float geoSpanDeg,
                               std::size_t sourceNpts,
                               const std::vector<Point>& fillPts, float fillW,
                               float fillH, float viewW, float viewH,
                               double fillArea, double bboxArea,
                               float fillCx, float clipCx) {
  if (rangeNm > kRegionalSilhouetteSuppressMaxNm || bboxArea < 1.0) {
    return false;
  }
  const double density = fillArea / bboxArea;

  // Legitimate shore/island rings (Pine Island is ~139 pts @ geoSpan 0.35).
  if (sourceNpts >= 50 && geoSpanDeg < 5.0f) {
    return false;
  }

  // GSHHG 4-point bbox rings chord-fill as large Mercator triangles over inlet
  // water (Pine Island Sound @ 10 NM) despite tiny geographic span.
  if (sourceNpts <= 4 && geoSpanDeg < 2.0f && fillW > viewW * 0.12f &&
      fillH > viewH * 0.22f && density < 0.42f) {
    return true;
  }

  // Regional lon-band underlay often chord-fills west over Pine Island Sound.
  if (sourceNpts > kSilhouetteLandMaxPts &&
      geoSpanDeg >= kRegionalLonBandMinGeoSpanDeg &&
      fillCx < clipCx - viewW * 0.04f && density < 0.72) {
    return true;
  }

  // True local islands stay small on screen at 10 NM.
  if (geoSpanDeg < 22.0f && fillW < viewW * 0.28f && fillH < viewH * 0.28f) {
    return false;
  }

  int diagonalLong = 0;
  int onBboxPerim = 0;
  float ringMinPx = fillPts[0].x;
  float maxPx = fillPts[0].x;
  float minPy = fillPts[0].y;
  float maxPy = fillPts[0].y;
  for (const Point& p : fillPts) {
    ringMinPx = std::min(ringMinPx, p.x);
    maxPx = std::max(maxPx, p.x);
    minPy = std::min(minPy, p.y);
    maxPy = std::max(maxPy, p.y);
  }
  constexpr float kBboxPerimEps = 2.5f;
  if (fillW > viewW * 0.08f && fillH > viewH * 0.08f) {
    for (std::size_t i = 0; i < fillPts.size(); ++i) {
      const Point& p = fillPts[i];
      if (std::fabs(p.x - ringMinPx) <= kBboxPerimEps ||
          std::fabs(p.x - maxPx) <= kBboxPerimEps ||
          std::fabs(p.y - minPy) <= kBboxPerimEps ||
          std::fabs(p.y - maxPy) <= kBboxPerimEps) {
        ++onBboxPerim;
      }
      const Point& a = fillPts[i];
      const Point& b = fillPts[(i + 1) % fillPts.size()];
      const float dx = std::fabs(b.x - a.x);
      const float dy = std::fabs(b.y - a.y);
      const float len = std::hypot(dx, dy);
      if (len >= viewW * 0.08f && dx > viewW * 0.03f && dy > viewH * 0.03f) {
        ++diagonalLong;
      }
    }
  }

  if (fillPts.size() >= 3 &&
      onBboxPerim >= static_cast<int>(fillPts.size() * 0.72) &&
      fillW > viewW * 0.10f && fillH > viewH * 0.10f && density < 0.62 &&
      (sourceNpts > kSilhouetteLandMaxPts || geoSpanDeg >= 18.0f)) {
    return true;
  }

  // Axis-aligned Mercator chord triangle (exactly ~½ the bbox; no diagonal edge).
  if (fillW > viewW * 0.10f && fillH > viewH * 0.10f && density >= 0.44 &&
      density <= 0.56 &&
      (sourceNpts > kSilhouetteLandMaxPts || geoSpanDeg >= 18.0f)) {
    return true;
  }

  // Screen chord triangle: ~half the bbox area with a long diagonal edge.
  if (fillW > viewW * 0.10f && fillH > viewH * 0.10f && density >= 0.36 &&
      density <= 0.58 && diagonalLong >= 1) {
    return true;
  }

  if (diagonalLong >= 1 && density < 0.65 &&
      (sourceNpts > kSilhouetteLandMaxPts || geoSpanDeg >= 20.0f) &&
      fillW > viewW * 0.08f && fillH > viewH * 0.08f) {
    return true;
  }

  if (sourceNpts > kSilhouetteLandMaxPts &&
      geoSpanDeg >= kRegionalLonBandMinGeoSpanDeg &&
      density < 0.62 &&
      (fillW > viewW * 0.12f || fillH > viewH * 0.12f)) {
    return true;
  }

  if (geoSpanDeg >= 25.0f && geoSpanDeg < kContinentalSilhouetteMinGeoSpanDeg) {
    if (fillH > viewH * 0.18f && fillW < viewW * 0.38f) return true;
    if (fillW > viewW * 0.18f && fillH < viewH * 0.38f && density < 0.45) {
      return true;
    }
    if (density < 0.52 && fillW > viewW * 0.16f && fillH > viewH * 0.16f) {
      return true;
    }
  }

  return false;
}

void drawLandMassFillFromScreenPts(Renderer& r, const std::vector<Point>& pts,
                                   const ClipBounds& clip, float rangeNm,
                                   float geoSpanDeg, std::size_t sourceNpts) {
  const int n = static_cast<int>(pts.size());
  if (n < 3) return;
  static thread_local std::vector<Point> clipped;
  static thread_local std::vector<Point> simplified;
  static thread_local std::vector<Point> fillPts;
  clipPolygonToRect(pts.data(), n, clip, 0.0f, clipped);
  if (clipped.size() < 3) return;
  const bool continentalFill =
      rangeNm >= kContinentalChartRangeNm &&
      geoSpanDeg >= kContinentalSilhouetteMinGeoSpanDeg;
  if (!continentalFill) {
    simplifyColinearRing(clipped,
                         rangeNm <= 5.0f ? 0.04f
                                         : rangeNm <= 15.0f ? 0.08f
                                                              : 0.25f,
                         simplified);
  }
  const std::vector<Point>& src =
      !continentalFill && simplified.size() >= 3 ? simplified : clipped;
  decimateClosedPolygon(src,
                        continentalFill ? kMaxLandFillVerts
                                        : maxLandFillVertsFor(rangeNm),
                        fillPts);
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
  // Mercator lon-band slices project as tall wedges over the ocean; local
  // barrier islands (Pine Island @ 10 NM) are also tall/narrow and must pass.
  if (rangeNm <= 120.0f && geoSpanDeg < kContinentalSilhouetteMinGeoSpanDeg &&
      fillH > viewH * 0.35f && fillW < viewW * 0.28f &&
      (sourceNpts > kSilhouetteLandMaxPts || geoSpanDeg >= 18.0f)) {
    return;
  }
  const double fillArea = std::fabs(screenRingSignedArea(fillPts));
  const double bboxArea = static_cast<double>(fillW) * static_cast<double>(fillH);
  // Close range: regional lon-band chord fills (40–55°) that project as sparse
  // wedges or tall triangles over inlet detail.
  if (rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
      sourceNpts > kSilhouetteLandMaxPts &&
      geoSpanDeg >= kRegionalLonBandMinGeoSpanDeg &&
      geoSpanDeg < kContinentalSilhouetteMinGeoSpanDeg + 5.0f &&
      bboxArea > 1.0) {
    if (fillArea / bboxArea < 0.22) return;
    if (fillH > viewH * 0.26f && fillW < viewW * 0.33f) return;
    if (fillH > viewH * 0.22f && fillW > viewW * 0.38f &&
        fillArea / bboxArea < 0.34) {
      return;
    }
    if (fillArea / bboxArea < 0.38 &&
        (fillW > viewW * 0.30f || fillH > viewH * 0.30f)) {
      return;
    }
  }
  if (rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
      sourceNpts <= kSilhouetteLandMaxPts &&
      geoSpanDeg >= 30.0f && geoSpanDeg < kRegionalLonBandMinGeoSpanDeg &&
      bboxArea > 1.0) {
    if (fillH > viewH * 0.28f && fillW < viewW * 0.34f) return;
    if (fillArea / bboxArea < 0.16 && fillH > viewH * 0.20f) return;
  }
  const double minFillDensity =
      rangeNm <= 5.0f ? 0.10 : rangeNm <= 15.0f ? 0.12 : 0.22;
  if (rangeNm <= 120.0f && bboxArea > 1.0 && fillH > viewH * 0.45f &&
      fillArea / bboxArea < minFillDensity) {
    return;
  }
  if (rangeNm >= kContinentalChartRangeNm &&
      geoSpanDeg < kContinentalSilhouetteMinGeoSpanDeg &&
      fillW >= viewW * 0.2f && fillH > viewH * 0.65f && fillW < viewW * 0.55f) {
    return;
  }
  const float fillCx = (minX + maxX) * 0.5f;
  const float clipCx = (clip.minX + clip.maxX) * 0.5f;
  if (isCloseRangeMercatorWedge(rangeNm, geoSpanDeg, sourceNpts, fillPts, fillW,
                                fillH, viewW, viewH, fillArea, bboxArea, fillCx,
                                clipCx)) {
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
                       const ClipBounds& clip, float geoSpanDeg) {
  const int n = static_cast<int>(pts.size());
  switch (line.landClass) {
    case LandClass::LandMass:
      drawLandMassFillFromScreenPts(r, pts, clip, rangeNm, geoSpanDeg,
                                    line.points.size());
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
      if (rangeNm >= 150.0f) {
        strokeClippedPolyline(r, pts.data(), n, chartOutlineWidthPx(rangeNm),
                              kChartOutlineStroke, clip,
                              kChartBorderClipMarginPx);
      } else {
        strokeDashedPolyline(r, pts.data(), n, 1.0f, kBorderStroke, &clip);
      }
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
      const float ga = landLineGeoSpan(map.landLines[a]);
      const float gb = landLineGeoSpan(map.landLines[b]);
      if (rangeNm <= kRegionalSilhouetteSuppressMaxNm) {
        // Close range: regional lon-band underlay, then local shore/island rings.
        auto fillTier = [](std::size_t npts, float geoSpan) {
          if (npts > kSilhouetteLandMaxPts &&
              geoSpan >= kRegionalLonBandMinGeoSpanDeg) {
            return 0;
          }
          return 1;
        };
        const int ta = fillTier(pa, ga);
        const int tb = fillTier(pb, gb);
        if (ta != tb) return ta < tb;
        if (ga < 25.0f && gb < 25.0f && pa != pb) return pa < pb;
        if (ga != gb) return ga > gb;
        return pa < pb;
      }
      if (rangeNm <= 120.0f) {
        // Continental silhouette under regional lon-bands under local shore rings.
        auto fillTier = [](std::size_t npts, float geoSpan) {
          if (geoSpan >= kContinentalSilhouetteMinGeoSpanDeg &&
              npts <= kSilhouetteLandMaxPts) {
            return 0;
          }
          if (npts > kSilhouetteLandMaxPts &&
              geoSpan >= kRegionalLonBandMinGeoSpanDeg) {
            return 1;
          }
          return 2;
        };
        const int ta = fillTier(pa, ga);
        const int tb = fillTier(pb, gb);
        if (ta != tb) return ta < tb;
        if (ta == 2) {
          if (ga != gb) return ga > gb;
          return pa < pb;
        }
        return pa > pb;
      }
      const int ta = pa <= 8000 ? 0 : 1;
      const int tb = pb <= 8000 ? 0 : 1;
      if (ta != tb) return ta < tb;
      return pa > pb;
    });
    for (std::size_t i : landIdx) {
      const MapLandLine& line = map.landLines[i];
      if (skipLandMassFill) continue;
      const float geoSpan = landLineGeoSpan(line);
      if (rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
          line.points.size() <= 4 && geoSpan > 0.2f && geoSpan < 1.0f) {
        continue;
      }
      if (rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
          geoSpan >= 8.0f && geoSpan < 25.0f &&
          line.points.size() <= kSilhouetteLandMaxPts) {
        continue;
      }
      if (rangeNm <= kRegionalSilhouetteSuppressMaxNm &&
          geoSpan >= 25.0f && geoSpan < kContinentalSilhouetteMinGeoSpanDeg) {
        continue;
      }
      if (!projectFillLine(line, proj, rangeNm, pts)) continue;
      if (!polylineIntersectsClip(pts.data(), static_cast<int>(pts.size()), clip,
                                  kChartBorderClipMarginPx)) {
        continue;
      }
      drawProjectedLine(r, line, pts, rangeNm, clip, geoSpan);
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
    drawProjectedLine(r, line, pts, rangeNm, clip, landLineGeoSpan(line));
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
    drawProjectedLine(r, line, pts, rangeNm, clip, 0.0f);
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
