#include <algorithm>
#include <cmath>

#include "avionics/MfdController.h"
#include "avionics/NavMath.h"
#include "avionics/DataSource.h"
#include "render/map/MapProjection.h"
#include "render/map/MapViewInternal.h"

// Navigation Map pointer / pan (Pilot's Guide, Map Panning): the RANGE joystick
// places a pan cursor; moving the joystick slides the cursor across the map.
// The map stays centered until the cursor reaches the inner edge of the view,
// then scrolls to keep it on screen (Working Title MapPointerRTRController).
namespace avionics {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kNmPerDeg = 60.0;

// Each FMS-knob click pans this fraction of the current map range, so panning
// covers the same on-screen distance per click at every zoom (matching the
// real unit's joystick feel, where one nudge moves a fixed part of the view
// rather than a fixed ground distance).
constexpr double kPanStepFrac = 0.10;
// Smallest pan step (NM), so the tightest range still pans noticeably.
constexpr double kPanStepMinNm = 0.05;

// How close (as a fraction of the current range) the pointer must be to a map
// feature for it to be "selected" -- highlighted and shown in the Map Pointer
// information box. Keeps the snap radius a few pixels of the pointer tip at any
// zoom.
constexpr double kMapPointerSnapFrac = 0.04;
constexpr double kMapPointerSnapMinNm = 0.1;

// Inner zone where the pointer moves freely before the map scrolls (MFDNavMapPage
// pointerBoundsOffset = 0.1 on each side).
constexpr float kPointerBoundsFrac = 0.1f;

float mapRotationDeg(MapOrientation orientation, const FlightData& d) {
  switch (orientation) {
    case MapOrientation::HeadingUp:
      return d.headingDeg;
    case MapOrientation::TrackUp:
      return d.trackDeg;
    case MapOrientation::NorthUp:
      break;
  }
  return 0.0f;
}

// WPT group page index for a map feature (Airport / Intersection / NDB / VOR).
int wptPageIndexFor(MapFeatureType type) {
  switch (type) {
    case MapFeatureType::Airport:
      return 0;
    case MapFeatureType::Fix:
    case MapFeatureType::Waypoint:
      return 1;
    case MapFeatureType::Ndb:
      return 2;
    case MapFeatureType::Vor:
      return 3;
  }
  return 0;
}

}  // namespace

void MfdController::mapResetPointer() {
  mapPointerActive_ = false;
  mapViewportValid_ = false;
}

void MfdController::setMapViewport(float x, float y, float w, float h,
                                   float displayH) {
  mapViewportX_ = x;
  mapViewportY_ = y;
  mapViewportW_ = w;
  mapViewportH_ = h;
  mapViewportDisplayH_ = displayH;
  mapViewportValid_ = w > 0.0f && h > 0.0f;
}

void MfdController::mapPointerScrollTowardPointer() {
  if (!mapPointerActive_ || mapData_ == nullptr || !mapData_->positionValid) {
    return;
  }
  const double cosLat =
      std::max(0.05, std::cos(mapPanViewCenterLat_ * kDegToRad));
  const double dLatNm =
      (mapPointerLat_ - mapPanViewCenterLat_) * kNmPerDeg;
  double dLon = mapPointerLon_ - mapPanViewCenterLon_;
  while (dLon > 180.0) dLon -= 360.0;
  while (dLon < -180.0) dLon += 360.0;
  const double dLonNm = dLon * kNmPerDeg * cosLat;
  const double distNm =
      std::sqrt(dLatNm * dLatNm + dLonNm * dLonNm);
  // Inner zone is (1 - 2*kPointerBoundsFrac) of the viewport; the pointer may
  // sit that far from the view center before the map scrolls. MFD north-up span
  // is ~4*rangeNm corner-to-corner along a meridian (mapRangeSpanFrac = 0.25).
  const double scrollThresholdNm =
      rangeNm() * (1.0f - 2.0f * kPointerBoundsFrac) * 2.0f;
  if (distNm <= scrollThresholdNm) {
    return;
  }
  const double scale = (distNm - scrollThresholdNm) / distNm;
  const double eastRad =
      map::lonDeltaDeg(mapPointerLon_, mapPanViewCenterLon_) * map::kDegToRad *
      scale;
  const double northRad =
      (map::mercatorYRad(mapPointerLat_) -
       map::mercatorYRad(mapPanViewCenterLat_)) *
      scale;
  map::offsetMercatorCenter(mapPanViewCenterLat_, mapPanViewCenterLon_,
                            eastRad, northRad, mapPanViewCenterLat_,
                            mapPanViewCenterLon_);
}

void MfdController::mapPointerSyncScroll(const FlightData& flight) {
  if (!mapPointerActive_ || !mapViewportValid_ || mapData_ == nullptr ||
      !mapData_->positionValid) {
    return;
  }

  MapViewConfig config;
  config.x = mapViewportX_;
  config.y = mapViewportY_;
  config.w = mapViewportW_;
  config.h = mapViewportH_;
  config.orientation = mapOrientation_;
  config.rangeNm = rangeNm();
  config.displayRangeNm = displayRangeNm();

  const float cx = mapViewportX_ + mapViewportW_ * 0.5f;
  const float cy = mapViewportY_ + mapViewportH_ * 0.5f;
  const float mapRadiusPx = mapview::mapRangeSpanPx(config);
  const float scaleRangeNm =
      std::max(kMapRangeMinNm, displayRangeNm() > 0.0f ? displayRangeNm() : rangeNm());
  const float pixelsPerNm = mapRadiusPx / scaleRangeNm;
  const float rotation = mapRotationDeg(mapOrientation_, flight);
  const double rot = static_cast<double>(rotation) * map::kDegToRad;
  const double cosR = std::cos(rot);
  const double sinR = std::sin(rot);
  const float mercatorPxPerRad =
      pixelsPerNm * static_cast<float>(map::kNmPerEarthRad);

  const float minX = mapViewportX_ + mapViewportW_ * kPointerBoundsFrac;
  const float maxX = mapViewportX_ + mapViewportW_ * (1.0f - kPointerBoundsFrac);
  const float minY = mapViewportY_ + mapViewportH_ * kPointerBoundsFrac;
  const float maxY = mapViewportY_ + mapViewportH_ * (1.0f - kPointerBoundsFrac);

  for (int pass = 0; pass < 2; ++pass) {
    float px = 0.0f;
    float py = 0.0f;
    map::latLonToLocalPx(mapPointerLat_, mapPointerLon_, mapPanViewCenterLat_,
                         mapPanViewCenterLon_, cx, cy, pixelsPerNm, rotation,
                         px, py);

    const float clx = std::clamp(px, minX, maxX);
    const float cly = std::clamp(py, minY, maxY);
    const float scrollDx = px - clx;
    const float scrollDy = py - cly;
    if (scrollDx == 0.0f && scrollDy == 0.0f) {
      return;
    }

    double eastRad = 0.0;
    double northRad = 0.0;
    map::screenDeltaToMercatorRad(scrollDx, scrollDy, mercatorPxPerRad, cosR,
                                  sinR, eastRad, northRad);
    map::offsetMercatorCenter(mapPanViewCenterLat_, mapPanViewCenterLon_,
                              eastRad, northRad, mapPanViewCenterLat_,
                              mapPanViewCenterLon_);
  }
}

const MapFeature* MfdController::mapPointerFeature() const {
  if (!mapPointerActive_ || mapData_ == nullptr) return nullptr;
  const double snapNm =
      std::max(kMapPointerSnapMinNm, rangeNm() * kMapPointerSnapFrac);
  const MapFeature* best = nullptr;
  double bestNm = snapNm;
  for (const MapFeature& f : mapData_->features) {
    const double dNm =
        navDistanceNm(mapPointerLat_, mapPointerLon_, f.lat, f.lon);
    if (dNm < bestNm) {
      bestNm = dNm;
      best = &f;
    }
  }
  return best;
}

const MapObstacle* MfdController::mapPointerObstacle() const {
  if (!mapPointerActive_ || mapData_ == nullptr) return nullptr;
  const double snapNm =
      std::max(kMapPointerSnapMinNm, rangeNm() * kMapPointerSnapFrac);
  const MapObstacle* best = nullptr;
  double bestNm = snapNm;
  for (const MapObstacle& ob : mapData_->obstacles) {
    const double dNm =
        navDistanceNm(mapPointerLat_, mapPointerLon_, ob.lat, ob.lon);
    if (dNm < bestNm) {
      bestNm = dNm;
      best = &ob;
    }
  }
  return best;
}

bool MfdController::mapBezelKey(BezelKey key) {
  if (page() != MfdPage::NavigationMap) return false;

  // The RANGE joystick drives panning (Pilot's Guide: push the Joystick to
  // bring up the Map Pointer, move it to pan). The FMS knob is not involved.
  if (key == BezelKey::PanPush) {
    if (!mapPointerActive_ && mapData_ != nullptr && mapData_->positionValid) {
      mapPointerLat_ = mapData_->ownshipLat;
      mapPointerLon_ = mapData_->ownshipLon;
      mapPanViewCenterLat_ = mapData_->ownshipLat;
      mapPanViewCenterLon_ = mapData_->ownshipLon;
    }
    mapPointerActive_ = !mapPointerActive_;
    if (!mapPointerActive_) {
      mapViewportValid_ = false;
    }
    return true;
  }

  if (!mapPointerActive_) return false;

  // ENT on a highlighted waypoint opens its Waypoint Information page (Pilot's
  // Guide, Map Panning).
  if (key == BezelKey::Ent) {
    const MapFeature* sel = mapPointerFeature();
    if (sel != nullptr) {
      wptEntry_.reset();
      wptFeature_ = *sel;
      wptHasSelection_ = true;
      pageIndex_[static_cast<int>(MfdPageGroup::Waypoint)] =
          wptPageIndexFor(sel->type);
      mapResetPointer();
      pageGroup_ = MfdPageGroup::Waypoint;
      pageSelectSec_ = kPageSelectSeconds;
    }
    return true;
  }

  const double stepNm = std::max(kPanStepMinNm, rangeNm() * kPanStepFrac);
  const double stepRad = stepNm / map::kNmPerEarthRad;
  double eastRad = 0.0;
  double northRad = 0.0;
  switch (key) {
    case BezelKey::PanRight:
      eastRad = stepRad;
      break;
    case BezelKey::PanLeft:
      eastRad = -stepRad;
      break;
    case BezelKey::PanUp:
      northRad = stepRad;
      break;
    case BezelKey::PanDown:
      northRad = -stepRad;
      break;
  default:
    return false;
  }
  map::offsetMercatorCenter(mapPointerLat_, mapPointerLon_, eastRad, northRad,
                            mapPointerLat_, mapPointerLon_);
  mapPointerScrollTowardPointer();
  return true;
}

void MfdController::applyMapPanToDataSource(DataSource& source,
                                            const FlightData& flight) {
  if (!mapPointerActive_) {
    source.setMapPanCenter(false, 0.0, 0.0);
    return;
  }
  if (mapViewportValid_) {
    mapPointerSyncScroll(flight);
  } else {
    mapPointerScrollTowardPointer();
  }
  source.setMapPanCenter(true, mapPanViewCenterLat_, mapPanViewCenterLon_);
}

}  // namespace avionics
