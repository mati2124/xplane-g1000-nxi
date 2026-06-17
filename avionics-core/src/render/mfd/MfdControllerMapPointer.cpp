#include <algorithm>
#include <cmath>

#include "avionics/MfdController.h"
#include "avionics/NavMath.h"

// Navigation Map pointer / pan (Pilot's Guide, Map Panning): the RANGE joystick
// places a pan cursor and pans the map center; the pointer snaps to nearby
// features, and ENT on one opens its Waypoint Information page.
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
// information box. Keeps the snap radius a few pixels of the crosshair at any
// zoom.
constexpr double kMapPointerSnapFrac = 0.04;
constexpr double kMapPointerSnapMinNm = 0.1;

void offsetNm(double lat, double lon, double bearingDeg, double distNm,
              double& outLat, double& outLon) {
  const double cosLat = std::max(0.05, std::cos(lat * kDegToRad));
  const double brg = bearingDeg * kDegToRad;
  outLat = lat + (distNm * std::cos(brg)) / kNmPerDeg;
  outLon = lon + (distNm * std::sin(brg)) / kNmPerDeg / cosLat;
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

void MfdController::mapResetPointer() { mapPointerActive_ = false; }

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
    }
    mapPointerActive_ = !mapPointerActive_;
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
  double bearing = 0.0;
  switch (key) {
    case BezelKey::PanRight:
      bearing = 90.0;
      break;
    case BezelKey::PanLeft:
      bearing = 270.0;
      break;
    case BezelKey::PanUp:
      bearing = 0.0;
      break;
    case BezelKey::PanDown:
      bearing = 180.0;
      break;
  default:
    return false;
  }
  offsetNm(mapPointerLat_, mapPointerLon_, bearing, stepNm,
           mapPointerLat_, mapPointerLon_);
  return true;
}

}  // namespace avionics
