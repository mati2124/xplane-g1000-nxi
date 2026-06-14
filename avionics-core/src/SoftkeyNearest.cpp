#include "avionics/SoftkeyController.h"

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include "avionics/NavMath.h"

// Nearest Airports window (Pilot's Guide Fig. 5-28): up to 25 airports within
// 200 NM, sorted by distance, each with its primary contact frequency and best
// available approach type (matching the WT NearestStore priority).
namespace avionics {
namespace {

// Nearest Airports window: list capacity and search radius (Pilot's Guide:
// "a list of up to 25 of the nearest airports", "None Within 200nm").
constexpr int kNearestMaxAirports = 25;
constexpr double kNearestMaxRangeNm = 200.0;

int approachTypeRank(const std::string& type) {
  if (type == "ILS") return 0;
  if (type == "LOC") return 1;
  if (type == "RNA") return 2;
  if (type == "VOR") return 3;
  if (type == "NDB") return 4;
  return 5;  // VFR
}

std::string normalizeApproachPrefix(const std::string& name) {
  if (name.size() < 3) return {};
  std::string prefix = name.substr(0, 3);
  for (char& c : prefix) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  if (prefix == "ILS" || prefix == "LOC" || prefix == "RNA" ||
      prefix == "VOR" || prefix == "NDB") {
    return prefix;
  }
  return {};
}

std::string approachTypeFromKind(const std::string& kind) {
  if (kind == "I") return "ILS";
  if (kind == "L") return "LOC";
  if (kind == "R") return "RNA";
  if (kind == "V") return "VOR";
  if (kind == "N") return "NDB";
  return {};
}

// Best approach on the longest runway, matching WT NearestStore priority.
std::string bestApproachType(const std::vector<MapProcedure>& procedures) {
  std::string best = "VFR";
  for (const MapProcedure& proc : procedures) {
    std::string candidate = normalizeApproachPrefix(proc.name);
    if (candidate.empty()) candidate = approachTypeFromKind(proc.approachKind);
    if (candidate.empty()) continue;
    if (approachTypeRank(candidate) < approachTypeRank(best)) {
      best = std::move(candidate);
    }
  }
  return best;
}

int contactFreqRank(AirportCommService service) {
  switch (service) {
    case AirportCommService::Tower:
      return 0;
    case AirportCommService::Unicom:
      return 1;
    default:
      return 99;
  }
}

const char* contactFreqLabel(AirportCommService service) {
  switch (service) {
    case AirportCommService::Tower:
      return "TOWER";
    case AirportCommService::Unicom:
      return "UNICOM";
    default:
      return "MULTICOM";
  }
}

void pickContactFrequency(const std::vector<MapAirportFrequency>& frequencies,
                          float& mhzOut, std::string& labelOut) {
  mhzOut = 0.0f;
  labelOut.clear();
  const MapAirportFrequency* best = nullptr;
  int bestRank = 99;
  for (const MapAirportFrequency& freq : frequencies) {
    if (freq.mhz <= 0.0f) continue;
    const int rank = contactFreqRank(freq.service);
    if (rank < bestRank) {
      bestRank = rank;
      best = &freq;
    } else if (rank == bestRank && best == nullptr) {
      best = &freq;
    }
  }
  if (best == nullptr) {
    for (const MapAirportFrequency& freq : frequencies) {
      if (freq.mhz > 0.0f) {
        best = &freq;
        break;
      }
    }
  }
  if (best == nullptr) return;
  mhzOut = best->mhz;
  labelOut = contactFreqLabel(best->service);
}

}  // namespace

void SoftkeyController::rebuildNearest(const MapData& map) {
  nearest_.clear();
  if (!map.positionValid) {
    nearestCursor_ = 0;
    return;
  }
  for (const MapFeature& f : map.features) {
    if (f.type != MapFeatureType::Airport) continue;
    const double distNm =
        navDistanceNm(map.ownshipLat, map.ownshipLon, f.lat, f.lon);
    if (distNm > kNearestMaxRangeNm) continue;
    NearestAirport a;
    a.id = f.id;
    a.distanceNm = static_cast<float>(distNm);
    a.bearingDeg = static_cast<float>(
        navBearingDeg(map.ownshipLat, map.ownshipLon, f.lat, f.lon));
    a.longestRunwayFt = f.longestRunwayFt;
    a.airportTowered = f.airportTowered;
    a.airportServiced = f.airportServiced;
    a.airportKind = f.airportKind;
    if (navSource_ != nullptr && navSource_->ready()) {
      pickContactFrequency(navSource_->airportFrequencies(f.id), a.frequencyMhz,
                           a.comLabel);
      a.approachType = bestApproachType(navSource_->proceduresForAirport(
          f.id, ProcedureType::Approach));
      if (a.approachType == "VFR") {
        for (const MapApproach& ap : navSource_->approachesForAirport(f.id)) {
          const std::string ils = ap.hasGlideslope ? "ILS" : "LOC";
          if (approachTypeRank(ils) < approachTypeRank(a.approachType)) {
            a.approachType = ils;
          }
        }
      }
    } else {
      a.frequencyMhz = f.frequency;
      a.approachType = "VFR";
    }
    nearest_.push_back(std::move(a));
  }
  std::sort(nearest_.begin(), nearest_.end(),
            [](const NearestAirport& a, const NearestAirport& b) {
              return a.distanceNm < b.distanceNm;
            });
  if (nearest_.size() > static_cast<size_t>(kNearestMaxAirports)) {
    nearest_.resize(kNearestMaxAirports);
  }
  nearestCursor_ = std::max(
      0, std::min(nearestCursor_, static_cast<int>(nearest_.size()) - 1));
}

}  // namespace avionics
