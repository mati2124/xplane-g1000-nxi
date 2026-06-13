#include "avionics/ComDecode.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "avionics/NavMath.h"

namespace avionics {
namespace {

struct ComDecodeCandidate {
  float distanceNm = 0.0f;
  int serviceRank = 99;
  std::string ident;
};

// Lower rank wins when multiple airports publish the same COM frequency.
int comDecodeServiceRank(AirportCommService service) {
  switch (service) {
    case AirportCommService::Tower:
      return 0;
    case AirportCommService::Clearance:
    case AirportCommService::Ground:
    case AirportCommService::Departure:
    case AirportCommService::Approach:
    case AirportCommService::Atis:
      return 1;
    case AirportCommService::Unicom:
      return 2;
    case AirportCommService::Other:
      break;
  }
  return 99;
}

std::string decodeComIdent(float activeMhz, const MapData& map,
                           const NavFeatureSource* navSource) {
  if (activeMhz <= 0.0f || navSource == nullptr || !navSource->ready() ||
      !map.positionValid) {
    return {};
  }

  std::vector<ComDecodeCandidate> matches;
  for (const MapFeature& f : map.features) {
    if (f.type != MapFeatureType::Airport) continue;
    const double distNm =
        navDistanceNm(map.ownshipLat, map.ownshipLon, f.lat, f.lon);
    for (const MapAirportFrequency& freq :
         navSource->airportFrequencies(f.id)) {
      if (freq.mhz <= 0.0f || !comFrequenciesMatch(freq.mhz, activeMhz)) {
        continue;
      }
      const char* service = airportCommServiceLabel(freq.service);
      if (service == nullptr) continue;
      ComDecodeCandidate c;
      c.distanceNm = static_cast<float>(distNm);
      c.serviceRank = comDecodeServiceRank(freq.service);
      c.ident = f.id + " " + service;
      matches.push_back(std::move(c));
    }
  }

  if (matches.empty()) return {};

  const auto best = std::min_element(
      matches.begin(), matches.end(),
      [](const ComDecodeCandidate& a, const ComDecodeCandidate& b) {
        if (a.serviceRank != b.serviceRank) {
          return a.serviceRank < b.serviceRank;
        }
        return a.distanceNm < b.distanceNm;
      });
  return best->ident;
}

}  // namespace

const char* airportCommServiceLabel(AirportCommService service) {
  switch (service) {
    case AirportCommService::Atis:
      return "ATIS";
    case AirportCommService::Unicom:
      return "UNICOM";
    case AirportCommService::Clearance:
      return "CLEARANCE";
    case AirportCommService::Ground:
      return "GROUND";
    case AirportCommService::Tower:
      return "TOWER";
    case AirportCommService::Approach:
      return "APPROACH";
    case AirportCommService::Departure:
      return "DEPARTURE";
    case AirportCommService::Other:
      break;
  }
  return nullptr;
}

bool comFrequenciesMatch(float aMhz, float bMhz) {
  // Half a 25 kHz channel; also tolerates minor float rounding from datarefs.
  return std::fabs(aMhz - bMhz) < 0.0126f;
}

void applyComDecodedIdents(FlightData& data, const MapData& map,
                           const NavFeatureSource* navSource) {
  if (!data.dataLinkValid) {
    data.com1Ident.clear();
    data.com2Ident.clear();
    return;
  }
  data.com1Ident = decodeComIdent(data.com1ActiveMhz, map, navSource);
  data.com2Ident = decodeComIdent(data.com2ActiveMhz, map, navSource);
}

}  // namespace avionics
