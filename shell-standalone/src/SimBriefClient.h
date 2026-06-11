#pragma once

#include <string>
#include <vector>

#include "avionics/MapData.h"

namespace avionics {

// Result of one SimBrief OFP fetch. SimBrief geocodes every navlog fix, so the
// legs arrive ready to load as the active flight plan (no airway/procedure
// resolution against the local nav database is needed).
struct SimBriefFetchResult {
  bool ok = false;
  std::string error;  // human-readable summary when !ok

  std::vector<MapLeg> legs;     // origin + navlog fixes + destination
  std::string originIcao;
  std::string destinationIcao;
  std::string route;            // filed route string
  std::string generatedUtc;     // OFP generation time, e.g. "10JUN 19:42Z"
};

// Blocking fetch of the account's latest OFP from SimBrief's public fetcher
// (https://www.simbrief.com/api/xml.fetcher.php, JSON v2 format). No
// authentication is involved: the numeric Pilot ID selects the account. Run
// it on a background thread (see SimBriefStore); a fetch can take seconds.
SimBriefFetchResult FetchSimBriefOfp(const std::string& pilotId);

}  // namespace avionics
