#include "FmsRouteProgrammer.h"

#include "FmsDebugOverlay.h"



#include <cctype>

#include <cmath>

#include <cstring>

#include <string>



#include "XPLMNavigation.h"

#include "avionics/NavMath.h"



namespace avionics {

namespace {



constexpr int kFmsIdBufferSize = 256;

constexpr int kNoAltitudeConstraint = 0;

std::string trimNavIdent(const char* id) {

  if (id == nullptr) return {};

  std::string out(id);

  while (!out.empty() &&

         std::isspace(static_cast<unsigned char>(out.back()))) {

    out.pop_back();

  }

  std::size_t start = 0;

  while (start < out.size() &&

         std::isspace(static_cast<unsigned char>(out[start]))) {

    ++start;

  }

  return out.substr(start);

}



bool navIdentEquals(const char* found, const std::string& expected) {

  const std::string a = trimNavIdent(found);

  if (a.empty() || expected.empty()) return false;

  if (a.size() != expected.size()) return false;

  for (std::size_t i = 0; i < a.size(); ++i) {

    if (std::toupper(static_cast<unsigned char>(a[i])) !=

        std::toupper(static_cast<unsigned char>(expected[i]))) {

      return false;

    }

  }

  return true;

}



bool navPositionNear(const MapLeg& leg, float lat, float lon) {

  if (leg.lat == 0.0 && leg.lon == 0.0) return false;

  return navDistanceNm(leg.lat, leg.lon, static_cast<double>(lat),

                       static_cast<double>(lon)) < 0.75;

}



bool navRefMatchesLeg(XPLMNavRef ref, const MapLeg& leg) {

  if (ref == XPLM_NAV_NOT_FOUND) return false;



  char foundId[kFmsIdBufferSize] = {};

  float outLat = 0.0f;

  float outLon = 0.0f;

  XPLMGetNavAidInfo(ref, nullptr, &outLat, &outLon, nullptr, nullptr, nullptr,

                    foundId, nullptr, nullptr);

  foundId[sizeof(foundId) - 1] = '\0';



  if (navIdentEquals(foundId, leg.id)) return true;

  return navPositionNear(leg, outLat, outLon);

}



XPLMNavRef findNavAidById(const MapLeg& leg, XPLMNavType type) {
  return XPLMFindNavAid(nullptr, leg.id.c_str(), nullptr, nullptr, nullptr, type);
}

XPLMNavRef findNavAidNearest(const MapLeg& leg, XPLMNavType type) {
  float lat = static_cast<float>(leg.lat);
  float lon = static_cast<float>(leg.lon);
  return XPLMFindNavAid(nullptr, nullptr, &lat, &lon, nullptr, type);
}

XPLMNavRef resolveNavRef(const MapLeg& leg) {
  if (leg.id.empty()) return XPLM_NAV_NOT_FOUND;

  const XPLMNavType types[] = {xplm_Nav_Airport, xplm_Nav_VOR, xplm_Nav_NDB,
                               xplm_Nav_Fix, xplm_Nav_DME};
  for (XPLMNavType type : types) {
    XPLMNavRef ref = findNavAidById(leg, type);
    if (navRefMatchesLeg(ref, leg)) return ref;
  }

  if (leg.lat != 0.0 || leg.lon != 0.0) {
    for (XPLMNavType type : types) {
      XPLMNavRef ref = findNavAidNearest(leg, type);
      if (navRefMatchesLeg(ref, leg)) return ref;
    }
  }

  return XPLM_NAV_NOT_FOUND;
}



void writeEntry(int index, const MapLeg& leg) {

  const XPLMNavRef ref = resolveNavRef(leg);

  if (ref != XPLM_NAV_NOT_FOUND) {

    XPLMSetFMSEntryInfo(index, ref, kNoAltitudeConstraint);

  } else {

    XPLMSetFMSEntryLatLon(index, static_cast<float>(leg.lat),

                          static_cast<float>(leg.lon), kNoAltitudeConstraint);

  }

}



}  // namespace



void programFmsRoute(const std::vector<MapLeg>& legs) {

  const int oldCount = XPLMCountFMSEntries();

  const int newCount = static_cast<int>(legs.size());



  for (int i = 0; i < newCount; ++i) {

    writeEntry(i, legs[static_cast<std::size_t>(i)]);

  }

  for (int i = oldCount - 1; i >= newCount; --i) {

    XPLMClearFMSEntry(i);

  }

  if (newCount >= 2) {
    int dest = XPLMGetDestinationFMSEntry();
    if (dest < 1 || dest >= newCount) dest = 1;
    XPLMSetDestinationFMSEntry(dest);
  }

  char detail[96] = {};
  if (newCount == 0) {
    std::snprintf(detail, sizeof(detail), "clear FMS");
  } else if (!legs.empty()) {
    std::snprintf(detail, sizeof(detail), "%d legs (%s ... %s)", newCount,
                  legs.front().id.c_str(), legs.back().id.c_str());
  } else {
    std::snprintf(detail, sizeof(detail), "%d legs", newCount);
  }
  FmsDebugOverlay::recordWrite("Route", detail);
}



int findFmsEntryIndexForLeg(const MapLeg& target) {
  const int count = XPLMCountFMSEntries();
  for (int i = 0; i < count; ++i) {
    XPLMNavType type = xplm_Nav_Unknown;
    char id[kFmsIdBufferSize] = {};
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int altitude = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    XPLMGetFMSEntryInfo(i, &type, id, &ref, &altitude, &lat, &lon);
    id[sizeof(id) - 1] = '\0';
    if (!target.id.empty() && navIdentEquals(id, target.id)) return i;
    if (navPositionNear(target, lat, lon)) return i;
  }
  return -1;
}

void programFmsActiveLeg(int legIndex, const std::vector<MapLeg>& plan) {
  if (legIndex < 0 || plan.empty() ||
      legIndex >= static_cast<int>(plan.size())) {
    return;
  }

  const MapLeg& target = plan[static_cast<std::size_t>(legIndex)];
  int targetIndex = findFmsEntryIndexForLeg(target);
  if (targetIndex < 0 && legIndex < XPLMCountFMSEntries()) {
    targetIndex = legIndex;
  }
  if (targetIndex < 0) return;

  if (XPLMGetDestinationFMSEntry() == targetIndex) return;
  XPLMSetDestinationFMSEntry(targetIndex);

  char detail[96] = {};
  std::snprintf(detail, sizeof(detail), "leg %d -> FMS idx %d (%s)", legIndex,
                targetIndex, target.id.c_str());
  FmsDebugOverlay::recordWrite("ActiveLeg", detail);
}

void programFmsDirectTo(bool active, const MapLeg& target) {

  if (!active) {

    const int count = XPLMCountFMSEntries();

    if (count >= 2) XPLMSetDestinationFMSEntry(1);

    FmsDebugOverlay::recordWrite("DirectTo", "clear");
    return;

  }

  int targetIndex = findFmsEntryIndexForLeg(target);

  if (targetIndex < 0) {
    targetIndex = XPLMCountFMSEntries();
    writeEntry(targetIndex, target);
  }



#if defined(XPLM410)

  XPLMSetDirectToFMSFlightPlanEntry(xplm_Fpl_Pilot_Primary, targetIndex);

#else

  XPLMSetDestinationFMSEntry(targetIndex);

#endif

  char detail[96] = {};
  std::snprintf(detail, sizeof(detail), "FMS idx %d (%s)", targetIndex,
                target.id.c_str());
  FmsDebugOverlay::recordWrite("DirectTo", detail);
}



}  // namespace avionics


