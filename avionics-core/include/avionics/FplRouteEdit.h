#pragma once

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/FlightData.h"

namespace avionics {

// Matches the PFD Navigation Status Box: GPS Direct-To with no FROM waypoint.
inline bool navDirectToActive(const FlightData& d) {
  return d.fmaFromWpt.empty() && !d.fmaToWpt.empty();
}

class NavFeatureSource;

// Layout geometry shared by PFD and MFD approach flight-plan lists.
constexpr float kFplSectionIdentIndentPx = 10.0f;

// Mutable route-edit state shared between the PFD FPL window and MFD FPL page.
struct FplRouteEdit {
  std::vector<MapLeg>& legs;
  bool& destinationFilled;
  int& approachLegStart;
  int& approachLegCount;
  int& cursorRow;
  MapProcedure* loadedApproach = nullptr;
  std::string* approachHeaderLabel = nullptr;
  bool directToActive = false;
  bool localDraft = false;
};

// Leg count for section-row cursor math. During GPS Direct-To the editor shows
// the blank Origin/Enroute/Destination template even if stale legs remain.
inline int fplEditSectionLegCount(const FplRouteEdit& edit) {
  if (edit.directToActive && !edit.localDraft && edit.approachLegCount <= 0) {
    return 0;
  }
  return static_cast<int>(edit.legs.size());
}

enum class FplCursorLayout {
  SectionRows,  // PFD (and MFD with a loaded approach)
  FlatLegList,  // MFD without approach: leg index + append slot
};

bool flightPlanLegsEqual(const std::vector<MapLeg>& a,
                         const std::vector<MapLeg>& b);

bool isAirportIdent(const std::string& id);

std::string airportIcaoBeforeIndex(const std::vector<MapLeg>& legs, int before);

std::string lastAirportInPlan(const std::vector<MapLeg>& legs);

std::string directToAirportIcao(const MapData* map);

// Nearest airport to ownship from the map feature list (PROC default airport).
std::string nearestAirportIcao(const MapData* map);
std::vector<std::string> nearestAirportIds(const MapData* map, int maxCount = 25);

bool fplDestinationFilledFromLegCount(int legCount);

// Direct-To with a single leg is destination-only (not origin).
bool fplDestinationFilledForDisplay(int legCount, bool directToActive);

inline bool fplApproachLayoutDestFilled(bool destinationFilled, int approachStart) {
  return destinationFilled || approachStart >= 2;
}

std::string fplApproachAirportIcao(const std::vector<MapLeg>& legs,
                                   int approachStart, const MapData* map,
                                   const std::string& loadedApproachAirportIcao = {});

int fplCursorLegIndex(const FplRouteEdit& edit,
                      const std::string& approachAirport,
                      FplCursorLayout layout);

int fplCursorSelectableLast(const FplRouteEdit& edit,
                            const std::string& approachAirport,
                            FplCursorLayout layout);

void fplClampCursorRow(FplRouteEdit& edit, const std::string& approachAirport,
                       FplCursorLayout layout);

int fplActiveSelectableRow(const FplRouteEdit& edit,
                           const std::string& approachAirport,
                           const std::string& activeToIdent,
                           FplCursorLayout layout = FplCursorLayout::SectionRows);

void fplSyncListCursorToActiveLeg(FplRouteEdit& edit,
                                  const std::string& approachAirport,
                                  const std::string& activeToIdent,
                                  bool& listCursorFollowsActive,
                                  FplCursorLayout layout = FplCursorLayout::SectionRows);

void fplRefreshDestinationFilledAfterRemove(FplRouteEdit& edit, int legCountAfter);

void fplAdjustApproachGroupingAfterRemove(FplRouteEdit& edit, int removedLegIndex);

bool fplRemoveLegAtIndex(FplRouteEdit& edit, int legIndex);

void fplClearFlightPlan(FplRouteEdit& edit);

// During GPS Direct-To, mirror the map plan when it still carries a loaded
// procedure so the FPL pages keep showing approach legs (blank enroute template).
bool fplAdoptMapPlanDuringDirectTo(FplRouteEdit& edit,
                                   const std::vector<MapLeg>& mapPlan,
                                   const std::vector<MapLeg>& lastPublished);

// Insert or replace the waypoint at the highlighted row. Returns false when
// the airway expansion failed (caller keeps the entry window open).
bool fplCommitWaypointIdent(FplRouteEdit& edit, const NavFeatureSource* navSource,
                            const MapFeature& match, const std::string& ident,
                            int selectableCursorRow,
                            const std::string& approachAirport,
                            FplCursorLayout layout);

// Initial ident for the FPL waypoint-entry window at the highlighted row
// (blank origin/destination slots return empty).
std::string fplIdentEntrySeedAtCursor(const FplRouteEdit& edit,
                                      const std::string& approachAirport,
                                      FplCursorLayout layout);

}  // namespace avionics
