#pragma once

#include <functional>
#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FplRouteEdit.h"
#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"
#include "avionics/ProcedureMenuTypes.h"
#include "avionics/render/BezelKeys.h"

namespace avionics {

// Host wiring for the shared Procedures window logic (PFD + MFD). Mirrors the
// FplRouteEdit pattern: the state lives in ProcedureMenuState, and each
// controller supplies flight-plan refs plus a few host-specific callbacks.
struct ProcedureMenuHost {
  ProcedureMenuState& state;
  const NavFeatureSource* nav = nullptr;
  const MapData* map = nullptr;
  std::vector<MapLeg>& fplLegs;
  int& approachLegStart;
  int& approachLegCount;
  int& cursorRow;
  MapProcedure& loadedApproach;
  PersistedLoadedApproach& persistedRestore;
  std::string* approachHeaderLabel = nullptr;

  std::function<std::string()> defaultAirportIcao;
  std::function<std::vector<std::string>()> nearestAirportIds;
  std::function<void()> publishFlightPlanEdit;
  std::function<void(int legIndex)> requestActivateLeg;
  std::function<void()> requestActivateMissed;
  std::function<void()> closeProceduresMenu;
  std::function<bool()> minimumsBaroEnabled;
  std::function<void(bool)> setMinimumsBaro;
  std::function<float()> minimumsAltitudeFt;
  std::function<void(float)> setMinimumsAltitudeFt;
};

// Optional sub-list to drop open when entering the Approach Loading page.
// Used by dev screenshot states/tests to capture the open dropdowns.
enum class ProcLoadingList { None, Approach, Transition };

void procedureMenuBuild(ProcedureMenuHost& host);
void procedureMenuOpenApproachSelect(ProcedureMenuHost& host);
void procedureMenuOpenApproachLoading(
    ProcedureMenuHost& host, const std::string& icao,
    const std::string& approachName, const std::string& transition,
    ProcLoadingList openList = ProcLoadingList::None);
const char* procedureMenuWindowTitle(const ProcedureMenuHost& host);
const std::string& procedureMenuItemText(const ProcedureMenuHost& host, int i);
bool procedureMenuItemEnabled(const ProcedureMenuHost& host, int i);
std::string procedureMenuAirportIcao(const ProcedureMenuHost& host);
MapFeature procedureMenuAirportFeature(const ProcedureMenuHost& host);
std::vector<std::string> procedureMenuProcedureNames(const ProcedureMenuHost& host,
                                                       ProcedureType type);
std::vector<std::string> procedureMenuTransitions(const ProcedureMenuHost& host,
                                                    ProcedureType type,
                                                    const std::string& name);
std::vector<std::string> procedureMenuTransitionLabels(
    const ProcedureMenuHost& host, ProcedureType type, const std::string& name);
std::vector<std::string> procedureMenuListItems(const ProcedureMenuHost& host);
std::string procedureMenuApproachDisplayName(const ProcedureMenuHost& host,
                                             int index);
MapProcedure procedureMenuSelectedProcedure(const ProcedureMenuHost& host);
std::string procedureMenuAirportCityLine(const ProcedureMenuHost& host);
std::string procedureMenuAirportNameLine(const ProcedureMenuHost& host);
std::string procedureMenuSelectedApproachDisplay(const ProcedureMenuHost& host);
std::string procedureMenuSelectedTransitionDisplay(const ProcedureMenuHost& host);
float procedureMenuPrimaryFreqMhz(const ProcedureMenuHost& host);
bool procedureMenuPrimaryNavIsNdb(const ProcedureMenuHost& host);
bool procedureMenuShowsPrimaryNavFreq(const ProcedureMenuHost& host);
std::string procedureMenuPrimaryIdent(const ProcedureMenuHost& host);
std::vector<MapLeg> procedureMenuPreviewLegs(const ProcedureMenuHost& host);
bool procedureMenuBezelKey(ProcedureMenuHost& host, BezelKey key);

}  // namespace avionics
