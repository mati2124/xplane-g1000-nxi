#pragma once

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/FmsWaypointEntry.h"
#include "avionics/MapData.h"
#include "avionics/FlightData.h"

namespace avionics {

// Matches the PFD Navigation Status Box: GPS Direct-To with no FROM waypoint.
inline bool navDirectToActive(const FlightData& d) {
  return d.fmaFromWpt.empty() && !d.fmaToWpt.empty();
}

class NavFeatureSource;
// Used only by reference in resolveDirectToTargetLeg below; the full definition
// lives in FmsWaypointEntry.h, which callers include alongside this header.
class FmsWaypointEntry;

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
  // MFD Load Airway display: when groupAirways is set, legs carrying a viaAirway
  // tag are routed through the procedure display rows so the "Airway -" header +
  // collapse/expand grouping applies (the PFD window leaves this off). The MFD
  // sets airwaysCollapsed to its current collapse-toggle state so the cursor
  // math matches the rendered rows.
  bool groupAirways = false;
  bool airwaysCollapsed = false;
  // Optional departure / arrival procedure blocks. When set, cursor-row math
  // uses the same procedure display rows as the PFD/MFD FPL list renderer.
  int* departureLegStart = nullptr;
  int* departureLegCount = nullptr;
  std::string* departureHeaderLabel = nullptr;
  MapProcedure* loadedDeparture = nullptr;
  int* arrivalLegStart = nullptr;
  int* arrivalLegCount = nullptr;
  std::string* arrivalHeaderLabel = nullptr;
  MapProcedure* loadedArrival = nullptr;
};

inline void fplRouteEditWireTerminalProcedures(
    FplRouteEdit& edit, int& departureLegStart, int& departureLegCount,
    std::string& departureHeaderLabel, int& arrivalLegStart,
    int& arrivalLegCount, std::string& arrivalHeaderLabel,
    MapProcedure* loadedDeparture = nullptr,
    MapProcedure* loadedArrival = nullptr) {
  edit.departureLegStart = &departureLegStart;
  edit.departureLegCount = &departureLegCount;
  edit.departureHeaderLabel = &departureHeaderLabel;
  edit.loadedDeparture = loadedDeparture;
  edit.arrivalLegStart = &arrivalLegStart;
  edit.arrivalLegCount = &arrivalLegCount;
  edit.arrivalHeaderLabel = &arrivalHeaderLabel;
  edit.loadedArrival = loadedArrival;
}

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

// Looser airport-code shape than isAirportIdent: a four-character ICAO code
// starting with a region letter (remaining three may be digits: K1H2, KX01), or
// a three-character FAA local identifier that contains a digit (1H2, 06C). Used
// to gate the nav-DB lookup so airports whose idents contain digits -- and small
// US fields that have no ICAO code at all -- are still recognised.
bool isAirportCodeFormat(const std::string& id);

// Airport-format code that contains a digit (K1H2, KX01, 1H2). A DB-free signal
// that the ident is an airport (enroute fixes/navaids never take this shape),
// used so FPL layout still recognises small airports the nav database has not
// loaded.
bool isAirportCodeWithDigit(const std::string& id);

// True when `id` is a 4-character ICAO that resolves to an airport in the nav
// database (not a VOR/fix that happens to use the same shape). When the database
// is not ready yet, accepts any well-formed ICAO ident so charts are not blocked
// on startup.
bool isKnownAirportIdent(const std::string& id, const MapData* map,
                         const NavFeatureSource* navSource);

// True when `id` is a flight-plan airport waypoint: 4-letter ICAO, nav-database
// airport, or a digit-bearing airport code (K1H2 / KX01) that counts as a
// destination even when the nav database has not loaded that field.
inline bool isFlightPlanAirportIdent(const std::string& id, const MapData* map,
                                     const NavFeatureSource* navSource) {
  return isAirportIdent(id) || isKnownAirportIdent(id, map, navSource) ||
         isAirportCodeWithDigit(id);
}

// First/last airport in a plan, using isKnownAirportIdent (not bare format).
std::string firstKnownAirportInPlan(const std::vector<MapLeg>& legs,
                                    const MapData* map,
                                    const NavFeatureSource* navSource);
std::string lastKnownAirportInPlan(const std::vector<MapLeg>& legs,
                                   const MapData* map,
                                   const NavFeatureSource* navSource);

std::string airportIcaoBeforeIndex(const std::vector<MapLeg>& legs, int before);

// Last airport ident among legs[0, before), accepting any nav-database airport
// (so digit-format ICAOs like K1H2 / KX01 are recognized) as well as plain
// 4-letter idents. Limiting the scan to before the approach block keeps RNAV
// runway/approach fixes (RW18, CF36) from being mistaken for the destination.
std::string lastKnownAirportBeforeIndex(const std::vector<MapLeg>& legs,
                                        int before, const MapData* map,
                                        const NavFeatureSource* navSource);

// First / last 4-letter airport ident in the leg list (skips fixes, airways, etc.).
std::string firstAirportInPlan(const std::vector<MapLeg>& legs);
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

// FPL list layout: destination is the airport (plus STAR/approach when loaded).
// The sim-readiness destinationFilled flag may be true for fix-ending routes;
// layout uses this stricter rule instead.
inline bool fplLayoutDestinationFilled(const std::vector<MapLeg>& legs,
                                       bool destinationFilled,
                                       int approachLegCount) {
  if (approachLegCount > 0) return true;
  if (!destinationFilled || legs.empty()) return false;
  return isAirportCodeFormat(legs.back().id);
}

inline bool fplEditLayoutDestinationFilled(const FplRouteEdit& edit) {
  return fplLayoutDestinationFilled(edit.legs, edit.destinationFilled,
                                    edit.approachLegCount);
}

std::string fplApproachAirportIcao(const std::vector<MapLeg>& legs,
                                   int approachStart, const MapData* map,
                                   const std::string& loadedApproachAirportIcao = {});

// Inputs for resolving the flight-plan destination airport (PROC default,
// charts, etc.). Mirrors the FPL header destination ident.
struct FplDestinationAirportQuery {
  const std::vector<MapLeg>& legs;
  bool destinationFilled = false;
  int approachLegStart = 0;
  int approachLegCount = 0;
  int arrivalLegStart = 0;
  int arrivalLegCount = 0;
  const MapData* map = nullptr;
  const NavFeatureSource* nav = nullptr;
  std::string loadedApproachAirportIcao;
  std::string arrivalAirportIcao;
  std::string simbriefDestinationIcao;
};

std::string fplDestinationAirportIcao(const FplDestinationAirportQuery& query);

int fplCursorLegIndex(const FplRouteEdit& edit,
                      const std::string& approachAirport,
                      FplCursorLayout layout);

// True when the FPL list cursor sits on a published "HOLD" display row (not the
// parent fix row above it).
bool fplCursorOnHoldRow(const FplRouteEdit& edit,
                        const std::string& approachAirport,
                        FplCursorLayout layout);

// Which loaded terminal-procedure header (if any) the FPL list cursor is on.
// The procedure header rows are selectable cursor stops; landing on one and
// pressing CLR removes the whole SID/STAR/approach (Pilot's Guide 5.6).
enum class FplCursorProcedureBlock { None, Departure, Arrival, Approach };
FplCursorProcedureBlock fplCursorProcedureHeader(const FplRouteEdit& edit,
                                                 const std::string& approachAirport,
                                                 FplCursorLayout layout);

// Exit-fix leg index of the "Airway - <name>.<exit>" header row under the FPL
// list cursor, or -1 when the cursor is not on an airway header. The airway
// header is a selectable cursor stop; landing on it and pressing CLR removes
// the whole loaded-airway segment (Pilot's Guide, Flight Planning - Load
// Airway).
int fplCursorAirwayHeaderExitLeg(const FplRouteEdit& edit,
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

// Remove a whole loaded terminal procedure (Pilot's Guide 5.6, "Remove
// Departure / Arrival / Approach"): erase the procedure's contiguous leg block,
// clear its grouping (header label + loaded procedure), and slide the remaining
// procedure blocks' start indices to follow the shorter leg list. Returns false
// when no such procedure is loaded (nothing removed).
bool fplRemoveDeparture(FplRouteEdit& edit);
bool fplRemoveArrival(FplRouteEdit& edit);
bool fplRemoveApproach(FplRouteEdit& edit);

// Remove the whole loaded-airway segment that the leg at `anyLegIndex` belongs
// to: the contiguous run of legs sharing its viaAirway tag (the fixes shown
// under one "Airway - <name>.<exit>" header). Slides any following procedure
// blocks to follow the shorter leg list. Returns false when that leg carries no
// airway tag (nothing removed).
bool fplRemoveAirwaySegment(FplRouteEdit& edit, int anyLegIndex);

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

// Leg-type suffix beside the ident on FPL / procedure sequence lists (iaf, faf,
// hold, mahp, ...). HILPT and other published holds use "hold" when the CIFP
// row did not already assign a procedure role.
std::string fplLegDisplayRole(const MapLeg& leg);

// Resolve the MapLeg (including hold metadata) for a Direct-To activation.
MapLeg resolveDirectToTargetLeg(const FmsWaypointEntry& entry, bool preservePlan,
                                  int preserveLegIndex,
                                  const std::vector<MapLeg>& planLegs);

}  // namespace avionics
