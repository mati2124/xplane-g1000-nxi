#pragma once

#include <string>
#include <vector>

#include "avionics/FlightPlanPersistence.h"
#include "avionics/MapData.h"
#include "avionics/SimBriefOfpSupport.h"

namespace avionics {

// Stored flight plans the pilot can preview and activate from the MFD FPL group
// Flight Plan Catalog page (G1000 NXi Pilot's Guide, Section 5 "Flight Plan
// Storage"). Imported SimBrief/Navigraph OFPs land here as new stored plans
// instead of being auto-loaded onto the map; only an explicit Activate copies a
// stored plan into the active flight plan that draws on the map.
//
// Each entry reuses PersistedFlightPlan (the same shape the active plan saves),
// so the catalog serializes with the existing per-leg helpers and a stored plan
// can be loaded straight back into the active route.

// Maximum stored flight plans, matching the real unit's 99 catalog slots.
inline constexpr int kFlightPlanCatalogMaxPlans = 99;

class FlightPlanCatalog {
 public:
  int size() const { return static_cast<int>(plans_.size()); }
  bool empty() const { return plans_.empty(); }
  bool full() const { return size() >= kFlightPlanCatalogMaxPlans; }

  const std::vector<PersistedFlightPlan>& plans() const { return plans_; }
  const PersistedFlightPlan& plan(int index) const { return plans_.at(index); }

  // Replace the whole catalog (used to restore the persisted catalog at start).
  // Drops any entries beyond the slot cap.
  void setPlans(std::vector<PersistedFlightPlan> plans);

  // Append a stored plan in the next free slot. Returns its index, or -1 when
  // the catalog is full.
  int addPlan(const PersistedFlightPlan& entry);

  // Build a stored-plan entry from a set of legs (e.g. a SimBrief OFP). The
  // destination is treated as filled when the route has at least an origin and
  // destination; approach grouping is inferred from procedure-tagged legs.
  static PersistedFlightPlan makeEntryFromLegs(const std::vector<MapLeg>& legs);

  // Build a stored-plan entry from a parsed SimBrief OFP (legs + SID/STAR meta).
  static PersistedFlightPlan makeEntryFromSimBriefImport(
      const SimBriefOfpImport& imp);

  // Store a route from legs in the next free slot (returns the index, or -1).
  int addPlanFromLegs(const std::vector<MapLeg>& legs);
  int addPlanFromSimBriefImport(const SimBriefOfpImport& imp);

  // Remove the entry at index; later entries shift up to fill the slot. Returns
  // false for an out-of-range index.
  bool removePlan(int index);
  void clear() { plans_.clear(); }

  // ---- display helpers (read by the catalog page renderer) ----
  // Origin / destination idents, taken from the first / last leg (dashes shown
  // by the renderer when empty).
  static std::string originIdent(const PersistedFlightPlan& entry);
  static std::string destIdent(const PersistedFlightPlan& entry);
  static int legCount(const PersistedFlightPlan& entry) {
    return static_cast<int>(entry.legs.size());
  }
  // Total great-circle distance over the stored legs, in NM.
  static double totalDistanceNm(const PersistedFlightPlan& entry);

  // A copy of the entry with its legs reversed (Invert): origin and destination
  // swap and the loaded approach is dropped (it no longer applies once the route
  // runs the other way), matching the real unit's Invert behavior.
  static PersistedFlightPlan inverted(const PersistedFlightPlan& entry);

 private:
  std::vector<PersistedFlightPlan> plans_;
};

}  // namespace avionics
