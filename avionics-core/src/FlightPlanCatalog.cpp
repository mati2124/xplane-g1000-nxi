#include "avionics/FlightPlanCatalog.h"

#include <algorithm>

#include "avionics/NavMath.h"

namespace avionics {
namespace {

bool sameRouteLegs(const std::vector<MapLeg>& a, const std::vector<MapLeg>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id || a[i].lat != b[i].lat || a[i].lon != b[i].lon) {
      return false;
    }
  }
  return true;
}

}  // namespace

void FlightPlanCatalog::setPlans(std::vector<PersistedFlightPlan> plans) {
  if (static_cast<int>(plans.size()) > kFlightPlanCatalogMaxPlans) {
    plans.resize(static_cast<std::size_t>(kFlightPlanCatalogMaxPlans));
  }
  plans_ = std::move(plans);
}

int FlightPlanCatalog::addPlan(const PersistedFlightPlan& entry) {
  if (full()) return -1;
  plans_.push_back(entry);
  return size() - 1;
}

PersistedFlightPlan FlightPlanCatalog::makeEntryFromLegs(
    const std::vector<MapLeg>& legs) {
  PersistedFlightPlan out;
  out.active = true;
  out.legs = legs;
  // An OFP runs origin -> ... -> destination, so the destination is filled once
  // the route has both endpoints.
  out.destinationFilled = legs.size() >= 2;
  // Fill in approach grouping from any procedure-tagged legs (a SimBrief OFP
  // usually has none, but a stored plan copied from the active route may).
  enrichPersistedFlightPlanFromLegs(out);
  return out;
}

PersistedFlightPlan FlightPlanCatalog::makeEntryFromSimBriefImport(
    const SimBriefOfpImport& imp) {
  return persistedFlightPlanFromSimBriefImport(imp);
}

int FlightPlanCatalog::addPlanFromLegs(const std::vector<MapLeg>& legs) {
  return addPlan(makeEntryFromLegs(legs));
}

int FlightPlanCatalog::addPlanFromSimBriefImport(const SimBriefOfpImport& imp) {
  const PersistedFlightPlan entry = makeEntryFromSimBriefImport(imp);
  for (int i = 0; i < size(); ++i) {
    if (sameRouteLegs(plans_[static_cast<std::size_t>(i)].legs, entry.legs)) {
      plans_[static_cast<std::size_t>(i)] = entry;
      return i;
    }
  }
  return addPlan(entry);
}

int FlightPlanCatalog::dedupeByRoute() {
  const int before = size();
  std::vector<PersistedFlightPlan> unique;
  unique.reserve(plans_.size());
  for (const PersistedFlightPlan& plan : plans_) {
    bool duplicate = false;
    for (const PersistedFlightPlan& kept : unique) {
      if (sameRouteLegs(kept.legs, plan.legs)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) unique.push_back(plan);
  }
  plans_ = std::move(unique);
  return before - size();
}

bool FlightPlanCatalog::removePlan(int index) {
  if (index < 0 || index >= size()) return false;
  plans_.erase(plans_.begin() + index);
  return true;
}

std::string FlightPlanCatalog::originIdent(const PersistedFlightPlan& entry) {
  if (entry.legs.empty()) return {};
  return entry.legs.front().id;
}

std::string FlightPlanCatalog::destIdent(const PersistedFlightPlan& entry) {
  if (entry.legs.empty()) return {};
  return entry.legs.back().id;
}

double FlightPlanCatalog::totalDistanceNm(const PersistedFlightPlan& entry) {
  double total = 0.0;
  for (std::size_t i = 1; i < entry.legs.size(); ++i) {
    total += navDistanceNm(entry.legs[i - 1].lat, entry.legs[i - 1].lon,
                           entry.legs[i].lat, entry.legs[i].lon);
  }
  return total;
}

PersistedFlightPlan FlightPlanCatalog::inverted(
    const PersistedFlightPlan& entry) {
  PersistedFlightPlan out = entry;
  std::reverse(out.legs.begin(), out.legs.end());
  // Reversing the route invalidates a loaded approach (it belonged to the old
  // destination), so drop the approach grouping/metadata.
  out.approachLegStart = -1;
  out.approachLegCount = 0;
  out.approachAirportIcao.clear();
  out.approachMeta = PersistedLoadedApproach{};
  out.departureLegStart = -1;
  out.departureLegCount = 0;
  out.departureMeta = PersistedLoadedApproach{};
  out.arrivalLegStart = -1;
  out.arrivalLegCount = 0;
  out.arrivalMeta = PersistedLoadedApproach{};
  for (MapLeg& leg : out.legs) {
    leg.procedureRole.clear();
  }
  out.destinationFilled = out.legs.size() >= 2;
  return out;
}

}  // namespace avionics
